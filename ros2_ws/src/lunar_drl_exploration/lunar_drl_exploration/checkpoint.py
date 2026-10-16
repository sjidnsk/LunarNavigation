"""Complete learner-boundary continuation; one atomically replaced local resume."""
from contextlib import contextmanager
from dataclasses import dataclass
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import random
import time

import numpy as np
import torch
from .config import config_record, resume_semantics
from .metrics import ResourceLimitError, artifact_bytes, live_hardware
from .replay import ReplayBuffer, dumps_transport, loads_transport, snapshot_sample_count
from .schedule import UpdateSchedule
from .feature_schema import LEARNER_SCHEMA, require_frontier_semantics
from .artifacts import (BudgetWriter as _BudgetWriter, atomic_write, initialize_artifact_root,
                        artifact_usage, MANIFEST, _safe_path)

SCHEMA='bounded_training_v1'
REPLAY_BOOTSTRAP_SCHEMA='ordered_a2_replay_bootstrap_v1'
REPLAY_BOOTSTRAP_TRANSITIONS=64


def validate_replay_bootstrap(config, schedule, counters, *, replay=None, count=None, fork=None):
    """A narrow, persisted empty-replay exception for an explicit learned A2 fork.

    An active bootstrap admits exactly 64 new transitions without any optimizer
    update. Its credit must be derivable from the original schedule, and all
    admitted replay survives each checkpoint. Completed records stay as audit
    evidence while normal replay eviction and SAC updates resume.
    """
    from fractions import Fraction
    from .ordered_curriculum import TASKS
    bootstrap = counters.get('replay_bootstrap')
    if bootstrap is None:
        return None
    if (not isinstance(bootstrap, dict) or bootstrap.get('schema') != REPLAY_BOOTSTRAP_SCHEMA
            or config.policy_graph_mode != 'a2' or not config.ordered_curriculum
            or config.remote_frontier_positions != 32):
        raise ValueError('invalid ordered A2 replay bootstrap identity')
    for key in ('source_transitions', 'source_updates', 'required', 'admitted'):
        if type(bootstrap.get(key)) is not int or bootstrap[key] < 0:
            raise ValueError('invalid replay bootstrap counter: ' + key)
    if type(bootstrap.get('active')) is not bool:
        raise ValueError('invalid replay bootstrap active state')
    if (bootstrap['required'] != REPLAY_BOOTSTRAP_TRANSITIONS or
            config.learning.batch_size != REPLAY_BOOTSTRAP_TRANSITIONS or
            not 0 <= bootstrap['admitted'] <= REPLAY_BOOTSTRAP_TRANSITIONS):
        raise ValueError('replay bootstrap requires exactly 64 admissions and batch_size=64')
    sha = bootstrap.get('source_checkpoint_sha256')
    if not isinstance(sha, str) or len(sha) != 64 or any(c not in '0123456789abcdef' for c in sha):
        raise ValueError('invalid replay bootstrap source checkpoint SHA')
    if fork is not None and (fork.get('experiment') != 'ordered-a2' or any(
            fork.get(key) != bootstrap[key] for key in
            ('source_checkpoint_sha256', 'source_transitions', 'source_updates'))):
        raise ValueError('replay bootstrap disagrees with ordered-a2 fork provenance')
    try:
        source_credit = Fraction(bootstrap['source_credit'])
    except (KeyError, TypeError, ValueError, ZeroDivisionError) as error:
        raise ValueError('invalid replay bootstrap source credit') from error
    derived = max(0, bootstrap['source_transitions']-schedule.warmup)*schedule.ratio-bootstrap['source_updates']
    if (source_credit != derived or source_credit < 0 or
            bootstrap['source_transitions'] < schedule.warmup or
            source_credit + REPLAY_BOOTSTRAP_TRANSITIONS*schedule.ratio > schedule.max_credit):
        raise ValueError('replay bootstrap source credit or dispatch capacity is inconsistent')
    tasks = bootstrap.get('required_tasks')
    by_task = bootstrap.get('by_task')
    if (not isinstance(tasks, list) or len(tasks) != len(set(tasks)) or
            not tasks or any(task not in TASKS for task in tasks) or
            not isinstance(by_task, dict) or any(task not in TASKS or type(n) is not int or n <= 0
                for task,n in by_task.items()) or sum(by_task.values()) != bootstrap['admitted']):
        raise ValueError('invalid replay bootstrap task evidence')
    if tasks != list(TASKS[:len(tasks)]):
        raise ValueError('replay bootstrap tasks must follow ordered course phases')
    delta = schedule.transitions-bootstrap['source_transitions']
    if bootstrap['active']:
        course = counters.get('course', {})
        phase = course.get('phase')
        if type(phase) is not int or not 0 <= phase < len(TASKS) or tasks != list(TASKS[:phase+1]):
            raise ValueError('replay bootstrap must include every positive-weight course task')
        if (bootstrap['admitted'] >= REPLAY_BOOTSTRAP_TRANSITIONS or
                bootstrap['admitted'] != delta or schedule.updates != bootstrap['source_updates'] or
                schedule.credit != source_credit+delta*schedule.ratio):
            raise ValueError('active replay bootstrap counters or earned credit disagree')
        if count is not None and count != bootstrap['admitted']:
            raise ValueError('active replay bootstrap must retain every admitted sample')
        if replay is not None and (len(replay) != bootstrap['admitted'] or replay.task_counts != by_task):
            raise ValueError('active replay bootstrap samples disagree with task evidence')
    elif (bootstrap['admitted'] != REPLAY_BOOTSTRAP_TRANSITIONS or delta < REPLAY_BOOTSTRAP_TRANSITIONS
            or schedule.updates < bootstrap['source_updates'] or any(by_task.get(task,0) == 0 for task in tasks)):
        raise ValueError('completed replay bootstrap requires all 64 samples and active course tasks')
    return bootstrap


class UpdateBoundary:
    """Task8 must wrap update AND schedule.updated in this guard.

    A failed partial optimizer step permanently poisons this run's save boundary.
    Preserve the previous resume; reconstruct from it to continue training.
    """
    def __init__(self):
        self.safe=True; self.failed=False

    @contextmanager
    def update(self):
        if not self.safe or self.failed: raise ValueError('invalid optimizer boundary')
        self.safe=False
        try:
            yield
        except BaseException:
            self.failed=True
            raise
        else:
            self.safe=True

    def require_safe(self):
        if not self.safe or self.failed: raise ValueError('checkpoint requires completed optimizer boundary')


@dataclass
class RestoredTraining:
    replay: ReplayBuffer
    schedule: UpdateSchedule
    replay_rng: np.random.Generator
    curriculum_rng: np.random.Generator
    curriculum: dict
    collector_state: dict
    counters: dict


def _generator(state):
    name=state['bit_generator']
    cls=getattr(np.random,name,None)
    if cls is None: raise ValueError(f'unknown numpy RNG: {name}')
    result=np.random.Generator(cls()); result.bit_generator.state=state
    return result


@dataclass
class TrainingState:
    """Owned immutable-time snapshot, captured after collector acknowledgement barrier."""
    record: dict

    @classmethod
    def fork_from(cls, source, config, *, experiment=None):
        """Read a trusted checkpoint into a new collection run, retaining its learner.

        Collection forks allow stagnation limits and output resource settings.
        Explicit paired experiments additionally allow their single learning
        variable, paired scenes and a nonincreasing saved-replay budget. Learning state
        remains intact, including target weights and optimizer history.
        ordered-extent allows only ordered 50-100m to 30-100m collection and
        relocated output/artifact roots, preserving every other parameter.
        ordered-ratio changes only current cave_inside rehearsal .2 to .5,
        rebasing allocation quotas while preserving all learned state and totals.
        ordered-a2 changes only fine to the accepted A2 action/graph contract,
        replacing experience with a bounded 64-transition replay bootstrap.
        This operation neither creates the destination nor writes the source.
        """
        from .config import training_config_from_record
        if experiment == 'course-two-region-query':
            from .region_query_migration import _retired
            _retired()
        if experiment not in (None, 'paired-polyak', 'paired-multistep', 'paired-entropy', 'ordered-extent', 'ordered-ratio', 'ordered-a2', 'course-two', 'course-two-rebalance', 'course-two-potential', 'course-two-region-query'):
            raise ValueError('unknown fork experiment: ' + str(experiment))
        source = Path(source).resolve()
        destination = Path(config.output_dir).resolve()
        if source.parent == destination or source == (destination / 'resume.pt').resolve():
            raise ValueError('fork source and destination must be different runs')
        if destination.exists() and (not destination.is_dir() or any(destination.iterdir())):
            raise ValueError('fork destination must be empty')
        state = cls(torch.load(source, map_location='cpu', weights_only=False))
        state.validate()
        original = training_config_from_record(state.record['config'])
        # Validate its original identity before replacing any metadata. A fork
        # cannot make an incompatible or corrupt old checkpoint look current.
        state.validate(original)
        before, after = config_record(original), config_record(config)
        fields = ('stagnation_window_steps', 'stagnation_max_positions',
                  'stagnation_position_tolerance_m', 'artifact_root',
                  'total_output_max_bytes', 'metrics_max_bytes', 'reward_diagnostics')
        if experiment in ('course-two','course-two-rebalance','course-two-potential'):
            curriculum=loads_transport(state.record['curriculum'])
            if (not original.ordered_curriculum or original.course_two or not config.course_two
                    or config.ordered_curriculum or original.policy_graph_mode!='a2'
                    or config.policy_graph_mode!='a2' or curriculum.get('course',{}).get('phase')!=3):
                raise ValueError('course-two requires completed-phase ordered A2 source and a new course-two run')
            fields=('course_two','ordered_curriculum','artifact_root','total_output_max_bytes',
                    'snapshot_max_bytes','metrics_max_bytes','save_interval_s',
                    'owned_pss_limit_bytes','system_reserve_bytes')
            if experiment in ('course-two-rebalance','course-two-potential'):
                if not config.course_two_variant or not original.environments<=config.environments<=32:
                    raise ValueError('rebalance requires variant and bounded environment increase')
                fields+=('course_two_variant','course_two_catalog','course_two_catalog_sha','course_two_task_scope','course_two_sampling','course_two_scene_profile','environments')
        if experiment=='course-two-potential':
            if original.approach_reward is not None or config.approach_reward is None:
                raise ValueError('potential fork requires unshaped baseline and explicit new shaping')
            fields+=('approach_reward',)
        if experiment == 'ordered-a2':
            if (not original.ordered_curriculum or not config.ordered_curriculum or
                    original.policy_graph_mode != 'fine' or config.policy_graph_mode != 'a2' or
                    config.remote_frontier_positions != 32):
                raise ValueError('ordered-a2 fork requires ordered fine to a2 with remote_frontier_positions=32')
            fields = ('artifact_root', 'policy_graph_mode', 'remote_frontier_positions')
        if experiment == 'ordered-extent':
            # Explicit expansion of the ordered training distribution only. All
            # SAC, rewards, budgets and collection/resource settings stay fixed.
            if original.ordered_curriculum is not True or config.ordered_curriculum is not True:
                raise ValueError('ordered-extent requires ordered_curriculum=true in both runs')
            previous, requested = before['curriculum_extents_m'], after['curriculum_extents_m']
            if (len(previous) != 3 or len(requested) != 3 or any(
                    len(old) != 2 or len(new) != 2 or old[0] != 50. or new[0] != 30. or old[1] != 100. or new[1] != old[1]
                    for old, new in zip(previous, requested))):
                raise ValueError('ordered-extent allows only all training extents 50-100 to 30-100')
            fields = ('artifact_root', 'curriculum_extents_m')
        if experiment == 'ordered-ratio':
            if not original.ordered_curriculum or not config.ordered_curriculum:
                raise ValueError('ordered-ratio requires ordered_curriculum=true')
            curriculum = loads_transport(state.record['curriculum'])
            course = curriculum.get('course', {})
            if course.get('phase') != 1:
                raise ValueError('ordered-ratio requires current cave_inside phase')
            from .ordered_curriculum import OrderedCourse
            OrderedCourse(1, original.ordered_rehearsal_for_phase(1), state=course,
                success_coverage=original.success_coverage)
            if (original.ordered_rehearsal_for_phase(1) != .2 or
                    config.ordered_rehearsal_by_phase is None or
                    config.ordered_rehearsal_for_phase(1) != .5 or any(
                        original.ordered_rehearsal_for_phase(phase) != config.ordered_rehearsal_for_phase(phase)
                        for phase in (0,2,3))):
                raise ValueError('ordered-ratio changes only cave_inside rehearsal .2 to .5')
            fields = ('artifact_root', 'ordered_rehearsal_by_phase')
        if experiment in ('paired-multistep', 'paired-entropy'):
            # This comparison changes only the return horizon. Collection and
            # output caps remain the source contract, unlike a collection fork.
            fields = ()
        # A relocated installation does not alter the platform capability. Its
        # values still compare exactly, matching resume_semantics' path exclusion.
        before_identity, after_identity = [dict(record, platform={key: value
            for key, value in record['platform'].items() if key != 'capability_path'})
            for record in (before, after)]
        learning_changes = {}
        if experiment in ('paired-polyak', 'paired-multistep', 'paired-entropy'):
            learning_fields = {'paired-polyak': ('polyak',),
                'paired-multistep': ('return_steps',),
                'paired-entropy': ('entropy_mode', 'temperature_learning_rate')}[experiment]
            if experiment == 'paired-entropy' and (original.learning.entropy_mode != 'joint'
                    or original.learning.return_steps != 1 or config.learning.return_steps != 1):
                raise ValueError('paired-entropy requires a joint one-step source and learning.return_steps=1')
            if config.paired_scene_sequence is not True:
                raise ValueError(f'{experiment} fork requires paired_scene_sequence=true')
            if config.snapshot_max_bytes > original.snapshot_max_bytes:
                raise ValueError(f'{experiment} fork cannot increase snapshot_max_bytes')
            fields += ('paired_scene_sequence', 'snapshot_max_bytes')
            different_learning = sorted(key for key in before['learning']
                if key not in learning_fields and before['learning'][key] != after['learning'][key])
            if different_learning:
                raise ValueError('incompatible fork configuration: ' + ', '.join(
                    'learning.' + key for key in different_learning))
            for learning_field in learning_fields:
                if before['learning'][learning_field] != after['learning'][learning_field]:
                    learning_changes[learning_field] = dict(before=before['learning'][learning_field],
                        after=after['learning'][learning_field])
            for identity in (before_identity, after_identity):
                identity['learning'] = {key: value for key, value in identity['learning'].items()
                    if key not in learning_fields}
        different = sorted(key for key in before
            if key not in (*fields, 'output_dir') and before_identity[key] != after_identity[key])
        if different:
            raise ValueError('incompatible fork configuration: ' + ', '.join(different))
        state.record['fork'] = dict(source_checkpoint=str(source),
            source_transitions=state.record['schedule']['transitions'],
            source_updates=state.record['schedule']['updates'],
            changes={key: dict(before=before[key], after=after[key])
                for key in fields if before[key] != after[key]})
        if experiment is not None:
            state.record['fork']['experiment'] = experiment
        if experiment in ('ordered-extent', 'ordered-ratio', 'ordered-a2', 'course-two','course-two-rebalance','course-two-potential'):
            import hashlib
            state.record['fork']['source_checkpoint_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
        if experiment in ('course-two','course-two-rebalance','course-two-potential'):
            from .course_two import CourseTwo
            curriculum=loads_transport(state.record['curriculum'])
            curriculum['course_one_history']=copy.deepcopy(curriculum['course'])
            if config.course_two_variant:
                from .course_two_rebalance import RebalancedCourse,course_for
                course=course_for(config)
            else:course=CourseTwo('A1',success_coverage=config.success_coverage)
            curriculum['course']=course.state_dict()
            curriculum['active_specs']={};curriculum['course_two_ordinals']={}
            state.record['curriculum']=dumps_transport(curriculum)
            counters=loads_transport(state.record['counters'])
            counters['course_one_history']=copy.deepcopy(counters.get('course',{}))
            counters['course']=copy.deepcopy(curriculum['course'])
            diagnostics=counters.get('reward_diagnostics')
            if diagnostics is not None and config.environments!=original.environments:
                diagnostics['environments']=config.environments
                diagnostics['active'].extend([None]*(config.environments-original.environments))
            if 'replay_bootstrap' in counters:
                if counters['replay_bootstrap']['active']:
                    raise ValueError('course-two cannot fork an unfinished A2 bootstrap')
                counters['course_one_replay_bootstrap']=counters.pop('replay_bootstrap')
            state.record['counters']=dumps_transport(counters)
        if experiment=='course-two-potential':
            state.record['replay']=ReplayBuffer(config.replay_max_bytes).snapshot(config.snapshot_max_bytes)
            state.record.pop('replay_integrity', None)  # This explicit fork resets experience.
            counters=loads_transport(state.record['counters'])
            counters.pop('reward_diagnostics',None)
            state.record['counters']=dumps_transport(counters)
            state.record['fork']['reward_replay_reset']=True
        if experiment == 'ordered-a2':
            curriculum = loads_transport(state.record['curriculum'])
            course = curriculum.get('course', {})
            from .ordered_curriculum import OrderedCourse
            OrderedCourse(course.get('phase'), original.ordered_rehearsal_for_phase(course.get('phase')),
                state=course, success_coverage=original.success_coverage)
            schedule = UpdateSchedule.from_state_dict(state.record['schedule'])
            counters = loads_transport(state.record['counters'])
            counters['replay_bootstrap'] = dict(schema=REPLAY_BOOTSTRAP_SCHEMA,
                source_checkpoint_sha256=state.record['fork']['source_checkpoint_sha256'],
                source_transitions=schedule.transitions, source_updates=schedule.updates,
                source_credit=str(schedule.credit), required=REPLAY_BOOTSTRAP_TRANSITIONS,
                admitted=0, active=True, required_tasks=list(course['weights']), by_task={})
            state.record['counters'] = dumps_transport(counters)
            state.record['replay'] = ReplayBuffer(config.replay_max_bytes).snapshot(config.snapshot_max_bytes)
            state.record.pop('replay_integrity', None)  # Ordered-A2 explicitly bootstraps new replay.
        if experiment == 'ordered-ratio':
            # Restart quotas at this explicit allocation boundary, retaining all
            # episode/coverage totals and avoiding historical catch-up collection.
            course = copy.deepcopy(course)
            state.record['fork']['allocation_before'] = copy.deepcopy(course)
            course.update(rehearsal=.5, weights={'moon_inside':.5, 'cave_inside':.5},
                phase_transitions={'moon_inside':0, 'cave_inside':0},
                phase_samples={'moon_inside':0, 'cave_inside':0},
                sampling_weights={}, missing_replay_tasks=[])
            curriculum['course'] = course
            state.record['curriculum'] = dumps_transport(curriculum)
            counters = loads_transport(state.record['counters'])
            if 'course' in counters:
                counters['course'] = copy.deepcopy(course)
            state.record['counters'] = dumps_transport(counters)
        if learning_changes:
            state.record['fork']['changes']['learning'] = learning_changes
        if experiment == 'paired-multistep':
            # The source was validated before this explicit semantic migration.
            # Weights, four optimizers, RNG and replay remain byte-for-byte owned
            # source state; ordinary resume/load still compares return_steps.
            state.record['learner']['learning_config'] = dict(
                state.record['learner']['learning_config'], return_steps=config.learning.return_steps)
        if experiment == 'paired-entropy':
            # Fork alone authorizes changing the entropy objective. Both split
            # temperatures inherit the actual source value and Adam history;
            # initial_alpha must never silently reset a trained temperature.
            learner = state.record['learner']
            learner['learning_config'] = dict(after['learning'])
            if config.learning.entropy_mode == 'position_heading':
                learner['log_alpha_heading'] = learner['log_alpha'].clone()
                learner['heading_alpha_optimizer'] = copy.deepcopy(learner['alpha_optimizer'])
        state.record['config'] = after
        state.record['semantics'] = resume_semantics(config)
        state.validate(config)
        return state

    @classmethod
    def capture(cls, *, learner, config, replay, schedule, boundary, replay_rng,
                curriculum_rng, curriculum, collector_state, counters, fork=None,
                arrival_contract_migrations=None):
        boundary.require_safe()
        if len(replay) and replay.observation_schema != config.model.observation_schema:
            raise ValueError('replay observation schema differs from captured model')
        if len(replay) > schedule.transitions:
            raise ValueError('replay admission count exceeds learner-owned counter')
        if (schedule.warmup != config.warmup or
                schedule.ratio != type(schedule.ratio)(str(config.update_ratio))):
            raise ValueError('schedule config differs from captured config')
        if learner.updates != schedule.updates:
            raise ValueError('learner/schedule counters disagree at checkpoint boundary')
        if collector_state['actor_version'] > learner.updates:
            raise ValueError('collector Actor version exceeds completed learner updates')
        # Collector must supply its CPU Actor sampling RNG and seed/curriculum
        # counters. Environment-local unfinished motion is intentionally absent.
        if not isinstance(collector_state.get('policy_rng'),torch.Tensor):
            raise ValueError('collector policy_rng tensor is required')
        bootstrap = validate_replay_bootstrap(config, schedule, counters, replay=replay, fork=fork or {})
        active_bootstrap = bootstrap is not None and bootstrap['active']
        index=replay.course_two_index if config.course_two_variant else None
        required=index.required_indices() if index is not None else ()
        saved_replay=replay.snapshot(config.snapshot_max_bytes,
            require_sample=schedule.can_update and not active_bootstrap,required_indices=required,
            require_all=True)
        if index is not None and index.status()['ready']:
            saved=ReplayBuffer.from_snapshot(saved_replay,config.replay_max_bytes)
            if not saved.enable_course_two_sampling(completion_bonus=config.completion_bonus, task_scope=config.course_two_task_scope, sampling=config.course_two_sampling, scene_profile=config.course_two_scene_profile, exploration_reward_scale=config.exploration_reward_scale, scene_tasks=config.course_two_scene_tasks).status()['ready']:
                raise ValueError('snapshot lost balanced replay readiness')
            del saved
        if active_bootstrap and snapshot_sample_count(saved_replay) != bootstrap['admitted']:
            raise ValueError('active replay bootstrap snapshot must retain every admitted sample')
        rng=dict(python=random.getstate(),numpy=dumps_transport(np.random.get_state()),
            torch_cpu=torch.get_rng_state().clone(),
            torch_cuda=[state.clone() for state in torch.cuda.get_rng_state_all()]
                if torch.cuda.is_initialized() else [],
            replay=dumps_transport(replay_rng.bit_generator.state),
            curriculum=dumps_transport(curriculum_rng.bit_generator.state))
        record = dict(schema=SCHEMA,semantics=resume_semantics(config),config=config_record(config),
            learner=learner.state_dict(),schedule=schedule.state_dict(),rng=rng,
            replay=saved_replay,
            replay_integrity=dict(mode='complete_v1', count=len(replay),
                sha256=hashlib.sha256(saved_replay).hexdigest()),
            curriculum=dumps_transport(curriculum),collector_state=copy.deepcopy(collector_state),
            counters=dumps_transport(counters))
        if fork is not None:
            record['fork'] = copy.deepcopy(fork)
        if arrival_contract_migrations is not None:
            record['arrival_contract_migrations'] = copy.deepcopy(arrival_contract_migrations)
        return cls(record)

    def validate(self, config=None, *, materialize_replay=False):
        from .config import (canonical_model_config, canonical_learning_config,
                             training_config_from_record)
        record=self.record
        if record.get('schema') != SCHEMA: raise ValueError('training checkpoint schema mismatch')
        for key in ('semantics','config','learner','schedule','rng','replay',
                    'curriculum','collector_state','counters'):
            if key not in record: raise ValueError(f'checkpoint missing {key}')
        # Save/export are equally strict even without a caller-supplied config.
        if config is None:
            config = training_config_from_record(record['config'])
        require_frontier_semantics(record['semantics'])
        if record['learner'].get('schema') != LEARNER_SCHEMA:
            raise ValueError('incompatible learner schema')
        require_frontier_semantics(record['learner'])
        semantics = dict(record['semantics'])
        semantics['model'] = canonical_model_config(semantics['model'])
        semantics['learning'] = canonical_learning_config(semantics['learning'])
        # Optimization cadence is not an experience/target semantic, including
        # when canonicalization supplied the absent legacy default.
        semantics['learning'].pop('temperature_learning_rate', None)
        if config is not None:
            expected=resume_semantics(config)
            different=sorted(key for key in set(expected)|set(semantics)
                if expected.get(key)!=semantics.get(key))
            if different: raise ValueError('incompatible full resume semantics: '+', '.join(different))
        schedule=UpdateSchedule.from_state_dict(record['schedule'])
        schedule_config=record['semantics']['schedule']
        if (schedule.warmup != schedule_config['warmup'] or
                schedule.ratio != type(schedule.ratio)(str(schedule_config['ratio']))):
            raise ValueError('checkpoint schedule config disagrees with semantics')
        if (canonical_model_config(record['learner']['model_config']) != semantics['model'] or
                canonical_model_config(record['config']['model']) != semantics['model']):
            raise ValueError('checkpoint model config disagrees with semantics')
        collector_actor = record['collector_state'].get('actor_record')
        if (not isinstance(collector_actor, dict) or
                not isinstance(collector_actor.get('model_config'), dict) or
                canonical_model_config(collector_actor['model_config']) != semantics['model']):
            raise ValueError('checkpoint collector model config missing or disagrees with semantics')
        from .runtime import ActorPolicy
        # Validate collector publication without retaining an extra random Actor.
        with torch.random.fork_rng(devices=[]):
            ActorPolicy.from_record(collector_actor)
        if collector_actor['version'] != record['collector_state']['actor_version']:
            raise ValueError('checkpoint collector weights/publication version disagree')
        learner_learning = canonical_learning_config(record['learner']['learning_config'])
        recorded_learning = canonical_learning_config(record['config']['learning'])
        for key, value in semantics['learning'].items():
            if learner_learning.get(key) != value or recorded_learning.get(key) != value:
                raise ValueError('checkpoint learning config disagrees with semantics')
        split = learner_learning['entropy_mode'] == 'position_heading'
        temperature_keys = ('log_alpha_heading', 'heading_alpha_optimizer')
        if split:
            if any(key not in record['learner'] for key in temperature_keys):
                raise ValueError('checkpoint independent entropy temperature state is missing')
            heading = record['learner']['log_alpha_heading']
            if not isinstance(heading, torch.Tensor) or heading.numel() != 1 or not torch.isfinite(heading).all():
                raise ValueError('checkpoint heading entropy temperature is not finite scalar')
            if not isinstance(record['learner']['heading_alpha_optimizer'], dict):
                raise ValueError('checkpoint heading temperature optimizer is invalid')
        elif any(key in record['learner'] for key in temperature_keys):
            raise ValueError('joint entropy checkpoint cannot contain independent temperature state')
        if record['learner'].get('updates') != schedule.updates:
            raise ValueError('checkpoint learner/schedule counters disagree')
        if record['collector_state']['actor_version']>schedule.updates:
            raise ValueError('checkpoint collector version exceeds learner')
        count=snapshot_sample_count(record['replay'])
        if count>schedule.transitions:
            raise ValueError('checkpoint replay admission count is inconsistent')
        counters = loads_transport(record['counters'])
        if config.course_two:
            if config.environments != record['config']['environments']:
                raise ValueError('incompatible course-two environment count; use explicit slot reconfiguration')
            from .course_two import CourseTwo
            curriculum=loads_transport(record['curriculum'])
            saved=curriculum.get('course',{})
            if config.course_two_variant:
                from .course_two_rebalance import RebalancedCourse,course_for
                course_for(config,state=saved)
            else:CourseTwo(saved.get('stage'),state=saved,success_coverage=config.success_coverage)
            from .course_two_scene import validate_case
            for spec in curriculum.get('active_specs',{}).values():
                case=spec.get('course_two_case')
                if case is not None:
                    if case.get('version') in ('region_sensor_rebalanced_scene_v2','region_sensor_rebalanced_scene_v3'):
                        from .course_two_rebalanced_scene import validate_case as validate_rebalanced
                        validate_rebalanced(case)
                    else:validate_case(case)
                    if spec.get('curriculum_task')!=case['curriculum_task'] or spec.get('seed')!=case['seed']:
                        raise ValueError('saved scenario dispatch identity mismatch')
        bootstrap = validate_replay_bootstrap(config, schedule, counters, count=count,
            fork=record.get('fork', {}))
        active_bootstrap = bootstrap is not None and bootstrap['active']
        if schedule.can_update and count == 0 and not active_bootstrap:
            raise ValueError('checkpoint earned update credit requires a replay sample')
        integrity = record.get('replay_integrity')
        if integrity is not None and (not isinstance(integrity, dict) or
                integrity.get('mode') != 'complete_v1' or type(integrity.get('count')) is not int or
                integrity['count'] != count or integrity.get('sha256') != hashlib.sha256(record['replay']).hexdigest()):
            raise ValueError('checkpoint replay sample count/digest integrity mismatch')
        # A current envelope cannot conceal legacy or semantically mismatched content.
        replay = ReplayBuffer.from_snapshot(record['replay'], config.replay_max_bytes,
            validate_only=not (materialize_replay or active_bootstrap),
            expected_observation_schema=config.model.observation_schema)
        if active_bootstrap:
            validate_replay_bootstrap(config, schedule, counters, replay=replay,
                fork=record.get('fork', {}))
        return replay

    def restore(self, learner, config):
        replay=self.validate(config, materialize_replay=True)
        record=self.record
        # Parse owned state first; no environment Scene/TerrainGrid regeneration.
        schedule=UpdateSchedule.from_state_dict(record['schedule'])
        # A larger operational credit cap is safe; a smaller one must still cover
        # all saved earned credit (no truncation and no silent credit loss).
        if schedule.credit>config.max_update_credit:
            raise ValueError('requested credit cap is below saved pending credit')
        schedule.max_credit=type(schedule.max_credit)(config.max_update_credit)
        rng=record['rng']
        replay_rng=_generator(loads_transport(rng['replay']))
        curriculum_rng=_generator(loads_transport(rng['curriculum']))
        if rng['torch_cuda'] and learner.device.type=='cuda' and len(rng['torch_cuda'])!=torch.cuda.device_count():
            raise ValueError('CUDA RNG device count mismatch')
        curriculum=loads_transport(record['curriculum'])
        counters=loads_transport(record['counters'])
        collector_state=copy.deepcopy(record['collector_state'])
        learner.load_state_dict(record['learner'])
        random.setstate(rng['python']); np.random.set_state(loads_transport(rng['numpy']))
        torch.set_rng_state(rng['torch_cpu'])
        if rng['torch_cuda'] and learner.device.type=='cuda':
            torch.cuda.set_rng_state_all(rng['torch_cuda'])
        return RestoredTraining(replay,schedule,replay_rng,curriculum_rng,
            curriculum,collector_state,counters)


class _CheckpointStream:
    """Stream full replay metadata without bypassing atomic-write budgets.

    Legacy torch serialization writes pickle directly to this stream. Hiding
    fileno keeps tensor storage on BudgetWriter.write as well.
    """
    def __init__(self, stream):
        self.stream = stream

    def fileno(self):
        raise io.UnsupportedOperation('checkpoint storage must use budgeted writes')

    def __getattr__(self, name):
        return getattr(self.stream, name)


class CheckpointManager:
    def __init__(self, output_dir, interval_s=1800, *, total_max_bytes=20*1024**3,
                 snapshot_max_bytes=2*1024**3, clock=time.monotonic, artifact_root=''):
        if interval_s<=0 or total_max_bytes<=0 or snapshot_max_bytes<=0:
            raise ValueError('positive checkpoint budgets required')
        self.output_dir=_safe_path(output_dir)
        if artifact_root:
            root, output = _safe_path(artifact_root), _safe_path(output_dir)
            if not output.is_relative_to(root) or output == root:
                raise ValueError('training output must be inside, not outside or equal to artifact root')
            marker = root / MANIFEST
            if marker.exists() or marker.is_symlink():
                # An existing prepared root owns its immutable diagnostic limit.
                # Keep the requested total exact; initialize rejects any mismatch.
                declared = artifact_usage(root)
                initialize_artifact_root(root, total_max_bytes,
                    diagnostic_max_bytes=declared['diagnostic_max_bytes'])
            else:
                initialize_artifact_root(root, total_max_bytes)
        self.output_dir.mkdir(parents=True,exist_ok=True)
        self.interval_s=interval_s; self.total_max_bytes=total_max_bytes
        self.snapshot_max_bytes=snapshot_max_bytes; self.clock=clock
        self.last_save=clock()
        self.requested_reason=None; self.stop_requested=False

    @property
    def due(self):
        return self.clock()-self.last_save>=self.interval_s

    @property
    def save_requested(self):
        return self.requested_reason is not None or self.due

    def request_save(self, reason, *, stop=False):
        self.requested_reason=str(reason)[:2048]
        self.stop_requested = self.stop_requested or stop

    def request_sigint(self, signum, frame):
        """Install from Task8 main thread; handler sets flags and never saves/raises."""
        self.request_save('SIGINT', stop=True)

    def _atomic(self, name, write):
        if Path(name).name != name:
            raise ValueError('checkpoint artifact name must be a basename')
        atomic_write(self.output_dir/name, write, diagnostic=Path(name).suffix != '.pt',
            local_total_max_bytes=self.total_max_bytes)

    def save(self, state):
        if not isinstance(state.record.get('replay'), bytes):
            raise ValueError('checkpoint replay snapshot bytes are required')
        if len(state.record['replay'])>self.snapshot_max_bytes:
            raise ResourceLimitError('replay snapshot bytes',len(state.record['replay']),self.snapshot_max_bytes)
        state.validate()
        self._atomic('resume.pt', lambda stream: torch.save(state.record,
            _CheckpointStream(stream), pickle_protocol=5, _use_new_zipfile_serialization=False))
        self.last_save=self.clock()
        self.requested_reason=None
        return self.output_dir/'resume.pt'

    def load(self, config=None):
        # Local artifacts are trusted; Torch itself is explicit about pickle load.
        state=TrainingState(torch.load(self.output_dir/'resume.pt',map_location='cpu',weights_only=False))
        state.validate(config)
        return state

    def write_run(self, config, *, owned_pids, code_revision, extra=None):
        record=dict(schema=SCHEMA,code_revision=code_revision,config=config_record(config),semantics=resume_semantics(config),
            hardware=live_hardware(self.output_dir),owned_pids=sorted(set(owned_pids)),
            observations=extra or {})
        payload=(json.dumps(record,ensure_ascii=False,allow_nan=False,indent=2)+'\n').encode('utf-8')
        self._atomic('run.json',lambda stream:stream.write(payload))
        return record
