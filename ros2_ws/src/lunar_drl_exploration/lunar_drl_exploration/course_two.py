"""Admission quotas and durable statistics for the second, sensor-aware course."""
from collections import Counter
from copy import deepcopy
from .ordered_curriculum import OrderedCourse, TASKS as OLD_TASKS
from .course_two_scene import STAGES

SCHEMA = 'region_sensor_course_two_v1'
TASKS = (*OLD_TASKS, *('c2_'+s for s in STAGES))
COUNTERS = ('transitions', 'samples', 'ended', 'successes', 'episode_steps', 'first_roi',
    'zero_decisions', 'initial_completed')


class CourseTwo(OrderedCourse):
    """Reuse the proven admission/terminal accounting, with separate task IDs."""
    def __init__(self, stage, state=None, success_coverage=.99):
        if stage not in STAGES: raise ValueError('invalid course-two stage')
        self.stage=stage;self.phase=STAGES.index(stage);self.task='c2_'+stage
        self.rehearsal=.2;self.success_coverage=success_coverage
        self.weights={task:.05 for task in OLD_TASKS}
        self.weights[self.task]=.8 if self.phase==0 else .6
        if self.phase:
            self.weights.update({'c2_'+s:.2/self.phase for s in STAGES[:self.phase]})
        self.active={};self.episodes={}
        self.by_task={task:dict.fromkeys(COUNTERS,0) for task in TASKS}
        self.coverage_stats={task:dict(count=0,sum=0.,recent=[]) for task in TASKS}
        self.phase_transitions=dict.fromkeys(self.weights,0)
        self.phase_samples=dict.fromkeys(self.weights,0)
        self.bootstrap_updates=0;self.missing_replay_tasks=[];self.last_sampling_weights={}
        if state is not None:
            if state.get('schema')!=SCHEMA or state.get('stage') not in STAGES:
                raise ValueError('invalid course-two saved state')
            previous=STAGES.index(state['stage'])
            if self.phase not in (previous,previous+1): raise ValueError('course-two resume cannot skip/reverse stages')
            self.by_task=deepcopy(state['by_task']);self.coverage_stats=deepcopy(state['coverage_stats'])
            if set(self.by_task)!=set(TASKS) or set(self.coverage_stats)!=set(TASKS):
                raise ValueError('course-two saved task identity changed')
            for values in self.by_task.values():
                if set(values)!=set(COUNTERS) or any(type(v) is not int or v<0 for v in values.values()):
                    raise ValueError('invalid course-two counters')
            self.bootstrap_updates=state['bootstrap_updates']
            if previous==self.phase:
                if state['weights']!=self.weights: raise ValueError('course-two sampling weights changed')
                self.phase_transitions=deepcopy(state['phase_transitions'])
                self.phase_samples=deepcopy(state['phase_samples'])
                for counts in (self.phase_transitions,self.phase_samples):
                    if set(counts)!=set(self.weights) or any(type(v) is not int or v<0 for v in counts.values()):
                        raise ValueError('invalid course-two phase counters')

    def backfill_coverage(self, replay):
        # This course starts new counters. Course-one totals are retained in a
        # separate history record, never reconstructed from truncated replay.
        return

    def select_pending(self, pending, reservations):
        projected=Counter(self.phase_transitions)
        projected.update(self.active[env] for env in reservations if env in self.active)
        total=sum(projected.values())+1
        active=set(self.active.values())
        if not active: return None
        # A low-resource run can have fewer slots than task classes. Wait for a
        # slow *active* class, but never deadlock waiting for an absent class.
        desired=max(active,key=lambda task:self.weights[task]*total-projected[task])
        candidates=[env for env in pending if self.active.get(env)==desired and env not in reservations]
        return min(candidates) if candidates else None

    def next_task(self):
        # Slots track concurrent episodes, not accumulated transition deficits.
        # Otherwise short new episodes duplicate slow rehearsal workers until
        # only one new-task worker remains. select_pending owns admission quotas.
        return super().next_task()

    def ready_for_learning(self, counts):
        return (self.phase_transitions[self.task]>=64 and
                all(counts.get(task,0)>0 for task in self.weights))

    def record_progress(self, env, progress):
        episode=self.episodes[env]
        if progress.get('first_roi') is not None and not episode.get('roi_seen'):
            episode['roi_seen']=True
            self.by_task[self.active[env]]['first_roi']+=1

    def observe(self, env, *, terminated, truncated, coverage, collision):
        task=self.active[env];episode=self.episodes[env]
        zero=bool((terminated or truncated) and episode['steps']==0)
        initial=bool(zero and terminated and not (collision or episode['collision'])
            and coverage>=self.success_coverage)
        super().observe(env,terminated=terminated,truncated=truncated,coverage=coverage,collision=collision)
        if zero:
            self.by_task[task]['zero_decisions']+=1
            self.by_task[task]['initial_completed']+=int(initial)
            self.by_task[task]['successes']-=int(initial)

    def sampling_weights(self, counts):
        missing=[task for task in self.weights if counts.get(task,0)<=0]
        if missing: raise ValueError('course-two replay lost required tasks: '+', '.join(missing))
        self.missing_replay_tasks=[];self.last_sampling_weights=dict(self.weights)
        return dict(self.weights)

    def summary(self):
        return dict(schema=SCHEMA,stage=self.stage,phase=self.phase,task=self.task,rehearsal=.2,
            weights=dict(self.weights),by_task=deepcopy(self.by_task),coverage_stats=deepcopy(self.coverage_stats),
            phase_transitions=dict(self.phase_transitions),phase_samples=dict(self.phase_samples),
            bootstrap_updates=self.bootstrap_updates,missing_replay_tasks=list(self.missing_replay_tasks),
            sampling_weights=dict(self.last_sampling_weights))


def reconfigure_checkpoint(state, config):
    """Explicit offline slot repair/expansion; caller owns pause, backup and save.

    Ordinary resume remains strict. Keep eligible scene/sensor specifications,
    archive surplus slots, and issue fresh identities for replacements. Learned
    state and admission budgets are unchanged. As on any resume, physical episode
    segments are interrupted, never counted as completed or failed episodes.
    """
    from .checkpoint import TrainingState, _generator
    from .config import config_record, resume_semantics, training_config_from_record
    from .replay import loads_transport, dumps_transport
    from .reward_diagnostics import RewardDiagnostics
    from .training import Curriculum

    original = training_config_from_record(state.record['config'])
    state.validate(original)
    before, after = config_record(original), config_record(config)
    different = [key for key in before if before[key] != after[key] and key != 'environments']
    if (not original.course_two or not config.course_two or different or
            not original.environments <= config.environments <= 233):
        raise ValueError('course-two slot repair allows only an environment increase')
    curriculum = loads_transport(state.record['curriculum'])
    course = CourseTwo(curriculum['course']['stage'], state=curriculum['course'],
        success_coverage=config.success_coverage)
    if config.environments < len(course.weights):
        raise ValueError('slot repair needs one environment per active task')
    for env in range(config.environments): course.activate(env, course.next_task())
    desired = Counter(course.active.values())
    kept, retired, counts = {}, {}, Counter()
    for key, spec in sorted(curriculum.get('active_specs', {}).items(), key=lambda item: int(item[0])):
        task = spec['curriculum_task']
        if (str(int(key)) != key or not 0 <= int(key) < original.environments or
                task not in course.weights):
            raise ValueError('invalid saved environment allocation')
        if counts[task] < desired[task]:
            kept[key] = deepcopy(spec)
            counts[task] += 1
        else:
            retired[key] = deepcopy(spec)

    rng = _generator(loads_transport(state.record['rng']['curriculum']))
    source_curriculum = deepcopy(curriculum)
    curriculum['active_specs'] = kept
    resumed = Curriculum(config, rng, curriculum)
    if (resumed.course.stage != source_curriculum['course']['stage'] or
            resumed.state.get('sensor_curriculum') != source_curriculum.get('sensor_curriculum')):
        raise ValueError('slot repair cannot change curriculum stage or sensor phase')
    # Activate kept scenes first so generation accounts for all existing slots,
    # independent of worker reset arrival order after the eventual real resume.
    for key in kept: resumed.next(int(key), state.record['schedule']['transitions'])
    issued = {}
    for env in range(config.environments):
        if str(env) not in kept:
            issued[str(env)] = resumed.next(env, state.record['schedule']['transitions'])
    curriculum = resumed.snapshot()
    curriculum['course'] = source_curriculum['course']  # Preserve every statistic.
    audit = dict(schema='course_two_slots_v1', stage=course.stage,
        environments_before=original.environments, environments_after=config.environments,
        schedule=deepcopy(state.record['schedule']), retained_slots=list(kept),
        retired_specs=retired, issued_specs=deepcopy(issued), allocation=dict(desired))

    counters = loads_transport(state.record['counters'])
    diagnostics = counters.get('reward_diagnostics')
    if diagnostics is not None:
        RewardDiagnostics(original.environments, original.completion_bonus,
            original.success_coverage, diagnostics, exploration_reward_scale=original.exploration_reward_scale)
        diagnostics['environments'] = config.environments
        diagnostics['active'].extend([None]*(config.environments-original.environments))
        RewardDiagnostics(config.environments, config.completion_bonus, config.success_coverage, diagnostics,
            exploration_reward_scale=config.exploration_reward_scale)
    counters.setdefault('course_two_reconfigurations', []).append(deepcopy(audit))
    record = deepcopy(state.record)
    record.update(config=after, semantics=resume_semantics(config),
        curriculum=dumps_transport(curriculum), counters=dumps_transport(counters))
    record['rng']['curriculum'] = dumps_transport(rng.bit_generator.state)
    result = TrainingState(record)
    result.validate(config)
    return result, audit
