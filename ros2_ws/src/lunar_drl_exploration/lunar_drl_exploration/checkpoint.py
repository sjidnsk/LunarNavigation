"""Complete learner-boundary continuation; one atomically replaced local resume."""
from contextlib import contextmanager
from dataclasses import dataclass
import copy
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
from .artifacts import BudgetWriter as _BudgetWriter, atomic_write, initialize_artifact_root, _safe_path

SCHEMA='bounded_training_v1'


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
    def fork_from(cls, source, config):
        """Read a trusted checkpoint into a new collection run, retaining its learner.

        Only training stagnation limits and output resource settings may change. Existing replay remains
        valid off-policy experience under the unchanged reward and task contract.
        This operation neither creates the destination nor writes the source.
        """
        from .config import training_config_from_record
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
        # A relocated installation does not alter the platform capability. Its
        # values still compare exactly, matching resume_semantics' path exclusion.
        before_identity, after_identity = [dict(record, platform={key: value
            for key, value in record['platform'].items() if key != 'capability_path'})
            for record in (before, after)]
        different = sorted(key for key in before
            if key not in (*fields, 'output_dir') and before_identity[key] != after_identity[key])
        if different:
            raise ValueError('incompatible fork configuration: ' + ', '.join(different))
        state.record['fork'] = dict(source_checkpoint=str(source),
            source_transitions=state.record['schedule']['transitions'],
            source_updates=state.record['schedule']['updates'],
            changes={key: dict(before=before[key], after=after[key])
                for key in fields if before[key] != after[key]})
        state.record['config'] = after
        state.record['semantics'] = resume_semantics(config)
        state.validate(config)
        return state

    @classmethod
    def capture(cls, *, learner, config, replay, schedule, boundary, replay_rng,
                curriculum_rng, curriculum, collector_state, counters):
        boundary.require_safe()
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
        saved_replay=replay.snapshot(config.snapshot_max_bytes, require_sample=schedule.can_update)
        rng=dict(python=random.getstate(),numpy=dumps_transport(np.random.get_state()),
            torch_cpu=torch.get_rng_state().clone(),
            torch_cuda=[state.clone() for state in torch.cuda.get_rng_state_all()]
                if torch.cuda.is_initialized() else [],
            replay=dumps_transport(replay_rng.bit_generator.state),
            curriculum=dumps_transport(curriculum_rng.bit_generator.state))
        return cls(dict(schema=SCHEMA,semantics=resume_semantics(config),config=config_record(config),
            learner=learner.state_dict(),schedule=schedule.state_dict(),rng=rng,
            replay=saved_replay,
            curriculum=dumps_transport(curriculum),collector_state=copy.deepcopy(collector_state),
            counters=dumps_transport(counters)))

    def validate(self, config=None):
        from .config import canonical_model_config
        record=self.record
        if record.get('schema') != SCHEMA: raise ValueError('training checkpoint schema mismatch')
        for key in ('semantics','config','learner','schedule','rng','replay',
                    'curriculum','collector_state','counters'):
            if key not in record: raise ValueError(f'checkpoint missing {key}')
        semantics = dict(record['semantics'])
        semantics['model'] = canonical_model_config(semantics['model'])
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
        for key, value in record['semantics']['learning'].items():
            if record['learner']['learning_config'].get(key) != value:
                raise ValueError('checkpoint learning config disagrees with semantics')
        if record['learner'].get('updates') != schedule.updates:
            raise ValueError('checkpoint learner/schedule counters disagree')
        if record['collector_state']['actor_version']>schedule.updates:
            raise ValueError('checkpoint collector version exceeds learner')
        count=snapshot_sample_count(record['replay'])
        if count>schedule.transitions:
            raise ValueError('checkpoint replay admission count is inconsistent')
        if schedule.can_update and count == 0:
            raise ValueError('checkpoint earned update credit requires a replay sample')

    def restore(self, learner, config):
        self.validate(config)
        record=self.record
        # Parse owned state first; no environment Scene/TerrainGrid regeneration.
        replay=ReplayBuffer.from_snapshot(record['replay'],config.replay_max_bytes)
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
        state.validate()
        if len(state.record['replay'])>self.snapshot_max_bytes:
            raise ResourceLimitError('replay snapshot bytes',len(state.record['replay']),self.snapshot_max_bytes)
        self._atomic('resume.pt',lambda stream:torch.save(state.record,stream))
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
