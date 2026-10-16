"""Paired continuation must identify restored state, not random initialization."""
from dataclasses import replace
import copy
import json

import numpy as np


def test_state_checksum_is_stable_and_detects_target_optimizer_and_rng_changes():
    import torch
    from lunar_drl_exploration.training import state_checksum
    record = {'target': torch.tensor([1., 2.]),
              'optimizer': {'step': 7, 'moment': torch.tensor([.1])},
              'rng': np.random.default_rng(42).bit_generator.state}
    expected = state_checksum(record)
    assert state_checksum(copy.deepcopy(record)) == expected
    assert state_checksum(dict(reversed(list(record.items())))) == expected
    for key in ('target', 'optimizer', 'rng'):
        changed = copy.deepcopy(record)
        if key == 'target':
            changed[key][0] += 1
        elif key == 'optimizer':
            changed[key]['moment'][0] += .01
        else:
            changed[key] = np.random.default_rng(43).bit_generator.state
        assert state_checksum(changed) != expected


def test_initial_identity_excludes_only_polyak_and_does_not_advance_rng():
    import random
    import torch
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.training import initial_training_identity, state_checksum
    learner = SACLearner(ModelConfig(width=16, heads=2, layers=1))
    replay_rng, curriculum_rng = np.random.default_rng(1), np.random.default_rng(2)
    kwargs = dict(replay=ReplayBuffer(1000000), schedule=UpdateSchedule(),
                  replay_rng=replay_rng, curriculum_rng=curriculum_rng,
                  curriculum_state={'episodes_issued': 3}, collector_state={'version': 0},
                  counters={'last_admitted': {0: 7}},
                  source_replay_sha256='source-replay-sha')
    def rng_state():
        return state_checksum((random.getstate(), np.random.get_state(), torch.get_rng_state(),
                               replay_rng.bit_generator.state, curriculum_rng.bit_generator.state))
    rng_before = rng_state()
    before = initial_training_identity(learner, **kwargs)
    learner.learning_config = replace(learner.learning_config, polyak=.02)
    assert initial_training_identity(learner, **kwargs) == before
    assert rng_state() == rng_before
    kwargs['counters']['last_admitted'][0] += 1
    assert initial_training_identity(learner, **kwargs)['counters_sha256'] != before['counters_sha256']
    learner.learning_config = replace(learner.learning_config, gamma=.99)
    assert initial_training_identity(learner, **kwargs)['learner_sha256'] != before['learner_sha256']


def test_resumed_initial_checksum_identifies_saved_weights(tmp_path):
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training, initial_model_checksum
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration.sac import SACLearner
    config = replace(TrainingConfig(), environments=1, warmup=0, paired_scene_sequence=True,
                     output_dir=str(tmp_path), system_reserve_bytes=0)
    kwargs = dict(device='cpu', env_factory='worker_fixtures:ControlledEnv',
                  probe_extent=40, probe_budget=3)
    run_training(config, max_transitions=8, **kwargs)
    first = json.loads((tmp_path/'run.json').read_text())['observations']['initial_model_checksum']
    state = CheckpointManager(tmp_path).load(config)
    learner = SACLearner(config.model, config.learning)
    state.restore(learner, config)
    expected = initial_model_checksum(learner)
    assert expected != first, 'fixture must have performed real learning'
    source_transitions = state.record['schedule']['transitions']
    run_training(config, resume=True, max_transitions=1, **kwargs)
    observed = json.loads((tmp_path/'run.json').read_text())['observations']
    assert observed['initial_model_checksum'] == first, 'legacy multi-stage runner compares initialization lineage'
    assert observed['restored_model_checksum'] == expected
    assert observed['initial_state_identity']['schedule']['transitions'] == source_transitions
