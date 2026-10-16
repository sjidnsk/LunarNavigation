"""Return horizon is explicit continuation semantics, never a hidden reset."""
import copy
from dataclasses import replace
import hashlib
from pathlib import Path

import numpy as np
import pytest

from lunar_drl_exploration.config import LearningConfig, config_record, training_config_from_record
from test_training_fork import learned_checkpoint, assert_state_equal


@pytest.mark.parametrize('value', [0, 9, -1, True, 1.0, None])
def test_return_steps_rejects_non_integer_or_out_of_bounds(value):
    with pytest.raises(ValueError, match='return_steps'):
        LearningConfig(return_steps=value)


def test_legacy_config_defaults_to_one(learned_checkpoint):
    config, _, _ = learned_checkpoint
    record = config_record(config)
    record['learning'].pop('return_steps', None)
    assert training_config_from_record(record).learning.return_steps == 1


@pytest.mark.parametrize('steps', [1, 8])
def test_explicit_multistep_fork_preserves_all_state_except_declared_horizon(
        learned_checkpoint, tmp_path, steps):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.sac import SACLearner
    config, source, original = learned_checkpoint
    digest = hashlib.sha256(source.read_bytes()).digest()
    changed = replace(config, output_dir=str(tmp_path/'fork'), paired_scene_sequence=True,
        snapshot_max_bytes=512*1024**2, learning=replace(config.learning, return_steps=steps))
    fork = TrainingState.fork_from(source, changed, experiment='paired-multistep')
    expected = copy.deepcopy(original.record['learner'])
    expected['learning_config']['return_steps'] = steps
    assert_state_equal(fork.record['learner'], expected)
    for key in ('replay', 'schedule', 'rng', 'curriculum', 'collector_state', 'counters'):
        assert_state_equal(fork.record[key], original.record[key])
    learner = SACLearner(config.model, changed.learning)
    restored = fork.restore(learner, changed)
    assert_state_equal(learner.state_dict(), expected)
    assert len(restored.replay) == 1
    assert restored.schedule.state_dict() == original.record['schedule']
    changes = fork.record['fork']['changes']
    assert changes.get('learning', {}) == ({'return_steps': {'before': 1, 'after': 8}} if steps == 8 else {})
    assert fork.record['fork']['experiment'] == 'paired-multistep'
    assert hashlib.sha256(source.read_bytes()).digest() == digest
    assert not Path(changed.output_dir).exists()


def test_legacy_checkpoint_validates_and_restores_as_one_step(learned_checkpoint):
    from lunar_drl_exploration.sac import SACLearner
    config, _, state = learned_checkpoint
    for record in (state.record['config']['learning'], state.record['semantics']['learning'],
                   state.record['learner']['learning_config']):
        record.pop('return_steps', None)
    state.validate(config)
    learner = SACLearner(config.model, config.learning)
    state.restore(learner, config)
    assert learner.learning_config.return_steps == 1
    assert learner.updates == 1


@pytest.mark.parametrize('location', ['config', 'semantics', 'learner'])
def test_checkpoint_rejects_horizon_identity_mismatch(learned_checkpoint, location):
    _, _, state = learned_checkpoint
    key = 'learning_config' if location == 'learner' else 'learning'
    state.record[location][key]['return_steps'] = 8
    with pytest.raises(ValueError, match='learning'):
        state.validate()


@pytest.mark.parametrize('experiment', [None, 'paired-polyak'])
def test_other_forks_cannot_change_return_steps(learned_checkpoint, tmp_path, experiment):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, state = learned_checkpoint
    changed = replace(config, output_dir=str(tmp_path/'fork'), paired_scene_sequence=True,
        learning=replace(config.learning, return_steps=8))
    with pytest.raises(ValueError, match='learning'):
        state.validate(changed)
    with pytest.raises(ValueError, match='learning'):
        TrainingState.fork_from(source, changed, experiment=experiment)


@pytest.mark.parametrize('field,value', [('polyak', .02), ('gamma', .99), ('learning_rate', 3e-5),
    ('microbatch_size', 8), ('initial_alpha', 4e-5)])
def test_multistep_fork_rejects_unrelated_learning_changes(learned_checkpoint, tmp_path, field, value):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    changed = replace(config, output_dir=str(tmp_path/'fork'), paired_scene_sequence=True,
        learning=replace(config.learning, return_steps=8, **{field: value}))
    with pytest.raises(ValueError, match=r'fork.*learning\.' + field):
        TrainingState.fork_from(source, changed, experiment='paired-multistep')


@pytest.mark.parametrize('field,value', [
    ('stagnation_window_steps', 32), ('stagnation_max_positions', 4),
    ('stagnation_position_tolerance_m', .5), ('artifact_root', '/different-root'),
    ('total_output_max_bytes', 8*1024**3), ('metrics_max_bytes', 1024**2),
    ('reward_diagnostics', True), ('seed', 42), ('environments', 1),
])
def test_multistep_fork_cannot_change_collection_or_output_contract(
        learned_checkpoint, tmp_path, field, value):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    changed = replace(config, output_dir=str(tmp_path/'fork'), paired_scene_sequence=True,
        learning=replace(config.learning, return_steps=8), **{field: value})
    with pytest.raises(ValueError, match='fork.*' + field):
        TrainingState.fork_from(source, changed, experiment='paired-multistep')


def test_multistep_cli_requires_explicit_fork():
    from lunar_drl_exploration.cli import parser, load_config
    args = parser().parse_args(['train', '--fork-experiment', 'paired-multistep'])
    with pytest.raises(ValueError, match='requires --fork-from'):
        load_config(args)
    args = parser().parse_args(['train', '--fork-experiment', 'paired-multistep', '--fork-from', '/source.pt'])
    assert args.fork_experiment == 'paired-multistep'


def test_multistep_initial_identity_excludes_only_return_steps(learned_checkpoint):
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.training import initial_training_identity
    config, _, state = learned_checkpoint
    learner = SACLearner(config.model, config.learning)
    restored = state.restore(learner, config)
    kwargs = dict(replay=restored.replay, schedule=restored.schedule,
        replay_rng=restored.replay_rng, curriculum_rng=restored.curriculum_rng,
        curriculum_state=restored.curriculum, collector_state=restored.collector_state,
        counters=restored.counters, source_replay_sha256='source', excluded_learning_fields=('return_steps',))
    before = initial_training_identity(learner, **kwargs)
    assert before['excluded_learning_fields'] == ['return_steps']
    learner.learning_config = replace(config.learning, return_steps=8)
    assert initial_training_identity(learner, **kwargs) == before
    learner.learning_config = replace(learner.learning_config, polyak=.02)
    assert initial_training_identity(learner, **kwargs)['learner_sha256'] != before['learner_sha256']


def test_training_uses_real_sequences_and_checkpoints_cumulative_trace_metrics(tmp_path):
    import json
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.replay import loads_transport
    from lunar_drl_exploration.training import run_training
    config = replace(TrainingConfig(), environments=1, warmup=0,
        learning=replace(LearningConfig(), return_steps=8),
        output_dir=str(tmp_path/'run'), system_reserve_bytes=0)
    result = run_training(config, device='cpu', max_transitions=24,
        env_factory='worker_fixtures:ControlledEnv', probe_extent=40, probe_budget=20)
    observed = json.loads((tmp_path/'run/run.json').read_text())['observations']['final']['trace_totals']
    assert observed['trace_samples'] == result['updates'] * 64
    assert observed['trace_multi_count'] > 0
    assert observed['trace_length_sum'] > observed['trace_samples']
    assert observed['trace_legacy_count'] == 0
    assert observed['trace_samples'] < observed['trace_coefficient_mass_sum'] <= observed['trace_length_sum']
    state = CheckpointManager(config.output_dir).load(config)
    assert loads_transport(state.record['counters'])['trace_totals'] == observed
