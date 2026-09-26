"""Explicit training forks retain learning state without rewriting source identity."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from lunar_drl_exploration.config import TrainingConfig, ModelConfig


@pytest.fixture
def learned_checkpoint(tmp_path):
    torch = pytest.importorskip('torch')
    from lunar_drl_exploration.checkpoint import CheckpointManager, UpdateBoundary
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.schedule import UpdateSchedule
    from test_replay_checkpoint import capture_state, scene, transition
    torch.set_num_threads(1)
    config = replace(TrainingConfig(), model=ModelConfig(width=16, heads=2, layers=1),
        warmup=0, output_dir=str(tmp_path / 'source'))
    learner = SACLearner(config.model, config.learning)
    replay = ReplayBuffer(1000000)
    replay.add(transition(), {'s': scene()})
    schedule = UpdateSchedule(warmup=0)
    for _ in range(7):
        schedule.collected()
    boundary = UpdateBoundary()
    with boundary.update():
        learner.update(replay.sample(64, np.random.default_rng(2)), replay.scenes)
        schedule.updated()
    state = capture_state(learner, config, replay, schedule, boundary)
    path = CheckpointManager(config.output_dir).save(state)
    return config, path, state


def assert_state_equal(left, right):
    import torch
    if isinstance(left, torch.Tensor):
        torch.testing.assert_close(left, right, rtol=0, atol=0)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            assert_state_equal(left[key], right[key])
    elif isinstance(left, (list, tuple)):
        assert len(left) == len(right)
        for old, new in zip(left, right):
            assert_state_equal(old, new)
    else:
        assert left == right


def test_fork_preserves_learning_replay_counters_and_rng_with_explicit_new_identity(learned_checkpoint, tmp_path):
    from lunar_drl_exploration.checkpoint import CheckpointManager, TrainingState
    from lunar_drl_exploration.sac import SACLearner
    config, source, original = learned_checkpoint
    digest = hashlib.sha256(source.read_bytes()).digest()
    changed = replace(config, output_dir=str(tmp_path / 'fork'), stagnation_window_steps=32)
    with pytest.raises(ValueError, match='training_stagnation'):
        original.validate(changed)
    fork = TrainingState.fork_from(source, changed)
    fork.validate(changed)
    for key in ('learner', 'replay', 'schedule', 'rng', 'curriculum', 'collector_state', 'counters'):
        assert_state_equal(original.record[key], fork.record[key])
    restored = SACLearner(changed.model, changed.learning)
    progress = fork.restore(restored, changed)
    assert (progress.schedule.transitions, progress.schedule.updates, len(progress.replay)) == (7, 1, 1)
    assert_state_equal(original.record['learner'], restored.state_dict())
    provenance = fork.record['fork']
    assert provenance['source_checkpoint'] == str(source.resolve())
    assert (provenance['source_transitions'], provenance['source_updates']) == (7, 1)
    assert provenance['changes'] == {'stagnation_window_steps': {'before': 0, 'after': 32}}
    CheckpointManager(changed.output_dir).save(fork)
    assert hashlib.sha256(source.read_bytes()).digest() == digest
    CheckpointManager(config.output_dir).load(config)


@pytest.mark.parametrize('changes', [
    {'completion_bonus': 3.}, {'success_coverage': .95},
    {'curriculum_transition_boundaries': (10000, 60000)},
    {'curriculum_mixtures': ((1., 0., 0.), (.5, .5, 0.), (.15, .25, .6))},
    {'seed': 11}, {'target_rtf': 5.},
])
def test_fork_rejects_unrelated_config_changes(learned_checkpoint, tmp_path, changes):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    changed = replace(config, output_dir=str(tmp_path / 'fork'), **changes)
    with pytest.raises(ValueError, match='fork.*' + next(iter(changes))):
        TrainingState.fork_from(source, changed)
    assert not Path(changed.output_dir).exists()


def test_fork_records_resource_changes_without_changing_learning_identity(learned_checkpoint, tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.config import resume_semantics
    config, source, _ = learned_checkpoint
    changed = replace(config, output_dir=str(tmp_path / 'stage' / 'run'),
        artifact_root=str(tmp_path / 'stage'), total_output_max_bytes=8 * 1024**3,
        metrics_max_bytes=1024**2, reward_diagnostics=True)
    fork = TrainingState.fork_from(source, changed)
    assert resume_semantics(config) == resume_semantics(changed)
    assert set(fork.record['fork']['changes']) == {
        'artifact_root', 'total_output_max_bytes', 'metrics_max_bytes', 'reward_diagnostics'}


@pytest.mark.parametrize('key', ['replay_max_bytes', 'snapshot_max_bytes'])
def test_fork_resource_allowlist_does_not_allow_replay_changes(learned_checkpoint, tmp_path, key):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    changed = replace(config, output_dir=str(tmp_path / 'fork'), **{key: 1024**2})
    with pytest.raises(ValueError, match='fork.*' + key):
        TrainingState.fork_from(source, changed)


@pytest.mark.parametrize('location', ['config', 'collector', 'missing_collector'])
def test_checkpoint_validates_every_recorded_model_identity(learned_checkpoint, location):
    _, _, state = learned_checkpoint
    if location == 'config':
        state.record['config']['model']['actor_score_bound'] = 10.
    elif location == 'collector':
        state.record['collector_state']['actor_record'] = {
            'model_config': dict(state.record['learner']['model_config'], actor_score_bound=10.)}
    else:
        state.record['collector_state'].pop('actor_record', None)
    with pytest.raises(ValueError, match='model config'):
        state.validate()


def test_fork_checks_original_semantics_before_rewriting_them(learned_checkpoint, tmp_path):
    import torch
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, original = learned_checkpoint
    corrupt = tmp_path / 'corrupt.pt'
    original.record['semantics']['reward'] = 'unrecognized reward'
    torch.save(original.record, corrupt)
    changed = replace(config, output_dir=str(tmp_path / 'fork'), stagnation_window_steps=32)
    with pytest.raises(ValueError, match='reward'):
        TrainingState.fork_from(corrupt, changed)


def test_legacy_fork_accepts_relocated_capability_file_with_identical_values(learned_checkpoint, tmp_path):
    import torch
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, original = learned_checkpoint
    for key in ('stagnation_window_steps', 'stagnation_max_positions',
                'stagnation_position_tolerance_m'):
        original.record['config'].pop(key)
    torch.save(original.record, source)
    changed = replace(config, output_dir=str(tmp_path / 'fork'), stagnation_window_steps=32,
        platform=replace(config.platform, capability_path='/new-install/config/wheel.yaml'))
    TrainingState.fork_from(source, changed).validate(changed)
    unsafe = replace(changed, platform=replace(changed.platform, maximum_forward_speed_mps=.3))
    with pytest.raises(ValueError, match='fork.*platform'):
        TrainingState.fork_from(source, unsafe)


def test_fork_refuses_source_directory_even_under_a_symlink(learned_checkpoint, tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    alias = tmp_path / 'alias'
    alias.symlink_to(source.parent, target_is_directory=True)
    with pytest.raises(ValueError, match='different runs'):
        TrainingState.fork_from(source, replace(config, output_dir=str(alias)))


def test_cli_fork_and_resume_are_mutually_exclusive():
    from lunar_drl_exploration.cli import parser
    args = parser().parse_args(['train', '--fork-from', '/saved/resume.pt'])
    assert args.fork_from == Path('/saved/resume.pt') and not args.resume
    with pytest.raises(SystemExit):
        parser().parse_args(['train', '--resume', '--fork-from', '/saved/resume.pt'])


@pytest.mark.parametrize('artifact', ['resume.pt', 'run.json', 'metrics.jsonl', 'other.log'])
def test_fork_refuses_nonempty_destination_before_writing(tmp_path, artifact):
    from lunar_drl_exploration.training import run_training
    destination = tmp_path / 'fork'
    destination.mkdir()
    occupied = destination / artifact
    occupied.write_bytes(b'preserve')
    config = replace(TrainingConfig(), output_dir=str(destination))
    with pytest.raises(ValueError, match='fork.*empty'):
        run_training(config, fork_from=tmp_path / 'source.pt', device='cpu', max_transitions=1)
    assert occupied.read_bytes() == b'preserve'
    assert list(destination.iterdir()) == [occupied]


def test_training_fork_continues_progress_in_new_run_and_can_resume(tmp_path):
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    source_dir, destination = tmp_path / 'source', tmp_path / 'fork'
    config = replace(TrainingConfig(), environments=1, warmup=0,
        output_dir=str(source_dir), system_reserve_bytes=0)
    kwargs = dict(device='cpu', env_factory='worker_fixtures:ControlledEnv',
        probe_extent=40, probe_budget=3)
    initial = run_training(config, max_transitions=8, **kwargs)
    source = source_dir / 'resume.pt'
    digest = hashlib.sha256(source.read_bytes()).digest()
    changed = replace(config, output_dir=str(destination), stagnation_window_steps=32)
    result = run_training(changed, fork_from=source, max_transitions=2, **kwargs)
    assert result['transitions'] >= initial['transitions'] + 2
    assert result['updates'] >= initial['updates'] >= 1
    assert result['episodes_issued'] > initial['episodes_issued']
    assert set(result['episode_ids']).isdisjoint(initial['episode_ids'])
    assert hashlib.sha256(source.read_bytes()).digest() == digest
    run = json.loads((destination / 'run.json').read_text())
    provenance = run['observations']['fork']
    assert run['observations']['resume'] is False
    assert provenance['source_checkpoint'] == str(source.resolve())
    assert provenance['source_transitions'] == initial['transitions']
    assert provenance['source_updates'] == initial['updates']
    saved = CheckpointManager(destination).load(changed)
    assert saved.record['fork'] == provenance
    events = [json.loads(line) for line in (destination / 'metrics.jsonl').read_text().splitlines()]
    assert events[0]['event'] == 'checkpoint'
    assert events[0]['transitions'] == initial['transitions']
    resumed = run_training(changed, resume=True, max_transitions=1, **kwargs)
    assert resumed['transitions'] > result['transitions']
    assert json.loads((destination / 'run.json').read_text())['observations']['fork'] == provenance
