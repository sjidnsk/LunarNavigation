"""A continuation checkpoint must preserve the entire live replay, or fail closed."""
from dataclasses import replace
import numpy as np
import pytest

from lunar_drl_exploration.replay import ReplayBuffer, _same_content, snapshot_sample_count
from lunar_drl_exploration.metrics import ResourceLimitError
from test_replay_checkpoint import observation, scene, transition, capture_state


def populated(count=48):
    replay = ReplayBuffer(32 * 1024**2)
    truths = {task: replace(scene(task), generator_descriptor={
        'curriculum_task': task, 'task_scope': 'whole_cave_v1'})
        for task in ('moon_outside', 'cave_inside')}
    for i in range(count):
        task = tuple(truths)[i % 2]
        replay.add(replace(transition(observation(i), observation(i + 1), task),
            episode_id=f'episode-{i // 4}', step_index=i, behavior_log_prob=-.5), truths)
    # Exercise nonzero FIFO origin and swap-deleted task bucket ordering.
    for _ in range(3):
        replay.evict_oldest()
    return replay


def test_capture_refuses_partial_replay_even_when_some_samples_fit():
    from lunar_drl_exploration.config import TrainingConfig, ModelConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.checkpoint import UpdateBoundary
    replay = populated(256)
    budget = 180000
    old_count = snapshot_sample_count(replay.snapshot(budget))
    assert 0 < old_count < len(replay)  # Reproduce the original silent loss.
    config = replace(TrainingConfig(), model=ModelConfig(width=16, heads=2, layers=1),
        snapshot_max_bytes=budget, replay_max_bytes=replay.max_bytes)
    schedule = UpdateSchedule()
    for _ in range(256):
        schedule.collected()
    learner = SACLearner(config.model, config.learning)
    with pytest.raises(ResourceLimitError, match='complete replay'):
        capture_state(learner, config, replay, schedule, UpdateBoundary())


def test_full_snapshot_preserves_contents_fifo_tasks_aliases_and_sample_sequence():
    replay = populated(128)
    replay.enable_course_two_sampling(task_scope='whole_cave_v1', sampling='scene_episode_v1',
        scene_tasks=('moon_outside', 'cave_inside'))
    payload = replay.snapshot(replay.max_bytes, require_all=True)
    restored = ReplayBuffer.from_snapshot(payload, replay.max_bytes)
    assert len(restored) == len(replay) == snapshot_sample_count(payload)
    assert restored.task_counts == replay.task_counts
    assert restored.scene_refcounts == replay.scene_refcounts
    assert _same_content(dict(replay.scenes), dict(restored.scenes))
    assert all(_same_content(a[0], b[0]) for a, b in
        zip(replay._entries.values(), restored._entries.values()))
    assert restored.course_two_index.status() == replay.course_two_index.status()
    for sampler in ('sample', 'sample_course_two'):
        before_rng, after_rng = np.random.default_rng(91), np.random.default_rng(91)
        for _ in range(4):
            args = (64,) if sampler == 'sample' else ()
            before = getattr(replay, sampler)(*args, before_rng)
            after = getattr(restored, sampler)(*args, after_rng)
            assert [(v.episode_id, v.step_index) for v in before] == [
                (v.episode_id, v.step_index) for v in after]
    a = replay.sample_stratified(64, np.random.default_rng(5), dict.fromkeys(replay.task_counts, .5))
    b = restored.sample_stratified(64, np.random.default_rng(5), dict.fromkeys(replay.task_counts, .5))
    assert [v.step_index for v in a] == [v.step_index for v in b]
    assert snapshot_sample_count(restored.snapshot(restored.max_bytes, require_all=True)) == len(replay)


@pytest.mark.parametrize('validate_only', [False, True])
def test_full_restore_rejects_budget_that_would_evict_samples(validate_only):
    replay = populated()
    payload = replay.snapshot(replay.max_bytes, require_all=True)
    with pytest.raises(ResourceLimitError, match='complete replay'):
        ReplayBuffer.from_snapshot(payload, 180000, validate_only=validate_only)


def test_two_atomic_continuations_preserve_all_samples_and_integrity(tmp_path):
    from lunar_drl_exploration.config import TrainingConfig, ModelConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.checkpoint import CheckpointManager, UpdateBoundary, TrainingState
    config = replace(TrainingConfig(), model=ModelConfig(width=16, heads=2, layers=1),
        snapshot_max_bytes=32*1024**2, replay_max_bytes=32*1024**2)
    replay = populated()
    schedule = UpdateSchedule()
    for _ in range(48):
        schedule.collected()
    learner = SACLearner(config.model, config.learning)
    manager = CheckpointManager(tmp_path, snapshot_max_bytes=config.snapshot_max_bytes)
    for _ in range(2):
        state = capture_state(learner, config, replay, schedule, UpdateBoundary())
        manager.save(state)
        loaded = manager.load(config)
        result = loaded.restore(learner, config)
        assert len(result.replay) == len(replay) == 45
        assert loaded.record['replay_integrity']['count'] == 45
        assert result.schedule.state_dict() == schedule.state_dict()
        replay = result.replay
    bad = dict(loaded.record)
    bad['replay_integrity'] = dict(bad['replay_integrity'], count=44)
    with pytest.raises(ValueError, match='replay.*integrity'):
        TrainingState(bad).restore(learner, config)
