from collections import Counter
from dataclasses import replace

import numpy as np
import pytest

from lunar_drl_exploration.metrics import ResourceLimitError
from lunar_drl_exploration.replay import ReplayBuffer, transition_task
from test_replay_checkpoint import scene, transition
from test_replay_sequences import step


def truth(task):
    return replace(scene(task), generator_descriptor={'curriculum_task': task})


def populated(counts):
    replay = ReplayBuffer(10_000_000)
    for task, count in counts.items():
        for index in range(count):
            replay.add(step(index, key=task, episode=task), {task: truth(task)})
    return replay


def counts(replay, values):
    return Counter(transition_task(value, replay.scenes) for value in values)


def test_integer_quota_and_three_equal_old_tasks():
    replay = populated({'moon_inside': 5, 'cave_inside': 5,
                        'moon_outside': 5, 'cave_outside': 5})
    batch = replay.sample_stratified(64, np.random.default_rng(7),
        {'cave_inside': .8, 'moon_inside': .2})
    assert counts(replay, batch) == {'cave_inside': 51, 'moon_inside': 13}
    batch = replay.sample_stratified(64, np.random.default_rng(7),
        {'cave_outside': .8, 'moon_inside': .2/3,
         'cave_inside': .2/3, 'moon_outside': .2/3})
    found = counts(replay, batch)
    assert found['cave_outside'] == 51
    assert sorted(found[task] for task in ('moon_inside', 'cave_inside', 'moon_outside')) == [4, 4, 5]


def test_missing_task_and_invalid_weights_fail_without_rng_consumption():
    replay = populated({'moon_inside': 4})
    rng = np.random.default_rng(2)
    before = rng.bit_generator.state
    for weights in ({'moon_inside': .8, 'cave_inside': .2}, {},
                    {'moon_inside': -1}, {'moon_inside': float('nan')}):
        with pytest.raises(ValueError):
            replay.sample_stratified(64, rng, weights)
    assert rng.bit_generator.state == before


def test_eviction_indexes_and_sequence_anchors():
    replay = populated({'moon_inside': 4, 'cave_inside': 4})
    replay.evict_oldest()
    assert replay.task_counts == {'moon_inside': 3, 'cave_inside': 4}
    traces = replay.sample_sequences(64, np.random.default_rng(4), 8,
                                    weights={'moon_inside': .2, 'cave_inside': .8})
    assert counts(replay, [trace[0] for trace in traces]) == {'moon_inside': 13, 'cave_inside': 51}
    assert all(len({value.episode_id for value in trace}) == 1 for trace in traces)
    assert all(value.step_index >= 1 for trace in traces if trace[0].episode_id == 'moon_inside'
               for value in trace)
    while len(replay):
        replay.evict_oldest()
    assert replay.task_counts == {} and replay.bytes_used == 0


def test_snapshot_preserves_rare_old_task_under_recent_task_pressure():
    replay = populated({'moon_inside': 1, 'cave_inside': 100})
    payload = replay.snapshot(200_000, require_sample=True)
    restored = ReplayBuffer.from_snapshot(payload)
    assert set(restored.task_counts) == {'moon_inside', 'cave_inside'}
    assert len(restored) < len(replay)
    assert len(payload) <= 200_000
    assert counts(restored, restored.sample_stratified(64, np.random.default_rng(9),
        {'cave_inside': .8, 'moon_inside': .2})) == {'cave_inside': 51, 'moon_inside': 13}


def test_snapshot_cannot_silently_drop_task_and_legacy_task_helper():
    replay = populated({'moon_inside': 1, 'cave_inside': 1})
    with pytest.raises(ResourceLimitError, match='task'):
        replay.snapshot(1_000, require_sample=True)
    assert transition_task(transition(), {'s': scene()}) == 'unclassified'
    assert transition_task(transition(), {'s': replace(scene(), generator_descriptor={
        'curriculum_task': None})}) == 'unclassified'


def test_snapshot_falls_back_to_small_old_representative():
    replay = populated({'moon_inside': 1, 'cave_inside': 1})
    huge = replace(step(1, key='moon_inside', episode='moon_inside'),
                   observation=replace(step(1).observation, polygon=np.zeros((20_000, 2))))
    replay.add(huge, {'moon_inside': truth('moon_inside')})
    saved = replay.snapshot(200_000)
    restored = ReplayBuffer.from_snapshot(saved)
    assert set(restored.task_counts) == {'moon_inside', 'cave_inside'}
    assert all(value.observation.polygon.shape[0] < 20_000
               for value in restored.sample_stratified(64, np.random.default_rng(9),
                   {'moon_inside': .5, 'cave_inside': .5}))


def test_restore_small_budget_cannot_silently_drop_task():
    replay = populated({'moon_inside': 1, 'cave_inside': 1})
    payload = replay.snapshot(1_000_000)
    one = populated({'moon_inside': 1})
    with pytest.raises(ResourceLimitError, match='task'):
        ReplayBuffer.from_snapshot(payload, max_bytes=one.bytes_used + 512)


def test_stratified_sampling_restores_rng_exactly():
    replay = populated({'moon_inside': 4, 'cave_inside': 4})
    replay.evict_oldest()  # Swap-delete changes the bucket's chronological order.
    restored = ReplayBuffer.from_snapshot(replay.snapshot(1_000_000))
    a, b = np.random.default_rng(20), np.random.default_rng(20)
    weights = {'moon_inside': .2, 'cave_inside': .8}
    first = replay.sample_stratified(64, a, weights)
    second = restored.sample_stratified(64, b, weights)
    assert [(t.episode_id, t.step_index) for t in first] == [(t.episode_id, t.step_index) for t in second]
    assert a.bit_generator.state == b.bit_generator.state
