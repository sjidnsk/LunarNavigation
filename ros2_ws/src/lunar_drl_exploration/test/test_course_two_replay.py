"""Learning quotas protect actual phases and diverse completed trajectories."""
from collections import Counter
from dataclasses import replace

import numpy as np
import pytest

from lunar_drl_exploration.replay import ReplayBuffer
from test_replay_checkpoint import scene, transition


OLD = ('moon_inside', 'cave_inside', 'moon_outside', 'cave_outside')


def packed(count):
    bits = np.zeros(100, np.uint8)
    bits[:count] = 1
    return np.packbits(bits, bitorder='little')


def add_episode(replay, task, episode, count=8, covered=50, complete=False):
    truth = replace(scene(episode), reference_shape=(10, 10), packed_reference=packed(100),
        reference_offsets=np.array([0,50,100]), reference_indices=np.arange(100),
        generator_descriptor=dict(curriculum_task=task))
    records = []
    for step in range(count):
        t = transition(key=episode)
        terminal = complete and step == count-1
        t = replace(t, episode_id=episode, step_index=step, behavior_log_prob=-.7,
            privileged=replace(t.privileged, observed=packed(covered)),
            next_privileged=replace(t.next_privileged, observed=packed(100 if terminal else covered)),
            parts=replace(t.parts, new_area_m2=float(100-covered if terminal else 0), removed_frontiers_in=0),
            reward=19.999 if terminal else -.001, terminated=terminal)
        replay.add(t, {episode:truth})
        records.append(t)
    return records


def full_replay():
    replay = ReplayBuffer(128*1024**2)
    for task in OLD:
        for i in range(4):
            add_episode(replay, task, f'{task}-{i}')
    for phase, coverage in [('approach',0), ('cover',50), ('finish',95)]:
        for i in range(6):
            add_episode(replay, 'c2_A1_'+phase, f'{phase}-{i}', covered=coverage,
                complete=phase=='finish' and i<2)
    return replay


def enable(replay):
    assert hasattr(replay, 'enable_course_two_sampling'), 'missing phase-aware replay integration'
    return replay.enable_course_two_sampling(completion_bonus=20.)


def test_phase_is_preaction_state_not_the_scenario_role():
    replay = ReplayBuffer()
    for coverage in (0, 89, 90, 98):
        add_episode(replay, 'c2_A1_approach', str(coverage), count=1, covered=coverage)
    index = enable(replay)
    assert index.status()['pool_counts'] == {'new_approach':1, 'new_cover':1, 'new_finish':2}


def test_batch_has_exact_quotas_global_episode_cap_and_no_duplicate_transition():
    replay = full_replay()
    index = enable(replay)
    assert index.status()['ready']
    rng = np.random.default_rng(15)
    for _ in range(15):
        batch = replay.sample_course_two(rng)
        assert len(batch) == len({(t.episode_id,t.step_index) for t in batch}) == 64
        assert max(Counter(t.episode_id for t in batch).values()) <= 4
        phases = Counter(index.classify(t,replay.scenes) for t in batch)
        assert phases == {**dict.fromkeys(OLD,8),'new_cover':16,'new_approach':8,'new_finish':8}
        assert sum(t.episode_id in ('finish-0','finish-1') for t in batch) >= 4


def test_long_failure_episode_does_not_take_over_the_batch():
    replay = full_replay()
    enable(replay)
    add_episode(replay, 'c2_A1_finish', 'long-failure', count=500, covered=95)
    hits = [sum(t.episode_id=='long-failure' for t in replay.sample_course_two(np.random.default_rng(i)))
        for i in range(50)]
    assert max(hits) <= 4
    assert sum(hits) < 100  # An episode draw, not 500 weighted tickets.


def test_missing_phase_never_silently_fills_a_batch_with_other_tasks():
    replay = ReplayBuffer()
    for task in OLD:
        for i in range(4):add_episode(replay,task,f'{task}-{i}')
    index = enable(replay)
    assert not index.status()['ready']
    assert 'new_approach' in index.status()['missing']
    with pytest.raises(ValueError, match='missing|supply|bucket'):
        replay.sample_course_two(np.random.default_rng(1))


def test_success_pool_requires_real_completion_bonus_and_retained_terminal():
    replay = full_replay()
    enable(replay)
    add_episode(replay,'c2_A1_finish','window',count=30,covered=95,complete=True)
    index = replay.course_two_index
    candidates = index.success_indices
    steps = [replay._entries[k][0].step_index for k in candidates
        if replay._entries[k][0].episode_id=='window']
    assert sorted(steps)==list(range(14,30))
    # Coverage alone cannot certify collision-free rewarded completion.
    false = add_episode(replay,'c2_A1_finish','false',count=1,covered=95)[0]
    false = replace(false, episode_id='false-terminal', terminated=True,
        next_privileged=replace(false.next_privileged,observed=packed(100)))
    replay.add(false, replay.scenes)
    assert all(replay._entries[k][0].episode_id!='false-terminal' for k in index.success_indices)


def test_balanced_snapshot_restores_supply_and_reproducible_rng():
    replay = full_replay()
    index = enable(replay)
    payload = replay.snapshot(32*1024**2, required_indices=index.required_indices())
    restored = ReplayBuffer.from_snapshot(payload, max_bytes=128*1024**2)
    enable(restored)
    assert restored.course_two_index.status()['ready']
    left = replay.sample_course_two(np.random.default_rng(8))
    right = restored.sample_course_two(np.random.default_rng(8))
    assert [(t.episode_id,t.step_index) for t in left]==[(t.episode_id,t.step_index) for t in right]
    with pytest.raises((ValueError,RuntimeError),match='preserve|required|budget'):
        replay.snapshot(4096, required_indices=index.required_indices())


def test_eviction_removes_stale_episode_supply_and_success_proof():
    replay=full_replay()
    index=enable(replay)
    while len(replay):replay.evict_oldest()
    assert index.status()['pool_counts']=={}
    assert not index.status()['ready']
    assert not index.success_indices
