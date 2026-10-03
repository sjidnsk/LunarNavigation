"""Contiguous replay traces retain uniform anchors and immutable owned storage."""
from dataclasses import replace
import gc
import pickle
import sys
import weakref

import numpy as np
import pytest

from lunar_drl_exploration.replay import ReplayBuffer, dumps_transport, loads_transport
from test_replay_checkpoint import observation, scene, transition


def step(index, *, episode='episode', key='s', **changes):
    value = transition(observation(index), observation(index + 1), key)
    return replace(value, episode_id=episode, step_index=index,
                   behavior_log_prob=-0.7, **changes)


def buffer_for(values, max_bytes=10_000_000):
    replay = ReplayBuffer(max_bytes)
    truths = {value.privileged.scene_id: scene(value.privileged.scene_id) for value in values}
    for value in values:
        replay.add(value, truths)
    return replay


def sampled_by_step(replay, max_steps=8):
    traces = replay.sample_sequences(256, np.random.default_rng(203), max_steps)
    return {(trace[0].episode_id, trace[0].step_index): trace for trace in traces}


@pytest.mark.parametrize('metadata', [
    {'step_index': 0}, {'behavior_log_prob': -0.1},
    {'step_index': -1, 'behavior_log_prob': -0.1},
    {'step_index': 0.5, 'behavior_log_prob': -0.1},
    {'step_index': True, 'behavior_log_prob': -0.1},
    {'step_index': 0, 'behavior_log_prob': float('nan')},
    {'step_index': 0, 'behavior_log_prob': float('inf')},
    {'step_index': 0, 'behavior_log_prob': float('-inf')},
    {'step_index': 0, 'behavior_log_prob': 0.1},
    {'step_index': 0, 'behavior_log_prob': True},
])
def test_transition_rejects_partial_or_invalid_trace_metadata(metadata):
    with pytest.raises(ValueError):
        replace(transition(), **metadata)


def test_legacy_transport_without_new_fields_restores_as_one_step():
    record = pickle.loads(dumps_transport(transition()))
    for node in record['nodes']:
        if node[0] == 'contract' and node[1] == 'Transition':
            node[2].pop('step_index', None)
            node[2].pop('behavior_log_prob', None)
    restored = loads_transport(pickle.dumps(record, protocol=5))
    assert restored.step_index is None and restored.behavior_log_prob is None
    replay = buffer_for([restored])
    assert all(len(trace) == 1 and trace[0] is restored
               for trace in replay.sample_sequences(64, np.random.default_rng(3), 8))
    saved = replay.snapshot(100_000)
    assert len(saved) <= 100_000
    # An actual old bounded_replay_v1 payload also lacks both contract fields.
    snapshot_record = pickle.loads(saved[16:])
    for node in snapshot_record['nodes']:
        if node[0] == 'contract' and node[1] == 'Transition':
            node[2].pop('step_index', None)
            node[2].pop('behavior_log_prob', None)
    legacy_snapshot = b'LDRLRP2\0' + (1).to_bytes(8, 'little') + pickle.dumps(snapshot_record, protocol=5)
    again = ReplayBuffer.from_snapshot(legacy_snapshot)
    assert len(again.sample_sequences(1, np.random.default_rng(3), 8)[0]) == 1


@pytest.mark.parametrize('max_steps', [1, 3, 8])
def test_uniform_anchor_sampling_and_rng_state_exactly_match_original(max_steps):
    values = [step(i) for i in range(5)] + [transition()]
    replay = buffer_for(values)
    before = np.random.default_rng(41)
    after = np.random.default_rng(41)
    anchors = replay.sample(64, before)
    traces = replay.sample_sequences(64, after, max_steps)
    assert all(trace[0] is anchor for trace, anchor in zip(traces, anchors))
    assert before.bit_generator.state == after.bit_generator.state
    assert all(isinstance(trace, tuple) and 1 <= len(trace) <= max_steps for trace in traces)
    assert all(any(value is original for original in values) for trace in traces for value in trace)


def test_interleaved_episodes_extend_by_step_across_actor_versions_and_equal_content():
    values = [step(0, episode='a', actor_version=10), step(0, episode='b'),
              step(1, episode='a', actor_version=11), step(1, episode='b'),
              step(2, episode='a', actor_version=12)]
    # Real transport materializes equal boundaries separately, not as shared objects.
    values = [loads_transport(dumps_transport(value)) for value in values]
    assert values[0].next_observation is not values[2].observation
    traces = sampled_by_step(buffer_for(values))
    assert [item.step_index for item in traces['a', 0]] == [0, 1, 2]
    assert [item.actor_version for item in traces['a', 0]] == [10, 11, 12]
    assert [item.step_index for item in traces['b', 0]] == [0, 1]
    assert len(traces['a', 2]) == 1


@pytest.mark.parametrize('boundary', ['gap', 'scene', 'visible', 'privileged', 'actions', 'legacy'])
def test_sequences_stop_at_missing_or_inconsistent_boundary(boundary):
    first, following = step(0), step(1)
    if boundary == 'gap':
        following = step(2)
    elif boundary == 'scene':
        following = step(1, key='different')
    elif boundary == 'visible':
        features = following.observation.features.copy()
        features[0, 0] = 1
        following = replace(following, observation=replace(following.observation, features=features))
    elif boundary == 'privileged':
        following = replace(following, privileged=replace(following.privileged, observed=np.array([7])))
    elif boundary == 'actions':
        context = replace(following.privileged.actions, removed_frontiers=np.array([2, 1]))
        following = replace(following, privileged=replace(following.privileged, actions=context))
    else:
        following = replace(following, step_index=None, behavior_log_prob=None)
    traces = sampled_by_step(buffer_for([first, following]))
    assert traces['episode', 0] == (first,)


@pytest.mark.parametrize('ending', ['terminated', 'truncated'])
def test_terminal_and_truncated_records_end_trace_without_changing_flags(ending):
    values = [step(0), step(1, **{ending: True}), step(2)]
    traces = sampled_by_step(buffer_for(values))
    assert [item.step_index for item in traces['episode', 0]] == [0, 1]
    assert len(traces['episode', 1]) == 1
    assert getattr(traces['episode', 0][-1], ending) is True
    assert traces['episode', 0][-1].terminated is (ending == 'terminated')


def test_out_of_order_admission_links_only_actual_successors():
    values = [step(2), step(0), step(1)]
    traces = sampled_by_step(buffer_for(values))
    assert [item.step_index for item in traces['episode', 0]] == [0, 1, 2]


def test_duplicate_step_rejected_before_eviction_or_owned_state_mutation():
    original = step(0)
    replay = buffer_for([original, step(1)])
    before = (len(replay), replay.bytes_used, replay.scene_refcounts)
    with pytest.raises(ValueError, match='duplicate'):
        replay.add(replace(original, reward=9), {'s': scene()})
    assert (len(replay), replay.bytes_used, replay.scene_refcounts) == before
    assert sampled_by_step(replay)['episode', 0][0] is original


def test_eviction_releases_index_references_and_allows_reusing_evicted_key():
    first = step(0)
    watched = weakref.ref(first)
    replay = buffer_for([first, step(1), step(2)])
    del first
    replay.evict_oldest()
    gc.collect()
    assert watched() is None
    assert set(sampled_by_step(replay)) == {('episode', 1), ('episode', 2)}
    replay.add(step(0), {'s': scene()})
    assert [item.step_index for item in sampled_by_step(replay)['episode', 0]] == [0, 1, 2]
    while len(replay):
        replay.evict_oldest()
    assert replay.bytes_used == 0 and not replay.scenes
    replay.add(step(0), {'s': scene()})
    assert len(replay) == 1


def test_snapshot_restores_metadata_gaps_and_uniform_anchor_rng():
    replay = buffer_for([step(0), step(1), step(3), transition()])
    saved = replay.snapshot(1_000_000)
    assert len(saved) <= 1_000_000
    restored = ReplayBuffer.from_snapshot(saved)
    traces = sampled_by_step(restored)
    assert [item.step_index for item in traces['episode', 0]] == [0, 1]
    assert len(traces['episode', 3]) == 1
    assert len(traces['episode', None]) == 1
    assert traces['episode', 0][0].behavior_log_prob == -0.7
    with pytest.raises(ValueError):
        traces['episode', 0][0].observation.features.setflags(write=True)
    rng_a, rng_b = np.random.default_rng(45), np.random.default_rng(45)
    keys_a = [(t.episode_id, t.step_index) for t in replay.sample(64, rng_a)]
    keys_b = [(s[0].episode_id, s[0].step_index) for s in restored.sample_sequences(64, rng_b, 8)]
    assert keys_a == keys_b and rng_a.bit_generator.state == rng_b.bit_generator.state


def test_snapshot_budget_omits_oversized_middle_steps_without_bridging_gap():
    states = [observation(index) for index in range(5)]
    states[2] = replace(states[2], polygon=np.zeros((20_000, 2)))
    values = [replace(step(index), observation=states[index], next_observation=states[index + 1])
              for index in range(4)]
    replay = buffer_for(values)
    assert [item.step_index for item in sampled_by_step(replay)['episode', 0]] == [0, 1, 2, 3]
    # The middle state's 160KB array makes either adjacent transition too large,
    # while both end transitions fit. Thus the snapshot itself creates the gap.
    saved = replay.snapshot(150_000)
    assert len(saved) <= 150_000
    restored = ReplayBuffer.from_snapshot(saved)
    traces = sampled_by_step(restored)
    assert set(traces) == {('episode', 0), ('episode', 3)}
    assert all(len(trace) == 1 for trace in traces.values())


def test_sequence_index_memory_is_charged_and_single_entry_budget_stays_bounded():
    original = step(0)
    truth = scene()
    replay = ReplayBuffer(10_000_000)
    for index in range(300):
        replay.add(replace(original, step_index=index), {'s': truth})
    # Independently walk all owned metadata allocations once, excluding payloads
    # and root membership objects already charged by the existing ledger.
    seen = set(replay._ledger.items)
    for keys in [entry[2] for entry in replay._entries.values()] + list(replay._scene_keys.values()):
        seen.add(id(keys))
        seen.update(id(key) for key in keys)
    minimum = replay._ledger.bytes + replay._ledger.membership_bytes + sys.getsizeof(replay._ledger.items)
    pending = [value for value in replay.__dict__.values() if isinstance(value, dict)]
    while pending:
        value = pending.pop()
        if id(value) in seen:
            continue
        seen.add(id(value))
        minimum += sys.getsizeof(value)
        if isinstance(value, dict):
            for key, item in value.items():
                pending.extend((key, item))
        elif isinstance(value, (tuple, list, set)):
            pending.extend(value)
    assert replay.bytes_used >= minimum
    single = buffer_for([original])
    limited = ReplayBuffer(single.bytes_used + 512)
    for index in range(100):
        limited.add(replace(original, episode_id=str(index)), {'s': truth})
        assert limited.bytes_used <= limited.max_bytes
    assert len(limited) == 1
    assert sampled_by_step(limited)['99', 0][0].episode_id == '99'
    before = (limited.bytes_used, len(limited))
    oversized = replace(original, observation=replace(original.observation,
        polygon=np.zeros((20_000, 2))))
    with pytest.raises(ValueError, match='exceed replay budget'):
        limited.add(oversized, {'s': truth})
    assert (limited.bytes_used, len(limited)) == before


@pytest.mark.parametrize('max_steps', [0, -1, 1.5, True, 9])
def test_invalid_horizon_rejected_without_consuming_rng(max_steps):
    replay = buffer_for([transition()])
    rng = np.random.default_rng(5)
    before = rng.bit_generator.state
    with pytest.raises(ValueError):
        replay.sample_sequences(1, rng, max_steps)
    assert rng.bit_generator.state == before
