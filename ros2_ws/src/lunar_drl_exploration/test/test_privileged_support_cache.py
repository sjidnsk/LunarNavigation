"""Static query reuse must never reuse the current frontier/observed state."""
from dataclasses import replace
import numpy as np
import pytest

from lunar_drl_exploration.contracts import Pose, PrivilegedActionContext, PrivilegedState
from lunar_drl_exploration.privileged import PrivilegedBuilder, _RemovalPrediction, build_truth
from lunar_drl_exploration.sensor import SensorModel
from test_privileged_actions import fixture, initial_known, measured_frontiers, observation_at


def count_native_queries(monkeypatch):
    from lunar_drl_exploration.metric_graph import native
    calls = []
    original = native.distances
    def counted(*args):
        calls.append(args[2].copy())
        return original(*args)
    monkeypatch.setattr(native, 'distances', counted)
    return calls


def setup_builder(**kwargs):
    terrain, task, sensor, reference = fixture()
    builder = PrivilegedBuilder(terrain, build_truth(terrain, reference), task, sensor, **kwargs)
    known = initial_known(terrain)
    bits = np.packbits(known.ravel(), bitorder='little')
    fronts = measured_frontiers(terrain, task, sensor, known)
    return builder, task, known, bits, fronts


def uncached_state(builder, observation, observed, fronts):
    """The pre-cache builder algorithm, independently assembling every query."""
    positions, inverse = np.unique(observation.goals[:, :2], axis=0, return_inverse=True)
    offsets, supports, distances = [0], [], []
    terrain = builder.terrain
    origin = np.asarray(terrain.origin[:2])
    for xy in positions:
        cell = np.floor((xy - origin) / terrain.resolution_m).astype(np.int64)
        entry = float(np.linalg.norm((cell + .5) * terrain.resolution_m + origin - xy))
        distance = builder.grid.distances(cell.reshape(1, 2), np.array([entry]),
            builder.config.coverage_radius_m + entry, builder.cells)
        selected = np.flatnonzero(np.isfinite(distance))
        supports.extend(selected.tolist())
        distances.extend(distance[selected].tolist())
        offsets.append(len(supports))
    prediction = _RemovalPrediction(fronts, terrain) if len(observation.goals) else None
    removed = np.asarray([prediction.predict(terrain, Pose(*goal), builder.sensor)
                          for goal in observation.goals], dtype=np.int64)
    actions = PrivilegedActionContext(positions, inverse, observation.goals[:, 2],
        np.asarray(offsets), np.asarray(supports, np.int64), np.asarray(distances, np.float32), removed)
    return PrivilegedState(builder.scene.scene_id, observed, actions)


def assert_same_bytes(actual, expected):
    assert actual.scene_id == expected.scene_id and actual.schema == expected.schema
    assert actual.actions.schema == expected.actions.schema
    for a, b in [(actual.observed, expected.observed)] + [
        (getattr(actual.actions, name), getattr(expected.actions, name)) for name in (
            'positions', 'action_positions', 'action_yaws', 'support_offsets',
            'support_indices', 'support_distances', 'removed_frontiers')]:
        assert a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes()


def test_repeat_and_expanded_prefix_skip_native_static_queries(monkeypatch):
    builder, _, _, bits, fronts = setup_builder()
    calls = count_native_queries(monkeypatch)
    original = observation_at([[5.25, 5.25], [6.25, 5.25]], [0., np.pi])
    first = builder.build(original, bits, frontiers=fronts)
    assert len(calls) == 2
    repeated = builder.build(original, bits, frontiers=fronts)
    assert len(calls) == 2
    assert_same_bytes(repeated, first)
    expanded = observation_at([[5.25, 5.25], [6.25, 5.25], [7.25, 5.25]], [0., np.pi])
    builder.build(expanded, bits, frontiers=fronts)
    assert len(calls) == 3


def test_hits_recompute_frontier_removal_and_own_immutable_outputs(monkeypatch):
    builder, task, known, bits, fronts = setup_builder()
    calls = count_native_queries(monkeypatch)
    obs = observation_at([[5.25, 5.25], [6.25, 5.25]], [0., np.pi])
    before = builder.build(obs, bits, frontiers=fronts)
    measurement = SensorModel.observe(builder.terrain, Pose(*obs.goals[0]), builder.sensor)
    known.ravel()[measurement.indices] = True
    new_bits = np.packbits(known.ravel(), bitorder='little')
    new_fronts = measured_frontiers(builder.terrain, task, builder.sensor, known, revision=2)
    after = builder.build(obs, new_bits, frontiers=new_fronts)
    assert len(calls) == 2
    np.testing.assert_array_equal(before.actions.removed_frontiers, [5, 5, 0, 13])
    np.testing.assert_array_equal(after.actions.removed_frontiers, [0, 8, 0, 8])
    for state in (before, after):
        for array in (state.observed, state.actions.support_indices, state.actions.support_distances):
            with pytest.raises(ValueError):
                array.setflags(write=True)
    assert not np.shares_memory(before.actions.support_distances, after.actions.support_distances)
    bits[:] = 255
    assert before.observed.tobytes() != bits.tobytes()


def test_lru_capacity_keeps_recent_queries_and_evicts_old_ones(monkeypatch):
    builder, _, _, bits, fronts = setup_builder(support_cache_size=2)
    calls = count_native_queries(monkeypatch)
    for x in [5.25, 6.25, 5.25, 7.25, 5.25]:
        builder.build(observation_at([[x, 5.25]], [0.]), bits, frontiers=fronts)
        assert len(builder._support_cache) <= 2
    assert len(calls) == 3
    builder.build(observation_at([[6.25, 5.25]], [0.]), bits, frontiers=fronts)
    assert len(calls) == 4 and len(builder._support_cache) == 2


def test_zero_capacity_disables_reuse(monkeypatch):
    builder, _, _, bits, fronts = setup_builder(support_cache_size=0)
    calls = count_native_queries(monkeypatch)
    obs = observation_at([[5.25, 5.25]], [0.])
    for _ in range(3):
        builder.build(obs, bits, frontiers=fronts)
    assert len(calls) == 3 and not builder._support_cache


@pytest.mark.parametrize('capacity', [-1, 1.5, '2', True])
def test_cache_capacity_requires_nonnegative_integer(capacity):
    with pytest.raises(ValueError, match='cache size'):
        setup_builder(support_cache_size=capacity)


def test_scene_and_builder_geometry_do_not_share_cache(monkeypatch):
    builder, task, _, bits, fronts = setup_builder()
    second_scene = replace(builder.scene, scene_id='second', positions=builder.scene.positions[::-1])
    second = PrivilegedBuilder(builder.terrain, second_scene, task, builder.sensor)
    calls = count_native_queries(monkeypatch)
    obs = observation_at([[5.25, 5.25]], [0.])
    first = builder.build(obs, bits, frontiers=fronts)
    changed = second.build(obs, bits, frontiers=fronts)
    assert len(calls) == 2 and first.scene_id != changed.scene_id
    assert not np.array_equal(first.actions.support_indices, changed.actions.support_indices)
    builder.build(obs, bits, frontiers=fronts)
    second.build(obs, bits, frontiers=fronts)
    assert len(calls) == 2


def test_exact_continuous_xy_is_not_rounded_to_containing_cell(monkeypatch):
    builder, _, _, bits, fronts = setup_builder()
    calls = count_native_queries(monkeypatch)
    obs = observation_at([[5.01, 5.25], [5.25, 5.25]], [0.])
    state = builder.build(obs, bits, frontiers=fronts)
    assert len(calls) == 2
    a, b, c = state.actions.support_offsets
    assert not np.array_equal(state.actions.support_distances[a:b], state.actions.support_distances[b:c])
    builder.build(obs, bits, frontiers=fronts)
    assert len(calls) == 2


def test_hit_keeps_native_sensor_pose_and_start_validation():
    builder, _, _, bits, fronts = setup_builder()
    obs = observation_at([[5.25, 5.25]], [0.])
    builder.build(obs, bits, frontiers=fronts)
    builder.sensor = replace(builder.sensor, range_m=-1.)
    with pytest.raises(ValueError, match='invalid sensor'):
        builder.build(obs, bits, frontiers=fronts)
    builder.sensor = replace(builder.sensor, range_m=2.)
    builder.terrain.navigation = builder.terrain.navigation.copy()
    builder.terrain.navigation[10, 10] = 2
    with pytest.raises(ValueError, match='FREE start connection'):
        builder.build(obs, bits, frontiers=fronts)


def test_cached_descriptors_are_byte_identical_to_uncached_builder():
    builder, task, _, _, _ = setup_builder()
    rng = np.random.default_rng(10407)
    for revision in range(1, 9):
        known = rng.random(builder.terrain.shape) < .75
        bits = np.packbits(known.ravel(), bitorder='little')
        fronts = measured_frontiers(builder.terrain, task, builder.sensor, known, revision)
        obs = observation_at([[5.01, 5.25], [5.25, 5.25], [6.25, 5.25]],
                             [0., np.pi/4, np.pi])
        order = rng.permutation(len(obs.goals))
        obs = replace(obs, goals=obs.goals[order], action_nodes=obs.action_nodes[order],
                      action_yaws=obs.action_yaws[order])
        assert_same_bytes(builder.build(obs, bits, frontiers=fronts),
                          uncached_state(builder, obs, bits, fronts))
