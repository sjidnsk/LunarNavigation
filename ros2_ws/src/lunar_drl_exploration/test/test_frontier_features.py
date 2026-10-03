"""Domain frontier counts, pending turns and frozen action-column alignment."""
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import Pose, SensorSpec
from lunar_drl_exploration.decision import DecisionCore
from lunar_drl_exploration.frontiers import FrontierSet, build_frontiers
from lunar_drl_exploration.history import HEADINGS
from lunar_drl_exploration.task_analysis import measured_workspace
from lunar_drl_exploration.sensor import directional_visibility, visible_cells
from test_task_analysis import snapshot, task


def single_frontier(b, center=(4, 4)):
    sensor = SensorSpec(range_m=10, fov_deg=360)
    s = snapshot(b, b, pose=Pose(2.5, 2.5, 0))
    w = measured_workspace(s, task(0, 0, 9, 9), sensor)
    f = build_frontiers(s, w, task(0, 0, 9, 9)).inside
    i = np.flatnonzero(np.all(f.centers == np.array(center)+.5, axis=1))[0]
    ids = f.demand_ids[f.demand_offsets[i]:f.demand_offsets[i+1]]
    return FrontierSet(f.cell_ids[i:i+1], f.centers[i:i+1], [0, len(ids)], ids), w


@pytest.mark.parametrize('cause', ['blocked', 'fov'])
def test_visible_center_requires_a_visible_same_domain_demand(cause):
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    b = np.ones((9, 9), np.uint8)
    if cause == 'blocked':
        b[5, :] = b[:, 5] = 2; b[5, 5] = 0
        source, sensor, heading = [2.5, 2.5], SensorSpec(range_m=10, fov_deg=360), 1
    else:
        b[5, 5] = 0
        source, sensor, heading = [2.5, 4.5], SensorSpec(range_m=10, fov_deg=5), 0
    f, w = single_frontier(b)
    local = np.array([source])-w.origin
    center = np.floor((f.centers-w.origin)).astype(int)
    demand = np.column_stack((f.demand_ids >> 32, (f.demand_ids << 32) >> 32))-w.bounds[:2]
    seen = directional_visibility(w.intrinsic, np.floor(local[0]).astype(int),
        np.vstack((center, demand)), 10, HEADINGS, np.deg2rad(sensor.fov_deg))
    assert seen[0, heading] and not seen[1:, heading].any()
    counts = directional_frontier_counts(f, w, np.array([source]), sensor, 1.)
    assert counts.dtype == np.int64 and counts.shape == (1, 8)
    assert counts[0, heading] == 0


def test_two_demands_with_one_visible_count_the_free_center_once():
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    b = np.ones((9, 9), np.uint8)
    b[5, :] = b[:, 5] = 2; b[5, 5] = b[4, 3] = 0
    f, w = single_frontier(b)
    assert len(f.demand_ids) == 2
    source = [2.5, 2.5]
    sensor = SensorSpec(range_m=10, fov_deg=360)
    rows, cols = visible_cells(w.intrinsic, w.origin, 1., Pose(*source, 0), sensor)
    seen = set(zip(cols+w.bounds[0], rows+w.bounds[1]))
    assert (4, 4) in seen and (3, 4) in seen and (5, 5) not in seen
    np.testing.assert_array_equal(directional_frontier_counts(f, w, [source], sensor, 1.),
                                  np.ones((1, 8), np.int64))


def test_visible_demand_alone_does_not_count_an_unseen_free_center():
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    b = np.ones((9, 9), np.uint8); b[3, 4] = 0
    f, w = single_frontier(b)
    sensor = SensorSpec(range_m=10, fov_deg=5, offset_yaw_rad=np.arctan2(-1, 2))
    source = [2.5, 4.5]
    rows, cols = visible_cells(w.intrinsic, w.origin, 1., Pose(*source, 0), sensor)
    seen = set(zip(cols+w.bounds[0], rows+w.bounds[1]))
    assert (4, 3) in seen and (4, 4) not in seen
    assert directional_frontier_counts(f, w, [source], sensor, 1.)[0, 0] == 0


@pytest.mark.parametrize('initial_x', [-1e-16, 1e-16])
def test_independent_workspace_reuses_snapshot_lattice_near_modulo_boundary(initial_x):
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    sensor = SensorSpec(range_m=2., fov_deg=360)
    b = np.ones((9, 9), np.uint8)
    b[5, :] = b[:, 5] = 2; b[5, 5] = 0
    origin = (initial_x, .02469135781, 0.)
    xy = [initial_x+.5, origin[1]+.5]
    s = snapshot(b, b, pose=Pose(*xy, 0), resolution=.2, origin=origin)
    t = task(-2, -2, 2, 2)
    w = measured_workspace(s, t, sensor)
    f = build_frontiers(s, w, t).inside
    center = [initial_x+.9, origin[1]+.9]
    i = np.flatnonzero(np.all(np.isclose(f.centers, center), axis=1))[0]
    ids = f.demand_ids[f.demand_offsets[i]:f.demand_offsets[i+1]]
    f = FrontierSet(f.cell_ids[i:i+1], f.centers[i:i+1], [0, len(ids)], ids)
    np.testing.assert_array_equal(directional_frontier_counts(f, w, [xy], sensor, .2),
                                  np.zeros((1, 8), np.int64))
    from lunar_drl_exploration.graph import GraphBuilder
    from lunar_drl_exploration.history import DirectionHistory
    core = DecisionCore(t, sensor)
    frozen, report = core.observe(s)
    independent = GraphBuilder().build(s, report, DirectionHistory(.1), t, sensor, (0, 0))
    np.testing.assert_array_equal(independent.features, frozen.features)


def test_core_passes_frozen_frontiers_without_rebuilding_and_standalone_matches(monkeypatch):
    import lunar_drl_exploration.graph as graph
    from lunar_drl_exploration.history import DirectionHistory
    sensor = SensorSpec(range_m=3)
    b = np.zeros((9, 9), np.uint8); b[3:6, 3:6] = 1
    s, t = snapshot(b, b), task(0, 0, 9, 9)
    core = DecisionCore(t, sensor)
    def forbidden(*args):
        raise AssertionError('Core must pass its analyzer frozen frontier snapshot')
    with monkeypatch.context() as scoped:
        scoped.setattr(graph, 'build_frontiers', forbidden)
        frozen, report = core.observe(s)
    standalone = graph.GraphBuilder().build(s, report, DirectionHistory(.1), t, sensor, (0, 0))
    np.testing.assert_array_equal(frozen.features, standalone.features)
    with pytest.raises(ValueError, match='frontiers do not match'):
        graph.GraphBuilder().build(s, report, core.history, t, sensor, (0, 0),
                                  frontiers=replace(core.frontiers, revision=2))


def test_outside_stance_inside_gain_and_outside_channel_are_independent():
    b = np.full((13, 15), 2, np.uint8)
    b[6, 2:11] = 1; b[6, 11] = b[5, 2] = 0
    sensor = SensorSpec(range_m=10, fov_deg=90)
    core = DecisionCore(task(7, 4, 13, 9), sensor)
    o, _ = core.observe(snapshot(b, b, pose=Pose(4.5, 6.5, np.pi)))
    assert o.schema == 'task_graph_v4' and o.features.shape[1] == 27
    assert o.features[o.current_index, 2] == 0
    assert o.features[o.current_index, 3] > 0
    assert o.features[o.current_index, 15] > 0
    assert o.features[o.current_index, 7] == 0
    assert o.features[o.current_index, 11] == 0


def test_external_frontiers_do_not_create_task_turn():
    b = np.zeros((16, 45), np.uint8); b[5:11, 2:9] = 1
    core = DecisionCore(task(30, 5, 40, 12), SensorSpec(range_m=3))
    o, report = core.observe(snapshot(b, b, pose=Pose(4.5, 7.5, 0)))
    assert not len(core.frontiers.inside.cell_ids) and len(core.frontiers.outside.cell_ids)
    assert not len(report.frontier_cells)
    assert o.features[o.current_index, 11:19].max() > 0
    assert len(o.goals) and np.all(o.action_nodes != o.current_index)


def test_cross_boundary_pending_turn_survives_zero_inside_frontiers():
    b = np.full((9, 9), 2, np.uint8)
    b[4, 2:5] = 1; b[4, 5] = 0
    core = DecisionCore(task(5, 4, 6, 5), SensorSpec(range_m=4))
    o, report = core.observe(snapshot(b, b, pose=Pose(3.5, 4.5, np.pi)))
    assert not len(core.frontiers.inside.cell_ids) and len(report.frontier_cells)
    assert np.all(o.features[:, 3:11] == 0)
    assert np.any(o.action_nodes == o.current_index)


def test_frontier_counts_match_native_observation_for_all_directions_and_lattice_shifts():
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    from test_cumulative_coverage import shifted_snapshot
    from lunar_drl_exploration.task_analysis import TaskAnalyzer
    t = task(-3, -3, 3, 3)
    sensor = SensorSpec(range_m=.8, fov_deg=90, offset_yaw_rad=.17)
    a = TaskAnalyzer(t, sensor)
    cells = [(-1, 0, 1), (0, 0, 1), (1, 0, 1)]
    before = None
    for revision, origin in enumerate([(.02469135781, .15, 0.),
                                      (51.22469135781, -51.05, 0.)], 1):
        s = shifted_snapshot(cells, origin=origin, revision=revision)
        a.update(s); w, f = a.workspace, a.frontiers.inside
        points = np.array([[.32469135781, .25], [-.07530864219, .25]])
        counts = directional_frontier_counts(f, w, points, sensor, .2)
        for i, xy in enumerate(points):
            for heading in range(8):
                rows, cols = visible_cells(w.intrinsic, w.origin, .2,
                    Pose(*xy, HEADINGS[heading]), sensor)
                visible = set(zip(cols, rows))
                offset = np.array(w.bounds[:2])+w.lattice_offset
                expected = 0
                for j, center in enumerate(f.centers):
                    center_cell = tuple(np.floor((center-w.origin)/.2).astype(int))
                    ids = f.demand_ids[f.demand_offsets[j]:f.demand_offsets[j+1]]
                    demands = np.column_stack((ids >> 32, (ids << 32) >> 32))-offset
                    expected += center_cell in visible and any(tuple(d) in visible for d in demands)
                assert counts[i, heading] == expected
        if before is not None: np.testing.assert_array_equal(counts, before)
        before = counts


def test_manual_action_columns_and_actor_critic_outputs_follow_action_order():
    torch = pytest.importorskip('torch')
    from lunar_drl_exploration.batch import pack_observations
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.model import Actor, Critic
    from test_model import observation, scene, state
    torch.manual_seed(29)
    base = observation(n=3, locations=2)
    features = np.zeros((3, 27), np.float32)
    features[:, :2] = base.features[:, :2]
    for node in range(3):
        features[node, 3:11] = (100*node+np.arange(8))/100.
        features[node, 11:19] = (200*node+np.arange(8)+10)/100.
        features[node, 19:27] = (node+np.arange(8)) % 2
    order = np.array([15, 0, 10, 4, 8, 7])
    o = replace(base, features=features, action_nodes=base.action_nodes[order],
                action_yaws=base.action_yaws[order], goals=base.goals[order])
    packed = pack_observations([o, o], 'cpu')
    headings = np.array([7, 0, 2, 4, 0, 7])
    expected = np.column_stack((features[o.action_nodes, :2],
        np.cos(o.action_yaws), np.sin(o.action_yaws),
        features[o.action_nodes, 3+headings], features[o.action_nodes, 11+headings],
        features[o.action_nodes, 19+headings]))
    np.testing.assert_allclose(packed.action_features.numpy(), np.vstack((expected, expected)))
    assert packed.action_features.shape == (12, 7)
    config = ModelConfig(width=16, heads=2, layers=1)
    actor, critic = Actor(config).eval(), Critic(config).eval()
    perm = np.array([3, 0, 5, 1, 4, 2])
    shuffled = replace(o, action_nodes=o.action_nodes[perm],
                       action_yaws=o.action_yaws[perm], goals=o.goals[perm])
    with torch.no_grad():
        logits = actor([o, shuffled])
        values = critic([o, shuffled], [state(obs=o), state(obs=shuffled)], {'s': scene()})
    torch.testing.assert_close(logits[1].logits, logits[0].logits[perm])
    torch.testing.assert_close(values[1], values[0][perm])
