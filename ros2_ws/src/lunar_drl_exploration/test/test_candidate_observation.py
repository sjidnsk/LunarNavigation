"""v5 preserves real targets and fine-graph cost while removing duplicate inputs."""
from dataclasses import replace
import importlib
import numpy as np
import pytest

from lunar_drl_exploration.contracts import Pose
from test_policy_graph import observation as raw_observation


def implementation():
    try:
        return importlib.import_module('lunar_drl_exploration.candidate_observation')
    except ImportError:
        pytest.fail('candidate observation implementation is missing')


def fine_observation():
    obs = raw_observation([(0, 0), (1, 0), (2, 0), (3, 0), (1, 1)],
        [(0, 1), (1, 2), (2, 3), (1, 4)], actions=[4])
    features = obs.features.copy()
    features[:, 19:] = 0
    return replace(obs, features=features, context=np.array([1., 0., .1, .2, 2., .5, .72, 1.]),
        polygon=np.array([[1., -.4], [2., -.4], [2., .4], [1., .4]]))


def candidate_observation(**kwargs):
    return implementation().prepare_candidate_observation(
        fine_observation(), Pose(0., 0., 0.), max_remote_positions=0, **kwargs).observation


def test_fine_distance_survives_contraction_and_world_goals_are_frozen():
    source = fine_observation()
    result = implementation().prepare_candidate_observation(source, Pose(0., 0., 0.),
        max_remote_positions=0)
    obs = result.observation
    assert obs.schema == 'task_graph_v5'
    assert obs.node_geometry.shape == (3, 5)
    assert obs.node_views.shape == (3, 8, 3)
    assert obs.node_is_real.tolist() == [True, False, True]
    assert obs.action_pairs.tolist() == [[2, 0]]
    np.testing.assert_allclose(obs.node_geometry[2], [.1, .1, .9, 0., .2], atol=1e-7)
    # The hub's representative edge sum is 4 m; original goal distance is 2 m.
    assert sum(obs.edge_lengths) == 4.
    np.testing.assert_array_equal(obs.node_geometry[1], 0.)
    np.testing.assert_array_equal(obs.node_views[1], 0.)
    np.testing.assert_array_equal(obs.goals, source.goals)
    np.testing.assert_array_equal(obs.context, source.context[2:6])
    np.testing.assert_array_equal(result.fine_observation.features, source.features)
    assert obs.build_pose == Pose(0., 0., 0.)
    for value in (obs.node_geometry, obs.node_views, obs.goals, obs.action_pairs):
        with pytest.raises(ValueError):
            value.setflags(write=True)


def test_remote_targets_are_real_and_zero_gain_local_transit_remains():
    raw = raw_observation([(i, 0) for i in range(12)], [(i, i+1) for i in range(11)],
        actions=np.repeat([1], 8), yaws=np.arange(8)*np.pi/4, gains=[9, 10], outside_gains=[11])
    f = raw.features.copy(); f[:, 19:] = 0
    result = implementation().prepare_candidate_observation(replace(raw, features=f), Pose(0., 0., 0.))
    obs = result.observation
    assert len(obs.node_ids) == 6 and len(obs.goals) == 24
    assert set(obs.node_ids[obs.action_nodes]) == {11, 19, 20}
    assert obs.node_is_real[obs.action_nodes].all()
    np.testing.assert_array_equal(obs.goals[:8], raw.goals)
    np.testing.assert_allclose(obs.node_geometry[obs.action_nodes[8], 4], .9)


def test_geometry_is_body_frame_and_heading_identity_tracks_world_catalog():
    raw = fine_observation()
    pose = Pose(10., 20., np.pi/2)
    rotate = np.array([[0., 1.], [-1., 0.]])
    positions = raw.positions @ rotate + [10., 20.]
    features = raw.features.copy()
    features[:, :2] = (positions - [10., 20.]) / 10.
    yaws = raw.action_yaws + np.pi/2
    rotated = replace(raw, positions=positions, features=features,
        polygon=raw.polygon @ rotate, action_yaws=yaws,
        goals=np.column_stack((positions[raw.action_nodes], yaws)))
    obs = implementation().prepare_candidate_observation(rotated, pose, max_remote_positions=0).observation
    expected = candidate_observation()
    np.testing.assert_allclose(obs.node_geometry, expected.node_geometry, atol=2e-7)
    np.testing.assert_allclose(obs.polygon, expected.polygon, atol=2e-7)
    assert obs.action_pairs[0, 1] == 2
    np.testing.assert_allclose(obs.heading_features[2], [1., 0.], atol=1e-7)


def test_concave_polygon_nearest_ties_boundary_and_vertex_order():
    module = implementation()
    polygon = np.array([[0., 0.], [4., 0.], [4., 1.], [1., 1.], [1., 4.], [0., 4.]])
    points = np.array([[.5, .5], [1., 2.], [2., 2.], [5., .5], [-1., -1.]])
    expected = np.array([[0., 0.], [0., 0.], [-1., 0.], [-1., 0.], [1., 1.]])
    for region in (polygon, polygon[::-1], np.roll(polygon, 3, axis=0)):
        np.testing.assert_allclose(module.nearest_region_vectors(points, region), expected)


def test_unreachable_fine_nodes_fail_instead_of_fabricating_finite_costs():
    raw = fine_observation()
    with pytest.raises(ValueError, match='reachable'):
        implementation().prepare_candidate_observation(replace(raw, edges=raw.edges[:1],
            edge_lengths=raw.edge_lengths[:1]), Pose(0., 0., 0.))


@pytest.mark.parametrize('field,value', [
    ('action_pairs', np.array([[1, 0]])),  # virtual target
    ('action_pairs', np.array([[2, 8]])),
    ('node_geometry', np.full((3, 5), np.nan)),
    ('context', np.zeros(8)),
    ('schema', 'task_graph_v4'),
])
def test_invalid_contract_fails_before_inference(field, value):
    obs = candidate_observation()
    with pytest.raises(ValueError):
        replace(obs, **{field: value})


def test_transport_keeps_v5_fields_aliases_and_refuses_mixed_replay():
    from lunar_drl_exploration.replay import dumps_transport, loads_transport, ReplayBuffer
    from test_replay_checkpoint import transition, scene
    obs = candidate_observation()
    restored = loads_transport(dumps_transport([obs, obs]))
    assert restored[0] is restored[1]
    np.testing.assert_array_equal(restored[0].node_geometry, obs.node_geometry)
    assert not restored[0].node_views.flags.writeable
    replay = ReplayBuffer(1024**2)
    replay.add(transition(obs, obs), {'s': scene()})
    with pytest.raises(ValueError, match='schema'):
        replay.add(transition(), {'s': scene()})
    assert len(replay) == 1
