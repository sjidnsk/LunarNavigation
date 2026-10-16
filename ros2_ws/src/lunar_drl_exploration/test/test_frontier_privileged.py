"""Endpoint removal oracle shares real frozen-frontier elimination semantics."""
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import Pose, SensorSpec
from lunar_drl_exploration.frontiers import build_frontiers, count_removed
from lunar_drl_exploration.task_analysis import measured_workspace
from lunar_drl_exploration.sensor import visible_cells
from test_task_analysis import snapshot, task


def fixture(*, origin=(0., 0.), resolution=1.):
    from lunar_drl_exploration.scene import TerrainGrid
    from lunar_drl_exploration.config import load_platform_config
    terrain = TerrainGrid.from_heights(np.zeros((13,13),np.float32), resolution, origin,
                                       load_platform_config())
    measured = np.full(terrain.shape, 2, np.uint8)
    measured[6,6] = 1; measured[6,5] = measured[6,7] = 0
    t = task(-100, -100, 100, 100)
    pose = Pose(origin[0]+6.5*resolution, origin[1]+6.5*resolution, 0.)
    s = snapshot(measured, measured, pose=pose, origin=(*origin, 0.), resolution=resolution)
    sensor = SensorSpec(range_m=2.1*resolution, fov_deg=90.)
    before = build_frontiers(s, measured_workspace(s,t,sensor), t)
    return terrain, measured, t, s, sensor, before


def actual_after(terrain, measured, t, s, sensor, poses):
    measured = measured.copy()
    for pose in poses:
        rows, cols = visible_cells(terrain.intrinsic, terrain.origin, terrain.resolution_m, pose, sensor)
        valid = np.isfinite(terrain.heights[rows,cols]) & (terrain.intrinsic[rows,cols] != 0)
        rows, cols = rows[valid], cols[valid]
        measured[rows,cols] = terrain.intrinsic[rows,cols]
    new = replace(snapshot(measured, measured, pose=s.pose, origin=s.origin,
                           resolution=s.resolution_m), revision=s.revision+1)
    return build_frontiers(new, measured_workspace(new,t,sensor), t)


def test_one_of_two_demands_is_visible_but_not_removed_and_whole_route_differs():
    from lunar_drl_exploration.privileged import predict_removed
    terrain, measured, t, s, sensor, before = fixture()
    assert len(before.inside.cell_ids) == 1
    assert predict_removed(before, terrain, s.pose, sensor) == 0
    after = actual_after(terrain, measured, t, s, sensor,
                         [s.pose, replace(s.pose, yaw=np.pi)])
    assert count_removed(before, after, domain='inside') == 1


@pytest.mark.parametrize('invalid', ['nan', 'unknown', 'blocked_center', 'blocked_demand', None])
def test_effective_reveal_matches_actual_elimination_and_current_free_gate(invalid):
    from lunar_drl_exploration.privileged import predict_removed
    terrain, measured, t, s, sensor, before = fixture()
    sensor = replace(sensor, fov_deg=360.)
    terrain.heights = terrain.heights.copy()
    terrain.intrinsic = terrain.intrinsic.copy()
    if invalid == 'nan': terrain.heights[6,7] = np.nan
    if invalid == 'unknown': terrain.intrinsic[6,7] = 0
    if invalid == 'blocked_center': terrain.intrinsic[6,6] = 2
    if invalid == 'blocked_demand': terrain.intrinsic[6,7] = 2
    expected = int(invalid in (None, 'blocked_demand'))
    assert predict_removed(before, terrain, s.pose, sensor) == expected
    after = actual_after(terrain, measured, t, s, sensor, [s.pose])
    assert count_removed(before, after, domain='inside') == expected
    assert before.revision == 1 and len(before.inside.cell_ids) == 1


@pytest.mark.parametrize('origin', [(-10.05,-4.15), (-1e-16,1e-16), (1e-16,-1e-16)])
def test_prediction_retains_signed_fractional_lattice_ids(origin):
    from lunar_drl_exploration.privileged import predict_removed
    terrain, measured, t, s, sensor, before = fixture(origin=origin, resolution=.2)
    sensor = replace(sensor, fov_deg=360.)
    assert predict_removed(before, terrain, s.pose, sensor) == 1
    after = actual_after(terrain, measured, t, s, sensor, [s.pose])
    assert count_removed(before, after, domain='inside') == 1


def test_raw_count_pack_normalizes_once_and_rejects_fractional_counts():
    from lunar_drl_exploration.batch import pack_privileged
    from test_model import observation, state, scene
    obs = observation(); st = state(obs=obs)
    st = replace(st, actions=replace(st.actions,
        removed_frontiers=np.full(len(obs.goals), 3, np.int64)))
    batch = pack_privileged([obs], [st], {'s':scene()}, 'cpu')
    np.testing.assert_allclose(batch.expected_removed_frontiers.numpy(), 3/50)
    assert st.actions.removed_frontiers.dtype == np.int64
    assert not st.actions.removed_frontiers.flags.writeable
    with pytest.raises(ValueError, match='nonnegative integer'):
        replace(st.actions, removed_frontiers=np.full(len(obs.goals), .5))


def test_outside_stance_can_remove_inside_front_but_outside_front_is_not_scored():
    from lunar_drl_exploration.privileged import predict_removed
    terrain, measured, _, s, sensor, _ = fixture()
    for t, expected in [(task(6.5, -100, 100, 100), 1),
                        (task(-100, -100, 0, 100), 0)]:
        before = build_frontiers(s, measured_workspace(s,t,sensor), t)
        stance = Pose(5.5, 6.5, 0.)
        assert predict_removed(before, terrain, stance, sensor) == expected
        after = actual_after(terrain, measured, t, s, sensor, [stance])
        assert count_removed(before, after, domain='inside') == expected


def test_hidden_truth_changes_critic_counts_without_changing_actor_or_actions():
    import copy
    import torch
    from lunar_drl_exploration.privileged import PrivilegedBuilder, build_truth
    from lunar_drl_exploration.model import Actor
    from lunar_drl_exploration.config import ModelConfig
    from test_privileged_actions import fixture as action_fixture, observation_at, initial_known, measured_frontiers
    terrain, task_spec, sensor, ref = action_fixture()
    truth = build_truth(terrain, ref)
    known = initial_known(terrain)
    before = measured_frontiers(terrain, task_spec, sensor, known)
    obs = observation_at([[5.25,5.25], [6.25,5.25]], [0., np.pi])
    actor = Actor(ModelConfig(width=16, heads=4, layers=1)).eval()
    goals, features = obs.goals.copy(), obs.features.copy()
    with torch.no_grad(): logits = actor([obs])[0].logits.clone()
    bits = np.packbits(known.ravel(), bitorder='little')
    first = PrivilegedBuilder(terrain, truth, task_spec, sensor).build(obs, bits, frontiers=before)
    hidden = copy.copy(terrain); hidden.heights = terrain.heights.copy()
    hidden.heights[10,11] = np.nan
    second = PrivilegedBuilder(hidden, truth, task_spec, sensor).build(obs, bits, frontiers=before)
    assert np.any(first.actions.removed_frontiers != second.actions.removed_frontiers)
    np.testing.assert_array_equal(obs.goals, goals)
    np.testing.assert_array_equal(obs.features, features)
    np.testing.assert_array_equal(first.actions.action_positions, second.actions.action_positions)
    np.testing.assert_array_equal(first.actions.action_yaws, second.actions.action_yaws)
    with torch.no_grad(): torch.testing.assert_close(actor([obs])[0].logits, logits, atol=0, rtol=0)
