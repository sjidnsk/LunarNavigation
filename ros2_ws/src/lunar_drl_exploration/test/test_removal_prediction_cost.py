"""Exact endpoint predictions without work proportional to the whole known map."""
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.frontiers import FrontierSet, count_removed
from lunar_drl_exploration.geometry import lattice_displacement, pack_cell_ids
from lunar_drl_exploration.maps import FREE, UNKNOWN
from lunar_drl_exploration.sensor import visible_cells
from lunar_drl_exploration.privileged import predict_removed
from test_frontier_privileged import fixture


def snapshot_oracle(before, terrain, pose, sensor):
    """Pre-optimization virtual-snapshot algorithm, kept independent of predictor."""
    offset = lattice_displacement(terrain.origin, before.lattice_origin, before.resolution_m)
    rows, cols = visible_cells(terrain.intrinsic, terrain.origin, terrain.resolution_m, pose, sensor)
    classes = terrain.intrinsic[rows, cols]
    valid = np.isfinite(terrain.heights[rows, cols]) & (classes != UNKNOWN)
    hits = pack_cell_ids(cols[valid]+offset[0], rows[valid]+offset[1])
    known = np.union1d(before.known_ids, hits)
    free = np.union1d(np.setdiff1d(before.free_ids, hits), hits[classes[valid] == FREE])
    def remaining(old):
        unknown = ~np.isin(old.demand_ids, known)
        counts = (np.add.reduceat(unknown.astype(np.int64), old.demand_offsets[:-1])
                  if len(old.cell_ids) else np.empty(0, np.int64))
        kept = (counts > 0) & np.isin(old.cell_ids, free)
        return FrontierSet(old.cell_ids[kept], old.centers[kept],
            np.r_[0, np.cumsum(counts[kept])],
            old.demand_ids[unknown & np.repeat(kept, np.diff(old.demand_offsets))])
    after = replace(before, revision=before.revision+1, known_ids=known, free_ids=free,
                    inside=remaining(before.inside), outside=remaining(before.outside))
    return count_removed(before, after, domain='inside')


def test_candidate_prediction_does_not_merge_the_whole_known_map(monkeypatch):
    terrain, _, _, s, sensor, before = fixture()
    remote = pack_cell_ids(np.arange(100000)+100, np.full(100000, -100))
    before = replace(before, known_ids=np.union1d(before.known_ids, remote))
    expected = snapshot_oracle(before, terrain, s.pose, replace(sensor, fov_deg=360.))
    original = np.union1d
    def bounded_union(a, b):
        if len(a) > 10000 or len(b) > 10000:
            pytest.fail('candidate prediction merged the whole known map')
        return original(a, b)
    monkeypatch.setattr(np, 'union1d', bounded_union)
    assert predict_removed(before, terrain, s.pose, replace(sensor, fov_deg=360.)) == expected == 1


@pytest.mark.parametrize('origin', [(0., 0.), (-10.05, -4.15), (-1e-16, 1e-16)])
def test_prediction_matches_snapshot_oracle_for_mixed_effective_reveals(origin):
    terrain, _, _, s, sensor, before = fixture(origin=origin, resolution=.2)
    rng = np.random.default_rng(91017)
    for _ in range(30):
        terrain.intrinsic = rng.choice([UNKNOWN, FREE, 2], terrain.shape).astype(np.uint8)
        terrain.heights = np.where(rng.random(terrain.shape) < .1, np.nan, 0.).astype(np.float32)
        # Include already-known demands and a formerly non-FREE center. Public
        # frozen snapshots do not require K/demand disjointness or FREE centers.
        known = before.known_ids if rng.random() < .5 else np.union1d(before.known_ids, before.inside.demand_ids[:1])
        free = before.free_ids if rng.random() < .5 else np.empty(0, np.int64)
        current = replace(before, known_ids=known, free_ids=free)
        spec = replace(sensor, fov_deg=float(rng.choice([90, 360])), offset_yaw_rad=.31)
        pose = replace(s.pose, yaw=float(rng.uniform(-np.pi, np.pi)))
        assert predict_removed(current, terrain, pose, spec) == snapshot_oracle(current, terrain, pose, spec)
        assert not current.known_ids.flags.writeable


def test_empty_inside_still_validates_sensor_and_pose():
    terrain, _, _, s, sensor, before = fixture()
    empty = FrontierSet(np.empty(0, np.int64), np.empty((0,2)), [0], [])
    before = replace(before, inside=empty, outside=before.inside)
    assert predict_removed(before, terrain, s.pose, sensor) == 0
    with pytest.raises(ValueError, match='invalid sensor'):
        predict_removed(before, terrain, s.pose, replace(sensor, range_m=-1))
    with pytest.raises(ValueError, match='outside provided grid'):
        predict_removed(before, terrain, replace(s.pose, x=1000.), sensor)


def test_shared_demands_and_batch_refresh_match_oracle():
    from lunar_drl_exploration.contracts import Pose
    from lunar_drl_exploration.privileged import PrivilegedBuilder, build_truth
    from test_privileged_actions import fixture as action_fixture, observation_at, measured_frontiers
    terrain, task, sensor, ref = action_fixture()
    builder = PrivilegedBuilder(terrain, build_truth(terrain, ref), task, sensor)
    obs = observation_at([[5.25,5.25], [6.25,5.25]], np.arange(8)*np.pi/4)
    rng = np.random.default_rng(4051)
    for revision in range(1, 8):
        known = rng.random(terrain.shape) < .75
        before = measured_frontiers(terrain, task, sensor, known, revision)
        assert len(np.unique(before.inside.demand_ids)) < len(before.inside.demand_ids)
        expected = [snapshot_oracle(before, terrain, Pose(*goal), sensor) for goal in obs.goals]
        state = builder.build(obs, np.packbits(known.ravel(), bitorder='little'), frontiers=before)
        np.testing.assert_array_equal(state.actions.removed_frontiers, expected)
        assert not state.actions.removed_frontiers.flags.writeable
