"""Measured FREE centers and same-domain eight-neighbor demand contracts."""
from dataclasses import FrozenInstanceError
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import SensorSpec
from lunar_drl_exploration.frontiers import build_frontiers
from lunar_drl_exploration.task_analysis import measured_workspace, TaskAnalyzer
from test_task_analysis import snapshot, task


def frontiers(b, t=None, *, k=None, origin=(0., 0., 0.)):
    t = task(0, 0, 9, 9) if t is None else t
    s = snapshot(b, b, k, origin=origin)
    w = measured_workspace(s, t, SensorSpec(range_m=2))
    if k is not None:
        from lunar_drl_exploration.coverage import CoverageHistory
        h = CoverageHistory(t); h.consume(s)
        w = measured_workspace(s, t, SensorSpec(range_m=2), h)
    return build_frontiers(s, w, t)


def demands(f, center):
    i = f.centers.tolist().index(center)
    return f.demand_ids[f.demand_offsets[i]:f.demand_offsets[i+1]]


def test_only_measured_free_centers_with_eight_neighbor_unknown_demands():
    b = np.full((9, 9), 2, np.uint8)
    b[4, 4] = 1; b[3, 3] = b[4, 5] = 0
    f = frontiers(b)
    assert f.inside.centers.tolist() == [[4.5, 4.5]]
    assert f.inside.demand_offsets.tolist() == [0, 2]
    assert len(set(f.inside.demand_ids)) == 2
    assert not len(f.outside.cell_ids)
    k = b.copy(); k[4, 4] = 0
    assert not len(frontiers(b, k=k).inside.cell_ids)


def test_boundary_centers_and_unknown_support_both_belong_to_same_domain():
    b = np.full((9, 9), 2, np.uint8)
    b[4, 4] = b[4, 5] = 1
    b[3, 3] = b[3, 5] = 0
    t = task(0, 0, 4.5, 9)
    f = frontiers(b, t)
    assert f.inside.centers.tolist() == [[4.5, 4.5]]
    assert f.outside.centers.tolist() == [[5.5, 4.5]]
    assert len(demands(f.inside, [4.5, 4.5])) == 1
    assert len(demands(f.outside, [5.5, 4.5])) == 1
    b[3, 3] = 2
    assert not len(frontiers(b, t).inside.cell_ids)
    b[3, 3] = 0; b[3, 5] = 2
    assert not len(frontiers(b, t).outside.cell_ids)


def test_frozen_arrays_and_analyzer_core_exposure_preserve_unknown_certificates():
    from lunar_drl_exploration.decision import DecisionCore
    b = np.full((9, 9), 2, np.uint8); b[4, 4] = 1; b[4, 5] = 0
    s = snapshot(b, b); t = task(0, 0, 9, 9)
    a = TaskAnalyzer(t, SensorSpec(range_m=2)); report = a.update(s)
    f = a.frontiers
    assert f.inside.centers.tolist() == [[4.5, 4.5]]
    assert report.frontier_cells.tolist() == [[5, 4]]
    assert report.witnesses.tolist() == [[4, 4]]
    for array in (f.known_ids, f.free_ids, f.inside.cell_ids, f.inside.centers,
                  f.inside.demand_ids, f.inside.demand_offsets):
        with pytest.raises(ValueError): array.setflags(write=True)
    with pytest.raises(FrozenInstanceError): f.revision = 7
    c = DecisionCore(t, SensorSpec(range_m=2)); c.observe(s)
    assert c.frontiers is c.analyzer.frontiers
    with pytest.raises(AttributeError): c.frontiers = f


def test_signed_world_ids_and_metric_centers_survive_fractional_negative_origin_shifts():
    from test_cumulative_coverage import shifted_snapshot
    from lunar_drl_exploration.frontiers import count_removed
    t = task(-60,-60,60,60)
    a = TaskAnalyzer(t, SensorSpec(range_m=.2))
    cells = [(-257,-1,1),(-1,0,1),(0,0,1),(256,1,1)]
    first = shifted_snapshot(cells, origin=(.05,.15,0.))
    a.update(first); before = a.frontiers
    moved = shifted_snapshot(cells, origin=(-51.35,51.35,0.),revision=2)
    a.update(moved); after = a.frontiers
    assert after.lattice_origin == before.lattice_origin
    assert after.inside.cell_ids.tolist() == [-1099511627777,-4294967296,0,1099511627777]
    assert np.array_equal(before.inside.cell_ids, after.inside.cell_ids)
    np.testing.assert_allclose(after.inside.centers,
        [[-51.25,.05],[-.05,.25],[.15,.25],[51.35,.45]],atol=1e-12)
    assert np.array_equal(before.inside.demand_offsets, after.inside.demand_offsets)
    assert np.array_equal(before.inside.demand_ids, after.inside.demand_ids)
    assert np.array_equal(before.known_ids, after.known_ids)
    assert count_removed(before, after, domain='inside') == 0
    # Connectivity is irrelevant to measured classification and frontier membership.
    a.update(replace(moved, revision=3, start_connections=np.empty((0,2),int),
                     start_connection_status='INPUT_UNAVAILABLE'))
    assert np.array_equal(a.frontiers.inside.cell_ids, after.inside.cell_ids)


def test_frontier_csr_rejects_empty_demands_and_duplicate_centers():
    from lunar_drl_exploration.frontiers import FrontierSet
    with pytest.raises(ValueError):
        FrontierSet([1], [[.5,.5]], [0,0], [])
    with pytest.raises(ValueError):
        FrontierSet([1,1], [[.5,.5],[.5,.5]], [0,1,2], [2,3])


def test_world_id_range_and_lattice_validation_do_not_silently_alias():
    from lunar_drl_exploration.geometry import pack_cell_ids, lattice_coordinates
    assert pack_cell_ids([-1,0],[-1,-1]).tolist() == [-1,4294967295]
    for xy in [(2**31,0),(0,-2**31-1)]:
        with pytest.raises(ValueError): pack_cell_ids(*xy)
    for origin, resolution in [((0,0),0),((0,0),float('nan')),((float('inf'),0),1)]:
        with pytest.raises(ValueError): lattice_coordinates(origin,resolution)
