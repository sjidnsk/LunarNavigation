"""Removal counts old solved fronts, never net count or classification loss."""
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import SensorSpec
from lunar_drl_exploration.frontiers import count_removed
from lunar_drl_exploration.task_analysis import TaskAnalyzer
from test_task_analysis import snapshot, task


def run_sequence(arrays, t=None):
    a = TaskAnalyzer(task(0, 0, 9, 9) if t is None else t, SensorSpec(range_m=2))
    result = []
    for revision, b in enumerate(arrays, 1):
        a.update(replace(snapshot(b, b), revision=revision)); result.append(a.frontiers)
    return result


def test_partial_demands_do_not_remove_but_all_demands_do():
    b = np.full((9, 9), 2, np.uint8); b[4, 4] = 1; b[3, 3] = b[4, 5] = 0
    p = b.copy(); p[3, 3] = 2
    done = p.copy(); done[4, 5] = 2
    first, partial, final = run_sequence([b, p, done])
    assert count_removed(first, partial, domain='inside') == 0
    assert count_removed(first, final, domain='inside') == 1
    assert count_removed(final, final, domain='inside') == 0
    assert count_removed(first, replace(final, revision=first.revision), domain='inside') == 0


def test_missing_front_entry_is_not_evidence_that_all_frozen_demands_are_known():
    # A lost entry (e.g. a bounded consumer projection) must not bypass the
    # frozen-demand gate even when the old center is still measured FREE.
    b = np.full((9, 9), 2, np.uint8); b[4, 4] = 1; b[3, 3] = b[4, 5] = 0
    p = b.copy(); p[3, 3] = 2
    done = p.copy(); done[4, 5] = 2
    first, partial, final = run_sequence([b, p, done])
    dropped = replace(partial, inside=final.inside)
    assert count_removed(first, dropped, domain='inside') == 0
    # Conversely known demands alone do not remove an entry that still exists.
    retained = replace(final, inside=first.inside)
    assert count_removed(first, retained, domain='inside') == 0


def test_new_fronts_do_not_offset_removal_of_old_front():
    b = np.full((9, 9), 2, np.uint8); b[4, 4] = 1; b[4, 5] = 0
    b[1, 1:4] = 0
    done = b.copy(); done[4, 5] = 2; done[2, 1:4] = 1
    first, final = run_sequence([b, done])
    assert len(final.inside.cell_ids) == 3
    assert count_removed(first, final, domain='inside') == 1


def test_outside_observation_does_not_pay_inside_and_reclassification_never_pays():
    b = np.full((9, 9), 2, np.uint8)
    b[4, 4:6] = 1; b[3, 3] = b[3, 5] = 0
    done = b.copy(); done[3, 5] = 2
    first, final = run_sequence([b, done], task(0, 0, 4.5, 9))
    assert count_removed(first, final, domain='inside') == 0
    assert count_removed(first, final, domain='outside') == 1
    for center in (0, 2):
        done = b.copy(); done[3, 3] = done[3, 5] = 2; done[4, 4] = center
        first, final = run_sequence([b, done])
        assert count_removed(first, final, domain='inside') == 1  # only x=5 stays FREE


def test_window_drop_keeps_known_but_does_not_award_lost_free_center():
    b = np.full((9, 9), 2, np.uint8); b[4, 4] = 1; b[4, 5] = 0
    a = TaskAnalyzer(task(0, 0, 9, 9), SensorSpec(range_m=2))
    s = snapshot(b, b); a.update(s); first = a.frontiers
    a.update(replace(s, revision=2, tiles={})); final = a.frontiers
    assert np.array_equal(first.known_ids, final.known_ids)
    assert not len(final.free_ids)
    assert count_removed(first, final, domain='inside') == 0


@pytest.mark.parametrize('change', [dict(epoch='new'), dict(resolution_m=.5),
    dict(lattice_origin=(.25, 0.)), dict(task_id='other'), dict(polygon_digest='other')])
def test_incompatible_snapshots_rejected(change):
    b = np.ones((9, 9), np.uint8); first, final = run_sequence([b, b])
    with pytest.raises(ValueError): count_removed(first, replace(final, **change), domain='inside')


def test_invalid_domain_and_stale_revision_rejected():
    b = np.ones((9, 9), np.uint8); first, final = run_sequence([b, b])
    with pytest.raises(ValueError): count_removed(first, final, domain='both')
    with pytest.raises(ValueError): count_removed(final, first, domain='inside')


@pytest.mark.parametrize('coordinate', [.02469135781, -1e-16, 1e-16])
def test_independent_frontier_snapshots_accept_quantization_boundary_integer_shift(coordinate):
    from lunar_drl_exploration.frontiers import build_frontiers
    from lunar_drl_exploration.task_analysis import measured_workspace
    origin = (coordinate, coordinate, 0.)
    b = np.full((9,9),2,np.uint8); b[4,4] = 1; b[4,5] = 0
    first = snapshot(b,b,resolution=.2,origin=origin)
    b[4,5] = 2
    done = snapshot(b,b,resolution=.2,origin=origin)
    moved = replace(done,origin=(origin[0]+256*.2,origin[1]-256*.2,0.),
                    tiles={(-1,1):done.tiles[(0,0)]},revision=2,
                    start_connections=done.start_connections+[-256,256])
    t = task(-1,-1,3,3); sensor = SensorSpec(range_m=.2)
    before = build_frontiers(first,measured_workspace(first,t,sensor),t)
    after = build_frontiers(moved,measured_workspace(moved,t,sensor),t)
    assert len(before.inside.cell_ids) == 1 and len(after.inside.cell_ids) == 0
    assert count_removed(before,after,domain='inside') == 1
