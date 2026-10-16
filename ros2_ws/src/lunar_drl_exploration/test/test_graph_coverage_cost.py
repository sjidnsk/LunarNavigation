"""Coverage sweeps query only outstanding interfaces, without changing witnesses."""
from dataclasses import replace
import numpy as np
from lunar_drl_exploration.contracts import Pose, SensorSpec
from lunar_drl_exploration.decision import DecisionCore
from test_task_analysis import snapshot, task


def visibility_calls(monkeypatch):
    import lunar_drl_exploration.graph as graph
    calls = []
    original = graph.directional_visibility
    def counted(intrinsic, source, targets, *args):
        result = original(intrinsic, source, targets, *args)
        calls.append((tuple(source), targets.copy(), result.copy()))
        return result
    monkeypatch.setattr(graph, 'directional_visibility', counted)
    return calls


def test_coverage_stops_when_actual_anchor_represents_all_interfaces(monkeypatch):
    calls = visibility_calls(monkeypatch)
    measured = np.ones((18, 18), np.uint8)
    measured[8, 11] = 0
    core = DecisionCore(task(0, 0, 18, 18), SensorSpec(range_m=30.))
    observation, report = core.observe(snapshot(measured, measured, pose=Pose(8.5, 8.5, 0.)))
    assert len(report.frontier_cells) and len(core.builder._base_points) > 1
    assert calls[0][2].any(axis=1).all()
    # One complete query at the actual anchor for coverage, and its final
    # directional turn query. No executable base needs an optical query.
    assert len(calls) == 2
    assert not len(core.builder.unrepresented_interfaces)
    assert observation.features.shape[1] == 27


def test_coverage_skips_interface_rows_already_covered_by_earlier_sources(monkeypatch):
    calls = visibility_calls(monkeypatch)
    measured = np.ones((24, 24), np.uint8)
    measured[4:20:5, 4:20:5] = 0
    core = DecisionCore(task(0, 0, 24, 24), SensorSpec(range_m=4., offset_yaw_rad=.17))
    observation, report = core.observe(snapshot(measured, measured, pose=Pose(2.5, 2.5, .31)))
    assert not len(core.builder.unrepresented_interfaces)
    assert len(report.frontier_cells) > 2 and len(calls) > 2
    covered = set()
    # No supplemental witness is needed in this open scene; only the final
    # turn query intentionally revisits previously covered interfaces.
    for source, targets, seen in calls[:-1]:
        ids = set(map(tuple, targets))
        assert not ids & covered, 'coverage repeated an already represented interface'
        covered.update(map(tuple, targets[seen.any(axis=1)]))
    assert len(covered) == len(report.frontier_cells)
    assert len(observation.goals)


def test_sparse_witness_and_arrival_contraction_remain_represented():
    measured = np.zeros((12, 12), np.uint8)
    measured[3, 3:6] = 1
    intrinsic = np.ones((12, 12), np.uint8)
    intrinsic[4, 3] = 2
    intrinsic[5, 4] = 0
    snap = replace(snapshot(measured, intrinsic, pose=Pose(.7, .7, 0.), resolution=.2),
                   goal_position_tolerance_m=.3)
    core = DecisionCore(task(.8, 1., 1., 1.2), SensorSpec())
    observation, report = core.observe(snap)
    assert report.available and not report.exhausted
    assert not len(core.builder.unrepresented_interfaces)
    assert np.any(np.all(np.isclose(observation.goals[:, :2], [1.1, .7]), axis=1))
