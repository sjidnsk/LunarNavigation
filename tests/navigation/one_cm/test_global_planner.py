"""厘米细节、连续几何、有限搜索域与资源边界的行为测试。"""
import importlib
import tracemalloc
from dataclasses import replace

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WheelLimits, WorldSnapshot


def load_api():
    name = "scripts.navigation.one_cm.nav1cm.global_planner"
    assert importlib.util.find_spec(name) is not None, "global planner is not implemented"
    return importlib.import_module(name)


def world(rects=(), epoch=1):
    return WorldSnapshot(epoch, tuple(rects), (CellRect(-100, -100, 100, 100),))


def test_unknown_and_outside_known_map_preserve_identity_and_input():
    api = load_api()
    source = world()
    result = api.AdaptiveGlobalPlanner(WheelLimits()).plan(source, (0, 0), (1000, 37), 8)
    assert (result.planning_outcome, result.reason_code, result.has_reference) == (0, "PLAN_FOUND", True)
    assert result.route.map_epoch == 1 and result.route.version == 8
    assert tuple(result.route.points[-1]) == (1000, 37)
    assert result.domain[2] > 1000
    assert source == world()


def test_raw_goal_obstacle_differs_from_clearance_and_half_open_boundary():
    api = load_api()
    planner = api.AdaptiveGlobalPlanner(WheelLimits())
    source = world([CellRect(200, -100, 201, 100)])
    assert planner.check_goal(source, (2.005, 0)).reason_code == "GOAL_IN_OBSTACLE"
    assert planner.check_goal(source, (2.01, 0)).reason_code == "GOAL_CLEARANCE"
    assert planner.check_goal(source, (1.7, 0)).reason_code == "GOAL_CLEARANCE"
    assert planner.check_goal(source, (4, 0)).accepted
    assert planner.plan(source, (0, 0), (2.005, 0), 1).route is None


def test_complete_segment_collision_not_only_endpoints():
    api = load_api()
    planner = api.AdaptiveGlobalPlanner(WheelLimits())
    source = world([CellRect(200, -100, 201, 100)])
    result = planner.plan(source, (0, 0), (4, 0), 1)
    assert result.has_reference
    assert len(result.route.points) > 2
    # 任意穿越 x=2 的线段都必须越过墙端并保持半车宽净空。
    for a, b in zip(result.route.points[:-1], result.route.points[1:]):
        if min(a[0], b[0]) <= 2 <= max(a[0], b[0]) and a[0] != b[0]:
            y = a[1] + (b[1] - a[1]) * (2 - a[0]) / (b[0] - a[0])
            assert abs(y) > 1.409


@pytest.mark.parametrize("gap_cm", [82, 83])
def test_one_centimeter_gate_difference_survives_decomposition(gap_cm):
    api = load_api()
    planner = api.AdaptiveGlobalPlanner(WheelLimits(), domain_bounds_m=(-3, -4, 3, 4))
    open_gate = world([CellRect(-1, -1000, 1, 0), CellRect(-1, gap_cm, 1, 1000)])
    closed_gate = world([CellRect(-1, -1000, 1, 0), CellRect(-1, 81, 1, 1000)])
    good = planner.plan(open_gate, (-2, gap_cm / 200), (2, gap_cm / 200), 1)
    bad = planner.plan(closed_gate, (-2, gap_cm / 200), (2, gap_cm / 200), 1)
    assert good.has_reference
    assert bad.reason_code == "DOMAIN_EXHAUSTED" and not bad.has_reference


def test_closed_ring_proves_enclosure_not_domain_or_budget_exhaustion():
    api = load_api()
    ring = world([CellRect(-200, -200, 200, -190), CellRect(-200, 190, 200, 200),
                  CellRect(-200, -200, -190, 200), CellRect(190, -200, 200, 200)])
    planner = api.AdaptiveGlobalPlanner(WheelLimits())
    result = planner.plan(ring, (0, 0), (5, 0), 1)
    assert result.reason_code == "NO_PATH_ENCLOSED"
    assert planner.plan(ring, (0, 0), (5, 0), 1, budget_s=0).reason_code == "BUDGET_EXHAUSTED"


def test_radius_validation_and_start_goal_errors():
    api = load_api()
    with pytest.raises(ValueError):
        api.AdaptiveGlobalPlanner(WheelLimits(), clearance_radius_m=.1)
    planner = api.AdaptiveGlobalPlanner(WheelLimits(), clearance_radius_m=.72)
    assert planner.check_goal(world([CellRect(200, -100, 201, 100)]), (1.4, 0)).reason_code == "GOAL_CLEARANCE"
    assert planner.plan(world(), (0, 0), (0, 0), 1).reason_code == "ALREADY_AT_GOAL"
    with pytest.raises(ValueError):
        planner.plan(world(), (float("nan"), 0), (1, 0), 1)


def test_sparse_storage_does_not_scale_with_centimeter_area():
    api = load_api()
    planner = api.AdaptiveGlobalPlanner(WheelLimits())
    source = world([CellRect(49999, -100, 50001, 100)])
    tracemalloc.start()
    result = planner.plan(source, (0, 0), (1000, 0), 1, budget_s=5)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    assert result.has_reference
    assert result.storage["nodes"] < 100
    assert peak < 4_000_000


def test_map_epoch_update_rebuilds_and_rejects_new_wall():
    api = load_api()
    planner = api.AdaptiveGlobalPlanner(WheelLimits())
    first = planner.plan(world(), (0, 0), (4, 0), 4)
    second = planner.plan(world([CellRect(399, -10, 401, 10)], epoch=2), (0, 0), (4, 0), 5)
    assert first.has_reference
    assert second.reason_code == "GOAL_IN_OBSTACLE" and not second.has_reference


def test_storage_and_explicit_domain_limits_do_not_claim_no_path():
    api = load_api()
    planner = api.AdaptiveGlobalPlanner(WheelLimits(), max_nodes=2)
    assert planner.plan(world([CellRect(140, -100, 160, 100)]), (0, 0), (3, 0), 1).reason_code == "STORAGE_LIMIT"
    bounded = api.AdaptiveGlobalPlanner(WheelLimits(), domain_bounds_m=(-1, -1, 1, 1))
    assert bounded.plan(world(), (0, 0), (3, 0), 1).reason_code == "DOMAIN_EXHAUSTED"


def test_independent_benchmark_geometry_rejects_thin_wall_crossing():
    name = "scripts.navigation.one_cm.benchmark_global"
    assert importlib.util.find_spec(name) is not None, "global benchmark is not implemented"
    benchmark = importlib.import_module(name)
    assert benchmark.segment_hits_box((0, 0), (4, 0), (2, -1, 2.01, 1))
    assert not benchmark.segment_hits_box((0, 2), (4, 2), (2, -1, 2.01, 1))
    assert benchmark.segment_hits_box((0, 1), (4, 1), (2, -1, 2.01, 1))
    for scale, count in [(100, 100), (500, 500), (1000, 1000)]:
        source, start, goal = benchmark.make_scale(scale, count, 1)
        assert len(source.blocked) == count
        assert any(benchmark.segment_hits_box(start, goal, (r.x0/100, r.y0/100, r.x1/100, r.y1/100))
                   for r in source.blocked)


@pytest.mark.parametrize("case_name", [
    "serpentine_100m", "serpentine_500m", "serpentine_1000m",
    "branches_deadends", "multi_loop_bottlenecks", "loop_open", "loop_one_cm_blocked",
    "enclosed_gate_82cm", "enclosed_gate_81cm",
])
def test_enclosed_topology_routes_obey_mandatory_gates_and_map_updates(case_name):
    name = "scripts.navigation.one_cm.nav1cm.global_fixtures"
    assert importlib.util.find_spec(name) is not None, "topology fixtures are not implemented"
    fixtures = importlib.import_module(name)
    benchmark = importlib.import_module("scripts.navigation.one_cm.benchmark_global")
    case = next(c for c in fixtures.topology_scenarios(31) if c.name == case_name)
    result = load_api().AdaptiveGlobalPlanner(WheelLimits()).plan(case.world, case.start, case.goal, 6, budget_s=5)
    assert result.reason_code == case.expected_reason
    evidence = benchmark.validate_scenario(case, result, expected_version=6)
    assert evidence["valid"]
    if result.has_reference:
        assert evidence["inside_enclosure"]
        assert evidence["known_route_valid"]
        assert all(evidence["mandatory_gates"].values())
        assert evidence["length_m"] >= case.lower_bound_m - 1e-8
        assert evidence["length_ratio_to_known_route"] <= 1.5
    if case_name == "loop_one_cm_blocked":
        assert evidence["mandatory_gates"]["bottom_alternative"]


@pytest.mark.parametrize("outcome", [0, 1])
@pytest.mark.parametrize("reason", ["PLAN_FOUND", "BUDGET_EXHAUSTED"])
@pytest.mark.parametrize("has_reference", [True, False])
@pytest.mark.parametrize("keep_route", [True, False])
def test_benchmark_rejects_inconsistent_result_status_and_route(outcome, reason, has_reference, keep_route):
    benchmark = importlib.import_module("scripts.navigation.one_cm.benchmark_global")
    fixtures = importlib.import_module("scripts.navigation.one_cm.nav1cm.global_fixtures")
    case = fixtures.centimeter_gate(1, 82)
    source = case.world
    actual = load_api().AdaptiveGlobalPlanner(WheelLimits()).plan(source, case.start, case.goal, 7)
    altered = replace(actual, planning_outcome=outcome, reason_code=reason,
                      has_reference=has_reference, route=actual.route if keep_route else None)
    want_success = outcome == 0 and reason == "PLAN_FOUND" and has_reference and keep_route
    want_failure = outcome == 1 and reason == "BUDGET_EXHAUSTED" and not has_reference and not keep_route
    evidence = benchmark.validate(source, case.start, case.goal, altered, expected_version=7)
    assert evidence["valid"] == (want_success or want_failure)
    assert benchmark.validate_scenario(case, altered, expected_version=7)["valid"] == (want_success or want_failure)
    record = {"wall_s": .01, "budget_s": 1., "planning_outcome": outcome, "reason_code": reason,
              "has_reference": has_reference, "path": None if altered.route is None else altered.route.points.tolist(),
              "validation": evidence, "rss_after": {}, "storage": altered.storage}
    assert benchmark.summarize([record])["successes"] == int(want_success)


@pytest.mark.parametrize("changed", ["version", "goal", "map_epoch"])
def test_benchmark_rejects_route_identity_mismatch(changed):
    benchmark = importlib.import_module("scripts.navigation.one_cm.benchmark_global")
    source = world()
    actual = load_api().AdaptiveGlobalPlanner(WheelLimits()).plan(source, (0., 0.), (4., 0.), 7)
    changes = {"version": 8, "goal": (9., 0.), "map_epoch": 99}
    altered = replace(actual, route=replace(actual.route, **{changed: changes[changed]}))
    evidence = benchmark.validate(source, (0., 0.), (4., 0.), altered, expected_version=7)
    assert not evidence["valid"]
    record = {"wall_s": .01, "budget_s": 1., "planning_outcome": altered.planning_outcome,
              "reason_code": altered.reason_code, "has_reference": altered.has_reference,
              "path": altered.route.points.tolist(), "validation": evidence,
              "rss_after": {}, "storage": altered.storage}
    assert benchmark.summarize([record])["successes"] == 0

def test_soft_edge_costs_are_only_evaluated_on_search_demand():
    api = load_api()
    source = world([CellRect(200, -100, 210, 100)])
    result = api.AdaptiveGlobalPlanner(WheelLimits(), soft_clearance_m=.5).plan(
        source, (0, 0), (5, 0), 1, budget_s=10.)
    assert result.has_reference
    assert 0 < result.storage['edge_cost_evaluations'] < result.storage['edges']


def test_lazy_cost_keeps_connection_direction_when_reverse_is_expanded_first(monkeypatch):
    import sys
    api = load_api()
    original_cost = api.path_cost
    directions = {}
    observed = []

    def profile(frame, event, arg):
        if event == 'return' and frame.f_code.co_name == 'connect':
            values = frame.f_locals
            points = (values['nodes'][values['i']], values['nodes'][values['j']])
            directions[frozenset(points)] = points

    def recording_cost(points, *args, **kwargs):
        plan_frame = sys._getframe(1).f_back
        if 'search_began' in plan_frame.f_locals and 'shortcut_began' not in plan_frame.f_locals:
            expected = directions[frozenset(points)]
            observed.append(plan_frame.f_locals['nodes'][plan_frame.f_locals['current']] != expected[0])
            assert tuple(points) == expected
        return original_cost(points, *args, **kwargs)

    monkeypatch.setattr(api, 'path_cost', recording_cost)
    previous = sys.getprofile()
    sys.setprofile(profile)
    try:
        result = api.AdaptiveGlobalPlanner(WheelLimits(), soft_clearance_m=.5).plan(
            world([CellRect(200, -100, 210, 100)]), (5, 0), (0, 0), 1, budget_s=10.)
    finally:
        sys.setprofile(previous)
    assert result.has_reference
    assert any(observed), '必须覆盖反向首先展开的连接'


def test_zero_soft_band_does_not_integrate_edges(monkeypatch):
    api = load_api()
    def forbidden(*args, **kwargs):
        raise AssertionError('零软带不能积分')
    monkeypatch.setattr(api, 'path_cost', forbidden)
    result = api.AdaptiveGlobalPlanner(WheelLimits(), soft_clearance_m=0).plan(
        world([CellRect(200, -100, 210, 100)]), (0, 0), (5, 0), 1, budget_s=10.)
    assert result.has_reference
    assert result.storage['edge_cost_evaluations'] == 0


def test_lazy_edge_integration_obeys_budget(monkeypatch):
    api = load_api()
    clock = [0.]
    monkeypatch.setattr(api.time, 'perf_counter', lambda: clock[0])
    def exhausted(points, boxes, soft, weight, check, **kwargs):
        clock[0] = 2.
        check()
        raise AssertionError('预算耗尽后不得继续积分')
    monkeypatch.setattr(api, 'path_cost', exhausted)
    result = api.AdaptiveGlobalPlanner(WheelLimits(), soft_clearance_m=.5).plan(
        world([CellRect(200, -100, 210, 100)]), (0, 0), (5, 0), 1, budget_s=1.)
    assert result.reason_code == 'BUDGET_EXHAUSTED'
    assert result.route is None and not result.has_reference
    assert result.storage['edges'] > 0


def test_scalar_segment_box_overlap_matches_vector_reference():
    api = load_api()
    rng = np.random.default_rng(71)
    for _ in range(600):
        low = rng.uniform(-3, 3, 2)
        box = np.r_[low, low+rng.uniform(.01, 2, 2)]
        a, b = rng.uniform(-5, 5, (2, 2))
        assert api._segment_hits_box(a, b, box) == (not api._segment_clear(a, b, box[None, :]))
    box = (0., 0., 1., 1.)
    for a, b in [((-1., 0.), (2., 0.)), ((0., 0.), (0., 0.)),
                 ((-1., -1.), (0., 0.)), ((.5, -1.), (.5, 2.))]:
        assert api._segment_hits_box(a, b, box)
    assert not api._segment_hits_box((-1., -.01), (2., -.01), box)
