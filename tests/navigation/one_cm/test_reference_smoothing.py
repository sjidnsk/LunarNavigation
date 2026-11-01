"""全局参考整形的纯几何安全与有界回退测试。"""
import importlib

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import Route
from scripts.navigation.one_cm.nav1cm.global_planner import _segment_clear


def api():
    name = "scripts.navigation.one_cm.nav1cm.reference_smoothing"
    assert importlib.util.find_spec(name) is not None, "reference smoothing is not implemented"
    return importlib.import_module(name)


def route(points=((0, 0), (2, 0), (2, 2))):
    return Route(17, 23, np.asarray(points), (2, 2))


def shape(raw=None, boxes=(), **kwargs):
    return api().shape_route(raw or route(), np.asarray(boxes, dtype=float).reshape(-1, 4),
                             budget_s=10, **kwargs)


def test_open_corner_preserves_identity_endpoints_and_immutable_arrays():
    raw = route()
    result = shape(raw)
    assert result.changed and len(result.proof_boxes)
    assert result.route.version == 17 and result.route.map_epoch == 23
    assert result.route.goal == raw.goal
    np.testing.assert_array_equal(result.route.points[[0, -1]], raw.points[[0, -1]])
    np.testing.assert_array_equal(raw.points, ((0, 0), (2, 0), (2, 2)))
    assert np.all(np.linalg.norm(np.diff(result.route.points, axis=0), axis=1) > 0)
    curved = result.route.points[1:-1]
    assert np.max(np.linalg.norm(np.diff(curved, axis=0), axis=1)) <= .05 + 1e-12
    for array in (result.route.points, result.proof_boxes):
        with pytest.raises(ValueError):
            array.setflags(write=True)


def test_corner_is_tangent_and_dense_exact_curve_is_covered_by_proof():
    result = shape()
    points = result.route.points
    curved = points[(points[:, 0] > 1.5) & (points[:, 1] < .5)]
    assert curved[0, 0] > 1.5 and curved[0, 1] < .002
    assert curved[-1, 0] > 1.998 and curved[-1, 1] < .5
    t = np.linspace(0, 1, 10001)[:, None]
    dense = (1-t)**2 * (1.5, 0) + 2*t*(1-t) * (2, 0) + t**2 * (2, .5)
    boxes = result.proof_boxes
    covered = ((dense[:, None, :] >= boxes[None, :, :2]) &
               (dense[:, None, :] <= boxes[None, :, 2:])).all(axis=2).any(axis=1)
    assert covered.all()


def test_cutting_obstacle_halves_trim_and_all_output_edges_are_clear():
    blocked = np.asarray([[1.84, .14, 1.90, .20]])
    result = shape(boxes=blocked)
    assert result.changed
    assert api().reference_clear(result, blocked)
    assert all(_segment_clear(a, b, blocked) for a, b in zip(result.route.points[:-1], result.route.points[1:]))
    # 原始最大裁切曲线在 t=.5 为 (1.875,.125)，t=.6 时进入障碍。
    assert np.min(result.proof_boxes[:, 0]) > 1.5


def test_new_obstacle_between_curve_and_sampled_chord_rejects_reference():
    result = shape(step_m=.3)
    # 二次曲线 t=.37 的精确位置；此位置不在任何采样折线上。
    point = np.asarray((1.5 + .37 - .5*.37**2, .5*.37**2))
    blocked = np.asarray([np.r_[point - 1e-7, point + 1e-7]])
    assert all(_segment_clear(a, b, blocked) for a, b in zip(result.route.points[:-1], result.route.points[1:]))
    assert not api().reference_clear(result, blocked)


@pytest.mark.parametrize("points", [((0, 0), (1, 0)), ((0, 0), (1, 0), (2, 0)),
                                    ((0, 0), (1, 0), (0, 0)),
                                    ((0, 0), (1e-13, 0), (1e-13, 1e-13))])
def test_straight_reverse_and_extremely_short_routes_are_unchanged(points):
    raw = route(points)
    result = shape(raw)
    assert not result.changed and result.route is raw and result.proof_boxes.shape == (0, 4)


def test_budget_and_point_limit_roll_back_whole_result():
    raw = route(((0, 0), (2, 0), (2, 2), (4, 2)))
    for kwargs in ({"budget_s": 0}, {"max_points": 4}):
        result = api().shape_route(raw, np.empty((0, 4)), **kwargs)
        assert not result.changed and result.route is raw and not len(result.proof_boxes)
        assert any(word in result.reason.upper() for word in ("BUDGET", "POINT"))
    ticks = iter([0., 0., 0., 100.])
    result = api().shape_route(raw, np.empty((0, 4)), clock=lambda: next(ticks, 100.))
    assert not result.changed and result.route is raw and "BUDGET" in result.reason.upper()


def test_raw_collision_and_touch_are_rejected_even_with_no_budget():
    for box in ((.5, -.1, .6, .1), (1, 0, 1.1, .1)):
        with pytest.raises(ValueError, match="(?i)raw|collision"):
            api().shape_route(route(), np.asarray([box]), budget_s=0)


@pytest.mark.parametrize("kwargs", [{"step_m": 0}, {"step_m": np.nan}, {"max_trim_m": -1},
                                    {"budget_s": np.inf}, {"max_points": 1}, {"max_points": 3.5}])
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        api().shape_route(route(), np.empty((0, 4)), **kwargs)


@pytest.mark.parametrize("boxes", [[[0, np.nan, 1, 1]], [[1, 0, 0, 1]], [0, 1, 2]])
def test_invalid_obstacle_boxes_fail_closed(boxes):
    with pytest.raises(ValueError):
        api().shape_route(route(), np.asarray(boxes))
    good = shape()
    assert not api().reference_clear(good, np.asarray(boxes))


def test_new_map_touching_curve_proof_or_straight_connector_is_rejected():
    result = shape()
    assert api().reference_clear(result, np.empty((0, 4)))
    box = result.proof_boxes[0]
    touching = np.asarray([[box[0] - .001, box[1] - .001, box[0], box[1]]])
    assert not api().reference_clear(result, touching)
    assert not api().reference_clear(result, np.asarray([[.25, 0, .26, .01]]))


def test_subdivision_exhaustion_returns_original_without_partial_evidence():
    raw = route()
    result = shape(raw, step_m=1e-8)
    assert result.route is raw and not result.changed and not len(result.proof_boxes)
    assert "SUBDIVISION" in result.reason.upper()


def test_obstacle_preventing_every_retry_preserves_original_corner():
    raw = route()
    blocked = np.asarray([[1., 1e-10, 1.9999999, 1.]])
    result = shape(raw, boxes=blocked)
    assert result.route is raw and not result.changed and not len(result.proof_boxes)
    assert api().reference_clear(result, blocked)


def test_dense_shrunk_curve_oracle_stays_outside_obstacle():
    blocked = np.asarray([[1.84, .14, 1.90, .20]])
    result = shape(boxes=blocked)
    assert result.changed
    t = np.linspace(0, 1, 20001)[:, None]
    # 最大裁切0.5失败；第一次减半0.25的独立解析曲线。
    dense = (1-t)**2 * (1.75, 0) + 2*t*(1-t) * (2, 0) + t**2 * (2, .25)
    inside = ((dense[:, None, :] >= blocked[None, :, :2]) &
              (dense[:, None, :] <= blocked[None, :, 2:])).all(axis=2)
    assert not inside.any()
    boxes = result.proof_boxes
    assert ((dense[:, None, :] >= boxes[None, :, :2]) &
            (dense[:, None, :] <= boxes[None, :, 2:])).all(axis=2).any(axis=1).all()


def test_late_budget_exhaustion_discards_all_previously_accepted_corners():
    raw = route(((0, 0), (2, 0), (2, 2), (4, 2), (4, 4)))
    calls = 0

    def ticking_clock():
        nonlocal calls
        calls += 1
        return 0. if calls < 50 else 1.

    result = api().shape_route(raw, np.empty((0, 4)), clock=ticking_clock)
    assert calls >= 50
    assert result.route is raw and not result.changed and not len(result.proof_boxes)
    assert "BUDGET" in result.reason.upper()


def test_short_adjacent_edges_and_large_trim_keep_curve_inside_quarter_segments():
    raw = route(((0, 0), (.04, 0), (.04, .04)))
    result = shape(raw, max_trim_m=100)
    assert result.changed
    assert np.min(result.proof_boxes[:, 0]) >= .03 - 1e-12
    assert np.max(result.proof_boxes[:, 3]) <= .01 + 1e-12


def test_multiple_turns_have_strictly_positive_edges_and_preserve_goal():
    raw = route(((0, 0), (2, 0), (2, 2), (4, 2), (4, 4)))
    result = shape(raw)
    assert result.changed and api().reference_clear(result, np.empty((0, 4)))
    assert (result.route.version, result.route.map_epoch, result.route.goal) == (17, 23, (2., 2.))
    np.testing.assert_array_equal(result.route.points[[0, -1]], ((0, 0), (4, 4)))
    assert np.all(np.linalg.norm(np.diff(result.route.points, axis=0), axis=1) > 0)


def test_unrepresentable_trim_at_large_origin_keeps_original_route():
    raw = route(((1e16, 0), (1e16 + 4, 0), (1e16 + 4, 4)))
    result = shape(raw)
    assert result.route is raw and not result.changed and not len(result.proof_boxes)


def test_long_straight_connectors_do_not_consume_curve_point_budget():
    raw = route(((0, 0), (750, 0), (750, 2)))
    result = shape(raw)
    assert result.changed
    assert len(result.route.points) < 100
    np.testing.assert_array_equal(result.route.points[[0, -1]], raw.points[[0, -1]])
    np.testing.assert_array_equal(result.route.points[1], (749.5, 0))
    np.testing.assert_array_equal(result.route.points[-2], (750, .5))
    curved = result.route.points[1:-1]
    assert np.max(np.linalg.norm(np.diff(curved, axis=0), axis=1)) <= .05 + 1e-12
    assert api().reference_clear(result, np.empty((0, 4)))


def test_unverified_raw_with_obstacles_cannot_fall_back_when_budget_is_zero():
    blocked = np.asarray([[20., 20., 21., 21.]])
    with pytest.raises(ValueError, match='raw validation budget exhausted'):
        api().shape_route(route(), blocked, budget_s=0)


def test_already_verified_raw_has_constant_work_zero_budget_fallback(monkeypatch):
    raw = route()
    blocked = np.tile((20., 20., 21., 21.), (20000, 1))

    def forbidden_collision_scan(*args):
        pytest.fail('zero-budget trusted fallback must not scan obstacle geometry')

    monkeypatch.setattr(api(), '_segments_clear', forbidden_collision_scan)
    monkeypatch.setattr(api(), '_overlap_mask', forbidden_collision_scan)
    result = api().shape_route(raw, blocked, budget_s=0, raw_verified=True)
    assert result.route is raw and not result.changed and not len(result.proof_boxes)
    assert 'BUDGET' in result.reason


def test_unverified_raw_stops_collision_work_at_deadline(monkeypatch):
    raw = route(np.column_stack((np.arange(1000), np.zeros(1000))))
    blocked = np.tile((2000., 20., 2001., 21.), (20000, 1))
    now = 0.
    calls = 0
    original = api()._segments_clear

    def timed_segment(a, b, boxes, **kwargs):
        nonlocal now, calls
        assert len(boxes) <= 1024
        clear = original(a, b, boxes, **kwargs)
        calls += 1
        now += .01
        return clear

    monkeypatch.setattr(api(), '_segments_clear', timed_segment)
    with pytest.raises(ValueError, match='raw validation budget exhausted'):
        api().shape_route(raw, blocked, budget_s=.025, clock=lambda: now)
    assert calls == 3


def test_curve_and_final_revalidation_bound_obstacle_batches(monkeypatch):
    blocked = np.tile((20., 20., 21., 21.), (2050, 1))
    segment = api()._segments_clear
    overlaps = api()._overlap_mask
    segment_sizes, overlap_sizes = [], []

    def bounded_segment(a, b, boxes, **kwargs):
        segment_sizes.append(len(boxes))
        assert 0 < len(a) <= 64
        assert 0 < len(boxes) <= 1024
        return segment(a, b, boxes, **kwargs)

    def bounded_overlap(box, boxes, **kwargs):
        overlap_sizes.append(len(boxes))
        assert 0 < len(box) <= 64
        assert 0 < len(boxes) <= 1024
        return overlaps(box, boxes, **kwargs)

    monkeypatch.setattr(api(), '_segments_clear', bounded_segment)
    monkeypatch.setattr(api(), '_overlap_mask', bounded_overlap)
    result = shape(boxes=blocked, raw_verified=True)
    assert result.changed
    assert 1024 in segment_sizes and 2 in segment_sizes
    assert 1024 in overlap_sizes and 2 in overlap_sizes


def test_final_connector_revalidation_expiry_discards_candidate_immediately(monkeypatch):
    raw = route()
    blocked = np.asarray([[20., 20., 21., 21.]])
    original = api()._segments_clear
    expired = False

    def expire_on_final_connector(a, b, boxes, **kwargs):
        nonlocal expired
        assert not expired, 'geometry work continued after the final-check deadline'
        clear = original(a, b, boxes, **kwargs)
        if np.any(np.all(a == (0, 0), axis=1) & np.all(b == (1.5, 0), axis=1)):
            expired = True
        return clear

    monkeypatch.setattr(api(), '_segments_clear', expire_on_final_connector)
    result = api().shape_route(raw, blocked, raw_verified=True, clock=lambda: 1. if expired else 0.)
    assert expired
    assert result.route is raw and not result.changed and not len(result.proof_boxes)
    assert 'BUDGET' in result.reason


def test_trusted_raw_still_checks_curve_evidence_and_sampled_edges():
    blocked = np.asarray([[1.84, .14, 1.90, .20]])
    result = shape(boxes=blocked, raw_verified=True)
    assert result.changed and api().reference_clear(result, blocked)
    assert np.min(result.proof_boxes[:, 0]) > 1.5


def test_final_curve_proof_revalidation_checks_deadline_after_each_batch(monkeypatch):
    raw = route()
    blocked = np.asarray([[20., 20., 21., 21.]])
    original = api()._overlap_mask
    first_leaf = None
    expired = False

    def expire_on_repeated_leaf(box, boxes, **kwargs):
        nonlocal first_leaf, expired
        assert not expired, 'curve-proof work continued after the final-check deadline'
        collision = original(box, boxes, **kwargs)
        for leaf in box:
            if first_leaf is None and np.max(leaf[2:] - leaf[:2]) < .04:
                first_leaf = leaf.copy()
            elif first_leaf is not None and np.array_equal(leaf, first_leaf):
                expired = True
        return collision

    monkeypatch.setattr(api(), '_overlap_mask', expire_on_repeated_leaf)
    result = api().shape_route(raw, blocked, raw_verified=True, clock=lambda: 1. if expired else 0.)
    assert expired
    assert result.route is raw and not result.changed and not len(result.proof_boxes)
    assert 'BUDGET' in result.reason


def test_empty_obstacles_need_no_raw_edge_scan_for_zero_budget(monkeypatch):
    raw = route(np.column_stack((np.arange(1000), np.zeros(1000))))

    def forbidden_collision_scan(*args):
        pytest.fail('empty obstacles should not require a raw edge scan')

    monkeypatch.setattr(api(), '_segments_clear', forbidden_collision_scan)
    result = api().shape_route(raw, np.empty((0, 4)), budget_s=0)
    assert result.route is raw and not result.changed and 'BUDGET' in result.reason


# P3真实地图快照的四点规划路线与19个已膨胀障碍，仅保留纯几何回归数据。
REAL_FOUR_POINT_ROUTE = (
    (-4.842089032379527, 0.7480244785279971),
    (-8.881277422266072, -5.31872257773393),
    (-11.31872257773393, -5.31872257773393),
    (-14.40797233581543, -1.9836714267730713),
)
REAL_INFLATED_BOXES = (
    (-11.31872247773393, -4.31872247773393, -8.881277522266071, -2.2812775222660697),
    (-11.31872247773393, -4.118722477733931, -8.48127752226607, -2.0812775222660695),
    (-11.31872247773393, -3.9187224777339305, -8.28127752226607, -1.8812775222660696),
    (-11.118722477733929, -3.7187224777339303, -8.08127752226607, -1.6812775222660699),
    (-11.118722477733929, -3.51872247773393, -7.88127752226607, -1.28127752226607),
    (-11.118722477733929, -3.1187224777339306, -7.681277522266069, -1.0812775222660698),
    (-10.91872247773393, -2.9187224777339305, -7.681277522266069, -0.8812775222660698),
    (-11.118722477733929, -2.7187224777339303, -7.681277522266069, -0.6812775222660699),
    (-10.91872247773393, -2.51872247773393, -7.681277522266069, -0.2812775222660697),
    (-11.118722477733929, -2.11872247773393, -7.681277522266069, -0.08127752226606977),
    (-11.51872247773393, -1.9187224777339302, -7.88127752226607, 0.11872247773393019),
    (-11.71872247773393, -1.7187224777339303, -7.88127752226607, 0.5187224777339302),
    (-11.51872247773393, -1.3187224777339304, -7.88127752226607, 1.1187224777339302),
    (-11.71872247773393, -0.7187224777339303, -7.88127752226607, 1.3187224777339304),
    (-11.51872247773393, -0.5187224777339302, -7.88127752226607, 1.51872247773393),
    (-11.31872247773393, -0.31872247773393025, -7.88127752226607, 1.9187224777339302),
    (-11.71872247773393, 0.08127752226606977, -7.88127752226607, 2.11872247773393),
    (-11.91872247773393, 0.2812775222660697, -8.08127752226607, 2.31872247773393),
    (-11.91872247773393, 0.4812775222660697, -8.881277522266071, 2.51872247773393),
)

def test_real_four_point_route_fits_bounded_geometry_callback_budget():
    raw = route(REAL_FOUR_POINT_ROUTE)
    blocked = np.asarray(REAL_INFLATED_BOXES)
    calls = 0

    def work_clock():
        # 目标机已观察到重复小操作/时钟检查成本；此处固定工作配额，避免墙钟抖动。
        nonlocal calls
        calls += 1
        return calls * .0001

    result = api().shape_route(raw, blocked, raw_verified=True, clock=work_clock)
    assert result.changed, (result.reason, calls)
    assert len(result.route.points) == 68 and len(result.proof_boxes) == 64
    assert calls < 250
    assert api().reference_clear(result, blocked)


def recursive_oracle(a, control, b, step=.05):
    controls = np.asarray((a, control, b))
    pad = 32 * np.finfo(float).eps * max(1., float(np.max(np.abs(controls))))
    box = np.r_[np.min(controls, axis=0) - pad, np.max(controls, axis=0) + pad]
    if np.linalg.norm(control-a) + np.linalg.norm(b-control) <= step:
        return [b], [box]
    left, right = (a+control)/2, (control+b)/2
    middle = (left+right)/2
    first = recursive_oracle(a, left, middle, step)
    second = recursive_oracle(middle, right, b, step)
    return first[0] + second[0], first[1] + second[1]


@pytest.mark.parametrize('points', [((0, 0), (2, 0), (2, 2)), REAL_FOUR_POINT_ROUTE,
                                    ((0, 0), (7, 2), (9, 8), (12, 4))])
def test_batched_curve_samples_and_boxes_match_recursive_oracle_in_order(points):
    raw = route(points)
    result = shape(raw)
    expected_points, expected_boxes = [raw.points[0]], []
    for previous, corner, following in zip(raw.points[:-2], raw.points[1:-1], raw.points[2:]):
        before, after = np.linalg.norm(corner-previous), np.linalg.norm(following-corner)
        trim = min(.75, .25*before, .25*after)
        a = corner - trim*(corner-previous)/before
        b = corner + trim*(following-corner)/after
        samples, boxes = recursive_oracle(a, corner, b)
        expected_points.extend([a] + samples)
        expected_boxes.extend(boxes)
    expected_points.append(raw.points[-1])
    assert result.changed
    np.testing.assert_allclose(result.route.points, expected_points, atol=1e-14, rtol=0)
    np.testing.assert_allclose(result.proof_boxes, expected_boxes, atol=1e-14, rtol=0)


def test_vectorized_segment_clipping_matches_scalar_closed_interval_oracle():
    rng = np.random.default_rng(3719)
    starts, ends = rng.uniform(-3, 3, (2, 48, 2))
    starts[:6] = ((-2, 0), (0, -2), (1, 1), (2, 1), (0, 0), (2, 2))
    ends[:6] = ((2, 0), (0, 2), (1, 1), (2, 2), (1, 1), (3, 3))
    blocked = np.asarray(((0, 0, 1, 1), (-1.5, -.2, -1., .5)))
    expected = [_segment_clear(a, b, blocked) for a, b in zip(starts, ends)]
    for a, b, clear in zip(starts, ends, expected):
        assert api()._segments_clear(a[None], b[None], blocked) == clear
    assert api()._segments_clear(starts, ends, blocked) == all(expected)
    assert expected[:6] == [False, False, False, True, False, True]


def test_vectorized_proof_overlap_matches_scalar_touch_rejection():
    boxes = np.asarray(((-1, -1, 0, 0), (1, 1, 2, 2), (1.01, 1.01, 2, 2),
                        (-.5, -.5, -.1, -.1), (.2, .2, .8, .8)))
    blocked = np.asarray(((0, 0, 1, 1),))
    np.testing.assert_array_equal(api()._overlap_mask(boxes, blocked),
                                  (True, True, False, False, True))


def test_positive_budget_raw_collision_still_rejects_after_batching():
    with pytest.raises(ValueError, match='raw route collision'):
        shape(boxes=((.5, -.1, .6, .1),))
