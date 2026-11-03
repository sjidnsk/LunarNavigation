"""平滑以连续碰撞证明为接受条件，不因软代价升高撤销安全曲线。"""
import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import Route
from scripts.navigation.one_cm.nav1cm.global_planner import _path_cost_python
from scripts.navigation.one_cm.nav1cm.reference_smoothing import shape_route, reference_clear


def test_safe_smoothing_accepts_increased_clearance_cost():
    radius = 0.75
    points = np.array([[-3., 0.], [0., 0.], [0., 3.]])
    route = Route(1, 1, points, (0., 3.))
    boxes = np.array([[-0.81, 0.79, -0.79, 0.81]]) + [-radius, -radius, radius, radius]
    shaped = shape_route(route, boxes, budget_s=5., corner_radius=radius,
                         soft_clearance_m=0.5, clearance_weight=4.)
    assert shaped.changed, shaped.reason
    assert shaped.reason == "SMOOTHED"
    assert reference_clear(shaped, boxes, corner_radius=radius)
    assert _path_cost_python(shaped.route.points, boxes, .5, 4., corner_radius=radius) > _path_cost_python(
        points, boxes, .5, 4., corner_radius=radius)
    np.testing.assert_array_equal(shaped.route.points[[0, -1]], points[[0, -1]])


def test_removing_cost_veto_does_not_accept_colliding_raw_route():
    route = Route(1, 1, np.array([[-3., 0.], [0., 0.], [0., 3.]]), (0., 3.))
    with pytest.raises(ValueError, match="raw route collision"):
        shape_route(route, np.array([[-2., -.1, -1., .1]]), budget_s=5.,
                    soft_clearance_m=.5, clearance_weight=4.)


def test_straight_route_needs_no_corner_smoothing():
    route = Route(1, 1, np.array([[0., 0.], [3., 0.]]), (3., 0.))
    shaped = shape_route(route, np.empty((0, 4)), budget_s=5.,
                         soft_clearance_m=.5, clearance_weight=4.)
    assert not shaped.changed
    assert shaped.reason == "NO_ELIGIBLE_CORNER"
    np.testing.assert_array_equal(shaped.route.points, route.points)
