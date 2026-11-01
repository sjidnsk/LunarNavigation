"""末段姿态连接仅保证连续几何安全。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route
from scripts.navigation.one_cm.nav1cm.reference_smoothing import shape_route, reference_clear


def raw():
    return Route(1, 2, np.array(((0., 0.), (4., 0.))), (4., 0.))


def shape(yaw, **kw):
    return shape_route(raw(), np.empty((0, 4)), terminal_yaw=yaw, budget_s=10, **kw)


@pytest.mark.parametrize("yaw", [np.pi/2, -np.pi/2, np.pi/3, 4*np.pi/3])
def test_terminal_connection_endpoint_tangency_and_no_loop(yaw):
    result = shape(yaw)
    assert result.changed
    p = result.route.points
    np.testing.assert_array_equal(p[[0,-1]], raw().points)
    assert abs(p[2,1]-p[1,1]) < .002
    tangent = p[-1]-p[-2]
    direction = np.array((np.cos(yaw), np.sin(yaw)))
    assert abs((tangent[0]*direction[1]-tangent[1]*direction[0]) / np.linalg.norm(tangent)) < .06
    assert np.all(np.diff(p[:,0]) >= -1e-12)
    assert reference_clear(result, np.empty((0,4)))


def test_forward_reverse_yaw_equivalent():
    np.testing.assert_allclose(shape(np.pi/3).route.points,
                               shape(4*np.pi/3).route.points, atol=1e-12)


def test_none_keeps_original_and_budget_point_limits_fallback():
    assert not shape(None).changed
    result = shape_route(raw(), np.empty((0,4)), terminal_yaw=1., budget_s=0,
                         raw_verified=True)
    assert not result.changed and result.reason == "BUDGET_EXHAUSTED"
    assert not shape(1., max_points=2).changed


def test_obstacle_triggers_shortening_and_continuous_revalidation():
    blocked = np.array(((2., -.5, 2.2, -.1),))
    result = shape_route(raw(), blocked, terminal_yaw=np.pi/2, budget_s=10)
    assert result.changed
    assert result.route.points[1,0] > 1.
    assert reference_clear(result, blocked)


def test_nonfinite_yaw_rejected():
    with pytest.raises(ValueError):
        shape(float("nan"))


def test_long_straight_tail_only_shapes_last_three_metres():
    route = Route(1, 2, np.array(((0.,0.), (750.,0.))), (750.,0.))
    result = shape_route(route, np.empty((0,4)), terminal_yaw=np.pi/2, budget_s=10)
    assert result.changed
    np.testing.assert_array_equal(result.route.points[1], (747.,0.))


def test_exact_cubic_is_covered_by_convex_hull_proofs():
    result = shape(np.pi/2)
    t = np.linspace(0,1,10001)[:,None]
    # 相对角恰90度时优先显式+90度，末控制点位于goal下方。
    dense = (1-t)**3*(1.,0.) + 3*t*(1-t)**2*(2.,0.) + 3*t*t*(1-t)*(4.,-1.) + t**3*(4.,0.)
    boxes = result.proof_boxes
    assert ((dense[:,None,:] >= boxes[None,:,:2]) &
            (dense[:,None,:] <= boxes[None,:,2:])).all(axis=2).any(axis=1).all()


def test_goal_yaw_replacement_retires_route_and_forwards_current_yaw(monkeypatch):
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from scripts.navigation.one_cm.nav1cm import reference_smoothing
    coordinator = NavigationCoordinator(None, runner=object(), reference_smoothing=True,
                                        revalidate_current_map=True)
    calls = []
    original = reference_smoothing.shape_route
    def capture(*args, **kwargs):
        calls.append(kwargs['terminal_yaw'])
        kwargs['budget_s'] = 10
        return original(*args, **kwargs)
    monkeypatch.setattr(reference_smoothing, 'shape_route', capture)
    coordinator.replace_goal((4.,0.), goal_yaw=.3, generation=1)
    coordinator._adopt_route(raw(), [], raw_verified=True)
    first = coordinator.route
    coordinator.replace_goal((4.,0.), goal_yaw=-.7, generation=2)
    assert coordinator.route is None
    coordinator._adopt_route(raw(), [], raw_verified=True)
    np.testing.assert_allclose(calls, (.3, -.7))
    assert not np.array_equal(first.points, coordinator.route.points)


def test_no_terminal_space_safely_preserves_raw_route():
    # 原尾段下侧紧邻封闭障碍，不能为终点姿态切入障碍。
    blocked = np.array(((0., -2., 4., -1e-12),))
    route = raw()
    result = shape_route(route, blocked, terminal_yaw=np.pi/2, budget_s=10)
    assert not result.changed and result.route is route
    assert reference_clear(result, blocked)


@pytest.mark.parametrize("sign", [-1., 1.])
@pytest.mark.parametrize("delta", [0., -1e-8, 1e-8])
def test_right_angle_tie_prefers_explicit_yaw_and_strictly_shorter_flips(sign, delta):
    yaw = sign * (np.pi/2 + delta)
    result = shape(yaw)
    terminal = result.route.points[-1] - result.route.points[-2]
    expected_sign = sign if delta <= 0 else -sign
    assert np.sign(terminal[1]) == expected_sign
    assert np.all(np.diff(result.route.points[:,0]) >= -1e-12)
