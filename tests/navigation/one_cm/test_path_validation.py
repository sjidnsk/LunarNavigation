"""验证层独立动力学与 SAT 的解析校准，不调用生产认证或 plant。"""
from importlib import import_module, util
import math

import pytest


def api():
    name = 'scripts.navigation.one_cm.p3_path_validation.motion'
    assert util.find_spec(name) is not None, '独立 Path 运动验证尚未实现'
    return import_module(name)


def test_reverse_path_produces_negative_endpoint_velocity_and_real_displacement():
    m = api()
    follower = m.PathFollower()
    state = m.State()
    follower.offer([(0., 0., 0.), (-.6, 0., 0.)], 0.)
    rows = []
    for i in range(400):
        follower.offer([(state.x, 0., 0.), (-.6, 0., 0.)], i * .02)
        previous = state
        state, a, alpha, mode = follower.step(state, i * .02, .02)
        rows.append(state)
        assert abs(state.v - previous.v) <= .5 * .02 + 1e-12
        assert abs(state.v) <= .2 + 1e-12
    assert min(s.v for s in rows) < -.1
    assert state.x < -.5
    assert abs(state.v) < .01


def test_spin_preserves_xy_and_stops_with_bounded_endpoint_angular_velocity():
    m = api()
    follower, state = m.PathFollower(), m.State()
    for i in range(300):
        follower.offer([(0., 0., state.yaw), (0., 0., .6)], i * .02)
        previous = state
        state, a, alpha, mode = follower.step(state, i * .02, .02)
        assert abs(state.w - previous.w) <= .5 * .02 + 1e-12
        assert abs(state.w) <= 1. + 1e-12
    assert abs(m.wrap(state.yaw - .6)) <= .05
    assert math.hypot(state.x, state.y) == 0.
    assert abs(state.w) < .01


@pytest.mark.parametrize('empty', [True, False])
def test_empty_or_stale_path_brakes_continuously_without_velocity_reset(empty):
    m = api()
    follower, state = m.PathFollower(), m.State(v=.2)
    follower.offer([] if empty else [(0., 0., 0.), (1., 0., 0.)], 0.)
    old = state
    for i in range(30):
        state, a, alpha, mode = follower.step(state, 1. + i * .02, .02)
        assert abs(state.v - old.v) <= .5 * .02 + 1e-12
        old = state
    assert state.x == pytest.approx(.04, abs=1e-9)
    assert state.v == pytest.approx(0., abs=1e-12)


def test_midpoint_integrator_uses_endpoint_velocity_not_average_as_final_state():
    m = api()
    s = m.integrate(m.State(v=.1), .5, 0., .02)
    assert s.v == pytest.approx(.11)
    assert s.x == pytest.approx(.0021)


def test_sat_sweep_catches_corner_collision_with_clear_endpoints():
    m = api()
    rect = (.412, .412, .417, .417)
    start, end = m.State(), m.State(yaw=math.pi / 2)
    assert m.pose_clear(start, [rect])
    assert m.pose_clear(end, [rect])
    assert not m.swept_clear(start, end, [rect])


def test_sat_clear_sweep_and_body_overlap():
    m = api()
    assert m.swept_clear(m.State(), m.State(x=.2), [(2., 2., 3., 3.)])
    assert not m.pose_clear(m.State(), [(.5, -.01, .7, .01)])


def test_translate_waits_until_previous_spin_has_stopped():
    m = api()
    follower, state = m.PathFollower(), m.State(w=.2)
    follower.offer([(0., 0., 0.), (.6, 0., 0.)], 0.)
    next_state, _, _, _ = follower.step(state, 0., .02)
    assert next_state.v == 0.
    assert next_state.w == pytest.approx(.19)


def test_path_contract_rejects_nonmonotonic_times_and_nonunit_quaternion():
    name = 'scripts.navigation.one_cm.p3_path_validation.evaluate'
    assert util.find_spec(name) is not None, 'DDS 证据判定尚未实现'
    e = import_module(name)
    path = dict(frame='map', stamp_ns=100, poses=[dict(x=0., y=0., z=0., q=[0., 0., 0., 1.], stamp_ns=100, frame='map'),
                                               dict(x=-.1, y=0., z=0., q=[0., 0., 0., 1.], stamp_ns=200, frame='map')])
    assert e.path_valid(path, local=True)
    path['poses'][-1]['stamp_ns'] = 100
    assert not e.path_valid(path, local=True)
    path['poses'][-1]['stamp_ns'] = 200
    path['poses'][-1]['q'][-1] = 2.
    assert not e.path_valid(path, local=True)


def test_formal_success_requires_current_goal_generation_and_accepted_route():
    from scripts.navigation.one_cm.p3_path_validation import evaluate as e
    tick = dict(goal=[.6, 0.], generation=2, goal_revision=3,
                global_result=dict(planning_outcome=0, reason_code='PLAN_FOUND', has_reference=True,
                                   requested_goal=[.6, 0.], requested_epoch=8, requested_version=4),
                events=[dict(kind='GLOBAL_RESULT_RECEIVED', accepted_generation_at_submit=2, goal_revision_at_submit=3),
                        dict(kind='ROUTE_ACCEPTED', version=4, map_epoch=8, points=[[0., 0.], [.6, 0.]])])
    assert e.formal_acceptance(tick)
    tick['generation'] = 3
    assert not e.formal_acceptance(tick)
    tick['generation'] = 2
    tick['global_result']['requested_goal'] = [-.6, 0.]
    assert not e.formal_acceptance(tick)
    tick['global_result']['requested_goal'] = [.6, 0.]
    tick['events'][-1]['kind'] = 'ROUTE_CONTINUITY_RETAINED'
    assert not e.formal_acceptance(tick)


def test_enclosed_failure_uses_observed_domain_exhausted_not_invented_reason():
    from scripts.navigation.one_cm.p3_path_validation import evaluate as e
    raw = dict(planning_outcome=1, reason_code='DOMAIN_EXHAUSTED', has_reference=False, requested_goal=[0., 0.])
    assert e.enclosed_failure(raw)
    for key, value in [('planning_outcome', 0), ('reason_code', 'BUDGET_EXHAUSTED'), ('has_reference', True), ('requested_goal', [1., 0.])]:
        assert not e.enclosed_failure(dict(raw, **{key:value}))


def test_dds_graph_requires_actual_standard_types_and_expected_publishers():
    from scripts.navigation.one_cm.p3_path_validation import evaluate as e
    expected = {
        '/Car/T3/mapping/grid_map': ('grid_map_msgs/msg/GridMap','nav1cm_bounded_fixture'),
        '/Car/T3/localization/odometry': ('nav_msgs/msg/Odometry','nav1cm_bounded_path_consumer'),
        '/Car/T4/navigation/goal': ('geometry_msgs/msg/PoseStamped','nav1cm_bounded_fixture'),
        '/Car/T4/planning/global_route': ('nav_msgs/msg/Path','lunar_nav1cm_planner'),
        '/Car/T4/planning/local_path': ('nav_msgs/msg/Path','lunar_nav1cm_planner'),
        '/tf_static': ('tf2_msgs/msg/TFMessage','nav1cm_bounded_fixture'),
    }
    graph = dict(types={k:[v[0]] for k,v in expected.items()},
                 publishers={k:[dict(node_name=v[1], topic_type=v[0])] for k,v in expected.items()})
    assert e.graph_valid(graph, 'contract')
    graph['types']['/Car/T4/planning/local_path'] = ['std_msgs/msg/String']
    assert not e.graph_valid(graph, 'contract')
