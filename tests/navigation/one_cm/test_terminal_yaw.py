"""真实转向轨迹：最短角、残余速度、连续扫掠及闭环到达。"""
from importlib import import_module
import os
from pathlib import Path

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import (
    CellRect, LocalReference, Route, VehicleState, WheelLimits, WorldSnapshot,
)
from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.certification import certify
from scripts.navigation.one_cm.nav1cm.execution import execute_action
from scripts.navigation.one_cm.nav1cm.maps import extract_window, build_distance_field
from scripts.navigation.one_cm.nav1cm.reference import extract_reference
from tests.navigation.one_cm.test_coordinator import RejectPlanner, Runner
from tests.navigation.one_cm.test_async_execution import ManualLocal


def api():
    return import_module('scripts.navigation.one_cm.nav1cm.pose_goal')


def terminal_scene(state, yaw, blocked=(), limits=None):
    world = WorldSnapshot(1, blocked, (CellRect(-1000, -1000, 1000, 1000),))
    grid = extract_window(world, state, False)
    reference = LocalReference(1, 1, [[state.x, state.y]], [0.], terminal_yaw=yaw)
    return grid, reference, WheelLimits() if limits is None else limits


def test_shortest_cross_pi_spin_keeps_continuous_yaw_and_certifies():
    state = VehicleState(0, 0, np.pi - .04, 0, 0)
    grid, ref, limits = terminal_scene(state, -np.pi + .04)
    tr = api().terminal_candidate(state, grid, ref, limits)
    assert tr.source == 'terminal-yaw:bounded' and tr.mode == 'spin'
    assert np.array_equal(tr.states[0, :5], [state.x, state.y, state.yaw, state.v, state.w])
    assert tr.states[-1, 2] == pytest.approx(np.pi + .04)
    assert np.all(np.diff(tr.states[:, 2]) >= -1e-12)
    assert np.max(np.abs(np.diff(tr.states[:, 2]))) < .1
    assert certify(tr, grid, limits, 1).valid


@pytest.mark.parametrize('v,w', [(.005, .003), (-.005, -.003), (.02, .01)])
def test_residual_translation_brakes_from_actual_feedback_before_spin(v, w):
    state = VehicleState(0, 0, .2, v, w)
    grid, ref, limits = terminal_scene(state, .7)
    tr = api().terminal_candidate(state, grid, ref, limits)
    assert tr.source.startswith('terminal-yaw:bounded') and tr.mode=='spin'
    assert np.array_equal(tr.states[0, :5], [0, 0, .2, v, w])
    zero=int(np.flatnonzero(abs(tr.states[:,3])<=1e-10)[0])
    assert zero>0 and np.all(abs(tr.states[zero:,3])<=1e-10)
    assert tr.states[-1,2]>.3  # 制动后应在同一候选内继续目标转向。
    assert certify(tr, grid, limits, 1).valid


@pytest.mark.parametrize('w', [-.06, .09])
def test_opposing_or_overshooting_angular_motion_brakes_physically(w):
    state = VehicleState(0, 0, 0, 0, w)
    grid, ref, limits = terminal_scene(state, .002)
    tr = api().terminal_candidate(state, grid, ref, limits)
    assert tr.source.startswith('terminal-yaw:bounded')
    assert tr.states[-1,2]==pytest.approx(.002)
    assert tr.states[0, 4] == w and abs(tr.states[-1, 4]) < 1e-12
    assert np.sign(tr.controls[0, 1]) == -np.sign(w)
    assert certify(tr, grid, limits, 1).valid


def test_large_yaw_uses_bounded_rolling_horizon_and_braking_tail():
    state = VehicleState(0, 0, 0, 0, 0)
    grid, ref, limits = terminal_scene(state, 2.)
    tr = api().terminal_candidate(state, grid, ref, limits)
    assert tr.times[-1] <= 4. + 1e-12
    assert .2 < tr.states[-1, 2] < 2.
    assert np.max(abs(tr.states[:, 4])) <= limits.angular_speed_rps + 1e-12
    assert np.max(abs(tr.controls[:, 1])) <= limits.angular_accel_rps2 + 1e-12
    cert = certify(tr, grid, limits, 1)
    assert cert.valid and cert.brake_time_s > 0


def test_terminal_spin_sweep_collision_is_rejected_with_safe_endpoints():
    # 该障碍只触及yaw=0附近的矩形角，两个端点均安全。
    state = VehicleState(0, 0, -.05, 0, 0)
    grid, ref, limits = terminal_scene(state, .05, (CellRect(58, 40, 59, 41),))
    checker = api().check_pose_goal
    world = WorldSnapshot(1, (CellRect(58, 40, 59, 41),), ())
    assert checker(world, (0, 0), -.05, limits).accepted
    assert checker(world, (0, 0), .05, limits).accepted
    tr = api().terminal_candidate(state, grid, ref, limits)
    cert = certify(tr, grid, limits, 1)
    assert not cert.valid and cert.reason.value == 'COLLISION'
    c = NavigationCoordinator(RejectPlanner(), runner=Runner())
    r = c.tick(world, state, (0, 0), 0., goal_yaw=.05)
    assert r.action is None or r.action.source == 'brake'
    assert not any(event['kind'] == 'GOAL_REJECTED' for event in r.events)


@pytest.mark.parametrize('asynchronous', [False, True])
@pytest.mark.parametrize('yaw,start', [(.1, 0.), (1., 0.), (-np.pi + .04, np.pi - .04)])
def test_closed_loop_rotation_certifies_every_segment_and_arrives_stopped(asynchronous, yaw, start):
    world = WorldSnapshot(1, (), (CellRect(-1000, -1000, 1000, 1000),))
    if asynchronous:
        c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=Runner())
    else:
        c = NavigationCoordinator(RejectPlanner(), runner=Runner())
    state = VehicleState(0, 0, start, .005, .003)
    seen_spin = False
    for index in range(350):
        r = c.tick(world, state, (0, 0), index * .05, goal_yaw=yaw)
        assert r.action is not None, r
        assert r.action_certificate.matches(r.action, r.grid, c.limits, r.action.route_version)
        assert r.certificate.matches(r.candidate, r.grid, c.limits, r.candidate.route_version)
        assert np.array_equal(r.action.states[0, :5], list(vars(state).values()))
        seen_spin |= r.action.source == 'terminal-yaw:bounded'
        state = execute_action(r.action, state)
        if r.status == 'ARRIVED':
            break
    else:
        pytest.fail('terminal rotation did not converge')
    # 已在配置容差内无需额外转向；跨越±pi也遵守同一判据。
    assert seen_spin == (abs(api().angle_error(yaw,start)) > c.goal_yaw_tolerance_rad)
    assert np.hypot(state.x, state.y) <= .1
    assert abs(api().angle_error(yaw, state.yaw)) <= c.goal_yaw_tolerance_rad
    assert abs(state.v) <= .01 and abs(state.w) <= .01


def test_mpcc_terminal_reference_uses_body_yaw_without_reverse_pi():
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    state = VehicleState(0, 0, 0, 0, 0)
    grid, _, limits = terminal_scene(state, .2)
    ref, _ = extract_reference(Route(1, 1, [[0, 0], [-.3, 0]], (-.3, 0)),
                               state, grid, terminal_yaw=.2)
    planner = MpccPlanner.__new__(MpccPlanner)
    planner.offsets = np.array([-.394, 0, .394])
    planner.radius = .57
    planner.limits = limits
    states = np.zeros((3, 6))
    states[-1, 5] = ref.s[-1]
    params = planner._parameters(states, np.zeros((2, 3)), ref,
                                 build_distance_field(grid), 0., float(ref.s[-1]), mode="reverse")
    assert params[-1][5] == pytest.approx(.2)
    assert 0. < params[0][5] < .2  # 短末段已开始平滑调整，终点仍为真实车体yaw。


@pytest.mark.skipif(os.environ.get('NAV1CM_REAL_SOLVER') != '1', reason='real acados required')
@pytest.mark.parametrize('goal', [(.45, 0), (-.45, 0)])
def test_real_solver_approach_and_certified_terminal_pose(goal):
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    limits = WheelLimits()
    planner = MpccPlanner(limits, Path(os.environ['NAV1CM_BUILD_DIR']))
    world = WorldSnapshot(1, (), (CellRect(-1000, -1000, 1000, 1000),))
    c = NavigationCoordinator(planner, runner=Runner())
    state = VehicleState(0, 0, 0, 0, 0)
    sources = set()
    reasons = []
    for index in range(300):
        if c.runner.busy:
            c.runner.ready = True
        r = c.tick(world, state, goal, index * .05, goal_yaw=.1)
        reasons.append(r.status)
        assert r.action is not None, (r.status, r.events)
        assert r.action_certificate.matches(r.action, r.grid, limits, r.action.route_version)
        assert r.certificate.matches(r.candidate, r.grid, limits, r.candidate.route_version)
        sources.add(r.action.source)
        state = execute_action(r.action, state)
        if r.status == 'ARRIVED':
            break
    else:
        pytest.fail(f'real solver terminal pose did not converge: {state}, {reasons[-20:]}')
    assert 'mpcc:acados-sqp-rti' in sources
    assert abs(api().angle_error(.1, state.yaw)) <= .05
    assert np.linalg.norm(np.array(goal) - [state.x, state.y]) <= .1
    assert abs(state.v) <= .01 and abs(state.w) <= .01
