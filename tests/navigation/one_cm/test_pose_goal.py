"""完整目标身份、已知障碍位姿检查与最终参考边界。"""
from importlib import import_module
import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.contracts import (
    CellRect, LocalReference, ProgressToken, Route, VehicleState, WheelLimits, WorldSnapshot,
)
from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
from scripts.navigation.one_cm.nav1cm.maps import extract_window
from scripts.navigation.one_cm.nav1cm.reference import extract_reference
from tests.navigation.one_cm.test_async_execution import ManualLocal, PendingGlobal
from tests.navigation.one_cm.test_coordinator import RejectPlanner, Runner


def world(blocked=(), free=(CellRect(-2000, -2000, 2000, 2000),), epoch=1):
    return WorldSnapshot(epoch, tuple(blocked), tuple(free))


def coordinator(asynchronous=False, runner=None):
    runner = Runner() if runner is None else runner
    if asynchronous:
        return AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner)
    return NavigationCoordinator(RejectPlanner(), runner=runner)


def pose_api():
    return import_module('scripts.navigation.one_cm.nav1cm.pose_goal')


@pytest.mark.parametrize('asynchronous', [False, True])
def test_same_xy_new_yaw_is_a_new_goal_and_cannot_arrive(asynchronous):
    c = coordinator(asynchronous)
    state = VehicleState(0, 0, 0, 0, 0)
    first = c.tick(world(), state, (0, 0), 0., goal_yaw=0.)
    revision = c.goal_revision
    assert first.status == 'ARRIVED'
    second = c.tick(world(), state, (0, 0), .05, goal_yaw=1.)
    assert second.status != 'ARRIVED'
    assert c.goal_revision > revision
    assert second.action.source == 'terminal-yaw:bounded'
    assert second.action.states[-1, 2] > 0


@pytest.mark.parametrize('asynchronous', [False, True])
def test_final_yaw_satisfied_stops_and_requires_stillness(asynchronous):
    c = coordinator(asynchronous)
    measured = VehicleState(0, 0, 1., .02, .005)
    r = c.tick(world(), measured, (0, 0), 0., goal_yaw=1.)
    assert r.status == 'GOAL_BRAKING'
    assert r.action.source == 'brake'
    assert np.array_equal(r.action.states[0, :5], [0, 0, 1., .02, .005])
    r = c.tick(world(), VehicleState(0, 0, 1.01, 0, 0), (0, 0), .05, goal_yaw=1.)
    assert r.status == 'ARRIVED' and r.action.source == 'brake'
    assert np.all(r.action.states[:, 3:5] == 0)


@pytest.mark.parametrize('asynchronous', [False, True])
@pytest.mark.parametrize('boundary', ['yaw', 'position'])
def test_arrival_requires_braking_endpoint_inside_pose_tolerances(asynchronous, boundary):
    c = coordinator(asynchronous)
    state = (VehicleState(0, 0, .04999, 0, .009) if boundary == 'yaw' else
             VehicleState(.09999, 0, 0, .009, 0))
    r = c.tick(world(), state, (0, 0), 0., goal_yaw=0.)
    assert r.status == 'GOAL_BRAKING'
    assert r.action.source == 'brake'
    assert np.array_equal(r.action.states[0, :5], list(vars(state).values()))


@pytest.mark.parametrize('asynchronous', [False, True])
def test_xy_only_interface_preserves_arrival_without_yaw_constraint(asynchronous):
    c = coordinator(asynchronous)
    r = c.tick(world(), VehicleState(0, 0, 2., 0, 0), (0, 0), 0.)
    assert r.status == 'ARRIVED'


def test_angle_error_uses_shortest_arc_and_normalized_goal_identity():
    api = pose_api()
    assert api.angle_error(-np.pi + .02, np.pi - .02) == pytest.approx(.04)
    assert api.normalize_yaw(2 * np.pi + .1) == pytest.approx(.1)
    c = coordinator()
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (2, 0), 0., goal_yaw=.1)
    revision = c.goal_revision
    c.tick(world(), state, (2, 0), .05, goal_yaw=2 * np.pi + .1)
    assert c.goal_revision == revision
    with pytest.raises(ValueError, match='finite'):
        c.tick(world(), state, (2, 0), .1, goal_yaw=np.nan)


@pytest.mark.parametrize('asynchronous', [False, True])
@pytest.mark.parametrize('sequence', [(0., 1.), (0., 1., 0.)])
@pytest.mark.parametrize('spacing', [.01, .05])
def test_old_global_result_rejected_for_same_xy_yaw_and_a_b_a(asynchronous, sequence, spacing):
    runner = Runner()
    c = coordinator(asynchronous, runner)
    state = VehicleState(0, 0, 0, 0, 0)
    for index, yaw in enumerate(sequence):
        c.tick(world(), state, (3, 0), index * spacing, goal_yaw=yaw)
    runner.ready = True
    r = c.tick(world(), state, (3, 0), .2, goal_yaw=sequence[-1])
    assert c.route is None
    assert any(event['kind'] == 'STALE_GLOBAL_RESULT' for event in r.events)
    assert len(runner.requests) == 2


@pytest.mark.parametrize('sequence', [(0., 1.), (0., 1., 0.)])
def test_old_local_result_and_buffer_rejected_after_pose_change(sequence):
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal())
    state = VehicleState(0, 0, 0, .1, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=sequence[0])
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    c.tick(world(), state, (3, 0), .05, goal_yaw=sequence[0])
    expected = dict(worker.requests[0]['identity'])
    assert expected['goal_revision'] == c.goal_revision
    assert expected['goal_yaw'] == sequence[0]
    for index, yaw in enumerate(sequence[1:], start=2):
        c.tick(world(), state, (3, 0), index * .05, goal_yaw=yaw)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    worker.ready = True
    r = c.tick(world(), state, (3, 0), .2, goal_yaw=sequence[-1])
    assert any(event['kind'] == 'STALE_LOCAL_RESULT' for event in r.events)
    assert r.action.source == 'brake'
    assert not any(event['kind'] == 'LOCAL_ACCEPTED' for event in r.events)


def test_installed_buffer_is_cleared_immediately_for_between_tick_yaw_changes():
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal())
    state = VehicleState(0, 0, 0, .1, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    c.route = route
    c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    worker.ready = True
    accepted = c.tick(world(), state, (3, 0), .1, goal_yaw=0.)
    assert any(event['kind'] == 'LOCAL_ACCEPTED' for event in accepted.events)
    assert c.buffer.candidate is not None
    revision = c.goal_revision
    assert c.tick(world(), state, (3, 0), .11, goal_yaw=1.).status == 'NOT_DUE'
    assert c.buffer.candidate is None
    c.tick(world(), state, (3, 0), .12, goal_yaw=0.)
    assert c.goal_revision == revision + 2
    c.route = route
    r = c.tick(world(), state, (3, 0), .15, goal_yaw=0.)
    assert r.action.source == 'brake'


def test_pose_validity_uses_rectangle_not_global_clearance_and_accepts_unknown():
    api = pose_api()
    limits = WheelLimits()
    obstacle = world((CellRect(60, -20, 70, 20),))
    assert api.check_pose_goal(obstacle, (0, 0), 0., limits).accepted
    assert not api.check_pose_goal(obstacle, (0, 0), np.pi / 4, limits).accepted
    # 图外与未知都不是已知障碍碰撞。
    unknown = world(free=())
    assert api.check_pose_goal(unknown, (0, 0), 0., limits).accepted
    assert api.check_pose_goal(unknown, (1000, -1000), 2., limits).accepted
    inside = api.check_pose_goal(world((CellRect(-10, -10, 10, 10),)), (0, 0), 0., limits)
    assert not inside.accepted and inside.reason_code == 'GOAL_IN_OBSTACLE'


@pytest.mark.parametrize('asynchronous', [False, True])
def test_clear_pose_near_obstacle_is_not_goal_rejected(asynchronous):
    c = coordinator(asynchronous)
    r = c.tick(world((CellRect(60, -20, 70, 20),)), VehicleState(0, 0, 0, 0, 0),
               (0, 0), 0., goal_yaw=0.)
    assert r.status == 'ARRIVED'
    assert not any(event['kind'] == 'GOAL_REJECTED' for event in r.events)


def test_terminal_yaw_attaches_only_to_exact_route_end_not_near_window_exit():
    state = VehicleState(0, 0, 0, 0, 0)
    grid = extract_window(world(), state, False)
    # 出窗截点离真实终点仅5厘米，仍不是最终段。
    route = Route(2, 1, [[0, 0], [2.55, 0]], (2.55, 0))
    partial, _ = extract_reference(route, state, grid, terminal_yaw=1.)
    assert partial.points[-1, 0] == pytest.approx(2.5)
    assert partial.terminal_yaw is None
    route = Route(3, 1, [[0, 0], [2., 0]], (2., 0))
    final, _ = extract_reference(route, state, grid, terminal_yaw=1.)
    assert final.terminal_yaw == 1.
    assert LocalReference(1, 1, [[0, 0]], [0.]).terminal_yaw is None


@pytest.mark.parametrize('asynchronous', [False, True])
def test_projection_at_decimal_route_endpoint_keeps_terminal_yaw(asynchronous):
    c = coordinator(asynchronous)
    c.tick(world(), VehicleState(-.1, 0, 0, 0, 0), (.3, 0), 0., goal_yaw=1.)
    c.route = Route(1, 1, [[-.1, 0], [.3, 0]], (.3, 0))
    r = c.tick(world(), VehicleState(.3, 0, 0, 0, 0), (.3, 0), .05, goal_yaw=1.)
    assert r.reference.terminal_yaw == pytest.approx(1.)
    assert r.action.source == 'terminal-yaw:bounded'


@pytest.mark.parametrize('asynchronous', [False, True])
@pytest.mark.parametrize('remaining', [1e-13, 1e-14])
def test_terminal_near_endpoint_uses_current_goal_not_deduplicated_route(asynchronous, remaining):
    c = coordinator(asynchronous)
    c.tick(world(), VehicleState(-.1, 0, 0, 0, 0), (.3, 0), 0., goal_yaw=1.)
    c.route = Route(1, 1, [[-.1, 0], [.3, 0]], (.3, 0))
    state = VehicleState(.3 - remaining, 0, 0, 0, 0)
    r = c.tick(world(), state, (.3, 0), .05, goal_yaw=1.)
    assert r.reference.terminal_yaw == pytest.approx(1.)
    assert np.array_equal(r.reference.points, [[state.x, state.y]])
    assert r.action.source == 'terminal-yaw:bounded'
    assert r.action_certificate.matches(r.action, r.grid, c.limits, 1)


@pytest.mark.parametrize('asynchronous', [False, True])
def test_terminal_route_reentry_ignores_old_progress_and_window_exit(asynchronous):
    c = coordinator(asynchronous)
    goal = (0, .2)
    c.tick(world(), VehicleState(0, 0, 0, 0, 0), goal, 0., goal_yaw=1.)
    c.route = Route(1, 1, [[0, 0], [3, 0], [3, 3], [0, 3], [0, .2]], goal)
    c.progress = ProgressToken(1, 0.)
    state = VehicleState(0, .2, 0, 0, 0)
    r = c.tick(world(), state, goal, .05, goal_yaw=1.)
    assert r.reference.terminal_yaw == pytest.approx(1.)
    assert np.array_equal(r.reference.points, [[state.x, state.y]])
    assert r.action.source == 'terminal-yaw:bounded'
    assert r.action_certificate.matches(r.action, r.grid, c.limits, 1)
    assert c.progress == ProgressToken(1, 0.)


def test_solver_failure_is_not_a_pose_invalidity_proof():
    c = coordinator()
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (2, 0), 0., goal_yaw=.2)
    c.runner.ready = True
    r = c.tick(world(), state, (2, 0), .05, goal_yaw=.2)
    assert r.status == 'QP_FAILED'
    assert c._goal == (2., 0.) and c._goal_yaw == pytest.approx(.2)
    assert not any(event['kind'] == 'GOAL_REJECTED' for event in r.events)



@pytest.mark.parametrize('model',['differential_drive','curvature_limited'])
@pytest.mark.parametrize('v',[1e-6,-1e-6,.005,-.005])
def test_terminal_residual_velocity_has_certified_braking_then_turn(model,v):
    from dataclasses import replace
    from scripts.navigation.one_cm.nav1cm.pose_goal import terminal_candidate
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    limits=replace(WheelLimits(),motion_model=model)
    state=VehicleState(0,0,0,v,0);grid=extract_window(world(),state,False)
    reference=LocalReference(1,grid.epoch,[[0,0]],[0.],0.,.3)
    c=terminal_candidate(state,grid,reference,limits)
    assert c.mode=='spin' and c.source.startswith('terminal-yaw:bounded')
    assert np.array_equal(c.states[0,:5],list(vars(state).values()))
    assert np.max(abs(c.states[:,2]))>.05
    zero=int(np.flatnonzero(abs(c.states[:,3])<=1e-10)[0])
    assert zero>0 and np.all(abs(c.states[zero:,3])<=1e-10)
    for i,dt in enumerate(np.diff(c.times)):
        assert np.allclose(step(c.states[i],c.controls[i],dt),c.states[i+1],atol=1e-12,rtol=0.)
    assert certify(c,grid,limits,1).valid


@pytest.mark.parametrize('w',[-.03,.03])
def test_terminal_braking_entry_keeps_real_angular_state_and_finishes_yaw(w):
    from dataclasses import replace
    from scripts.navigation.one_cm.nav1cm.pose_goal import terminal_candidate
    from scripts.navigation.one_cm.nav1cm.certification import certify
    limits=replace(WheelLimits(),motion_model='differential_drive')
    state=VehicleState(0,0,0,.005,w);grid=extract_window(world(),state,False)
    reference=LocalReference(1,grid.epoch,[[0,0]],[0.],0.,.2)
    c=terminal_candidate(state,grid,reference,limits)
    assert c.source.startswith('terminal-yaw:bounded')
    assert np.array_equal(c.states[0,:5],list(vars(state).values()))
    assert c.states[-1,2]==pytest.approx(.2,abs=1e-8)
    assert certify(c,grid,limits,1).valid



def test_terminal_braking_prefix_collision_is_still_rejected():
    from dataclasses import replace
    from scripts.navigation.one_cm.nav1cm.pose_goal import terminal_candidate
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from scripts.navigation.one_cm.nav1cm.contracts import CertificationReason
    limits=replace(WheelLimits(),motion_model='differential_drive')
    state=VehicleState(0,0,0,.1,0)
    blocked=WorldSnapshot(2,(CellRect(60,-1,61,1),),(CellRect(-1000,-1000,1000,1000),))
    grid=extract_window(blocked,state,False)
    reference=LocalReference(1,grid.epoch,[[0,0]],[0.],0.,.3)
    c=terminal_candidate(state,grid,reference,limits)
    assert c.source.startswith('terminal-yaw:bounded')
    assert certify(c,grid,limits,1).reason==CertificationReason.COLLISION
