"""未执行的新计划按实际状态接纳；已执行缓冲才按 elapsed 裁剪。"""
from dataclasses import replace
import os
from pathlib import Path
import time

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.certification import certify
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, Route, Trajectory, VehicleState, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.dynamics import step
from scripts.navigation.one_cm.nav1cm.execution import execute_action
from scripts.navigation.one_cm.nav1cm.maps import extract_window
from scripts.navigation.one_cm.nav1cm.trajectory_buffer import TrajectoryBuffer
from tests.navigation.one_cm.test_async_execution import PendingGlobal


def scene(epoch=1):
    world = WorldSnapshot(epoch, (), (CellRect(-1000, -1000, 1000 + epoch, 1000),))
    return world, extract_window(world, VehicleState(0, 0, 0, 0, 0), False)


def trajectory(velocities=(0., 0.), *, mode='forward', state=None, times=(0., .2, .4)):
    state = np.zeros(6) if state is None else np.asarray(state, dtype=float)
    controls = np.array([[velocities[0], 0., 0.], [velocities[1], 0., 0.]])
    states = [state]
    for control, dt in zip(controls, np.diff(times)):
        states.append(step(states[-1], control, dt))
    return Trajectory(1, 1, times, states, controls, mode, 'handover-analytic')


@pytest.mark.parametrize('sign,mode', [(1., 'forward'), (-1., 'reverse')])
def test_unexecuted_acceleration_is_not_lost_to_computation_age(sign, mode):
    _, grid = scene()
    limits = WheelLimits()
    original = trajectory((sign * .5, -sign * .5), mode=mode)
    measured = VehicleState(0, 0, 0, 0, 0)
    assert certify(original, grid, limits, 1).valid
    wrongly_aged = TrajectoryBuffer().rebase(original, measured, .1, .05)
    assert certify(wrongly_aged, grid, limits, 1).reason.value == 'MODE'
    buffer = TrajectoryBuffer()
    assert callable(getattr(buffer, 'reanchor', None)), '未执行候选的物理状态接续入口尚未实现'
    accepted = buffer.reanchor(original, measured, .05, limits)
    assert certify(accepted, grid, limits, 1).valid
    assert np.array_equal(accepted.states[0, :5], [0, 0, 0, 0, 0])
    assert accepted.states[1, 3] * sign > 0
    assert accepted.states[-1, 3] == pytest.approx(0.)
    assert accepted.times[-1] == pytest.approx(.4)
    assert buffer.last_alignment['source_phase_s'] == pytest.approx(0.)
    assert original.digest() == buffer.last_alignment['original_candidate_digest']


def test_moving_candidate_uses_physical_bridge_instead_of_velocity_offset_or_clipping():
    _, grid = scene()
    limits = WheelLimits()
    original = trajectory((.5, -.5), state=[0., 0., 0., .1, 0., 0.], times=(0., .2, .6))
    assert certify(original, grid, limits, 1).valid
    measured = VehicleState(.035, 0, 0, .18, 0)
    buffer = TrajectoryBuffer()
    assert callable(getattr(buffer, 'reanchor', None)), '物理速度衔接尚未实现'
    accepted = buffer.reanchor(original, measured, .1, limits)
    assert certify(accepted, grid, limits, 1).valid
    assert np.array_equal(accepted.states[0, :5], list(vars(measured).values()))
    assert np.max(accepted.states[:, 3]) <= limits.linear_speed_mps + 1e-9
    assert np.min(accepted.states[:, 3]) >= -1e-9
    assert accepted.states[-1, 3] == pytest.approx(0., abs=1e-12)
    assert buffer.last_alignment['bridge_duration_s'] > 0
    # 独立核对每一段是受限制动/加速，不允许裁状态速度蒙混 dynamics。
    for index, control in enumerate(accepted.controls):
        assert abs(control[0]) <= limits.linear_accel_mps2 + 1e-12
        assert np.allclose(step(accepted.states[index], control, np.diff(accepted.times)[index]),
                           accepted.states[index + 1], atol=1e-12, rtol=0.)


def test_already_executing_buffer_still_uses_elapsed_time():
    _, grid = scene()
    original = trajectory((.5, -.5))
    buffer = TrajectoryBuffer()
    buffer.install(original, 10.)
    measured = VehicleState(*step(original.states[0], original.controls[0], .1)[:5])
    remaining = buffer.current(measured, 10.1, .05)
    assert remaining.times[-1] == pytest.approx(.3)
    assert remaining.states[0, 3] == pytest.approx(.05)
    assert remaining.states[1, 3] == pytest.approx(.1)
    assert certify(remaining, grid, WheelLimits(), 1).valid


@pytest.mark.parametrize('case', ['slower_forward', 'faster_forward', 'faster_spin'])
def test_reference_buffer_physically_aligns_legal_retimed_feedback(case):
    _, grid = scene()
    limits = WheelLimits(angular_speed_rps=.2)
    if case == 'slower_forward':
        original = trajectory((.5, -.5))
        measured = VehicleState(.001, 0, 0, .02, 0)
    elif case == 'faster_forward':
        original = trajectory((.5, -.5), state=[0, 0, 0, .1, 0, 0], times=(0, .2, .6))
        measured = VehicleState(.015, 0, 0, .18, 0)
    else:
        source = trajectory((.5, -.5), state=[0, 0, 0, .1, 0, 0], times=(0, .2, .6))
        states = source.states.copy(); controls = source.controls.copy()
        states[:, 2], states[:, 4] = source.states[:, 0], source.states[:, 3]
        states[:, 0] = states[:, 3] = 0.
        controls[:, 1], controls[:, 0] = source.controls[:, 0], 0.
        original = replace(source, states=states, controls=controls, mode='spin')
        measured = VehicleState(0, 0, .015, 0, .18)
    assert certify(original, grid, limits, 1).valid
    buffer = TrajectoryBuffer()
    buffer.install(original, 10.)
    wrongly_offset = buffer.current(measured, 10.1, .05)
    assert not certify(wrongly_offset, grid, limits, 1).valid
    # 严格控制的 elapsed 语义仍在；Path 另用固定source的物理几何进度。
    from scripts.navigation.one_cm.nav1cm.reference_buffer import ReferenceTrajectoryBuffer
    reference = ReferenceTrajectoryBuffer()
    reference.install(original, 10.)
    aligned = reference.current(measured, 10.1, .05, limits=limits)
    assert reference.source is original
    assert reference.last_alignment['bridge_duration_s'] > 0
    assert certify(aligned, grid, limits, 1).valid
    assert np.array_equal(aligned.states[0, :5], list(vars(measured).values()))
    assert reference.last_alignment['derived_distance_m'] <= reference.last_alignment['remaining_source_distance_m'] + 1e-9
    assert reference.last_alignment['derived_angle_rad'] <= reference.last_alignment['remaining_source_angle_rad'] + 1e-9


def test_alignment_does_not_clip_out_of_limit_measured_initial_velocity():
    _, grid = scene()
    limits = WheelLimits()
    original = trajectory((.5, -.5), state=[0, 0, 0, .1, 0, 0], times=(0, .2, .6))
    buffer = TrajectoryBuffer()
    measured = VehicleState(.03, 0, 0, .25, 0)
    aligned = buffer.reanchor(original, measured, .05, limits)
    assert aligned.states[0, 3] == .25
    assert certify(aligned, grid, limits, 1).reason.value == 'LIMIT'


class PulseWorker:
    ready = False
    pending = None
    @property
    def busy(self): return self.pending is not None
    def submit(self, request): self.pending = request
    def poll(self):
        if not self.ready or self.pending is None: return None
        request, self.pending = self.pending, None
        self.ready = False
        candidate = trajectory((.5, -.5))
        return {'request_identity': request['identity'], 'candidate': candidate, 'timings_ms': {}}
    def close(self): pass


def test_production_delayed_new_plan_starts_after_250ms_and_keeps_source_evidence():
    worker = PulseWorker()
    coordinator = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    coordinator._set_goal((.6, 0), 0.)
    coordinator.route = Route(1, 1, [[0, 0], [.6, 0]], (.6, 0))
    world, _ = scene()
    measured = VehicleState(0, 0, 0, 0, 0)
    coordinator.tick(world, measured, (.6, 0), 0., goal_yaw=0.)
    worker.ready = True
    updated, _ = scene(2)
    result = coordinator.tick(updated, measured, (.6, 0), .25, goal_yaw=0.)
    assert any(event['kind'] == 'LOCAL_ACCEPTED' for event in result.events)
    assert result.action.states[-1, 3] > 0
    assert result.certificate.matches(result.candidate, result.grid, coordinator.limits, 1)
    assert result.candidate.map_epoch == 2
    event = next(event for event in result.events if event['kind'] == 'LOCAL_CANDIDATE_REALIGNED')
    assert event['request_identity']['epoch'] == 1
    assert event['computation_age_s'] == .25
    assert event['alignment']['source_phase_s'] == 0.
    assert event['alignment']['original_candidate_digest'] == trajectory((.5, -.5)).digest()


def test_production_buffer_continues_when_consumer_legally_retimes_then_goal_retires_it():
    worker = PulseWorker()
    coordinator = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    coordinator._set_goal((.6, 0), 0.)
    coordinator.route = Route(1, 1, [[0, 0], [.6, 0]], (.6, 0))
    world, _ = scene()
    stationary = VehicleState(0, 0, 0, 0, 0)
    coordinator.tick(world, stationary, (.6, 0), 0., goal_yaw=0.)
    worker.ready = True
    accepted = coordinator.tick(world, stationary, (.6, 0), .25, goal_yaw=0.)
    assert any(e['kind'] == 'LOCAL_ACCEPTED' for e in accepted.events)
    measured = VehicleState(.0003, 0, 0, .01, 0)
    result = coordinator.tick(world, measured, (.6, 0), .3, goal_yaw=0.)
    assert result.status == 'BUFFER_EXECUTING'
    assert not any(e['kind'] == 'BRAKE_REQUIRED' for e in result.events)
    assert result.certificate.matches(result.candidate, result.grid, coordinator.limits, 1)
    event = next(e for e in result.events if e['kind'] == 'BUFFER_CURRENT_STATE_REALIGNED')
    assert event['alignment']['source_phase_s'] == pytest.approx(np.sqrt(4 * .0003), abs=1e-6)
    coordinator.replace_goal((0, .6), goal_yaw=0., generation=1)
    assert coordinator.buffer.candidate is None
    coordinator.route = Route(1, 1, [[0, 0], [0, .6]], (0, .6))
    worker.ready = True
    result = coordinator.tick(world, measured, (0, .6), .4, goal_yaw=0.)
    assert not any(e['kind'] == 'LOCAL_ACCEPTED' for e in result.events)
    assert any(e['kind'] == 'STALE_LOCAL_RESULT' for e in result.events)


def test_reference_buffer_accepts_half_speed_pause_and_resume_with_geometry_remaining():
    worker = PulseWorker()
    worker.pending = {}  # 全程仍在计算，无新候选掩盖缓冲续用行为。
    coordinator = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    coordinator._set_goal((2., 0.), 0.)
    coordinator.route = Route(1, 1, [[0, 0], [2, 0]], (2, 0))
    coordinator._policy = False
    world, grid = scene()
    source = trajectory((0., 0.), state=[0, 0, 0, .2, 0, 0], times=(0, 2., 4.))
    assert certify(source, grid, coordinator.limits, 1).valid
    coordinator.buffer.install(source, 0.)
    measured = VehicleState(0, 0, 0, .2, 0)
    initial_digest = source.digest()
    paused_x = []
    for index in range(1, 141):
        now = index * .05
        # 独立指定合法下游重定时，不读取 candidate 控制或预测状态。
        desired = 0. if 1.5 <= now < 3.5 else .1
        acceleration = float(np.clip((desired - measured.v) / .05, -.5, .5))
        actual = step(np.r_[list(vars(measured).values()), 0.], [acceleration, 0., 0.], .05)
        measured = VehicleState(*actual[:5])
        result = coordinator.tick(world, measured, (2., 0.), now, goal_yaw=0.)
        assert result.status == 'BUFFER_EXECUTING', (now, result.events)
        assert result.certificate.matches(result.candidate, result.grid, coordinator.limits, 1)
        assert np.array_equal(result.candidate.states[0, :5], list(vars(measured).values()))
        assert coordinator._goal == (2., 0.) and result.grid.epoch == world.epoch
        if 2. <= now <= 3.4:
            paused_x.append(measured.x)
    assert max(paused_x) - min(paused_x) < 1e-12
    assert measured.x > .4 and measured.v == pytest.approx(.1)
    assert source.states[-1, 0] - measured.x > .2  # 原几何仍有充分未行进余量。
    assert source.digest() == initial_digest


class DelayThirdRequest:
    def __call__(self, runner, request):
        if request['identity']['sequence'] == 3:
            time.sleep(.25)
        return runner(request)


@pytest.mark.skipif(os.environ.get('NAV1CM_REAL_SOLVER') != '1', reason='real acados required')
@pytest.mark.parametrize('direction', [1., -1.])
def test_real_async_solver_starts_and_arrives_with_moving_late_plan(direction):
    from scripts.navigation.one_cm.nav1cm.local_worker import LocalWorker
    limits = WheelLimits(angular_speed_rps=1.)
    worker = LocalWorker(limits, Path(os.environ['NAV1CM_BUILD_DIR']), request_wrapper=DelayThirdRequest())
    coordinator = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), limits=limits,
                                             revalidate_current_map=True)
    goal = (direction * .6, 0.)
    coordinator._set_goal(goal, 0.)
    coordinator.route = Route(1, 1, [[0, 0], goal], goal)
    measured = VehicleState(0, 0, 0, 0, 0)
    began = time.monotonic()
    events, velocities = [], []
    try:
        for index in range(200):
            while time.monotonic() - began < index * .05:
                time.sleep(.001)
            world, _ = scene(index + 1)
            result = coordinator.tick(world, measured, goal, index * .05, goal_yaw=0.)
            events.extend(result.events)
            assert result.action is not None
            assert np.array_equal(result.action.states[0, :5], list(vars(measured).values()))
            assert result.action_certificate.matches(result.action, result.grid, limits, result.action.route_version)
            measured = execute_action(result.action, measured)
            velocities.append(measured.v)
            if result.status == 'ARRIVED': break
    finally:
        coordinator.close()
    assert abs(measured.x - goal[0]) <= coordinator.goal_tolerance_m
    assert abs(measured.v) <= coordinator.stopped_linear_mps
    assert max(v * direction for v in velocities) > .05
    assert min(v * direction for v in velocities) >= -1e-9
    assert any(e['kind'] == 'LOCAL_ACCEPTED' and e['age_s'] >= .25 for e in events)


def test_async_existing_buffer_braking_entry_publishes_certified_short_action():
    limits=WheelLimits(motion_model='differential_drive')
    worker=PulseWorker();worker.pending={'held':True}
    coordinator=AsyncNavigationCoordinator(local_worker=worker,global_runner=PendingGlobal(),limits=limits)
    coordinator._set_goal((.6,0),0.)
    coordinator.route=Route(1,1,[[0,0],[.6,0]],(.6,0))
    world,grid=scene(); measured=VehicleState(0,0,0,.1,0)
    initial=np.array([0.,0.,0.,.1,0.,0.]); controls=np.array([[-.5,0.,0.],[0.,0.,0.]])
    states=[initial]
    for control in controls: states.append(step(states[-1],control,.2))
    candidate=Trajectory(1,1,[0.,.2,.4],states,controls,'spin','old-buffer')
    assert certify(candidate,grid,limits,1).valid
    coordinator._policy=coordinator.allow_unknown
    coordinator.buffer.install(candidate,0.)
    result=coordinator.tick(world,measured,(.6,0),0.,goal_yaw=0.)
    assert result.status == 'BUFFER_EXECUTING', result.events
    assert result.candidate.source == 'old-buffer' and result.action.source != 'brake'
    assert result.action.states[-1,3] == pytest.approx(.075)
    assert result.action_certificate.valid
    assert not any(e['kind'] in ('ACTION_REJECTED','BRAKE_REQUIRED') for e in result.events)


def test_session_full_candidate_survives_bad_unused_action_but_offline_rejects_it(monkeypatch):
    import scripts.navigation.one_cm.nav1cm.async_coordinator as module
    from scripts.navigation.one_cm.nav1cm.p3_session import NavigationSession
    from scripts.navigation.one_cm.nav1cm.ros_conversion import PoseGoal
    original=module.action_prefix; calls=[]
    def damaged(candidate,state,*args):
        result=original(candidate,state,*args); calls.append(candidate.source)
        if candidate.source == 'old-buffer':
            states=result.states.copy(); states[-1,0]+=.01
            return replace(result,states=states)
        return result
    monkeypatch.setattr(module,'action_prefix',damaged)
    def setup():
        worker=PulseWorker(); worker.pending={'held':True}
        c=AsyncNavigationCoordinator(local_worker=worker,global_runner=PendingGlobal(),limits=WheelLimits(motion_model='differential_drive'))
        session=NavigationSession(c); world,_=scene();session.set_world(world);session.offer_state(VehicleState(0,0,0,.1,0))
        assert session.offer_goal(PoseGoal(.6,0,0)); c.route=Route(1,1,[[0,0],[.6,0]],(.6,0));c._policy=c.allow_unknown
        initial=np.array([0.,0.,0.,.1,0.,0.]);controls=np.array([[-.5,0.,0.],[0.,0.,0.]])
        states=[initial]
        for control in controls:states.append(step(states[-1],control,.2))
        c.buffer.install(Trajectory(1,1,[0.,.2,.4],states,controls,'spin','old-buffer'),0.)
        return session
    session=setup(); packet=session.tick(0.)
    assert packet.candidate.source == 'old-buffer' and packet.result.certificate.valid and not calls
    offline=setup(); result=offline.coordinator.tick(offline.world,offline.state,(.6,0),0.,goal_yaw=0.)
    assert calls and any(e['kind']=='ACTION_REJECTED' and e['reason']=='DYNAMICS' for e in result.events)
    assert result.candidate.source == 'brake' and result.action_certificate.valid


def test_session_does_not_publish_uncertified_full_brake_on_collision():
    from scripts.navigation.one_cm.nav1cm.p3_session import NavigationSession
    from scripts.navigation.one_cm.nav1cm.ros_conversion import PoseGoal
    c=AsyncNavigationCoordinator(local_worker=PulseWorker(),global_runner=PendingGlobal())
    session=NavigationSession(c);session.set_world(WorldSnapshot(1,(),(CellRect(-1000,-1000,1000,1000),)))
    session.offer_state(VehicleState(0,0,0,.1,0));assert session.offer_goal(PoseGoal(.6,0,0))
    session.set_world(WorldSnapshot(2,(CellRect(-1,-1,1,1),),(CellRect(-1000,-1000,1000,1000),)))
    packet=session.tick(0.)
    assert packet.candidate is None and not packet.result.certificate.valid


def test_session_rejects_full_candidate_with_bad_dynamics(monkeypatch):
    from scripts.navigation.one_cm.nav1cm.p3_session import NavigationSession
    from scripts.navigation.one_cm.nav1cm.ros_conversion import PoseGoal
    worker=PulseWorker();worker.pending={'held':True}
    c=AsyncNavigationCoordinator(local_worker=worker,global_runner=PendingGlobal(),limits=WheelLimits(motion_model='differential_drive'))
    session=NavigationSession(c);world,_=scene();session.set_world(world);session.offer_state(VehicleState(0,0,0,.1,0))
    assert session.offer_goal(PoseGoal(.6,0,0));c.route=Route(1,1,[[0,0],[.6,0]],(.6,0));c._policy=c.allow_unknown
    initial=np.array([0.,0.,0.,.1,0.,0.]);control=np.array([-.5,0.,0.]);end=step(initial,control,.2);end[0]+=.01
    candidate=Trajectory(1,1,[0.,.2],[initial,end],[control],'spin','invalid-full')
    monkeypatch.setattr(c.buffer,'current',lambda *a,**kw:candidate)
    packet=session.tick(0.)
    assert any(e.get('reason')=='BUFFER_CERTIFICATION_DYNAMICS' for e in packet.result.events)
    assert packet.candidate is None or packet.candidate.source != 'invalid-full'
