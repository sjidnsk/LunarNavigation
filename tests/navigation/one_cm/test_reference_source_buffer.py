"""固定有限来源与当次派生分离；重定时不复制或充值原尾段。"""
from importlib import import_module, util
import math

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.certification import certify
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, Route, Trajectory, VehicleState, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.dynamics import step
from tests.navigation.one_cm.test_candidate_handover import PulseWorker
from tests.navigation.one_cm.test_async_execution import PendingGlobal


def source(curved=False):
    dt = .05
    controls = np.zeros((80, 3))
    controls[:, 2] = .2
    if curved:
        controls[4:12, 1] = .5
    states = [np.array([0., 0., 0., .2, 0., 0.])]
    for control in controls:
        states.append(step(states[-1], control, dt))
    return Trajectory(1, 1, np.arange(81) * dt, states, controls, 'forward', 'fixed-source-probe')


def make_source_coordinator(original):
    worker = PulseWorker(); worker.pending = {}
    limits = WheelLimits(angular_speed_rps=1.)
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(),
                                   limits=limits, revalidate_current_map=True)
    c._set_goal((2., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [2, 0]], (2, 0))
    c._policy = False
    world = WorldSnapshot(1, (), (CellRect(-1000, -1000, 1000, 1000),))
    c.buffer.install(original, 0.)
    return c, world


def test_long_half_speed_source_tail_is_finite_and_brakes_before_original_end():
    original = source()
    c, world = make_source_coordinator(original)
    state = VehicleState(0, 0, 0, .2, 0)
    stopping = False
    first_stop = None
    for index in range(1, 301):
        now = index * .05
        desired = 0. if stopping or 1.5 <= now < 3.5 else .1
        a = float(np.clip((desired - state.v) / .05, -.5, .5))
        state = VehicleState(*step(np.r_[list(vars(state).values()), 0.], [a, 0, 0], .05)[:5])
        result = c.tick(world, state, (2, 0), now, goal_yaw=0.)
        assert result.candidate is not None
        assert result.certificate.matches(result.candidate, result.grid, c.limits, 1)
        if result.candidate.source == 'brake':
            stopping = True
            if first_stop is None: first_stop = (now, state.x)
        else:
            assert result.candidate.states[-1, 0] <= original.states[-1, 0] + 1e-6
    assert first_stop is not None
    assert first_stop[1] < original.states[-1, 0]
    assert state.x <= original.states[-1, 0] + 1e-6 and abs(state.v) < 1e-10


def test_original_corner_does_not_stay_in_the_future_after_physical_progress():
    original = source(curved=True)
    c, world = make_source_coordinator(original)
    exposed_turn = []
    for index in range(1, 101):
        now = index * .05
        # 与审查 probe 一样固定合法直行反馈；偏离源弯道后可以拒绝，不能
        # 把弯道整体后推成永远可沿直线走的模板。
        state = VehicleState(.1 * now, 0, 0, .1, 0)
        result = c.tick(world, state, (2, 0), now, goal_yaw=0.)
        if result.candidate is None or result.candidate.source == 'brake':
            continue
        angular = np.flatnonzero(np.abs(result.candidate.controls[:, 1]) > 1e-7)
        if .05 <= state.x <= .10 and len(angular):
            exposed_turn.append(float(result.candidate.times[angular[0]]))
    assert exposed_turn and min(exposed_turn) < .1


def buffer_api():
    name = 'scripts.navigation.one_cm.nav1cm.reference_buffer'
    assert util.find_spec(name) is not None, '固定source/独立派生buffer尚未实现'
    return import_module(name).ReferenceTrajectoryBuffer


def test_source_digest_identity_and_monotonic_phase_survive_pause_and_derived_refresh():
    Buffer = buffer_api()
    original = source()
    c, world = make_source_coordinator(original)
    assert isinstance(c.buffer, Buffer)
    digest = original.digest()
    phase = 0.
    for index in range(1, 61):
        x = min(index * .002, .04)
        state = VehicleState(x, 0, 0, .04 if index < 20 else 0., 0)
        result = c.tick(world, state, (2, 0), index * .05, goal_yaw=0.)
        assert result.status == 'BUFFER_EXECUTING'
        assert c.buffer.source is original
        assert c.buffer.source.digest() == digest
        assert c.buffer.source_phase_s >= phase - 1e-12
        phase = c.buffer.source_phase_s
        assert phase == pytest.approx(x / .2, abs=1e-5)
    assert c.buffer.source_phase_s == pytest.approx(.2, abs=1e-5)
    assert c.buffer.remaining_s(1000.) == pytest.approx(3.8, abs=1e-5)


def test_reference_spin_retimed_velocity_uses_physical_bridge_and_finite_angle():
    Buffer = buffer_api()
    dt = .05
    controls = np.zeros((80, 3))
    states = [np.array([0., 0., 0., 0., .4, 0.])]
    for control in controls: states.append(step(states[-1], control, dt))
    original = Trajectory(1, 1, np.arange(81) * dt, states, controls, 'spin', 'spin-source')
    buffer = Buffer()
    buffer.install(original, 0.)
    state = VehicleState(0, 0, .2, 0, 0)
    limits = WheelLimits(angular_speed_rps=1.)
    c, world = make_source_coordinator(source())
    from scripts.navigation.one_cm.nav1cm.maps import extract_window
    grid = extract_window(world, state, False)
    derived = buffer.current(state, 20., .05, limits=limits)
    assert np.array_equal(derived.states[0, :5], list(vars(state).values()))
    assert np.any(derived.controls[:, 1] > 0)
    assert derived.states[-1, 2] <= original.states[-1, 2] + 1e-6
    assert certify(derived, grid, limits, 1).valid


def test_new_obstacle_and_new_goal_retire_fixed_source():
    buffer_api()
    c, world = make_source_coordinator(source())
    state = VehicleState(0, 0, 0, .1, 0)
    result = c.tick(world, state, (2, 0), .05, goal_yaw=0.)
    assert c.buffer.source is not None
    changed = WorldSnapshot(2, (CellRect(70, -10, 75, 10),), world.known_free)
    result = c.tick(changed, state, (2, 0), .1, goal_yaw=0.)
    assert c.buffer.source is None
    assert result.candidate is None or result.candidate.source == 'brake'
    c.buffer.install(source(), .1)
    c.replace_goal((0, 2), goal_yaw=0., generation=1)
    assert c.buffer.source is None


@pytest.mark.parametrize('v,w', [(.2, 0.), (-.2, 0.), (0., .4), (.2, .2), (.05, -.03)])
def test_source_reserve_matches_existing_physical_brake(v, w):
    buffer_api()
    from scripts.navigation.one_cm.nav1cm.reference_buffer import _stopping_geometry, _travel
    from scripts.navigation.one_cm.nav1cm.seeds import brake_candidate
    from types import SimpleNamespace
    limits = WheelLimits(angular_speed_rps=1.)
    actual = VehicleState(0, 0, 0, v, w)
    tail = brake_candidate(actual, SimpleNamespace(epoch=1), SimpleNamespace(route_version=1, progress_s=0.), limits)
    distance = sum(_travel(tail.states[i, 3], u[0], dt) for i, (u, dt) in enumerate(zip(tail.controls, np.diff(tail.times))))
    angle = sum(_travel(tail.states[i, 4], u[1], dt) for i, (u, dt) in enumerate(zip(tail.controls, np.diff(tail.times))))
    reserve = _stopping_geometry(np.r_[list(vars(actual).values()), 0.], limits, .05)
    assert reserve == pytest.approx((abs(v) * .05 + distance, abs(w) * .05 + angle), abs=1e-12)
