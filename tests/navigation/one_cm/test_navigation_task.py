"""任务结果必须来自真实位姿/停稳，不能来自段尾或传输状态。"""
from dataclasses import replace
from importlib import import_module, util
from tests.navigation.one_cm.test_p3_session import make_session
from tests.navigation.one_cm.test_coordinator import Runner
from scripts.navigation.one_cm.nav1cm.contracts import VehicleState


def task_api(session):
    name = 'scripts.navigation.one_cm.nav1cm.navigation_task'
    assert util.find_spec(name) is not None, '共用任务状态机尚未实现'
    return import_module(name).NavigationTaskCoordinator(session)


def test_invalid_replacement_keeps_active():
    session, Goal = make_session()
    task = task_api(session)
    task.submit('a', Goal(0, 3, 0), 0.)
    rejected = task.submit('bad', Goal(2.6, 0, 0), .1)
    assert rejected.outcome == 'INVALID_GOAL'
    assert rejected.reason_code == 'GOAL_IN_OBSTACLE'
    assert task.active.task_id == 'a' and session.accepted_generation == 1


def test_a_b_a_and_late_cancel():
    session, Goal = make_session()
    task = task_api(session)
    task.submit('a1', Goal(0, 3, 0), 0.)
    task.submit('b', Goal(0, -3, 0), .1)
    task.submit('a2', Goal(0, 3, 0), .2)
    assert task.active.generation == 3
    assert not task.cancel('a1', .3)
    assert task.active.task_id == 'a2'
    task.submit('a3', Goal(0, 3, 0), .4)
    assert task.active.generation == 4


def test_segment_end_not_arrival():
    session, Goal = make_session()
    task = task_api(session)
    task.submit('a', Goal(0, 3, 0), 0.)
    packet = session.tick(0.)
    packet = replace(packet, result=replace(packet.result, status='ARRIVED'))
    task.observe(packet, 0.)
    task.observe(packet, 1.)
    assert task.active.outcome is None


def test_moving_at_goal_not_success():
    session, Goal = make_session()
    task = task_api(session)
    task.submit('a', Goal(0, 0, 0), 0.)
    session.offer_state(VehicleState(0, 0, 0, .1, 0))
    task.observe(session.tick(0.), 0.)
    task.observe(session.tick(.4), .4)
    assert task.active.outcome is None
    session.offer_state(VehicleState(0, 0, 0, 0, 0))
    task.observe(session.tick(.5), .5)
    task.observe(session.tick(.79), .79)
    assert task.active.outcome is None
    task.observe(session.tick(.81), .81)
    assert task.active.outcome == 'GOAL_REACHED'


def test_cancel_requires_measured_stop():
    session, Goal = make_session()
    task = task_api(session)
    task.submit('a', Goal(0, 3, 0), 0.)
    session.offer_state(VehicleState(0, 0, 0, .2, .1))
    assert task.cancel('a', .1)
    packet = session.tick(.1)
    assert packet.route is None and packet.candidate.source.startswith('brake')
    assert abs(packet.candidate.states[-1, 3]) < .01
    task.observe(packet, .1)
    assert task.active.phase == 'CANCELING' and task.active.outcome is None
    task.observe(session.tick(1.), 1.)
    assert task.active.outcome is None
    session.offer_state(VehicleState(0, 0, 0, 0, 0))
    task.observe(session.tick(1.1), 1.1)
    assert task.active.outcome == 'CANCELED'


def test_manual_goal_replaces_managed():
    session, Goal = make_session()
    events = []
    task = task_api(session)
    task.event_sink = events.append
    task.submit('a', Goal(0, 3, 0), 0.)
    task.submit(None, Goal(0, -3, 0), .1)
    assert events[-1].task_id == 'a'
    assert events[-1].outcome == 'CANCELED' and events[-1].reason_code == 'GOAL_REPLACED'
    assert task.active is None and session.goal.y == -3


def test_missing_yaw_waits_measured_heading():
    session, Goal = make_session()
    task = task_api(session)
    session.offer_state(None)
    waiting = task.submit('a', (0, 3, None), 0.)
    assert waiting.reason_code == 'WAITING_FOR_POSE' and session.goal is None
    session.offer_state(VehicleState(0, 0, 1.2, 0, 0))
    task.submit('a', (0, 3, None), .1)
    assert session.goal.yaw == 1.2 and task.active.task_id == 'a'


def test_future_worker_result_invalid_after_cancel():
    runner = Runner()
    session, Goal = make_session(runner)
    task = task_api(session)
    task.submit('a', Goal(0, 3, 0), 0.)
    old = session.tick(0.)
    task.cancel('a', .1)
    assert not session.publication_current(old)
    runner.ready = True
    task.submit('b', Goal(0, 3, 0), .2)
    packet = session.tick(.2)
    assert packet.route is None
    assert any(e['kind'] == 'STALE_GLOBAL_RESULT' for e in packet.result.events)

def test_late_local_worker_result_after_cancel_cannot_reenter_buffer():
    from scripts.navigation.one_cm.nav1cm.contracts import Route
    session, Goal = make_session()
    task = task_api(session)
    task.submit('a', Goal(0, 3, 0), 0.)
    session.coordinator.route = Route(1, 1, [[0, 0], [0, 3]], (0, 3))
    session.tick(0.)
    assert session.coordinator.local_worker.busy
    task.cancel('a', .1)
    assert session.coordinator.buffer.candidate is None
    task.submit('b', Goal(0, 3, 0), .2)
    session.coordinator.route = Route(2, 1, [[0, 0], [0, 3]], (0, 3))
    session.coordinator.local_worker.ready = True
    packet = session.tick(.2)
    assert any(e['kind'] == 'STALE_LOCAL_RESULT' for e in packet.result.events)
    assert not any(e['kind'] == 'LOCAL_ACCEPTED' for e in packet.result.events)
    assert packet.candidate.source == 'brake'


def test_yaw_and_configured_tolerance_and_new_measurements_gate_success():
    session, Goal = make_session()
    task = task_api(session)
    session.coordinator.goal_tolerance_m = .04
    session.coordinator.goal_yaw_tolerance_rad = .02
    task.submit('a', Goal(0, 0, 1.), 0.)
    session.offer_state(VehicleState(.05, 0, 1., 0, 0))
    task.observe(session.tick(0.), 0.)
    task.observe(session.tick(.4), .4)
    assert task.active.outcome is None
    session.offer_state(VehicleState(0, 0, .97, 0, 0))
    task.observe(session.tick(.5), .5)
    task.observe(session.tick(.9), .9)
    assert task.active.outcome is None
    session.offer_state(VehicleState(0, 0, 1., 0, 0))
    task.observe(session.tick(1.), 1.)
    task.observe(session.tick(1.4), 1.4, state_updated=False)
    assert task.active.outcome is None
    task.observe(session.tick(1.5), 1.5)
    assert task.active.outcome == 'GOAL_REACHED'


def test_unavailable_local_buffer_with_global_route_waits_across_not_due():
    from types import SimpleNamespace as NS
    from scripts.navigation.one_cm.nav1cm.p3_session import SessionPacket
    session,Goal=make_session();task=task_api(session);task.submit('a',Goal(0,3,0),0.)
    route=NS(version=7)
    packet=SessionPacket(session.accepted_generation,route,NS(status='BUFFER_UNAVAILABLE'))
    packet=replace(packet,candidate=NS(source="brake"))
    task.observe(packet,.1)
    assert task.active.reason_code=='WAITING_FOR_PATH' and task.active.phase=='PLANNING'
    task.observe(replace(packet,result=NS(status='NOT_DUE')),.11)
    assert task.active.reason_code=='WAITING_FOR_PATH' and task.active.outcome is None
    assert packet.route is route


def test_not_due_retains_valid_executing_feedback_and_never_inherits_old_goal_wait():
    from types import SimpleNamespace as NS
    from scripts.navigation.one_cm.nav1cm.p3_session import SessionPacket
    session,Goal=make_session();task=task_api(session);task.submit('a',Goal(0,3,0),0.)
    route=NS(version=7)
    packet=SessionPacket(session.accepted_generation,route,NS(status='BUFFER_EXECUTING'),candidate=NS())
    task.observe(packet,.1)
    task.observe(replace(packet,result=NS(status='NOT_DUE'),candidate=None),.11)
    assert task.active.phase=='EXECUTING' and task.active.reason_code=='BUFFER_EXECUTING'
    task.observe(replace(packet,result=NS(status='BUFFER_UNAVAILABLE'),candidate=None),.2)
    assert task.active.reason_code=='WAITING_FOR_PATH'
    task.submit('b',Goal(0,-3,0),.3)
    new=replace(packet,generation=session.accepted_generation,result=NS(status='NOT_DUE'),candidate=None)
    task.observe(new,.31)
    assert task.active.task_id=='b' and task.active.phase=='PLANNING' and task.active.reason_code=='GOAL_ACCEPTED'


def test_missing_input_and_retired_route_take_priority_over_not_due_history():
    from types import SimpleNamespace as NS
    from scripts.navigation.one_cm.nav1cm.p3_session import SessionPacket
    session,Goal=make_session();task=task_api(session);task.submit('a',Goal(0,3,0),0.)
    packet=SessionPacket(session.accepted_generation,NS(version=1),NS(status='BUFFER_EXECUTING'),candidate=NS())
    task.observe(packet,.1)
    absent=replace(packet,result=NS(status='NOT_DUE'),candidate=None,route=None)
    task.observe(absent,.11)
    assert task.active.phase=='PLANNING' and task.active.reason_code=='WAITING_FOR_PATH'
    task.observe(replace(absent,waiting=('world','state')),.12)
    assert task.active.reason_code=='WAITING_FOR_MAP'
    task.observe(replace(absent,waiting=('state',)),.13)
    assert task.active.reason_code=='WAITING_FOR_POSE' and task.active.outcome is None



def test_goal_blocked_after_acceptance_cannot_report_arrived():
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot
    session,Goal=make_session();task=task_api(session)
    task.submit('a',Goal(0,0,0),0.)
    session.set_world(WorldSnapshot(2,(CellRect(-20,-20,20,20),),(CellRect(-1000,-1000,1000,1000),)))
    for now in (0.,.4):task.observe(session.tick(now),now)
    assert task.active.outcome is None


def test_task_uses_observed_stop_tolerance_at_acceptance_edge():
    session,Goal=make_session();task=task_api(session)
    task.submit('a',Goal(0,0,0),0.)
    session.offer_state(VehicleState(.09999,0,0,.009,0))
    for now in (0.,.4):
        packet=session.tick(now)
        assert packet.result.status=='ARRIVED'
        task.observe(packet,now)
    assert task.active.outcome == 'GOAL_REACHED'


def test_new_observation_outside_acceptance_resets_arrival_hold():
    session,Goal=make_session();task=task_api(session)
    task.submit('a',Goal(0,0,0),0.)
    session.offer_state(VehicleState(.09999,0,0,.009,0))
    task.observe(session.tick(0.),0.)
    assert task.active.outcome is None
    session.offer_state(VehicleState(.1001,0,0,0,0))
    task.observe(session.tick(.4),.4)
    assert task.active.outcome is None


def test_certified_arrival_survives_not_due_but_not_new_world():
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot
    session,Goal=make_session();task=task_api(session)
    session.coordinator.local_period_s=1.
    task.submit('a',Goal(0,0,0),0.)
    task.observe(session.tick(0.),0.)
    packet=session.tick(.31)
    assert packet.result.status=='NOT_DUE'
    task.observe(packet,.31)
    assert task.active.outcome=='GOAL_REACHED'
    # 另一个任务期间，即使计算槽未到，新障碍也应使原证据失效。
    session,Goal=make_session();task=task_api(session)
    session.coordinator.local_period_s=1.
    task.submit('b',Goal(0,0,0),0.);task.observe(session.tick(0.),0.)
    session.set_world(WorldSnapshot(2,(CellRect(-20,-20,20,20),),(CellRect(-1000,-1000,1000,1000),)))
    task.observe(session.tick(.31),.31)
    assert task.active.outcome is None
