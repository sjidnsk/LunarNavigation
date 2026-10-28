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
