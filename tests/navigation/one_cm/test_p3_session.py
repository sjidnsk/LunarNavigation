"""P3 会话目标生命周期；使用真实协调器，worker 由可控单在途入口隔离。"""
from importlib import import_module, util

import pytest

from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, VehicleState, WorldSnapshot
from tests.navigation.one_cm.test_async_execution import ManualLocal, PendingGlobal
from tests.navigation.one_cm.test_coordinator import Runner


def api(name):
    qualified = 'scripts.navigation.one_cm.nav1cm.' + name
    assert util.find_spec(qualified) is not None, f'{name} 接口尚未实现'
    return import_module(qualified)


def make_session(runner=None):
    session_api, conversion = api('p3_session'), api('ros_conversion')
    coordinator = AsyncNavigationCoordinator(local_worker=ManualLocal(),
                                            global_runner=runner or PendingGlobal(),
                                            revalidate_current_map=True)
    session = session_api.NavigationSession(coordinator)
    session.set_world(WorldSnapshot(1, (CellRect(250, -10, 270, 10),),
                                    (CellRect(-1000, -1000, 1000, 1000),)))
    session.offer_state(VehicleState(0, 0, 0, 0, 0))
    return session, conversion.PoseGoal


def test_valid_goal_then_invalid_before_tick_keeps_valid_goal():
    session, Goal = make_session()
    assert session.offer_goal(Goal(0, 2, .3))
    generation = session.accepted_generation
    assert not session.offer_goal(Goal(2.6, 0, 0))
    assert session.goal == Goal(0, 2, .3)
    assert session.accepted_generation == generation
    packet = session.tick(0.)
    assert packet.generation == generation
    assert session.coordinator._goal == (0., 2.)


def test_unknown_and_outside_goal_accepted_and_later_block_does_not_cancel():
    session, Goal = make_session()
    assert session.offer_goal(Goal(1000, -1000, 1.))
    generation = session.accepted_generation
    session.set_world(WorldSnapshot(2, (CellRect(99990, -100010, 100010, -99990),), ()))
    packet = session.tick(0.)
    assert session.goal == Goal(1000, -1000, 1.)
    assert session.accepted_generation == generation
    assert any(e['kind'] == 'GOAL_REJECTED' and e['reason'] == 'GOAL_IN_OBSTACLE' for e in packet.result.events)
    assert packet.route is None


def test_a_b_a_between_ticks_retires_original_a_and_old_packet():
    runner = Runner()
    session, Goal = make_session(runner)
    assert session.offer_goal(Goal(0, 3, 0))
    first = session.tick(0.)
    revision = session.coordinator.goal_revision
    assert session.offer_goal(Goal(0, -3, 1))
    assert session.offer_goal(Goal(0, 3, 0))
    assert not session.publication_current(first)
    runner.ready = True
    packet = session.tick(.05)
    assert session.coordinator.goal_revision == revision + 2
    assert packet.route is None
    assert any(e['kind'] == 'STALE_GLOBAL_RESULT' for e in packet.result.events)


def test_valid_new_goal_kept_when_global_search_reports_no_path():
    runner = Runner()
    session, Goal = make_session(runner)
    assert session.offer_goal(Goal(0, 3, 0))
    session.tick(0.)
    assert session.offer_goal(Goal(0, -3, 0))
    session.coordinator.global_budget_s = 0.
    runner.ready = True
    session.tick(.05)
    # global_budget=0 是有界正式失败，不是目标有效性的证据。
    runner.ready = True
    packet = session.tick(.1)
    assert session.goal == Goal(0, -3, 0)
    assert session.accepted_generation == 2
    assert session.publication_current(packet)
    assert any(e['kind'] == 'GLOBAL_FAILED' and e['reason'] == 'BUDGET_EXHAUSTED' for e in packet.result.events)


def test_session_waits_for_required_data_and_not_due_packet_is_not_publishable():
    session_api, conversion = api('p3_session'), api('ros_conversion')
    coordinator = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=PendingGlobal())
    session = session_api.NavigationSession(coordinator)
    assert session.tick(0.).waiting
    assert session.offer_goal(conversion.PoseGoal(0, 0, 0))
    assert session.tick(.1).waiting
    session.set_world(WorldSnapshot(1, (), (CellRect(-1000, -1000, 1000, 1000),)))
    session.offer_state(VehicleState(0, 0, 0, 0, 0))
    packet = session.tick(.2)
    assert packet.result.status == 'ARRIVED'
    assert session.publication_current(packet)
    assert not session.publication_current(session.tick(.201))


def test_identical_periodic_goal_does_not_change_generation_or_retire_result():
    runner = Runner()
    session, Goal = make_session(runner)
    assert session.offer_goal(Goal(0, 3, .2))
    packet = session.tick(0.)
    revision, generation = session.coordinator.goal_revision, session.accepted_generation
    assert session.offer_goal(Goal(0, 3, .2 + 2 * 3.141592653589793))
    assert session.accepted_generation == generation
    assert session.coordinator.goal_revision == revision
    assert session.publication_current(packet)
    runner.ready = True
    assert session.tick(.05).route is not None
