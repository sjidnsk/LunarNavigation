"""生产模式重验旧地图结果，保持原请求证据与安全参考连续性。"""
import pytest

from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, Route, VehicleState, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.execution import action_prefix, execute_action
from scripts.navigation.one_cm.nav1cm.certification import certify
from tests.navigation.one_cm.test_async_execution import ManualLocal, PendingGlobal
from tests.navigation.one_cm.test_coordinator import Runner


def world(epoch=1, blocked=(), extent=2000):
    return WorldSnapshot(epoch, tuple(blocked), (CellRect(-extent, -extent, extent, extent),))


def test_rolling_map_accepts_delayed_global_but_keeps_original_formal_result():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    for i in range(1, 5):
        c.tick(world(i + 1, extent=2000 + i), state, (3, 0), i * .05, goal_yaw=0.)
    runner.ready = True
    result = c.tick(world(6, extent=2006), state, (3, 0), .25, goal_yaw=0.)
    assert c.route is not None and c.route.map_epoch == 1
    assert result.global_result['requested_epoch'] == 1
    assert result.global_result['planning_outcome'] == 0
    assert result.global_result['reason_code'] == 'PLAN_FOUND'
    assert result.global_result['has_reference'] is True
    assert any(e['kind'] == 'GLOBAL_CURRENT_MAP_REVALIDATED' and e['current_epoch'] == 6 for e in result.events)
    version = c.route.version
    for i in range(7, 14):
        result = c.tick(world(i, extent=2000 + i), state, (3, 0), i * .05, goal_yaw=0.)
        assert c.route.version == version
    assert len(runner.requests) == 1


def test_new_obstacle_rejects_old_global_geometry_and_old_goal_always_rejected():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    runner.ready = True
    result = c.tick(world(2, (CellRect(130, -10, 140, 10),)), state, (3, 0), .1, goal_yaw=0.)
    assert c.route is None
    assert any(e['kind'] == 'GLOBAL_CURRENT_MAP_REJECTED' for e in result.events)
    runner.ready = True
    result = c.tick(world(3), state, (0, 3), .2, goal_yaw=0.)
    assert c.route is None
    assert any(e['kind'] == 'STALE_GLOBAL_RESULT' for e in result.events)


def test_removed_obstacle_accepts_new_shorter_global_geometry():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    obstructed = world(1, (CellRect(145, -10, 155, 10),))
    c.tick(obstructed, state, (3, 0), 0., goal_yaw=0.)
    runner.ready = True
    c.tick(obstructed, state, (3, 0), .05, goal_yaw=0.)
    detour = c.route
    assert detour is not None and len(detour.points) > 2
    c.tick(world(2), state, (3, 0), .1, goal_yaw=0.)
    assert len(runner.requests) == 1
    c.tick(world(2), state, (3, 0), 1., goal_yaw=0.)
    runner.ready = True
    result = c.tick(world(2), state, (3, 0), 1.05, goal_yaw=0.)
    assert c.route.version != detour.version
    assert c.route.points.tolist() == [[0., 0.], [3., 0.]]
    assert any(e['kind'] == 'ROUTE_ACCEPTED' for e in result.events)


def test_equivalent_global_suffix_retains_route_version_after_far_block_change():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    c._set_goal((3., 0.), 0.)
    c.route = Route(7, 1, [[0, 0], [3, 0]], (3, 0))
    c._version = 7
    state = VehicleState(.1, 0, 0, 0, 0)
    c.tick(world(2, (CellRect(1000, 1000, 1100, 1100),)), state, (3, 0), 0., goal_yaw=0.)
    runner.ready = True
    result = c.tick(world(3, (CellRect(1000, 1000, 1100, 1100),), extent=2001), state, (3, 0), .1, goal_yaw=0.)
    assert c.route.version == 7
    assert any(e['kind'] == 'ROUTE_CONTINUITY_RETAINED' for e in result.events)


def test_safe_route_coalesces_continuous_blocked_changes_until_global_period():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    runner.ready = True
    c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    for i in range(2, 20):
        runner.ready = True
        c.tick(world(i, (CellRect(1000 + i, 1000, 1100 + i, 1100),)), state,
               (3, 0), i * .05, goal_yaw=0.)
        assert len(runner.requests) == 1
    c.tick(world(20), state, (3, 0), 1., goal_yaw=0.)
    assert len(runner.requests) == 2


def test_rolling_map_delayed_local_reintegrates_and_recertifies_current_grid():
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    state = VehicleState(0, 0, 0, .1, 0)
    c._set_goal((3., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    state = execute_action(c.tick(world(), state, (3, 0), 0., goal_yaw=0.).action, state)
    for i in range(1, 5):
        result = c.tick(world(i + 1, extent=2000 + i), state, (3, 0), i * .05, goal_yaw=0.)
        state = execute_action(result.action, state)
    original_identity = dict(worker.requests[0]['identity'])
    worker.ready = True
    result = c.tick(world(6, extent=2006), state, (3, 0), .25, goal_yaw=0.)
    assert any(e['kind'] == 'LOCAL_ACCEPTED' for e in result.events)
    assert any(e['kind'] == 'LOCAL_CURRENT_MAP_REVALIDATED' for e in result.events)
    assert result.candidate.map_epoch == 6
    assert result.certificate.matches(result.candidate, result.grid, c.limits, 1)
    assert worker.requests[0]['identity'] == original_identity
    state = execute_action(result.action, state)
    for i in range(6, 11):
        result = c.tick(world(i + 1, extent=2000 + i), state, (3, 0), i * .05, goal_yaw=0.)
        assert result.candidate.source != 'brake'
        state = execute_action(result.action, state)


def test_new_obstacle_rejects_delayed_local_even_if_global_route_cached():
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    c._set_goal((3., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    state = VehicleState(0, 0, 0, .1, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    worker.ready = True
    result = c.tick(world(2, (CellRect(60, -10, 65, 10),)), state, (3, 0), .25, goal_yaw=0.)
    assert not any(e['kind'] == 'LOCAL_ACCEPTED' for e in result.events)
    assert result.candidate is None or result.candidate.source == 'brake'


def test_production_buffer_brakes_with_physical_reserve_before_exhaustion():
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    c._set_goal((3., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    state = VehicleState(0, 0, 0, .2, 0)
    result = c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    state = execute_action(result.action, state)
    worker.ready = True
    result = c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    assert result.candidate.source != 'brake'
    # 剩余 0.15s，而当前v需要0.35s刹停，必须提前转为完整认证制动。
    candidate = action_prefix(result.candidate, state, .15)
    assert certify(candidate, result.grid, c.limits, 1).valid
    c.buffer.install(candidate, .05)
    result = c.tick(world(), state, (3, 0), .1, goal_yaw=0.)
    assert result.candidate.source == 'brake'
    assert any(e['kind'] == 'BRAKE_REQUIRED' and 'RESERVE' in e['reason'] for e in result.events)
