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


def test_removed_obstacle_preserves_valid_detour_until_periodic_shortcut_refresh():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    obstructed = world(1, (CellRect(145, -10, 155, 10),))
    c.tick(obstructed, state, (3, 0), 0., goal_yaw=0.)
    runner.ready = True
    c.tick(obstructed, state, (3, 0), .05, goal_yaw=0.)
    detour, progress = c.route, c.progress
    assert detour is not None and len(detour.points) > 2
    for now in (.1, 1., 10., 14.9):
        c.tick(world(2), state, (3, 0), now, goal_yaw=0.)
    assert c.route is detour and c.progress == progress
    assert len(runner.requests) == 1
    c.tick(world(2), state, (3, 0), 15., goal_yaw=0.)
    assert c.route is detour and c.progress == progress
    runner.ready = True
    c.tick(world(2), state, (3, 0), 15.05, goal_yaw=0.)
    assert c.route is not detour and len(c.route.points) == 2


def test_static_turning_free_only_and_unrelated_block_changes_wait_for_periodic_refresh():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    runner.ready = True
    c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    route, progress = c.route, c.progress
    for i in range(1, 150):
        scene = world(2 + i, () if i < 300 else (CellRect(1000 + i, 1000, 1100 + i, 1100),), extent=2000 + i)
        c.tick(scene, VehicleState(0, 0, i * .01, 0, .1), (3, 0), i * .1, goal_yaw=0.)
        assert c.route is route and c.progress == progress
    assert len(runner.requests) == 1
    assert runner.requests[0]['request_reason'] in ('INITIAL', 'GOAL_CHANGED')


def test_real_route_blockage_submits_once_and_retires_progress():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0.)
    runner.ready = True
    c.tick(world(), state, (3, 0), .05)
    blocked = world(2, (CellRect(145, -10, 155, 10),))
    for now in (.1, .2, 1., 30.):
        c.tick(blocked, state, (3, 0), now)
        assert c.route is None and c.progress is None
    assert len(runner.requests) == 2
    assert runner.requests[-1]['request_reason'] == 'MAP_CHANGED'


def test_reference_reentry_is_one_event_and_retires_failed_route():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0.)
    runner.ready = True
    c.tick(world(), state, (3, 0), .05)
    displaced = VehicleState(0, 5, 0, 0, 0)
    c.tick(world(), displaced, (3, 0), .1)
    assert c.route is None and c.progress is None
    for now in (.2, 1., 30.):
        c.tick(world(), displaced, (3, 0), now)
    assert len(runner.requests) == 2
    assert runner.requests[-1]['request_reason'] == 'REFERENCE_REENTRY'


@pytest.mark.parametrize('failure', ['no_route', 'submit', 'poll'])
def test_global_failure_consumes_event_without_hot_retry_and_keeps_goal(failure):
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from tests.navigation.one_cm.test_coordinator import RejectPlanner
    class FailingRunner(Runner):
        def submit(self, request):
            if failure == 'submit':
                self.requests.append(request)
                raise RuntimeError('submit failed')
            super().submit(request)
        def poll(self):
            if failure == 'poll' and self.pending is not None:
                self.pending = None
                raise RuntimeError('worker failed')
            result = super().poll()
            if result is not None:
                result.update(route=None, reason_code='NO_PATH', has_reference=False)
            return result
    runner = FailingRunner()
    c = NavigationCoordinator(RejectPlanner(), runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0.)
    runner.ready = True
    for i, now in enumerate((.1, 1., 10., 14.9)):
        c.tick(world(2 + i, extent=2000 + i), state, (3, 0), now)
    assert len(runner.requests) == 1
    assert c._goal == (3., 0.) and c.route is None
    c.tick(world(6, extent=2004), state, (3, 0), 15.)
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


def test_production_buffer_preserves_short_source_and_certifies_explicit_tail():
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
    # 来源短于停车时间时保留有限后缀，并认证附加的实际制动尾。
    candidate = action_prefix(result.candidate, state, .15)
    assert certify(candidate, result.grid, c.limits, 1).valid
    c.buffer.install(candidate, .05)
    result = c.tick(world(), state, (3, 0), .1, goal_yaw=0.)
    assert result.candidate.source == candidate.source
    assert c.buffer.source is candidate and result.certificate.valid
    assert result.candidate.states[-1,3] == pytest.approx(0., abs=1e-9)
    assert not any(e['kind'] == 'BRAKE_REQUIRED' for e in result.events)

@pytest.mark.parametrize('offset', [0., .01])
def test_compatible_route_refresh_keeps_finite_certified_source(offset):
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    c._set_goal((3., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    state = VehicleState(0, 0, 0, .1, 0)
    state = execute_action(c.tick(world(), state, (3, 0), 0., goal_yaw=0.).action, state)
    worker.ready = True
    accepted = c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    state = execute_action(accepted.action, state)
    source, identity = c.buffer.source, dict(c.buffer.source_identity)
    c.route = Route(2, 1, [[state.x, offset], [3, 0]], (3, 0))
    result = c.tick(world(), state, (3, 0), .1, goal_yaw=0.)
    assert result.action is not None, result.events
    assert result.action.source != 'brake'
    assert c.buffer.source is source
    assert dict(c.buffer.source_identity) == identity
    assert result.reference.route_version == 2
    assert result.candidate.route_version == 1
    assert any(e['kind'] == 'BUFFER_SOURCE_AWAITING_HANDOVER' for e in result.events)
    assert result.certificate.matches(result.candidate, result.grid, c.limits, 1)
    assert result.action_certificate.matches(result.action, result.grid, c.limits, 1)

@pytest.mark.parametrize('mode,v,w', [('forward', .1, 0.), ('reverse', -.1, 0.), ('spin', 0., .05)])
def test_route_refresh_retains_motion_source_until_latest_candidate_takes_over(mode, v, w):
    import numpy as np
    from scripts.navigation.one_cm.nav1cm.contracts import Trajectory
    from scripts.navigation.one_cm.nav1cm.seeds import rollout
    worker = ManualLocal()
    original_poll = worker.poll
    def poll():
        result = original_poll()
        if result is not None:
            tr = result['candidate']
            result['candidate'] = Trajectory(tr.map_epoch, tr.route_version, tr.times,
                                             tr.states, tr.controls, mode)
        return result
    worker.poll = poll
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    c._set_goal((3., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    state = VehicleState(0, 0, 0, v, w)
    c.tick(world(), state, (3, 0), 0., goal_yaw=0.)
    controls = np.zeros((10, 3))
    source = Trajectory(1, 1, np.linspace(0, 1, 11),
                        rollout([0, 0, 0, v, w, 0], controls, .1), controls, mode)
    c.buffer.install(source, 0.)
    c.route = Route(2, 1, [[0, .01], [3, 0]], (3, 0))
    result = c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    assert result.action is not None, result.events
    assert result.action.source != 'brake'
    assert result.action.mode == mode
    assert result.action.route_version == 1
    state = execute_action(result.action, state)
    # 旧请求必须拒绝；随后提交最新路线请求，旧source继续执行。
    worker.ready = True
    result = c.tick(world(), state, (3, 0), .1, goal_yaw=0.)
    assert any(e['kind'] == 'STALE_LOCAL_RESULT' for e in result.events)
    assert c.buffer.source is source
    state = execute_action(result.action, state)
    worker.ready = True
    result = c.tick(world(), state, (3, 0), .15, goal_yaw=0.)
    assert any(e['kind'] == 'LOCAL_ACCEPTED' for e in result.events)
    assert c.buffer.source is not source
    assert result.action.route_version == 2
    assert result.action_certificate.matches(result.action, result.grid, c.limits, 2)


@pytest.mark.parametrize('retire', ['goal', 'policy', 'obstacle', 'exhausted'])
def test_route_refresh_does_not_keep_invalid_source(retire):
    worker = ManualLocal()
    c = AsyncNavigationCoordinator(local_worker=worker, global_runner=PendingGlobal(), revalidate_current_map=True)
    c._set_goal((3., 0.), 0.)
    c.route = Route(1, 1, [[0, 0], [3, 0]], (3, 0))
    state = VehicleState(0, 0, 0, .1, 0)
    state = execute_action(c.tick(world(), state, (3, 0), 0., goal_yaw=0.).action, state)
    worker.ready = True
    result = c.tick(world(), state, (3, 0), .05, goal_yaw=0.)
    state = execute_action(result.action, state)
    c.route = Route(2, 1, [[0, .01], [3, 0]], (3, 0))
    goal, scene = (3, 0), world()
    if retire == 'goal': goal = (0, 3)
    if retire == 'policy': c.allow_unknown = True
    if retire == 'obstacle': scene = world(2, (CellRect(0, -10, 20, 10),))
    if retire == 'exhausted':
        source = c.buffer.source
        state = VehicleState(*source.states[-1, :5])
    result = c.tick(scene, state, goal, .1, goal_yaw=0.)
    assert result.action is None or result.action.source == 'brake'
    assert c.buffer.source is None


@pytest.mark.parametrize('change', ['goal', 'blocked', 'irrelevant'])
def test_inflight_event_binds_latest_goal_and_revalidates_latest_map(change):
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0.)
    goal = (0, 3) if change == 'goal' else (3, 0)
    obstacle = CellRect(145, -10, 155, 10) if change == 'blocked' else CellRect(1000, 1000, 1100, 1100)
    scene = world(2, (obstacle,))
    c.tick(scene, state, goal, .1)
    assert len(runner.requests) == 1
    runner.ready = True
    result = c.tick(scene, state, goal, .2)
    if change == 'irrelevant':
        assert len(runner.requests) == 1 and c.route is not None
    else:
        assert c.route is None and len(runner.requests) == 2
        assert runner.requests[-1]['goal'] == list(goal)
        assert runner.requests[-1]['world']['epoch'] == 2
        runner.ready = True
        c.tick(scene, state, goal, .3)
        assert c.route is not None
    for now in (1., 10., 14.9):
        c.tick(scene, state, goal, now)
    assert len(runner.requests) == (1 if change == 'irrelevant' else 2)


def test_failed_old_request_does_not_retire_existing_valid_route():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0.)
    route = Route(9, 1, [[0, 0], [3, 0]], (3, 0))
    c.route = route
    old_poll = runner.poll
    def poll():
        raw = old_poll()
        if raw is not None:
            raw.update(route=None, reason_code='NO_PATH', has_reference=False)
        return raw
    runner.poll = poll
    runner.ready = True
    c.tick(world(), state, (3, 0), .1)
    assert c.route is route and len(runner.requests) == 1


def test_production_global_period_default_and_factory_forwarding():
    import ast
    from pathlib import Path
    root = Path(__file__).resolve().parents[3] / 'scripts/navigation/one_cm/ros2/lunar_nav1cm_planner'
    tree = ast.parse((root / 'lunar_nav1cm_planner/node.py').read_text(encoding='utf-8'))
    defaults = next(n.value for n in ast.walk(tree)
                    if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'defaults' for t in n.targets))
    assert any(isinstance(k, ast.Constant) and k.value == 'global_period_s' and
               ast.literal_eval(v) == 15. for k, v in zip(defaults.keys, defaults.values))
    assert any(isinstance(n, ast.keyword) and n.arg == 'global_period_s' and
               ast.unparse(n.value) == "settings['global_period_s']" for n in ast.walk(tree))
    assert 'global_period_s: 15.0' in (root / 'config/planner.yaml').read_text(encoding='utf-8')


def test_local_unknown_policy_change_does_not_reject_global_geometry():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world(), state, (3, 0), 0.)
    c.allow_unknown = True
    runner.ready = True
    result = c.tick(world(2, extent=2100), state, (3, 0), .1)
    assert c.route is not None and len(runner.requests) == 1
    assert not any(e['kind'] == 'STALE_GLOBAL_RESULT' for e in result.events)


@pytest.mark.parametrize('smooth', [False, True])
@pytest.mark.parametrize('side', ['rear', 'front'])
def test_existing_route_map_revalidation_only_checks_untravelled_suffix(smooth, side):
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    runner = Runner()
    c = NavigationCoordinator(None, runner=runner, revalidate_current_map=True, reference_smoothing=smooth)
    c._set_goal((10, 0), None)
    c._observe_world(world())
    c._adopt_route(Route(7, 1, [[0, 0], [10, 0]], (10, 0)), [])
    c._pending_reason = None
    c._next_global = 15.
    route = c.route
    progress = ProgressToken(7, 5.)
    c.progress = progress
    obstacle = CellRect(100, -10, 110, 10) if side == 'rear' else CellRect(700, -10, 710, 10)
    events = []
    c._update_route(world(2, (obstacle,)), VehicleState(5, 0, 0, 0, 0), (10, 0), .1, events)
    if side == 'rear':
        assert c.route is route and c.progress is progress
        assert not runner.requests
    else:
        assert c.route is None and c.progress is None
        assert len(runner.requests) == 1 and runner.requests[0]['request_reason'] == 'MAP_CHANGED'


@pytest.mark.parametrize('progress_version,progress_s', [(None, 5.), (8, 5.), (7, float('nan')), (7, -1.), (7, 11.), (7, 0.)])
def test_invalid_or_lagging_progress_cannot_skip_unverified_route_prefix(progress_version, progress_s):
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    c = NavigationCoordinator(None, runner=Runner(), revalidate_current_map=True)
    c._set_goal((10, 0), None)
    c._observe_world(world())
    c._adopt_route(Route(7, 1, [[0, 0], [10, 0]], (10, 0)), [])
    c._pending_reason = None
    c._next_global = 15.
    c.progress = None if progress_version is None else ProgressToken(progress_version, progress_s)
    c._update_route(world(2, (CellRect(100, -10, 110, 10),)), VehicleState(5, 0, 0, 0, 0), (10, 0), .1, [])
    assert c.route is None and c.progress is None


def test_bounded_progress_projection_matches_reference_when_progress_lags_slightly():
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken, WheelLimits
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    c = NavigationCoordinator(None, runner=Runner(), revalidate_current_map=True,
        limits=WheelLimits(body_length_m=.01, body_width_m=.01, minimum_clearance_m=0), clearance_radius_m=.01)
    c._set_goal((10, 0), None)
    c._observe_world(world())
    c._adopt_route(Route(7, 1, [[0, 0], [10, 0]], (10, 0)), [])
    c._pending_reason = None
    c._next_global = 15.
    route = c.route
    progress = ProgressToken(7, 4.8)
    c.progress = progress
    c._update_route(world(2, (CellRect(482, -1, 484, 1),)), VehicleState(5, 0, 0, 0, 0), (10, 0), .1, [])
    assert c.route is route and c.progress is progress


@pytest.mark.parametrize('points,state,progress_s,obstacle', [
    ([[0,0],[4,4],[0,4],[4,0]], (2,2), 2.8, CellRect(295,95,305,105)),
    ([[0,0],[6,0],[0,0],[0,6]], (3,0), 3., CellRect(95,-5,105,5)),
])
def test_future_crossing_and_return_legs_are_not_dropped_as_history(points, state, progress_s, obstacle):
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken, WheelLimits
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    c = NavigationCoordinator(None, runner=Runner(), revalidate_current_map=True,
        limits=WheelLimits(body_length_m=.01, body_width_m=.01, minimum_clearance_m=0), clearance_radius_m=.01)
    goal = tuple(points[-1])
    c._set_goal(goal, None)
    c._observe_world(world())
    c._adopt_route(Route(7, 1, points, goal), [])
    c._pending_reason = None
    c._next_global = 15.
    c.progress = ProgressToken(7, progress_s)
    c._update_route(world(2, (obstacle,)), VehicleState(*state, 0, 0, 0), goal, .1, [])
    assert c.route is None and c.progress is None


def test_passed_shaped_curve_proof_does_not_retire_remaining_sampled_reference():
    import numpy as np
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.reference import _arc_lengths, _point_at
    c = NavigationCoordinator(None, runner=Runner(), revalidate_current_map=True, reference_smoothing=True)
    c._set_goal((10, 10), None)
    c._observe_world(world())
    c._adopt_route(Route(7, 1, [[0,0],[10,0],[10,10]], (10,10)), [])
    c._pending_reason = None
    c._next_global = 15.
    route = c.route
    assert len(route.points) > 3 and len(c._shaped_route.proof_boxes) > 0
    arc = _arc_lengths(route.points)
    progress = ProgressToken(7, float(arc[-1] - 3.))
    c.progress = progress
    xy = _point_at(route.points, arc, progress.s)
    box = np.mean(c._shaped_route.proof_boxes[0].reshape(2,2), axis=0)
    obstacle = CellRect(int(box[0]*100), int(box[1]*100), int(box[0]*100)+1, int(box[1]*100)+1)
    c._update_route(world(2, (obstacle,)), VehicleState(*xy, 0, 0, 0), (10,10), .1, [])
    assert c.route is route and c.progress is progress


def test_incoming_global_candidate_is_checked_in_full_even_after_vehicle_advanced():
    runner = Runner()
    c = AsyncNavigationCoordinator(local_worker=ManualLocal(), global_runner=runner, revalidate_current_map=True)
    c.tick(world(), VehicleState(0, 0, 0, 0, 0), (10, 0), 0.)
    runner.ready = True
    result = c.tick(world(2, (CellRect(100, -10, 110, 10),)),
                    VehicleState(5, 0, 0, 0, 0), (10, 0), .1)
    assert c.route is None and c.progress is None
    assert any(e['kind'] == 'GLOBAL_CURRENT_MAP_REJECTED' for e in result.events)
    assert len(runner.requests) == 2


def test_periodic_refresh_retains_route_progress_and_single_inflight_then_same_suffix():
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken
    runner = Runner()
    c = NavigationCoordinator(None, runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c._update_route(world(), state, (10, 0), 0., [])
    runner.ready = True
    c._update_route(world(), state, (10, 0), .05, [])
    route = c.route
    progress = c.progress = ProgressToken(route.version, 2.)
    state = VehicleState(2, 0, .5, 0, .1)
    for now in (1., 10., 14.99):
        c._update_route(world(), state, (10, 0), now, [])
    assert len(runner.requests) == 1
    c._update_route(world(), state, (10, 0), 15., [])
    assert len(runner.requests) == 2 and runner.requests[-1]['request_reason'] == 'PERIODIC'
    assert c.route is route and c.progress is progress
    for now in (15.05, 16., 29.9):
        c._update_route(world(), state, (10, 0), now, [])
    assert len(runner.requests) == 2
    runner.ready = True
    events = []
    c._update_route(world(2, extent=2100), state, (10, 0), 29.95, events)
    assert c.route is route and c.progress is progress
    assert any(e['kind'] == 'ROUTE_CONTINUITY_RETAINED' for e in events)
    c._update_route(world(2), state, (10, 0), 30., [])
    assert len(runner.requests) == 3


@pytest.mark.parametrize('failure', ['submit', 'poll', 'no_route'])
def test_failed_periodic_refresh_keeps_valid_route_and_retries_at_next_period(failure):
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    runner = Runner()
    c = NavigationCoordinator(None, runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c._update_route(world(), state, (3, 0), 0., [])
    runner.ready = True
    c._update_route(world(), state, (3, 0), .05, [])
    route = c.route
    submit, poll = runner.submit, runner.poll
    def fail_submit(request):
        runner.requests.append(request)
        raise RuntimeError('submit failed')
    def fail_poll():
        if runner.pending is None: return None
        if failure == 'poll':
            runner.pending = None
            raise RuntimeError('worker failed')
        runner.ready = True
        result = poll()
        result.update(route=None, has_reference=False, reason_code='NO_PATH')
        return result
    if failure == 'submit': runner.submit = fail_submit
    else: runner.poll = fail_poll
    c._update_route(world(), state, (3, 0), 15., [])
    for now in (15.05, 16., 20., 29.99):
        c._update_route(world(), state, (3, 0), now, [])
        assert c.route is route and c._goal == (3., 0.)
    assert len(runner.requests) == 2 and c._pending_reason is None
    c._update_route(world(), state, (3, 0), 30., [])
    assert len(runner.requests) == 3


@pytest.mark.parametrize('change', ['goal', 'blocked', 'irrelevant'])
def test_periodic_inflight_result_validates_latest_goal_and_full_map(change):
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    runner = Runner()
    c = NavigationCoordinator(None, runner=runner, revalidate_current_map=True)
    state = VehicleState(0, 0, 0, 0, 0)
    c._update_route(world(), state, (3, 0), 0., [])
    runner.ready = True
    c._update_route(world(), state, (3, 0), .05, [])
    route = c.route
    c._update_route(world(), state, (3, 0), 15., [])
    assert len(runner.requests) == 2
    goal = (0, 3) if change == 'goal' else (3, 0)
    obstacle = CellRect(145, -10, 155, 10) if change == 'blocked' else CellRect(1000, 1000, 1100, 1100)
    scene = world(2, (obstacle,))
    c._update_route(scene, state, goal, 15.05, [])
    assert len(runner.requests) == 2
    runner.ready = True
    events = []
    c._update_route(scene, state, goal, 15.1, events)
    if change == 'irrelevant':
        assert c.route is route and len(runner.requests) == 2
    else:
        assert c.route is None and len(runner.requests) == 3
        assert runner.requests[-1]['goal'] == list(goal)
        assert runner.requests[-1]['world']['epoch'] == 2
        assert any(e['kind'] in ('STALE_GLOBAL_RESULT', 'GLOBAL_CURRENT_MAP_REJECTED') for e in events)
