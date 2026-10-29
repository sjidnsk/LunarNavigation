"""调度测试仅隔离线程与消息转换，不把测试地图/worker 作为正式规划证据。"""
from importlib import import_module, util
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import threading
import time
import json

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parents[1]))


def runtime_api():
    assert (PACKAGE / 'lunar_nav1cm_planner/runtime.py').exists(), 'ROS 非阻塞调度接口尚未实现'
    return import_module('lunar_nav1cm_planner.runtime')


def pose(x=0., y=0., frame='map'):
    return NS(header=NS(frame_id=frame), pose=NS(position=NS(x=x, y=y),
              orientation=NS(x=0., y=0., z=0., w=2.)))


def tf():
    return NS(transforms=[NS(header=NS(frame_id='map'), child_frame_id='goal_frame',
              transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))])


class GlobalPending:
    busy = True
    def poll(self): return None
    def close(self): pass


class LocalPending:
    busy = True
    def poll(self): return None
    def close(self): pass


def build_runtime(store=None):
    module = runtime_api()
    from nav1cm.async_coordinator import AsyncNavigationCoordinator
    from nav1cm.contracts import CellRect, VehicleState, WorldSnapshot
    from nav1cm.p3_session import NavigationSession
    session = NavigationSession(AsyncNavigationCoordinator(local_worker=LocalPending(),
                                global_runner=GlobalPending(), revalidate_current_map=True))
    session.set_world(WorldSnapshot(1, (CellRect(250, -10, 270, 10),), (CellRect(-1000, -1000, 1000, 1000),)))
    session.offer_state(VehicleState(0., 0., 0., 0., 0.))
    published, logs = [], []
    runtime = module.PlannerRuntime(session, store, publish=published.append,
                                    ros_now_ns=lambda: 1_000_000, notice=lambda *args: logs.append(args))
    return runtime, session, published, logs


def test_goal_queue_does_not_lose_valid_before_invalid_and_waits_for_tf():
    runtime, session, published, logs = build_runtime()
    runtime.offer_goal(pose(0, 3))
    runtime.offer_goal(pose(2.6, 0))
    runtime._tick_once(0.)
    assert (session.goal.x, session.goal.y) == (0., 3.)
    assert session.accepted_generation == 1
    runtime.offer_goal(pose(0, -3, 'goal_frame'))
    runtime._tick_once(.05)
    assert session.goal.y == 3.
    runtime.offer_tf(tf())
    runtime._tick_once(.1)
    assert session.goal.y == -3.
    assert session.accepted_generation == 2
    assert len(published) == 3


def test_older_unresolved_goal_cannot_override_newer_accepted_goal_after_tf_arrives():
    runtime, session, published, logs = build_runtime()
    runtime.offer_goal(pose(0, 3, 'goal_frame'))
    runtime.offer_goal(pose(0, -3))
    runtime._tick_once(0.)
    runtime.offer_tf(tf())
    runtime._tick_once(.05)
    assert session.goal.y == -3.
    assert session.accepted_generation == 1


def test_map_callback_and_state_goal_callbacks_do_not_wait_for_map_processing():
    module = runtime_api()
    from nav1cm.contracts import WorldSnapshot
    entered, release = threading.Event(), threading.Event()
    class MapStore:
        closed = False
        source_resolution_m = .03
        def apply(self, message, transform):
            entered.set()
            assert release.wait(2.)
            return WorldSnapshot(1, (), ())
        def close(self): self.closed = True
    store = MapStore()
    runtime, session, published, logs = build_runtime(store)
    runtime.start()
    try:
        runtime.offer_map(NS(header=NS(frame_id='map'), info=NS()))
        assert entered.wait(1.)
        started = time.monotonic()
        runtime.offer_goal(pose(0, 3))
        runtime.offer_odometry(NS())
        runtime.offer_tf(tf())
        assert time.monotonic() - started < .05
    finally:
        release.set()
        result = runtime.close()
    assert not result['threads_alive']
    assert store.closed
    applied_logs = [json.loads(detail) for kind, detail in logs if kind == 'MAP_APPLIED']
    assert applied_logs and applied_logs[0]['source_resolution_m'] == .03


def test_map_without_tf_is_retained_until_transform_available():
    runtime_api()
    from nav1cm.contracts import WorldSnapshot
    applied = threading.Event()
    class MapStore:
        def apply(self, message, transform):
            applied.set()
            return WorldSnapshot(1, (), ())
        def close(self): pass
    runtime, session, published, logs = build_runtime(MapStore())
    runtime.start()
    try:
        runtime.offer_map(NS(header=NS(frame_id='goal_frame'), info=NS()))
        assert not applied.wait(.1)
        runtime.offer_tf(tf())
        assert applied.wait(1.)
    finally:
        runtime.close()


def test_notice_memory_is_bounded_by_kind_and_changed_details_log_immediately():
    runtime, session, published, logs = build_runtime()
    for i in range(100):
        runtime._notice('MAP_APPLIED', str(i))
    assert len(runtime._notices) == 1
    assert len(logs) == 100
    runtime._notice('MAP_APPLIED', '99')
    assert len(logs) == 100


def test_actual_grid_and_original_global_success_fields_are_logged_separately():
    runtime, session, published, logs = build_runtime()
    from nav1cm.global_worker import run_request
    class ImmediateGlobal:
        pending = None
        @property
        def busy(self): return self.pending is not None
        def submit(self, request): self.pending = request
        def poll(self):
            if self.pending is None: return None
            request, self.pending = self.pending, None
            return run_request(request)
        def close(self): pass
    session.coordinator.runner = ImmediateGlobal()
    runtime.offer_goal(pose(0, 3))
    runtime._tick_once(0.)
    runtime._tick_once(.05)
    grid = [json.loads(detail) for kind, detail in logs if kind == 'LOCAL_GRID']
    assert grid[0]['resolution_m'] == .01 and grid[0]['shape'] == [500, 500]
    results = [json.loads(detail) for kind, detail in logs if kind == 'GLOBAL_RESULT']
    assert results[0]['planning_outcome'] == 0
    assert results[0]['reason_code'] == 'PLAN_FOUND'
    assert results[0]['has_reference'] is True
    assert results[0]['requested_epoch'] == 1
    assert results[0]['accepted_generation'] == 1
