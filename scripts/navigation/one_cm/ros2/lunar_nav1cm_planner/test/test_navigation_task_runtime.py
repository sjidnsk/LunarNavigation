"""Runtime 同一输入队列内的人工、任务和取消竞争。"""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).parent))
from test_runtime import build_runtime, pose
from nav1cm.contracts import VehicleState


def managed(x=0., y=3., yaw=0., has_yaw=True):
    return NS(target_x_m=x, target_y_m=y, target_yaw_rad=yaw, has_target_yaw=has_yaw)


def test_runtime_rejected_replacement_and_manual_identity():
    runtime, session, published, logs = build_runtime()
    events = []
    runtime.event_sink = events.append
    assert hasattr(runtime, 'offer_managed_goal'), 'Runtime任务入口尚未实现'
    runtime.offer_managed_goal('a', managed())
    runtime.offer_managed_goal('bad', managed(2.6, 0))
    runtime._tick_once(0.)
    assert session.goal.y == 3 and session.accepted_generation == 1
    assert any(e.task_id == 'bad' and e.outcome == 'INVALID_GOAL' for e in events)
    runtime.offer_goal(pose(0, -3))
    runtime._tick_once(.1)
    assert any(e.task_id == 'a' and e.reason_code == 'GOAL_REPLACED' for e in events)


def test_pending_yaw_cancel_and_late_old_cancel_leave_active():
    runtime, session, published, logs = build_runtime()
    events = []
    runtime.event_sink = events.append
    assert hasattr(runtime, 'offer_managed_goal'), 'Runtime任务入口尚未实现'
    session.offer_state(None)
    runtime.offer_managed_goal('waiting', managed(has_yaw=False))
    runtime._tick_once(0.)
    assert events[-1].reason_code == 'WAITING_FOR_POSE'
    runtime.request_cancel('waiting')
    runtime._tick_once(.1)
    assert events[-1].outcome == 'CANCELED'
    session.offer_state(VehicleState(0, 0, 1.2, 0, 0))
    runtime.offer_managed_goal('new', managed(has_yaw=False))
    runtime.request_cancel('waiting')
    runtime._tick_once(.2)
    assert session.goal.yaw == 1.2 and runtime.tasks.active.task_id == 'new'


def test_cancel_queued_during_tick_blocks_old_publication():
    runtime, session, published, logs = build_runtime()
    assert hasattr(runtime, 'offer_managed_goal'), 'Runtime任务入口尚未实现'
    runtime.offer_managed_goal('a', managed())
    tick = session.tick
    def cancel_inside(now):
        packet = tick(now)
        runtime.request_cancel('a')
        return packet
    session.tick = cancel_inside
    runtime._tick_once(0.)
    assert not published
    session.tick = tick
    runtime._tick_once(.1)
    assert session.goal is None and published[-1].global_route.poses == ()
