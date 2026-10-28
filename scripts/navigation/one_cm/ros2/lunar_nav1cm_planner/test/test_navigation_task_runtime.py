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


def odometry_at_goal(speed=0.):
    return NS(header=NS(frame_id='map'), pose=NS(pose=pose().pose),
              twist=NS(twist=NS(linear=NS(x=speed), angular=NS(z=0.))))


def arriving_runtime():
    runtime, session, published, logs = build_runtime()
    events = []
    runtime.event_sink = events.append
    runtime.offer_managed_goal('a', managed(0., 0.))
    runtime.offer_odometry(odometry_at_goal())
    runtime._tick_once(0.)
    return runtime, session, events


def assert_cancel_brakes_then_finishes(runtime, session, events):
    # 排队取消之后实测又在运动，不能把旧停止观测当取消完成。
    runtime.offer_odometry(odometry_at_goal(.2))
    packet = runtime._tick_once(.45)
    assert session.goal is None and session.canceling
    assert packet.candidate.source == 'brake'
    assert runtime.tasks.active.phase == 'CANCELING'
    assert runtime.tasks.active.outcome is None
    runtime.offer_odometry(odometry_at_goal())
    runtime._tick_once(.5)
    assert runtime.tasks.active.outcome == 'CANCELED'
    assert not any(event.outcome == 'GOAL_REACHED' for event in events)


def test_cancel_after_observation_entry_cannot_commit_old_arrival():
    """审查I1的确定性交错：guard以后、observe内部执行以前重入cancel。"""
    runtime, session, events = arriving_runtime()
    original = runtime.tasks.observe
    def cancel_after_guard(packet, now_s, **kwargs):
        runtime.request_cancel('a')
        assert runtime.tasks.active.outcome is None
        return original(packet, now_s, **kwargs)
    runtime.tasks.observe = cancel_after_guard
    runtime.offer_odometry(odometry_at_goal())
    runtime._tick_once(.4)
    runtime.tasks.observe = original
    assert not any(event.outcome == 'GOAL_REACHED' for event in events)
    assert_cancel_brakes_then_finishes(runtime, session, events)


def test_input_thread_enqueues_cancel_before_observation_commit():
    """实际输入线程在确定性的计算交接点入队，不用压力循环猜测调度。"""
    import threading
    runtime, session, events = arriving_runtime()
    entered, continue_observe, enqueued = threading.Event(), threading.Event(), threading.Event()
    errors = []
    original = runtime.tasks.observe
    def observe_paused(packet, now_s, **kwargs):
        entered.set()
        assert continue_observe.wait(2.)
        return original(packet, now_s, **kwargs)
    def tick():
        try:
            runtime._tick_once(.4)
        except BaseException as exc:
            errors.append(exc)
    def cancel():
        runtime.request_cancel('a')
        enqueued.set()
    runtime.tasks.observe = observe_paused
    runtime.offer_odometry(odometry_at_goal())
    tick_thread = threading.Thread(target=tick, daemon=True)
    input_thread = threading.Thread(target=cancel, daemon=True)
    tick_thread.start()
    try:
        assert entered.wait(2.)
        input_thread.start()
        assert enqueued.wait(2.), '任务观测计算不应占用输入入队锁'
    finally:
        continue_observe.set()
        tick_thread.join(2.)
        if input_thread.ident is not None:
            input_thread.join(2.)
        runtime.tasks.observe = original
    assert not tick_thread.is_alive() and not input_thread.is_alive() and not errors
    assert not any(event.outcome == 'GOAL_REACHED' for event in events)
    assert_cancel_brakes_then_finishes(runtime, session, events)


def test_terminal_event_delivery_allows_reentrant_and_other_thread_input():
    """终态先提交，外部sink再运行；sink可重入也可等待另一个输入线程。"""
    import threading
    runtime, session, events = arriving_runtime()
    enqueue_completed = threading.Event()
    worker = None
    def sink(event):
        nonlocal worker
        events.append(event)
        if event.outcome == 'GOAL_REACHED':
            assert runtime.tasks.active is event
            runtime.request_cancel('a')  # 同线程重入必须排在该终态之后。
            worker = threading.Thread(target=lambda: (runtime.request_cancel('a'), enqueue_completed.set()), daemon=True)
            worker.start()
            assert enqueue_completed.wait(2.), '外部sink不能在输入锁内执行'
    runtime.event_sink = sink
    runtime.offer_odometry(odometry_at_goal())
    try:
        runtime._tick_once(.4)
    finally:
        if worker is not None:
            worker.join(2.)
    assert enqueue_completed.is_set()
    runtime._tick_once(.45)
    assert runtime.tasks.active.outcome == 'GOAL_REACHED'
    assert not session.canceling  # 终态提交以后的旧取消不撤销已提交结果。
