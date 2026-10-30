"""执行输入独立接收，控制线程按原接收时刻消费，溢出不得继续执行。"""
from collections import deque
from threading import Lock
from types import SimpleNamespace as S
import pytest
from lunar_pure_wheeled_controller import node as module
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode as C


def receiver(capacity=128):
    n=S(_feedback_lock=Lock(),_execution_inputs=deque(),_execution_input_capacity=capacity,
        _execution_ingress_overflow=False)
    n._receive_execution_input=lambda kind,msg:C._receive_execution_input(n,kind,msg)
    return n


def test_execution_receive_is_ordered_snapshot_only_and_does_not_restamp(monkeypatch):
    now=[10.];monkeypatch.setattr(module,'monotonic',lambda:now[0])
    n=receiver();calls=[]
    n._on_path=lambda msg,received_at=None:calls.append(('path',msg,received_at))
    n._on_execution_reference=lambda msg,received_at=None:calls.append(('reference',msg,received_at))
    ref,path=object(),object()
    C._receive_execution_reference(n,ref);now[0]=10.1;C._receive_execution_path(n,path)
    assert calls==[]
    now[0]=10.3;assert C._drain_standard_execution(n) is True
    assert calls==[('reference',ref,10.),('path',path,10.1)]
    assert C._drain_standard_execution(n) is True and len(calls)==2


def test_execution_ingress_overflow_latches_without_silent_drop(monkeypatch):
    monkeypatch.setattr(module,'monotonic',lambda:10.)
    n=receiver(2);n._on_path=lambda *a,**k:pytest.fail('overflowed stream must not be consumed')
    n._on_execution_reference=n._on_path
    C._receive_execution_path(n,object());C._receive_execution_reference(n,object())
    C._receive_execution_path(n,object())
    assert len(n._execution_inputs)==2 and n._execution_ingress_overflow
    assert C._drain_standard_execution(n) is False
    assert C._drain_standard_execution(n) is False


def test_source_expiry_rejected_before_executor_can_latch_invalid_state():
    reasons=[]
    n=S(_odometry=object(),_odometry_received_at=0.,_odometry_timeout_s=4.5,
        _active=object(),_path_received_at=0.,_path_timeout_s=.35,_standard_state=object(),
        _standard_state_ns=1_000_000_000,_last_control_ns=None,_input_is_stale=lambda *a:False,
        _standard_grid=lambda *a:object(),get_clock=lambda:S(now=lambda:S(nanoseconds=6_000_000_000)),
        _standard_unavailable=lambda reason,**kw:reasons.append(reason),
        _executor=S(request_stop=lambda *a:None,set_map=lambda *a:None,update=lambda *a,**k:pytest.fail('expired source reached executor')))
    C._tick_standard_path(n)
    assert reasons==['stale_source_feedback']


def test_path_and_reference_callbacks_receive_while_control_is_blocked(monkeypatch):
    import time
    from threading import Event
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from nav_msgs.msg import Path
    from lunar_planning_msgs.msg import PathExecutionReference
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower',
                     '-p','input_mode:=incremental_path','-p','execution_contract_enabled:=true'])
    node=C();peer=rclpy.create_node('execution_ingress_probe')
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node);executor.add_node(peer)
    entered=Event();release=Event()
    def block():
        entered.set();release.wait(3.)
    monkeypatch.setattr(node,'_tick',block)
    pubs=[peer.create_publisher(cls,node.get_parameter(topic).value,10) for cls,topic in
          ((Path,'path_topic'),(PathExecutionReference,'execution_reference_topic'))]
    try:
        end=time.monotonic()+3.
        while not entered.is_set() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        assert entered.is_set()
        end=time.monotonic()+1.
        while time.monotonic()<end:
            pubs[0].publish(Path());pubs[1].publish(PathExecutionReference())
            executor.spin_once(timeout_sec=.02)
            with node._feedback_lock:kinds={x[0] for x in node._execution_inputs}
            if kinds=={'path','reference'}:break
        assert kinds=={'path','reference'}, 'execution receive callbacks blocked by control'
        assert node._execution_offered is None
        calls=[]
        monkeypatch.setattr(node,'_on_path',lambda msg,received_at=None:calls.append(received_at))
        monkeypatch.setattr(node,'_on_execution_reference',lambda msg,received_at=None:calls.append(received_at))
        with node._feedback_lock:received=[x[2] for x in node._execution_inputs]
        release.set()
        assert node._drain_standard_execution()
        assert calls==received
    finally:
        release.set();executor.shutdown();node.destroy_node();peer.destroy_node();rclpy.shutdown()
