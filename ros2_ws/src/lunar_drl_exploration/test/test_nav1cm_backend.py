"""动作身份和停稳语义：传输替身仅模拟 ROS action 边界。"""
from types import SimpleNamespace as NS
from threading import Lock
import pytest

class Future:
    def __init__(self, value=None): self.value=value
    def done(self): return self.value is not None
    def result(self): return self.value

class Handle:
    accepted=True
    goal_id=NS(uuid=bytes(range(16)))
    def __init__(self): self.result=Future(); self.cancel=Future()
    def get_result_async(self): return self.result
    def cancel_goal_async(self): return self.cancel

class Action:
    def __init__(self): self.handle=Handle(); self.callbacks=[]
    def server_is_ready(self): return True
    def send_goal_async(self, request, feedback_callback):
        self.callbacks.append(feedback_callback)
        return Future(self.handle)


def adapter():
    from lunar_drl_exploration.nav1cm_navigation import Nav1cmNavigationAdapter
    a=Nav1cmNavigationAdapter.__new__(Nav1cmNavigationAdapter)
    a.action=Action();a._action_type=NS(Goal=lambda **k:NS(**k))
    a._execution_lock=Lock();a._goal_serial=0;a._send_future=None
    a._handle=None;a._result_future=None;a.sent_goal=None
    a._execution_route=None;a._feedback=None;a._execution_feedback_wall_s=0.
    a._cancel=False;a._cancel_future=None;a.velocity=(0.,0.)
    a.pose_received_monotonic=0.;a.last_goal_execution=None
    a.last_goal_result=None;a.last_goal_execution_serial=None
    return a


def test_invalid_goal_is_ordinary_result_and_keeps_current_attempt():
    a=adapter(); serial=a.begin_goal((1.,2.,0.));a.poll_goal()
    with pytest.raises(ValueError): a.begin_goal((float('nan'),0.,0.))
    assert a._goal_serial==serial and a.sent_goal==(1.,2.,0.) and a.inflight
    a.action.handle.result.value=NS(status=6,result=NS(outcome=2,reason_code='INVALID_GOAL',last_segment_revision=0))
    assert a.poll_goal().outcome==2


def test_current_uuid_feedback_only_and_cancel_waits_for_measured_stop():
    a=adapter();a.begin_goal((1.,2.,0.));a.poll_goal()
    callback=a.action.callbacks[0]
    feedback=NS(session_state=1,planning_cycle=7,active_segment_revision=4,reason_code='EXECUTING')
    callback(NS(goal_id=NS(uuid=b'wrong'),feedback=feedback))
    assert a.execution_state()['feedback'] is None
    callback(NS(goal_id=a.action.handle.goal_id,feedback=feedback))
    assert a.execution_state()['feedback']['planning_cycle']==7
    assert a.execution_state()['session_id']==bytes(range(16)).hex()
    a.cancel_goal();a.velocity=(.1,0.)
    a.action.handle.result.value=NS(status=5,result=NS(outcome=5,reason_code='CANCELED',last_segment_revision=4))
    assert a.poll_goal() is None and a.inflight
    a.velocity=(0.,0.)
    import time
    a.pose_received_monotonic=time.monotonic()
    result=a.poll_goal()
    assert result.reason_code=='CANCELED' and not a.inflight
    assert a.last_goal_execution_serial==1
    callback(NS(goal_id=a.action.handle.goal_id,feedback=feedback))
    assert a.execution_state()['feedback'] is None


def test_server_rejection_and_internal_error_are_integrity_failures():
    from lunar_drl_exploration.ros_messages import InfrastructureError
    a=adapter();a.action.handle.accepted=False;a.begin_goal((1.,2.,0.))
    with pytest.raises(InfrastructureError,match='rejected'):a.poll_goal()
    a=adapter();a.begin_goal((1.,2.,0.));a.poll_goal()
    a.action.handle.result.value=NS(status=6,result=NS(outcome=6,reason_code='INTERNAL_ERROR',last_segment_revision=0))
    with pytest.raises(InfrastructureError):a.poll_goal()