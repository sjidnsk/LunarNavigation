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


def test_current_uuid_feedback_only_and_cancel_consumes_confirmed_terminal():
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

@pytest.mark.parametrize('outcome,reason',[(0,'GOAL_REACHED'),(5,'CANCELED')])
def test_nav1cm_terminal_is_delivered_without_second_odometry_gate(outcome,reason):
    a=adapter();a.begin_goal((1.,2.,0.));a.poll_goal()
    a.action.handle.result.value=NS(status=4,result=NS(outcome=outcome,reason_code=reason,last_segment_revision=4))
    # 导航owner已确认终态；客户端慢帧或过期缓存不能重新否决该结论。
    a.pose_received_monotonic=0.;a.velocity=(.1,0.)
    result=a.poll_goal()
    assert result is not None and result.reason_code==reason and not a.inflight



def test_inference_consumes_nav1cm_terminal_stop_without_native_terrain():
    import ast
    from pathlib import Path
    # 执行真实tick方法，仅隔离本机缺少的地形扩展；不冒充整节点测试。
    source=Path(__file__).parents[1]/'lunar_drl_exploration/runtime.py'
    tree=ast.parse(source.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='InferenceRuntime')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
    module=ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[]))
    scope={'velocity_is_stopped':lambda value:value==(0.,0.)}
    exec(compile(module,str(source),'exec'),scope)
    a=adapter();a.begin_goal((1.,2.,0.));a.poll_goal()
    a.action.handle.result.value=NS(status=5,result=NS(outcome=5,reason_code='CANCELED_STOPPED',last_segment_revision=4))
    a.velocity=(.1,0.);a.poll_snapshot=lambda *args:None;a.require_fresh_snapshot=lambda:None
    a.processed_stamp_ns=0
    r=NS(core=object(),adapter=a,last_result=None,_stamp=-1,state='PAUSING')
    scope['tick'](r)
    assert r.state=='PAUSED' and a.terminal_stop_confirmed
