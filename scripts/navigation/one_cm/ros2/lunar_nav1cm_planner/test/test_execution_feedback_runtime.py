"""匹配跟踪故障经实际Runtime队列进入Action任务；不启动ROS。"""
from pathlib import Path
import sys,time
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).parent))
from test_runtime import build_runtime
from nav1cm.contracts import VehicleState
from nav1cm.execution_contract import ExecutionReference,ExecutionStatus,BRAKE,FOLLOW


def test_tracking_failure_retires_planning_then_independently_stops_and_finishes():
    runtime,session,published,_=build_runtime();events=[];runtime.event_sink=events.append
    runtime.enable_execution_contract(timeout_s=2.)
    t=time.monotonic();task='a'*32
    runtime.offer_managed_goal(task,S(target_x_m=0.,target_y_m=3.,target_yaw_rad=0.,has_target_yaw=True))
    runtime._tick_once(t)
    ref=ExecutionReference(task,1,'d'*64,FOLLOW,1)
    runtime.register_execution_reference(ref)
    session.offer_state(VehicleState(0.,0.,0.,.02,0.))
    runtime.offer_tracking_status(ExecutionStatus(task,1,'d'*64,FOLLOW,6,True,2,'TRACKING_STALLED'))
    runtime._tick_once(t+.1)
    assert session.goal is None
    assert runtime.tasks.active.phase=='ERROR_STOPPING'
    assert runtime.tasks.active.outcome is None
    assert runtime.execution_context[0:2]==(task,1)
    assert runtime.execution_context[2]==BRAKE
    session.offer_state(VehicleState(0.,0.,0.,0.,0.));runtime._odometry_sequence+=1
    runtime._tick_once(t+.2)
    assert runtime.tasks.active.outcome=='INTERNAL_ERROR'
    assert runtime.tasks.active.reason_code=='TRACKING_STALLED'
    assert any(e.outcome=='INTERNAL_ERROR' for e in events)
