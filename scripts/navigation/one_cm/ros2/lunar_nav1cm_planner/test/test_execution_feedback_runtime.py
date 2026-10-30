"""匹配跟踪故障经实际Runtime队列进入Action任务；不启动ROS。"""
from pathlib import Path
import sys,time
import pytest
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


def test_pose_input_recovery_needs_two_seconds_and_source_time_progress(monkeypatch):
    runtime,session,published,logs=build_runtime()
    runtime.configure_pose_input(timeout_s=4.5)
    module=__import__('lunar_nav1cm_planner.runtime',fromlist=['state_from_odometry'])
    state=VehicleState(0.,0.,0.,0.,0.)
    monkeypatch.setattr(module,'state_from_odometry',lambda *a:state)
    clock=[10_000_000_000];runtime.ros_now_ns=lambda:clock[0]
    message=S(header=S(stamp=S(sec=8,nanosec=0)))
    runtime._apply_pose_input(message,0.,0.,S())
    assert session.state is None
    runtime._apply_pose_input(message,2.,2.,S())
    assert session.state is None, '重复源时间不能作为定位恢复'
    clock[0]=12_000_000_000;message.header.stamp.sec=10
    runtime._apply_pose_input(message,2.1,2.1,S())
    assert session.state is state
    session.offer_goal(__import__('nav1cm.ros_conversion',fromlist=['PoseGoal']).PoseGoal(0.,3.,0.))
    generation=session.accepted_generation;revision=session.coordinator.goal_revision
    clock[0]=20_000_000_000
    runtime._apply_pose_input(message,2.1,10.,S())
    assert session.state is None and session.goal.y==3. and session.accepted_generation==generation
    assert session.coordinator.goal_revision>revision


def test_local_recovery_hold_keeps_only_fresh_matched_wait_heartbeat_alive():
    from nav1cm.execution_contract import ExecutionStatus,ExecutionReference,FOLLOW,execution_session_id
    from nav1cm.ros_conversion import PoseGoal
    runtime,session,published,logs=build_runtime();task=runtime.tasks
    task.configure_execution_feedback(required=True,timeout_s=2.)
    task.submit('a',PoseGoal(0.,3.,0.),0.)
    ref=ExecutionReference(execution_session_id('a',session.accepted_generation),1,'a'*64,FOLLOW,1)
    task.register_execution_reference(ref,0.)
    wait=ExecutionStatus(ref.session_id,1,ref.path_digest,FOLLOW,0,False,1,'WAITING_FOR_FRESH_REFERENCE')
    assert task.offer_execution_status(wait,10.)
    assert task.check_execution_fault(10.1,pose_waiting=True) is None
    assert task.check_execution_fault(11.9,pose_waiting=False) is None
    assert task.check_execution_fault(12.1,pose_waiting=True)=='EXECUTION_FEEDBACK_TIMEOUT'
    assert task._execution_last_accepted is None


def test_pose_source_rollback_during_recovery_never_restores_motion(monkeypatch):
    runtime,session,published,logs=build_runtime();runtime.configure_pose_input(4.5)
    module=__import__('lunar_nav1cm_planner.runtime',fromlist=['state_from_odometry'])
    monkeypatch.setattr(module,'state_from_odometry',lambda *a:VehicleState(0.,0.,0.,0.,0.))
    clock=[100_000_000_000];runtime.ros_now_ns=lambda:clock[0]
    def odom(sec,nano=0):return S(header=S(stamp=S(sec=sec,nanosec=nano)))
    runtime._apply_pose_input(odom(98),0.,0.,S())
    runtime._apply_pose_input(odom(98,500_000_000),1.,1.,S())
    clock[0]=100_100_000_000
    runtime._apply_pose_input(odom(98,200_000_000),2.1,2.1,S())
    assert session.state is None and runtime._pose_integrity_fault=='POSE_SOURCE_ROLLBACK'


@pytest.mark.parametrize("cancel_during_wait",[False,True])
def test_actual_runtime_tick_preserves_target_across_loss_and_stable_recovery(monkeypatch,cancel_during_wait):
    runtime,session,published,logs=build_runtime();runtime.configure_pose_input(4.5)
    module=__import__('lunar_nav1cm_planner.runtime',fromlist=['state_from_odometry'])
    state=VehicleState(0.,0.,0.,0.,0.)
    monkeypatch.setattr(module,'state_from_odometry',lambda *a:state)
    mono=[10.];ros=[100_000_000_000]
    monkeypatch.setattr(module.time,'monotonic',lambda:mono[0]);runtime.ros_now_ns=lambda:ros[0]
    def feed(at,sec):
        mono[0]=at;ros[0]=int((at+90.)*1e9)
        runtime.offer_odometry(S(header=S(stamp=S(sec=sec,nanosec=0))))
        return runtime._tick_once(at)
    task='d'*32
    runtime.offer_managed_goal(task,S(target_x_m=0.,target_y_m=3.,has_target_yaw=True,target_yaw_rad=0.))
    feed(10.,98)
    assert runtime.tasks.active.reason_code=='WAITING_FOR_POSE'
    feed(12.1,100)
    generation=session.accepted_generation;goal=session.goal
    assert session.state is state and runtime.tasks.active.outcome is None
    mono[0]=20.;ros[0]=110_000_000_000;packet=runtime._tick_once(20.)
    assert 'state' in packet.waiting and session.goal is goal
    assert runtime.tasks.active.reason_code=='WAITING_FOR_POSE' and runtime.tasks.active.outcome is None
    if cancel_during_wait:
        runtime.request_cancel(task)
        mono[0]=20.1;runtime._tick_once(20.1)
    feed(22.,110)
    assert session.state is None
    feed(24.1,112)
    assert session.state is state and runtime.tasks.active.task_id==task
    if cancel_during_wait:
        assert session.goal is None and runtime.tasks.active.outcome=='CANCELED'
    else:
        assert session.goal is goal and session.accepted_generation==generation
        assert runtime.tasks.active.outcome is None


@pytest.mark.parametrize('delay,gap,expected',[(2.5,0.,True),(3.8,0.,True),(2.5,2.5,True),(4.5,0.,True),(4.5001,0.,True),(10.,4.5001,False)])
def test_preflight_pose_time_gate_matches_runtime(delay,gap,expected):
    import importlib.util
    path=next(p/'scripts/simulation/p3_input_timing.py' for p in Path(__file__).resolve().parents if (p/'scripts/simulation/p3_input_timing.py').exists())
    spec=importlib.util.spec_from_file_location('p3_runtime_time_parity',path)
    timing=importlib.util.module_from_spec(spec);spec.loader.exec_module(timing)
    runtime,_,_,_=build_runtime();runtime.configure_pose_input(4.5)
    source=int((100-delay)*1e9);now=100_000_000_000+int(gap*1e9)
    runtime.ros_now_ns=lambda:now
    message=S(header=S(stamp=S(sec=source//10**9,nanosec=source%10**9)))
    sample=timing.record_sample(None,source,100_000_000_000,10.)
    actual=runtime._pose_source_current(message,10.,10.+gap)[0]
    report=timing.runtime_input_health('odom',sample,now,10.+gap,odometry_timeout_s=4.5,p3_map_timeout_s=6.)
    assert actual==report['ready']==expected


def test_delayed_pose_continuity_and_duplicate_stream_timeout(monkeypatch):
    runtime,session,_,_=build_runtime();runtime.configure_pose_input(4.5)
    module=__import__('lunar_nav1cm_planner.runtime',fromlist=['state_from_odometry'])
    monkeypatch.setattr(module,'state_from_odometry',lambda *a:VehicleState(0.,0.,0.,0.,0.))
    clock=[100_000_000_000];runtime.ros_now_ns=lambda:clock[0]
    message=S(header=S(stamp=S(sec=90,nanosec=0)))
    assert runtime._pose_source_current(message,10.,10.)[0]
    runtime._apply_pose_input(message,10.,10.,S())
    clock[0]=103_000_000_000;message.header.stamp.sec=93
    runtime._apply_pose_input(message,13.,13.,S())
    assert session.state is not None
    clock[0]=108_000_000_000
    runtime._apply_pose_input(message,18.,18.,S())
    assert session.state is None, '持续接收重复旧帧不能维持定位有效'
