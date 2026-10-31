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


def test_brake_diagnostic_keeps_real_rejection_and_snapshot():
    module = runtime_api()
    candidate = NS(source='brake', mode='spin', route_version=7)
    result = NS(status='BRAKING', reference=NS(route_version=8), events=(
        {'kind':'LOCAL_REJECTED','reason':'COLLISION'},
        {'kind':'EXECUTION_TICK','buffer_remaining_s':0.2,'local_busy':True}))
    packet = NS(candidate=candidate, result=result, generation=3)
    record = module.brake_diagnostic(packet, 12.5, NS(source_phase_s=0.4))
    assert record['events'] == [{'kind':'LOCAL_REJECTED','reason':'COLLISION'}]
    assert record['planning_route_version'] == 8 and record['candidate_route_version'] == 7
    assert record['buffer_remaining_s'] == .2 and record['source_phase_s'] == .4
    assert record['local_busy'] is True and record['cycle_s'] == 12.5


def test_brake_diagnostic_marks_missing_events_and_nonfinite_values():
    module = runtime_api()
    packet = NS(candidate=NS(source='brake', mode='drive', route_version=1), generation=2,
                result=NS(status='BRAKING', reference=None, events=(
                    {'kind':'EXECUTION_TICK','buffer_remaining_s':float('nan'),'local_busy':False},)))
    record = module.brake_diagnostic(packet, 2., NS(source_phase_s=float('inf')))
    assert record['events'] == [] and record['event_state'] == 'none'
    assert record['buffer_remaining_s'] is None and record['source_phase_s'] is None


def test_published_brake_logs_reference_rejection_once_and_diagnostics_cannot_break_publish(monkeypatch):
    import nav1cm.async_coordinator as coordinator
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
    runtime, session, published, logs = build_runtime()
    session.coordinator.runner = ImmediateGlobal()
    runtime.offer_goal(pose(0, 3)); runtime._tick_once(0.)
    def reject(*args, **kwargs): raise ValueError('reference proof')
    monkeypatch.setattr(coordinator, 'extract_reference', reject)
    before = len(published)
    packet = runtime._tick_once(.05)
    assert packet.candidate.source == 'brake' and len(published) == before + 1
    records = [json.loads(detail) for kind, detail in logs if kind == 'EXECUTION_BRAKE']
    assert len(records) == 2
    assert {'kind':'REFERENCE_REJECTED','reason':'reference proof'} in records[-1]['events']
    monkeypatch.setattr(session, 'publication_current', lambda packet: False)
    before = len(published); packet = runtime._tick_once(.1)
    assert packet.candidate.source == 'brake' and len(published) == before
    assert len([1 for kind, _ in logs if kind == 'EXECUTION_BRAKE']) == 2
    monkeypatch.setattr(session, 'publication_current', lambda packet: True)
    monkeypatch.setattr(runtime_api(), 'brake_diagnostic', lambda *a: (_ for _ in ()).throw(ValueError('diagnostic failure')))
    before = len(published); packet = runtime._tick_once(.15)
    assert packet.candidate.source == 'brake' and len(published) == before + 1
    assert len([1 for kind, _ in logs if kind == 'EXECUTION_BRAKE']) == 2


def test_brake_diagnostic_includes_worker_and_submit_errors():
    module = runtime_api()
    errors = ({'kind':'LOCAL_SUBMIT_ERROR','reason':'submit'}, {'kind':'LOCAL_WORKER_START_ERROR','reason':'start'})
    packet = NS(candidate=NS(source='brake', mode='drive', route_version=1), generation=2,
                result=NS(status='WAITING_GLOBAL', reference=None, events=errors))
    assert module.brake_diagnostic(packet, 1., NS())['events'] == list(errors)


def test_direct_control_brakes_from_measured_state_and_missing_input_zero():
    from nav1cm.contracts import VehicleState
    runtime, session, published, logs = build_runtime()
    commands=[]
    runtime.configure_direct_control(lambda v,w:commands.append((v,w)),command_hold_s=1.2)
    runtime.enable_execution_contract()
    assert not runtime.tasks._feedback_required
    assert not runtime.execution_contract_enabled
    session.offer_state(VehicleState(0,0,0,.1,0))
    runtime.tasks.submit('direct-a',(0,3,0),0.)
    packet=runtime._tick_once(0.)
    assert packet.result.action is None
    assert commands[-1] == (0.,0.)  # brake候选只发布零目标，实测速度仍保留
    assert session.state.v == .1  # 模型积分不得推进在线实测状态。
    runtime.request_cancel(runtime.tasks.active.task_id)
    runtime._tick_once(.05)
    assert commands[-1] == (0.,0.)  # brake候选只发布零目标，实测速度仍保留
    session.offer_state(None)
    runtime._tick_once(.1)
    assert commands[-1] == (0.,0.)


def test_direct_control_old_generation_never_emits_movement():
    from nav1cm.contracts import VehicleState
    runtime, session, published, logs = build_runtime()
    commands=[]
    runtime.configure_direct_control(lambda v,w:commands.append((v,w)),command_hold_s=1.2)
    session.offer_state(VehicleState(0,0,0,.1,0))
    runtime.tasks.submit('direct-a',(0,3,0),0.)
    original=session.tick
    def racing_tick(now, **kwargs):
        packet=original(now, **kwargs)
        runtime.offer_goal(pose(0,-3))
        return packet
    session.tick=racing_tick
    runtime._tick_once(0.)
    assert commands == [(0.,0.)]


def test_direct_control_arrives_and_cancels_with_body_feedback_only():
    from nav1cm.contracts import VehicleState
    runtime, session, published, logs = build_runtime()
    commands=[]
    runtime.configure_direct_control(lambda v,w:commands.append((v,w)),command_hold_s=1.2)
    runtime.tasks.submit('direct-a',(0,0,0),0.)
    feedback=NS(header=NS(frame_id='map',stamp=NS(sec=1,nanosec=0)),pose=NS(pose=pose().pose),
        twist=NS(twist=NS(linear=NS(x=0.),angular=NS(z=0.))))
    runtime.offer_odometry(feedback)
    runtime._tick_once(0.)
    feedback.header.stamp.sec=2
    runtime.offer_odometry(feedback)
    runtime._tick_once(.4)
    assert runtime.tasks.active.outcome == 'GOAL_REACHED'
    runtime.tasks.submit('direct-a',(0,3,0),0.)
    runtime._tick_once(.5)
    runtime.request_cancel(runtime.tasks.active.task_id)
    feedback.header.stamp.sec=3
    runtime.offer_odometry(feedback)
    runtime._tick_once(.6)
    assert runtime.tasks.active.outcome == 'CANCELED'
    assert all(command == (0.,0.) for command in commands)


def test_direct_rejects_map_replacement_after_tick():
    from nav1cm.contracts import VehicleState, WorldSnapshot, CellRect
    runtime, session, published, logs=build_runtime()
    commands=[];runtime.configure_direct_control(lambda v,w:commands.append((v,w)),command_hold_s=1.2)
    session.offer_state(VehicleState(0,0,0,.1,0))
    runtime.tasks.submit('direct-map',(0,3,0),0.)
    tick=session.tick
    def changed(now, **kwargs):
        packet=tick(now)
        runtime._world=WorldSnapshot(2,(CellRect(-20,-20,20,20),),session.world.known_free)
        return packet
    session.tick=changed
    runtime._tick_once(0.)
    assert commands[-1] == (0.,0.)


def test_direct_duplicate_source_cannot_complete_task():
    runtime, session, published, logs=build_runtime()
    runtime.configure_direct_control(lambda *args:None,command_hold_s=1.2)
    runtime.tasks.submit('direct-duplicate',(0,0,0),0.)
    feedback=NS(header=NS(frame_id='map',stamp=NS(sec=0,nanosec=900000)),pose=NS(pose=pose().pose),
        twist=NS(twist=NS(linear=NS(x=0.),angular=NS(z=0.))))
    runtime.offer_odometry(feedback);runtime._tick_once(0.)
    runtime.offer_odometry(feedback);runtime._tick_once(.32)
    assert runtime.tasks.active.outcome is None


@pytest.mark.parametrize('changed',[False,True])
def test_direct_publication_latest_map_window_and_body_state(changed):
    from nav1cm.contracts import VehicleState, WorldSnapshot, CellRect
    runtime,session,published,logs=build_runtime()
    commands=[];runtime.configure_direct_control(lambda v,w:commands.append((v,w)),command_hold_s=1.2)
    session.offer_state(VehicleState(0,0,0,.1,0));runtime.tasks.submit('race',(0,3,0),0.)
    tick=session.tick
    def raced(now, **kwargs):
        packet=tick(now)
        runtime._world=WorldSnapshot(2,session.world.blocked+(CellRect(8000,8000,8010,8010),),session.world.known_free)
        if changed:
            feedback=NS(header=NS(frame_id='map',stamp=NS(sec=1,nanosec=0)),pose=NS(pose=pose(.3).pose),
                twist=NS(twist=NS(linear=NS(x=.1),angular=NS(z=0.))))
            runtime.offer_odometry(feedback)
        return packet
    session.tick=raced
    runtime._tick_once(0.)
    assert commands[-1] == (0.,0.)  # brake候选只发布零目标，实测速度仍保留


@pytest.mark.parametrize('hold',[0.,-1.,float('nan'),float('inf'),True])
def test_direct_requires_explicit_valid_hold(hold):
    runtime,*_=build_runtime()
    with pytest.raises(ValueError,match='command_hold_s'):
        runtime.configure_direct_control(lambda *args:None,command_hold_s=hold)


def test_direct_source_time_must_cover_arrival_hold():
    runtime,session,*_=build_runtime()
    runtime.configure_direct_control(lambda *args:None,command_hold_s=1.2)
    runtime.tasks.submit('source-time',(0,0,0),0.)
    feedback=NS(header=NS(frame_id='map',stamp=NS(sec=1,nanosec=0)),pose=NS(pose=pose().pose),
        twist=NS(twist=NS(linear=NS(x=0.),angular=NS(z=0.))))
    runtime.offer_odometry(feedback);runtime._tick_once(0.)
    feedback.header.stamp.nanosec=1
    runtime.offer_odometry(feedback);runtime._tick_once(.4)
    assert runtime.tasks.active.outcome is None
    feedback.header.stamp.nanosec=400000000
    runtime.offer_odometry(feedback);runtime._tick_once(.8)
    assert runtime.tasks.active.outcome=='GOAL_REACHED'


def test_direct_safe_changed_window_recertifies_once_without_braking():
    from nav1cm.contracts import VehicleState,WorldSnapshot,CellRect
    runtime,session,*_=build_runtime();commands=[]
    runtime.configure_direct_control(lambda v,w:commands.append((v,w)),command_hold_s=1.2)
    session.offer_state(VehicleState(0,0,0,.1,0));runtime.tasks.submit('safe-map',(0,3,0),0.)
    proofs=[]
    original_proof=runtime._publication_map_proof
    def check(packet,world):
        assert packet.result.action is None and packet.result.action_certificate is None
        accepted=original_proof(packet,world);proofs.append(accepted[1]);return accepted
    runtime._publication_map_proof=check
    tick=session.tick
    def raced(now, **kwargs):
        packet=tick(now)
        # 窗口内变化，但远离整车、短动作、有限持令和停车尾。
        runtime._world=WorldSnapshot(2,session.world.blocked+(CellRect(190,190,200,200),),session.world.known_free)
        return packet
    session.tick=raced;runtime._tick_once(0.)
    assert proofs==[True]
    assert commands[-1] == (0.,0.)  # brake候选只发布零目标，实测速度仍保留

def test_goal_replacement_retains_actual_last_command_and_zero_publish_resets(monkeypatch):
    import nav1cm.execution as execution
    from nav1cm.contracts import VehicleState
    runtime,session,*_=build_runtime();commands=[];seen=[]
    runtime.configure_direct_control(lambda *command:commands.append(command),command_hold_s=.1)
    session.offer_state(VehicleState(0,0,0,0,0))
    runtime._publish_issued((.06,.02))
    stamp=runtime._last_issued_s
    def command(result,state,*args,**kwargs):
        seen.append(kwargs['last_command']);return (.07,.03)
    monkeypatch.setattr(execution,'certified_velocity_command',command)
    runtime.offer_goal(pose(0,3));runtime._tick_once(0.)
    assert seen[-1]==(.06,.02)
    assert runtime._last_issued_command==commands[-1]==(.07,.03)
    assert runtime._last_issued_s>=stamp
    runtime.offer_goal(pose(0,4));runtime._tick_once(.1)
    assert seen[-1]==(.07,.03)
    runtime._publish_issued((0.,0.))
    runtime.offer_goal(pose(0,5));runtime._tick_once(.2)
    assert seen[-1]==(0.,0.)



def test_direct_sampled_map_update_does_not_insert_zero_and_next_cycle_blocks():
    import numpy as np
    from nav1cm.contracts import VehicleState,WorldSnapshot,CellRect,Route,Trajectory
    from nav1cm.seeds import rollout
    runtime,session,*_=build_runtime();commands=[]
    runtime.configure_direct_control(lambda *cmd:commands.append(cmd),command_hold_s=.1)
    runtime.tasks.submit('sampled-map',(0,3,0),0.)
    session.offer_state(VehicleState(0,0,0,0,0))
    c=session.coordinator;c.route=Route(1,1,[[0,0],[0,3]],(0,3));c._policy=False
    u=np.array([[0.,.4,0.]])
    source=Trajectory(1,1,[0.,.2],rollout([0,0,0,0,0,0],u,.2),u,'spin','mpcc')
    c.buffer.install(source,0.)
    tick=session.tick
    def raced(now, **kwargs):
        packet=tick(now)
        runtime._world=WorldSnapshot(2,(CellRect(-20,-20,20,20),),session.world.known_free)
        return packet
    session.tick=raced
    runtime._tick_once(0.)
    assert commands[-1][1]>0
    session.tick=tick
    runtime._tick_once(.05)
    assert commands[-1]==(0.,0.)


def test_command_diagnostics_merge_into_existing_notice(monkeypatch):
    import nav1cm.execution as execution
    from nav1cm.contracts import VehicleState
    runtime,session,*_=build_runtime(); notices=[]
    runtime._notice=lambda kind,detail:notices.append((kind,detail))
    runtime.configure_direct_control(lambda *command:None,command_hold_s=.1)
    session.offer_state(VehicleState(0,0,0,0,0))
    def command(result,state,*args,**kwargs):
        kwargs['diagnostics'].update(target_vw=[.07,.03],command_reject_reason=None)
        return (.07,.03)
    monkeypatch.setattr(execution,'certified_velocity_command',command)
    runtime.offer_goal(pose(0,3));runtime._tick_once(0.)
    records=[json.loads(detail) for kind,detail in notices if kind=='MPCC_COMMAND']
    assert len(records)==1
    assert records[0]['target_vw']==[.07,.03]
    assert records[0]['p3_new_sample'] is False
    assert records[0]['source_identity']=={}
    assert (records[0]['v'],records[0]['w'])==(.07,.03)
