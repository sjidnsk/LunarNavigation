"""Ordered standard Path behavior, independent of ROS transport."""
from dataclasses import replace
import importlib
import math
import numpy as np
import pytest
from lunar_pure_wheeled_controller.mppi import MppiPolicy
from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
from lunar_pure_wheeled_controller.mppi_map import GridMap
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState


def follower(**policy):
    try:
        cls=importlib.import_module('lunar_pure_wheeled_controller.standard_path_follower').StandardPathFollower
    except ModuleNotFoundError:
        cls=MppiPathFollower
    e=cls(replace(TrackingPolicy(),goal_position_tolerance_m=.1,goal_yaw_tolerance_rad=.05,
        stopped_angular_radps=.01,execution_clearance_m=.001,control_response_time_s=.001,**policy),
        MppiPolicy(batch_size=12,time_steps=12,nominal_response_time_s=0.))
    assert hasattr(e,'set_standard_path'), 'standard Path must have an independent ordered entry point'
    e.set_map(GridMap(np.ones((100,100),np.uint8),.1,(-5.,-5.)))
    return e


def tick(e,s,dt=.05):
    r=e.update(s,dt)
    e.note_command(r.command.linear_x_mps,r.command.angular_z_radps)
    return r


@pytest.mark.parametrize('yaw,sign',[(0.,1),(math.pi,-1)])
def test_rolling_updates_preserve_real_progress_and_history(yaw,sign):
    e=follower();s=TrackingState(0.,0.,yaw)
    for i in range(80):
        assert e.set_standard_path(((s.x_m,0.,yaw),(s.x_m+1.,0.,yaw)),identity=i)
        history=[(t,u.copy()) for t,u in e._command_history]
        r=tick(e,s)
        assert r.phase!='BRAKING'
        if history:assert any(t==history[-1][0] and np.array_equal(u,history[-1][1]) for t,u in e._command_history)
        v=r.command.linear_x_mps;w=r.command.angular_z_radps
        assert v*sign>=-1e-9
        s=TrackingState(s.x_m+v*math.cos(s.yaw_rad)*.05,0.,yaw,v,w)
    assert s.x_m>.2
    assert e.progress_m>.15


def test_identical_geometry_new_identity_does_not_reset_or_brake():
    e=follower();path=((0.,0.,0.),(2.,0.,0.));e.set_standard_path(path,1)
    tick(e,TrackingState(.2,0.,0.,.1));e.note_command(.1,0.)
    progress=e.progress_m;history=list(e._command_history);e.no_progress_s=.4
    e.set_standard_path(path,2)
    assert e.progress_m==progress and getattr(e,'no_progress_s',0.)==.4
    assert len(e._command_history)==len(history)
    assert e.phase!='BRAKING'


def test_rolling_reference_does_not_add_a_tracker_task_timeout():
    e=follower(no_progress_timeout_s=.3)
    for i in range(12):
        e.set_standard_path(((0.,0.,0.),(1.+i*.001,0.,0.)),i)
        r=tick(e,TrackingState(0.,0.,0.))
    assert e.failure is None and getattr(e,'no_progress_s',0.)==0.


@pytest.mark.parametrize('target',[math.pi/2,-math.pi+.03])
def test_repeated_xy_spin_survives_rolling_trim(target):
    initial=0. if target>0 else math.pi-.03
    e=follower();e.set_standard_path(((0.,0.,initial),(0.,0.,target),(0.,1.,target)))
    r=tick(e,TrackingState(0.,0.,initial))
    assert r.command.linear_x_mps==0. and r.command.angular_z_radps>0.
    e.set_standard_path(((0.,0.,target),(0.,1.,target)),2)
    r=tick(e,TrackingState(0.,0.,initial))
    assert r.command.linear_x_mps==0. and r.command.angular_z_radps>0.


def test_corner_requires_body_alignment_before_next_leg():
    e=follower();e.set_standard_path(((0.,0.,0.),(1.,0.,math.pi/2),(1.,1.,math.pi/2)))
    r=tick(e,TrackingState(1.,0.,0.))
    assert r.command.linear_x_mps==0. and r.command.angular_z_radps>0.
    assert not r.command.complete


def test_local_endpoint_waits_and_future_path_can_continue():
    e=follower();e.set_standard_path(((0.,0.,0.),(.5,0.,0.)))
    r=tick(e,TrackingState(.5,0.,0.))
    assert r.phase=='WAITING' and not r.command.complete
    e.set_standard_path(((.5,0.,0.),(1.5,0.,0.)))
    assert tick(e,TrackingState(.5,0.,0.)).command.linear_x_mps>0.


@pytest.mark.parametrize('intent',['empty','request','invalid'])
def test_stop_intent_preserves_history_and_bounds_velocity_decay(intent):
    e=follower();e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    e.note_command(.2,0.);s=TrackingState(0.,0.,0.,.2)
    previous=.2
    for i in range(9):
        if intent=='empty':e.set_standard_path(())
        elif intent=='invalid':e.set_standard_path(((float('nan'),0.,0.),))
        else:e.request_stop('test')
        assert len(e._command_history)>=1
        r=tick(e,s);v=r.command.linear_x_mps
        assert 0.<=previous-v<=.02500001
        s=TrackingState(s.x_m+v*.05,0.,0.,v);previous=v
    assert previous==pytest.approx(0.)
    assert r.phase=='WAITING' and not r.command.complete


def test_discontinuity_brakes_instead_of_switching_direction_at_speed():
    e=follower();e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    tick(e,TrackingState(0.,0.,0.,.2));e.note_command(.2,0.)
    e.set_standard_path(((0.,0.,0.),(-2.,0.,0.)))
    r=tick(e,TrackingState(0.,0.,0.,.2))
    assert r.phase=='BRAKING' and 0.<r.command.linear_x_mps<.2


def test_self_crossing_does_not_project_onto_remote_future_leg():
    e=follower();e.set_standard_path(((0.,0.,0.),(2.,0.,0.),(2.,2.,math.pi/2),
        (0.,2.,math.pi),(0.,0.,-math.pi/2),(0.,-2.,-math.pi/2)))
    r=tick(e,TrackingState(0.,0.,0.))
    assert r.progress_m<.1 and r.command.linear_x_mps>0.


def test_after_corner_turn_physical_progress_uses_the_next_leg():
    e=follower(no_progress_timeout_s=.2)
    e.set_standard_path(((0.,0.,0.),(1.,0.,math.pi/2),(1.,1.,math.pi/2)))
    tick(e,TrackingState(.9,0.,0.))
    tick(e,TrackingState(1.,0.,math.pi/2))
    for i in range(12):
        r=tick(e,TrackingState(1.,.04*i,math.pi/2,.1))
    assert e.failure is None and r.command.linear_x_mps>0.


def test_direction_change_after_stopping_can_make_physical_reverse_progress():
    e=follower(no_progress_timeout_s=.2)
    e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    tick(e,TrackingState(0.,0.,0.,.2));e.note_command(.2,0.)
    e.set_standard_path(((0.,0.,0.),(-2.,0.,0.)))
    tick(e,TrackingState(0.,0.,0.,.2));tick(e,TrackingState(0.,0.,0.))
    for i in range(12):r=tick(e,TrackingState(-i*.04,0.,0.,-.1))
    assert e.failure is None and r.command.linear_x_mps<0.


def test_same_geometry_changed_body_yaw_requires_braking():
    e=follower();e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    tick(e,TrackingState(0.,0.,0.,.2));e.note_command(.2,0.)
    e.set_standard_path(((0.,0.,.4),(2.,0.,.4)))
    r=tick(e,TrackingState(0.,0.,0.,.2))
    assert r.phase=='BRAKING' and 0.<r.command.linear_x_mps<.2


def test_local_exhaustion_has_bounded_stop_before_waiting():
    e=follower();e.set_standard_path(((0.,0.,0.),(.5,0.,0.)))
    e.note_command(.2,0.)
    r=tick(e,TrackingState(.5,0.,0.,.2))
    assert r.phase=='BRAKING' and r.command.linear_x_mps==pytest.approx(.175)


def test_rolling_update_retaining_executed_prefix_does_not_revisit_corner():
    e=follower();path=((0.,0.,0.),(1.,0.,math.pi/2),(1.,2.,math.pi/2))
    e.set_standard_path(path);tick(e,TrackingState(1.,0.,math.pi/2))
    tick(e,TrackingState(1.,.4,math.pi/2,.1));e.note_command(.1,0.)
    e.set_standard_path(path[:-1]+((1.,2.1,math.pi/2),))
    r=tick(e,TrackingState(1.,.4,math.pi/2,.1))
    assert r.phase=='TRACKING' and r.command.linear_x_mps>0.


def test_dense_rolling_path_preserves_cursor_beyond_initial_projection_window():
    e=follower();path=tuple((i*.1,0.,0.) for i in range(41))
    e.set_standard_path(path)
    for x in np.linspace(0.,2.,41):
        tick(e,TrackingState(float(x),0.,0.,.1))
    assert e._local_progress==pytest.approx(2.)
    before=e.progress_m
    e.set_standard_path(path+((4.1,0.,0.),))
    r=tick(e,TrackingState(2.,0.,0.,.1))
    assert e._local_progress==pytest.approx(2.)
    assert e.progress_m>=before
    assert r.phase=='TRACKING' and r.command.linear_x_mps>0.
    assert abs(r.heading_error_rad)<.05


def test_latest_active_geometry_cancels_older_pending_reference():
    e=follower();a=((0.,0.,0.),(2.,0.,0.));b=((0.,0.,0.),(-2.,0.,0.))
    e.set_standard_path(a);tick(e,TrackingState(.2,0.,0.,.1))
    e.no_progress_s=.4
    progress=e.progress_m
    history=[(t,u.copy()) for t,u in e._command_history]
    e.set_standard_path(b,2);e.set_standard_path(a,3)
    assert e.progress_m==progress and getattr(e,'no_progress_s',0.)==.4
    assert len(e._command_history)==len(history)
    assert all(t==old_t and np.array_equal(u,old_u) for (t,u),(old_t,old_u) in zip(e._command_history,history))
    r=tick(e,TrackingState(.2,0.,0.,.1))
    assert e.reference.points==a
    assert r.phase=='TRACKING' and r.command.linear_x_mps>0.
    assert e.progress_m>=progress and getattr(e,'no_progress_s',0.)>=.4


def test_dense_spin_rolling_makes_physical_progress():
    e=follower();s=TrackingState(0.,0.,0.)
    for i in range(160):
        e.set_standard_path(tuple((0.,0.,float(y)) for y in np.linspace(s.yaw_rad,1.5,98)),i)
        r=tick(e,s)
        w=s.angular_radps+float(np.clip(r.command.angular_z_radps-s.angular_radps,-.025,.025))
        s=TrackingState(0.,0.,s.yaw_rad+w*.05,0.,w)
    assert abs(s.yaw_rad-1.5)<.05
    assert abs(s.angular_radps)<=.01


def test_publication_rejects_continuous_sign_crossing_despite_safe_endpoint():
    e=follower();e.set_standard_path(((0.,0.,0.),(-1.,0.,0.)))
    e.note_command(.003384,0.,0.)
    s=TrackingState(0.,0.,0.,.003384,0.)
    target=np.array([-.0148655,-.0148655])
    v,w=s.linear_mps,s.angular_radps
    breach=False
    for _ in range(100):
        v+=float(np.clip(target[0]-v,-.0005,.0005))
        w+=float(np.clip(target[1]-w,-.0005,.0005))
        breach |= abs(v)>.01 and abs(w)>abs(v)+1e-9
    assert breach  # independent actuator oracle, not production rollout
    assert not e.certify_publication(s,*target,0.)


@pytest.mark.parametrize('targets',[(1.,-.6), (2*math.pi,), (math.pi+.4,)])
def test_dense_spin_preserves_unwrapped_order_and_reversals(targets):
    e=follower();s=TrackingState(0.,0.,0.)
    points=[(0.,0.,0.)];start=0.
    for end in targets:
        points.extend((0.,0.,float(y)) for y in np.linspace(start,end,100)[1:]);start=end
    e.set_standard_path(points)
    reached=[];yaw_trace=[]
    for i in range(1200):
        e.set_standard_path(points,identity=i)
        r=tick(e,s)
        w=s.angular_radps+float(np.clip(r.command.angular_z_radps-s.angular_radps,-.025,.025))
        s=TrackingState(0.,0.,s.yaw_rad+w*.05,0.,w);yaw_trace.append(s.yaw_rad)
        if len(reached)<len(targets) and abs(s.yaw_rad-targets[len(reached)])<.05:
            reached.append(targets[len(reached)])
        if r.phase=='WAITING':break
    assert reached==list(targets)
    assert abs(s.yaw_rad-targets[-1])<.05 and abs(s.angular_radps)<=.01
    assert r.phase=='WAITING'


@pytest.mark.parametrize('delay',[0.,.1])
def test_accepted_targets_obey_independent_fine_actuator_with_history(delay):
    e=follower();e.policy=replace(e.policy,control_response_time_s=delay)
    e.set_standard_path(((0.,0.,0.),(-1.,0.,0.)))
    v,w=.003384,0.;pending=[];last=np.array([v,w]);e.note_command(*last,0.)
    # Include the real failing reversal, safe braking and both movement signs.
    proposals=[(-.0148655,-.0148655),(0.,0.),(-.02,-.01),(-.04,-.02),(0.,0.),(.02,.01)]
    for i,proposal in enumerate(proposals):
        t=i*.05;e._model_time=t
        state=TrackingState(0.,0.,0.,v,w)
        target=np.array(proposal)
        if not e.certify_publication(state,*target,0.):target=np.zeros(2)
        assert e.certify_publication(state,*target,0.)
        e.note_command(*target,t);pending.append((t+delay,target))
        for j in range(50):
            now=t+(j+1)*.001
            for arrival,u in pending:
                if arrival<=now+1e-10:last=u
            old_v,old_w=v,w
            v+=float(np.clip(last[0]-v,-.0005,.0005))
            w+=float(np.clip(last[1]-w,-.0005,.0005))
            assert abs(v-old_v)<=.000500001 and abs(w-old_w)<=.000500001
            assert abs(v)<=.2 and abs(w)<=1.
            if abs(v)>1e-8:assert abs(w)<=abs(v)+1e-8


@pytest.mark.parametrize('route',['score','weighted','fallback'])
def test_safe_first_command_cannot_certify_later_continuous_reversal(route):
    e=follower();e.set_standard_path(((0.,0.,0.),(1.,0.,0.)))
    state=TrackingState(0.,0.,0.);e._accept_pending(state)
    e.config=replace(e.config,model_dt=.05)
    unsafe=np.array([[0.,0.],[.003384,0.],[-.0148655,-.0148655],[0.,0.]])
    safe=np.zeros_like(unsafe)
    sequences=np.stack((unsafe,safe))
    # Independent actuator oracle: all 50ms endpoints obey curvature, but the
    # third interval contains a moving violation with a safe first command.
    v=w=0.;breach=False
    for target in unsafe:
        for _ in range(50):
            v+=float(np.clip(target[0]-v,-.0005,.0005))
            w+=float(np.clip(target[1]-w,-.0005,.0005))
            breach |= abs(v)>.01 and abs(w)>abs(v)+1e-9
        assert abs(v)<=.01 or abs(w)<=abs(v)+1e-9
    assert breach
    assert e.certify_publication(state,0.,0.,0.)
    if route=='score':
        cost,_=e._score(state,sequences,np.array([1.,0.]),0.,.05)
        assert np.isinf(cost[0]) and np.isfinite(cost[1])
    else:
        costs=np.array([0.,1.]) if route=='fallback' else None
        valid,_=e._certify_sequences(state,sequences,.05,costs=costs)
        assert valid.tolist()==[False,True]


@pytest.mark.parametrize('cause',['unknown_map','pending_history'])
def test_transient_stopping_failure_recovers_only_after_real_conditions_change(cause):
    e=follower()
    e.policy=replace(e.policy,control_response_time_s=.15)
    path=((0.,0.,0.),(0.,0.,.6));e.set_standard_path(path)
    state=TrackingState(0.,0.,0.)
    if cause=='unknown_map':e.set_map(GridMap(np.zeros((100,100),np.uint8),.1,(-5.,-5.)))
    else:e.note_command(.02,.2,0.)  # real queued target, not safe merely because feedback is stopped
    r=e.update(state,.05,now_s=.05)
    assert e.failure=='MPPI_STOPPING_UNSAFE'
    history=[(t,u.copy()) for t,u in e._command_history]
    for i in range(2):
        e.set_standard_path(path,identity=i)
        r=e.update(state,.05,now_s=.06+i*.01)
        assert e.failure=='MPPI_STOPPING_UNSAFE'
        assert e.diagnostics['stop_certified'] is False
        assert e.diagnostics['measured_stopped'] is True
        assert r.command.linear_x_mps==r.command.angular_z_radps==0.
    assert all(t==old_t and np.array_equal(u,old_u) for (t,u),(old_t,old_u) in zip(e._command_history,history))
    before=getattr(e,'no_progress_s',0.)
    e.set_map(GridMap(np.ones((100,100),np.uint8),.1,(-5.,-5.)))
    # A published zero and elapsed delay drain the queue; retain every historical record.
    e.note_command(0.,0.,.1)
    r=e.update(state,.05,now_s=.5)
    assert e.failure is None and r.command.angular_z_radps>0.
    assert getattr(e,'no_progress_s',0.)>=before
    assert len(e._command_history)>=len(history)+1
    for i in range(100):
        e.note_command(r.command.linear_x_mps,r.command.angular_z_radps,.5+i*.05)
        w=state.angular_radps+float(np.clip(r.command.angular_z_radps-state.angular_radps,-.025,.025))
        assert r.command.linear_x_mps==0.
        state=TrackingState(0.,0.,state.yaw_rad+w*.05,0.,w)
        r=e.update(state,.05,now_s=.55+i*.05)
    assert abs(state.yaw_rad-.6)<.05 and abs(state.angular_radps)<=.01


@pytest.mark.parametrize('fault',['INVALID_STATE','UNSAFE'])
def test_fresh_standard_path_does_not_clear_hard_fault(fault):
    e=follower(no_progress_timeout_s=.1);path=((0.,0.,0.),(1.,0.,0.));e.set_standard_path(path)
    if fault=='INVALID_STATE':e.update(TrackingState(float('nan'),0.,0.),.05)
    else:e.failure=fault
    assert e.failure==fault
    elapsed=getattr(e,'no_progress_s',0.)
    e.set_standard_path(path,identity=99)
    r=e.update(TrackingState(0.,0.,0.),.05)
    assert e.failure==fault and r.command.linear_x_mps==r.command.angular_z_radps==0.
    assert getattr(e,'no_progress_s',0.)>=elapsed


def test_recovery_certifies_measured_motion_without_fabricating_a_stop():
    e=follower();e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    e.set_map(GridMap(np.zeros((100,100),np.uint8),.1,(-5.,-5.)))
    state=TrackingState(0.,0.,0.,.1,.05)
    e.note_command(.1,.05,0.)
    r=e.update(state,.05,now_s=.05)
    assert e.failure=='MPPI_STOPPING_UNSAFE' and r.phase=='BRAKING'
    history=[(t,u.copy()) for t,u in e._command_history]
    e.set_map(GridMap(np.ones((100,100),np.uint8),.1,(-5.,-5.)))
    r=e.update(state,.05,now_s=.1)
    assert e.failure is None
    assert e.diagnostics['stop_certified'] is True
    assert e.diagnostics['measured_stopped'] is False
    assert len(e._command_history)==len(history)
    assert all(t==old_t and np.array_equal(u,old_u) for (t,u),(old_t,old_u) in zip(e._command_history,history))
    assert state.linear_mps==.1 and state.angular_radps==.05
    v,w=state.linear_mps,state.angular_radps
    for _ in range(50):
        v+=float(np.clip(r.command.linear_x_mps-v,-.0005,.0005))
        w+=float(np.clip(r.command.angular_z_radps-w,-.0005,.0005))
        if abs(v)>1e-8:assert abs(w)<=abs(v)+1e-8


@pytest.mark.parametrize('curved',[False,True])
def test_stopped_spin_replaced_by_rolling_translation_counts_physical_progress(curved):
    e=follower(no_progress_timeout_s=.35)
    e.set_standard_path(((0.,0.,0.),(0.,0.,.5)))
    tick(e,TrackingState(0.,0.,.3))
    # 新参考到达时车已静止；旧旋转端点留有零平移方向。
    for i in range(30):
        s=i*.01;theta=.45+s if curved else .45
        x=math.sin(theta)-math.sin(.45) if curved else s*math.cos(theta)
        y=math.cos(.45)-math.cos(theta) if curved else s*math.sin(theta)
        points=tuple((x+j*.02*math.cos(theta),y+j*.02*math.sin(theta),theta) for j in range(51))
        e.set_standard_path(points,i)
        r=tick(e,TrackingState(x,y,theta,0. if i==0 else .2,.2 if curved and i else 0.))
        assert e.failure is None, (i,x,y,getattr(e,'no_progress_s',0.))
    assert math.hypot(x,y)>.28 and r.phase=='TRACKING'


@pytest.mark.parametrize('motion',['stationary','lateral','wrongway','oscillation'])
def test_motion_observation_is_not_a_second_task_timeout(motion):
    e=follower(no_progress_timeout_s=.4)
    for i in range(40):
        x=(-i*.01 if motion=='wrongway' else .02*(i%2) if motion=='oscillation' else 0.)
        y=i*.01 if motion=='lateral' else 0.
        e.set_standard_path(((0.,0.,0.),(2.+i*.001,0.,0.)),i)
        tick(e,TrackingState(x,y,0.))
    assert e.failure is None and getattr(e,'no_progress_s',0.)==0.


def test_stopped_incompatible_rebindings_preserve_command_history_without_task_clock():
    e=follower(no_progress_timeout_s=.3)
    for i in range(12):
        end=1. if i%2 else -1.
        e.set_standard_path(((0.,0.,0.),(end,0.,0.)),i)
        before=getattr(e,'no_progress_s',0.)
        history=[(t,u.copy()) for t,u in e._command_history]
        r=e.update(TrackingState(0.,0.,0.),.05)
        assert getattr(e,'no_progress_s',0.)>=before
        assert len(history)==len(e._command_history)
        assert all(t==old_t and np.array_equal(u,old_u) for (t,u),(old_t,old_u) in zip(e._command_history,history))
        e.note_command(r.command.linear_x_mps,r.command.angular_z_radps)
    assert e.failure is None and getattr(e,'no_progress_s',0.)==0.


@pytest.mark.parametrize('sign,start',[(1.,0.),(-1.,0.),(1.,2.8),(-1.,-2.8)])
def test_same_direction_spin_crosses_internal_boundary_without_stopping(sign,start):
    e=follower(motion_model='differential_drive')
    points=tuple((0.,0.,start+sign*y) for y in (0.,math.pi/2,5*math.pi/6))
    e.set_standard_path(points)
    tick(e,TrackingState(0.,0.,start))
    # 验证实际旋转跨段连续性，定位角度不需要累计圈数。
    for offset in (-.02,0.,.02):
        r=tick(e,TrackingState(0.,0.,start+sign*(math.pi/2+offset),0.,sign*.1))
        assert r.phase not in ('BRAKING','WAITING','FAILED')
        assert r.command.linear_x_mps==0.
        assert sign*r.command.angular_z_radps>0.
    assert e._stage_index==1
    assert e.path is e._stages[1].path


@pytest.mark.parametrize('sign',[1.,-1.])
def test_multiturn_spin_passes_all_internal_boundaries_and_stops_at_end(sign):
    e=follower(motion_model='differential_drive')
    yaws=[sign*i*math.pi/2 for i in range(10)]
    e.set_standard_path(tuple((0.,0.,y) for y in yaws))
    tick(e,TrackingState(0.,0.,0.))
    for i,yaw in enumerate(yaws[1:-1],1):
        # 验证实际旋转跨段连续性，定位角度不需要累计圈数。
        wrapped=math.atan2(math.sin(yaw),math.cos(yaw))
        r=tick(e,TrackingState(0.,0.,wrapped,0.,sign*.1))
        assert r.phase not in ('BRAKING','WAITING','FAILED')
        assert sign*r.command.angular_z_radps>0.
        assert e._stage_index==i
    r=tick(e,TrackingState(0.,0.,yaws[-1],0.,sign*.1))
    assert r.phase=='BRAKING'
    assert tick(e,TrackingState(0.,0.,yaws[-1])).phase=='WAITING'


def test_spin_reversal_keeps_actual_stop_boundary():
    e=follower(motion_model='differential_drive')
    e.set_standard_path(((0.,0.,0.),(0.,0.,math.pi/2),(0.,0.,0.)))
    tick(e,TrackingState(0.,0.,0.))
    r=tick(e,TrackingState(0.,0.,math.pi/2,0.,.1))
    assert r.phase=='BRAKING' and e._stage_index==0
    tick(e,TrackingState(0.,0.,math.pi/2))
    assert e._stage_index==1
    assert tick(e,TrackingState(0.,0.,math.pi/2)).command.angular_z_radps<0.


@pytest.mark.parametrize('end',[5*math.pi/6,-5*math.pi/6,4*math.pi])
def test_simulated_spin_has_no_internal_stop(end):
    e=follower(motion_model='differential_drive')
    points=tuple((0.,0.,float(y)) for y in np.linspace(0.,end,100))
    e.set_standard_path(points)
    s=TrackingState(0.,0.,0.);crossed=False
    for _ in range(1600):
        r=tick(e,s)
        if math.pi/2-.1<abs(s.yaw_rad)<abs(end)-.1:
            crossed=True
            assert r.phase not in ('BRAKING','WAITING','FAILED')
            assert abs(s.angular_radps)>.01
            assert r.command.angular_z_radps*end>0.
        w=s.angular_radps+float(np.clip(r.command.angular_z_radps-s.angular_radps,-.025,.025))
        s=TrackingState(0.,0.,s.yaw_rad+w*.05,0.,w)
        if r.phase=='WAITING':break
    assert crossed and r.phase=='WAITING'
    assert abs(s.yaw_rad-end)<.05 and abs(s.angular_radps)<=.01


def test_spin_to_translation_keeps_alignment_and_stop_boundary():
    e=follower(motion_model='differential_drive')
    e.set_standard_path(((0.,0.,0.),(0.,0.,math.pi/2),(0.,1.,math.pi/2)))
    tick(e,TrackingState(0.,0.,0.))
    r=tick(e,TrackingState(0.,0.,math.pi/2,0.,.1))
    assert r.phase=='BRAKING' and r.command.linear_x_mps==0.
    assert e._stage_index==0


@pytest.mark.parametrize('sign',[1.,-1.])
@pytest.mark.parametrize('start',[0.,2.8,-2.8])
def test_sparse_half_turn_with_small_initial_error_preserves_order(sign,start):
    e=follower(motion_model='differential_drive')
    points=tuple((0.,0.,start+sign*y) for y in (0.,math.pi,1.5*math.pi))
    e.set_standard_path(points)
    r=tick(e,TrackingState(0.,0.,start-sign*.01))
    assert e._stage_index==0
    assert all(abs(stage.spin_delta)<=math.pi/2+1e-12 for stage in e._stages)
    assert r.command.linear_x_mps==0. and sign*r.command.angular_z_radps>0.
