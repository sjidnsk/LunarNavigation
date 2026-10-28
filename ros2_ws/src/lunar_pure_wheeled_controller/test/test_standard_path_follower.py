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
    assert e.progress_m==progress and e.no_progress_s==.4
    assert len(e._command_history)==len(history)
    assert e.phase!='BRAKING'


def test_stall_survives_reference_revisions():
    e=follower(no_progress_timeout_s=.3)
    for i in range(12):
        e.set_standard_path(((0.,0.,0.),(1.+i*.001,0.,0.)),i)
        r=tick(e,TrackingState(0.,0.,0.))
    assert e.failure=='TRACKING_STALLED'
    assert r.command.linear_x_mps==0.


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
    assert e.progress_m==progress and e.no_progress_s==.4
    assert len(e._command_history)==len(history)
    assert all(t==old_t and np.array_equal(u,old_u) for (t,u),(old_t,old_u) in zip(e._command_history,history))
    r=tick(e,TrackingState(.2,0.,0.,.1))
    assert e.reference.points==a
    assert r.phase=='TRACKING' and r.command.linear_x_mps>0.
    assert e.progress_m>=progress and e.no_progress_s>=.4


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
