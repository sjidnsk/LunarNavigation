"""Independent behavior checks for the MPPI candidate backend."""
from dataclasses import replace
import math
import numpy as np
import pytest
from lunar_pure_wheeled_controller import mppi as module
from lunar_pure_wheeled_controller.mppi_map import GridMap, PolicyMapCache
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState


def grid(states=None, origin=(-2., -2.), resolution=.1, revision=1):
    return GridMap(np.ones((60,60),np.uint8) if states is None else states,
                   resolution,origin,revision=revision)


def executor(points=((0.,0.,0.),(1.,0.,0.)), **kwargs):
    e=module.MppiExecutor(TrackingPolicy(goal_position_tolerance_m=.1,goal_yaw_tolerance_rad=.05), **kwargs)
    e.set_path(points,identity=(b's',1))
    return e


def test_backend_exists_and_requires_a_map():
    e=executor()
    r=e.update(TrackingState(0,0,0),.05)
    assert r.command.linear_x_mps==0 and r.command.angular_z_radps==0
    assert r.phase!='COMPLETED'


def test_closed_segment_cannot_tunnel_through_thin_blocked_cell():
    states=np.ones((5,5),np.uint8);states[2,2]=2
    m=grid(states,origin=(0,0),resolution=.01)
    assert not m.valid_segments(np.array([[.015,.025]]),np.array([[.035,.025]]),0.)[0]
    assert not m.valid_segments(np.array([[.01,.02]]),np.array([[.02,.02]]),0.)[0]


def test_clearance_checks_cell_area_and_unknown():
    states=np.ones((5,5),np.uint8);states[2,2]=0
    m=grid(states,origin=(0,0),resolution=.1)
    a=np.array([[.1,.195]]);b=np.array([[.4,.195]])
    assert m.valid_segments(a,b,0.)[0]
    assert not m.valid_segments(a,b,.01)[0]


@pytest.mark.parametrize('reverse',[False,True])
def test_mppi_moves_with_symmetric_limits_and_validated_output(reverse):
    points=((0.,0.,0.),(-1. if reverse else 1.,0.,0.))
    e=executor(points);e.set_map(grid())
    r=e.update(TrackingState(0,0,0),.05)
    assert 0 < (-1 if reverse else 1)*r.command.linear_x_mps <= .025+1e-9
    assert abs(r.command.angular_z_radps)<=.025+1e-9
    assert e.diagnostics['valid_samples']>0
    assert e.diagnostics['weighted_updates']>=1


def test_start_heading_error_does_not_drive_through_one_centimeter_wall():
    states=np.ones((50,60),np.uint8);states[0,:]=2
    e=executor(((0.,0.,0.),(2.,0.,0.)));e.set_map(grid(states,origin=(-1.,-.11)))
    state=TrackingState(0,0,-.079)
    # Exact differential-drive plant, independently checked against y=-.01.
    for _ in range(70):
        r=e.update(state,.05);v,w=r.command.linear_x_mps,r.command.angular_z_radps
        e.note_command(v,w)
        yaw=state.yaw_rad+w*.05
        if abs(w)<1e-10:dx,dy=v*.05*math.cos(state.yaw_rad),v*.05*math.sin(state.yaw_rad)
        else:dx=v/w*(math.sin(yaw)-math.sin(state.yaw_rad));dy=v/w*(math.cos(state.yaw_rad)-math.cos(yaw))
        state=TrackingState(state.x_m+dx,state.y_m+dy,yaw,v,w)
        assert state.y_m>-.01
    assert state.x_m>.1


def test_weighted_sequence_is_rechecked_when_map_blocks_all_translation():
    e=executor();e.set_map(grid());e.update(TrackingState(0,0,0),.05)
    states=np.full((60,60),2,np.uint8)
    e.set_map(grid(states,revision=2))
    r=e.update(TrackingState(0,0,0),.05)
    assert r.command.linear_x_mps==0 and r.command.angular_z_radps==0
    assert r.phase=='FAILED'


def test_arrival_requires_position_yaw_and_measured_stop():
    e=executor();e.set_map(grid())
    assert e.update(TrackingState(1.,0,0,.1,0),.05).phase!='COMPLETED'
    assert e.update(TrackingState(1.,0,0,0,0),.05).phase=='COMPLETED'


def test_invalidated_reference_cannot_restart_from_duplicate():
    e=executor();e.set_map(grid());e.clear()
    e.set_path(((0.,0.,0.),(1.,0.,0.)),identity=(b's',1))
    # Node's invalidation revision gate is tested separately; clear must stop.
    e.clear()
    assert e.update(TrackingState(0,0,0),.05).command.linear_x_mps==0


def test_bad_state_and_policy_fail_closed():
    with pytest.raises(ValueError):module.MppiPolicy(temperature=0.)
    e=executor();e.set_map(grid())
    assert e.update(TrackingState(float('nan'),0,0),.05).command.linear_x_mps==0


def test_delta_cache_rejects_missing_base_and_epoch_changes():
    from types import SimpleNamespace as S
    tile=S(tile_x=-1,tile_y=-1,states=np.ones(65536,np.uint8),costs=np.zeros(65536))
    def response(**kw):
        d=dict(ready=True,full_snapshot=True,epoch='a',fine_revision=1,base_revision=0,
               frame_id='map',resolution_m=.1,origin=S(x=0.,y=0.),tiles=[tile],
               start_connection_x=[],start_connection_y=[])
        d.update(kw);return S(**d)
    cache=PolicyMapCache();cache.apply(response())
    m=cache.local_grid(-.05,-.05,1.)
    assert m.valid_segments(np.array([[-.05,-.05]]),np.array([[-.15,-.05]]),0.)[0]
    with pytest.raises(ValueError):cache.apply(response(full_snapshot=False,base_revision=0,fine_revision=3))
    with pytest.raises(ValueError):cache.apply(response(full_snapshot=False,epoch='b'))


def test_explicit_assumed_start_prefix_never_overrides_blocked_or_allows_reentry():
    from types import SimpleNamespace as S
    tile=S(tile_x=0,tile_y=0,states=np.ones(65536,np.uint8),costs=np.zeros(65536))
    tile.states[0]=0;tile.states[1]=2
    response=S(ready=True,full_snapshot=True,epoch='a',fine_revision=1,base_revision=0,
               frame_id='map',resolution_m=.1,origin=S(x=0.,y=0.),tiles=[tile],
               start_connection_x=[2],start_connection_y=[0],assumed_start_x=[0,1],assumed_start_y=[0,0])
    cache=PolicyMapCache();cache.apply(response);m=cache.local_grid(.05,.05,1.)
    assert m.sample(np.array([[.05,.05]]))[2][0]
    assert not m.sample(np.array([[.15,.05]]))[2][0]
    xy=np.array([[[.05,.05],[.05,.15],[.05,.05]]])
    assert not m.valid_trajectories(xy,0.)[0]


def test_delayed_straight_approach_finishes_without_curvature_breaking_clip():
    from collections import deque
    e=executor(((0.,0.,0.),(2.,0.,0.)));e.set_map(grid())
    s=TrackingState(0,0,0);pending=deque([(0.,0.)]*2)
    for _ in range(600):
        r=e.update(s,.05);e.note_command(r.command.linear_x_mps,r.command.angular_z_radps);pending.append((r.command.linear_x_mps,r.command.angular_z_radps))
        if abs(r.command.linear_x_mps)>.01:assert abs(r.command.angular_z_radps)<=abs(r.command.linear_x_mps)+1e-9
        cv,cw=pending.popleft()
        v=s.linear_mps+np.clip(cv-s.linear_mps,-.025,.025)
        w=s.angular_radps+np.clip(cw-s.angular_radps,-.025,.025)
        yaw=s.yaw_rad+w*.05
        if abs(w)<1e-12:dx,dy=v*.05*math.cos(s.yaw_rad),v*.05*math.sin(s.yaw_rad)
        else:dx=v/w*(math.sin(yaw)-math.sin(s.yaw_rad));dy=v/w*(math.cos(s.yaw_rad)-math.cos(yaw))
        s=TrackingState(s.x_m+dx,s.y_m+dy,yaw,v,w)
        if r.phase in ('COMPLETED','FAILED'):break
    assert r.phase=='COMPLETED', r.command.failure_reason


def test_stopping_certificate_covers_shorter_as_well_as_maximum_response_delay():
    states=np.ones((50,60),np.uint8);states[0,:]=2
    e=executor();e.set_map(grid(states,origin=(-1.,-.11001)))
    e.note_command(0.,.2)
    # A delayed prior turn can make forward motion appear safe. Immediate
    # response to the new forward command instead travels toward the wall.
    assert not e._brake_safe(TrackingState(0,0,-.003),np.array([[.025,0.]]),.05)[0]


def test_continuous_motion_cannot_cross_assumed_cell_between_free_vertices():
    states=np.ones((3,5),np.uint8);states[1,2]=0
    assumed=np.zeros_like(states,bool);assumed[1,2]=True
    m=GridMap(states,.1,(0.,0.),assumed=assumed)
    assert not m.valid_trajectories(np.array([[[.15,.15],[.35,.15]]]),0.)[0]


def test_nonfinal_segment_does_not_require_final_yaw():
    e=executor();e.set_map(grid())
    e.set_path(((0.,0.,0.),(1.,0.,math.pi)),identity=(b's',2),final=False)
    assert e.update(TrackingState(1.,0.,0.),.05).phase=='COMPLETED'


def test_rolling_endpoint_changes_do_not_erase_real_physical_progress():
    e=executor(mppi_policy=module.MppiPolicy(batch_size=16,time_steps=10))
    e.set_map(grid())
    for tick in range(330):
        x=tick*.005
        if tick%1==0:e.set_path(((x,0.,0.),(x+1.,0.,0.)),identity=(b's',tick+2),final=False)
        r=e.update(TrackingState(x,0.,0.,.1,0.),.05)
        assert r.command.failure_reason!='TRACKING_STALLED'


def test_map_freshness_uses_processing_stamp_not_reply_receipt():
    from types import SimpleNamespace as S
    from lunar_pure_wheeled_controller.mppi_map import map_response_fresh
    response=S(processed_stamp=S(sec=1,nanosec=0))
    assert map_response_fresh(response,1_200_000_000,.5)
    assert not map_response_fresh(response,2_000_000_000,.5)
    assert not map_response_fresh(response,500_000_000,.5)


def test_unpublished_candidate_does_not_enter_the_actuator_history():
    e=executor();e.set_map(grid());state=TrackingState(0.,0.,0.)
    e.update(state,.05)
    e.update(state,.05)  # Both results discarded; no command was sent.
    poses,_=e._rollout(state,np.zeros((1,8,2)),.05,delay_ticks=2)
    assert np.max(np.abs(poses[:,:,:2]))==0.


def test_large_response_delay_certificate_reaches_full_stop_before_accepting():
    policy=replace(TrackingPolicy(),control_response_time_s=2.)
    e=module.MppiExecutor(policy)
    states=np.ones((100,300),np.uint8);states[:,185:]=2
    e.set_map(GridMap(states,.01,(-1.,-.5)))  # Wall begins at x=.85.
    e.note_command(.2,0.,stamp_s=-5.)
    assert not e._brake_safe(TrackingState(0.,0.,0.,.2,0.),np.array([[.2,0.]]),.05)[0]


@pytest.mark.parametrize('x,yaw,linear,angular,complete',[
    (.925,.04,0.,0.,True),
    (.925,.04,.02,0.,False),
    (.925,.04,0.,.04,False),
    (.899,.04,0.,0.,False),
    (.925,.051,0.,0.,False),
])
def test_public_arrival_contract_is_not_silently_halved(x,yaw,linear,angular,complete):
    e=executor();e.set_map(grid())
    result=e.update(TrackingState(x,0.,yaw,linear,angular),.05)
    assert result.command.complete is complete
    if complete:
        assert result.command.linear_x_mps==result.command.angular_z_radps==0.


def test_public_arrival_does_not_accept_an_unsafe_pose():
    e=executor();states=np.ones((60,60),np.uint8);states[20,29]=2
    e.set_map(grid(states))
    result=e.update(TrackingState(.925,0.,.04),.05)
    assert not result.command.complete
    assert result.command.linear_x_mps==result.command.angular_z_radps==0.


@pytest.mark.parametrize('delay,age',[(0,0),(2,0),(0,1),(2,1)])
@pytest.mark.parametrize('sign',[1.,-1.])
def test_published_targets_match_acceleration_prediction_with_delayed_state(delay,age,sign):
    """Fixed candidate removes sampling/map effects; independent 1-D actuator."""
    from collections import deque
    e=executor();dt=.05;steps=40;targets=np.tile([sign*.2,0.],(1,steps,1))
    prediction,_=e._rollout(TrackingState(0.,0.,0.),targets,dt,delay_ticks=delay,history_delay_ticks=delay)
    pending=deque([(0.,0.)]*delay);states=[TrackingState(0.,0.,0.)];x=v=0.;last_command=0.
    for i in range(steps):
        state=states[max(0,len(states)-1-age)]
        command=e._first_commands(state,targets[:,0],dt)[0]
        assert abs(command[0]-last_command)<=.025+1e-12
        last_command=float(command[0]);e.note_command(*command,stamp_s=i*dt)
        pending.append(tuple(command));applied=pending.popleft()
        v+=float(np.clip(applied[0]-v,-.025,.025));x+=v*dt
        states.append(TrackingState(x,0.,0.,v,0.))
    assert x==pytest.approx(prediction[0,-1,0],abs=1e-10)


def test_rollout_matches_published_targets_through_stop_spin_and_reverse():
    """No hidden model-side controller may alter a published actuator target."""
    from collections import deque
    e=executor();dt=.05
    targets=np.array([[[.2,0.]]*10+[[0.,.4]]*12+[[-.2,0.]]*12+[[0.,0.]]*12])
    prediction,_=e._rollout(TrackingState(0.,0.,0.),targets,dt,delay_ticks=2,history_delay_ticks=2)
    pending=deque([(0.,0.)]*2);states=[TrackingState(0.,0.,0.)];actual=[[0.,0.,0.]];x=y=yaw=v=w=0.
    for i in range(targets.shape[1]):
        observed=states[max(0,len(states)-2)]
        command=e._first_commands(observed,targets[:,i],dt)[0];e.note_command(*command,stamp_s=i*dt)
        pending.append(tuple(command));cv,cw=pending.popleft()
        v+=float(np.clip(cv-v,-.025,.025));w+=float(np.clip(cw-w,-.025,.025));theta=w*dt
        if abs(w)<1e-12:dx,dy=v*dt*math.cos(yaw),v*dt*math.sin(yaw)
        else:dx=v/w*(math.sin(yaw+theta)-math.sin(yaw));dy=v/w*(math.cos(yaw)-math.cos(yaw+theta))
        x+=dx;y+=dy;yaw+=theta;actual.append([x,y,yaw]);states.append(TrackingState(x,y,yaw,v,w))
    assert np.allclose(prediction[0],actual,atol=1e-10,rtol=0.)


def test_actuator_rollout_cannot_suppress_an_actually_published_spin_target():
    e=executor();state=TrackingState(0.,0.,0.,.1,0.)
    raw=np.array([[[0.,.2]]]);command=e._first_commands(state,raw[:,0],.05)[0]
    positions,actual=e._rollout(state,raw,.05,delay=False)
    # Independent slew for the actual emitted target, with no transport delay.
    expected_v=.1+float(np.clip(command[0]-.1,-.025,.025))
    expected_w=float(np.clip(command[1],-.025,.025))
    assert actual[0,0]==pytest.approx([expected_v,expected_w])
    assert positions[0,1,2]==pytest.approx(expected_w*.05)


@pytest.mark.parametrize('control_dt',[.025,.05,.075])
def test_candidate_first_target_matches_publication_at_current_control_interval(control_dt):
    e=executor();e.set_map(grid())
    e.update(TrackingState(0.,0.,0.),control_dt)
    raw=np.tile([.2,0.],(1,3,1))
    projected=e._project_targets(raw,e.config.model_dt)
    published=e._first_commands(TrackingState(0.,0.,0.),raw[:,0],min(control_dt,e.config.model_dt))
    assert projected[:,0]==pytest.approx(published)
    # Later prediction knots retain the frozen model integration interval.
    assert projected[0,1,0]-projected[0,0,0]==pytest.approx(.025)


@pytest.mark.parametrize('direction',[1.,-1.])
@pytest.mark.parametrize('control_dt',[.025,.05,.075])
def test_emitted_target_reuses_its_full_braking_certificate_once(direction,control_dt):
    e=executor(((0.,0.,0.),(direction*2.,0.,0.)));e.set_map(grid())
    calls=[];original=e._brake_safe
    def certify(state,first,dt):
        valid=original(state,first,dt)
        calls.extend((tuple(command),bool(safe)) for command,safe in zip(first,valid))
        return valid
    e._brake_safe=certify
    result=e.update(TrackingState(0.,0.,0.),control_dt)
    emitted=(result.command.linear_x_mps,result.command.angular_z_radps)
    assert result.phase=='TRACKING' and abs(emitted[0])>0.
    certificates=[safe for target,safe in calls if target==emitted]
    assert certificates==[True]


def test_rollout_batches_history_lookup_across_the_prediction_horizon(monkeypatch):
    e=executor();e.note_command(.1,0.,stamp_s=-1.)
    calls=[];original=np.searchsorted
    def lookup(*args,**kwargs):
        calls.append(1)
        return original(*args,**kwargs)
    monkeypatch.setattr(np,'searchsorted',lookup)
    poses,velocity=e._rollout(TrackingState(0.,0.,0.),np.zeros((2,40,2)),.05)
    assert len(calls)==1
    assert np.all(np.isfinite(poses)) and velocity.shape==(2,40,2)


@pytest.mark.parametrize('dtype',[np.float32,np.int64])
def test_rollout_keeps_integration_precision_for_unprojected_control_storage(dtype):
    e=executor();raw=np.zeros((1,1,2),dtype=dtype)
    poses,actual=e._rollout(TrackingState(0.,0.,0.,.1,0.),raw,.05,delay=False,project=False)
    # Independent float64 actuator step; storage dtype cannot change motion.
    assert poses[0,1,0]==pytest.approx((.1-.5*.05)*.05,rel=0.,abs=1e-16)
    assert actual.dtype==np.dtype(dtype)


@pytest.mark.parametrize('blocked_forward',[False,True])
@pytest.mark.parametrize('costs',[[3.,1.,2.],[1.,1.,1.]])
def test_fallback_certifies_only_the_cheapest_safe_candidate(blocked_forward,costs):
    e=executor();states=np.ones((60,60),np.uint8)
    if blocked_forward:states[:,21]=2
    e.set_map(grid(states))
    sequences=np.zeros((3,40,2));sequences[:,:,0]=np.array([.2,-.2,0.])[:,None]
    calls=[];original=e._brake_safe
    def certify(state,first,dt):
        calls.append(len(first));return original(state,first,dt)
    e._brake_safe=certify
    valid,poses=e._certify_sequences(TrackingState(0.,0.,0.),sequences,.05,costs=np.array(costs))
    expected=min(range(1 if blocked_forward else 0,3),key=lambda i:costs[i])
    assert np.flatnonzero(valid).tolist()==[expected]
    assert calls==[1]
    assert poses.shape==(3,41,3)


@pytest.mark.parametrize('last_safe',[False,True])
def test_fallback_reuses_identical_failed_braking_targets(last_safe):
    e=executor();resolution=.005;states=np.ones((200,500),np.uint8)
    states[:,225 if last_safe else 220]=2
    e.set_map(grid(states,origin=(-1.,-.5),resolution=resolution))
    e.note_command(.2,0.,stamp_s=-1.)
    sequences=np.zeros((12,1,2));sequences[:11,:,0]=.2
    state=TrackingState(0.,0.,0.,.2,0.)
    calls=[];original=e._brake_safe
    def certify(state,first,dt):
        calls.append(tuple(first[0]));return original(state,first,dt)
    e._brake_safe=certify
    valid,_=e._certify_sequences(state,sequences,.05,costs=np.arange(12))
    assert np.flatnonzero(valid).tolist()==([11] if last_safe else [])
    assert len(calls)==2
    assert calls[0]!=calls[1]


def stationary_grid(*, supported=True, blocked=False, anchor=(0.,0.)):
    states=np.zeros((20,20),np.uint8)
    if blocked:states[10,10]=2
    return GridMap(states,.1,(-1.05,-1.05),stationary_anchor=anchor if supported else None)


def test_native_stationary_support_allows_turn_without_inventing_observed_free():
    e=executor(((0.,0.,math.pi/2),));m=stationary_grid();e.set_map(m)
    s=TrackingState(0,0,0)
    r=e.update(s,.05)
    assert r.command.linear_x_mps==0
    assert r.command.angular_z_radps>0
    assert e.diagnostics['weighted_updates']>0
    assert np.all(m.states==0) and not np.any(m.assumed)
    assert e._brake_safe(s,np.array([[0.,.025]]),.05)[0]
    assert not e._brake_safe(s,np.array([[.025,0.]]),.05)[0]


@pytest.mark.parametrize('case',['no_support','blocked','other_anchor','translation','nonfinal','moving','slow_moving','slow_reverse','lateral','delayed_translation'])
def test_stationary_support_never_authorizes_other_motion(case):
    e=executor(((0.,0.,math.pi/2),))
    m=stationary_grid(supported=case!='no_support',blocked=case=='blocked',anchor=(.1,0.) if case=='other_anchor' else (0.,0.))
    if case=='translation':e.set_path(((0.,0.,0.),(.1,0.,0.)),identity=(b's',2))
    if case=='nonfinal':e.set_path(((0.,0.,math.pi/2),),identity=(b's',2),final=False)
    if case=='delayed_translation':e.note_command(.1,0.,stamp_s=0.)
    e.set_map(m)
    r=e.update(TrackingState(0,0,0,.1 if case=='moving' else .001 if case=='slow_moving' else -.001 if case=='slow_reverse' else 0.,0.,lateral_mps=.1 if case=='lateral' else 0.),.05,now_s=.05)
    assert r.command.linear_x_mps==0 and r.command.angular_z_radps==0
    assert r.phase!='COMPLETED'


def test_revoking_native_support_stops_turn_and_prevents_false_completion():
    e=executor(((0.,0.,math.pi/2),));e.set_map(stationary_grid())
    assert e.update(TrackingState(0,0,0),.05).command.angular_z_radps>0
    e.set_map(stationary_grid(supported=False))
    assert e.update(TrackingState(0,0,math.pi/2),.05).phase!='COMPLETED'
    assert e.update(TrackingState(0,0,math.pi/2),.05).command.angular_z_radps==0


def test_stationary_map_certificate_uses_current_response_anchor_and_is_not_sticky():
    from types import SimpleNamespace as S
    r=S(ready=True,full_snapshot=True,epoch='s',fine_revision=1,base_revision=0,
        frame_id='map',resolution_m=.1,origin=S(x=0.,y=0.),tiles=[],
        start_connection_x=[],start_connection_y=[],stationary_support_ready=True,
        anchor_pose=S(position=S(x=.05,y=.05)))
    cache=PolicyMapCache();cache.apply(r)
    assert cache.local_grid(.05,.05,1.).stationary_anchor==(.05,.05)
    r.full_snapshot=False;r.base_revision=1;r.stationary_support_ready=False
    cache.apply(r)
    assert cache.local_grid(.05,.05,1.).stationary_anchor is None
    r.stationary_support_ready=True;r.anchor_pose.position.x=float('nan')
    with pytest.raises(ValueError):cache.apply(r)


def test_stationary_authority_does_not_change_regular_tracking_or_rng():
    a=executor();b=executor()
    normal=grid();a.set_map(normal)
    b.set_map(GridMap(normal.states,normal.resolution,normal.origin,stationary_anchor=(0.,0.)))
    for x in (0.,.02,.04):
        state=TrackingState(x,0.,0.)
        assert a.update(state,.05)==b.update(state,.05)
        assert np.array_equal(a._nominal,b._nominal)
        assert a.rng.bit_generator.state==b.rng.bit_generator.state


def test_certified_pure_spin_does_not_repeat_raw_unknown_geometry_checks(monkeypatch):
    e=executor(((0.,0.,math.pi/2),));e.set_map(stationary_grid())
    def unexpected(*args,**kwargs):
        raise AssertionError('native-certified stationary support was checked as raw UNKNOWN geometry')
    for name in ['valid_points','valid_motion','valid_segments']:
        monkeypatch.setattr(GridMap,name,unexpected)
    r=e.update(TrackingState(0,0,0),.05)
    assert r.command.angular_z_radps>0 and r.command.linear_x_mps==0
    assert e.diagnostics['weighted_updates']>0


@pytest.mark.parametrize('direction', [-1., 1.])
def test_native_prefix_clearance_survives_center_entry_without_allowing_reentry(direction):
    states=np.ones((5,7),np.uint8);states[2,3]=0
    assumed=np.zeros_like(states,bool);assumed[2,3]=True
    m=GridMap(states,.1,(0.,0.),assumed=assumed)
    # Center has left the authorized start cell; its 1 cm clearance still
    # overlaps that cell. Native clearance accepts this support in either direction.
    p=np.array([[.35+direction*.051,.25]])
    q=p+np.array([[direction*.02,0.]])
    assert m.valid_points(p,.01,allow_assumed=False)[0]
    assert m.valid_segments(p,q,.01,allow_assumed=False)[0]
    assert not m.valid_segments(q,np.array([[.35,.25]]),.01,allow_assumed=False)[0]
    assert not m.valid_trajectories(np.array([[p[0],q[0],[.35,.25]]]),.01)[0]
    # Identical geometric contact with an unapproved UNKNOWN cell must fail.
    unapproved=GridMap(states,.1,(0.,0.))
    assert not unapproved.valid_points(p,.01,allow_assumed=False)[0]
    assert not unapproved.valid_segments(p,q,.01,allow_assumed=False)[0]


def test_publication_recertification_uses_physical_age_and_preserves_solver_state(monkeypatch):
    e=executor();e.set_map(grid());e._state_age_s=.02
    observed=[]
    monkeypatch.setattr(e,'_brake_safe',lambda state,first,dt: observed.append((e._state_age_s,first.copy())) or np.array([True]))
    assert e.certify_publication(TrackingState(0,0,0),.1,0.,.18)
    assert observed[0][0]==.18
    np.testing.assert_array_equal(observed[0][1],[[.1,0.]])
    assert e._state_age_s==.02
    assert not e.certify_publication(TrackingState(0,0,0),.1,0.,float('nan'))


def test_calibrated_nominal_delay_does_not_shrink_braking_delay_envelope(monkeypatch):
    e=executor(mppi_policy=module.MppiPolicy(nominal_response_time_s=0.))
    e.set_map(grid());e._state_age_s=.05;e._model_time=1.;e._first_target_step=.05
    e._command_history=[(0.,np.array([0.,0.]))]
    state=TrackingState(0,0,0)
    _,actual=e._rollout(state,np.tile([.2,0.],(1,40,1)),.05)
    assert actual[0,0,0]==0. and actual[0,1,0]>0.
    # Full-stop certification still tests the maximum physical response,
    # source age AND the independent compute/publication envelope.
    calls=[];roll=e._rollout
    def record(*a,**kw):calls.append(kw);return roll(*a,**kw)
    monkeypatch.setattr(e,'_rollout',record)
    e._brake_safe(state,np.array([[.1,0.]]),.05)
    assert np.max(calls[-1]['history_delay_ticks'])==3
    assert np.max(calls[-1]['delay_ticks'])==5


def test_path_follower_requires_no_map_or_geometric_certificates(monkeypatch):
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(),module.MppiPolicy(nominal_response_time_s=0.))
    e.set_path(((0.,0.,0.),(2.,0.,0.)),identity=(b'follow',1))
    def forbidden(*a,**kw):raise AssertionError('map/braking certificate in path follower')
    monkeypatch.setattr(e,'_brake_safe',forbidden)
    monkeypatch.setattr(e,'_stationary_supported',forbidden)
    r=e.update(TrackingState(0,0,0),.05)
    assert not e.requires_map and e.map is None
    assert r.phase=='TRACKING' and r.command.linear_x_mps>0
    assert abs(r.command.linear_x_mps)<=e.policy.max_linear_accel_mps2*.05
    assert e.diagnostics['weighted_updates']==1
    assert np.all(np.isfinite(e.predicted_trajectory))


def test_path_follower_arrival_stop_and_reference_cancellation():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(goal_position_tolerance_m=.1,goal_yaw_tolerance_rad=.05))
    e.set_path(((0.,0.,0.),(1.,0.,0.)),identity=(b'follow',1))
    assert e.update(TrackingState(1.,0.,0.,.1,0.),.05).phase=='BRAKING'
    assert e.update(TrackingState(1.,0.,0.),.05).phase=='COMPLETED'
    e.clear()
    assert e.update(TrackingState(1.,0.,0.),.05).command.linear_x_mps==0.
    assert e.path is None


@pytest.mark.parametrize('dt',[.025,.05,.075])
@pytest.mark.parametrize('batch',[1,9,32])
def test_follower_batched_projection_matches_original_without_per_step_helper(monkeypatch,dt,batch):
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());old=module.MppiExecutor(e.policy)
    rng=np.random.default_rng(111)
    for initial in ([.2,.2],[-.2,-.2],[0.,.5]):
        e._last_command=old._last_command=np.array(initial)
        e._first_target_step=old._first_target_step=dt
        raw=rng.normal(size=(batch,40,2))*.3
        expected=old._project_targets(raw,.05)
        def forbidden(*a,**kw):raise AssertionError('repeated per-step projection helper')
        monkeypatch.setattr(e,'_limit_target_step',forbidden)
        np.testing.assert_array_equal(e._project_targets(raw,.05),expected)


def test_follower_cost_distinguishes_centimeter_drift_and_entire_heading_history():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());e.set_path(((0.,0.,0.),(2.,0.,0.)))
    poses=np.zeros((3,41,3));poses[:,:,0]=np.linspace(0,.35,41)
    poses[1,:,1]=.01;poses[2,1:-1,2]=.08
    controls=np.zeros((3,40,2));controls[:,:,0]=.2
    cost=e._tracking_cost(TrackingState(0,0,0),controls,np.array([.35,0.]),0.,poses)
    assert cost[1]-cost[0]>e.config.temperature*5
    assert cost[2]>cost[0]+e.config.temperature*5


def test_follower_continuous_reference_cost_is_invariant_to_vertex_density():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());s=TrackingState(0,0,0)
    poses=np.zeros((1,41,3));poses[:,:,0]=np.linspace(.013,.337,41)
    controls=np.zeros((1,40,2))
    e.set_path(((0.,0.,0.),(2.,0.,0.)));a=e._tracking_cost(s,controls,np.array([.35,0.]),0.,poses)
    e.set_path(tuple((x,0.,0.) for x in np.linspace(0,2,51)));b=e._tracking_cost(s,controls,np.array([.35,0.]),0.,poses)
    np.testing.assert_allclose(a,b,atol=1e-12)


def test_follower_final_heading_uses_public_arrival_region():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(goal_position_tolerance_m=.1));e.set_path(((0.,0.,0.),(1.,0.,np.pi/2)))
    poses=np.zeros((2,41,3));poses[:,:,0]=.92;poses[1,:,2]=np.linspace(0,np.pi/2,41)
    cost=e._tracking_cost(TrackingState(.92,0,0),np.zeros((2,40,2)),np.array([1.,0.]),0.,poses)
    assert cost[1]<cost[0]


def test_follower_noise_has_reflected_angular_pairs_and_no_batch_bias():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());noise=e._control_noise()[10:]
    np.testing.assert_array_equal(noise[::2,:,0],noise[1::2,:,0])
    np.testing.assert_array_equal(noise[::2,:,1],-noise[1::2,:,1])
    np.testing.assert_allclose(noise.mean(axis=0),0.,atol=1e-16)


def test_follower_prefers_small_progress_loss_over_centimeter_lateral_error():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());e.set_path(((0.,0.,0.),(2.,0.,0.)))
    poses=np.zeros((3,41,3));poses[:,:,0]=np.linspace(0,.35,41);poses[1,:,0]=np.linspace(0,.34,41);poses[2,:,1]=.01
    c=e._tracking_cost(TrackingState(0,0,0),np.zeros((3,40,2)),np.array([.35,0.]),0.,poses)
    assert c[0]<c[1]<c[2]


def test_follower_corner_does_not_replace_external_reference_or_complete_route():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(goal_position_tolerance_m=.1))
    points=((0.,0.,0.),(1.,0.,0.),(1.,1.,np.pi/2))
    e.set_path(points,identity=(b'route',1))
    r=e.update(TrackingState(1.,0.,np.pi/2),.05)
    assert e.path.original==points and e.identity==(b'route',1)
    assert r.phase!='COMPLETED'


def test_follower_corner_turn_keeps_travel_direction_when_heading_crosses_ninety_degrees():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,1.,np.pi/2)),identity=(b'route',1))
    e.update(TrackingState(1.,0.,0.),.05)
    result=e.update(TrackingState(1.,0.,np.pi/2+.01),.05)
    assert e._passed_corner==(1.,0.) and result.phase=='TRACKING'


@pytest.mark.parametrize('seed',[17,91,123])
def test_follower_exact_straight_reference_does_not_generate_lateral_drift(seed):
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(),module.MppiPolicy(seed=seed,nominal_response_time_s=0.))
    e.set_path(((0.,0.,0.),(2.,0.,0.)));state=TrackingState(0,0,0)
    for _ in range(80):
        result=e.update(state,.05);v=result.command.linear_x_mps;w=result.command.angular_z_radps
        e.note_command(v,w)
        state=TrackingState(state.x_m+v*.05*math.cos(state.yaw_rad),state.y_m+v*.05*math.sin(state.yaw_rad),state.yaw_rad+w*.05,v,w)
        assert abs(state.y_m)<1e-10 and abs(state.yaw_rad)<1e-10


def test_follower_speed_coordination_reduces_speed_for_heading_and_stopping_distance():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());e.set_path(((0.,0.,0.),(2.,0.,0.)))
    raw=np.full((2,40,2),.2);s=TrackingState(0,0,0)
    e.heading_error=0.;a=e._limit_sample_speeds(s,raw.copy(),False)[0,0,0]
    e.heading_error=.7;b=e._limit_sample_speeds(s,raw.copy(),False)[0,0,0]
    e.heading_error=0.;e.progress_m=1.999;c=e._limit_sample_speeds(TrackingState(1.999,0,0),raw.copy(),False)[0,0,0]
    assert 0.<c<a and 0.<b<a


def test_follower_terminal_projection_keeps_endpoint_correction_speed_available():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(goal_position_tolerance_m=.1));e.set_path(((0.,0.,0.),(1.,0.,0.)))
    e.update(TrackingState(1.,.2,-np.pi/2),.05)
    assert e.diagnostics['speed_limit_mps']>0.


def test_follower_active_corner_is_not_skipped_by_projection_overshoot():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(goal_position_tolerance_m=.1));e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,1.,np.pi/2)),identity=(b'route',1))
    e._corner_target=((1.,0.),np.pi/2);e.progress_m=1.
    e.update(TrackingState(1.,.02,0.),.05)
    assert e.diagnostics['distance_m']<.03
    assert e._corner_target==((1.,0.),np.pi/2)


def test_follower_hot_reference_removing_corner_releases_internal_target():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,1.,np.pi/2)),identity=(b'route',1))
    e._corner_target=((1.,0.),np.pi/2)
    e.set_path(((.8,0.,0.),(2.,0.,0.)),identity=(b'route',2))
    assert e._corner_target is None


def test_follower_forward_only_reconciles_reversed_outgoing_corner_heading():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(allow_reverse=False))
    e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,1.,np.pi/2)),identity=(b'route',1))
    e._corner_target=((1.,0.),np.pi/2)
    e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,-1.,-np.pi/2)),identity=(b'route',2))
    assert e._corner_target is None


def test_follower_near_endpoint_speed_respects_prediction_horizon():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy());e.set_path(((0.,0.,0.),(1.,0.,0.)));e.progress_m=.98
    raw=np.full((2,40,2),.2);e._limit_sample_speeds(TrackingState(.98,0,0),raw,False)
    assert e.diagnostics['speed_limit_mps']<=.02/(e.config.time_steps*e.config.model_dt)+1e-12


def test_follower_endpoint_recovery_heading_points_back_to_goal():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    e=MppiPathFollower(TrackingPolicy(goal_position_tolerance_m=.1));e.set_path(((0.,0.,0.),(1.,0.,0.)));e.progress_m=1.
    poses=np.zeros((2,41,3));poses[:,:,0]=1.;poses[:,:,1]=.2;poses[0,:,2]=-np.pi/2
    cost=e._tracking_cost(TrackingState(1.,.2,-np.pi/2),np.zeros((2,40,2)),np.array([1.,0.]),-np.pi/2,poses)
    assert cost[0]<cost[1]
