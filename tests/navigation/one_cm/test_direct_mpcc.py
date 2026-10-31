import pytest
import numpy as np
from scripts.navigation.one_cm.nav1cm.execution import certified_command, action_prefix
from scripts.navigation.one_cm.nav1cm.contracts import VehicleState, Trajectory
from scripts.navigation.one_cm.nav1cm.seeds import rollout
from tests.navigation.one_cm.test_p3_session import make_session

@pytest.mark.parametrize("v,w,a,b",[(.1,0,.2,0),(0,.1,0,.2),(-.1,0,-.2,0),(0,0,0,0)])
def test_current_short_control(v,w,a,b):
    from types import SimpleNamespace as S
    measured=VehicleState(0,0,0,v,w)
    u=np.array([[a,b,0.]])
    full=Trajectory(1,1,[0.,.2],rollout([0,0,0,v,w,0],u,.2),u)
    action=action_prefix(full,measured,.05)
    class Certificate:
        def matches(self,*args): return True
    result=S(candidate=full,certificate=Certificate(),action=action,action_certificate=Certificate(),grid=object())
    assert certified_command(result,measured,object()) == pytest.approx((v+a*.05,w+b*.05))
    assert measured == VehicleState(0,0,0,v,w)
    result.action_certificate=None
    assert certified_command(result,measured,object()) == (0.,0.)

def test_direct_session_does_not_request_legacy_action_or_cancel_prefix():
    session,Goal=make_session()
    session.direct_control=True
    session.offer_goal(Goal(0,2,0))
    packet=session.tick(0.)
    assert packet.result.action is None
    assert packet.result.action_certificate is None
    session.cancel_goal()
    packet=session.tick(.05)
    assert packet.result.action is None
    assert packet.result.action_certificate is None


@pytest.mark.parametrize("change",['state','map','limits','version','controls'])
def test_real_certificate_refuses_changed_execution_inputs(change):
    from dataclasses import replace
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    session,Goal=make_session()
    session.direct_control=True
    session.offer_state(VehicleState(0,0,0,.1,0))
    session.cancel_goal()
    result=session.coordinator.cancel_tick(session.world,session.state,0.,need_action=True)
    measured=session.state
    limits=session.coordinator.limits
    assert 0 < certified_command(result,measured,limits)[0] < .1
    if change=='state':measured=replace(measured,v=.2)
    if change=='map':result=replace(result,grid=replace(result.grid,epoch=999))
    if change=='limits':limits=replace(limits,linear_speed_mps=.3)
    if change=='version':result=replace(result,action=replace(result.action,route_version=999))
    if change=='controls':
        u=result.action.controls.copy();u[0,0]=100.
        result=replace(result,action=replace(result.action,controls=u))
    assert certified_command(result,measured,limits)==(0.,0.)


@pytest.mark.parametrize('obstacle',[None,.9,1.15])
def test_hold_proof_checks_hold_without_extra_stop_tail(obstacle):
    from dataclasses import replace
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from scripts.navigation.one_cm.nav1cm.contracts import GridSnapshot, WheelLimits
    from scripts.navigation.one_cm.nav1cm.execution import command_hold_proof
    limits=WheelLimits(body_length_m=.2,body_width_m=.2,linear_speed_mps=1.,linear_accel_mps2=.5)
    cells=np.zeros((500,500),dtype=np.int8)
    if obstacle is not None:cells[245:255,int((obstacle+2.5)/.01)]=1
    grid=GridSnapshot(1,(-2.5,-2.5),.01,cells)
    measured=VehicleState(0,0,0,.5,0)
    u=np.array([[0.,0.,0.]])
    action=Trajectory(1,1,[0.,.05],rollout([0,0,0,.5,0,0],u,.05),u)
    cert=certify(action,grid,limits,1)
    assert cert.valid
    result=S(candidate=action,certificate=cert,grid=grid,action=action,action_certificate=cert)
    proof=command_hold_proof(action,grid,limits,1.8)
    assert proof.states[-1,3] == pytest.approx(.5)
    assert proof.times[-1] == pytest.approx(1.85)
    cmd=certified_command(result,measured,limits,command_hold_s=1.8)
    assert cmd == ((0.,0.) if obstacle==.9 else (.5,0.))


def test_hold_retains_certified_forward_metadata_after_translation_stops():
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    from scripts.navigation.one_cm.nav1cm.execution import command_hold_proof
    limits=WheelLimits(motion_model='differential_drive');grid=free_grid()
    state=VehicleState(0,0,0,.025,.1)
    u=np.array([[-.5,0.,.01]])
    action=Trajectory(1,1,[0.,.05],rollout([0,0,0,.025,.1,0],u,.05),u,'forward','mpcc')
    cert=certify(action,grid,limits,1)
    assert cert.valid
    proof=command_hold_proof(action,grid,limits,1.2)
    assert proof.mode==action.mode and proof.source==action.source
    assert certify(proof,grid,limits,1).valid
    result=S(candidate=action,certificate=cert,action=action,action_certificate=cert,grid=grid)
    assert certified_command(result,state,limits,command_hold_s=1.2)==pytest.approx((0.,.1))


def test_hold_proof_and_command_share_exact_measured_integration(monkeypatch):
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm import execution
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits();grid=free_grid()
    measured=VehicleState(0,0,0,.1000005,0)
    u=np.array([[0.,0.,0.]])
    action=Trajectory(1,1,[0.,.05],rollout([0,0,0,.1,0,0],u,.05),u,'forward','mpcc')
    cert=certify(action,grid,limits,1);assert cert.valid
    result=S(candidate=action,certificate=cert,action=action,action_certificate=cert,grid=grid)
    proofs=[];original=execution.command_hold_proof
    def capture(*args,**kwargs):
        proof=original(*args,**kwargs);proofs.append(proof);return proof
    monkeypatch.setattr(execution,'command_hold_proof',capture)
    command=execution.certified_command(result,measured,limits,command_hold_s=1.2)
    assert command[0]==measured.v
    assert proofs[0].states[0,3]==measured.v
    assert proofs[0].states[1,3]==command[0]

def test_velocity_reference_uses_first_shooting_interval_and_last_published_command():
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits(motion_model='differential_drive'); grid=free_grid()
    state=VehicleState(0,0,0,0,0)
    u=np.array([[0.,.4,0.]])
    full=Trajectory(grid.epoch,1,[0.,.2],rollout([0,0,0,0,0,0],u,.2),u,'spin','mpcc')
    cert=certify(full,grid,limits,1)
    result=S(candidate=full,certificate=cert,grid=grid,status='TRACKING')
    command=certified_velocity_command(result,state,limits,reference_interval_s=.2,
        last_command=(0.,.06),elapsed_s=.05,command_hold_s=.1)
    assert command == pytest.approx((0.,.08))
    assert state.w == 0.

@pytest.mark.parametrize('mode,control,expected', [('spin',[0.,.4,0.],(0.,.08)),('forward',[.4,0.,0.],(.08,0.))])
def test_delayed_nearzero_p3_does_not_restart_output_and_proof_binds_issued(monkeypatch,mode,control,expected):
    from types import SimpleNamespace as S
    import scripts.navigation.one_cm.nav1cm.execution as execution
    import scripts.navigation.one_cm.nav1cm.certification as certification
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits(motion_model='differential_drive');grid=free_grid();state=VehicleState(0,0,0,0,0)
    u=np.array([control]);full=Trajectory(grid.epoch,1,[0.,.2],rollout([0,0,0,0,0,0],u,.2),u,mode,'mpcc')
    result=S(candidate=full,certificate=certification.certify(full,grid,limits,1),grid=grid,status='TRACKING')
    original=certification.certify;proofs=[]
    def capture(proof,*args,**kwargs):
        proofs.append(proof);return original(proof,*args,**kwargs)
    monkeypatch.setattr(certification,'certify',capture)
    previous=(0.,0.)
    for _ in range(70):
        previous=execution.certified_velocity_command(result,state,limits,reference_interval_s=.2,
            last_command=previous,elapsed_s=.05,command_hold_s=.1)
        assert np.array_equal(proofs[-1].states[0,:5], [0.,0.,0.,0.,0.])
        hold=np.flatnonzero(np.all(np.abs(proofs[-1].controls)<1e-12,axis=1))
        assert len(hold)
        assert proofs[-1].states[hold[0],3:5] == pytest.approx(previous)
    assert previous == pytest.approx(expected)
    assert state.v==state.w==0.
    result.status='ARRIVED'
    assert execution.certified_velocity_command(result,state,limits,reference_interval_s=.2,
        last_command=previous,elapsed_s=.05,command_hold_s=.1)==(0.,0.)

@pytest.mark.parametrize('change',['map','limits','collision','state'])
def test_velocity_issued_certificate_rejects_changed_inputs(change):
    from dataclasses import replace
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits(motion_model='differential_drive');grid=free_grid();state=VehicleState(0,0,0,0,0)
    u=np.array([[0.,.4,0.]])
    full=Trajectory(grid.epoch,1,[0.,.2],rollout([0,0,0,0,0,0],u,.2),u,'spin','mpcc')
    result=S(candidate=full,certificate=certify(full,grid,limits,1),grid=grid,status='TRACKING')
    if change=='map':result.grid=replace(grid,epoch=grid.epoch+1)
    if change=='limits':limits=replace(limits,angular_accel_rps2=.3)
    if change=='collision':
        cells=grid.cells.copy();cells[:]=1;result.grid=replace(grid,cells=cells)
    if change=='state':state=replace(state,x=.1)
    assert certified_velocity_command(result,state,limits,reference_interval_s=.2,
        last_command=(0.,.06),elapsed_s=.05,command_hold_s=.1)==(0.,0.)


def test_online_reference_remote_tail_outside_window_is_not_execution_permission(monkeypatch):
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm import execution, certification
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits(motion_model='differential_drive');grid=free_grid()
    u=np.array([[.4,0.,0.],[0.,0.,0.]])
    states=[[0,0,0,0,0,0]]
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    states.append(step(states[-1],u[0],.2));states.append(step(states[-1],u[1],100.))
    source=Trajectory(grid.epoch,1,[0.,.2,100.2],states,u,'forward','mpcc')
    assert not certification.certify(source,grid,limits,1).valid
    state=VehicleState(.01,0,0,.0001,0)
    result=S(candidate=source,certificate=None,grid=grid,status='BUFFER_EXECUTING',reference_phase_s=0.)
    proofs=[];original=certification.certify
    def capture(proof,*args,**kwargs):
        proofs.append(proof);return original(proof,*args,**kwargs)
    monkeypatch.setattr(certification,'certify',capture)
    command=execution.certified_velocity_command(result,state,limits,reference_interval_s=.2,
        last_command=(.06,0.),elapsed_s=.05,command_hold_s=.1)
    assert command==pytest.approx((.08,0.))
    assert len(proofs)==1
    assert proofs[0].states[0,:5]==pytest.approx([.01,0,0,.0001,0])
    assert proofs[0].states[-1,3:5]==pytest.approx(command)


def test_online_reference_geometry_consumes_without_reanchor_or_remote_cert(monkeypatch):
    from tests.navigation.one_cm.test_reference_source_buffer import source, make_source_coordinator
    from scripts.navigation.one_cm.nav1cm import async_coordinator
    original=source();c,world=make_source_coordinator(original)
    c.velocity_reference=True
    monkeypatch.setattr(c.buffer._conditioner,'reanchor',lambda *a,**k: pytest.fail('online reanchor'))
    monkeypatch.setattr(async_coordinator,'certify',lambda *a,**k: pytest.fail('online full certificate'))
    result=c.tick(world,VehicleState(.04,0,0,.0001,0),(2,0),.05,goal_yaw=0.,need_action=False)
    assert result.status=='BUFFER_EXECUTING'
    assert result.certificate is None
    assert result.candidate is original or result.candidate.digest()==original.digest()
    assert result.reference_phase_s==pytest.approx(.2,abs=1e-5)


@pytest.mark.parametrize('hazard',['body','hold','stop','unknown','window'])
def test_online_reference_actual_issued_proof_rejects_hazards(hazard):
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.contracts import GridSnapshot,WheelLimits
    from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
    limits=WheelLimits(body_length_m=.2,body_width_m=.2,linear_speed_mps=1.,linear_accel_mps2=.5)
    cells=np.zeros((500,500),dtype=np.int8)
    if hazard!='window':
        x={'body':0.,'hold':.5,'stop':1.13,'unknown':.5}[hazard]
        cells[245:255,int((x+2.5)/.01)]=(-1 if hazard=='unknown' else 1)
    grid=GridSnapshot(1,(-2.5,-2.5),.01,cells)
    u=np.zeros((1,3));source=Trajectory(1,1,[0.,10.],rollout([0,0,0,.5,0,0],u,10.),u,'forward','mpcc')
    result=S(candidate=source,certificate=None,reference_phase_s=0.,grid=grid,status='BUFFER_EXECUTING')
    state=VehicleState(2.3 if hazard=='window' else 0.,0,0,.5,0)
    expected=(.5,0.) if hazard=='stop' else (0.,0.)
    assert certified_velocity_command(result,state,limits,reference_interval_s=.2,
        last_command=(.5,0.),elapsed_s=.05,command_hold_s=1.8)==expected


def test_online_source_exhaustion_goal_and_offline_keep_strict_boundary():
    from tests.navigation.one_cm.test_reference_source_buffer import source,make_source_coordinator
    original=source();c,world=make_source_coordinator(original);c.velocity_reference=True
    result=c.tick(world,VehicleState(.8,0,0,.2,0),(2,0),.05,need_action=False)
    assert result.reference_phase_s is None
    assert c.buffer.source is None
    c.buffer.install(original,.05)
    c.replace_goal((3,0),generation=2)
    assert c.buffer.source is None
    c.buffer.install(original,.05)
    c.invalidate_goal(generation=3)
    assert c.buffer.source is None
    c,world=make_source_coordinator(original);c.velocity_reference=True
    result=c.tick(world,VehicleState(.04,0,0,.0001,0),(2,0),.05,need_action=True)
    assert result.reference_phase_s is None
    assert result.certificate is not None


def test_online_microscopic_spin_does_not_replay_old_braking_prefix(monkeypatch):
    from scripts.navigation.one_cm.nav1cm.reference_buffer import ReferenceTrajectoryBuffer
    from scripts.navigation.one_cm.nav1cm.seeds import braking_prefix,join_trajectories
    from scripts.navigation.one_cm.nav1cm.contracts import LocalReference,WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
    limits=WheelLimits(motion_model='differential_drive');grid=free_grid()
    initial=VehicleState(0,0,0,.0002,0)
    ref=LocalReference(1,grid.epoch,[[0,0]],[0.],0.)
    prefix=braking_prefix(initial,grid,ref,limits,retain_angular=True)
    u=np.array([[0.,.4,0.]])
    spin=Trajectory(1,1,[0.,.2],rollout(prefix.states[-1],u,.2),u,'spin','mpcc')
    original=join_trajectories(prefix,spin)
    buffer=ReferenceTrajectoryBuffer();buffer.install(original,0.)
    state=VehicleState(0,0,0,.0001,0)
    monkeypatch.setattr(buffer._conditioner,'reanchor',lambda *a,**k:pytest.fail('replayed old brake'))
    candidate=buffer.current(state,.1,.05,limits=limits,velocity_reference=True)
    result=S(candidate=candidate,certificate=None,grid=grid,status='BUFFER_EXECUTING',reference_phase_s=buffer.source_phase_s)
    v,w=certified_velocity_command(result,state,limits,reference_interval_s=.2,
        last_command=(0.,.06),elapsed_s=.05,command_hold_s=.1)
    assert v==0 and w>0


@pytest.mark.parametrize('incoming',['failed','stale','fresh','nonfinite','mode'])
def test_online_worker_handover_keeps_finite_source_until_legal_replacement(incoming):
    from dataclasses import replace
    from tests.navigation.one_cm.test_reference_source_buffer import source,make_source_coordinator
    original=source();c,world=make_source_coordinator(original);c.velocity_reference=True
    c.local_worker.pending=None
    state=VehicleState(0,0,0,.0001,0)
    c.tick(world,state,(2,0),.05,goal_yaw=0.,need_action=False)
    identity=dict(c._inflight)
    candidate=replace(original,source='new-source')
    result=dict(request_identity=identity,candidate=candidate,reason='CANDIDATE_UNCERTIFIED')
    if incoming=='failed':result['candidate']=None
    if incoming=='stale':result['request_identity']=dict(identity,goal_revision=999)
    if incoming=='nonfinite':
        states=candidate.states.copy();states[-1,0]=float('nan')
        result['candidate']=replace(candidate,states=states)
    if incoming=='mode':result['candidate']=replace(candidate,mode='unsupported')
    c.local_worker.poll=lambda:result
    observed=c.tick(world,state,(2,0),.1,goal_yaw=0.,need_action=False)
    assert observed.status=='BUFFER_EXECUTING'
    assert observed.certificate is None
    assert c.buffer.source.source==('new-source' if incoming=='fresh' else original.source)
    if incoming=='fresh':
        assert any(e['kind']=='LOCAL_REFERENCE_ACCEPTED' for e in observed.events)
        assert not any(e['kind']=='LOCAL_CURRENT_MAP_REVALIDATED' for e in observed.events)


def test_velocity_diagnostics_preserve_command_and_single_certification(monkeypatch):
    from types import SimpleNamespace as S
    import scripts.navigation.one_cm.nav1cm.execution as execution
    import scripts.navigation.one_cm.nav1cm.certification as certification
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits(motion_model='differential_drive'); grid=free_grid()
    state=VehicleState(0,0,0,0,0); u=np.array([[0.,.4,0.]])
    full=Trajectory(grid.epoch,1,[0.,.2],rollout([0,0,0,0,0,0],u,.2),u,'spin','mpcc')
    result=S(candidate=full,certificate=certification.certify(full,grid,limits,1),grid=grid,status='TRACKING')
    calls=[]; original=certification.certify
    def certify(*args,**kwargs):
        calls.append(1); return original(*args,**kwargs)
    monkeypatch.setattr(certification,'certify',certify)
    kwargs=dict(reference_interval_s=.2,last_command=(0.,.06),elapsed_s=.05,command_hold_s=.1)
    plain=execution.certified_velocity_command(result,state,limits,**kwargs)
    diagnostics={}
    observed=execution.certified_velocity_command(result,state,limits,diagnostics=diagnostics,**kwargs)
    assert observed == plain
    assert len(calls)==2
    assert diagnostics['command_reject_reason'] is None
    assert diagnostics['target_raw_vw'] == pytest.approx([0.,.08])
    assert diagnostics['target_vw'] == pytest.approx([0.,.08])
    assert diagnostics['issued_vw'] == pytest.approx(observed)
    assert diagnostics['source_initial_vw'] == [0.,0.]
    assert diagnostics['command_total_ms'] >= diagnostics['command_certification_ms'] >= 0
    result.status='ARRIVED'
    assert execution.certified_velocity_command(result,state,limits,diagnostics=diagnostics,**kwargs)==(0.,0.)
    assert diagnostics['command_reject_reason']=='ARRIVED'
    assert len(calls)==2
    assert 'target_raw_vw' not in diagnostics


@pytest.mark.parametrize('change,reason', [
    ('missing','MISSING_RESULT_STATE_OR_CANDIDATE'),
    ('brake','BRAKE_SOURCE'),('interval','INVALID_COMMAND_INTERVAL'),
    ('phase','INVALID_REFERENCE_PHASE_OR_MODE'),('previous','INVALID_PREVIOUS_COMMAND'),
    ('collision','ISSUED_CERTIFICATE_MISMATCH')])
def test_velocity_diagnostic_rejection_reasons(change,reason):
    from dataclasses import replace
    from types import SimpleNamespace as S
    from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    from tests.navigation.one_cm.test_certification import free_grid
    limits=WheelLimits(motion_model='differential_drive'); grid=free_grid()
    state=VehicleState(0,0,0,0,0); u=np.array([[0.,.4,0.]])
    full=Trajectory(grid.epoch,1,[0.,.2],rollout([0,0,0,0,0,0],u,.2),u,'spin','mpcc')
    result=S(candidate=full,certificate=None,grid=grid,status='TRACKING',reference_phase_s=0.)
    kwargs=dict(reference_interval_s=.2,last_command=(0.,.06),elapsed_s=.05,command_hold_s=.1)
    if change=='missing': result=None
    if change=='brake': result.candidate=replace(full,source='brake')
    if change=='interval': kwargs['elapsed_s']=0.
    if change=='phase': result.reference_phase_s=-1.
    if change=='previous': kwargs['last_command']=1.
    if change=='collision':
        cells=grid.cells.copy(); cells[:]=1; result.grid=replace(grid,cells=cells)
    diagnostic={}
    assert certified_velocity_command(result,state,limits,diagnostics=diagnostic,**kwargs)==(0.,0.)
    assert diagnostic['command_reject_reason']==reason
    assert diagnostic['command_total_ms']>=0
