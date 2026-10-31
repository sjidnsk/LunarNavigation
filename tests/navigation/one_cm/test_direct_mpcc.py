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
def test_hold_and_explicit_stop_tail_certification(obstacle):
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
    assert proof.states[-1,3] == pytest.approx(0.)
    assert proof.times[-1] > 1.8
    cmd=certified_command(result,measured,limits,command_hold_s=1.8)
    assert cmd == ((.5,0.) if obstacle is None else (0.,0.))


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
