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

def test_direct_session_requests_action_and_cancel_prefix():
    session,Goal=make_session()
    session.direct_control=True
    session.offer_goal(Goal(0,2,0))
    packet=session.tick(0.)
    assert packet.result.action is not None
    session.cancel_goal()
    packet=session.tick(.05)
    assert packet.result.action is not None
    assert packet.result.action.times[-1] == pytest.approx(.05)
    assert packet.result.action_certificate.matches(packet.result.action,packet.result.grid,session.coordinator.limits,-1)


@pytest.mark.parametrize("change",['state','map','limits','version','controls'])
def test_real_certificate_refuses_changed_execution_inputs(change):
    from dataclasses import replace
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    session,Goal=make_session()
    session.direct_control=True
    session.offer_state(VehicleState(0,0,0,.1,0))
    session.cancel_goal()
    result=session.tick(0.).result
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
