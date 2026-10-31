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
