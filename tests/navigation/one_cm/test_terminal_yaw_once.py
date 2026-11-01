"""同一目标的位置阶段只完成一次，后续漂移不阻止朝向与实测完成。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import VehicleState
from tests.navigation.one_cm.test_pose_goal import coordinator,world
from tests.navigation.one_cm.test_navigation_task import make_session,task_api

@pytest.mark.parametrize('asynchronous',[False,True])
@pytest.mark.parametrize('offset',[.12,.30,1.0])
def test_position_completion_does_not_reopen_after_drift(asynchronous,offset):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    near=VehicleState(.015,0.,0.,0.,0.)
    first=c.tick(world(),near,(0.,0.),0.,goal_yaw=1.,observed_state=near)
    assert first.status=='GOAL_TURNING'
    drifted=VehicleState(offset,0.,.5,0.,0.)
    result=c.tick(world(),drifted,(0.,0.),.05,goal_yaw=1.,observed_state=drifted)
    assert c._position_completed
    assert result.reference.terminal_yaw==1.
    assert result.status=='GOAL_TURNING'
    assert np.all(result.action.states[:,3]==0.)
    final=VehicleState(offset,0.,1.,0.,0.)
    result=c.tick(world(),final,(0.,0.),.1,goal_yaw=1.,observed_state=final)
    assert result.status=='ARRIVED' and c.goal_reached(final)


def test_task_completes_yaw_after_actual_position_drift():
    session,Goal=make_session();session.direct_control=True
    session.coordinator.goal_position_target_m=.02
    task=task_api(session);task.submit('one-way',Goal(0.,0.,1.),0.)
    session.offer_state(VehicleState(.015,0.,0.,0.,0.))
    task.observe(session.tick(0.),0.)
    session.offer_state(VehicleState(.30,0.,1.,0.,0.))
    task.observe(session.tick(.1),.1)
    assert task.active.outcome is None
    task.observe(session.tick(.5),.5)
    assert task.active.outcome=='GOAL_REACHED'


def test_unfinished_position_cannot_be_skipped_by_matching_yaw():
    c=coordinator();c.goal_position_target_m=.02
    far=VehicleState(.30,0.,1.,0.,0.)
    c.tick(world(),far,(0.,0.),0.,goal_yaw=1.,observed_state=far)
    assert not c._position_completed and not c.goal_reached(far)
