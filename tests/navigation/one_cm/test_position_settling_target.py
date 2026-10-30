"""验收范围不能替代继续收敛到目标位置的执行目标。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route,VehicleState
from tests.navigation.one_cm.test_pose_goal import coordinator,world
from tests.navigation.one_cm.test_navigation_task import make_session,task_api


@pytest.mark.parametrize('asynchronous',[False,True])
def test_inside_acceptance_band_still_runs_local_position_planning(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    state=VehicleState(.08,0.,0.,0.,0.)
    c.replace_goal((0.,0.),goal_yaw=1.,generation=1)
    c.route=Route(1,1,[[.08,0.],[0.,0.]],(0.,0.))
    r=c.tick(world(),state,(0.,0.),0.,goal_yaw=1.)
    assert r.status not in ('GOAL_TURNING','GOAL_BRAKING','ARRIVED')
    assert len(r.reference.points)>=2
    if asynchronous:assert c.local_worker.requests
    else:assert c.planner.calls==1
    assert c.goal_tolerance_m==.1


@pytest.mark.parametrize('asynchronous',[False,True])
def test_position_settled_then_uses_existing_pure_rotation(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    r=c.tick(world(),VehicleState(.015,0.,0.,0.,0.),(0.,0.),0.,goal_yaw=1.)
    assert r.status=='GOAL_TURNING'
    assert np.all(r.action.states[:,0]==.015) and np.all(r.action.states[:,3]==0.)


@pytest.mark.parametrize('asynchronous',[False,True])
def test_braking_endpoint_must_remain_inside_quality_target(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    r=c.tick(world(),VehicleState(.019999,0.,0.,.009,0.),(0.,0.),0.,goal_yaw=0.)
    assert r.status=='GOAL_BRAKING' and r.action.source=='brake'


def test_effective_target_cannot_relax_stricter_acceptance():
    c=coordinator();c.goal_tolerance_m=.01;c.goal_position_target_m=.02
    assert c.position_target_tolerance_m==.01


def test_task_cannot_finish_in_outer_band_without_execution_sideband():
    session,Goal=make_session();session.coordinator.goal_position_target_m=.02
    task=task_api(session);task.submit('precision',Goal(0.,0.,0.),0.)
    session.offer_state(VehicleState(.08,0.,0.,0.,0.))
    task.observe(session.tick(0.),0.);task.observe(session.tick(.4),.4)
    assert task.active.outcome is None
    session.offer_state(VehicleState(.015,0.,0.,0.,0.))
    task.observe(session.tick(.5),.5);task.observe(session.tick(.9),.9)
    assert task.active.outcome=='GOAL_REACHED'
