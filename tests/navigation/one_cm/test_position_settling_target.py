"""移动时使用位置收敛目标；实测已停在验收范围则完成朝向与任务。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route,VehicleState
from tests.navigation.one_cm.test_pose_goal import coordinator,world
from tests.navigation.one_cm.test_navigation_task import make_session,task_api


@pytest.mark.parametrize('asynchronous',[False,True])
def test_stopped_inside_acceptance_band_first_finishes_position_target(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    state=VehicleState(.08,0.,0.,0.,0.)
    c.replace_goal((0.,0.),goal_yaw=1.,generation=1)
    c.route=Route(1,1,[[.08,0.],[0.,0.]],(0.,0.))
    r=c.tick(world(),state,(0.,0.),0.,goal_yaw=1.)
    assert r.reference.terminal_yaw is None
    assert r.status not in ('GOAL_TURNING','ARRIVED')
    assert np.all(r.action.states[:,0]==.08)
    assert np.all(r.action.states[:,3]==0.)
    if asynchronous:assert len(c.local_worker.requests)==1
    else:assert c.planner.calls==1
    assert c.goal_tolerance_m==.1


@pytest.mark.parametrize('asynchronous',[False,True])
def test_position_settled_then_uses_existing_pure_rotation(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    r=c.tick(world(),VehicleState(.015,0.,0.,0.,0.),(0.,0.),0.,goal_yaw=1.)
    assert r.status=='GOAL_TURNING'
    assert np.all(r.action.states[:,0]==.015) and np.all(r.action.states[:,3]==0.)


@pytest.mark.parametrize('asynchronous',[False,True])
def test_observed_stop_is_not_tightened_by_quality_target_tail(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    r=c.tick(world(),VehicleState(.019999,0.,0.,.009,0.),(0.,0.),0.,goal_yaw=0.)
    assert r.status=='ARRIVED' and r.action.source=='brake'
    assert c.goal_reached(VehicleState(.019999,0.,0.,.009,0.))


def test_effective_target_cannot_relax_stricter_acceptance():
    c=coordinator();c.goal_tolerance_m=.01;c.goal_position_target_m=.02
    assert c.position_target_tolerance_m==.01


def test_task_finishes_from_certified_observed_stop_in_acceptance_band():
    session,Goal=make_session();session.coordinator.goal_position_target_m=.02
    task=task_api(session);task.submit('precision',Goal(0.,0.,0.),0.)
    session.offer_state(VehicleState(.08,0.,0.,0.,0.))
    task.observe(session.tick(0.),0.);task.observe(session.tick(.4),.4)
    assert task.active.outcome=='GOAL_REACHED'
