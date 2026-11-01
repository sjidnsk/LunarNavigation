"""终点侧偏不能因沿线投影到头而丢失位置修正。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route,VehicleState
from tests.navigation.one_cm.test_pose_goal import coordinator,world

@pytest.mark.parametrize('asynchronous',[False,True])
@pytest.mark.parametrize('lateral',[-.15,.15])
def test_sideways_endpoint_still_submits_position_reference(asynchronous,lateral):
    c=coordinator(asynchronous)
    goal=(0.,0.)
    c.tick(world(),VehicleState(-1.,0.,0.,0.,0.),goal,0.,goal_yaw=0.)
    c.route=Route(1,1,[[-1.,0.],goal],goal)
    before=c.route
    state=VehicleState(0.,lateral,0.,0.,0.)
    result=c.tick(world(),state,goal,.05,goal_yaw=0.)
    assert len(result.reference.points)>=2
    np.testing.assert_array_equal(result.reference.points[0],[state.x,state.y])
    np.testing.assert_array_equal(result.reference.points[-1],goal)
    assert result.reference.terminal_yaw is None
    assert c.route is before and c._goal_yaw==0.
    assert result.reference.s[-1]-result.reference.progress_s==pytest.approx(abs(lateral))
    if asynchronous:
        assert c.local_worker.requests
    else:
        assert c.planner.calls==1


@pytest.mark.parametrize('asynchronous',[False,True])
def test_ordinary_travel_keeps_original_route_reference(asynchronous):
    c=coordinator(asynchronous);goal=(2.,0.)
    c.tick(world(),VehicleState(0.,0.,0.,0.,0.),goal,0.,goal_yaw=1.)
    c.route=Route(1,1,[[0.,0.],goal],goal)
    result=c.tick(world(),VehicleState(.5,.15,0.,0.,0.),goal,.05,goal_yaw=1.)
    np.testing.assert_allclose(result.reference.points[0],[.5,0.])
    assert result.reference.terminal_yaw==1.

@pytest.mark.parametrize('asynchronous',[False,True])
def test_near_goal_does_not_shortcut_unfinished_route(asynchronous):
    from scripts.navigation.one_cm.nav1cm.contracts import ProgressToken
    c=coordinator(asynchronous);goal=(0.,0.)
    c.tick(world(),VehicleState(-1.,0.,0.,0.,0.),goal,0.,goal_yaw=1.)
    c.route=Route(1,1,[[-1.,0.],[-1.,1.],[0.,1.],goal],goal)
    c.progress=ProgressToken(1,0.)
    result=c.tick(world(),VehicleState(0.,.15,0.,0.,0.),goal,.05,goal_yaw=1.)
    assert result.reference.terminal_yaw==1.
    assert result.reference.s[-1]-result.reference.progress_s>.8

@pytest.mark.parametrize('asynchronous',[False,True])
@pytest.mark.parametrize('unknown',[False,True])
def test_shortcut_respects_blocked_and_unknown_policy(asynchronous,unknown):
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits,CellRect,WorldSnapshot
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
    from tests.navigation.one_cm.test_coordinator import RejectPlanner,Runner
    from tests.navigation.one_cm.test_async_execution import ManualLocal
    limits=WheelLimits(body_length_m=.1,body_width_m=.1,motion_model='differential_drive')
    options=dict(limits=limits,clearance_radius_m=.05,allow_unknown=False)
    c=(AsyncNavigationCoordinator(local_worker=ManualLocal(),global_runner=Runner(),**options) if asynchronous else
       NavigationCoordinator(RejectPlanner(),runner=Runner(),**options))
    hole=CellRect(8,18,12,22)
    free=(CellRect(-100,-100,8,100),CellRect(12,-100,100,100),CellRect(8,-100,12,18),CellRect(8,22,12,100))
    w=WorldSnapshot(1,() if unknown else (hole,),free)
    goal=(0.,0.)
    c.tick(w,VehicleState(-.2,.4,0.,0.,0.),goal,0.,goal_yaw=0.)
    c.route=Route(1,1,[[-.2,.4],[-.2,0.],goal],goal)
    result=c.tick(w,VehicleState(.2,.4,0.,0.,0.),goal,.05,goal_yaw=0.)
    np.testing.assert_allclose(result.reference.points[0],[-.2,.4])
    np.testing.assert_allclose(result.reference.points[-2],[-.2,0.])
    assert result.reference.terminal_yaw is None

@pytest.mark.parametrize('asynchronous',[False,True])
def test_after_position_acceptance_existing_final_yaw_remains(asynchronous):
    c=coordinator(asynchronous)
    result=c.tick(world(),VehicleState(0.,.09,0.,0.,0.),(0.,0.),0.,goal_yaw=1.)
    assert len(result.reference.points)==1 and result.reference.terminal_yaw==1.
    assert result.action.source.startswith('terminal-yaw:bounded')

@pytest.mark.parametrize('lateral',[-.15,.15])
@pytest.mark.parametrize('mode',['forward','reverse'])
def test_mpcc_shooting_yaws_follow_position_connection(lateral,mode):
    from types import SimpleNamespace
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner,angle_error
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    c=coordinator(False);goal=(0.,0.)
    c.tick(world(),VehicleState(-1.,0.,0.,0.,0.),goal,0.,goal_yaw=0.)
    c.route=Route(1,1,[[-1.,0.],goal],goal)
    reference=c.tick(world(),VehicleState(0.,lateral,0.,0.,0.),goal,.05,goal_yaw=0.).reference
    planner=object.__new__(MpccPlanner);planner.limits=WheelLimits();planner.offsets=(0.,);planner.radius=.6
    states=np.zeros((3,6));states[:,5]=np.linspace(reference.s[0],reference.s[-1],3)
    parameters=planner._parameters(states,np.zeros((2,3)),reference,SimpleNamespace(sample=lambda x,y:(2.,0.,0.)),0.,reference.s[-1],mode=mode)
    expected=np.arctan2(-lateral,0.)+(np.pi if mode=='reverse' else 0.)
    assert all(abs(angle_error(p[5],expected))<1e-12 for p in parameters)


@pytest.mark.parametrize('asynchronous',[False,True])
def test_terminal_heading_retains_position_completion_until_acceptance_boundary(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    goal=(0.,0.);w=world()
    first=c.tick(w,VehicleState(.015,0.,0.,0.,0.),goal,0.,goal_yaw=1.)
    assert first.reference.terminal_yaw==1.
    for index,distance in enumerate((.03,.09,.1),1):
        result=c.tick(w,VehicleState(distance,0.,0.,.02,.02),goal,.05*index,goal_yaw=1.)
        assert result.reference.terminal_yaw==1.
    result=c.tick(w,VehicleState(.101,0.,0.,0.,0.),goal,.2,goal_yaw=1.)
    assert result.reference.terminal_yaw is None
    result=c.tick(w,VehicleState(.09,0.,0.,0.,0.),goal,.25,goal_yaw=1.)
    assert result.reference.terminal_yaw is None


@pytest.mark.parametrize('asynchronous',[False,True])
def test_stopped_near_acceptance_edge_does_not_start_heading_before_position_target(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    result=c.tick(world(),VehicleState(.099,0.,0.,0.,0.),(0.,0.),0.,goal_yaw=1.)
    assert result.reference.terminal_yaw is None


@pytest.mark.parametrize('asynchronous',[False,True])
def test_already_satisfied_pose_keeps_existing_acceptance(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    result=c.tick(world(),VehicleState(.09,0.,1.,0.,0.),(0.,0.),0.,goal_yaw=1.)
    assert result.status=='ARRIVED'


@pytest.mark.parametrize('asynchronous',[False,True])
@pytest.mark.parametrize('change',['xy','yaw','generation','cancel'])
def test_completed_position_is_not_inherited_by_new_goal(asynchronous,change):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    goal=(0.,0.);yaw=1.
    c.tick(world(),VehicleState(.01,0.,0.,0.,0.),goal,0.,goal_yaw=yaw)
    if change=='xy':goal=(.01,0.)
    elif change=='yaw':yaw=1.5
    elif change=='generation':c._set_goal(goal,yaw,generation=17)
    else:c.invalidate_goal(generation=18)
    result=c.tick(world(),VehicleState(.07,0.,0.,.02,0.),goal,.05,goal_yaw=yaw)
    assert result.reference.terminal_yaw is None


@pytest.mark.parametrize('asynchronous',[False,True])
def test_xy_only_goal_preserves_existing_position_and_braking_decisions(asynchronous):
    c=coordinator(asynchronous);c.goal_position_target_m=.02
    r=c.tick(world(),VehicleState(.09,0.,0.,0.,.02),(0.,0.),0.)
    assert r.status=='GOAL_BRAKING'
    c.tick(world(),VehicleState(.01,0.,0.,0.,0.),(0.,0.),.05)
    r=c.tick(world(),VehicleState(.09,0.,0.,.03,0.),(0.,0.),.1)
    assert r.status not in ('GOAL_BRAKING','ARRIVED')
