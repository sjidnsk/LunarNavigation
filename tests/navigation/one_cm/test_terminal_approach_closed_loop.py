"""真实MPCC→标准Path MPPI短组件闭环；不启动ROS/训练或实车。"""
import math
import os
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytestmark=pytest.mark.skipif(os.environ.get('NAV1CM_REAL_SOLVER')!='1',reason='real acados required')


@pytest.mark.parametrize('initial,goal,direction',[
    ((.08306,1.72503,1.777,0.,0.),(.1,1.9,-2.356194490192345),1),
    ((0.,0.,0.,0.,0.),(.6,0.,.65),1),
    ((0.,0.,0.,0.,0.),(-.6,0.,.65),-1),
])
def test_real_mpcc_standard_path_reaches_position_and_final_yaw(initial,goal,direction):
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits,VehicleState,GridSnapshot,Route
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.reference import extract_reference
    from scripts.navigation.one_cm.nav1cm.maps import build_distance_field
    from scripts.navigation.one_cm.nav1cm.certification import certify
    from scripts.navigation.one_cm.nav1cm.pose_goal import angle_error
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower
    from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState
    from lunar_pure_wheeled_controller.mppi import MppiPolicy
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    limits=WheelLimits(angular_speed_rps=1.,minimum_clearance_m=.2)
    planner=MpccPlanner(limits,Path(os.environ['NAV1CM_BUILD_DIR']))
    origin=(initial[0]-2.5,initial[1]-2.5)
    grid=GridSnapshot(9,origin,.01,np.zeros((500,500),np.int8))
    field=build_distance_field(grid)
    policy=replace(TrackingPolicy(),max_angular_radps=1.,goal_position_tolerance_m=.1,
                   goal_yaw_tolerance_rad=.05,stopped_angular_radps=.01)
    follower=StandardPathFollower(policy,MppiPolicy(nominal_response_time_s=.15))
    follower.set_map(GridMap(np.ones((500,500),np.uint8),.01,origin))
    coordinator=NavigationCoordinator(planner,runner=SimpleNamespace(),limits=limits)
    coordinator.replace_goal(goal[:2],goal_yaw=goal[2],generation=1)
    route=Route(1,9,[initial[:2],goal[:2]],goal[:2])
    state=np.array(initial);progress=None;dt=.05;translations=[];mixed_motion=False;terminal_seen=False
    for i in range(900):
        measured=VehicleState(*state)
        if i%2==0:
            distance=float(np.linalg.norm(state[:2]-goal[:2]))
            if distance<=coordinator.goal_tolerance_m:
                from scripts.navigation.one_cm.nav1cm.contracts import LocalReference
                ref=LocalReference(1,9,[state[:2]],[0.],0.,goal[2])
                candidate,status=coordinator._terminal_plan(measured,grid,ref)
                terminal_seen|=status=='GOAL_TURNING'
            else:
                ref,progress=extract_reference(route,measured,grid,progress,terminal_yaw=goal[2])
                result=planner.plan(measured,grid,ref,distance_field=field,field_grid_digest=grid.digest())
                candidate=result.candidate
                assert candidate is not None,(i,state,result.reason)
            certificate=certify(candidate,grid,limits,1)
            assert certificate.valid,(i,certificate.reason)
            follower.set_standard_path(candidate.states[:,:3].tolist(),identity=i)
        result=follower.update(TrackingState(*state),dt,now_s=(i+1)*dt)
        assert follower.failure is None,(i,follower.failure,state)
        command=np.array([result.command.linear_x_mps,result.command.angular_z_radps])
        follower.note_command(*command,stamp_s=(i+1)*dt)
        v=state[3]+float(np.clip(command[0]-state[3],-.5*dt,.5*dt))
        w=state[4]+float(np.clip(command[1]-state[4],-.5*dt,.5*dt))
        yaw=state[2]+w*dt
        if abs(w)<1e-12:dx,dy=v*dt*math.cos(state[2]),v*dt*math.sin(state[2])
        else:dx,dy=v/w*(math.sin(yaw)-math.sin(state[2])),v/w*(math.cos(state[2])-math.cos(yaw))
        state=np.array([state[0]+dx,state[1]+dy,yaw,v,w])
        if abs(v)>1e-4:translations.append(v)
        if abs(v)>1e-3 and abs(w)>1e-3 and np.linalg.norm(state[:2]-goal[:2])>.1:
            mixed_motion=True
        if np.linalg.norm(state[:2]-goal[:2])<=.1 and abs(angle_error(goal[2],state[2]))<=.05 and max(abs(v),abs(w))<=.01:
            break
    else:pytest.fail(f'45s component did not arrive: state={state}, goal={goal}, terminal_seen={terminal_seen}')
    assert any(v*direction>1e-3 for v in translations)
    assert mixed_motion, 'approach must adjust yaw during translation, not only spin at goal'
    assert terminal_seen, 'remaining final yaw must use the existing bounded terminal planner'
