"""真实acados+理想模型积分；不连接车辆，不替代P3闭环验收。"""
import os,json
from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route,VehicleState,WheelLimits,WorldSnapshot,CellRect
from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
from scripts.navigation.one_cm.nav1cm.pose_goal import angle_error
from scripts.navigation.one_cm.nav1cm.execution import execute_action,action_prefix
from scripts.navigation.one_cm.nav1cm.certification import certify
from tests.navigation.one_cm.test_async_execution import PendingGlobal
pytestmark=pytest.mark.skipif(os.environ.get('NAV1CM_REAL_SOLVER')!='1',reason='real acados required')

@pytest.fixture(scope='module')
def planner():
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    limits=replace(WheelLimits(),motion_model='differential_drive',minimum_clearance_m=.2)
    return MpccPlanner(limits,Path(os.environ['NAV1CM_BUILD_DIR'])),limits

@pytest.mark.parametrize('lateral',[-.15,.15])
@pytest.mark.parametrize('yaw',[0.,np.pi])
def test_real_solver_corrects_side_offset_then_final_yaw(planner,lateral,yaw):
    mpcc,limits=planner
    c=NavigationCoordinator(mpcc,runner=PendingGlobal(),limits=limits,allow_unknown=True,local_period_s=.2)
    c.goal_position_target_m=.02
    goal=(0.,0.);c._set_goal(goal,0.)
    c.route=Route(1,1,[[-1.,0.],goal],goal)
    w=WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),))
    state=VehicleState(0.,lateral,yaw,0.,0.)
    trace=[]
    for index in range(450):
        result=c.tick(w,state,goal,index*.2,goal_yaw=0.,need_action=False)
        assert result.candidate is not None,(index,result.status,result.events)
        assert result.certificate.matches(result.candidate,result.grid,limits,result.reference.route_version)
        action=action_prefix(result.candidate,state,.2)
        assert action.times[-1]==pytest.approx(.2)
        proof=certify(action,result.grid,limits,result.reference.route_version)
        assert proof.matches(action,result.grid,limits,result.reference.route_version)
        trace.append([index*.2,state.x,state.y,state.yaw,state.v,state.w,result.status])
        state=execute_action(action,state)
        if result.status=='ARRIVED':break
    out=os.environ.get('NAV1CM_TEST_ARTIFACT')
    if out:
        path=Path(out);path.mkdir(exist_ok=True,parents=True)
        (path/f'side-{lateral}-yaw-{yaw}.json').write_text(json.dumps(trace),encoding='utf-8')
    assert result.status=='ARRIVED',(state,result.status,trace[-5:])
    assert np.hypot(state.x,state.y)<=c.goal_tolerance_m
    assert abs(angle_error(0.,state.yaw))<=c.goal_yaw_tolerance_rad
    assert abs(state.v)<=c.stopped_linear_mps and abs(state.w)<=c.stopped_angular_rps
    assert any(np.hypot(row[1],row[2])<abs(lateral)-.03 for row in trace)
