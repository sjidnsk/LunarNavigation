"""真实acados验证终点几何参考与局部MPCC相接，不发布车辆指令。"""
import os
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route,VehicleState,WheelLimits,WorldSnapshot,CellRect
from scripts.navigation.one_cm.nav1cm.maps import extract_window
from scripts.navigation.one_cm.nav1cm.reference import extract_reference
from scripts.navigation.one_cm.nav1cm.reference_smoothing import shape_route
from scripts.navigation.one_cm.nav1cm.certification import certify

pytestmark=pytest.mark.skipif(os.environ.get('NAV1CM_REAL_SOLVER')!='1',reason='real acados required')

@pytest.fixture(scope='module')
def real_planner():
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    limits=replace(WheelLimits(),motion_model='differential_drive',minimum_clearance_m=.2)
    return MpccPlanner(limits,Path(os.environ['NAV1CM_BUILD_DIR'])),limits

@pytest.mark.parametrize('yaw,reverse',[(np.pi/2,False),(-np.pi/2,False),(4*np.pi/3,True)])
def test_terminal_geometric_reference_has_certified_local_candidate(real_planner,yaw,reverse):
    planner,limits=real_planner
    raw=Route(7,1,[[0.,0.],[4.,0.]],(4.,0.))
    shaped=shape_route(raw,np.empty((0,4)),terminal_yaw=yaw)
    assert shaped.changed
    points=shaped.route.points
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    k=max(1,min(len(points)-2,int(np.searchsorted(arc,arc[-1]-1.4))))
    direction=points[k+1]-points[k]
    heading=float(np.arctan2(direction[1],direction[0]))+(np.pi if reverse else 0.)
    state=VehicleState(*points[k],heading,0.,0.)
    world=WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),))
    grid=extract_window(world,state,False)
    reference,_=extract_reference(shaped.route,state,grid,terminal_yaw=yaw)
    result=planner.plan(state,grid,reference)
    assert result.candidate is not None,(result.reason,result.attempts)
    candidate=result.candidate
    assert candidate.source=='mpcc:acados-sqp-rti'
    assert candidate.mode==('reverse' if reverse else 'forward')
    assert certify(candidate,grid,limits,reference.route_version).valid
    assert candidate.states[-1,5]>=reference.progress_s
