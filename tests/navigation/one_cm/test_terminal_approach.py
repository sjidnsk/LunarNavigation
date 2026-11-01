"""末端平滑目标属于MPCC；行进对准不得被最终yaw污染。"""
import inspect
import math
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import LocalReference, VehicleState, WheelLimits
from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner, select_mode


def reference(yaw=1., *, progress=0., reverse=False):
    sign=-1 if reverse else 1
    return LocalReference(1,1,[[0.,0.],[sign*.8,0.]],[0.,.8],progress,yaw)


def parameters(ref, mode=None, heading=0.):
    planner=MpccPlanner.__new__(MpccPlanner)
    planner.N=20;planner.dt=.2;planner.limits=WheelLimits()
    planner.offsets=np.array([0.]);planner.radius=.01
    states=np.zeros((5,6));states[:,2]=heading;states[:,5]=np.linspace(0.,.8,5)
    field=SimpleNamespace(sample=lambda x,y:(100.,0.,0.))
    kwargs={} if mode is None else dict(mode=mode)
    return planner._parameters(states,np.zeros((4,3)),ref,field,heading,.8,**kwargs)


def test_moving_yaw_is_gradual_before_terminal_knot():
    yaw=np.array([p[5] for p in parameters(reference())])
    assert yaw[0]==pytest.approx(0.)
    assert np.all(np.diff(yaw)>0.), yaw
    assert yaw[-1]==pytest.approx(1.)
    assert yaw[1]<yaw[2]<yaw[3]<1.


def test_travel_alignment_spin_never_carries_final_yaw():
    assert 'mode' in inspect.signature(MpccPlanner._parameters).parameters
    yaw=[p[5] for p in parameters(reference(2.5),mode='spin',heading=.2)]
    assert yaw==pytest.approx([.2]*5)


@pytest.mark.parametrize('reverse',[False,True])
def test_terminal_approach_does_not_reset_to_route_tangent(reverse):
    ref=reference(1.,progress=.6,reverse=reverse)
    # 倒车朝向仍为0附近，行进切线为pi；最终车体yaw不再加pi。
    assert select_mode(VehicleState(0,0,.5,0,0),ref)==('reverse' if reverse else 'forward')


def test_wrong_side_heading_still_requires_travel_alignment():
    assert select_mode(VehicleState(0,0,-.5,0,0),reference(1.,progress=.6))=='spin'
    assert select_mode(VehicleState(0,0,1.8,0,0),reference(2.5,progress=.6))=='spin'


def test_partial_reference_does_not_invent_final_orientation():
    yaw=[p[5] for p in parameters(reference(None))]
    assert yaw==pytest.approx([0.]*5)
    assert select_mode(VehicleState(0,0,.5,0,0),reference(None,progress=.6))=='spin'


def test_reverse_final_body_yaw_is_not_shifted_by_pi():
    yaw=[p[5] for p in parameters(reference(.4,reverse=True),mode="reverse",heading=0.)]
    assert yaw[-1]==pytest.approx(.4)
    assert 0.<yaw[2]<.4


def test_shortest_rotation_crosses_pi_without_full_turn():
    heading=math.pi-.2
    ref = reference(-math.pi+.2)
    # 路线切线才是moving yaw来源，预测状态heading只决定连续角度分支。
    ref = LocalReference(1,1,[[0.,0.],[.8*math.cos(heading),.8*math.sin(heading)]],[0.,.8],0.,ref.terminal_yaw)
    yaw=np.array([p[5] for p in parameters(ref,heading=heading)])
    assert np.all(np.diff(yaw)>0.)
    assert yaw[-1]-yaw[0]==pytest.approx(.4)


def test_moving_and_angular_braking_guards_are_preserved():
    ref=reference(1.,progress=.6)
    assert select_mode(VehicleState(0,0,.5,0,.05),ref)=='spin'
    assert select_mode(VehicleState(0,0,0,-.1,0),ref)=='brake'


def test_residual_angular_motion_in_approach_is_braked_without_tangent_reset():
    from scripts.navigation.one_cm.nav1cm import local_mpcc as module
    assert hasattr(module,'travel_target_yaw')
    state=VehicleState(0,0,.5,0,.03)
    assert module.travel_target_yaw(state,reference(1.,progress=.6),'spin',.8)==pytest.approx(.84375)
    # 错侧朝向仍需对准真实路线，不因末端目标获准提前旋转。
    wrong=VehicleState(0,0,-.5,0,.03)
    assert module.travel_target_yaw(wrong,reference(1.,progress=.6),'spin',.8)==pytest.approx(0.)


@pytest.mark.parametrize('angular_speed',[0.,.03])
def test_small_tracking_overshoot_does_not_undo_terminal_turn(angular_speed):
    from scripts.navigation.one_cm.nav1cm.local_mpcc import travel_target_yaw
    state=VehicleState(0,0,.85,0,angular_speed)
    ref=reference(1.,progress=.6)
    assert select_mode(state,ref)==('spin' if angular_speed else 'forward')
    if angular_speed:
        assert travel_target_yaw(state,ref,'spin',.8)==pytest.approx(.84375)
    # 大幅超调仍须回到真实行进方向，不能无限扩大放行区间。
    assert select_mode(VehicleState(0,0,1.2,0,0),ref)=='spin'
