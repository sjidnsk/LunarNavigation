from dataclasses import replace
import numpy as np
import pytest
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState, _valid_policy
from lunar_pure_wheeled_controller.mppi import MppiExecutor
from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower

@pytest.mark.parametrize("v", [.02, -.02, 1e-8, 0.])
def test_independent_yaw_capability(v):
    e=MppiPathFollower(TrackingPolicy(motion_model="differential_drive", max_lateral_accel_mps2=1e-8))
    e.note_command(v,.4)
    raw=np.array([[[v,.4]]])
    assert np.allclose(e._project_targets(raw,.05),raw)
    _,velocity=e._rollout(TrackingState(0,0,0,v,.4),raw,.05,delay=False)
    assert e._kinematically_valid(velocity).all()
    assert e._rollout_kinematic_valid.all()

@pytest.mark.parametrize("cls", [MppiExecutor,MppiPathFollower])
def test_output_bounds_and_slew(cls):
    p=TrackingPolicy(motion_model="differential_drive")
    e=cls(p)
    raw=np.array([[[10.,10.]]])
    result=e._project_targets(raw,.05)
    assert result[0,0,0]<=p.max_linear_accel_mps2*.05
    assert result[0,0,1]<=p.max_angular_accel_radps2*.05
    for _ in range(30):
        e.note_command(*result[0,0])
        result=e._project_targets(raw,.05)
    assert result[0,0,0]<=p.max_linear_mps
    assert result[0,0,1]<=p.max_angular_radps

def test_old_model_rejects_high_curvature():
    e=MppiPathFollower(TrackingPolicy())
    assert not e._kinematically_valid(np.array([[[.02,.4]]])).all()

def test_model_validation():
    assert TrackingPolicy().motion_model=="curvature_limited"
    assert _valid_policy(TrackingPolicy(motion_model="differential_drive"))
    assert not _valid_policy(TrackingPolicy(motion_model="unknown"))

def test_measured_overspeed_is_retained():
    e=MppiPathFollower(TrackingPolicy(motion_model="differential_drive"))
    state=TrackingState(0,0,0,2.,3.)
    _,vel=e._rollout(state,np.zeros((1,1,2)),.05,delay=False)
    assert state.linear_mps==2. and state.angular_radps==3.
    assert vel[0,0,0]>e.policy.max_linear_mps
    assert vel[0,0,1]>e.policy.max_angular_radps
    assert e._kinematically_valid(vel).all()


def _standard_follower(model="differential_drive", **kwargs):
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower
    from lunar_pure_wheeled_controller.mppi import MppiPolicy
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    policy=replace(TrackingPolicy(),motion_model=model,goal_position_tolerance_m=.1,
        goal_yaw_tolerance_rad=.05,stopped_angular_radps=.01,
        execution_clearance_m=.001,control_response_time_s=.001,**kwargs)
    e=StandardPathFollower(policy,MppiPolicy(batch_size=12,time_steps=12,
        nominal_response_time_s=0.))
    e.set_map(GridMap(np.ones((100,100),np.uint8),.1,(-5.,-5.)))
    return e


def test_standard_update_publishes_moving_high_curvature():
    import math
    e=_standard_follower(max_linear_mps=.03,max_reverse_mps=.03)
    assert e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    state=TrackingState(0.,0.,-.15)
    witnessed=False
    for _ in range(5):
        result=e.update(state,.05)
        v=result.command.linear_x_mps;w=result.command.angular_z_radps
        assert result.command.failure_reason is None
        assert abs(v)<=e.policy.max_linear_mps and abs(w)<=e.policy.max_angular_radps
        assert e.certify_publication(state,v,w,0.)
        witnessed |= v>.01 and w/v>1.5
        e.note_command(v,w)
        state=TrackingState(state.x_m+v*math.cos(state.yaw_rad)*.05,
            state.y_m+v*math.sin(state.yaw_rad)*.05,state.yaw_rad+w*.05,v,w)
    assert witnessed


def test_publication_certificate_obeys_motion_model():
    state=TrackingState(0.,0.,0.,.02,.4)
    assert _standard_follower().certify_publication(state,.02,.4,0.)
    assert not _standard_follower("curvature_limited").certify_publication(state,.02,.4,0.)


def test_update_overspeed_stops_with_unmodified_feedback_and_real_braking_tail():
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    e=_standard_follower()
    cells=np.ones((100,100),np.uint8)
    cells[:,55]=2  # 前方0.5m障碍；正常限速停车可达，实测2m/s停车不可达。
    e.set_map(GridMap(cells,.1,(-5.,-5.)))
    assert e.set_standard_path(((0.,0.,0.),(2.,0.,0.)))
    state=TrackingState(0.,0.,0.,2.,0.)
    assert not e.certify_publication(state,0.,0.,0.)
    assert e.certify_publication(replace(state,linear_mps=.2),0.,0.,0.)
    result=e.update(state,.05)
    assert abs(result.command.linear_x_mps)<=e.policy.max_linear_mps
    assert abs(result.command.angular_z_radps)<=e.policy.max_angular_radps
    assert state.linear_mps==2. and state.angular_radps==0.
    assert result.command.linear_x_mps==0.
    assert e.failure=="MPPI_STOPPING_UNSAFE"
