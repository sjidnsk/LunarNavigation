"""标准Path控制序列应由弧长与车体yaw推导，进入同一个优化/认证过程。"""
from dataclasses import replace
import math
import numpy as np
import pytest
from test_standard_path_follower import follower
from lunar_pure_wheeled_controller.mppi import MppiPolicy
from lunar_pure_wheeled_controller.tracking import TrackingState


def curved_reference(direction=1.,curvature=1.):
    arcs=np.linspace(0.,.1,31)
    return tuple((direction*math.sin(curvature*s)/curvature,
                  direction*(1-math.cos(curvature*s))/curvature,curvature*s) for s in arcs)


@pytest.mark.parametrize('direction',[1.,-1.])
def test_reference_seed_couples_speed_and_body_yaw_in_both_directions(direction):
    e=follower(motion_model='differential_drive');e.config=MppiPolicy();e._nominal=np.zeros((40,2));e.policy=replace(e.policy,goal_position_tolerance_m=.005)
    e.set_standard_path(curved_reference(direction));state=TrackingState(0.,0.,0.)
    e._accept_pending(state);e.direction=int(direction);e.heading_error=0.;e.progress_m=0.;e.phase='TRACKING'
    seed=e._seed_sequence(state,np.array([direction*.2,.075]),False)
    assert seed.shape==(40,2)
    assert np.all(direction*seed[:,0]>0.)
    np.testing.assert_allclose(seed[:,1],abs(seed[:,0]),rtol=1e-4,atol=1e-6)
    target=np.array(e.path.points[-1])
    score,_=e._score(state,seed[None],target,.1,.05)
    stop,_=e._score(state,np.zeros((1,40,2)),target,.1,.05)
    assert score[0]<stop[0]*.25


def test_reference_seed_retains_spin_and_curvature_limits():
    state=TrackingState(0.,0.,0.)
    e=follower(motion_model='differential_drive');e.set_standard_path(((0.,0.,0.),(0.,0.,1.)))
    e._accept_pending(state)
    spin=e._seed_sequence(state,np.array([0.,.4]),True)
    np.testing.assert_array_equal(spin,np.tile([0.,.4],(e.config.time_steps,1)))
    e=follower(motion_model='curvature_limited');e.set_standard_path(curved_reference(curvature=4.))
    e._accept_pending(state);e.direction=1;e.heading_error=0.
    seed=e._seed_sequence(state,np.array([.2,.2]),False)
    assert np.all(abs(seed[:,1])<=abs(seed[:,0])*e.policy.max_curvature_per_m+1e-12)


def test_rolling_warm_start_and_reference_proposal_share_weighted_solver(monkeypatch):
    e=follower(motion_model='differential_drive');e.config=MppiPolicy();e._nominal=np.zeros((40,2));e.policy=replace(e.policy,goal_position_tolerance_m=.005)
    e.set_standard_path(curved_reference());state=TrackingState(0.,0.,0.)
    e._accept_pending(state);e._nominal[:]=[.0001,0.]
    warm=e._nominal.copy();samples=[];certificates=[];score=e._score;certify=e._certify_sequences
    def record_score(*args):samples.append(args[1].copy());return score(*args)
    def record_certificate(*args,**kwargs):certificates.append(args[1].copy());return certify(*args,**kwargs)
    monkeypatch.setattr(e,'_score',record_score);monkeypatch.setattr(e,'_certify_sequences',record_certificate)
    result=e.update(state,.05)
    assert result.phase=='TRACKING' and result.command.linear_x_mps>0.005
    np.testing.assert_array_equal(samples[0][0],warm)
    assert samples[0][1,0,1]>0. and samples[0][1,0,0]<.1
    assert e.diagnostics['weighted_updates']==1 and certificates
    # 没有通过认证的参考控制不得发布；仍沿用原失败/制动路径。
    e.failure=None;e._brake_safe=lambda *args:np.zeros(len(args[1]),bool)
    blocked=e.update(state,.05)
    assert blocked.command.linear_x_mps==0. and blocked.command.angular_z_radps==0.


def test_reference_yaw_wrap_does_not_generate_reverse_spin():
    from lunar_pure_wheeled_controller.path_geometry import angle
    yaw=math.pi-.02;c,s=math.cos(yaw),math.sin(yaw)
    points=tuple((c*x-s*y,s*x+c*y,angle(yaw+a)) for x,y,a in curved_reference())
    e=follower(motion_model='differential_drive');e.set_standard_path(points)
    state=TrackingState(0.,0.,yaw);e._accept_pending(state);e.direction=1;e.heading_error=0.
    seed=e._seed_sequence(state,np.array([.2,.1]),False)
    assert np.all(seed[:,1]>0.) and np.max(seed[:,1])<.5


def test_endpoint_correction_preserves_existing_seed():
    e=follower(motion_model='differential_drive');e.set_standard_path(((0.,0.,0.),(.1,0.,0.)))
    state=TrackingState(.1,.02,0.);e._accept_pending(state);e.progress_m=e.path.length
    seed=np.array([0.,-.4])
    np.testing.assert_array_equal(e._seed_sequence(state,seed,False),np.tile(seed,(e.config.time_steps,1)))
