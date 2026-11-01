"""带符号速度的加速/制动语义：硬编码解析真值与连续折点。"""
from dataclasses import replace
import numpy as np
import pytest
from lunar_pure_wheeled_controller.mppi import MppiExecutor,MppiPolicy
from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState


def executor(cls=MppiExecutor,model='differential_drive'):
    p=replace(TrackingPolicy(),motion_model=model,max_linear_accel_mps2=.1,max_linear_decel_mps2=.5)
    e=cls(p,MppiPolicy(batch_size=8,time_steps=3,model_dt=.1,nominal_response_time_s=0.))
    e._first_target_step=.1
    return e


@pytest.mark.parametrize('current,target,expected',[
    (0.,.1,.01),(0.,-.1,-.01),(.1,0.,.05),(-.1,0.,-.05),
    (.1,.15,.11),(-.1,-.15,-.11),(.01,-.2,-.008),(-.01,.2,.008),
    (.1,-.2,.05),(-.1,.2,-.05)])
@pytest.mark.parametrize('route',['command','physical'])
def test_asymmetric_speed_change_uses_magnitude_and_zero_crossing(current,target,expected,route):
    e=executor()
    if route=='command':
        value=e._limit_target_step(np.array([[current,0.]]),np.array([[target,0.]]),.1)[0,0]
    else:
        _,actual=e._rollout(TrackingState(0.,0.,0.,current),np.array([[[target,0.]]]),.1,delay=False,project=False)
        value=actual[0,0,0]
    assert value==pytest.approx(expected,abs=1e-12)


@pytest.mark.parametrize('model',['differential_drive','curvature_limited'])
@pytest.mark.parametrize('cls',[MppiExecutor,MppiPathFollower])
@pytest.mark.parametrize('sign',[1.,-1.])
def test_batch_projection_and_first_command_share_reversal_rule(model,cls,sign):
    e=executor(cls,model);e._last_command=np.array([sign*.01,0.])
    targets=np.array([[[-sign*.2,0.]]*3])
    projected=e._project_targets(targets,.1)
    np.testing.assert_allclose(projected[0,:,0],-sign*np.array([.008,.018,.028]),atol=1e-12)
    first=e._first_commands(TrackingState(0.,0.,0.),targets[:,0],.1)
    np.testing.assert_array_equal(first,projected[:,0])


@pytest.mark.parametrize('sign',[1.,-1.])
def test_continuous_certificate_checks_actual_asymmetric_zero_time(sign):
    e=executor(MppiPathFollower,'curvature_limited')
    state=TrackingState(0.,0.,0.,sign*.01,.001)
    tv=np.array([[-sign*.2]]);tw=np.array([[.001]])
    # .02秒刹到零，剩余.02秒反向加速；两个端点都满足曲率，内部零速却不满足。
    actual=np.array([[[-sign*.002,.001]]])
    assert abs(state.angular_radps)<=abs(state.linear_mps)
    assert abs(actual[0,0,1])<=abs(actual[0,0,0])
    assert not e._continuous_kinematics(state,tv,tw,actual,.04)[0]



@pytest.mark.parametrize('accel,brake',[(.1,.5),(.5,.1),(.5,.5)])
def test_speed_slew_matches_independent_fine_actuator_and_time_partition(accel,brake):
    from lunar_pure_wheeled_controller.speed_slew import speed_slew
    rng=np.random.default_rng(20261031)
    current=rng.uniform(-.2,.2,128);target=rng.uniform(-.2,.2,128)
    result=speed_slew(current,target,.3,accel,brake)
    half=speed_slew(current,target,.15,accel,brake)
    np.testing.assert_allclose(result,speed_slew(half,target,.15,accel,brake),atol=1e-15,rtol=0.)
    mirrored=speed_slew(-current,-target,.3,accel,brake)
    np.testing.assert_array_equal(mirrored,-result)
    # 独立细步执行器：先刹至零，下一细步才反向加速。量化误差不超过一个细步。
    fine=current.copy();dt=.0001
    for _ in range(3000):
        reversing=fine*target<0.
        rate=np.where(reversing,brake,np.where(abs(target)>abs(fine),accel,brake))
        delta=np.sign(target-fine)*np.minimum(abs(target-fine),rate*dt)
        next_v=fine+delta
        fine=np.where(reversing&(next_v*fine<=0.),0.,next_v)
    np.testing.assert_allclose(result,fine,atol=max(accel,brake)*dt+1e-12,rtol=0.)


def test_reversal_can_be_certified_when_angular_velocity_stops_at_same_zero():
    e=executor(MppiPathFollower,'curvature_limited')
    e.policy=replace(e.policy,max_angular_accel_radps2=.05)
    state=TrackingState(0.,0.,0.,.01,.001)
    actual=np.array([[[-.002,0.]]])
    assert e._continuous_kinematics(state,np.array([[-.2]]),np.array([[0.]]),actual,.04)[0]


def test_delayed_committed_target_uses_measured_velocity_not_last_command():
    e=executor();e.note_command(.2,0.,-1.)
    _,actual=e._rollout(TrackingState(0.,0.,0.,-.1),np.array([[[-.2,0.]]*2]),.1,
        delay_ticks=np.array([1]),history_delay_ticks=np.array([0]),project=False)
    assert actual[0,0,0]==pytest.approx(-.05)
    assert actual[0,1,0]==pytest.approx(-.06)


def test_measured_overspeed_is_braked_without_clipping_measurement():
    e=executor()
    _,actual=e._rollout(TrackingState(0.,0.,0.,.3),np.array([[[0.,0.]]]),.1,delay=False,project=False)
    assert actual[0,0,0]==pytest.approx(.25)
