"""保留全部延迟场景；完全相同的物理轨迹只做一次几何认证。"""
import math
import numpy as np
import pytest
from lunar_pure_wheeled_controller.mppi import MppiExecutor
from lunar_pure_wheeled_controller.mppi_map import GridMap
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState


def setup(history=False):
    e=MppiExecutor(TrackingPolicy(motion_model='differential_drive'))
    e._state_age_s=.15;e._model_time=1.;e._first_target_step=.05
    if history:e._command_history=[(.2,np.array([.2,.1])),(.98,np.array([0.,0.]))]
    return e


def full_certificate(e,state,first,dt):
    p=e.policy;latency=math.ceil(p.control_response_time_s/dt)
    wait=math.ceil((e._state_age_s+e.config.compute_budget_s)/dt)
    hold=latency+wait+1
    stop=math.ceil(max(p.max_linear_mps,p.max_reverse_mps,abs(state.linear_mps))/p.max_linear_decel_mps2/dt)+math.ceil(max(p.max_angular_radps,abs(state.angular_radps))/p.max_angular_accel_radps2/dt)+2
    u=np.zeros((len(first),latency+wait+hold+stop,2));u[:,:hold]=first[:,None,:]
    scenarios=[(l,q) for l in range(latency+1) for q in range(wait+1)]
    poses,actual=e._rollout(state,np.tile(u,(len(scenarios),1,1)),dt,
        delay_ticks=np.repeat([l+q for l,q in scenarios],len(first)),
        history_delay_ticks=np.repeat([l for l,q in scenarios],len(first)),project=False)
    valid=e._stationary_supported(state,poses,actual)
    remaining=~valid
    if remaining.any():valid[remaining]=e.map.valid_motion(poses[remaining],actual[remaining],dt,p.execution_clearance_m)
    valid &= e._rollout_kinematic_valid
    valid &= (abs(actual[:,-1,0])<=p.stopped_linear_mps)&(abs(actual[:,-1,1])<=p.stopped_angular_radps)
    return valid.reshape(len(scenarios),len(first)).all(axis=0)


def test_identical_delay_outcomes_share_geometry_without_skipping_scenarios(monkeypatch):
    e=setup();e.set_map(GridMap(np.ones((100,100),np.uint8),.05,(-2.5,-2.5)))
    checked=[];original=GridMap.valid_motion
    def record(self,poses,*a):checked.append(len(poses));return original(self,poses,*a)
    monkeypatch.setattr(GridMap,'valid_motion',record)
    assert e._brake_safe(TrackingState(0,0,0),np.array([[0.,0.]]),.05)[0]
    assert checked==[1]


@pytest.mark.parametrize('history',[False,True])
@pytest.mark.parametrize('speed,lateral',[(0.,0.),(.13,0.),(-.1,.02)])
@pytest.mark.parametrize('surface',['free','obstacle','unknown','assumed'])
def test_shared_geometry_matches_full_delay_enumeration(history,speed,lateral,surface):
    e=setup(history);states=np.ones((60,60),np.uint8);assumed=np.zeros_like(states,bool)
    if surface=='obstacle':states[30,31]=2
    if surface=='unknown':states[30,31]=0
    if surface=='assumed':states[30,30]=0;assumed[30,30]=True
    e.set_map(GridMap(states,.05,(-1.5,-1.5),assumed=assumed))
    state=TrackingState(0,0,.2,speed,.07,lateral)
    first=np.array([[0.,0.],[.1,.1],[-.1,-.1],[0.,0.]])
    expected=full_certificate(e,state,first,.05)
    np.testing.assert_array_equal(e._brake_safe(state,first,.05),expected)


def test_identical_geometry_keeps_each_original_kinematic_verdict(monkeypatch):
    e=setup();e.set_map(GridMap(np.ones((100,100),np.uint8),.05,(-2.5,-2.5)))
    def kinematic(state,tv,tw,actual,dt):
        flags=np.ones(len(actual),bool);flags[1::2]=False
        return flags
    monkeypatch.setattr(e,'_continuous_kinematics',kinematic)
    np.testing.assert_array_equal(e._brake_safe(TrackingState(0,0,0),np.zeros((2,2)),.05),[True,False])
