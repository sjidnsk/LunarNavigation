"""P3实测运动决定预测速度；已发命令不能冒充真实运动。"""
import importlib.util
from pathlib import Path
import sys
import pytest
CORE=Path(__file__).resolve().parents[3]/'scripts/navigation/one_cm'
sys.path.insert(0,str(CORE))
from nav1cm.contracts import VehicleState,WheelLimits
PREDICT=CORE/'ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/command_prediction.py'
spec=importlib.util.spec_from_file_location('observed_prediction_test',PREDICT)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
CommandPrediction=module.CommandPrediction


def correct(p,state,source_s,now=None):
    now=source_s if now is None else now
    p.correct(state,int(source_s*1e9),now,int(now*1e9))


def test_command_cannot_invent_acceleration_from_stationary_pose():
    p=CommandPrediction();p.issue(0.,(.2,.1));correct(p,VehicleState(0,0,0,0,0),0.)
    s=p.predict(3.,WheelLimits())
    assert s.x==0. and s.yaw==0.


def test_pose_difference_recovers_motion_when_reported_twist_stays_zero():
    p=CommandPrediction();p.issue(0.,(.2,0.));correct(p,VehicleState(0,0,0,0,0),0.)
    correct(p,VehicleState(.05,0,0,0,0),1.)
    s=p.predict(3.,WheelLimits())
    assert s.x==pytest.approx(.15) and s.v==pytest.approx(.05)
    assert p.state.v==0.  # 原P3观测不被估计值覆盖。


def test_pose_difference_uses_source_time_and_ignores_duplicate_stamp():
    p=CommandPrediction();p.issue(0.,(.2,0.));correct(p,VehicleState(0,0,0,0,0),0.)
    correct(p,VehicleState(.05,0,0,0,0),1.,3.)
    before=p.predict(3.,WheelLimits())
    correct(p,VehicleState(9,9,0,0,0),1.,4.)
    assert p.predict(3.,WheelLimits())==before
    assert before.x==pytest.approx(.15)


def test_angular_prediction_uses_observed_rate_and_shortest_angle():
    import math
    p=CommandPrediction();p.issue(0.,(0.,.1))
    correct(p,VehicleState(0,0,math.pi-.01,0,0),0.)
    correct(p,VehicleState(0,0,-math.pi+.01,0,0),1.)
    s=p.predict(3.,WheelLimits())
    assert s.w==pytest.approx(.02)
    assert math.atan2(math.sin(s.yaw-(-math.pi+.05)),math.cos(s.yaw-(-math.pi+.05)))==pytest.approx(0.)


def test_stop_or_reverse_does_not_predict_unobserved_reverse_acceleration():
    p=CommandPrediction();p.issue(0.,(.2,0.));correct(p,VehicleState(0,0,0,.05,0),0.)
    correct(p,VehicleState(.05,0,0,.05,0),1.)
    p.issue(1.,(-.2,0.));s=p.predict(2.,WheelLimits())
    assert .05<=s.x<=.055 and s.v==0.


def test_missing_command_history_returns_measurement_without_invented_motion():
    p=CommandPrediction();correct(p,VehicleState(1,2,.4,.03,.01),1.)
    assert p.predict(2.,WheelLimits())==p.state


def test_new_p3_motion_restores_speed_after_commanded_stop_without_hard_clipping():
    p=CommandPrediction();p.issue(0.,(.4,0.));correct(p,VehicleState(0,0,0,.3,0),0.)
    p.issue(.1,(0.,0.));assert p.predict(1.,WheelLimits()).v==0.
    # 相邻实测给出0.3m/s，即使高于配置速度也不改写真实物理运动。
    correct(p,VehicleState(.3,0,0,.3,0),1.)
    p.issue(1.,(.4,0.));s=p.predict(2.,WheelLimits())
    assert s.v==pytest.approx(.3) and s.x==pytest.approx(.6)



def test_prediction_at_earlier_query_rebuilds_from_observed_velocity():
    p=CommandPrediction();p.issue(0.,(.2,0.));correct(p,VehicleState(0,0,0,0,0),0.)
    correct(p,VehicleState(.05,0,0,0,0),1.)
    before=p.predict(2.,WheelLimits());p.predict(3.,WheelLimits())
    assert p.predict(2.,WheelLimits())==before
