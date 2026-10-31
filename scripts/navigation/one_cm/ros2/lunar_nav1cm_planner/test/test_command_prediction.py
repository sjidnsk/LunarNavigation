from pathlib import Path
import sys
import pytest
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE)); sys.path.insert(0,str(PACKAGE.parents[1]))
from lunar_nav1cm_planner.command_prediction import CommandPrediction
from nav1cm.contracts import VehicleState, WheelLimits

def test_acceleration_braking_and_repeated_prediction():
    p=CommandPrediction(); l=WheelLimits()
    p.issue(0.,(1.,0.)); p.correct(VehicleState(0,0,0,0,0),1000000000,0.,1000000000)
    a=p.predict(1.,l)
    assert a.x==pytest.approx(.25); assert a.v==pytest.approx(.5)
    assert p.predict(1.,l)==a
    p.issue(1.,(0.,0.))
    assert p.predict(2.,l).x==pytest.approx(.5)
    assert p.predict(2.,l).v==pytest.approx(0.)

def test_rotation_delayed_correction_and_old_stamp():
    p=CommandPrediction(); l=WheelLimits(); p.issue(0.,(0.,1.))
    p.correct(VehicleState(0,0,0,0,0),1000000000,1.,2000000000)
    assert p.predict(1.,l).yaw==pytest.approx(.25)
    p.correct(VehicleState(3,0,.1,0,.2),1500000000,1.,2000000000)
    a=p.predict(1.,l)
    assert a.x==3; assert a.yaw==pytest.approx(.2625)
    p.correct(VehicleState(9,0,0,0,0),1000000000,1.,2000000000)
    assert p.predict(1.,l)==a

def test_missing_history_and_clock_reset():
    p=CommandPrediction(); s=VehicleState(0,0,0,.2,0); l=WheelLimits()
    p.issue(.5,(.2,0.)); p.correct(s,1000000000,1.,2000000000)
    assert p.predict(1.,l)==s
    p.correct(s,2000000000,2.,3000000000)
    assert p.predict(2.,l).x==pytest.approx(.2)
    p.correct(s,100000000,3.,200000000)
    assert p.predict(3.,l)==s

def test_history_bounded_and_coalesced():
    p=CommandPrediction(max_commands=3)
    for i in range(20):p.issue(i,(.2,0.))
    assert len(p.commands)==1
    for i in range(20,30):p.issue(i,(i,0.))
    assert len(p.commands)==3


def test_runtime_direct_ignores_duplicate_and_late_feedback(monkeypatch):
    from types import SimpleNamespace as S
    from test_runtime import build_runtime
    import lunar_nav1cm_planner.runtime as module
    runtime,session,_,_=build_runtime()
    runtime.configure_direct_control(lambda *a:None,command_hold_s=.1)
    def odom(stamp):return S(header=S(stamp=S(sec=stamp,nanosec=0)))
    monkeypatch.setattr(module.time,'monotonic',lambda:5.)
    runtime.offer_odometry(odom(3))
    original=runtime._odometry
    monkeypatch.setattr(module.time,'monotonic',lambda:6.)
    runtime.offer_odometry(odom(3));runtime.offer_odometry(odom(2))
    assert runtime._odometry is original
    assert runtime._odometry_received_s==5.
    runtime.offer_odometry(odom(4))
    assert runtime._odometry.header.stamp.sec==4
    assert runtime._odometry_received_s==6.


def test_runtime_planning_estimate_keeps_measured_state(monkeypatch):
    from test_runtime import build_runtime,pose
    runtime,session,_,_=build_runtime()
    runtime.configure_direct_control(lambda *a:None,command_hold_s=.1)
    s=VehicleState(0,0,0,0,0)
    session.offer_state(s)
    runtime._command_prediction.issue(0.,(.2,0.))
    runtime._command_prediction.correct(s,1000000000,0.,1000000000)
    runtime.offer_goal(pose(0,3))
    calls=[]; original=session.tick
    def tick(now,**kwargs):
        calls.append(kwargs.get('planning_state'))
        assert session.state==s
        return original(now,**kwargs)
    monkeypatch.setattr(session,'tick',tick)
    runtime._tick_once(1.)
    assert calls[0].x==pytest.approx(.16)
    assert calls[0].v==pytest.approx(.2)
    assert session.state==s
