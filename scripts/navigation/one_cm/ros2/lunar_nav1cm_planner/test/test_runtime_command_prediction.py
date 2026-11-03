"""发布指令外推只影响规划快照，P3实测继续提供任务证据。"""
from types import SimpleNamespace as S
import pytest
from test_runtime import build_runtime, pose
from nav1cm.contracts import VehicleState


def velocity(v,w=0.):
    return S(linear=S(x=v),angular=S(z=w))


def configured():
    runtime,session,_,logs=build_runtime()
    runtime.configure_direct_control(lambda *args:None,command_hold_s=1.2)
    runtime.configure_command_prediction(spin_prepare_s=0.)
    runtime.ros_now_ns=lambda:3_000_000_000
    runtime._applied_pose_stamp_ns=runtime._pose_source_high_water=1_000_000_000
    runtime.offer_published_command(velocity(.1),received_s=0.)
    return runtime,session,logs


def test_prediction_and_short_command_proof_share_one_snapshot(monkeypatch):
    import nav1cm.execution as execution
    runtime,session,_=configured()
    measured=session.state
    runtime.offer_goal(pose(0,3))
    planning=[];proof=[];tick=session.tick
    def observe_tick(now,**kwargs):
        planning.append(kwargs.get('planning_state'))
        return tick(now,**kwargs)
    def certified(result,state,*args,**kwargs):
        proof.append(state)
        return (0.,0.)
    monkeypatch.setattr(session,'tick',observe_tick)
    monkeypatch.setattr(execution,'certified_velocity_command',certified)
    runtime._tick_once(2.)
    assert planning[0].x>0.15
    assert proof[0] is planning[0]
    assert session.state is measured
    assert runtime._odometry_sequence==0


def test_same_p3_stamp_correction_reanchors_without_new_observation():
    runtime,session,_=configured()
    first,detail=runtime._prediction_snapshot(2.)
    assert first.x==pytest.approx(.19,abs=1e-8)
    session.offer_state(VehicleState(.5,0,0,0,0))
    corrected,_=runtime._prediction_snapshot(2.)
    assert corrected.x==pytest.approx(.69,abs=1e-8)
    assert session.state.x==.5
    assert runtime._observed_pose_stamp_ns is None


def test_missing_history_returns_measured_without_replacing_it():
    runtime,session,_=configured()
    # 被观测时刻在首条真实发布记录之前，不猜测此前指令。
    runtime._command_prediction.commands.clear()
    runtime._published_command_inputs.clear()
    runtime.offer_published_command(velocity(.1),received_s=1.)
    state,detail=runtime._prediction_snapshot(2.)
    assert state is session.state
    assert not detail['used_prediction']


def test_own_publish_is_not_mistaken_for_final_vehicle_command():
    runtime,session,_=configured()
    runtime._prediction_snapshot(2.)
    before=list(runtime._command_prediction.commands)
    runtime._publish_issued((0.,.1))
    assert list(runtime._command_prediction.commands)==before


def test_command_update_does_not_rewrite_p3_or_its_source_evidence():
    runtime,session,_=configured()
    old=session.state
    runtime.offer_published_command(velocity(0.,.1),received_s=1.)
    runtime._prediction_snapshot(2.)
    assert session.state is old
    assert runtime._applied_pose_stamp_ns==1_000_000_000
    assert runtime._pose_source_high_water==1_000_000_000
    assert runtime._odometry_sequence==0
