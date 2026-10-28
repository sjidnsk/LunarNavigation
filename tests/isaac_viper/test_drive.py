"""纯执行策略的手算行为夹具；不依赖 ROS 或 Isaac。"""
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
ZERO = [0.0] * 4
SPIN = [-0.8842347308751209, 0.8842347308751209,
        0.8842347308751209, -0.8842347308751209]
LEFT = [0.5511304336887006, 0.29711662598950883,
        -0.5511304336887006, -0.29711662598950883]


def controller():
    from tools.isaac_viper.config import load_profile
    from tools.isaac_viper.drive import DriveController
    return DriveController(load_profile(ROOT / "config/isaac_viper.yaml"))


def settled(vx, wz, angles):
    c = controller()
    assert c.receive_twist(vx, 0, wz, 0)
    return c, c.step(2.0, 0.1, angles, (0.0, 0.0))


@pytest.mark.parametrize("vx,want", [(0.2, 1.2539184952978057),
                                     (-0.2, -1.2539184952978057),
                                     (9, 1.2539184952978057),
                                     (-9, -1.2539184952978057)])
def test_translation_and_signed_body_speed_limit(vx, want):
    _, out = settled(vx, 0, ZERO)
    assert out["steering_rad"] == pytest.approx(ZERO)
    assert out["wheel_radps"] == pytest.approx([want] * 4)
    assert not out["safe_stop"]


@pytest.mark.parametrize("wz,sign", [(1, 1), (-1, -1), (20, 1)])
def test_spin_uses_tangential_wheel_speed_above_translation_limit(wz, sign):
    _, out = settled(0, wz, SPIN)
    assert out["steering_rad"] == pytest.approx(SPIN)
    assert out["wheel_radps"] == pytest.approx(
        [sign * -3.31341623034235, sign * 3.31341623034235,
         sign * -3.31341623034235, sign * 3.31341623034235])


@pytest.mark.parametrize("vx,wz,angles,speeds", [
    (0.2, 0.2, LEFT, [0.9787808285108142, 1.7506882774152568,
                      0.9787808285108142, 1.7506882774152568]),
    (0.2, -0.2, [-LEFT[1], -LEFT[0], LEFT[1], LEFT[0]],
     [1.7506882774152568, 0.9787808285108142,
      1.7506882774152568, 0.9787808285108142]),
    (-0.2, -0.2, LEFT, [-0.9787808285108142, -1.7506882774152568,
                        -0.9787808285108142, -1.7506882774152568]),
])
def test_hand_checked_arc(vx, wz, angles, speeds):
    _, out = settled(vx, wz, angles)
    assert out["steering_rad"] == pytest.approx(angles)
    assert out["wheel_radps"] == pytest.approx(speeds)


def test_curvature_clamp_keeps_translation_and_limits_yaw():
    _, out = settled(.2, 10, LEFT)
    assert out["wheel_radps"] == pytest.approx(
        [0.9787808285108142, 1.7506882774152568] * 2)


def test_lateral_acceleration_limit_is_consumed_from_profile():
    from tools.isaac_viper.config import load_profile
    from tools.isaac_viper.drive import DriveController
    p = load_profile(ROOT / "config/isaac_viper.yaml")
    p["capability"]["maximum_lateral_acceleration_mps2"] = .02
    c = DriveController(p)
    c.receive_twist(.2, 0, .2, 0)
    # yaw .1 at vx .2: lateral acceleration .02, front-left atan(.040875/.1665).
    out = c.step(2, .1, ZERO, (0, 0))
    assert out["steering_rad"] == pytest.approx(
        [0.24073465677724296, 0.17329760843312567,
         -0.24073465677724296, -0.17329760843312567], abs=1e-6)
    assert out["wheel_radps"] == ZERO


def test_acceleration_and_normal_braking_are_rate_limited():
    c = controller()
    c.receive_twist(.2, 0, 0, 0)
    out = c.step(.1, .1, ZERO, (0, 0))
    assert out["wheel_radps"] == pytest.approx([.3134796238244514] * 4)
    c.receive_twist(0, 0, 0, .11)
    out = c.step(.05, .15, ZERO, (.05, 0))
    assert out["wheel_radps"] == pytest.approx([.1567398119122257] * 4)


def test_yaw_acceleration_is_rate_limited():
    c = controller()
    c.receive_twist(0, 0, 1, 0)
    out = c.step(.1, .1, SPIN, (0, 0))
    assert out["wheel_radps"] == pytest.approx(
        [-.1656708115171175, .1656708115171175] * 2)


def test_steering_alignment_precedes_any_wheel_motion():
    c = controller()
    c.receive_twist(0, 0, 1, 0)
    out = c.step(.1, .1, ZERO, (0, 0))
    assert out["steering_rad"] == pytest.approx(SPIN)
    assert out["wheel_radps"] == ZERO
    assert out["safe_stop"] and out["reason"] == "steering_alignment"
    out = c.step(.1, .2, SPIN, (0, 0))
    assert out["wheel_radps"][1] == pytest.approx(.1656708115171175)


def test_reverse_switch_brakes_and_waits_for_measured_stop():
    c, _ = settled(.2, 0, ZERO)
    c.receive_twist(-.2, 0, 0, .11)
    out = c.step(.1, .2, ZERO, (.2, 0))
    assert out["wheel_radps"] == pytest.approx([.9404388714733542] * 4)
    out = c.step(1, .3, ZERO, (.1, 0))
    assert out["wheel_radps"] == ZERO
    assert out["reason"] == "waiting_for_stop"
    out = c.step(.1, .4, ZERO, (0, 0))
    assert out["wheel_radps"] == pytest.approx([-.3134796238244514] * 4)


def test_spin_to_translation_waits_for_measured_yaw_stop():
    c, _ = settled(0, 1, SPIN)
    c.receive_twist(.2, 0, 0, .11)
    out = c.step(3, .2, SPIN, (0, .1))
    assert out["wheel_radps"] == ZERO
    assert out["steering_rad"] == pytest.approx(SPIN)
    assert out["reason"] == "waiting_for_stop"
    out = c.step(.1, .3, SPIN, (0, 0))
    assert out["steering_rad"] == ZERO
    assert out["wheel_radps"] == ZERO
    out = c.step(.1, .4, ZERO, (0, 0))
    assert out["wheel_radps"][0] > 0


def test_moving_steering_change_stops_before_reorienting():
    c, _ = settled(.2, 0, ZERO)
    c.receive_twist(.2, 0, .2, .11)
    out = c.step(.1, .2, ZERO, (.2, 0))
    assert out["steering_rad"] == ZERO
    assert out["wheel_radps"][0] > 0
    out = c.step(1, .3, ZERO, (.1, 0))
    assert out["steering_rad"] == ZERO
    out = c.step(.1, .4, ZERO, (0, 0))
    assert out["steering_rad"] == pytest.approx(LEFT)
    assert out["wheel_radps"] == ZERO


@pytest.mark.parametrize("age,stopped", [(.4999, False), (.5, True), (1, True)])
def test_wall_clock_watchdog_boundary(age, stopped):
    c, _ = settled(.2, 0, ZERO)
    out = c.step(.1, age, ZERO, (.2, 0))
    assert out["safe_stop"] is stopped
    if stopped:
        assert out["wheel_radps"] == ZERO
        assert out["reason"] == "command_timeout"


@pytest.mark.parametrize("vx,vy,wz,t", [
    (math.nan, 0, 0, 0), (0, 0, math.inf, 0),
    (0, .01, 0, 0), (0, 0, 0, math.nan), (0, 0, 0, -1),
])
def test_invalid_command_clears_previous_motion(vx, vy, wz, t):
    c, _ = settled(.2, 0, ZERO)
    assert not c.receive_twist(vx, vy, wz, t)
    out = c.step(.1, .2, ZERO, (0, 0))
    assert out["safe_stop"] and out["wheel_radps"] == ZERO
    assert out["reason"] == "invalid_command"


@pytest.mark.parametrize("dt,t,angles,speed", [
    (0, .2, ZERO, (0, 0)), (-1, .2, ZERO, (0, 0)),
    (math.nan, .2, ZERO, (0, 0)), (.1, math.nan, ZERO, (0, 0)),
    (.1, .05, ZERO, (0, 0)), (.1, .2, [math.nan] * 4, (0, 0)),
    (.1, .2, [0] * 3, (0, 0)), (.1, .2, ZERO, (0, math.inf)),
])
def test_bad_step_feedback_and_time_fail_closed(dt, t, angles, speed):
    c, _ = settled(.2, 0, ZERO)
    out = c.step(dt, t, angles, speed)
    assert out["safe_stop"] and out["wheel_radps"] == ZERO
    assert all(math.isfinite(a) for a in out["steering_rad"])


def test_scalar_body_speed_remains_supported_for_translation():
    c = controller()
    c.receive_twist(.2, 0, 0, 0)
    assert c.step(.1, .1, ZERO, 0.0)["wheel_radps"][0] > 0


def test_new_controller_has_no_command_and_never_rolls():
    out = controller().step(.1, 0, ZERO, (0, 0))
    assert out["safe_stop"] and out["wheel_radps"] == ZERO

def test_configured_steering_limit_constrains_high_curvature_requests():
    from tools.isaac_viper.config import load_profile
    from tools.isaac_viper.drive import DriveController
    p = load_profile(ROOT / "config/isaac_viper.yaml")
    p["capability"]["maximum_curvature_per_m"] = 10
    p["drive"]["steering_limit_rad"] = .9
    c = DriveController(p)
    c.receive_twist(.2, 0, 1, 0)
    out = c.step(.1, .1, ZERO, (0, 0))
    assert max(abs(a) for a in out["steering_rad"]) == pytest.approx(.9)
    assert out["wheel_radps"] == ZERO


def test_braking_uses_braking_limit_instead_of_acceleration_limit():
    from tools.isaac_viper.config import load_profile
    from tools.isaac_viper.drive import DriveController
    p = load_profile(ROOT / "config/isaac_viper.yaml")
    p["capability"]["maximum_braking_deceleration_mps2"] = .1
    c = DriveController(p)
    c.receive_twist(.2, 0, 0, 0)
    c.step(1, .1, ZERO, (0, 0))
    c.receive_twist(0, 0, 0, .11)
    out = c.step(.1, .2, ZERO, (.2, 0))
    # .2 - .1*.1 = .19 m/s; .19/.1595 = 1.19122257 rad/s.
    assert out["wheel_radps"] == pytest.approx([1.1912225705329154] * 4)


def test_watchdog_recovery_waits_for_measured_motion_to_stop():
    c, _ = settled(.2, 0, ZERO)
    c.step(.1, .5, ZERO, (.2, 0))
    c.receive_twist(-.2, 0, 0, .51)
    out = c.step(.1, .6, ZERO, (.1, 0))
    assert out["wheel_radps"] == ZERO
    assert out["reason"] == "waiting_for_stop"
    assert c.step(.1, .7, ZERO, (0, 0))["wheel_radps"][0] < 0


def test_async_receipt_between_consumption_and_step_is_not_clock_reversal():
    c = controller()
    assert c.receive_twist(.1, 0, 0, 1.)
    c.step(.01, 1.01, ZERO, (0., 0.))
    # 已消费旧mailbox后，DDS在1.015收到新包；主线程先完成1.02这一控制步。
    c.step(.01, 1.02, ZERO, (0., 0.))
    assert c.receive_twist(.2, 0, 0, 1.015)
    out = c.step(.01, 1.03, ZERO, (0., 0.))
    assert out['reason'] == 'tracking'
    assert not out['safe_stop']
    assert c.command_time == 1.015


def test_delayed_receipt_keeps_watchdog_and_receive_stream_ordering():
    c = controller()
    c.receive_twist(.1, 0, 0, 1.)
    c.step(.01, 1.02, ZERO, (0., 0.))
    assert c.receive_twist(.2, 0, 0, 1.015)
    assert c.step(.01, 1.6, ZERO, (0., 0.))['reason'] == 'command_timeout'
    assert not c.receive_twist(.2, 0, 0, 1.014)
    assert c.step(.01, 1.61, ZERO, (0., 0.))['reason'] == 'invalid_command'


def test_invalid_mailbox_then_already_received_valid_packet_is_consumed_safely():
    from tools.isaac_viper.ros_bridge import LatestCommandMailbox
    box, c = LatestCommandMailbox(), controller()
    c.receive_twist(.1, 0, 0, 1.)
    c.step(.01, 1.01, ZERO, (0., 0.))
    box.receive([float('nan'),0,0,0,0,0],1.015)
    box.receive([.2,0,0,0,0,0],1.016)
    bad = box.take_after(0)
    assert bad.planar is None
    c.receive_twist(float('nan'), 0, 0, bad.rx_wall)
    assert c.step(.01,1.02,ZERO,(0.,0.))['safe_stop']
    valid = box.take_after(bad.sequence)
    assert c.receive_twist(*valid.planar, valid.rx_wall)
    assert c.step(.01,1.03,ZERO,(0.,0.))['reason'] == 'tracking'
