"""已发布车体速度在P3源时刻锚点上的延迟回放。"""
from pathlib import Path
import math
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parents[1]))
from lunar_nav1cm_planner.command_prediction import CommandPrediction
from nav1cm.contracts import VehicleState, WheelLimits


def anchored(command=(.2, 0.), state=None, *, delay=0., **kwargs):
    p = CommandPrediction(use_published_velocity=True, **kwargs)
    p.issue(0., command)
    state = state or VehicleState(0., 0., 0., 0., 0.)
    p.correct(state, 1_000_000_000, delay, int((1. + delay) * 1e9))
    return p, WheelLimits(), state


def test_starts_from_zero_and_replays_p3_source_time():
    p, limits, state = anchored(delay=2.)
    value = p.predict(2., limits)
    assert p.sample_s == 0.
    assert value.x == pytest.approx(.36, abs=1e-12)
    assert value.v == pytest.approx(.2)
    assert p.state == state
    assert p.predict(2., limits) == value


def test_published_curve_matches_analytic_constant_twist():
    state = VehicleState(1., -2., .3, .2, .1)
    p, limits, _ = anchored((.2, .1), state)
    value = p.predict(3., limits)
    assert value.x == pytest.approx(1. + 2. * (math.sin(.6) - math.sin(.3)), abs=1e-12)
    assert value.y == pytest.approx(-2. + 2. * (math.cos(.3) - math.cos(.6)), abs=1e-12)
    assert value.yaw == pytest.approx(.6)


def test_same_stamp_correction_reanchors_but_older_stamp_is_ignored():
    p, limits, _ = anchored()
    p.predict(2., limits)
    revised = VehicleState(3., 1., .4, .1, 0.)
    p.correct(revised, 1_000_000_000, 2., 3_000_000_000)
    assert p.state == revised and p.stamp == 1_000_000_000
    value = p.predict(2., limits)
    distance = .2 * 2. - .01
    assert value.x == pytest.approx(3. + distance * math.cos(.4))
    assert value.y == pytest.approx(1. + distance * math.sin(.4))
    p.correct(VehicleState(99., 0., 0., 0., 0.), 500_000_000, 2., 3_000_000_000)
    assert p.predict(2., limits) == value


def test_new_anchor_keeps_measured_velocity_instead_of_pose_difference():
    p, limits, _ = anchored()
    state = VehicleState(10., 0., 0., 0., 0.)
    p.correct(state, 2_000_000_000, 2., 3_000_000_000)
    assert p.predict(2., limits).x == pytest.approx(10.16)


def test_reverse_request_brakes_then_accelerates_from_measured_state():
    p, limits, _ = anchored((-.2, 0.), VehicleState(0., 0., 0., .2, 0.))
    value = p.predict(1., limits)
    assert value.v == pytest.approx(-.2)
    assert value.x == pytest.approx(-.04, abs=1e-12)


def test_spin_prepare_repeats_do_not_restart_and_flip_or_zero_do_restart():
    p, limits, _ = anchored((0., .1), spin_prepare_s=3.)
    assert p.predict(2., limits).yaw == 0.
    p.issue(2., (0., .1))
    assert p.predict(3., limits).yaw == 0.
    assert p.predict(4., limits).yaw == pytest.approx(.09)
    p.issue(4., (0., -.1))
    assert p.predict(6., limits).yaw == pytest.approx(.1)
    p.issue(6., (0., 0.))
    p.issue(7., (0., -.1))
    assert p.predict(10., limits).yaw == pytest.approx(.1)
    assert p.predict(11., limits).yaw == pytest.approx(.01, abs=1e-12)


def test_missing_or_truncated_history_falls_back_to_p3():
    p = CommandPrediction(use_published_velocity=True, max_commands=2)
    state = VehicleState(1., 0., 0., 0., 0.)
    p.issue(.5, (.2, 0.))
    p.correct(state, 1_000_000_000, 2., 3_000_000_000)
    assert not p.covered(2.) and p.predict(2., WheelLimits()) == state
    p.issue(1., (.1, 0.))
    p.issue(2., (.2, 0.))
    assert p.predict(3., WheelLimits()) == state


def test_long_age_is_allowed_with_real_history_and_steady_prediction_is_constant_cost(monkeypatch):
    p, limits, _ = anchored((.2, .1), VehicleState(0., 0., 0., .2, .1))
    import lunar_nav1cm_planner.command_prediction as module
    calls = []
    original = module.math.cos
    monkeypatch.setattr(module.math, 'cos', lambda x: (calls.append(x), original(x))[1])
    assert p.covered(60.)
    value = p.predict(60., limits)
    assert value.yaw == pytest.approx(6.)
    assert len(calls) < 20
    calls.clear()
    assert p.predict(60., limits) == value and not calls


def test_query_partition_and_command_partition_do_not_change_prediction():
    p, limits, _ = anchored((.2, .1))
    q, _, _ = anchored((.2, .1))
    for t in (.007, .031, .081, .173, .391, .509, .777):
        p.predict(t, limits)
        p.issue(t, (.2, .1))
    a, b = p.predict(1.3, limits), q.predict(1.3, limits)
    for field in ('x', 'y', 'yaw', 'v', 'w'):
        assert getattr(a, field) == pytest.approx(getattr(b, field), abs=1e-12)
    assert len(p.commands) == 1
    assert abs(a.v) <= .2 + 1e-12 and abs(a.w) <= .1 + 1e-12


def test_clock_rollback_and_future_anchor_do_not_invent_history():
    p, limits, _ = anchored()
    state = VehicleState(4., 0., 0., 0., 0.)
    p.correct(state, 500_000_000, 2., 700_000_000)
    assert p.predict(3., limits) == state
    p.issue(3., (.2, 0.))
    p.correct(state, 4_000_000_000, 3., 3_000_000_000)
    assert not p.covered(3.) and p.predict(3., limits) == state


def test_historical_command_revision_invalidates_cached_future():
    p, limits, _ = anchored()
    p.predict(2., limits)
    p.issue(1., (0., 0.))
    assert p.predict(2., limits).x == pytest.approx(.2, abs=1e-12)


def test_accelerating_curve_matches_constant_radius_geometry():
    p, _, _ = anchored((.2, .1))
    limits = WheelLimits(angular_accel_rps2=.25)
    value = p.predict(1., limits)
    angle = .08  # 0.4s加速面积0.02rad，再恒速0.6s。
    assert value.yaw == pytest.approx(angle, abs=1e-12)
    assert value.x == pytest.approx(2. * math.sin(angle), abs=1e-12)
    assert value.y == pytest.approx(2. * (1. - math.cos(angle)), abs=1e-12)


def test_cached_prediction_cannot_bridge_dropped_anchor_history():
    p, limits, state = anchored(max_commands=2)
    p.predict(2., limits)
    p.issue(2., (.1, 0.))
    p.issue(3., (.2, 0.))
    assert not p.covered(4.)
    assert p.predict(4., limits) == state


def test_acceleration_limits_change_invalidates_prediction_cache():
    p, limits, _ = anchored()
    p.predict(1., limits)
    slower = WheelLimits(linear_accel_mps2=.1)
    assert p.predict(1., slower).x == pytest.approx(.05, abs=1e-12)
    assert p.predict(1., slower).v == pytest.approx(.1, abs=1e-12)
