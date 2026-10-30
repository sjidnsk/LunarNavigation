"""加载真实源配置并验证损坏输入拒绝，不复刻加载实现。"""
import math
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / "config/isaac_viper.yaml"


def load(path=PROFILE):
    from tools.isaac_viper.config import load_profile
    return load_profile(path)


def fixture_profile(tmp_path, *, capability_mutation=None, profile_mutation=None,
                    exploration_mutation=None):
    raw = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    cap = yaml.safe_load((ROOT / "config/wheel_orin.yaml").read_text(encoding="utf-8"))
    exp = yaml.safe_load((ROOT / "config/exploration_navigation.yaml").read_text(encoding="utf-8"))
    if capability_mutation:
        capability_mutation(cap)
    if profile_mutation:
        profile_mutation(raw)
    if exploration_mutation:
        exploration_mutation(exp)
    for name, data in [("wheel.yaml", cap), ("exploration.yaml", exp)]:
        (tmp_path / name).write_text(yaml.safe_dump(data), encoding="utf-8")
    raw["sources"] = {"capability": "wheel.yaml", "exploration": "exploration.yaml"}
    target = tmp_path / "profile.yaml"
    target.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return target


def test_resolves_authority_and_simulation_assumptions():
    p = load()
    assert {"capability", "simulation", "sensors", "ros"} <= p.keys()
    assert p["capability"]["wheel_diameter_m"] == .319
    assert p["simulation"]["mass"]["total_kg"] == 50
    assert p["simulation"]["mass"]["calibration_status"] == "pending_real_calibration"
    assert p["sensors"]["stereo"]["baseline_m"] == .14
    assert p["sensors"]["effective_observation"] == {
        "minimum_range_m": .05, "maximum_range_m": 10.0, "horizontal_fov_deg": 90.0}
    assert p["simulation"]["wheel_centers_m"] == [
        [.40875, .335, .1595], [.40875, -.335, .1595],
        [-.40875, .335, .1595], [-.40875, -.335, .1595]]


def test_reads_changed_capability_instead_of_embedded_geometry(tmp_path):
    path = fixture_profile(tmp_path, capability_mutation=lambda d: d["capability"].update(
        wheelbase_m=.8, maximum_forward_speed_mps=.1))
    p = load(path)
    assert p["simulation"]["wheel_centers_m"][0][0] == .4
    assert p["capability"]["maximum_forward_speed_mps"] == .1


def test_reads_changed_exploration_effective_range_and_fov(tmp_path):
    path = fixture_profile(tmp_path, exploration_mutation=lambda d: d["exploration"].update(
        sensor_range_m=8, sensor_fov_deg=80))
    assert load(path)["sensors"]["effective_observation"]["maximum_range_m"] == 8
    assert load(path)["sensors"]["effective_observation"]["horizontal_fov_deg"] == 80


@pytest.mark.parametrize("key,value", [
    ("wheel_diameter_m", 0), ("wheelbase_m", -1), ("track_width_m", math.nan),
    ("maximum_forward_speed_mps", math.inf), ("maximum_curvature_per_m", 0),
    ("wheel_width_m", .9), ("wheelbase_m", 2),
])
def test_rejects_nonphysical_capability(tmp_path, key, value):
    path = fixture_profile(tmp_path, capability_mutation=lambda d: d["capability"].update({key: value}))
    with pytest.raises(ValueError):
        load(path)


def test_missing_required_source_field_rejected(tmp_path):
    path = fixture_profile(tmp_path, capability_mutation=lambda d: d["capability"].pop("maximum_yaw_acceleration_radps2"))
    with pytest.raises(ValueError, match="maximum_yaw_acceleration"):
        load(path)


@pytest.mark.parametrize("change", [
    lambda p: p["ros"].update(domain_id=0),
    lambda p: p["ros"].update(localhost_only=False),
    lambda p: p["drive"].update(command_timeout_s=-1),
    lambda p: p["sensors"]["stereo"].update(left_origin_m=[-.567, .07, 1.1]),
    lambda p: p["sensors"]["stereo"].update(baseline_m=.2),
    lambda p: p["sensors"]["lidar"].update(frequency_hz=math.nan),
    lambda p: p["simulation"]["mass"].update(calibration_status="calibrated"),
    lambda p: p["simulation"]["mast"].update(center_m=[1, 0, 1]),
    lambda p: p["ros"]["topics"].update(imu="/Car/T3/imu"),
])
def test_rejects_unsafe_or_inconsistent_profile(tmp_path, change):
    with pytest.raises(ValueError):
        load(fixture_profile(tmp_path, profile_mutation=change))


@pytest.mark.parametrize("key,value", [("sensor_range_m", 0), ("sensor_fov_deg", 361),
                                      ("sensor_range_m", math.inf)])
def test_invalid_effective_observation_rejected(tmp_path, key, value):
    path = fixture_profile(tmp_path, exploration_mutation=lambda d: d["exploration"].update({key: value}))
    with pytest.raises(ValueError):
        load(path)


def test_all_sensor_origins_are_front_mounted_and_topics_are_explicit():
    p = load()
    assert p["sensors"]["stereo"]["left_origin_m"] == [.567, .07, 1.1]
    assert p["sensors"]["stereo"]["right_origin_m"] == [.567, -.07, 1.1]
    assert p["sensors"]["lidar"]["origin_m"] == [.587, 0, 1.235]
    topics = p["ros"]["topics"]
    assert topics["command"] == "/Car/T5/Car_Cmd_Vel"
    assert topics["odometry"] == "/Car/T3/localization/odometry"
    for name in ["left_image", "right_image", "left_camera_info", "right_camera_info",
                 "depth", "depth_points", "lidar_raw", "lidar_effective", "imu"]:
        assert topics[name].startswith("/Car/T3/sensors/")

def test_imu_origin_must_stay_inside_vehicle(tmp_path):
    path = fixture_profile(tmp_path, profile_mutation=lambda p: p["sensors"]["imu"].update(
        origin_m=[0, 0, -1]))
    with pytest.raises(ValueError):
        load(path)

def test_footprint_uses_half_extent_bounds(tmp_path):
    path = fixture_profile(tmp_path, capability_mutation=lambda p: p["capability"].update(
        footprint_xy_m=[[.7, .409], [.7, -.409], [-.591, -.409], [-.591, .409]]))
    with pytest.raises(ValueError):
        load(path)

def test_ros_control_frame_is_separate_from_ground_geometry_frame():
    p = load()
    assert p["ros"]["frames"]["map"] == "map"
    assert p["ros"]["frames"]["odom"] == "odom"
    assert p["ros"]["frames"]["base"] == "base_link"
    assert p["ros"]["frames"]["footprint"] == "base_footprint"
    assert p["simulation"]["base_frame_id"] == "base_footprint"
    assert p["sensors"]["origin_frame"] == "base_footprint"
