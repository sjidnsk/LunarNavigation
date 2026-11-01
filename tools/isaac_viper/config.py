"""解析仿真适配配置；源 YAML 保持几何、能力和高层有效观测的权威。"""
from copy import deepcopy
import math
from pathlib import Path

import yaml

CALIBRATION = "pending_real_calibration"
SENSOR_TOPICS = (
    "left_image", "right_image", "left_camera_info", "right_camera_info",
    "depth", "depth_points", "lidar_raw", "lidar_effective", "imu",
)
POSITIVE_CAPABILITY = (
    "wheel_diameter_m", "wheel_width_m", "wheelbase_m", "track_width_m",
    "minimum_underbody_clearance_m", "maximum_forward_speed_mps",
    "maximum_reverse_speed_mps", "maximum_spin_rate_radps",
    "maximum_acceleration_mps2", "maximum_braking_deceleration_mps2",
    "maximum_yaw_acceleration_radps2", "maximum_lateral_acceleration_mps2",
    "maximum_curvature_per_m",
)


def _number(value, label, *, positive=True):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (value <= 0 if positive else value < 0)):
        raise ValueError(f"{label}: 需要有限{'正' if positive else '非负'}数")
    return float(value)


def _vector(value, label, length=3, positive=False):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label}: 需要 {length} 维向量")
    for item in value:
        if (isinstance(item, bool) or not isinstance(item, (int, float))
                or not math.isfinite(item) or (positive and item <= 0)):
            raise ValueError(f"{label}: 向量值不合理")
    return value


def _read(path):
    with Path(path).open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict):
        raise ValueError(f"{path}: YAML 根必须是映射")
    return result


def _box(center, size, extent, label):
    _vector(center, label)
    _vector(size, label, positive=True)
    if center[0] <= 0:
        raise ValueError(f"{label}: 必须前置于车头 +X")
    bounds = [(-extent[0] / 2, extent[0] / 2),
              (-extent[1] / 2, extent[1] / 2), (0, extent[2])]
    for c, s, (low, high) in zip(center, size, bounds):
        if c - s / 2 < low or c + s / 2 > high:
            raise ValueError(f"{label}: 杆/外壳超出名义包络")


def load_profile(path) -> dict:
    """返回 capability/simulation/sensors/ros/drive 的独立字典。

    sources 路径相对于 profile；不导入 ROS/Isaac。轮心顺序固定 fl/fr/rl/rr，
    坐标来自 base_footprint 地面原点，X 前 Y 左 Z 上。camera 此处 +X 朝前；
    USD camera -Z 与 ROS optical +Z 的旋转由场景/桥接适配器显式完成。
    缺字段、非物理值及破坏隔离的配置均抛出 ValueError。
    """
    path = Path(path).resolve()
    try:
        p = deepcopy(_read(path))
        cap_source = _read(path.parent / p["sources"]["capability"])
        exp_source = _read(path.parent / p["sources"]["exploration"])
        cap = deepcopy(cap_source["capability"])
        for key in POSITIVE_CAPABILITY:
            _number(cap[key], key)
        extent = _vector(cap["body_extent_m"], "body_extent_m", positive=True)
        footprint = cap["footprint_xy_m"]
        if not isinstance(footprint, list) or len(footprint) < 3:
            raise ValueError("footprint_xy_m: 至少三个点")
        for point in footprint:
            _vector(point, "footprint_xy_m", length=2)
            if abs(point[0]) > extent[0] / 2 + 1e-9 or abs(point[1]) > extent[1] / 2 + 1e-9:
                raise ValueError("footprint_xy_m: 超出名义包络")
        if (cap["wheelbase_m"] + cap["wheel_diameter_m"] > extent[0] + 1e-9
                or cap["track_width_m"] + cap["wheel_width_m"] > extent[1] + 1e-9
                or cap["minimum_underbody_clearance_m"] >= extent[2]
                or cap["wheel_diameter_m"] > extent[2]):
            raise ValueError("车轮几何或底盘间隙超出名义包络")
        p["capability"] = cap
        sim = p["simulation"]
        sim["base_frame_id"] = cap_source["base_frame_id"]
        sim["wheel_radius_m"] = cap["wheel_diameter_m"] / 2
        x, y, z = cap["wheelbase_m"] / 2, cap["track_width_m"] / 2, sim["wheel_radius_m"]
        sim["wheel_centers_m"] = [[x, y, z], [x, -y, z], [-x, y, z], [-x, -y, z]]
        mass = sim["mass"]
        for key in ("chassis_kg", "steering_knuckle_kg", "wheel_kg", "total_kg"):
            _number(mass[key], key)
        if not math.isclose(mass["total_kg"], mass["chassis_kg"] +
                            4 * (mass["steering_knuckle_kg"] + mass["wheel_kg"])):
            raise ValueError("mass: 总质量不等于九刚体质量之和")
        if mass["calibration_status"] != CALIBRATION:
            raise ValueError("mass: 旧仿真质量必须标记待实机标定")
        if sim["odometry_adapter"] != "simulation_truth":
            raise ValueError("里程计必须标记为 simulation_truth")
        _box(sim["mast"]["center_m"], sim["mast"]["size_m"], extent, "mast")
        sensors = p["sensors"]
        if (sensors["calibration_status"] != CALIBRATION
                or sensors["origin_frame"] != sim["base_frame_id"]
                or sensors["forward_axis"] != "+X"):
            raise ValueError("sensors: 标定状态或原点/朝向不一致")
        stereo, lidar, imu = sensors["stereo"], sensors["lidar"], sensors["imu"]
        for name in ("stereo", "lidar", "imu"):
            sensor = sensors[name]
            _number(sensor["frequency_hz"], f"{name}.frequency_hz")
            if sensor["calibration_status"] != CALIBRATION:
                raise ValueError(f"{name}: 外参/频率必须标记待实机标定")
        for name, sensor in (("stereo", stereo), ("lidar", lidar)):
            _box(sensor["housing_center_m"], sensor["housing_size_m"], extent, name)
        for name, origin in (("left", stereo["left_origin_m"]),
                             ("right", stereo["right_origin_m"]), ("lidar", lidar["origin_m"])):
            _vector(origin, name)
            if not (0 < origin[0] <= extent[0] / 2 and abs(origin[1]) <= extent[1] / 2
                    and 0 < origin[2] <= extent[2]):
                raise ValueError(f"{name}: 发射原点须在车头名义包络内")
        for sensor, origin in ((stereo, stereo["left_origin_m"]),
                               (stereo, stereo["right_origin_m"]), (lidar, lidar["origin_m"])):
            if origin[0] <= sensor["housing_center_m"][0] + sensor["housing_size_m"][0] / 2:
                raise ValueError("光学/发射原点必须在外壳前方")
        left, right = stereo["left_origin_m"], stereo["right_origin_m"]
        _number(stereo["baseline_m"], "baseline_m")
        if not (math.isclose(left[0], right[0]) and math.isclose(left[2], right[2])
                and math.isclose(left[1] - right[1], stereo["baseline_m"])):
            raise ValueError("stereo: 原点与基线不一致")
        resolution = _vector(stereo["resolution"], "resolution", length=2, positive=True)
        if any(not isinstance(v, int) for v in resolution):
            raise ValueError("resolution: 必须是整数像素")
        for key in ("focal_length_mm", "horizontal_aperture_mm", "vertical_aperture_mm"):
            _number(stereo[key], key)
        _vector(imu["origin_m"], "imu.origin_m")
        ix, iy, iz = imu["origin_m"]
        if abs(ix) > extent[0] / 2 or abs(iy) > extent[1] / 2 or not 0 <= iz <= extent[2]:
            raise ValueError("imu: 原点超出车辆名义包络")
        observation = exp_source["exploration"]
        minimum = _number(sensors["minimum_effective_range_m"], "minimum_effective_range_m")
        maximum = _number(observation["sensor_range_m"], "sensor_range_m")
        fov = _number(observation["sensor_fov_deg"], "sensor_fov_deg")
        if minimum >= maximum or fov > 360:
            raise ValueError("有效观测范围或视场不合理")
        sensors["effective_observation"] = {
            "minimum_range_m": minimum, "maximum_range_m": maximum, "horizontal_fov_deg": fov}
        ros = p["ros"]
        if ros["domain_id"] != 117 or ros["localhost_only"] is not True or ros["use_sim_time"] is not True:
            raise ValueError("ROS: 必须使用 domain117 + localhost_only + sim_time")
        if (ros["frames"]["footprint"] != sim["base_frame_id"]
                or ros["frames"]["map"] != exp_source["common"]["frames"]["map"]
                or ros["frames"]["odom"] != exp_source["common"]["frames"]["odom"]
                or ros["frames"]["base"] != exp_source["common"]["frames"]["base_link"]):
            raise ValueError("ROS 控制帧或地面几何帧与源配置不一致")
        canonical = {"command": "/Car/T5/Car_Cmd_Vel", "odometry": "/Car/T3/localization/odometry",
                     "tf": "/tf", "clock": "/clock"}
        topics = ros["topics"]
        if any(topics[key] != value for key, value in canonical.items()):
            raise ValueError("ROS: canonical 接口不得变更")
        if any(not isinstance(topics[key], str) or not topics[key].startswith("/Car/T3/sensors/")
               for key in SENSOR_TOPICS):
            raise ValueError("ROS: 新传感器话题必须在 /Car/T3/sensors/ 下")
        if len(set(topics.values())) != len(topics):
            raise ValueError("ROS: 话题不能重复")
        drive = p["drive"]
        for key in ("command_timeout_s", "steering_tolerance_rad", "steering_limit_rad",
                    "stopped_linear_speed_mps", "stopped_yaw_rate_radps"):
            _number(drive[key], key)
        spin_angle = math.atan2(cap["wheelbase_m"], cap["track_width_m"])
        if (drive["command_timeout_s"] > .5 or drive["steering_limit_rad"] > math.pi / 2
                or drive["steering_limit_rad"] < spin_angle
                or drive["steering_tolerance_rad"] >= drive["steering_limit_rad"]):
            raise ValueError("drive: 超时或转向门限不合理")
        return p
    except (KeyError, TypeError, IndexError) as exc:
        raise ValueError(f"profile 缺字段或字段结构错误: {exc}") from exc
