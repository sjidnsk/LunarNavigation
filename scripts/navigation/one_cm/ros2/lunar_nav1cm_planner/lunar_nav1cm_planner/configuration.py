"""平台能力 → 共用平面运动约束 → 显式覆盖；地形调用各平台既有 core。"""
from dataclasses import replace
from pathlib import Path

import yaml

from nav1cm.contracts import WheelLimits, resolve_minimum_clearance
from nav1cm.terrain_native import terrain_capability_from_mapping


def load_configuration(path, overrides=None):
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    capability = document['capability']
    extent = capability['body_extent_m']
    platform = document.get('platform', 'wheel')
    if platform not in {'wheel', 'legged'}:
        raise ValueError('unsupported navigation platform')
    legged = platform == 'legged'
    acceleration = capability['maximum_linear_acceleration_mps2'] if legged else min(
        capability['maximum_acceleration_mps2'], capability['maximum_braking_deceleration_mps2'])
    limits = WheelLimits(
        body_length_m=float(extent[0]), body_width_m=float(extent[1]),
        motion_model=('differential_drive' if legged else capability.get('motion_model','curvature_limited')),
        linear_speed_mps=min(float(capability['maximum_forward_speed_mps']),
                             float(capability['maximum_reverse_speed_mps'])),
        angular_speed_rps=float(capability['maximum_yaw_rate_radps' if legged else 'maximum_spin_rate_radps']),
        # 规划模型只有一项对称导数上限，取增速/制动共同可执行的包络；不混用正负方向。
        linear_accel_mps2=float(acceleration),
        angular_accel_rps2=float(capability['maximum_yaw_acceleration_radps2']),
        curvature_inv_m=(1.0 if legged else float(capability['maximum_curvature_per_m'])),
        minimum_clearance_m=resolve_minimum_clearance(
            capability['minimum_body_clearance_m'] if legged else capability.get('minimum_clearance_m',0.0)))
    if overrides:
        values = {key: (value if key=='motion_model' else float(value)) for key,value in overrides.items()}
        values["minimum_clearance_m"] = resolve_minimum_clearance(limits.minimum_clearance_m, values.get("minimum_clearance_m"))
        limits = replace(limits, **values)
    return limits, terrain_capability_from_mapping(document)
