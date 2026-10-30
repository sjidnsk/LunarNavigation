"""wheel.yaml → WheelLimits → 显式参数覆盖；地形阈值仍由同一能力文件提供。"""
from dataclasses import replace
from pathlib import Path

import yaml

from nav1cm.contracts import WheelLimits, resolve_minimum_clearance
from nav1cm.terrain_native import WheelTerrainCapability


def load_configuration(path, overrides=None):
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    capability = document['capability']
    extent = capability['body_extent_m']
    limits = WheelLimits(
        body_length_m=float(extent[0]), body_width_m=float(extent[1]),
        motion_model=capability.get('motion_model','curvature_limited'),
        linear_speed_mps=min(float(capability['maximum_forward_speed_mps']),
                             float(capability['maximum_reverse_speed_mps'])),
        angular_speed_rps=float(capability['maximum_spin_rate_radps']),
        linear_accel_mps2=min(float(capability['maximum_acceleration_mps2']),
                              float(capability['maximum_braking_deceleration_mps2'])),
        angular_accel_rps2=float(capability['maximum_yaw_acceleration_radps2']),
        curvature_inv_m=float(capability['maximum_curvature_per_m']),
        minimum_clearance_m=resolve_minimum_clearance(capability.get('minimum_clearance_m',0.0)))
    if overrides:
        values = {key: (value if key=='motion_model' else float(value)) for key,value in overrides.items()}
        values["minimum_clearance_m"] = resolve_minimum_clearance(limits.minimum_clearance_m, values.get("minimum_clearance_m"))
        limits = replace(limits, **values)
    return limits, WheelTerrainCapability.from_mapping(document)
