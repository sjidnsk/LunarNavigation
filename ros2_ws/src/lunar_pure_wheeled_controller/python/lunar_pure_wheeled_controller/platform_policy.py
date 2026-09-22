"""Clamp planar tracking limits to the selected platform's capability schema."""
from math import isfinite

from .tracking import TrackingPolicy, _valid_policy


def platform_policy(values, document, platform_type):
    """Keep braking/curvature as controller limits when absent from legged schema."""
    if platform_type not in {'wheel', 'legged'}:
        raise ValueError('platform_type must be wheel or legged')
    if not isinstance(document, dict) or document.get('platform') != platform_type:
        raise ValueError('platform_config does not match platform_type')
    capability = document.get('capability')
    if not isinstance(capability, dict):
        raise ValueError('platform_config must contain capability')
    policy = dict(values)
    if not _valid_policy(TrackingPolicy(**policy)):
        raise ValueError('invalid requested tracking limits')
    names = dict(
        max_linear_mps='maximum_forward_speed_mps',
        max_reverse_mps='maximum_reverse_speed_mps',
        max_angular_accel_radps2='maximum_yaw_acceleration_radps2',
    )
    if platform_type == 'wheel':
        names.update(
            max_angular_radps='maximum_spin_rate_radps',
            max_linear_accel_mps2='maximum_acceleration_mps2',
            max_linear_decel_mps2='maximum_braking_deceleration_mps2',
            max_curvature_per_m='maximum_curvature_per_m',
            max_lateral_accel_mps2='maximum_lateral_acceleration_mps2',
        )
    else:
        names.update(
            max_angular_radps='maximum_yaw_rate_radps',
            max_linear_accel_mps2='maximum_linear_acceleration_mps2',
        )
    for target, source in names.items():
        value = capability.get(source)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value) or value <= 0:
            raise ValueError(f'platform capability {source} must be finite and positive')
        policy[target] = min(float(policy[target]), float(value))
    return TrackingPolicy(**policy)
