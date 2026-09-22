"""Platform limits must constrain, never enlarge, operator control limits."""
from dataclasses import asdict
from pathlib import Path

import pytest
import yaml

from lunar_pure_wheeled_controller.platform_policy import platform_policy
from lunar_pure_wheeled_controller.tracking import TrackingPolicy

ROOT = Path(__file__).resolve().parents[4]


def profile(name):
    return yaml.safe_load((ROOT / 'config' / f'{name}_orin.yaml').read_text())


def test_legged_limits_use_yaw_and_linear_acceleration_without_wheel_fields():
    requested = asdict(TrackingPolicy())
    requested.update(max_linear_mps=2., max_angular_radps=2., max_linear_accel_mps2=2.,
                     max_linear_decel_mps2=.15, allow_reverse=False)
    result = platform_policy(requested, profile('legged'), 'legged')
    assert result.max_linear_mps == 1.5
    assert result.max_angular_radps == 1.
    assert result.max_linear_accel_mps2 == 1.
    assert result.max_linear_decel_mps2 == .15
    assert not result.allow_reverse


def test_wheel_mapping_preserves_existing_capability_caps():
    document = profile('wheel')
    result = platform_policy(asdict(TrackingPolicy()), document, 'wheel')
    assert result.max_angular_radps == min(.5, document['capability']['maximum_spin_rate_radps'])
    assert result.max_linear_decel_mps2 == min(.5, document['capability']['maximum_braking_deceleration_mps2'])


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1., 0., True])
def test_invalid_platform_limit_is_not_hidden_by_min(value):
    document = profile('legged')
    document['capability']['maximum_yaw_rate_radps'] = value
    with pytest.raises(ValueError, match='maximum_yaw_rate_radps'):
        platform_policy(asdict(TrackingPolicy()), document, 'legged')


def test_platform_mismatch_rejected():
    with pytest.raises(ValueError, match='platform'):
        platform_policy(asdict(TrackingPolicy()), profile('wheel'), 'legged')
