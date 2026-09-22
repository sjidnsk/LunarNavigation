"""Exercise deployment policies against an ideal planar plant, without ROS nodes."""
from dataclasses import asdict
import math
from pathlib import Path

import pytest
import yaml

from lunar_pure_wheeled_controller.execution import PathExecutor
from lunar_pure_wheeled_controller.platform_policy import platform_policy
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState

ROOT = Path(__file__).resolve().parents[4]


def deployment_policy(platform):
    config = 'orin_controller' if platform == 'wheel' else 'orin_legged_controller'
    params = yaml.safe_load((ROOT / f'config/{config}.yaml').read_text())['lunar_pure_wheeled_controller']['ros__parameters']
    values = asdict(TrackingPolicy())
    values.update({key: value for key, value in params.items() if key in values})
    return platform_policy(values, yaml.safe_load((ROOT / f'config/{platform}_orin.yaml').read_text()), platform)


@pytest.mark.parametrize('platform', ['wheel', 'legged'])
@pytest.mark.parametrize('offset', [.05, .10])
def test_deployment_corrects_lateral_offset_and_stops_at_goal(platform, offset):
    policy = deployment_policy(platform)
    executor = PathExecutor(policy)
    executor.set_path([(0., 0., 0.), (2., 0., .5)], ('test', 1))
    x, y, yaw, v, w = 0., offset, 0., 0., 0.
    for _ in range(3000):
        result = executor.update(TrackingState(x, y, yaw, v, w), .05)
        assert result.command.failure_reason is None
        v, w = result.command.linear_x_mps, result.command.angular_z_radps
        assert -policy.max_reverse_mps <= v <= policy.max_linear_mps
        assert abs(w) <= policy.max_angular_radps
        if result.command.complete:
            assert math.hypot(x - 2., y) <= .1
            assert abs(yaw - .5) <= .02
            assert v == w == 0.
            break
        x += v * math.cos(yaw) * .05
        y += v * math.sin(yaw) * .05
        yaw += w * .05
    else:
        pytest.fail('ideal plant did not reach and stop at final goal')


@pytest.mark.parametrize('platform', ['wheel', 'legged'])
def test_deployment_uses_bounded_reverse_when_path_is_behind(platform):
    policy = deployment_policy(platform)
    executor = PathExecutor(policy)
    executor.set_path([(0., 0., 0.), (-2., 0., 0.)], ('test', 1))
    result = executor.update(TrackingState(0., 0., 0.), .05)
    assert result.direction == -1
    assert -policy.max_reverse_mps <= result.command.linear_x_mps < 0.
