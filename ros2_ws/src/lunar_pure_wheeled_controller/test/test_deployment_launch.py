"""Launch defaults must load deployment files, while explicit overrides win."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / 'ros2_ws/src/lunar_pure_wheeled_controller'


def launched_parameters(platform, **overrides):
    filename = PACKAGE / 'launch' / f'pure_{"wheeled" if platform == "wheel" else "legged"}_controller.launch.py'
    spec = importlib.util.spec_from_file_location('deployment_launch', filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    description = module.generate_launch_description()
    nodes = []
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            nodes.extend(action.execute(context))
        else:
            nodes.append(action)
    parameters = evaluate_parameters(context, nodes[0]._Node__parameters)
    resolved = {}
    for value in parameters:
        if isinstance(value, Path):
            resolved.update(yaml.safe_load(value.read_text())['lunar_pure_wheeled_controller']['ros__parameters'])
        else:
            resolved.update(value)
    return resolved


@pytest.mark.parametrize('platform', ['wheel', 'legged'])
def test_launch_uses_matching_deployment_policy(platform):
    params = launched_parameters(platform)
    assert params['input_mode'] == 'incremental_reference'
    assert params['goal_yaw_tolerance_rad'] == .02
    assert params['goal_position_tolerance_m'] == .1
    assert params['tf_timeout_s'] == 5.
    assert params['max_linear_mps'] == (.2 if platform == 'wheel' else .1)
    assert yaml.safe_load(Path(params['platform_config']).read_text())['platform'] == platform


@pytest.mark.parametrize('platform', ['wheel', 'legged'])
def test_explicit_launch_overrides_win_without_erasing_other_yaml_values(platform, tmp_path):
    config = tmp_path / 'custom.yaml'
    config.write_text(yaml.safe_dump({'lunar_pure_wheeled_controller': {'ros__parameters': {
        'max_linear_mps': .07, 'tf_timeout_s': 8., 'command_topic': '/custom/cmd',
    }}}))
    params = launched_parameters(platform, config_file=str(config), max_linear_mps='.06',
                                 allow_reverse='false', command_topic='/override/cmd')
    assert params['max_linear_mps'] == .06
    assert params['tf_timeout_s'] == 8.
    assert params['allow_reverse'] is False
    assert params['command_topic'] == '/override/cmd'


def test_wheel_legacy_demo_can_explicitly_select_motion_reference():
    params = launched_parameters('wheel', input_mode='motion_reference', tf_topic='/demo/tf',
                                 goal_position_tolerance_m='.2', max_angular_radps='.5')
    assert params['input_mode'] == 'motion_reference'
    assert params['tf_topic'] == '/demo/tf'
    assert params['goal_position_tolerance_m'] == .2
    assert params['max_angular_radps'] == .5


def test_missing_selected_yaml_fails_before_creating_node(tmp_path):
    with pytest.raises(ValueError, match='config_file does not exist'):
        launched_parameters('wheel', config_file=str(tmp_path / 'absent.yaml'))


def test_installed_launch_resolves_installed_configs(tmp_path, monkeypatch):
    from lunar_pure_wheeled_controller.deployment_launch import config_directory
    prefix = tmp_path / 'install'
    package = prefix / 'share/lunar_pure_wheeled_controller'
    launch = package / 'launch/pure_wheeled_controller.launch.py'
    launch.parent.mkdir(parents=True)
    launch.write_text('# installed launch fixture')
    (package / 'config').mkdir()
    index = prefix / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    (index / 'lunar_pure_wheeled_controller').touch()
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    assert config_directory(launch) == package / 'config'
    assert config_directory(PACKAGE / 'launch/pure_wheeled_controller.launch.py') == ROOT / 'config'
