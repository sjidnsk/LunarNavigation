"""Deployment launch contract shared by wheel and stage-one legged adapters.

The selected YAML is authoritative. Optional launch arguments override only the
values explicitly supplied, so defaults cannot silently mask a custom YAML.
"""
from dataclasses import fields
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from .tracking import TrackingPolicy

_INHERIT = '__from_config__'
_STRING_PARAMETERS = (
    'input_mode', 'map_frame', 'odom_frame', 'base_frame', 'path_reference_topic',
    'tracking_status_topic', 'reference_topic', 'path_topic', 'odometry_topic',
    'command_topic', 'execution_cancel_topic', 'tf_topic',
)


def config_directory(launch_file):
    """Use adjacent source config, otherwise the installed package config."""
    source = Path(launch_file).resolve().parents[4] / 'config'
    if (source / 'orin_controller.yaml').is_file():
        return source
    return Path(get_package_share_directory('lunar_pure_wheeled_controller')) / 'config'


def generate_controller_launch(platform, launch_file):
    if platform not in {'wheel', 'legged'}:
        raise ValueError('controller platform must be wheel or legged')
    directory = config_directory(launch_file)
    filename = 'orin_controller.yaml' if platform == 'wheel' else 'orin_legged_controller.yaml'
    arguments = {
        'config_file': str(directory / filename),
        'platform_config': str(directory / f'{platform}_orin.yaml'),
    }
    types = {name: str for name in _STRING_PARAMETERS}
    types.update({field.name: bool if field.name == 'allow_reverse' else float
                  for field in fields(TrackingPolicy)})
    types.update(control_rate_hz=float, odometry_timeout_s=float, tf_timeout_s=float,
                 use_sim_time=bool)
    arguments.update({name: _INHERIT for name in types})

    def launch_node(context):
        config = LaunchConfiguration('config_file').perform(context)
        # Fail before starting a publisher if an explicitly selected config is absent.
        if not Path(config).is_file():
            raise ValueError(f'controller config_file does not exist: {config}')
        overrides = {
            'platform_type': platform,
            'platform_config': ParameterValue(
                LaunchConfiguration('platform_config').perform(context), value_type=str),
        }
        for name, value_type in types.items():
            value = LaunchConfiguration(name).perform(context)
            if value != _INHERIT:
                overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
        return [Node(
            package='lunar_pure_wheeled_controller',
            executable='lunar_pure_wheeled_controller_node.py',
            name='lunar_pure_wheeled_controller', output='screen',
            parameters=[config, overrides],
        )]

    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()),
        OpaqueFunction(function=launch_node),
    ])
