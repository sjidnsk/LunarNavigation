"""Stage-one legged vx/omega adapter using the shared reference executor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('lunar_pure_wheeled_controller'), 'config'])
    arguments = {
        'config_file': PathJoinSubstitution([config_dir, 'orin_legged_controller.yaml']),
        'platform_config': PathJoinSubstitution([config_dir, 'legged_orin.yaml']),
        'command_topic': '/Car/T5/Car_Cmd_Vel',
        'odometry_topic': '/Car/T3/localization/odometry',
        'tf_topic': '/P4/input/map_to_odom',
        'path_reference_topic': '/Car/T4/planning/path_reference',
        'tracking_status_topic': '/Car/T4/control/tracking_status',
        'use_sim_time': 'false',
    }
    overrides = {name: ParameterValue(LaunchConfiguration(name), value_type=str)
                 for name in arguments if name not in {'config_file', 'use_sim_time'}}
    overrides.update(platform_type='legged', input_mode='incremental_reference',
                     use_sim_time=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool))
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()),
        Node(package='lunar_pure_wheeled_controller', executable='lunar_pure_wheeled_controller_node.py',
             name='lunar_pure_wheeled_controller', output='screen',
             parameters=[LaunchConfiguration('config_file'), overrides]),
    ])
