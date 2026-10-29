"""启动独立 P3 Path 规划器；topic 可用参数或 ROS remap 覆盖。"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory('lunar_nav1cm_planner')
    return LaunchDescription([
        DeclareLaunchArgument('build_dir', default_value=EnvironmentVariable('NAV1CM_BUILD_DIR', default_value='')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config/planner.yaml')),
        Node(package='lunar_nav1cm_planner', executable='planner_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), {'build_dir': LaunchConfiguration('build_dir')}]),
    ])
