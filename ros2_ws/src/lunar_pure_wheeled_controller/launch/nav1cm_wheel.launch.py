"""轮式标准Path组合入口；默认仅规划，控制须显式启用。"""
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from lunar_nav1cm_planner.configuration import load_configuration

DEFAULTS = {
    'start_controller':'false', 'start_rviz':'false', 'use_sim_time':'false',
    'params_file':'', 'map_frame':'map', 'map_topic':'/Car/T3/mapping/grid_map',
    'odometry_topic':'/Car/T3/localization/odometry', 'tf_topic':'/P4/input/map_to_odom', 'tf_static_topic':'/tf_static',
    'local_path_topic':'/Car/T4/planning/local_path', 'global_route_topic':'/Car/T4/planning/global_route',
    'goal_topic':'/Car/T4/navigation/goal', 'command_topic':'/Car/T5/Car_Cmd_Vel',
    'allow_unknown':'false', 'linear_speed_mps':'0.2', 'angular_speed_rps':'0.1',
    'linear_accel_mps2':'0.5', 'angular_accel_rps2':'0.5', 'curvature_inv_m':'1.0',
    'goal_tolerance_m':'0.1', 'goal_yaw_tolerance_rad':'0.02',
    'stopped_linear_mps':'0.01', 'stopped_angular_rps':'0.01',
    'mppi_nominal_response_time_s':'0.15',
}
SHARED = ('map_frame','odometry_topic','tf_topic','tf_static_topic','allow_unknown','use_sim_time','goal_yaw_tolerance_rad','stopped_linear_mps')
FOLLOWER_MAP = {
    'p3_map_topic':'map_topic', 'path_topic':'local_path_topic', 'platform_config':'wheel_config',
    'max_linear_mps':'linear_speed_mps', 'max_reverse_mps':'linear_speed_mps',
    'max_angular_radps':'angular_speed_rps', 'max_linear_accel_mps2':'linear_accel_mps2',
    'max_linear_decel_mps2':'linear_accel_mps2', 'max_angular_accel_radps2':'angular_accel_rps2',
    'max_curvature_per_m':'curvature_inv_m', 'goal_position_tolerance_m':'goal_tolerance_m',
    'stopped_angular_radps':'stopped_angular_rps',
}


def compose(context):
    values = {name:LaunchConfiguration(name).perform(context) for name in (*DEFAULTS,'wheel_config','build_dir')}
    for name in ('start_controller','start_rviz','use_sim_time','allow_unknown'):
        text=values[name].lower()
        if text not in ('true','false'): raise ValueError(f'{name} must be true or false')
        values[name]=text=='true'
    baseline,_=load_configuration(values['wheel_config'])
    for name in ('linear_speed_mps','angular_speed_rps','linear_accel_mps2','angular_accel_rps2','curvature_inv_m',
                 'goal_tolerance_m','goal_yaw_tolerance_rad','stopped_linear_mps','stopped_angular_rps','mppi_nominal_response_time_s'):
        values[name]=float(values[name])
        if not math.isfinite(values[name]) or values[name]<0 or (name!='mppi_nominal_response_time_s' and values[name]==0):
            raise ValueError(f'invalid {name}')
        if hasattr(baseline,name): values[name]=min(values[name],getattr(baseline,name))
    planner={name:value for name,value in values.items() if name not in
             ('start_controller','start_rviz','params_file','command_topic','mppi_nominal_response_time_s')}
    # 参数文件提供规划器专用设置；共享接口及能力约束由此组合入口最后统一覆盖。
    params=([values['params_file']] if values['params_file'] else [])+[planner]
    actions=[Node(package='lunar_nav1cm_planner', executable='planner_node',output='screen',parameters=params)]
    if values['start_controller']:
        follower={name:values[name] for name in SHARED}
        follower.update({name:values[shared] for name,shared in FOLLOWER_MAP.items()})
        follower.update(input_mode='incremental_path',controller_backend='mppi_path_follower',standard_map_source='direct_p3',
                        command_topic=values['command_topic'],mppi_nominal_response_time_s=values['mppi_nominal_response_time_s'])
        actions.append(Node(package='lunar_pure_wheeled_controller',executable='lunar_pure_wheeled_controller_node.py',output='screen',parameters=[follower]))
    if values['start_rviz']:
        actions.append(Node(package='rviz2',executable='rviz2',parameters=[{'use_sim_time':values['use_sim_time']}],remappings=[('/goal_pose',values['goal_topic'])]))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('lunar_pure_wheeled_controller'))
    return LaunchDescription([
        *(DeclareLaunchArgument(name,default_value=value) for name,value in DEFAULTS.items()),
        DeclareLaunchArgument('wheel_config',default_value=str(share/'config/wheel_orin.yaml')),
        DeclareLaunchArgument('build_dir',default_value=EnvironmentVariable('NAV1CM_BUILD_DIR',default_value='')),
        OpaqueFunction(function=compose),
    ])
