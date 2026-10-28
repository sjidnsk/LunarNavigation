"""用真实LaunchContext验证组合入口及能力上限。"""
import importlib.util
from pathlib import Path
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
from launch_ros.actions import Node


def load():
    path=Path(__file__).resolve().parents[1]/'launch/nav1cm_wheel.launch.py'
    spec=importlib.util.spec_from_file_location('nav1cm_wheel',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_navigation_only_and_shared_limits():
    from ament_index_python.packages import get_package_share_directory
    module=load();ctx=LaunchContext()
    ctx.launch_configurations.update(module.DEFAULTS)
    ctx.launch_configurations.update(wheel_config=str(Path(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml'),build_dir='/tmp/nav1cm-unused')
    assert len(module.compose(ctx))==1
    ctx.launch_configurations.update(start_controller='true',map_topic='/private/map',tf_topic='/private/tf',tf_static_topic='/private/static',local_path_topic='/private/path',angular_speed_rps='12.0')
    nodes=module.compose(ctx);assert len(nodes)==2
    planner,follower=[evaluate_parameters(ctx,n._Node__parameters)[-1] for n in nodes]
    assert planner['map_topic']==follower['p3_map_topic']=='/private/map'
    assert planner['local_path_topic']==follower['path_topic']=='/private/path'
    assert planner['angular_speed_rps']==follower['max_angular_radps']==1.0
    assert planner['goal_yaw_tolerance_rad']==follower['goal_yaw_tolerance_rad']==0.02
    for key in ('tf_topic','tf_static_topic','map_frame','allow_unknown','odometry_topic'): assert planner[key]==follower[key]
    assert follower['mppi_nominal_response_time_s']==0.15
    assert follower['standard_map_source']=='direct_p3'

    ctx.launch_configurations.update(start_rviz='true',goal_topic='/private/goal')
    rviz=module.compose(ctx)[-1]
    from launch.utilities import perform_substitutions
    remaps=[(perform_substitutions(ctx,a),perform_substitutions(ctx,b)) for a,b in rviz._Node__remappings]
    assert ('/goal_pose','/private/goal') in remaps


def test_application_profile_retains_explicit_deployment_limits():
    module=load()
    assert module.DEFAULTS['start_controller']=='false'
    assert float(module.DEFAULTS['angular_speed_rps'])==.1
    assert float(module.DEFAULTS['goal_yaw_tolerance_rad'])==.02
    assert float(module.DEFAULTS['mppi_nominal_response_time_s'])==.15
    assert module.DEFAULTS['tf_topic']=='/P4/input/map_to_odom'


def test_clearance_file_and_request_are_shared_and_never_lowered():
    import yaml
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml'
    configured=float(yaml.safe_load(config.read_text())['capability']['minimum_clearance_m'])
    module=load();ctx=LaunchContext();ctx.launch_configurations.update(module.DEFAULTS)
    ctx.launch_configurations.update(wheel_config=str(config),build_dir='/tmp/nav1cm-unused',start_controller='true')
    for request,expected in [('',configured),('0',configured),(str(configured+.2),configured+.2)]:
        ctx.launch_configurations['minimum_clearance_m']=request
        planner,follower=[evaluate_parameters(ctx,n._Node__parameters)[-1] for n in module.compose(ctx)]
        assert planner['minimum_clearance_m']==follower['minimum_clearance_m']==expected
