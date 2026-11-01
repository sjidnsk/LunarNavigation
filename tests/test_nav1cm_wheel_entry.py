"""轮式入口显式共享标准Path参数，导航默认不发控制。"""
import ast
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
LAUNCH=ROOT/'ros2_ws/src/lunar_pure_wheeled_controller/launch/nav1cm_wheel.launch.py'

def constants():
    assert LAUNCH.is_file(), 'missing wheel composite launch'
    tree=ast.parse(LAUNCH.read_text(encoding='utf-8'))
    return {n.targets[0].id:ast.literal_eval(n.value) for n in tree.body
            if isinstance(n,ast.Assign) and isinstance(n.targets[0],ast.Name)}

def test_navigation_only_and_production_response():
    c=constants()
    assert c['DEFAULTS']['start_controller']=='false'
    assert c['DEFAULTS']['mppi_nominal_response_time_s']=='0.15'
    assert c['DEFAULTS']['angular_speed_rps']=='1.0'
    assert float(c['DEFAULTS']['goal_yaw_tolerance_rad'])==math.radians(5.)

def test_shared_interfaces_and_limits():
    c=constants()
    follower=c['FOLLOWER_MAP']
    for name, shared in {'p3_map_topic':'map_topic','path_topic':'local_path_topic',
        'platform_config':'wheel_config','max_linear_mps':'linear_speed_mps',
        'max_angular_radps':'angular_speed_rps','goal_position_tolerance_m':'goal_tolerance_m',
        'stopped_angular_radps':'stopped_angular_rps'}.items():
        assert follower[name]==shared
    assert {'tf_topic','tf_static_topic','map_frame','odometry_topic','allow_unknown'} <= set(c['SHARED'])

def test_wheel_selector_and_external_discovery():
    start=(ROOT/'scripts/orin/start_navigation.sh').read_text(encoding='utf-8')
    assert 'wheel_navigation.launch.py' in start
    assert 'enable_controller:=false' in start
    assert 'incremental_v2' in start and 'legged' in start
    build=(ROOT/'scripts/orin/build.sh').read_text(encoding='utf-8')
    assert '--base-paths' in build and 'scripts/navigation/one_cm/ros2/lunar_nav1cm_planner' in build
    assert 'LUNAR_INSTALL_ROOT' in build


def test_goal_helper_uses_numeric_argument_parser_and_no_action_in_new_mode():
    helper=ROOT/'scripts/orin/navigation_goal.py'
    assert helper.exists()
    import importlib.util
    spec=importlib.util.spec_from_file_location('navigation_goal',helper)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    import json,math,pytest
    cmd=module.command(['1','2'])
    assert cmd[:5]==['ros2','topic','pub','--once','/Car/T4/navigation/goal']
    data=json.loads(cmd[-1]);assert data['pose']['orientation']=={'z':0.0,'w':1.0}
    cmd=module.command(['1','2','1.2','--mode','incremental_v2'])
    assert json.loads(cmd[-1])['has_target_yaw'] is True
    assert json.loads(module.command(['1','2','--mode','incremental_v2'])[-1])['has_target_yaw'] is False
    with pytest.raises(SystemExit): module.command(['$(touch BAD)','2'])
    with pytest.raises(SystemExit): module.command(['nan','2'])
