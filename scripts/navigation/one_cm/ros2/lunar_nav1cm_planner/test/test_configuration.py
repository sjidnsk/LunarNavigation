"""安装配置与能力链，测试不导入 rclpy。"""
from pathlib import Path
import ast
import sys
from importlib import import_module
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parents[1]))


def test_wheel_yaml_drives_full_limits_then_explicit_overrides():
    assert (PACKAGE / 'lunar_nav1cm_planner/configuration.py').exists(), '能力配置入口尚未实现'
    module = import_module('lunar_nav1cm_planner.configuration')
    limits, capability = module.load_configuration(PACKAGE.parents[4] / 'config/wheel.yaml')
    assert (limits.body_length_m, limits.body_width_m, limits.linear_speed_mps,
            limits.angular_speed_rps, limits.linear_accel_mps2,
            limits.angular_accel_rps2, limits.curvature_inv_m) == pytest.approx((1.182, .818, .2, 1., .5, .5, 1.))
    altered, same_terrain = module.load_configuration(PACKAGE.parents[4] / 'config/wheel.yaml', {'angular_speed_rps': .4})
    assert altered.angular_speed_rps == .4
    assert same_terrain == capability


def test_package_installs_core_native_config_and_three_path_outputs():
    assert (PACKAGE / 'CMakeLists.txt').exists(), '独立 ament 安装包尚未实现'
    cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'ament_python_install_package(nav1cm' in cmake
    assert 'lunar_incremental_navigation_core::lunar_incremental_navigation_core' in cmake
    assert 'native_geometry.cpp' in cmake and 'distance_field_native.cpp' in cmake
    assert 'terrain_bridge.cpp' in cmake and 'wheel.yaml' in cmake
    assert (PACKAGE / 'launch/planner.launch.py').exists()
    node = (PACKAGE / 'lunar_nav1cm_planner/node.py').read_text(encoding='utf-8')
    assert 'Car_Cmd_Vel' not in node and 'verification_plant' not in node
    tree = ast.parse(node)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PlannerNode')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    # 验证三个路径输出的类型、参数话题与保留深度，不限制可选发布器数量。
    publishers = {}
    for statement in init.body:
        if isinstance(statement, ast.Assign) and isinstance(statement.value, ast.Call):
            for target in statement.targets:
                if isinstance(target, ast.Attribute) and target.attr in (
                        '_global_publisher', '_reference_publisher', '_local_publisher'):
                    publishers[target.attr] = statement
    created = []
    settings = {key: '/test/' + key for key in (
        'global_route_topic', 'global_reference_topic', 'local_path_topic')}
    probe = SimpleNamespace(create_publisher=lambda *args: created.append(args))
    namespace = dict(self=probe, settings=settings, PathMessage=object())
    exec(compile(ast.Module(body=list(publishers.values()), type_ignores=[]),
                 str(PACKAGE / 'lunar_nav1cm_planner/node.py'), 'exec'), namespace)
    assert created == [(namespace['PathMessage'], settings[key], 1) for key in (
        'global_route_topic', 'global_reference_topic', 'local_path_topic')]


@pytest.mark.parametrize('backend', ['path_mppi', 'mpcc_cmd_vel'])
@pytest.mark.parametrize('controller', [False, True])
@pytest.mark.parametrize('execution_contract', [False, True])
def test_control_publishers_require_their_enabled_backend(
        backend, controller, execution_contract, monkeypatch):
    # 仅执行实际初始化中的控制接线，ROS 消息和运行资源用最小替身隔离。
    path = PACKAGE / 'lunar_nav1cm_planner/node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PlannerNode')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    selected = []
    for statement in init.body:
        if isinstance(statement, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'direct' or
                isinstance(t, ast.Attribute) and t.attr in (
                    '_execution_contract_enabled', '_execution_revision')
                for t in statement.targets):
            selected.append(statement)
        elif isinstance(statement, ast.If) and (
                isinstance(statement.test, ast.Name) and statement.test.id == 'direct' or
                isinstance(statement.test, ast.Attribute) and
                statement.test.attr == '_execution_contract_enabled'):
            selected.append(statement)
    direct_calls, execution_calls, published, subscribed = [], [], [], []
    probe = SimpleNamespace(
        create_publisher=lambda *args: published.append(args),
        create_subscription=lambda *args: subscribed.append(args),
        _publish_command=object(), _on_execution_status=object(),
        runtime=SimpleNamespace(
            configure_direct_control=lambda *a, **kw: direct_calls.append((a, kw)),
            enable_execution_contract=execution_calls.append))
    messages = SimpleNamespace(PathExecutionReference=object(), TrackingStatus=object())
    monkeypatch.setitem(sys.modules, 'lunar_planning_msgs.msg', messages)
    settings = dict(control_backend=backend, enable_controller=controller,
        execution_contract_enabled=execution_contract, command_topic='/test/command',
        command_hold_s=.2, horizon_dt_s=.1, command_spin_prepare_s=.3,
        execution_feedback_timeout_s=2., execution_reference_topic='/test/execution',
        tracking_status_topic='/test/status')
    twist = object()
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'),
         dict(self=probe, settings=settings, Twist=twist))
    direct = backend == 'mpcc_cmd_vel' and controller
    execution = backend == 'path_mppi' and execution_contract
    assert bool(direct_calls) == direct
    assert bool(execution_calls) == execution
    assert ((twist, '/test/command', 1) in published) == direct
    assert ((messages.PathExecutionReference, '/test/execution', 10) in published) == execution
    assert bool(subscribed) == execution
    if execution:
        assert subscribed == [(messages.TrackingStatus, '/test/status', probe._on_execution_status, 10)]


@pytest.mark.parametrize('custom_speed', [None, (.17, .13, .23)])
def test_tcp_merged_profile_loads_speed_and_unchanged_braking_limits(tmp_path, monkeypatch, custom_speed):
    import yaml
    root=PACKAGE.parents[4]
    monkeypatch.syspath_prepend(str(root))
    from scripts.simulation.p4_tcp_runtime import tcp_platform_profile
    base=yaml.safe_load((root/'config/wheel.yaml').read_text(encoding='utf-8'))
    speed=yaml.safe_load((root/'config/tcp_wheel_speed.yaml').read_text(encoding='utf-8'))
    if custom_speed is not None:
        speed = dict(zip(('maximum_forward_speed_mps', 'maximum_reverse_speed_mps',
                          'maximum_spin_rate_radps'), custom_speed))
    path=tmp_path/'tcp-profile.yaml';path.write_text(yaml.safe_dump(tcp_platform_profile(base,speed)),encoding='utf-8')
    limits,_=import_module('lunar_nav1cm_planner.configuration').load_configuration(path)
    capability = base['capability']
    assert (limits.linear_speed_mps, limits.angular_speed_rps,
            limits.linear_accel_mps2, limits.angular_accel_rps2) == pytest.approx((
        min(speed['maximum_forward_speed_mps'], speed['maximum_reverse_speed_mps']),
        speed['maximum_spin_rate_radps'],
        min(capability['maximum_acceleration_mps2'], capability['maximum_braking_deceleration_mps2']),
        capability['maximum_yaw_acceleration_radps2']))
