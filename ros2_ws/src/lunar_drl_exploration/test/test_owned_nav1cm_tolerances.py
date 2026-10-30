"""纯参数合同：执行真实owned分支，不构造ROS环境/训练/策略。"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from lunar_drl_exploration.navigation_backend import owned_nav1cm_specs


def assert_tolerances(specs, position, yaw):
    mapper, planner, follower = [values for _, _, values in specs]
    assert mapper.get('goal_position_tolerance_m', .1) == position
    assert planner.get('goal_tolerance_m', .1) == position
    assert follower.get('goal_position_tolerance_m', .1) == position
    assert all(values.get('goal_yaw_tolerance_rad', .05) == yaw for values in (mapper, planner, follower))


def test_helper_defaults_preserve_common_launch_tolerances():
    specs = owned_nav1cm_specs('/private', platform_config='/wheel.yaml', build_dir='/cache', use_sim_time=True)
    assert_tolerances(specs, .1, .05)


@pytest.mark.parametrize('position,yaw', [(.05, math.pi / 12), (.03, .07)])
def test_helper_carries_custom_tolerances_to_all_three_nodes(position, yaw):
    specs = owned_nav1cm_specs('/private', platform_config='/wheel.yaml', build_dir='/cache', use_sim_time=True,
        goal_position_tolerance_m=position, goal_yaw_tolerance_rad=yaw)
    assert_tolerances(specs, position, yaw)


@pytest.mark.parametrize('position,yaw', [(.05, math.pi / 12), (.03, .07)])
def test_rosenv_owned_branch_passes_configured_tolerances_to_real_specs(position, yaw):
    # 编译实际_start_ros中带return的nav1cm分支，跳过DomainLease/rclpy/线程启动。
    # AST节点原样执行；若caller漏传/错传或三方名称不符，记录的参数就会失败。
    path = Path(__file__).resolve().parents[1] / 'lunar_drl_exploration/ros_env.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'RosExplorationEnv')
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '_start_ros')
    branch = next(node for node in method.body if isinstance(node, ast.If)
                  and any(isinstance(child, ast.Return) for child in node.body))
    shell = ast.parse('def call(self, ns, env): pass')
    shell.body[0].body = [branch]
    ast.fix_missing_locations(shell)
    scope = dict(Path=Path, __package__='lunar_drl_exploration')
    exec(compile(shell, str(path), 'exec'), scope)
    starts = []
    instance = NS(_navigation_backend='nav1cm', config=NS(platform=NS(capability_path='/wheel.yaml'),
        artifact_root='/artifacts', output_dir='/unused', goal_position_tolerance_m=position,
        goal_yaw_tolerance_rad=yaw), owned=NS(start=lambda p, e, ns, parameters, env: starts.append((p, e, parameters))))
    scope['call'](instance, '/private', {})
    assert len(starts) == 3
    assert_tolerances(starts, position, yaw)
