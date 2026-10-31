"""安装配置与能力链，测试不导入 rclpy。"""
from pathlib import Path
import sys
from importlib import import_module

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
    assert node.count('create_publisher(') == 4
    assert 'execution_contract_enabled' in node


def test_tcp_merged_profile_loads_speed_and_unchanged_braking_limits(tmp_path, monkeypatch):
    import yaml
    root=PACKAGE.parents[4]
    monkeypatch.syspath_prepend(str(root))
    from scripts.simulation.p4_tcp_runtime import tcp_platform_profile
    base=yaml.safe_load((root/'config/wheel.yaml').read_text(encoding='utf-8'))
    speed=yaml.safe_load((root/'config/tcp_wheel_speed.yaml').read_text(encoding='utf-8'))
    path=tmp_path/'tcp-profile.yaml';path.write_text(yaml.safe_dump(tcp_platform_profile(base,speed)),encoding='utf-8')
    limits,_=import_module('lunar_nav1cm_planner.configuration').load_configuration(path)
    assert (limits.linear_speed_mps,limits.angular_speed_rps,limits.linear_accel_mps2,limits.angular_accel_rps2)==pytest.approx((.5,1.,.5,.5))
