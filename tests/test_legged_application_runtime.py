"""足式使用公共平面导航，能力和地形判定保持足式来源。"""
from pathlib import Path
import importlib
import sys
import yaml
import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'scripts/navigation/one_cm/ros2/lunar_nav1cm_planner'
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE.parents[1]))

def test_legged_loads_own_kinematics_and_terrain():
    config = importlib.import_module('lunar_nav1cm_planner.configuration')
    limits, terrain = config.load_configuration(ROOT / 'config/legged_orin.yaml')
    assert (limits.body_length_m, limits.body_width_m) == (.68, .33)
    assert (limits.linear_speed_mps, limits.angular_speed_rps) == (1.5, 1.)
    assert (limits.linear_accel_mps2, limits.angular_accel_rps2) == (1., 1.)
    assert limits.motion_model == 'differential_drive'
    assert type(terrain).__name__ == 'LeggedTerrainCapability'
    assert terrain.maximum_step_height_m == .5
    assert terrain.maximum_gap_width_m == .3
    assert limits.global_clearance_radius() == pytest.approx((.68**2 + .33**2)**.5/2 + .3)

def test_legged_overrides_do_not_change_terrain():
    config = importlib.import_module('lunar_nav1cm_planner.configuration')
    limits, terrain = config.load_configuration(ROOT / 'config/legged_orin.yaml', {'linear_speed_mps': .1})
    assert limits.linear_speed_mps == .1
    assert terrain.maximum_step_height_m == .5

def test_legged_uses_same_launch_graph_with_own_platform(monkeypatch):
    from tests.test_wheel_application_runtime import compose, helper
    _, actions, _, _ = compose(monkeypatch, platform_type='legged')
    args = actions[0].launch_arguments
    assert Path(args['platform_config']).name == 'legged_orin.yaml'
    assert args['platform_selector'] == 'legged'
    assert args['allow_unknown'] == 'false'
    assert args['control_backend'] == 'mpcc_cmd_vel'
    assert args['enable_controller'] == 'false'
    assert args['command_topic'] == '/Car/T5/Car_Cmd_Vel'
    monkeypatch.setattr(helper, 'LAUNCH', ROOT / 'launch/nav1cm_exploration.launch.py')
    shared, _, _ = helper._launch_module(monkeypatch)
    graph = shared._compose(helper._Context(args))
    params = [x.kwargs['parameters'][0] for x in graph]
    assert len(params) == 3
    assert params[2]['platform_selector'] == 'legged'
    assert params[0]['platform_config'] == params[1]['wheel_config'] == params[2]['platform_config']

def test_unknown_platform_is_rejected(monkeypatch):
    from tests.test_wheel_application_runtime import compose
    with pytest.raises((RuntimeError, ValueError), match='platform'):
        compose(monkeypatch, platform_type='unknown')

def test_legged_scripts_no_longer_fall_back_to_old_algorithm():
    for name in ('start_navigation.sh', 'start_exploration.sh'):
        text = (ROOT / 'scripts/orin' / name).read_text(encoding='utf-8')
        assert 'nav1cm 仅支持 wheel' not in text
        assert 'then stack_mode=incremental_v2' not in text
        assert 'then navigation_backend=incremental_v2' not in text
        assert 'platform_type:="$platform_type"' in text
    text = (ROOT / 'scripts/orin/start_hardware_navigation.sh').read_text(encoding='utf-8')
    assert 'start_navigation.sh" legged' in text

def test_legged_resources_installed():
    text = (ROOT / 'ros2_ws/src/lunar_pure_exploration_ros/CMakeLists.txt').read_text(encoding='utf-8')
    assert '/legged_navigation.yaml' in text
    assert '/legged_orin.yaml' in text


def test_native_terrain_uses_legged_evaluator_not_wheel():
    import os
    import numpy as np
    from nav1cm.terrain_native import NativeTerrain, LeggedTerrainCapability, WheelTerrainCapability
    library = os.environ.get('NAV1CM_LEGGED_TERRAIN_LIB')
    if not library:
        pytest.skip('newly built terrain library required')
    # 同一坡面仅改变能力，验证未把足式参数转给轮式 evaluator。
    z = np.tile(np.arange(9, dtype=np.float32)*.2*np.tan(.45), (9, 1))
    def center(capability):
        with NativeTerrain(library, capability=capability) as terrain:
            result = terrain.apply(z, resolution_m=.2, origin_xy_m=(0.,0.))
        return next(int(state) for x0,y0,x1,y1,state in result.regions if x0 <= 4 < x1 and y0 <= 4 < y1)
    assert center(LeggedTerrainCapability(.5235987756,.5,.3)) == 0
    assert center(WheelTerrainCapability(.34906585,.2,.21)) == 1
    assert center(LeggedTerrainCapability(.4,.5,.3)) == 1


def test_legged_configuration_is_unchanged_from_source():
    import subprocess
    original = subprocess.check_output(['git','show','ff75f52:config/legged_orin.yaml'])
    assert (ROOT/'config/legged_orin.yaml').read_bytes() == original
