"""TCP/OBJ只提供输入输出，导航通过同一公共组合启动。"""
import importlib.util
from pathlib import Path
import sys
import types

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('tcp_launch_helper', ROOT/'tests/launch/test_exploration_navigation_launch.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


def load(monkeypatch):
    _, _, include_type = helper._launch_module(monkeypatch)
    class Event:
        def __init__(self, *args, **kwargs):
            self.args, self.kwargs = args, kwargs
    monkeypatch.setattr(sys.modules['launch.actions'], 'RegisterEventHandler', Event, raising=False)
    events = types.ModuleType('launch.event_handlers')
    events.OnShutdown = Event
    monkeypatch.setitem(sys.modules, 'launch.event_handlers', events)
    monkeypatch.syspath_prepend(str(ROOT/'ros2_ws/src/lunar_obj_tcp_sim'))
    spec = importlib.util.spec_from_file_location('tcp_nav1cm_launch', ROOT/'ros2_ws/src/lunar_obj_tcp_sim/launch/obj_tcp.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, include_type


@pytest.mark.parametrize('mode,strategy', [('nav', 'none'), ('explore', 'frontier')])
def test_tcp_keeps_bridge_and_sensor_but_uses_shared_navigation(monkeypatch, tmp_path, mode, strategy):
    module, include_type = load(monkeypatch)
    config = yaml.safe_load((ROOT/'ros2_ws/src/lunar_obj_tcp_sim/config/simulation.yaml').read_text(encoding='utf-8'))
    config['simulation'].update(map_directory=str(tmp_path), mode=mode, start_rviz=False, start_local_rviz=False, initial_scan=False)
    path = tmp_path/'simulation.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')
    values = dict(config=str(path), host='', port='', map_directory='', mode='', prefix='',
                  start_rviz='', start_local_rviz='', platform_config='/shared/wheel.yaml',
                  build_dir=str(tmp_path/'solver'))
    actions = module.compose(helper._Context(values))
    includes = [a for a in actions if isinstance(a, include_type)]
    assert len(includes) == 1
    arguments = dict(includes[0].launch_arguments)
    assert arguments['exploration_strategy'] == strategy
    assert arguments['enable_controller'] == 'true'
    assert arguments['platform_config'] == '/shared/wheel.yaml'
    assert arguments['map_topic'] == '/lunar_sim/grid_map'
    assert arguments['odometry_topic'] == '/lunar_sim/odometry'
    assert arguments['command_topic'] == '/lunar_sim/cmd_vel'
    assert arguments['build_dir'] == str(tmp_path/'solver')
    executables = [a.kwargs.get('executable') for a in actions if hasattr(a, 'kwargs')]
    assert 'tcp_vehicle_bridge' in executables and 'obj_virtual_sensor' in executables
    assert 'lunar_incremental_navigation_node' not in executables
    assert 'obj_scan_controller' not in executables


def test_simulation_control_defaults_match_main_wheel_limits():
    config = yaml.safe_load((ROOT/'ros2_ws/src/lunar_obj_tcp_sim/config/simulation.yaml').read_text(encoding='utf-8'))['simulation']
    wheel = yaml.safe_load((ROOT/'config/wheel.yaml').read_text(encoding='utf-8'))['capability']
    assert config['max_linear_mps'] == wheel['maximum_forward_speed_mps']
    assert config['max_angular_radps'] == wheel['maximum_spin_rate_radps']
    assert config['max_angular_accel_radps2'] == wheel['maximum_yaw_acceleration_radps2']


@pytest.mark.parametrize('ready,sec,nsec,expected', [
    (False, 10, 1, False), (True, 9, 999999999, False),
    (True, 10, 1, True), (True, 10, 2, True)])
def test_obj_chunk_acknowledges_only_processed_input(ready, sec, nsec, expected, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'ros2_ws/src/lunar_obj_tcp_sim'))
    from lunar_obj_tcp_sim.observation_state import policy_map_covers_input
    response = types.SimpleNamespace(ready=ready, processed_stamp=types.SimpleNamespace(sec=sec, nanosec=nsec))
    assert policy_map_covers_input(response, 10000000001) is expected
