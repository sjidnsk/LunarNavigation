import importlib.util
from pathlib import Path
import pytest
from launch import LaunchContext

PATH = Path(__file__).resolve().parents[2] / 'ros2_ws/src/lunar_incremental_controller_demo/launch/controller_rviz.launch.py'

@pytest.mark.parametrize('period,enabled', [('0.0',False),('0.5',True)])
def test_one_period_configures_navigation_and_execution(monkeypatch,period,enabled):
    spec=importlib.util.spec_from_file_location('rolling_launch',PATH)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:'/test/'+name)
    monkeypatch.setattr(module,'Node',lambda **kw:kw)
    context=LaunchContext()
    context.launch_configurations.update(case='forward',start_rviz='false',observation_radius_m='10.0',
        rolling_replan_period_s=period,rolling_replan_distance_m='1.0')
    nodes=module.compose(context)
    nav=next(n for n in nodes if n.get('name')=='controller_demo_navigation')['parameters'][0]
    controller=next(n for n in nodes if n.get('name')=='controller_demo_executor')['parameters'][0]
    assert nav['rolling_replan_period_s']==float(period)
    assert controller['enable_rolling_updates'] is enabled
    assert controller['command_topic']=='/lunar_demo/controller/cmd_vel'


def test_launch_defaults_enable_both_ends(monkeypatch):
    from launch.actions import DeclareLaunchArgument
    spec = importlib.util.spec_from_file_location('rolling_launch_defaults', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: '/test/' + name)
    monkeypatch.setattr(module, 'Node', lambda **kw: kw)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = module.compose(context)
    nav = next(n for n in nodes if n.get('name') == 'controller_demo_navigation')['parameters'][0]
    controller = next(n for n in nodes if n.get('name') == 'controller_demo_executor')['parameters'][0]
    assert nav['rolling_replan_period_s'] == .5
    assert controller['enable_rolling_updates'] is True
