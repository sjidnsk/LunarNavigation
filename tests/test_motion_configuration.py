"""配置与组合入口使用同一运动能力；不依赖ROS运行时。"""
import importlib.util
from pathlib import Path
import sys
import types
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[1]
CORE=ROOT/'scripts/navigation/one_cm'
PACKAGE=CORE/'ros2/lunar_nav1cm_planner'
CONTROLLER=ROOT/'ros2_ws/src/lunar_pure_wheeled_controller'


def configuration(monkeypatch):
    monkeypatch.syspath_prepend(str(CORE));monkeypatch.syspath_prepend(str(PACKAGE))
    from lunar_nav1cm_planner.configuration import load_configuration
    return load_configuration


@pytest.mark.parametrize('profile',['wheel.yaml','wheel_orin.yaml'])
def test_current_wheel_selects_diffdrive_and_explicit_override_keeps_type(monkeypatch,profile):
    load=configuration(monkeypatch)
    limits,_=load(ROOT/'config'/profile)
    assert limits.motion_model=='differential_drive'
    changed,_=load(ROOT/'config/wheel.yaml',{'motion_model':'curvature_limited','angular_speed_rps':.7})
    assert changed.motion_model=='curvature_limited' and changed.angular_speed_rps==.7


def test_old_profile_keeps_legacy_and_unknown_model_is_rejected(monkeypatch,tmp_path):
    load=configuration(monkeypatch)
    data=yaml.safe_load((ROOT/'config/wheel.yaml').read_text(encoding='utf-8'))
    data['capability'].pop('motion_model',None)
    path=tmp_path/'old.yaml';path.write_text(yaml.safe_dump(data),encoding='utf-8')
    assert load(path)[0].motion_model=='curvature_limited'
    data['capability']['motion_model']='typo'
    path.write_text(yaml.safe_dump(data),encoding='utf-8')
    with pytest.raises(ValueError):load(path)


def resolver(monkeypatch):
    spec=importlib.util.spec_from_file_location('motion_configuration_test',CONTROLLER/'python/lunar_pure_wheeled_controller/motion_configuration.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.resolve_motion_model


def test_standard_node_resolves_file_and_override_but_legacy_keeps_own_contract(monkeypatch):
    resolve=resolver(monkeypatch)
    capability={'motion_model':'differential_drive'}
    assert resolve(capability,'curvature_limited',standard_path=True)=='differential_drive'
    assert resolve(capability,'curvature_limited',standard_path=True,explicit=True)=='curvature_limited'
    assert resolve(capability,'curvature_limited',standard_path=False)=='curvature_limited'
    with pytest.raises(ValueError):resolve({},'differential_drive',standard_path=False,explicit=True)
    with pytest.raises(ValueError):resolve({'motion_model':'typo'},'curvature_limited',standard_path=True)


def launch(monkeypatch):
    configuration(monkeypatch)
    def node(**kwargs):return kwargs
    class LC:
        def __init__(self,name):self.name=name
        def perform(self,context):return context[self.name]
    modules={
        'ament_index_python.packages':dict(get_package_share_directory=lambda name:str(PACKAGE)),
        'launch':dict(LaunchDescription=list),
        'launch.actions':dict(DeclareLaunchArgument=object,OpaqueFunction=object),
        'launch.substitutions':dict(EnvironmentVariable=object,LaunchConfiguration=LC),
        'launch_ros.actions':dict(Node=node),
    }
    for name,attrs in modules.items():
        module=types.ModuleType(name);module.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,module)
    spec=importlib.util.spec_from_file_location('motion_launch',CONTROLLER/'launch/nav1cm_wheel.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_launch_resolves_same_model_for_both_consumers(monkeypatch):
    module=launch(monkeypatch)
    context={**module.DEFAULTS,'wheel_config':str(ROOT/'config/wheel.yaml'),'build_dir':'/outside/build','start_controller':'true'}
    for override,expected in [('', 'differential_drive'),('curvature_limited','curvature_limited')]:
        context['motion_model']=override
        planner,follower=[n['parameters'][-1] for n in module.compose(context)]
        assert planner['motion_model']==follower['motion_model']==expected
    context['motion_model']='typo'
    with pytest.raises(ValueError):module.compose(context)
