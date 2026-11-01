"""统一硬边界和全局软偏好参数的实际接线回归。"""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts/navigation/one_cm'))
sys.path.insert(0,str(ROOT/'scripts/navigation/one_cm/ros2/lunar_nav1cm_planner'))
from nav1cm.contracts import CellRect, WorldSnapshot
from nav1cm.coordinator import NavigationCoordinator
from nav1cm import global_worker
from lunar_nav1cm_planner.configuration import load_configuration


@pytest.mark.parametrize('profile',['wheel.yaml','wheel_orin.yaml','tcp'])
def test_selected_profile_uses_total_hard_radius_075(profile,tmp_path,monkeypatch):
    if profile=='tcp':
        from scripts.simulation.p4_tcp_runtime import load_tcp_platform_profile
        path=tmp_path/'tcp.yaml';path.write_text(yaml.safe_dump(load_tcp_platform_profile(ROOT)),encoding='utf-8')
    else:path=ROOT/'config'/profile
    limits,_=load_configuration(path)
    assert limits.global_clearance_radius()==pytest.approx(.75)
    # MPCC真实装配前即计算radius；测试替身只替换求解器构建。
    from nav1cm.local_mpcc import MpccPlanner
    class Ocp:
        def __init__(self):
            self.solver_options=SimpleNamespace();self.constraints=SimpleNamespace();self.cost=SimpleNamespace()
    fake_ca=SimpleNamespace(SX=SimpleNamespace(sym=lambda name,n:np.zeros(n)),sin=np.sin,cos=np.cos,
        vertcat=lambda *v:np.concatenate([np.atleast_1d(x) for x in v]),sumsqr=lambda v:np.sum(v*v))
    monkeypatch.setitem(sys.modules,'casadi',fake_ca)
    monkeypatch.setitem(sys.modules,'acados_template',SimpleNamespace(AcadosModel=SimpleNamespace,
        AcadosOcp=Ocp,AcadosOcpSolver=lambda *a,**k:SimpleNamespace()))
    planner=MpccPlanner(limits,tmp_path)
    assert planner.radius==pytest.approx(.75)
    assert planner.offsets==[0.]
    c=NavigationCoordinator(None,limits=limits,runner=SimpleNamespace())
    assert c.clearance_radius_m==pytest.approx(.75)
    assert c._goal_checker.clearance_radius_m==pytest.approx(.75)
    # 标准跟踪按相同圆支持；数值格半对角安全余量保持原样。
    from nav1cm.follower_map import center_support
    world=WorldSnapshot(1,(CellRect(80,-300,81,300),),(CellRect(-500,-500,500,500),))
    support=center_support(world,0.,0.,body_length_m=limits.body_length_m,
        body_width_m=limits.body_width_m,minimum_clearance_m=limits.minimum_clearance_m)
    assert support.eligible[250,250]


def test_worker_request_preserves_soft_options_and_legacy_defaults(monkeypatch):
    captured=[]
    class Planner:
        def __init__(self,limits,radius,**kwargs):captured.append((radius,kwargs))
    monkeypatch.setattr(global_worker,'AdaptiveGlobalPlanner',Planner)
    limits,_=load_configuration(ROOT/'config/wheel.yaml')
    world=WorldSnapshot(1,(),())
    request=global_worker.make_request(world,(0,0),(1,0),1,limits,
        global_soft_clearance_m=.12,global_clearance_weight=3.)
    # 只核对构造，不依赖结果类型字段。
    class Stop(Exception):pass
    def stop(*args):raise Stop
    monkeypatch.setattr(Planner,'plan',stop,raising=False)
    with pytest.raises(Stop):global_worker.run_request(request)
    assert captured[-1]==(.75,{'soft_clearance_m':.12,'clearance_weight':3.})
    request.pop('global_soft_clearance_m');request.pop('global_clearance_weight')
    with pytest.raises(Stop):global_worker.run_request(request)
    assert captured[-1]==(.75,{'soft_clearance_m':.4,'clearance_weight':.5})


@pytest.mark.parametrize('key',['global_soft_clearance_m','global_clearance_weight'])
@pytest.mark.parametrize('value',[-.1,float('nan'),float('inf')])
def test_coordinator_rejects_invalid_soft_options(key,value):
    with pytest.raises(ValueError,match=key):
        NavigationCoordinator(None,runner=SimpleNamespace(),**{key:value})


def test_coordinator_submits_selected_soft_parameters():
    from nav1cm.contracts import VehicleState
    class Runner:
        busy=False
        def poll(self):return None
        def submit(self,request):self.request=request
    runner=Runner()
    limits,_=load_configuration(ROOT/'config/wheel.yaml')
    c=NavigationCoordinator(None,limits=limits,runner=runner,
        global_soft_clearance_m=.12,global_clearance_weight=3.)
    c.tick(WorldSnapshot(1,(),(CellRect(-500,-500,500,500),)),VehicleState(0,0,0,0,0),(3,0),0)
    assert runner.request['global_soft_clearance_m']==.12
    assert runner.request['global_clearance_weight']==3.
    assert runner.request['clearance_radius_m']==.75


def _launch_graph(monkeypatch,**values):
    import importlib.util
    spec=importlib.util.spec_from_file_location('clearance_launch_test_helpers',
        ROOT/'tests/launch/test_nav1cm_exploration_launch.py')
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    return helper.nodes(helper.graph(monkeypatch,**values))


@pytest.mark.parametrize('options,expected',[
    ({},(.4,.5)),
    ({'global_soft_clearance_m':'.12','global_clearance_weight':'3.'},(.12,3.)),
    ({'global_soft_clearance_m':'0','global_clearance_weight':'0'},(0.,0.))])
def test_launch_passes_soft_options_only_to_navigation(monkeypatch,options,expected):
    graph=_launch_graph(monkeypatch,enable_controller='true',**options)
    planner=graph[1]['parameters'][0]
    assert (planner['global_soft_clearance_m'],planner['global_clearance_weight'])==expected
    assert all('global_soft_clearance_m' not in item['parameters'][0]
        and 'global_clearance_weight' not in item['parameters'][0]
        for i,item in enumerate(graph) if i!=1)


@pytest.mark.parametrize('key',['global_soft_clearance_m','global_clearance_weight'])
@pytest.mark.parametrize('value',['-.1','nan','inf'])
def test_launch_rejects_invalid_soft_options(monkeypatch,key,value):
    with pytest.raises(ValueError,match=key):_launch_graph(monkeypatch,**{key:value})
