"""私有推理命名空间不能复用只接受training前缀的旧启动器。"""
import importlib.util
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]


def load(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'scripts/drl'))
    path=ROOT/'scripts/drl/verify_nav1cm_closed_loop.py'
    spec=importlib.util.spec_from_file_location('closure_launch_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    assert hasattr(module,'inference_ros_arguments')
    return module.inference_ros_arguments


def test_private_inference_arguments_preserve_ros_types(monkeypatch):
    args=load(monkeypatch)('/lunar_inference/closure',{'map_topic':'/lunar_inference/closure/grid','use_sim_time':False,'goal_tolerance_m':.1})
    assert 'use_sim_time:=false' in args
    assert 'goal_tolerance_m:=0.1' in args
    assert '__ns:=/lunar_inference/closure' in args


@pytest.mark.parametrize('namespace',['/Car/T4','/lunar_training/test','/lunar_inference/../bad'])
def test_nonprivate_or_traversal_namespace_is_rejected(monkeypatch,namespace):
    with pytest.raises(ValueError):load(monkeypatch)(namespace,{})


def test_production_topic_is_rejected(monkeypatch):
    with pytest.raises(ValueError):load(monkeypatch)('/lunar_inference/closure',{'command_topic':'/Car/T5/Car_Cmd_Vel'})


def test_event_evidence_is_canonical_json(monkeypatch):
    import json
    canonical_event=load(monkeypatch).__globals__['canonical_event']
    class ForeignFloat(float):pass
    event=canonical_event('goal',serial=1,goal=[ForeignFloat(.1),ForeignFloat(1.9),ForeignFloat(3.9)])
    assert all(type(v) is float for v in event['goal'])
    assert event==json.loads(json.dumps(event))


def test_inference_hook_registers_existing_acados_codec_before_installing_guard():
    source=(ROOT/'scripts/drl/inference_sitecustomize.py').read_text(encoding='utf-8')
    calls=[]
    import types,sys
    codec=types.SimpleNamespace(register=lambda:calls.append('codec'))
    guard=types.SimpleNamespace(install=lambda path:calls.append('guard'))
    from unittest.mock import patch
    with patch.dict(sys.modules,{'future_fstrings':codec,'inference_only_guard':guard}), patch.dict(__import__('os').environ,{'INFERENCE_ONLY_GUARD_DIR':'test'}):
        exec(compile(source,'sitecustomize.py','exec'),{})
    assert calls==['codec','guard']


def test_motion_override_reaches_only_planner_and_standard_follower(monkeypatch):
    helper=load(monkeypatch).__globals__['apply_motion_model']
    specs=[('map','node',{'x':1}),('lunar_nav1cm_planner','node',{}),('lunar_pure_wheeled_controller','node',{})]
    changed=helper(specs,'differential_drive')
    assert changed[0][2]=={'x':1}
    assert changed[1][2]['motion_model']==changed[2][2]['motion_model']=='differential_drive'
    assert specs[1][2]=={} and specs[2][2]=={}
    with pytest.raises(ValueError):helper(specs,'typo')


def test_execution_evidence_requires_matching_actual_acceptance(monkeypatch):
    helper=load(monkeypatch).__globals__['execution_evidence']
    ref=dict(kind='reference',session_id='a'*32,revision=1,path_digest='d'*64,intent=3,stamp_ns=10)
    status=dict(ref,kind='status',accepted=True,state=4,stamp_ns=11)
    assert helper([status,ref])['accepted_sessions']==['a'*32]
    assert not helper([ref,dict(status,accepted=False)])['accepted_sessions']
    assert not helper([ref,dict(status,path_digest='e'*64)])['accepted_sessions']
    assert not helper([ref,dict(status,stamp_ns=9)])['accepted_sessions']


def test_execution_finish_requires_fresh_terminal_status_before_result(monkeypatch):
    helper=load(monkeypatch).__globals__['execution_evidence']
    ref=dict(kind='reference',session_id='a'*32,revision=1,path_digest='d'*64,intent=3,stamp_ns=10,wall=1.)
    status=dict(ref,kind='status',accepted=True,state=5,stamp_ns=11,wall=2.)
    finish=dict(kind='finish',outcome=0,session_id='a'*32,wall=2.3)
    assert helper([ref,status],[finish])['terminal_completed_sessions']==['a'*32]
    for bad in (dict(status,wall=3.),dict(status,wall=.5),dict(status,state=3),dict(status,accepted=False)):
        assert not helper([ref,bad],[finish])['terminal_completed_sessions']


def test_extended_declaration_changes_only_explicit_duration_bound(monkeypatch,tmp_path):
    """原短窗保持90秒上限；闭环显式允许长窗，声明完整冻结预算。"""
    import json,sys,types
    base=load(monkeypatch).__globals__['base']
    (tmp_path/'runtime.py').write_text('# isolated declaration unit fixture',encoding='utf-8')
    for name,body in [('actor.pt','unit identity'),('config.json','{}'),('wheel.yaml','unit profile')]:
        (tmp_path/name).write_text(body,encoding='utf-8')
    parent=types.ModuleType('lunar_drl_exploration');parent.__path__=[]
    runtime=types.ModuleType('lunar_drl_exploration.runtime');runtime.__file__=str(tmp_path/'runtime.py')
    runtime.ActorPolicy=types.SimpleNamespace(load=lambda p:types.SimpleNamespace(actor=types.SimpleNamespace(config=object())))
    node=types.ModuleType('lunar_drl_exploration.inference_node');node.read_policy_settings=lambda *a:None
    for name,module in [('lunar_drl_exploration',parent),('lunar_drl_exploration.runtime',runtime),('lunar_drl_exploration.inference_node',node)]:
        monkeypatch.setitem(sys.modules,name,module)
    args=types.SimpleNamespace(actor=tmp_path/'actor.pt',config=tmp_path/'config.json',platform=tmp_path/'wheel.yaml',duration=600.,backend='nav1cm')
    with pytest.raises(ValueError):base.declaration(args)
    record=base.declaration(args,maximum_duration_s=900.)
    assert record['duration_s']==600. and record['training'] is False
    assert record['actor_sha256']==base.digest(args.actor)
    for duration in (0.,-1.,900.01,float('inf'),float('nan')):
        args.duration=duration
        with pytest.raises(ValueError):base.declaration(args,maximum_duration_s=900.)

def test_position_target_override_belongs_only_to_navigation_owner(monkeypatch):
    globals_=load(monkeypatch).__globals__
    apply=globals_['apply_position_target']
    specs=[('lunar_nav1cm_planner','node',dict(goal_tolerance_m=.1)),('lunar_pure_wheeled_controller','node',dict(goal_position_tolerance_m=.1))]
    result=apply(specs,.02)
    assert result[0][2]==dict(goal_tolerance_m=.1,goal_position_target_m=.02)
    assert result[1][2]==specs[1][2] and 'goal_position_target_m' not in specs[0][2]
    for bad in (0.,-.01,.11,float('nan'),float('inf')):
        with pytest.raises(ValueError):apply(specs,bad)


def test_position_quality_rejects_outer_band_success_and_wrong_owner_metadata(monkeypatch):
    check=load(monkeypatch).__globals__['position_target_errors']
    event=dict(kind='finish',outcome=0,session_id='a',position_error_m=.019)
    row=dict(kind='reference',session_id='a',position_tolerance_m=.02)
    assert check([event],[row],.02)==[]
    assert check([dict(event,position_error_m=.08)],[row],.02)
    assert check([event],[dict(row,position_tolerance_m=.1)],.02)
    assert check([event],[],.02)
    assert check([event],[row],None)==[]
