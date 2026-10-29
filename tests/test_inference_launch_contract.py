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
