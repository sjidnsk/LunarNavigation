"""禁止训练入口，不加载真实训练器或优化器。"""
import importlib.util
from pathlib import Path
import sys
import types
import pytest

ROOT=Path(__file__).resolve().parents[1]


def load():
    path=ROOT/'scripts/drl/inference_only_guard.py'
    assert path.is_file(), 'inference guard missing'
    spec=importlib.util.spec_from_file_location('inference_guard_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('name',['training','sac','replay','collector','ros_env'])
def test_training_module_is_rejected_before_import(tmp_path,name):
    module=load();guard=module.InferenceGuard(tmp_path)
    with pytest.raises(RuntimeError,match='inference-only'):
        guard.find_spec('lunar_drl_exploration.'+name,None)
    assert 'lunar_drl_exploration.'+name not in sys.modules
    assert any('blocked_import' in p.read_text() for p in tmp_path.glob('*.jsonl'))


def test_batch_and_normal_product_imports_are_not_blocked(tmp_path):
    guard=load().InferenceGuard(tmp_path)
    assert guard.find_spec('lunar_drl_exploration.batch',None) is None
    assert guard.find_spec('lunar_drl_exploration.runtime',None) is None


def test_optimizer_constructor_is_blocked_but_actor_is_untouched(tmp_path):
    module=load();guard=module.InferenceGuard(tmp_path)
    class Optimizer: pass
    class Actor: pass
    fake=types.SimpleNamespace(Optimizer=Optimizer,Actor=Actor)
    guard.patch_module('torch.optim.optimizer',fake)
    with pytest.raises(RuntimeError,match='inference-only'):Optimizer()
    assert isinstance(Actor(),Actor)
