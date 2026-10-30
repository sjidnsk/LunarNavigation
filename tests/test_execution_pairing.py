"""Path与侧带参考的乱序配对，测试不加载ROS。"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
import pytest
P=Path(__file__).resolve().parents[1]/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/execution_pairing.py'
spec=importlib.util.spec_from_file_location('pairing_contract',P)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def ref(session='a',revision=1,stamp=10,digest='d'):
    return S(session_id=session,revision=revision,stamp_ns=stamp,path_digest=digest)


@pytest.mark.parametrize('first',['path','metadata'])
def test_pairing_both_orders_never_accept_unpaired(first):
    x=module.ExecutionInbox();r=ref();parsed=('immutable',)
    if first=='path':x.add_path(10,'d',parsed,1.)
    else:x.add_reference(r,1.)
    assert x.pop_ready(1.) is None
    if first=='path':x.add_reference(r,1.01)
    else:x.add_path(10,'d',parsed,1.01)
    assert x.pop_ready(1.02)==(r,parsed,1.)
    assert x.pop_ready(1.03) is None


def test_identity_digest_retirement_and_bounded_cache():
    x=module.ExecutionInbox(capacity=2)
    x.add_path(10,'wrong',('wrong',),1.);x.add_reference(ref(),1.)
    assert x.pop_ready(1.) is None
    x.add_path(10,'d',('good',),1.);assert x.pop_ready(1.)[1]==('good',)
    assert not x.add_reference(ref(revision=1,stamp=11),1.)
    n=ref(session='b',stamp=20);x.add_reference(n,1.);x.add_path(20,'d',(),1.)
    assert x.pop_ready(1.)[0]==n
    assert not x.add_reference(ref(revision=99,stamp=30),1.)
    for i in range(10):x.add_path(100+i,'digest',(),1.)
    assert len(x.paths)==2


def test_stale_pair_does_not_execute():
    x=module.ExecutionInbox();x.add_path(10,'d',(),1.);x.add_reference(ref(),1.2)
    assert x.pop_ready(1.4) is None


@pytest.mark.parametrize('kwargs',[{'capacity':0},{'capacity':-1},{'max_age_s':0.},{'max_age_s':float('nan')}])
def test_invalid_pairing_bounds_are_rejected(kwargs):
    with pytest.raises(ValueError):module.ExecutionInbox(**kwargs)
