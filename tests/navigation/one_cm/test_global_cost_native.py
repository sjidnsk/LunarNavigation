import ctypes,os
from pathlib import Path
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import global_planner as api


def test_native_soft_cost_symbol_matches_python_reference():
    path=os.environ.get('NAV1CM_GEOMETRY_LIB')
    if not path:
        pytest.skip('native geometry library not requested')
    lib=ctypes.CDLL(path)
    assert hasattr(lib,'nav1cm_soft_path_cost'), 'optional native preference integral is missing'
    assert hasattr(api,'_path_cost_python')
    from scripts.navigation.one_cm.nav1cm.global_cost_native import load_cost_function
    function=load_cost_function()
    assert function is not None
    rng=np.random.default_rng(184)
    hard=np.asarray([[-.75,-.75,1.75,1.75],[3.,-2.,5.,3.]])
    paths=[np.asarray([[0.,2.],[10.,2.]]),np.asarray([[1.6,1.6],[2.1,1.6]]),np.asarray([[0.,2.],[0.,2000.]])]
    paths += [rng.uniform(-3,7,(4,2)) for _ in range(30)]
    for points in paths:
        for radius in (0.,.75):
            for soft,weight in ((.4,.5),(.4,2.),(0.,.5),(.4,0.)):
                expected=api._path_cost_python(points,hard,soft,weight,corner_radius=radius)
                actual=api.path_cost(points,hard,soft,weight,corner_radius=radius)
                direct=function(np.ascontiguousarray(points),len(points),hard,len(hard),soft,weight,radius)
                assert np.isfinite(direct)
                assert direct==pytest.approx(expected,rel=1e-11,abs=1e-8)
                assert actual==pytest.approx(expected,rel=1e-11,abs=1e-8)


def test_missing_optional_native_symbol_keeps_python_fallback(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import global_cost_native
    monkeypatch.setattr(global_cost_native,'load_cost_function',lambda:None)
    points=np.asarray([[1.6,1.6],[2.1,1.6]])
    hard=np.asarray([[-.75,-.75,1.75,1.75]])
    assert api.path_cost(points,hard,corner_radius=.75)==api._path_cost_python(points,hard,corner_radius=.75)


@pytest.mark.parametrize('soft,weight',[(-1.,.5),(.4,-1.),(float('nan'),.5),(.4,float('inf'))])
def test_native_wrapper_preserves_invalid_preference_rejection(soft,weight):
    with pytest.raises(ValueError):
        api.path_cost([[0.,0.],[1.,0.]],[],soft,weight)


def test_native_axis_split_work_cap_returns_nan_and_wrapper_falls_back(monkeypatch):
    from scripts.navigation.one_cm.nav1cm.global_cost_native import load_cost_function
    function=load_cost_function()
    if function is None:
        pytest.skip('optional native preference integral not available')
    # 所有轴向子段都有恒距遮盖，旧逻辑完全绕过采样work计数，却逐段扫描全部盒。
    x=np.linspace(-1.,9.,1500)
    hard=np.ascontiguousarray(np.column_stack((x-.75,np.full(len(x),-1.75),x+2.75,np.full(len(x),.75))))
    points=np.asarray([[0.,.85],[10.,.85]])
    assert np.isnan(function(points,len(points),hard,len(hard),.4,.5,.75))
    class BudgetExpired(Exception):
        pass
    calls=[]
    def reference(points,boxes,soft,weight,check,**kwargs):
        calls.append('fallback')
        check()
    monkeypatch.setattr(api,'_path_cost_python',reference)
    def check():
        if calls:
            raise BudgetExpired
    with pytest.raises(BudgetExpired):
        api.path_cost(points,hard,check=check,corner_radius=.75)
    assert calls==['fallback']
