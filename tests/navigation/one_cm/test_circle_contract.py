"""圆包络唯一公式与MPCC装配维数；SDK替身不证明真实求解器通过。"""
from dataclasses import replace
from types import SimpleNamespace as NS
import sys
import math
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import contracts
from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits,LocalReference
from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner


def assembled(monkeypatch,tmp_path,margin,model='differential_drive'):
    built=[]
    class Ocp:
        def __init__(self):
            self.solver_options=NS();self.constraints=NS();self.cost=NS()
    def solver(ocp,**kwargs):
        built.append(ocp);return NS()
    fake_ca=NS(SX=NS(sym=lambda name,n:np.zeros(n)),sin=np.sin,cos=np.cos,
               vertcat=lambda *v:np.concatenate([np.atleast_1d(x) for x in v]),
               sumsqr=lambda v:np.sum(v*v))
    monkeypatch.setitem(sys.modules,'casadi',fake_ca)
    monkeypatch.setitem(sys.modules,'acados_template',NS(AcadosModel=NS,AcadosOcp=Ocp,AcadosOcpSolver=solver))
    limits=replace(WheelLimits(),minimum_clearance_m=margin,motion_model=model)
    return MpccPlanner(limits,tmp_path,n=2),built


@pytest.mark.parametrize('margin',[.2,.4])
def test_shared_circumscribed_radius_adds_clearance_once(margin):
    radius=contracts.circumscribed_radius(1.182,.818,margin)
    assert radius==pytest.approx(math.hypot(1.182,.818)/2+margin)
    assert replace(WheelLimits(),minimum_clearance_m=margin).global_clearance_radius()==radius


@pytest.mark.parametrize('margin,circles,parameters',[(.2,1,14),(.4,1,14),(0.,3,22)])
@pytest.mark.parametrize('model',['differential_drive','curvature_limited'])
def test_generated_layout_has_only_distinct_circle_constraints(monkeypatch,tmp_path,margin,circles,parameters,model):
    p,built=assembled(monkeypatch,tmp_path,margin,model)
    assert len(p.offsets)==circles
    for mode,ocp in zip(('forward','reverse','spin'),built):
        constraints=circles+1+(2 if mode!='spin' and model=='curvature_limited' else 0)
        assert len(ocp.model.p)==parameters
        assert len(ocp.parameter_values)==parameters
        assert len(ocp.model.con_h_expr)==constraints
        assert len(ocp.constraints.lh)==constraints
        assert len(ocp.constraints.lh_e)==constraints
    assert p.model_fingerprint not in ('',None)


def test_positive_margin_parameters_query_one_circle_per_state(monkeypatch,tmp_path):
    p,_=assembled(monkeypatch,tmp_path,.2)
    calls=[]
    field=NS(sample=lambda x,y:(calls.append((x,y)) or (2.,1.,0.)))
    ref=LocalReference(1,1,[[0,0],[1,0]],[0.,1.])
    states=np.zeros((3,6));controls=np.zeros((2,3))
    params=p._parameters(states,controls,ref,field,0.,1.)
    assert len(calls)==len(states)
    assert all(len(row)==14 and row[-1]==1. for row in params)
    assert params[0][12]==pytest.approx(2.-p.radius-.003)



def test_three_consumers_use_the_same_circle_formula():
    from scripts.navigation.one_cm.nav1cm import follower_map,local_mpcc
    assert follower_map.circumscribed_radius is contracts.circumscribed_radius
    assert local_mpcc.circumscribed_radius is contracts.circumscribed_radius


@pytest.mark.parametrize('values',[(0.,1.,.2),(1.,-1.,.2),(1.,1.,-.1),(float('nan'),1.,0.),(1.,1.,float('inf'))])
def test_circle_formula_rejects_invalid_dimensions_or_margin(values):
    with pytest.raises(ValueError):contracts.circumscribed_radius(*values)


@pytest.mark.parametrize('distance_offset',[-.01,0.,.01])
def test_single_circle_keeps_old_feasibility_and_linearization(monkeypatch,tmp_path,distance_offset):
    p,_=assembled(monkeypatch,tmp_path,.2)
    field=NS(sample=lambda x,y:(p.radius+distance_offset,.6,.8))
    states=np.zeros((3,6));controls=np.zeros((2,3))
    ref=LocalReference(1,1,[[0,0],[1,0]],[0.,1.])
    single=p._parameters(states,controls,ref,field,0.,1.)
    verdict=p._candidate_reason(states,controls,'forward',field,1.)
    # 与历史三个重合圆逐项比较，可行集合和仿射值不改变。
    p.offsets=np.zeros(3)
    repeated=p._parameters(states,controls,ref,field,0.,1.)
    assert p._candidate_reason(states,controls,'forward',field,1.)==verdict
    for one,three in zip(single,repeated):
        for j in range(3):np.testing.assert_array_equal(one[9:13],three[9+4*j:13+4*j])


def test_new_circle_layout_does_not_reuse_old_generated_model(monkeypatch,tmp_path):
    import hashlib
    p,_=assembled(monkeypatch,tmp_path,.2)
    old=hashlib.sha256(f'pose-terminal-clearance-v3:{p.N}:{p.dt.hex()}:{p.limits.digest()}'.encode()).hexdigest()[:12]
    assert p.model_fingerprint!=old
    assert p.build_dir.name==p.model_fingerprint
