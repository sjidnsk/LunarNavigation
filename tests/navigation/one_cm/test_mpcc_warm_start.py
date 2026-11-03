"""warm只提供RTI初值；相位沿既有几何投影消费，地图约束每次重建。"""
from types import SimpleNamespace
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import local_mpcc as module
from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits, VehicleState, CellState
from scripts.navigation.one_cm.nav1cm.seeds import rollout


def planner(mode='forward'):
    p = object.__new__(module.MpccPlanner)
    p.N=4; p.dt=.2; p.limits=WheelLimits(); p.offsets=(0.,); p.radius=.01
    initial=np.array([0.,0.,3.12 if mode=='spin' else 0.,-.1 if mode=='reverse' else .1, .1 if mode=='spin' else 0.,0.])
    if mode=='spin': initial[3]=0.
    controls=np.array([[0., .01 if mode=='spin' else 0., .01*(k+1)] for k in range(4)])
    states=rollout(initial,controls,p.dt)
    key=(7,mode,None)
    p._warm=(key,states,controls,0.)
    return p,key,states,controls


@pytest.mark.parametrize('mode',['forward','reverse','spin'])
def test_warm_consumes_measured_phase_and_pads_without_acceleration(mode):
    p,key,old,controls=planner(mode)
    initial=old[2].copy(); initial[5]=.15
    if mode=='spin': initial[2]-=2*np.pi
    states, shifted=p._warm_seed(initial,key)
    assert np.allclose(shifted[:2],controls[2:])
    assert np.array_equal(shifted[2:],np.zeros((2,3)))
    assert np.array_equal(states[0],initial)
    assert np.allclose(states,rollout(initial,shifted,p.dt))
    # 同一来源只向前消费，即使反馈稍回退也不倒放加速段。
    _,again=p._warm_seed(old[1],key)
    assert np.allclose(again,shifted)


def test_stationary_spin_preparation_does_not_consume_wall_time():
    p,key,old,controls=planner('spin')
    for _ in range(3):
        _, actual=p._warm_seed(old[0],key)
        assert np.allclose(actual,controls,rtol=0.,atol=1e-14)
        assert p._warm[3] == 0.


@pytest.mark.parametrize('key',[(8,'forward',None),(7,'reverse',None),(7,'forward',1.)])
def test_warm_identity_change_resets(key):
    p,_,old,_=planner()
    assert p._warm_seed(old[0],key) is None


def test_warm_tail_is_bounded_at_end():
    p,key,old,_=planner()
    states,controls=p._warm_seed(old[-1],key)
    assert np.array_equal(controls,np.zeros((4,3)))
    assert np.isfinite(states).all()


def test_cross_map_warm_recomputes_parameters_and_candidate_field(monkeypatch):
    p,key,old,controls=planner()
    class Solver:
        def reset(self): self.values={}
        def set(self,k,name,value): self.values[k,name]=np.array(value)
        def solve(self): return 0
        def get(self,k,name): return self.values[k,name]
        def get_stats(self,name): return 0
    p._solvers={'forward':Solver()}
    fields=[]
    def build(grid):
        field=SimpleNamespace(sample=lambda x,y:(float(grid.epoch),0.,0.))
        fields.append(field)
        return field
    monkeypatch.setattr(module,'build_distance_field',build)
    checked=[]
    p._candidate_reason=lambda xs,us,mode,field,end: checked.append(field) or 'CANDIDATE'
    state=VehicleState(*old[1,:5])
    for epoch,origin in [(2,(0.,0.)),(3,(-1.,0.))]:
        grid=SimpleNamespace(epoch=epoch,origin_xy=origin,digest=lambda: str(epoch),cell_at_world=lambda *xy:CellState.FREE)
        ref=SimpleNamespace(map_epoch=epoch,route_version=7,terminal_yaw=None,points=np.array([[0.,0.],[3.,0.]]),s=np.array([0.,3.]),progress_s=.02)
        result=p.plan(state,grid,ref)
        assert result.attempts[0]['seed']=='warm'
        assert checked[-1] is fields[-1]
        assert p._solvers['forward'].values[0,'p'][12]==pytest.approx(epoch-p.radius)
        assert result.candidate.map_epoch==epoch


def test_fractional_phase_resamples_piecewise_controls():
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    p,key,old,controls=planner()
    observed=step(old[1], controls[1], .1)
    _,shifted=p._warm_seed(observed,key)
    assert np.allclose(shifted[:2],(controls[1:3]+controls[2:4])/2)
    assert np.allclose(shifted[2],controls[3]/2)
    assert np.array_equal(shifted[-1],np.zeros(3))


def test_multiturn_projection_retains_ordered_floor():
    p,key,_,_=planner('spin')
    p.N=40
    initial=np.array([0.,0.,0.,0.,1.,0.])
    controls=np.zeros((40,3));controls[:,2]=np.arange(40)*.001
    states=rollout(initial,controls,p.dt)
    p._warm=(key,states,controls,6.2)
    observed=states[35].copy();observed[2]-=2*np.pi
    _,shifted=p._warm_seed(observed,key)
    assert p._warm[3]==pytest.approx(7., abs=1e-5)
    assert shifted[0,2]==pytest.approx(controls[35,2])
    assert np.array_equal(shifted[5:],np.zeros((35,3)))


@pytest.mark.parametrize('phase',[.05,.1,.15,.65,.75])
@pytest.mark.parametrize('sign',[1.,-1.])
def test_fractional_acceleration_integrates_suffix_without_replay(monkeypatch,phase,sign):
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    p,key,_,_=planner('forward' if sign>0 else 'reverse')
    initial=np.array([0.,0.,0.,sign*.4,sign*.02,0.])
    controls=np.array([[sign*.5,sign*.2,.2],[0.,0.,.1],[-sign*.5,-sign*.2,.05],[0.,0.,.01]])
    old=rollout(initial,controls,p.dt)
    p._warm=(key,old,controls,0.)
    index=min(3,int(phase/p.dt))
    observed=step(old[index],controls[index],phase-index*p.dt)
    monkeypatch.setattr(module,'_project',lambda *args:phase)
    states,shifted=p._warm_seed(observed,key)
    for k in range(p.N):
        left=phase+k*p.dt;right=left+p.dt
        integral=sum((max(0.,min(right,(j+1)*p.dt)-max(left,j*p.dt))*u for j,u in enumerate(controls)), np.zeros(3))
        assert np.allclose(shifted[k]*p.dt,integral,atol=1e-12)
        assert np.allclose(states[k+1,[3,4,5]]-states[k,[3,4,5]],integral,atol=1e-12)
    assert np.max(np.abs(states[:,3]))<=.5+1e-12
    assert np.max(np.abs(states[:,4]))<=.06+1e-12
