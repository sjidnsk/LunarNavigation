"""逐位相同几何边共享证明的旧算法对照与完整制动回归。"""
import math
import numpy as np
import pytest
from lunar_pure_wheeled_controller import mppi_map as maps
from lunar_pure_wheeled_controller.mppi_map import GridMap, _segment_box_distance2
from lunar_pure_wheeled_controller.mppi import MppiExecutor, MppiPolicy
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState


# 优化前算法快照；作为独立对照，不能随生产实现一起改写。
def original_valid_segments(self, a, b, margin, allow_assumed=True):
    a,b=np.broadcast_arrays(np.asarray(a,dtype=float),np.asarray(b,dtype=float))
    shape=a.shape[:-1];aa=a.reshape(-1,2);bb=b.reshape(-1,2)
    radius=np.broadcast_to(np.asarray(margin,dtype=float),shape).reshape(-1)
    allowed=np.broadcast_to(np.asarray(allow_assumed,dtype=bool),shape).reshape(-1)
    if np.any(radius<0) or np.any(~np.isfinite(radius)):raise ValueError('invalid margin')
    valid=np.all(np.isfinite(aa)&np.isfinite(bb),axis=1)
    if not np.all(valid):return np.zeros(shape,dtype=bool)
    count=max(1,int(np.ceil(np.max(np.linalg.norm(bb-aa,axis=1),initial=0.)/(self.resolution*.5))))
    if count>4096:raise ValueError('segment certificate exceeds bounded local range')
    reach=int(np.ceil(float(radius.max())/self.resolution))+1
    oy,ox=np.mgrid[-reach:reach+1,-reach:reach+1]
    offsets=np.stack((ox.ravel(),oy.ravel()),axis=-1);h,w=self.states.shape
    first_free=np.full(len(aa),np.inf);last_assumed=np.full(len(aa),-np.inf)
    check_prefix=bool(np.any(self.assumed))
    for j in range(count):
        start=aa+(bb-aa)*(j/count);end=aa+(bb-aa)*((j+1)/count)
        cells=self._indices((start+end)*.5)[:,None,:]+offsets[None]
        ix,iy=cells[...,0],cells[...,1]
        inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
        x=np.clip(ix,0,w-1);y=np.clip(iy,0,h-1)
        free=inside&((self.states[y,x]==1)|self.assumed[y,x]|self.eligible[y,x])
        row,col=np.nonzero(valid[:,None]&~free)
        if len(row):
            lo=cells[row,col]*self.resolution+self.origin
            dist=_segment_box_distance2(start[row],end[row],lo,lo+self.resolution)
            hit=np.where(radius[row]==0,dist<=1e-20,dist+1e-14<radius[row]**2)
            valid[row[hit]]=False
        if check_prefix:
            row,col=np.nonzero(valid[:,None]&inside)
            lo=cells[row,col]*self.resolution+self.origin;hi=lo+self.resolution
            d=bb[row]-aa[row];nonzero=abs(d)>1e-15;safe=np.where(nonzero,d,1.)
            u=(lo-aa[row])/safe;v=(hi-aa[row])/safe
            enter=np.maximum(0.,np.max(np.where(nonzero,np.minimum(u,v),-np.inf),axis=-1))
            leave=np.minimum(1.,np.min(np.where(nonzero,np.maximum(u,v),np.inf),axis=-1))
            touch=(enter<=leave)&np.all(nonzero|((aa[row]>=lo)&(aa[row]<=hi)),axis=-1)
            free_contact=touch&(self.states[y[row,col],x[row,col]]==1)
            assumed_contact=touch&self.assumed[y[row,col],x[row,col]]
            np.minimum.at(first_free,row[free_contact],enter[free_contact])
            np.maximum.at(last_assumed,row[assumed_contact],leave[assumed_contact])
            valid[row[assumed_contact & ~allowed[row]]]=False
    # Canonical order of grid visitation is irrelevant: compare actual
    # clipped contact intervals, including closed boundary/corner contacts.
    valid &= last_assumed<=first_free+1e-12
    return valid.reshape(shape)


def test_identical_edges_share_box_proof_once(monkeypatch):
    grid=GridMap(np.zeros((6,6),np.uint8),.1,(0.,0.))
    a=np.tile([.25,.25],(200,1));b=np.tile([.26,.25],(200,1))
    counts=[];original=maps._segment_box_distance2
    def count(*args):
        counts.append(len(args[0]))
        return original(*args)
    monkeypatch.setattr(maps,'_segment_box_distance2',count)
    monkeypatch.setitem(original_valid_segments.__globals__,'_segment_box_distance2',count)
    expected=original_valid_segments(grid,a,b,0.)
    baseline=sum(counts);counts.clear()
    actual=grid.valid_segments(a,b,0.)
    np.testing.assert_array_equal(actual,expected)
    assert baseline>0
    assert sum(counts)==baseline//200


def test_mixed_random_maps_match_original_elementwise():
    rng=np.random.default_rng(312)
    for _ in range(120):
        states=rng.integers(0,3,(30,30),dtype=np.uint8)
        assumed=(states==0)&(rng.random((30,30))<.12)
        eligible=(states!=2)&(rng.random((30,30))<.1)
        grid=GridMap(states,.1,(-1.5,-1.5),assumed=assumed,eligible=eligible)
        a=rng.uniform(-1.8,1.8,(40,2));b=a+rng.uniform(-.15,.15,(40,2))
        b[:5]=a[:5]
        radius=rng.choice([0.,.01,.1],40);allowed=rng.choice([True,False],40)
        repeat=rng.integers(0,40,200)
        args=(a[repeat],b[repeat],radius[repeat],allowed[repeat])
        np.testing.assert_array_equal(grid.valid_segments(*args),original_valid_segments(grid,*args))


def test_direction_boundary_zero_length_and_permissions_match_original():
    states=np.ones((7,7),np.uint8);states[2,2]=0;states[3,3]=2
    assumed=np.zeros_like(states,dtype=bool);assumed[2,2]=True
    grid=GridMap(states,.1,(0.,0.),assumed=assumed)
    a=np.array([[.25,.25],[.45,.25],[.3,.3],[.2,.2],[.3,.2],[.25,.25]])
    b=np.array([[.45,.25],[.25,.25],[.3,.3],[.3,.3],[.3,.3],[.45,.25]])
    aa=np.tile(a,(4,1));bb=np.tile(b,(4,1))
    radius=np.repeat([0.,.01,0.,.01],len(a))
    allowed=np.repeat([True,True,False,False],len(a))
    expected=original_valid_segments(grid,aa,bb,radius,allowed)
    np.testing.assert_array_equal(grid.valid_segments(aa,bb,radius,allowed),expected)
    # 相同几何的授权不同，且反向进入已批准起始区不等价于离开。
    assert expected[0] and not expected[1] and not expected[12]
    np.testing.assert_array_equal(grid.valid_segments(aa.reshape(4,6,2),bb.reshape(4,6,2),radius.reshape(4,6),allowed.reshape(4,6)),expected.reshape(4,6))


@pytest.mark.parametrize('age',[0.,2.,4.,4.5])
@pytest.mark.parametrize('blocked',[False,True])
@pytest.mark.parametrize('motion_model',['curvature_limited','differential_drive'])
def test_complete_braking_delays_and_nonzero_history_match_original(monkeypatch,age,blocked,motion_model):
    policy=TrackingPolicy(max_linear_mps=.05,max_angular_radps=.1,motion_model=motion_model)
    executor=MppiExecutor(policy,MppiPolicy(time_steps=140,state_age_budget_s=4.5))
    states=np.ones((121,121),np.uint8)
    if blocked:states[:,64]=2
    executor.set_map(GridMap(states,.01,(-.6,-.6)))
    executor._state_age_s=age;executor._model_time=100.
    executor._command_history=[(95.,np.array([.025,.05])),(97.,np.array([-.02,-.05])),(99.,np.array([0.,0.]))]
    state=TrackingState(0.,0.,0.,.025,.05)
    first=np.array([[.025,0.],[0.,.05],[0.,0.]])
    rollout=executor._rollout;observed=[]
    def record(*args,**kwargs):
        poses,actual=rollout(*args,**kwargs)
        observed.append((kwargs['delay_ticks'].copy(),kwargs['history_delay_ticks'].copy(),actual.copy()))
        return poses,actual
    monkeypatch.setattr(executor,'_rollout',record)
    optimized=executor._brake_safe(state,first,.05)
    monkeypatch.setattr(GridMap,'valid_segments',original_valid_segments)
    reference=executor._brake_safe(state,first,.05)
    np.testing.assert_array_equal(optimized,reference)
    latency=math.ceil(policy.control_response_time_s/.05)
    wait=math.ceil((age+executor.config.compute_budget_s)/.05)
    scenarios=[(l,q) for l in range(latency+1) for q in range(wait+1)]
    holds=np.repeat([l+q for l,q in scenarios],len(first))
    transport=np.repeat([l for l,q in scenarios],len(first))
    stop=math.ceil(max(policy.max_linear_mps,policy.max_reverse_mps,abs(state.linear_mps))/policy.max_linear_decel_mps2/.05)+math.ceil(max(policy.max_angular_radps,abs(state.angular_radps))/policy.max_angular_accel_radps2/.05)+2
    for observed_holds,observed_transport,actual in observed:
        np.testing.assert_array_equal(observed_holds,holds)
        np.testing.assert_array_equal(observed_transport,transport)
        assert actual.shape==(len(scenarios)*len(first),2*(latency+wait)+1+stop,2)
        assert np.all(abs(actual[:,-1,0])<=policy.stopped_linear_mps)
        assert np.all(abs(actual[:,-1,1])<=policy.stopped_angular_radps)
    np.testing.assert_array_equal(observed[0][2],observed[1][2])


def test_signed_zero_invalid_inputs_and_bounds_preserve_original():
    grid=GridMap(np.ones((8,8),np.uint8),.1,(-.4,-.4))
    a=np.array([[0.,-0.],[-0.,0.],[0.,0.]])
    b=np.array([[.1,0.],[.1,-0.],[0.,0.]])
    np.testing.assert_array_equal(grid.valid_segments(a,b,0.),original_valid_segments(grid,a,b,0.))
    for invalid in (float('nan'),float('inf')):
        broken=a.copy();broken[0,0]=invalid
        np.testing.assert_array_equal(grid.valid_segments(broken,b,0.),original_valid_segments(grid,broken,b,0.))
    for invalid in (-.01,float('nan'),float('inf')):
        with pytest.raises(ValueError,match='invalid margin'):grid.valid_segments(a,b,invalid)
    for certify in (grid.valid_segments,lambda *args: original_valid_segments(grid,*args)):
        with pytest.raises(ValueError,match='bounded local range'):
            certify(np.zeros((1,2)),np.array([[1000.,0.]]),0.)
