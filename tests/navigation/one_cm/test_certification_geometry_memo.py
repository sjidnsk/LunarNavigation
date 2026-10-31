"""精确认证几何复用：全部证书字段与943555d冻结实现对照。"""
from dataclasses import replace
import os
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import certification as current
from scripts.navigation.one_cm.nav1cm.contracts import GridSnapshot, Trajectory, WheelLimits
from scripts.navigation.one_cm.nav1cm.seeds import rollout
from tests.navigation.one_cm import _certification_geometry_baseline as frozen


def make_trajectory(n=20, mode='forward', dt=.02):
    state=np.array([0.,0.,.2,.15,.03,0.])
    if mode=='reverse': state[3]=-.15
    if mode=='spin': state[3]=0.;state[4]=.08
    if mode=='stopped': state[3:5]=0.;mode='spin'
    controls=np.zeros((n,3))
    return Trajectory(1,7,np.arange(n+1)*dt,rollout(state,controls,dt),controls,mode,'memo-baseline')


def grid(kind):
    cells=np.zeros((500,500),dtype=np.int8)
    origin=(-2.5,-2.5)
    if kind=='blocked':cells[245:255,245:255]=1
    if kind=='unknown':cells[245:255,245:255]=-1
    if kind=='dense':cells[:]=-1;cells[150:350,150:350]=0
    if kind=='bounds':origin=(0.,0.)
    if kind=='near':cells[290:295,310:320]=1
    return GridSnapshot(1,origin,.01,cells,False)


def assert_equivalent(t,g,limits,backend=None):
    old=frozen.certify(t,g,limits,t.route_version,backend=backend)
    new=current.certify(t,g,limits,t.route_version,backend=backend)
    assert new == old  # dataclass equality includes count, time, detail and identity digests.
    return new


@pytest.mark.parametrize('kind',['free','blocked','unknown','dense','bounds','near'])
@pytest.mark.parametrize('mode',['forward','reverse','spin','stopped'])
@pytest.mark.parametrize('clearance',[0.,.2])
def test_complete_certificate_matches_frozen(kind,mode,clearance):
    limits=WheelLimits(motion_model='differential_drive',minimum_clearance_m=clearance)
    assert_equivalent(make_trajectory(mode=mode),grid(kind),limits)


@pytest.mark.parametrize('fault',['dynamics','limit','mode','epoch','route','nonfinite'])
def test_mixed_faults_preserve_priority_and_count(fault):
    t=make_trajectory(); states=t.states.copy()
    if fault=='dynamics':states[2,0]+=.01
    if fault=='limit':states[0,3]=1.
    if fault=='mode':states[0,3]=-.15
    if fault=='nonfinite':states[2,0]=float('nan')
    t=replace(t,states=states)
    if fault=='epoch':t=replace(t,map_epoch=2)
    if fault=='route':t=replace(t,route_version=8)
    # Blocking geometry competes with faults at different segments.
    old=frozen.certify(t,grid('blocked'),WheelLimits(),7)
    new=current.certify(t,grid('blocked'),WheelLimits(),7)
    assert new==old


def test_repeated_geometry_is_evaluated_once_but_count_is_unchanged():
    class CountingBackend:
        def __init__(self):self.calls=0
        def check_poses(self,poses,margin,context,limits):
            self.calls+=1
            return None,len(poses)
    t=make_trajectory(mode='stopped'); g=grid('free'); limits=WheelLimits()
    backend=CountingBackend()
    certificate=current.certify(t,g,limits,7,backend=backend)
    assert certificate.valid and certificate.checked_poses==40
    assert backend.calls==1
    current.certify(t,g,limits,7,backend=backend)
    assert backend.calls==2  # Never share geometry across certify calls.


def test_context_can_be_shared_without_sharing_geometry_results():
    t=make_trajectory(mode='stopped');g=grid('free');limits=WheelLimits()
    context=current.prepare_certification(g)
    a=current.certify(t,g,limits,7,context=context)
    b=current.certify(t,g,limits,7,context=context)
    assert a==b==frozen.certify(t,g,limits,7)


def test_native_certificates_match_frozen_if_configured():
    path=os.environ.get('NAV1CM_GEOMETRY_LIB')
    if not path:pytest.skip('native geometry library not configured')
    from scripts.navigation.one_cm.nav1cm.geometry_native import NativeGeometry
    backend=NativeGeometry(path)
    for kind in ('free','blocked','unknown','bounds','near'):
        for mode in ('forward','reverse','spin','stopped'):
            assert_equivalent(make_trajectory(mode=mode),grid(kind),WheelLimits(motion_model='differential_drive',minimum_clearance_m=.2),backend)


def test_exact_geometry_key_keeps_yaw_margin_and_position():
    cells=np.zeros((500,500),np.int8);cells[249:251,305:307]=1
    g=replace(grid('free'),cells=cells)
    limits=WheelLimits(body_length_m=1.2,body_width_m=.3)
    context=replace(current.prepare_certification(g),geometry_cache={})
    for pose,margin in [([0.,0.,0.],0.),([0.,0.,np.pi/2],0.),
                        ([-.3,0.,0.],0.),([-.3,0.,0.],.4),
                        ([0.,0.,np.pi/2],0.)]:
        assert current._body_collision(pose,margin,context,limits)==frozen._body_collision(pose,margin,frozen.prepare_certification(g),limits)


def test_braking_prefix_and_tail_match_complete_frozen_certificate():
    from scripts.navigation.one_cm.nav1cm.contracts import VehicleState,LocalReference
    from scripts.navigation.one_cm.nav1cm.seeds import brake_candidate
    for model in ('differential_drive','curvature_limited'):
        limits=WheelLimits(motion_model=model,minimum_clearance_m=.2)
        for sign in (-1.,1.):
            g=grid('near')
            ref=LocalReference(7,1,[[0.,0.]],[0.],0.)
            t=brake_candidate(VehicleState(0.,0.,.3,sign*.15,.05),g,ref,limits)
            assert_equivalent(t,g,limits)
            # Same physical braking prefix evaluated as entry to spin.
            assert_equivalent(replace(t,mode='spin'),g,limits)
            for count in (1,3,6):
                partial=replace(t,times=t.times[:count+1],states=t.states[:count+1],controls=t.controls[:count])
                assert_equivalent(partial,g,limits)


def test_seeded_mixed_maps_trajectories_and_faults_match_frozen():
    rng=np.random.default_rng(6271)
    for i in range(120):
        mode=('forward','reverse','spin')[i%3]
        n=int(rng.integers(1,26));dt=float(rng.uniform(.005,.2))
        initial=np.r_[rng.uniform(-1.5,1.5,2),rng.uniform(-6,6),.1,.02,0.]
        if mode=='reverse':initial[3]=-.1
        if mode=='spin':initial[3]=0.
        controls=rng.uniform(-.05,.05,(n,3));controls[:,2]=0.
        if mode=='spin':controls[:,0]=0.
        states=rollout(initial,controls,dt)
        if i%7==0:states[min(2,n),0]+=.01
        t=Trajectory(1,7,np.arange(n+1)*dt,states,controls,mode,'random-memo-proof')
        cells=np.zeros((500,500),np.int8)
        cells[rng.integers(0,500,120),rng.integers(0,500,120)]=rng.choice([-1,1],120)
        g=replace(grid('free'),cells=cells,allow_unknown=bool(i%2))
        limits=WheelLimits(motion_model=('differential_drive','curvature_limited')[i%2],minimum_clearance_m=(0.,.2)[i%2])
        assert_equivalent(t,g,limits)


def benchmark():
    """python -B -m tests.navigation.one_cm.test_certification_geometry_memo；非计时断言。"""
    from time import perf_counter
    from scripts.navigation.one_cm.nav1cm.execution import command_hold_proof
    limits=WheelLimits(motion_model='differential_drive',minimum_clearance_m=.2)
    free=grid('free');cells=free.cells.copy();cells[310:315,170:175]=1
    near=replace(free,cells=cells)
    y,x=np.mgrid[:500,:500]
    dense=replace(free,cells=np.where((x-250)**2+(y-250)**2>100**2,-1,0).astype(np.int8))
    for map_name,g in [('free',free),('near',near),('dense_disk',dense)]:
        for mode in ('forward','spin','stopped'):
            for n in (20,100):
                t=make_trajectory(n,mode)
                for label,trajectory in [('raw',t),('hold1.2',command_hold_proof(t,g,limits,1.2))]:
                    assert_equivalent(trajectory,g,limits)
                    samples=[[],[]]
                    # Interleave implementations to reduce temperature/load order bias.
                    for iteration in range(44):
                        for index in ((0,1) if iteration%2 else (1,0)):
                            module=(frozen,current)[index]
                            start=perf_counter();result=module.certify(trajectory,g,limits,7)
                            if iteration>=4:samples[index].append((perf_counter()-start)*1000)
                    print(map_name,mode,n,label,result.reason.value,result.checked_poses,
                          'old/new p50,p95 ms',*[np.percentile(row,[50,95]).round(3).tolist() for row in samples],flush=True)


if __name__=='__main__':
    benchmark()
