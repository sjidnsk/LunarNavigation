"""冻结投影原实现，确保优化保持候选顺序、边界及细化语义。"""
import math
from types import SimpleNamespace
from time import perf_counter
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import reference_buffer as module
from scripts.navigation.one_cm.nav1cm.reference_buffer import _at
from scripts.navigation.one_cm.nav1cm.trajectory_buffer import FeedbackTolerance
from scripts.navigation.one_cm.nav1cm.seeds import rollout

def legacy_project(source, observed, floor, tolerance):
    """只以几何/yaw 投影；参考速度不同不代表消费进度不同。"""
    scale = np.maximum([tolerance.position_m, tolerance.position_m, tolerance.yaw_rad], 1e-9)

    def score(phase):
        expected, _ = _at(source, phase)
        delta = observed[:3] - expected[:3]
        delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        return float(np.sum((delta / scale) ** 2))

    best = (float('inf'), floor, 0)
    for index, control in enumerate(source.controls):
        lo, hi = max(float(source.times[index]), floor), float(source.times[index + 1])
        if lo > hi:
            continue
        start, _ = _at(source, lo)
        delta = observed[:3] - start[:3]
        delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        vector = source.states[index + 1, :3] - start[:3]
        vector[2] = math.atan2(math.sin(vector[2]), math.cos(vector[2]))
        vector /= scale
        norm = float(vector @ vector)
        fraction = 0. if norm <= 1e-20 else float(np.clip((delta / scale) @ vector / norm, 0., 1.))
        for phase in (lo, hi, lo + fraction * (hi - lo)):
            value = score(phase)
            if value < best[0] - 1e-12:
                best = value, phase, index
    _, phase, index = best
    lo, hi = max(float(source.times[index]), floor), float(source.times[index + 1])
    # 只细化已选的小段；精确 step 的加速位移不是线性插值。
    ratio = (math.sqrt(5.) - 1.) / 2
    left, right = hi - ratio * (hi - lo), lo + ratio * (hi - lo)
    fl, fr = score(left), score(right)
    for _ in range(24):
        if fl < fr:
            hi, right, fr = right, left, fl
            left = hi - ratio * (hi - lo); fl = score(left)
        else:
            lo, left, fl = left, right, fr
            right = lo + ratio * (hi - lo); fr = score(right)
    refined = (lo + hi) / 2
    return max(floor, refined if score(refined) < best[0] - 1e-12 else phase)



def cases(n, count=30):
    rng=np.random.default_rng(4521+n)
    for i in range(count):
        initial=np.array([0.,0.,rng.uniform(-8,8), rng.uniform(-.3,.3),rng.uniform(-2.,2.),0.])
        controls=rng.uniform(-.5,.5,(n,3));controls[:,2]=.1
        if i%3==0: initial[3]=0.;controls[:,0]=0.
        states=rollout(initial,controls,.2)
        if i%4==0: states[1::3,0]+=.04  # 不一致端态也保留_at边界语义。
        source=SimpleNamespace(times=np.arange(n+1)*.2,states=states,controls=controls)
        floor=float(rng.choice(source.times[:-1])) if i%2 else rng.uniform(0.,n*.15)
        phase=rng.uniform(floor,n*.2)
        observed=_at(source,phase)[0];observed[:3]+=rng.normal(0,.01,3)
        observed[2]+=2*np.pi*(i%3-1)
        yield source,observed,floor


@pytest.mark.parametrize('n',[20,100])
def test_optimized_projection_matches_frozen_original(n):
    tolerance=FeedbackTolerance()
    for source,observed,floor in cases(n):
        expected=legacy_project(source,observed,floor,tolerance)
        actual=module._project(source,observed,floor,tolerance)
        assert actual==pytest.approx(expected,abs=1e-12,rel=0.)
        a=_at(source,actual)[0];b=_at(source,expected)[0]
        assert np.allclose(a,b,atol=1e-12,rtol=0.)


def test_projection_uses_known_segments_without_repeated_search(monkeypatch):
    source,observed,floor=next(cases(20,1))
    calls=[]
    original=module._at
    def counted(*args): calls.append(1);return original(*args)
    monkeypatch.setattr(module,'_at',counted)
    module._project(source,observed,floor,FeedbackTolerance())
    assert len(calls)==0


def benchmark():
    for n in (20,100):
        inputs=list(cases(n,12));tolerance=FeedbackTolerance()
        for fn in (legacy_project,module._project):
            started=perf_counter()
            for _ in range(8):
                for args in inputs:fn(*args,tolerance)
            print(n,fn.__name__,(perf_counter()-started)*1000/96)


@pytest.mark.parametrize('spin',[False,True])
def test_exact_boundaries_and_inconsistent_nodes_match_original(spin):
    controls=np.zeros((40,3));controls[:,0]=0. if spin else .12
    controls[:,1]=np.tile([.3,-.2,0.,-.1],10)
    initial=np.array([0.,0.,3.12,0. if spin else -.2,1.6,0.])
    states=rollout(initial,controls,.2)
    states[1::2,1]+=.07
    source=SimpleNamespace(times=np.arange(41)*.2,states=states,controls=controls)
    tolerance=FeedbackTolerance()
    for index in (0,1,10,20,39,40):
        for floor in (0.,source.times[max(0,index-1)],source.times[index]):
            observed=states[index].copy();observed[2]-=2*np.pi
            old=legacy_project(source,observed,float(floor),tolerance)
            new=module._project(source,observed,float(floor),tolerance)
            assert new==pytest.approx(old,abs=1e-12,rel=0.)
            delta_old=observed[:3]-_at(source,old)[0][:3]
            delta_new=observed[:3]-_at(source,new)[0][:3]
            assert np.allclose(delta_old,delta_new,atol=1e-12,rtol=0.)
