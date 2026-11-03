"""观测覆盖独立几何oracle及线性工作量回归；分类由world测试负责。"""
from dataclasses import replace
from collections import defaultdict
import math,random
import numpy as np
import pytest
from test_p3_terrain_native import compiled,_api
from test_convex_polygon_native import area
from scripts.navigation.one_cm.nav1cm import p3_evidence as kernel
from scripts.navigation.one_cm.nav1cm.base_map import SourceEvidence,EvidenceFragment,MEASUREMENT_DTYPE


def source(identity,side,res=.01,origin=(0.,0.),yaw=0.):
    values=np.array([(x,y,float(identity),(x+y)%3-1,np.nan,np.nan) for y in range(side) for x in range(side)],dtype=MEASUREMENT_DTYPE)
    return SourceEvidence(identity,res,origin,yaw,values,())


def polygon(s,r):
    c,t=math.cos(s.yaw),math.sin(s.yaw);scale=s.resolution_m*100
    x,y=int(r['x']),int(r['y'])
    return tuple((s.origin_xy_m[0]*100+scale*(c*u-t*v),s.origin_xy_m[1]*100+scale*(t*u+c*v))
                 for u,v in ((x,y),(x+1,y),(x+1,y+1),(x,y+1)))


def oracle(previous,incoming,indices,ops):
    clips=[polygon(incoming,incoming.measurements[i]) for i in indices]
    result=[]
    for s in previous:
        fragments=[]
        for f in s.observation_fragments:
            pieces=[f.polygon_cm]
            for clip in clips:
                pieces=[q for p in pieces for q in reference_difference(p,clip)]
            fragments.extend(EvidenceFragment(f.measurement_index,p) for p in pieces)
        if fragments:result.append(replace(s,observation_fragments=tuple(fragments)))
    if indices:result.append(replace(incoming,observation_fragments=tuple(EvidenceFragment(i,p) for i,p in zip(indices,clips))))
    return tuple(result)


def reference_clip(subject,a,b,inside=True):
    out=[]
    if not subject:return out
    def side(p):
        value=(b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
        return value if inside else -value
    prior=subject[-1];vp=side(prior)
    for current in subject:
        vc=side(current)
        if (vp>=0)!=(vc>=0):
            t=vp/(vp-vc)
            out.append((prior[0]+t*(current[0]-prior[0]),prior[1]+t*(current[1]-prior[1])))
        if vc>=0:out.append(current)
        prior,vp=current,vc
    return out


def reference_difference(subject,clip):
    # 独立Python参考不再把自身产生的近共线碎片送回被测native验证器。
    pieces=[];remaining=list(subject)
    for a,b in zip(clip,clip[1:]+clip[:1]):
        outside=reference_clip(remaining,a,b,False)
        if len(outside)>=3 and area((outside,))>0:pieces.append(tuple(outside))
        remaining=reference_clip(remaining,a,b)
    return pieces


def intersection_area(subject,clip):
    out=list(subject)
    for a,b in zip(clip,clip[1:]+clip[:1]):out=reference_clip(out,a,b)
    return area((out,)) if len(out)>=3 else 0.


def support(sources):
    result=defaultdict(list)
    for s in sources:
        for f in s.observation_fragments:
            r=s.measurements[f.measurement_index]
            result[s.source_id,int(r['x']),int(r['y']),float(r['height_m']),int(r['state'])].append(f.polygon_cm)
    return result


@pytest.mark.parametrize('seed',[3,17,91])
@pytest.mark.parametrize('yaw',[0.,.37])
def test_partial_rotated_subcm_coverage_matches_independent_oracle(compiled,seed,yaw):
    rng=random.Random(seed)
    with _api().NativeTerrain(compiled[0]) as ops:
        actual=expected=()
        for identity in range(1,5):
            s=source(identity,2,rng.choice((.005,.013,.02)),(rng.uniform(-.01,.01),rng.uniform(-.01,.01)),yaw)
            indices=tuple(i for i in range(4) if rng.random()<.75)
            saved=[(v.measurements.tobytes(),v.observation_fragments) for v in actual]
            prior=actual
            actual=kernel.update_evidence(actual,s,indices,ops);expected=oracle(expected,s,indices,ops)
            a,b=support(actual),support(expected);assert a.keys()==b.keys()
            for key in a:
                assert area(a[key])==pytest.approx(area(b[key]),abs=1e-8)
                common=sum(intersection_area(x,y) for x in a[key] for y in b[key])
                assert common==pytest.approx(area(b[key]),abs=1e-8)
            assert saved==[(v.measurements.tobytes(),v.observation_fragments) for v in prior]


@pytest.mark.parametrize('history',[False,True])
def test_full_grid_replacement_does_not_construct_classification_history(compiled,history):
    s=source(1,32)
    with _api().NativeTerrain(compiled[0]) as ops:
        previous=kernel.update_evidence((),s,tuple(range(len(s.measurements))),ops) if history else ()
        new=source(2,32)
        result=kernel.update_evidence(previous,new,tuple(range(len(new.measurements))),ops)
        assert len(result)==1 and len(result[0].observation_fragments)==32*32
        assert not hasattr(result[0],'classification_fragments')
        assert len(result[0].measurements)==32*32
