"""Bounded task approach shaping, based only on actual pose and task geometry."""
import math
import numpy as np


def validate_spec(spec):
    if spec is None:return
    if set(spec)!={'mode','bound','scale_m'} or spec['mode'] not in ('bounded_quadratic_v1','bounded_exponential_v1'):
        raise ValueError('invalid approach reward specification')
    for key in ('bound','scale_m'):
        if isinstance(spec[key],bool) or not math.isfinite(spec[key]) or spec[key]<=0:
            raise ValueError('invalid approach reward '+key)


def distance_to_region(position,polygon):
    p=np.asarray(position,dtype=float);a=np.asarray(polygon,dtype=float)
    if p.shape!=(2,) or a.ndim!=2 or a.shape[1]!=2 or len(a)<3 or not np.isfinite(p).all() or not np.isfinite(a).all():
        raise ValueError('invalid task geometry')
    b=np.roll(a,-1,axis=0);v=b-a
    denom=np.einsum('ij,ij->i',v,v)
    t=np.clip(np.einsum('ij,ij->i',p-a,v)/np.maximum(denom,1e-30),0,1)
    distance=float(np.linalg.norm(p-(a+t[:,None]*v),axis=1).min())
    if distance<=1e-9:return 0.
    inside=False
    for x,y in zip(a,b):
        if (x[1]>p[1])!=(y[1]>p[1]) and p[0]<(y[0]-x[0])*(p[1]-x[1])/(y[1]-x[1])+x[0]:
            inside=not inside
    return 0. if inside else distance


def observation_distance(observation):
    # The policy polygon is translated by actual robot pose, then divided by10m.
    return distance_to_region((0.,0.),np.asarray(observation.polygon,dtype=float)*10.)


def potential(distance,spec):
    validate_spec(spec)
    if not math.isfinite(distance) or distance<0:raise ValueError('invalid task distance')
    if spec is None:return 0.
    if distance==0:return 0.
    if spec['mode']=='bounded_exponential_v1':
        # expm1 preserves the useful linear signal arbitrarily close to the boundary.
        return spec['bound']*math.expm1(-distance/spec['scale_m'])
    # Reciprocal form avoids overflow at large finite distances.
    ratio=spec['scale_m']/distance
    if ratio>1e150:return 0.
    return -spec['bound']/(1.+ratio*ratio)


def shaping_reward(before,after,spec,*,terminated=False,gamma=1.):
    if not math.isfinite(gamma) or not 0<gamma<=1:raise ValueError('invalid discount')
    return gamma*(0. if terminated else potential(after,spec))-potential(before,spec)


def observation_shaping(before,after,spec,*,terminated=False,gamma=1.):
    if spec is None:return 0.
    return shaping_reward(observation_distance(before),observation_distance(after),spec,
                          terminated=terminated,gamma=gamma)
