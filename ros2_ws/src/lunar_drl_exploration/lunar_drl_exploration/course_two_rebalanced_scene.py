"""Versioned geometry for covering, approaching, finishing and returning.

Certificates and predecessor recipes are training/qualification metadata only.
Navigation still executes ordinary safe goals through the native ROS stack.
"""
from dataclasses import asdict, replace
import math
from types import MappingProxyType

import numpy as np

from .contracts import Pose, SensorSpec, TaskSpec
from .course_two_scene import _identity

VERSION = 'region_sensor_rebalanced_scene_v2'
SCOPED_VERSION = 'region_sensor_rebalanced_scene_v3'


def make_case(request, *, attempt=0):
    if type(attempt) is not int or not 0<=attempt<100:
        raise ValueError('bounded geometry attempt required')
    role=request['role']
    scoped=request.get('task_scope')=='whole_cave_v1'
    if scoped and (role not in ('cover','approach','finish') or
            request.get('terrain_template')=='outside_detour' or
            role=='finish' and request.get('planned_policy_start')!='inside'):
        raise ValueError('task scope requires inside finishing and no outside detour/return')
    if role not in ('cover','approach','finish','return','detour'):
        raise ValueError('invalid rebalanced role')
    family=request['family']
    if family not in ('moon','cave'):
        raise ValueError('invalid terrain family')
    rng=np.random.default_rng(np.random.SeedSequence([request['seed'],attempt,20261011]))
    sensor=SensorSpec(**request['sensor'])
    if not 10<=sensor.range_m<=100 or sensor.fov_deg not in (60,90,120,180,360):
        raise ValueError('sensor outside curriculum contract')
    low,high=request['roi_side_interval_m']
    width,height=map(float,rng.uniform(low,high,2))
    angle=int(request['bearing_bin'])*math.pi/4+float(rng.uniform(-math.pi/16,math.pi/16))
    visibility=request.get('visibility_after_native_initialization') if role=='approach' else 'V'
    gap=float(rng.uniform(*(request.get('blind_gap_m') or [1.,5.]))) if visibility=='U' else 0.
    distance=sensor.range_m+gap if visibility=='U' else 6. if role=='approach' else -3.
    if role=='finish' and request.get('planned_policy_start')=='outside':role='return'
    base_role='finish' if role in ('finish','return') else 'cover' if role=='detour' else role
    center=rng.uniform(-12.,12.,2)
    xy=center+(-width/2-distance)*np.array([math.cos(angle),math.sin(angle)])
    center+=.2*(np.floor(xy/.2)+.5)-xy
    case=dict(version=VERSION, request_id=request.get('request_id'), seed=int(request['seed']),
        attempt=attempt, family=family, role=role, stage='A1', bearing_bin=int(request['bearing_bin']),
        bearing_rad=angle, sensor=asdict(sensor), roi_width_m=width,roi_height_m=height,
        translation=list(map(float,center)), start_distance_m=distance, blind_gap_m=gap,
        initial_yaw_rad=angle if role!='approach' else angle+float(rng.uniform(-math.pi,math.pi)),
        terrain_template=request['terrain_template'], visibility_class=visibility,
        episode_budget=512, approach_decision_budget=128, resolution_m=.2,
        curriculum_task='c2_A1_'+base_role, prelude_recipe='measured_candidates_v1' if base_role=='finish' else None,
        max_prelude_decisions=512, return_route_max_m=10.)
    if scoped:case.update(version=SCOPED_VERSION,task_scope='whole_cave_v1')
    return dict(case,scenario_id=_identity(case))


def validate_case(case):
    record=dict(case);identity=record.pop('scenario_id',None)
    if record.get('version') not in (VERSION,SCOPED_VERSION) or _identity(record)!=identity:
        raise ValueError('rebalanced scene identity changed')
    if record['version']==SCOPED_VERSION and (record.get('task_scope')!='whole_cave_v1' or
            record['role'] not in ('cover','approach','finish') or record['terrain_template']=='outside_detour'):
        raise ValueError('invalid whole cave task scope')
    sensor=SensorSpec(**record['sensor'])
    from .sensor import validate_sensor
    validate_sensor(sensor)
    if (record['role'] not in ('cover','approach','finish','return','detour') or
            record['resolution_m']!=.2 or record['episode_budget']!=512):
        raise ValueError('invalid rebalanced scene semantics')
    return sensor


def validate_start(role, coverage, inside):
    valid=(role in ('cover','detour') and inside and 0<coverage<.8 or
           role=='approach' and not inside and coverage==0 or
           role=='finish' and inside and .9<=coverage<=.98 or
           role=='return' and not inside and .9<=coverage<=.98)
    if not valid:
        raise ValueError(f'policy start mismatch: {role}, coverage={coverage:.8f}, inside={inside}')


def build_case(case, platform):
    from scipy.ndimage import label
    from .scene import TerrainGrid, _segment_distance
    from .geometry import polygon_mask
    from .sensor import SensorModel
    sensor=validate_case(case)
    width,height=case['roi_width_m'],case['roi_height_m']
    a=case['bearing_rad'];c,s=math.cos(a),math.sin(a)
    rotation=np.array([[c,-s],[s,c]])
    center=np.asarray(case['translation'])
    local=np.array([-width/2-case['start_distance_m'],0.])
    polygon=np.array([[-width/2,-height/2],[width/2,-height/2],
                      [width/2,height/2],[-width/2,height/2]])@rotation.T+center
    start=local@rotation.T+center
    pose=Pose(*start,case['initial_yaw_rad'])
    context=np.vstack([polygon,start])
    origin=np.floor((context.min(0)-sensor.range_m-2)/16)*16
    high=np.ceil((context.max(0)+sensor.range_m+2)/16)*16
    if np.any(high-origin>512):raise ValueError('map exceeds 512 m')
    resolution=.2;nx,ny=np.rint((high-origin)/resolution).astype(int)
    x=origin[0]+(np.arange(nx)+.5)*resolution-center[0]
    y=origin[1]+(np.arange(ny)+.5)*resolution-center[1]
    xx=x[None,:]*c+y[:,None]*s;yy=-x[None,:]*s+y[:,None]*c
    role=case['role'];template=case['terrain_template']
    route=[local]
    wall_x=None
    if role=='approach' and case['visibility_class']=='U':
        route=[local,local+np.array([case['blind_gap_m']+2.,0.])]
    elif role=='approach':
        wall_x=local[0]+3.
        route=[local,np.array([local[0]-1.,8.5]),np.array([wall_x+2.5,8.5])]
    if case['family']=='cave':
        if case.get('task_scope')=='whole_cave_v1':
            floor=(np.abs(xx)<width/2-.6)&(np.abs(yy)<height/2-.6)
            floor|=(np.abs(yy)<2.8)&(xx>=-width/2)&(xx<-width/2+3)
            floor|=xx < -width/2  # open exterior before the cave mouth
        else:
            floor=(np.abs(xx)<width/2+4)&(np.abs(yy)<height/2+4)
            floor|=(np.abs(yy)<2.8)&(xx>local[0]-6)&(xx<width/2+5)
            for p,q in zip(route,route[1:]):
                floor|=_segment_distance(xx,yy,*p,*q)<2.8
        heights=np.where(floor,0.,3.).astype(np.float32)
    else:
        heights=(.02*np.sin(xx*.09+case['seed']%31)+.015*np.cos(yy*.11)).astype(np.float32)
        # Outside distractors remain present for all roles.
        for cx,cy in [(-width/3,height/2+7),(width/3,-height/2-7)]:
            heights[(xx-cx)**2+(yy-cy)**2<1.2**2]=1.5
    if template=='internal_occlusion':
        heights[(np.abs(xx+width*.12)<.6)&(yy>-height*.30)&(yy<height*.06)]=3.
        heights[(np.abs(xx-width*.17)<.6)&(yy>-height*.02)&(yy<height*.34)]=3.
    if template=='outside_detour':
        heights[(np.abs(xx)<.6)&(np.abs(yy)<height/2+1.2)]=3.
    if wall_x is not None:
        heights[(np.abs(xx-wall_x)<.5)&(np.abs(yy)<6.3)]=3.
    terrain=TerrainGrid.from_heights(heights,resolution,tuple(origin),platform)
    task=TaskSpec(case['scenario_id'],'map',polygon)
    roi=polygon_mask(terrain.shape,terrain.origin,resolution,polygon)
    effective=roi&(terrain.intrinsic!=0)
    sx,sy=terrain.world_to_cell(pose.x,pose.y)
    if terrain.navigation[sy,sx]!=1:raise ValueError('start not navigable')
    if bool(roi[sy,sx])!=(role!='approach'):raise ValueError('wrong physical start domain')
    def count(p, omni=False):
        spec=replace(sensor,fov_deg=360.) if omni else sensor
        return int(np.count_nonzero(SensorModel.observe(terrain,p,spec).mask&effective))
    initially=count(pose,omni=role=='approach')
    if role=='approach' and initially:raise ValueError('approach visible before initialization')
    if role!='approach' and not initially:raise ValueError('inside start has no ROI visibility')
    length=sum(float(np.linalg.norm(q-p)) for p,q in zip(route,route[1:]))
    if length>20.000001:raise ValueError('A1 first observation path exceeds 20 m')
    for p,q in zip(route,route[1:]):
        samples=np.linspace(p,q,max(2,math.ceil(np.linalg.norm(q-p)/.1)+1))@rotation.T+center
        cols=np.floor((samples[:,0]-origin[0])/resolution).astype(int)
        rows=np.floor((samples[:,1]-origin[1])/resolution).astype(int)
        if not np.all(terrain.navigation[rows,cols]==1):raise ValueError('reference approach route blocked')
    endpoint=np.asarray(route[-1])@rotation.T+center
    ep=Pose(*endpoint,math.atan2(center[1]-endpoint[1],center[0]-endpoint[0]))
    endpoint_count=count(ep)
    if not endpoint_count:raise ValueError('reference endpoint has no ROI observation')
    detour=False;witness=[]
    if template=='outside_detour':
        witness=(np.array([[-3.,0.],[3.,0.]])@rotation.T+center).tolist()
        cells=[terrain.world_to_cell(*p) for p in witness]
        full,_=label(terrain.navigation==1)
        inside,_=label((terrain.navigation==1)&roi)
        labels=[full[y,x] for x,y in cells];local_labels=[inside[y,x] for x,y in cells]
        detour=bool(labels[0] and labels[0]==labels[1] and local_labels[0] and
                    local_labels[1] and local_labels[0]!=local_labels[1])
        if not detour:raise ValueError('outside detour certificate failed')
    descriptor=dict(case,generator_version=VERSION,extent_m=float(max(high-origin)),
        provided_bounds=[*origin,*high],start_mode='outside' if role=='approach' else 'inside',
        policy_start_mode='outside' if role in ('approach','return') else 'inside',
        approach_distance_budget_m=max(20.,4*length),reference_path_m=length,
        reference_observation_pose=[ep.x,ep.y,ep.yaw],reference_endpoint_roi_cells=endpoint_count,
        initial_observable_roi_cells=initially,
        internal_obstacle_cells=int(np.count_nonzero((heights>1)&roi)),
        outside_detour_certified=detour,detour_witness_world=witness)
    if case.get('task_scope')=='whole_cave_v1':
        descriptor['generator_version']=SCOPED_VERSION
        if case['family']=='cave':
            outside=int(np.count_nonzero((heights==0)&(xx>=-width/2)&~roi))
            if outside:raise ValueError('whole cave task cuts interior')
            descriptor['cave_interior_outside_cells']=outside
    terrain.generator_descriptor=MappingProxyType(descriptor)
    return terrain,pose,task,sensor,descriptor
