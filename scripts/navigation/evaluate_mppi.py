#!/usr/bin/env python3
"""Paired controller evaluation using the independent existing KinematicPlant.

No training/actor/GPU is touched. This is a controller benchmark, not a policy
acceptance test. Each controller receives the same inflated map/reference and
initial state. The two-tick command transport delay is explicit.
"""
import argparse
from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace
import numpy as np
from lunar_pure_wheeled_controller.execution import PathExecutor
from lunar_pure_wheeled_controller.mppi import MppiExecutor,MppiPolicy
from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
from lunar_pure_wheeled_controller.mppi_map import GridMap
from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState
from lunar_drl_exploration.plant import KinematicPlant,GeometryFailure
from lunar_drl_exploration.contracts import Pose
from lunar_drl_exploration.config import load_platform_config


def fixtures():
    cases=[]
    def add(name,points,pose,states=None,origin=(-3.,-3.),resolution=.1,event=None):
        cases.append(dict(name=name,points=points,pose=pose,states=np.ones((60,60),np.uint8) if states is None else states,origin=origin,resolution=resolution,event=event))
    add('straight',[(0,0,0),(2,0,0)],(0,0,0))
    add('reverse',[(0,0,0),(-2,0,0)],(0,0,0))
    for yaw in [-.079,-.04,0.,.04,.079]:
        states=np.ones((40,60),np.uint8);states[0,:]=2
        add('wall_'+str(yaw),[(0,0,0),(2,0,0)],(0,0,yaw),states,(-1.,-.11))
    for yaw in [-.079,.079]:
        states=np.ones((40,60),np.uint8);states[0,:]=2
        add('wall_reverse_'+str(yaw),[(0,0,0),(-2,0,0)],(0,0,yaw),states,(-3.,-.11))
    for width in [.06,.1,.2]:
        n=int(round(width/.02));states=np.full((n+2,170),2,np.uint8);states[1:-1,:]=1
        add('corridor_'+str(width),[(0,0,0),(2,0,0)],(0,0,.06),states,(-.5,-width/2-.02),.02)
    for reverse in [False,True]:
        states=np.ones((50,50),np.uint8);states[20:22,20:22]=2
        points=[(-.15,-.3,0),(-.15,.35,0),(.5,.35,0)]
        add('corner_'+str(reverse),points,(-.15,-.3,math.pi/2+(math.pi if reverse else 0)),states,(-2.,-2.))
    add('final_yaw',[(0,0,0),(.8,0,math.pi/2)],(0,0,0))
    add('offset_start',[(0,0,0),(2,0,0)],(0,.15,-.05))
    for state in [0,2]:
        add('map_update_'+str(state),[(0,0,0),(2,0,0)],(0,0,0),event=(45,state))
    return cases


def run(case,backend,*,max_ticks=1600,delay=2,mppi_policy=None):
    policy=TrackingPolicy(goal_position_tolerance_m=.1,goal_yaw_tolerance_rad=.05)
    controller=(MppiExecutor(policy,mppi_policy) if backend=='mppi' else MppiPathFollower(policy,mppi_policy) if backend=='mppi_path_follower' else PathExecutor(policy,rolling_updates=True))
    controller.set_path(case['points'],identity=(b'eval',1))
    states=case['states'].copy();origin=case['origin'];resolution=case['resolution']
    truth=states.copy()
    terrain=SimpleNamespace(navigation=truth,origin=(*origin,0.),resolution_m=resolution)
    plant=KinematicPlant(terrain,Pose(*case['pose']),load_platform_config())
    if backend=='mppi':controller.set_map(GridMap(states,resolution,origin))
    pending=deque([(0.,0.)]*delay);trace=[];computes=[];selected=[];collision=False;reason='TIME_BUDGET';complete=False;start=perf_counter()
    for tick in range(max_ticks):
        event=case['event']
        if event and tick==event[0]:
            # Introduce a future blocker, not an obstacle under the vehicle.
            states[:,int(round((.8-origin[0])/resolution))]=event[1]
            if event[1]==2:
                truth[:,int(round((.8-origin[0])/resolution))]=2
            # Reference-only backends receive the upstream invalidation.
            if backend!='mppi':controller.clear()
            if backend=='mppi':controller.set_map(GridMap(states,resolution,origin,revision=2))
        pose=plant.pose
        state=TrackingState(pose.x,pose.y,pose.yaw,plant.linear_mps,plant.angular_radps)
        now=perf_counter();result=controller.update(state,.05);computes.append((perf_counter()-now)*1000.)
        selected.append(getattr(controller,'diagnostics',{}).get('selected'))
        command=(result.command.linear_x_mps,result.command.angular_z_radps)
        controller.note_command(*command)
        pending.append(command);applied=pending.popleft()
        try:plant.advance(*applied,.05)
        except GeometryFailure:collision=True;reason='COLLISION';break
        pose=plant.pose
        trace.append([tick*.05,pose.x,pose.y,pose.yaw,plant.linear_mps,plant.angular_radps,*command])
        if result.phase=='COMPLETED' and abs(plant.linear_mps)<=policy.stopped_linear_mps and abs(plant.angular_radps)<=policy.stopped_angular_radps:
            complete=True;reason='COMPLETED';break
        if result.phase=='FAILED':reason=result.command.failure_reason;break
        if event and tick>event[0] and result.phase in ('WAITING','BRAKING') and abs(plant.linear_mps)<=.01 and abs(plant.angular_radps)<=.03 and plant.pose.x<.8:
            reason='SAFE_STOP_AFTER_MAP_UPDATE';break
    goal=case['points'][-1]
    return dict(case=case['name'],backend=backend,complete=complete,collision=collision,reason=reason,
                simulation_s=(tick+1)*.05,wall_s=perf_counter()-start,distance_m=plant.distance_m,
                position_error_m=math.hypot(plant.pose.x-goal[0],plant.pose.y-goal[1]),
                yaw_error_rad=abs(math.atan2(math.sin(plant.pose.yaw-goal[2]),math.cos(plant.pose.yaw-goal[2]))),
                compute_ms=dict(p50=float(np.percentile(computes,50)),p95=float(np.percentile(computes,95)),max=max(computes)),
                selection_counts={key:selected.count(key) for key in ('weighted','sampled_fallback',None)},mppi_policy=asdict(controller.config) if backend in ('mppi','mppi_path_follower') else None,trace=trace,diagnostics=getattr(controller,'diagnostics',{}))


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--cases',nargs='*');ap.add_argument('--backends',nargs='+',default=['pure_pursuit','mppi']);ap.add_argument('--max-ticks',type=int,default=1600);ap.add_argument('--batch-size',type=int,default=32);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);results=[]
    for case in fixtures():
        if args.cases and case['name'] not in args.cases:continue
        for backend in args.backends:
            result=run(case,backend,max_ticks=args.max_ticks,mppi_policy=MppiPolicy(batch_size=args.batch_size));results.append({k:v for k,v in result.items() if k!='trace'})
            (args.output/(case['name']+'-'+backend+'.json')).write_text(json.dumps(result,indent=2)+'\n')
            (args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(results[-1]),flush=True)

if __name__=='__main__':main()
