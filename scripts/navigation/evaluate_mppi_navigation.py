#!/usr/bin/env python3
"""Bounded real navigation/map-producer smoke; never a full policy evaluation."""
import argparse
from dataclasses import replace,asdict
import json,math,time,traceback
from pathlib import Path
import numpy as np
from lunar_drl_exploration.config import training_config_from_record
from lunar_drl_exploration.ros_env import RosExplorationEnv
from lunar_drl_exploration.processes import OwnedProcesses
from lunar_drl_exploration.plant import KinematicPlant


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--experiment',type=Path,required=True);ap.add_argument('--case-files',type=Path,nargs='+',required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--domain',type=int,default=178);ap.add_argument('--backends',nargs='+',choices=['pure_pursuit','mppi'],default=['pure_pursuit','mppi']);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    record=json.loads((args.experiment/'config.json').read_text());original_start=OwnedProcesses.start;original_advance=KinematicPlant.advance
    active=None;current_backend=None;traces=[]
    def start(self,package,executable,namespace,parameters,env=None):
        if package=='lunar_pure_wheeled_controller':
            parameters=dict(parameters,controller_backend=current_backend,policy_map_service=namespace+'/get_map',mppi_diagnostics_topic=namespace+'/mppi_diagnostics')
        return original_start(self,package,executable,namespace,parameters,env)
    def advance(self,v,w,dt):
        try:return original_advance(self,v,w,dt)
        finally:
            if active is not None and self is active.plant:
                p=self.pose;traces.append([self.simulation_s,p.x,p.y,p.yaw,self.linear_mps,self.angular_radps,v,w])
    OwnedProcesses.start=start;KinematicPlant.advance=advance
    results=[]
    try:
        for file in args.case_files:
            case=json.loads(file.read_text())['proof']['case'];shared_goal=None
            for current_backend in args.backends:
                out=args.output/(case['case_id']+'-'+current_backend);out.mkdir(exist_ok=True)
                config=replace(training_config_from_record(record),environments=1,target_rtf=1.,artifact_root=str(out))
                env=RosExplorationEnv(config,0,domain_base=args.domain,wall_timeout_s=40.,training=False);active=env;traces=[];started=time.monotonic()
                result=dict(case=case['case_id'],backend=current_backend,scope='one navigation action, real native map/A*/controller; not actor rollout',target_rtf=1.)
                try:
                    env.reset(**case['spec']);p=env.plant.pose
                    result['initial_pose']=asdict(p);result['reference_id']=env.reference.reference_id
                    if shared_goal is None:
                        goals=[tuple(float(v) for v in g) for g in env.observation.goals]
                        candidates=[g for g in goals if math.hypot(g[0]-p.x,g[1]-p.y)>=1.5]
                        if not candidates:raise RuntimeError('no nontrivial frozen-observation goal')
                        shared_goal=min(candidates,key=lambda g:math.hypot(g[0]-p.x,g[1]-p.y))
                    result['goal']=shared_goal
                    navigation=env._execute_goal(shared_goal)
                    result.update(navigation=asdict(navigation),collision=bool(env.plant.collision),actual_pose=asdict(env.plant.pose),references=list(env.adapter.reference_history),tracking=list(env.adapter.tracking_history),budget=env.last_navigation_budget)
                    result['position_error_m']=math.hypot(env.plant.pose.x-shared_goal[0],env.plant.pose.y-shared_goal[1])
                    result['success']=navigation.outcome==0 and result['position_error_m']<=config.goal_position_tolerance_m and not env.plant.collision
                except Exception as exc:
                    result.update(success=False,error=str(exc),traceback=traceback.format_exc(),collision=bool(getattr(getattr(env,'plant',None),'collision',False)))
                finally:
                    result['wall_s']=time.monotonic()-started;result['trace']=traces
                    (out/'result.json').write_text(json.dumps(result,indent=2,default=str)+'\n')
                    results.append({k:v for k,v in result.items() if k not in ('trace','traceback','references','tracking','budget')})
                    (args.output/'summary.json').write_text(json.dumps(results,indent=2,default=str)+'\n');print(json.dumps(results[-1],default=str),flush=True)
                    active=None;env.close()
    finally:OwnedProcesses.start=original_start;KinematicPlant.advance=original_advance
if __name__=='__main__':main()
