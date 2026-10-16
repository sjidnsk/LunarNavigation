#!/usr/bin/env python3
"""Bounded native sensor/Actor/ROS checks; never updates the learned model."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time
import torch
from lunar_drl_exploration.config import training_config_from_record
from lunar_drl_exploration.course_two_scene import make_case
from lunar_drl_exploration.course_two_control import write, sha
from lunar_drl_exploration.ros_env import RosExplorationEnv
from lunar_drl_exploration.runtime import ActorPolicy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--actor',type=Path,required=True)
    parser.add_argument('--start',type=int,default=0)
    parser.add_argument('--count',type=int,default=24)
    parser.add_argument('--wall-timeout',type=float,default=120.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    torch.set_num_threads(1)
    config=replace(training_config_from_record(json.loads(args.config.read_text())),environments=1)
    actor=ActorPolicy.load(args.actor);rows=[];ordinal=-1
    for family in ('moon','cave'):
        for r in (10.,40.,100.):
            for fov in (60.,90.,180.,360.):
                ordinal+=1
                if not args.start<=ordinal<args.start+args.count:continue
                case=make_case(27102026+ordinal,'A1',ordinal,range_m=r,fov_deg=fov,
                    visibility='U',family=family)
                env=RosExplorationEnv(config,0,domain_base=212,wall_timeout_s=args.wall_timeout)
                start=time.monotonic();row=dict(family=family,range_m=r,fov_deg=fov,case=case)
                print(json.dumps(dict(ordinal=ordinal,phase='reset',family=family,range_m=r,fov_deg=fov)),flush=True)
                try:
                    env.reset(case['seed'],family,case['roi_width_m'],episode_budget=512,
                        start_mode='outside',curriculum_task=case['curriculum_task'],course_two_case=case)
                    row['reset_s']=time.monotonic()-start
                    sensor=env.sensor;reference=env.reference.reference_id
                    initial=env.reference.coverage_ratio(env.privileged.observed)
                    assert initial==0.,initial
                    assert asdict(sensor)==case['sensor']
                    assert env.core.sensor==sensor and env.privileged_builder.sensor==sensor
                    action=actor(env.observation)
                    row['action']=int(action)
                    row['goal']=env.observation.goals[int(action)].tolist()
                    row['initial_pose']=asdict(env.plant.pose)
                    row['context']=env.observation.context.tolist()
                    pump=env._pump;last=[0.]
                    def inspect_pump():
                        pump()
                        if time.monotonic()-last[0]>5:
                            last[0]=time.monotonic()
                            write(args.output/f'live-{ordinal}.json',dict(progress=env.progress(),
                                pose=asdict(env.plant.pose),command=env._command,
                                action=int(action),goal=row['goal'],wall_s=time.monotonic()-start,
                                diagnostics=list(env.adapter.diagnostics),references=list(env.adapter.references)))
                    env._pump=inspect_pump
                    transition=env.step(action)
                    assert env.sensor==sensor and env.reference.reference_id==reference
                    assert not env.plant.collision
                    row.update(status='passed',progress=env.progress(),wall_s=time.monotonic()-start)
                except Exception as exc:
                    row.update(status='failed',error=f'{type(exc).__name__}: {exc}',wall_s=time.monotonic()-start)
                    if not env._closed:row['progress']=env.progress()
                finally:env.close()
                rows.append(row)
                write(args.output/'native-preflight.json',dict(actor_sha=sha(args.actor),rows=rows,
                    status='running' if len(rows)<min(args.count,24-args.start) else ('passed' if all(r['status']=='passed' for r in rows) else 'failed')))
                print(json.dumps({k:v for k,v in row.items() if k not in ('progress','case')},ensure_ascii=False),flush=True)
                if row['status']=='failed':raise RuntimeError(row['error'])


if __name__=='__main__':main()
