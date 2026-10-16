#!/usr/bin/env python3
"""Qualify post-initialization evaluation classes in isolated native ROS workers."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
import multiprocessing
from pathlib import Path
import time

from lunar_drl_exploration.course_two_control import read, write, sha, runtime_digest, validation_cases


def qualify(root,output,slot,cases):
    import torch
    from lunar_drl_exploration.config import training_config_from_record
    from lunar_drl_exploration.course_two_eval import validate_policy_start
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.runtime import ActorPolicy
    torch.set_num_threads(1)
    config=replace(training_config_from_record(read(root/'config.json')),environments=1)
    actor=ActorPolicy.load(root/'A1-block4.pt')
    rows=[]
    for index,case in enumerate(cases):
        spec=case['case'];started=time.time()
        env=RosExplorationEnv(config,slot,domain_base=220)
        row=dict(case_id=case['case_id'],kind=case['kind'],declared_class=spec['visibility_class'],
            slot=slot,domain_id=220+slot,started_unix=started,status='running')
        try:
            env.reset(spec['seed'],spec['family'],spec['roi_width_m'],episode_budget=spec['episode_budget'],
                start_mode='outside',curriculum_task=spec['curriculum_task'],course_two_case=spec)
            initial=env.reference.coverage_ratio(env.privileged.observed)
            row.update(initial_coverage=initial,initialization=dict(env.episode_metadata))
            validate_policy_start(case,initial)
            if asdict(env.sensor)!=spec['sensor']:raise ValueError('sensor changed during initialization')
            if index==0:
                env.step(actor(env.observation))
                if env.plant.collision:raise ValueError('native first action collision')
                row.update(first_action_passed=True,first_action_coverage=env.reference.coverage_ratio(env.privileged.observed))
            row['status']='passed'
        except Exception as exc:
            row.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        finally:env.close()
        row.update(finished_unix=time.time(),wall_s=time.time()-started)
        write(output/(case['case_id']+'.json'),row);rows.append(row)
        if row['status']=='failed':break
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args()
    if not 1<=args.workers<=8:parser.error('workers must be in 1..8')
    args.output.mkdir(parents=True,exist_ok=False)
    cases=[c for c in validation_cases('A1') if c['kind'] in ('new','visible')]
    rows=[];started=time.time()
    record=dict(status='running',runtime=runtime_digest(),actor_sha=sha(args.root/'A1-block4.pt'),
        config_sha=sha(args.root/'config.json'),cases=cases,workers=args.workers,started_unix=started)
    write(args.output/'summary.json',dict(record,rows=rows))
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn')) as pool:
        futures=[pool.submit(qualify,args.root,args.output,slot,cases[slot::args.workers]) for slot in range(args.workers)]
        for future in as_completed(futures):
            rows.extend(future.result())
            write(args.output/'summary.json',dict(record,rows=rows))
            print(f'qualified {len(rows)}/{len(cases)}; errors {sum(r["status"]!="passed" for r in rows)}',flush=True)
    passed=len(rows)==len(cases) and all(r['status']=='passed' for r in rows)
    write(args.output/'summary.json',dict(record,rows=rows,status='passed' if passed else 'failed',
        wall_s=time.time()-started,first_actions=sum(bool(r.get('first_action_passed')) for r in rows)))
    if not passed:raise RuntimeError('evaluation preflight failed; inspect per-case evidence')


if __name__=='__main__':main()
