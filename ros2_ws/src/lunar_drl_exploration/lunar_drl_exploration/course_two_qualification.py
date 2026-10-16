"""Restartable native initialization and measured predecessor qualification."""
from dataclasses import asdict,replace
from pathlib import Path
import signal
import time
import numpy as np
from .course_two_control import read,write,runtime_digest,sha
from .course_two_parallel import validate_slot


def spec_for(case):
    return dict(seed=case['seed'],family=case['family'],extent=case['roi_width_m'],
        start_mode='outside' if case.get('role','approach')=='approach' else 'inside',
        curriculum_task=case['curriculum_task'],episode_budget=case['episode_budget'],course_two_case=case)


def qualify(config_path,request,output,*,slot=0,original=False,progress=None):
    from .config import training_config_from_record
    from .ros_env import RosExplorationEnv
    from .course_two_rebalanced_scene import make_case,build_case
    from .course_two_scene import _identity
    from .course_two_prelude import prepare_start,inside_task
    from .ordered_curriculum import area_budget
    validate_slot(slot)
    output=Path(output);progress=Path(progress or str(output)+'.progress.json')
    proof=dict(runtime=runtime_digest(),config_sha=sha(config_path),request=request,original=original)
    if output.exists():
        cached=read(output)
        if cached.get('proof')!=proof:raise ValueError('qualification cache identity changed')
        if cached.get('status')!='passed':raise ValueError('qualification previously exhausted attempts')
        return cached
    config=replace(training_config_from_record(read(config_path)),environments=1)
    env=RosExplorationEnv(config,slot,domain_base=200,wall_timeout_s=120.)
    attempts=[];start=time.monotonic()
    def interrupted(*_):raise KeyboardInterrupt('qualification interrupted')
    previous={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        for attempt in range(request.get('max_geometry_attempts',100)):
            began=time.monotonic()
            def live(record):
                write(progress,dict(record,request_id=request['request_id'],role=request['role'],
                    attempt=attempt,slot=slot,updated_unix=time.time()))
                if time.monotonic()-began>1200:raise ValueError('prelude qualification exceeded 1200 seconds')
            live(dict(phase='reset'))
            try:
                if request['role']=='old':
                    rng=np.random.default_rng(np.random.SeedSequence([request['seed'],attempt]))
                    extent=float(rng.uniform(30,100))
                    spec=dict(seed=request['seed']+(attempt<<48),family=request['family'],extent=extent,
                        start_mode=request['planned_policy_start'],curriculum_task=request['old_task'],
                        episode_budget=area_budget(extent,config.ordered_budget_rule))
                    if request.get('task_scope') and request['family']=='cave':spec['task_scope']=request['task_scope']
                    env.reset(**spec)
                    if env.completed:raise ValueError('old scene completed during initialization')
                    if inside_task(env)!=(spec['start_mode']=='inside'):raise ValueError('old native start domain mismatch')
                elif original:
                    from .course_two_scene import make_case as original_case
                    case=original_case(request['seed']+(attempt<<48),'A1',request['request_ordinal'],
                        range_m=10.,fov_deg=90.,family=request['family'])
                    spec=spec_for(case);env.reset(**spec)
                    if env.completed:raise ValueError('original scene initially completed')
                else:
                    case=make_case(request,attempt=attempt)
                    terrain,pose,task,sensor,descriptor=build_case(case,config.platform)
                    env.episode_budget=case['episode_budget']
                    env.reset_scene(terrain,pose,task,sensor=sensor,descriptor=descriptor)
                    prelude=prepare_start(env,case,construct=True,progress=live)
                    if prelude['goals']:
                        case['prelude_goals']=prelude['goals'];case.pop('scenario_id')
                        case['scenario_id']=_identity(case)
                    spec=spec_for(case)
                if asdict(env.sensor)!=dict(asdict(config.sensor) if request['role']=='old' else spec['course_two_case']['sensor']):
                    raise ValueError('scene sensor changed')
                evidence=dict(coverage=env.reference.coverage_ratio(env.privileged.observed),inside=inside_task(env),
                    initial_pose=env.episode_metadata['initial_pose'],reference_id=env.reference.reference_id,
                    terrain_id=env.terrain.terrain_id,reference_area_m2=env.reference.area_m2,
                    sensor=asdict(env.sensor),metadata=env.episode_metadata)
                completion=None
                if not original and request['role'] in ('finish','return'):
                    from .course_two_prelude import certify_completion
                    completion=certify_completion(env,progress=live,
                        return_limit=10. if spec['course_two_case']['role']=='return' else None)
                    # Independently replay the frozen prefix before acceptance.
                    env.reset(**spec)
                    evidence.update(coverage=env.reference.coverage_ratio(env.privileged.observed),
                        inside=inside_task(env),initial_pose=env.episode_metadata['initial_pose'],
                        reference_id=env.reference.reference_id,metadata=env.episode_metadata)
                result=dict(status='passed',proof=proof,spec=spec,evidence=evidence,rejections=attempts,
                    completion_certificate=completion,
                    wall_s=time.monotonic()-start)
                write(output,result);live(dict(phase='passed',coverage=evidence['coverage']))
                return result
            except (ValueError,RuntimeError) as exc:
                attempts.append(dict(attempt=attempt,error=f'{type(exc).__name__}: {exc}',wall_s=time.monotonic()-began))
                write(str(output)+'.attempts.json',dict(proof=proof,rejections=attempts))
            finally:env.close()
        write(output,dict(status='failed',proof=proof,rejections=attempts,wall_s=time.monotonic()-start))
        raise ValueError('native qualification exhausted geometry attempts')
    finally:
        env.close()
        for sig,handler in previous.items():signal.signal(sig,handler)


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True)
    p.add_argument('--request',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--slot',type=int,default=0);p.add_argument('--original',action='store_true')
    a=p.parse_args();qualify(a.config,read(a.request),a.output,slot=a.slot,original=a.original)


if __name__=='__main__':main()
