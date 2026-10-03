"""One frozen case in the selected wrapper; adds telemetry, never changes old loaders."""
import argparse
from dataclasses import replace
import importlib
import os
from pathlib import Path
import platform
import threading
import time
from controller import CASE_KEYS, PAIR_KEYS, read, write, sha, digest, validate_evaluation


def runtime_identity():
    modules={}
    for name in ('lunar_drl_exploration.evaluation','lunar_drl_exploration.ros_env',
                 'lunar_drl_exploration.reference','lunar_drl_terrain_native'):
        module=importlib.import_module(name);path=Path(module.__file__).resolve()
        modules[name]=dict(path=str(path),sha=sha(path))
    package=Path(importlib.import_module('lunar_drl_exploration.evaluation').__file__).resolve().parent
    from ament_index_python.packages import get_package_prefix
    native={}
    for package_name,executable in (('lunar_incremental_navigation_ros','lunar_incremental_navigation_node'),
                                    ('lunar_pure_wheeled_controller','lunar_pure_wheeled_controller_node.py')):
        path=Path(get_package_prefix(package_name))/'lib'/package_name/executable
        native[package_name]=dict(path=str(path),sha=sha(path))
    import numpy, scipy
    return dict(modules=modules,navigation=native,package_sha=digest([(path.name,sha(path)) for path in sorted(package.glob('*.py'))]), python=platform.python_version(), numpy=numpy.__version__,
                scipy=scipy.__version__,entry_sha=sha(__file__))


def effective_identity(config):
    from lunar_drl_exploration.config import config_record, resume_semantics
    record=config_record(config)
    platform_config=dict(record['platform']);platform_config.pop('capability_path',None)
    pair=dict(sensor=record['sensor'],platform=platform_config,
        integration_step_s=config.integration_step_s,observation_hz=config.observation_hz,
        goal_position_tolerance_m=config.goal_position_tolerance_m,
        success_coverage=config.success_coverage,generator_version=5)
    return dict(config_digest=digest(record),semantic_id=digest(resume_semantics(config)),
                pairing_config_digest=digest(pair))


def validate_cached(record,request,current):
    row=record['cases'][0]
    proof=record['provenance']
    expected_proof=dict(current,case={k:request[k] for k in CASE_KEYS},actor_sha=request['actor_sha'],
                        config_sha=request['config_sha'],wrapper_sha=sha(request['wrapper']),
                        source_commit=request['source_commit'],run_id=request['run_id'])
    expected_proof.update({key:request.get(key) for key in ('source_checkpoint','source_checkpoint_sha')})
    for key,value in expected_proof.items():
        if proof.get(key)!=value:raise ValueError('cached provenance changed: '+key)
    if row.get('provenance_digest')!=digest(proof):raise ValueError('cached provenance digest')
    for key in (*PAIR_KEYS,'config_digest','semantic_id','actor_sha','run_id','pid'):
        if row.get(key)!=proof.get(key):raise ValueError('cached row/provenance '+key)
    expected=dict(request,config_digest=current['config_digest'],**{key:proof[key] for key in PAIR_KEYS})
    if request.get('paired_with'):expected.update(request['paired_with'])
    validate_evaluation(record,actor_sha=request['actor_sha'],semantic_id=current['semantic_id'],cases=[expected])


class Telemetry:
    def __init__(self,root,request):
        self.path=Path(root)/'eval-progress.json'
        self.state=dict(pid=os.getpid(),run_id=request['run_id'],case_index=request['case_index'],
            coverage=None,step=0,zero=0,phase='加载模型')
        self.started=self.action_started=time.monotonic();self.closed=threading.Event()
        self.thread=None
    def update(self,**values):
        self.state=dict(self.state,**values);self.action_started=time.monotonic();self.emit()
    def emit(self):
        write(self.path,dict(self.state,updated_unix=time.time(),episode_wall_s=time.monotonic()-self.started,
                             action_wall_s=time.monotonic()-self.action_started))
    def start(self):
        def loop():
            while not self.closed.wait(1.):self.emit()
        self.thread=threading.Thread(target=loop,daemon=True);self.thread.start()
    def close(self):
        self.closed.set()
        if self.thread:self.thread.join()


class DomainMetrics:
    def __init__(self):
        self.outside_steps=self.outside_run=self.max_outside_run=self.exits=self.returns=0
        self.previous=None
    def observe(self,inside):
        if self.previous is not None:
            self.exits+=int(self.previous and not inside)
            self.returns+=int(not self.previous and inside)
        self.previous=inside
        self.outside_steps+=int(not inside)
        self.outside_run=0 if inside else self.outside_run+1
        self.max_outside_run=max(self.max_outside_run,self.outside_run)
    def record(self):
        return dict(outside_steps=self.outside_steps,max_outside_run=self.max_outside_run,
                    task_exits=self.exits,task_returns=self.returns)


def evaluate_case(request,output):
    from lunar_drl_exploration import ros_env
    from lunar_drl_exploration.cli import load_config, parser
    from lunar_drl_exploration.evaluation import evaluate
    from lunar_drl_exploration.config import config_record
    from lunar_drl_exploration.graph import _inside
    config=load_config(parser().parse_args(['evaluate','--config',request['config'],'--actor',request['actor']]))
    # Explicitly disable the collection-only limiter in the effective evaluation record.
    config=replace(config,stagnation_window_steps=0)
    if sha(request['actor'])!=request['actor_sha'] or sha(request['config'])!=request['config_sha']:
        raise ValueError('actor/config changed before evaluation')
    current=dict(effective_identity(config),runtime=runtime_identity())
    if Path(output).exists():
        cached=read(output)
        validate_cached(cached,request,current)
        return cached
    proof=dict(current,case={k:request[k] for k in CASE_KEYS},actor_sha=request['actor_sha'],
        config_sha=request['config_sha'],wrapper_sha=sha(request['wrapper']),source_commit=request['source_commit'],
        run_id=request['run_id'],pid=os.getpid(),source_checkpoint=request.get('source_checkpoint'),source_checkpoint_sha=request.get('source_checkpoint_sha'))
    telemetry=Telemetry(request['root'],request);domain=DomainMetrics();capture=None
    probe_root=Path(request['root'])/'fixed-probes'
    if not request['legacy'] and request['case_index']<2 and not (probe_root/'index.json').exists():
        from fixed_probes import ProbeCapture
        capture=ProbeCapture(probe_root,dict(proof,phase='evaluation'));capture.install()
    base=ros_env.RosExplorationEnv
    class ObservedEnv(base):
        def reset_scene(self,terrain,pose,task,*,descriptor=None):
            self.probe_initial_pose=[pose.x,pose.y,pose.yaw]
            return super().reset_scene(terrain,pose,task,descriptor=descriptor)
        def reset(self,seed,family,extent,*,episode_budget=None):
            telemetry.update(phase='初始化地图')
            obs,state=super().reset(seed,family,extent,episode_budget=episode_budget)
            proof.update(terrain_id=self.terrain.terrain_id,reference_id=self.reference.reference_id,
                actual_schema=obs.schema,episode_metadata=dict(self.episode_metadata),
                polygon_digest=digest(self.task.polygon.tolist()),initial_pose=self.probe_initial_pose,
                sensor=config_record(self.config.sensor),platform=config_record(self.config.platform))
            proof['pairing_digest']=digest(dict(config_pair=current['pairing_config_digest'],polygon_digest=proof['polygon_digest'],
                                               initial_pose=proof['initial_pose']))
            domain.previous=bool(_inside(obs.positions[obs.current_index:obs.current_index+1],obs.polygon)[0])
            if capture:capture.bind(self,obs,state,0)
            telemetry.update(phase='准备决策',coverage=self.reference.coverage_ratio(state.observed))
            return obs,state
        def step(self,action):
            telemetry.update(phase='执行动作')
            transition=super().step(action)
            obs=transition.next_observation
            domain.observe(bool(_inside(obs.positions[obs.current_index:obs.current_index+1],obs.polygon)[0]))
            step=telemetry.state['step']+1
            if capture:capture.bind(self,obs,transition.next_privileged,step)
            telemetry.update(phase='准备决策',step=step,coverage=self.reference.coverage_ratio(transition.next_privileged.observed),
                             zero=telemetry.state['zero']+1 if transition.parts.new_area_m2<=0 else 0)
            return transition
    ros_env.RosExplorationEnv=ObservedEnv
    telemetry.start()
    try:
        original=evaluate(config,request['actor'],seeds=[request['seed']],families=[request['family']],
            extents=[request['extent_m']],budget=request['budget'],output=None,domain_base=220)
    finally:
        telemetry.close();ros_env.RosExplorationEnv=base
        if capture:capture.close()
    if sha(request['actor'])!=request['actor_sha'] or sha(request['config'])!=request['config_sha']:
        raise ValueError('actor/config changed during evaluation')
    if runtime_identity()!=current['runtime']:raise ValueError('runtime modules changed during evaluation')
    row=dict(original['cases'][0],**domain.record(),**{k:request[k] for k in CASE_KEYS})
    for key in (*PAIR_KEYS,'config_digest','semantic_id','actor_sha','run_id','pid'):row[key]=proof.get(key)
    row.update(provenance_digest=digest(proof),exit_code=0)
    result=dict(status='finished',exit_code=0,actor_sha=request['actor_sha'],semantic_id=current['semantic_id'],
                cases=[row],provenance=proof,original=original)
    validate_cached(result,request,current)
    write(output,result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();evaluate_case(read(args.request),args.output)


if __name__=='__main__':main()
