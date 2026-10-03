"""Stage training entry and spawn-importable measured-capture environment factory."""
import os
from pathlib import Path


def captured_env(config,env_id,**kwargs):
    """Called inside each real worker via training's existing env_factory contract."""
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    root=Path(os.environ['DRL_ROOT'])/'fixed-probes'
    if env_id != 0 or (root/'index.json').exists():return RosExplorationEnv(config,env_id,**kwargs)
    from fixed_probes import ProbeCapture
    from eval_case import runtime_identity,effective_identity
    from controller import source_commit
    from lunar_drl_exploration.config import config_record
    capture=ProbeCapture(root,dict(run_id=os.environ['DRL_RUN_ID'],phase='warmup',worker=env_id,
        pid=os.getpid(),source_commit=source_commit(),effective_config=config_record(config),**effective_identity(config),runtime=runtime_identity()))
    capture.install()
    class CapturedEnv(RosExplorationEnv):
        def reset_scene(self,terrain,pose,task,*,descriptor=None):
            self.probe_initial_pose=[pose.x,pose.y,pose.yaw]
            return super().reset_scene(terrain,pose,task,descriptor=descriptor)
        def reset(self,*args,**kwargs):
            capture.pending.clear()
            capture.enabled=not (root/'index.json').exists() and len(list(root.glob('probe-*.json')))<6
            obs,state=super().reset(*args,**kwargs)
            if capture.enabled:capture.bind(self,obs,state,0)
            return obs,state
        def step(self,*args,**kwargs):
            capture.enabled=not (root/'index.json').exists() and len(list(root.glob('probe-*.json')))<6
            transition=super().step(*args,**kwargs)
            if capture.enabled:capture.bind(self,transition.next_observation,transition.next_privileged,self.steps)
            return transition
    return CapturedEnv(config,env_id,**kwargs)


def main(argv=None):
    from lunar_drl_exploration.worker import numerical_threads
    numerical_threads(4)
    from lunar_drl_exploration.cli import parser,load_config
    from lunar_drl_exploration.training import run_training
    args=parser().parse_args(argv)
    if args.command!='train':raise ValueError('training entry only')
    if args.probe and args.max_transitions is None:raise ValueError('probe requires finite budget')
    config=load_config(args)
    from lunar_drl_exploration import terminal
    original_render=terminal.TrainingTerminal.render
    stage=int(os.environ.get('DRL_STAGE','0'))
    def render(self,record,**kwargs):
        import json
        print('TASK_FRONTIER_PROGRESS '+json.dumps({key:record[key] for key in ('transitions','updates','credit')},allow_nan=False),flush=True)
        return original_render(self,record,**kwargs)
    terminal.TrainingTerminal.render=render
    original_status=terminal.format_status
    terminal.format_status=lambda record,**kwargs: terminal.format_stage_status(record,stage=stage)+original_status(record,**kwargs)
    return run_training(config,resume=args.resume,fork_from=args.fork_from,fork_experiment=args.fork_experiment,
        device=args.device,max_transitions=args.max_transitions,domain_base=args.domain_base,
        probe_extent=args.probe_extent,probe_budget=args.probe_budget,env_factory='train_live:captured_env')


if __name__=='__main__':main()
