"""Stage training entry and spawn-importable measured-capture environment factory."""
import os
from pathlib import Path


def smoke_snapshot(path,event,*,cuda_enabled=False,**fields):
    """Read actual learner allocator/RNG without consuming any random numbers."""
    import hashlib,json,time,torch
    def sha(value):return hashlib.sha256(value.cpu().numpy().tobytes()).hexdigest()
    row=dict(event=event,pid=os.getpid(),ts=time.time(),torch_cpu_rng_sha256=sha(torch.get_rng_state()),
        torch_cuda_rng_sha256=[sha(value) for value in torch.cuda.get_rng_state_all()] if cuda_enabled and torch.cuda.is_initialized() else [],**fields)
    if cuda_enabled and torch.cuda.is_initialized():
        row['cuda']=dict(device=torch.cuda.current_device(),allocated_bytes=torch.cuda.memory_allocated(),
            reserved_bytes=torch.cuda.memory_reserved(),peak_allocated_bytes=torch.cuda.max_memory_allocated(),
            peak_reserved_bytes=torch.cuda.max_memory_reserved())
    with Path(path).open('a',encoding='utf-8') as stream:stream.write(json.dumps(row,allow_nan=False)+'\n')


def install_smoke_telemetry():
    """Opt-in diagnostics only in this learner; return an undo for scoped tests."""
    path=os.environ.get('DRL_SMOKE_TELEMETRY')
    if not path:return None
    import hashlib,torch
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.checkpoint import TrainingState,CheckpointManager
    methods=[(SACLearner,'__init__'),(SACLearner,'update'),(TrainingState,'restore'),(CheckpointManager,'save')]
    original={key:getattr(*key) for key in methods}
    def init(self,*args,**kwargs):
        if str(kwargs.get('device','cpu')).startswith('cuda'):
            torch.cuda.init();torch.cuda.reset_peak_memory_stats()
        result=original[(SACLearner,'__init__')](self,*args,**kwargs)
        smoke_snapshot(path,'model-created',cuda_enabled=self.device.type=='cuda',updates=self.updates,model=vars(self.model_config),learning=vars(self.learning_config))
        return result
    def update(self,*args,**kwargs):
        result=original[(SACLearner,'update')](self,*args,**kwargs)
        smoke_snapshot(path,'update',cuda_enabled=self.device.type=='cuda',updates=self.updates)
        return result
    def restore(self,*args,**kwargs):
        result=original[(TrainingState,'restore')](self,*args,**kwargs)
        rng=self.record['rng']
        sha=lambda value:hashlib.sha256(value.cpu().numpy().tobytes()).hexdigest()
        smoke_snapshot(path,'restored',cuda_enabled=(args[0] if args else kwargs['learner']).device.type=='cuda',updates=self.record['schedule']['updates'],transitions=self.record['schedule']['transitions'],
            saved_torch_cpu_rng_sha256=sha(rng['torch_cpu']),saved_torch_cuda_rng_sha256=[sha(value) for value in rng['torch_cuda']])
        return result
    def save(self,state,*args,**kwargs):
        fields={key:state.record['schedule'][key] for key in ('updates','transitions')}
        fields['cuda_enabled']=bool(state.record['rng']['torch_cuda'])
        smoke_snapshot(path,'save-before',**fields)
        result=original[(CheckpointManager,'save')](self,state,*args,**kwargs)
        smoke_snapshot(path,'save-after',**fields)
        return result
    for (cls,name),replacement in zip(methods,(init,update,restore,save)):setattr(cls,name,replacement)
    def undo():
        for (cls,name),value in original.items():setattr(cls,name,value)
    return undo


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
    install_smoke_telemetry()
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
