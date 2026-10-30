"""隔离推理验证守卫：禁止训练模块与优化器/Critic构造。"""
import importlib.abc
import importlib.machinery
import json
import os
from pathlib import Path
import sys
import time

DENIED=tuple('lunar_drl_exploration.'+name for name in ('training','sac','collector','replay','ros_env'))
PATCHED=('torch.optim.optimizer','lunar_drl_exploration.model')


class GuardLoader(importlib.abc.Loader):
    def __init__(self,original,guard,name):self.original=original;self.guard=guard;self.name=name
    def create_module(self,spec):
        return self.original.create_module(spec) if hasattr(self.original,'create_module') else None
    def exec_module(self,module):
        self.original.exec_module(module);self.guard.patch_module(self.name,module)
    def __getattr__(self,name):return getattr(self.original,name)


class InferenceGuard(importlib.abc.MetaPathFinder):
    def __init__(self,log_dir):
        self.log_dir=Path(log_dir);self.log_dir.mkdir(parents=True,exist_ok=True)
    def record(self,kind,detail):
        with (self.log_dir/('guard-'+str(os.getpid())+'.jsonl')).open('a',encoding='utf-8') as f:
            f.write(json.dumps(dict(kind=kind,detail=detail,pid=os.getpid(),wall=time.monotonic()))+'\n')
    def find_spec(self,fullname,path,target=None):
        if any(fullname==name or fullname.startswith(name+'.') for name in DENIED):
            self.record('blocked_import',fullname)
            raise RuntimeError('inference-only guard blocked '+fullname)
        if fullname in PATCHED:
            spec=importlib.machinery.PathFinder.find_spec(fullname,path)
            if spec is not None and spec.loader is not None:
                spec.loader=GuardLoader(spec.loader,self,fullname)
            return spec
        return None
    def patch_module(self,name,module):
        cls=getattr(module,'Optimizer' if name=='torch.optim.optimizer' else 'Critic',None)
        if cls is None:return
        guard=self
        def forbidden(*args,**kwargs):
            guard.record('blocked_constructor',name)
            raise RuntimeError('inference-only guard blocked constructor '+name)
        cls.__init__=forbidden
        self.record('constructor_guard',name)


def install(log_dir):
    for finder in sys.meta_path:
        if isinstance(finder,InferenceGuard):return finder
    guard=InferenceGuard(log_dir)
    for name in DENIED:
        if name in sys.modules:
            guard.record('preloaded_training_module',name)
            raise RuntimeError('inference-only guard installed too late: '+name)
    sys.meta_path.insert(0,guard)
    for name in PATCHED:
        if name in sys.modules:guard.patch_module(name,sys.modules[name])
    starttime=Path('/proc/self/stat').read_text().split()[21] if Path('/proc/self/stat').is_file() else 'windows'
    guard.record('startup',dict(executable=sys.executable,argv=sys.argv,starttime=starttime))
    return guard
