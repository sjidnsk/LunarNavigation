"""实际推理状态机的纯生命周期回归：不导入ROS、Actor或训练模块。"""
import ast
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as S
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def runtime_class(monkeypatch):
    source=(ROOT/'ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/runtime.py').read_text(encoding='utf-8')
    node=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='InferenceRuntime')
    module=ast.Module(body=[node],type_ignores=[])
    prep=ModuleType('lunar_drl_exploration.policy_preparation')
    prep.prepare_policy_state=lambda observation,**kw:(observation,None,None,None)
    monkeypatch.setitem(sys.modules,prep.__name__,prep)
    class Core:
        task=S(task_id='test')
        def consume(self,snap):pass
        def record_observation(self,*args):pass
        def observe(self,*args):return S(goals=np.zeros((1,3))),S(available=True,completed=False)
    ns=dict(__package__='lunar_drl_exploration',np=np,SensorSpec=lambda:S(),DecisionCore=lambda *args:Core(),
        velocity_is_stopped=lambda velocity:all(abs(v)<=.01 for v in velocity),InfrastructureError=RuntimeError)
    exec(compile(ast.fix_missing_locations(module),'actual-inference-runtime','exec'),ns)
    return ns['InferenceRuntime']


class Adapter:
    def __init__(self):
        self.available_stamp=100;self.processed_stamp_ns=100;self.pose=S();self.velocity=(0.,0.)
        self.inflight=False;self.result=None;self.minimums=[];self.fresh_requests=0
    def require_fresh_snapshot(self):self.fresh_requests+=1
    def poll_snapshot(self,minimum_map_stamp_ns=0):
        self.minimums.append(minimum_map_stamp_ns)
        if self.available_stamp<minimum_map_stamp_ns:return None
        self.processed_stamp_ns=self.available_stamp
        return S(tiles={'tile':S(observed=np.ones((1,1),bool))})
    def begin_goal(self,goal):self.inflight=True
    def cancel_goal(self):pass
    def poll_goal(self):
        result=self.result
        if result is not None:self.result=None;self.inflight=False
        return result


def start(monkeypatch):
    adapter=Adapter();decisions=[]
    runtime=runtime_class(monkeypatch)(adapter,lambda observation:decisions.append(adapter.processed_stamp_ns) or 0)
    runtime.start(S());runtime.tick()
    assert decisions==[100] and adapter.inflight
    return runtime,adapter,decisions


def test_completion_waits_for_new_frame_not_just_new_response(monkeypatch):
    runtime,adapter,decisions=start(monkeypatch)
    adapter.result=S(outcome=0,reason_code='GOAL_REACHED')
    runtime.tick()
    assert decisions==[100] and not adapter.inflight
    assert runtime.state=='WAITING_FOR_INPUT'
    for _ in range(3):runtime.tick()
    assert decisions==[100] and adapter.minimums[-1]==101
    adapter.available_stamp=101;runtime.tick()
    assert decisions==[100,101] and adapter.inflight


def test_cancel_completes_without_waiting_for_a_new_map_frame(monkeypatch):
    runtime,adapter,decisions=start(monkeypatch)
    runtime.cancel();adapter.result=S(outcome=5,reason_code='CANCELED_STOPPED')
    runtime.tick()
    assert runtime.state=='CANCELED' and not adapter.inflight
    assert decisions==[100]


def test_resume_requires_new_frame_and_keeps_stop_lifecycle_independent(monkeypatch):
    runtime,adapter,decisions=start(monkeypatch)
    runtime.pause();adapter.result=S(outcome=5,reason_code='CANCELED_STOPPED')
    runtime.tick();assert runtime.state=='PAUSED'
    runtime.resume();runtime.tick()
    assert decisions==[100] and not adapter.inflight
    adapter.available_stamp=101;runtime.tick()
    assert decisions==[100,101]