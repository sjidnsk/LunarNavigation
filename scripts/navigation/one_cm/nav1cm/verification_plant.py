"""验证专用独立车辆时钟；计划仅含控制，实际状态永不复制规划末态。"""
from dataclasses import dataclass, asdict
from types import SimpleNamespace
import multiprocessing as mp
import queue
import math
from time import perf_counter, sleep
import numpy as np
from .contracts import VehicleState
from .dynamics import step
from .seeds import brake_candidate


@dataclass(frozen=True)
class TimedPlan:
    sequence: int
    source_feedback_s: float
    valid_start_s: float
    times: tuple
    controls: tuple
    epoch: int
    route_version: int
    mode: str
    source: str
    predicted_initial: tuple = ()
    source_feedback_id: int = -1
    source_plan_sequence: int = -1
    source_vector: tuple = ()

    @property
    def end_s(self): return self.valid_start_s+self.times[-1]


class PlantModel:
    def __init__(self,state,limits):
        self.vector=np.array([state.x,state.y,state.yaw,state.v,state.w,0.])
        self.limits=limits
        self.sample_s=0.
        self.active=self.pending=None
        self.sequence=0
        self.target_rejected=False
        self.last_source='brake'
        self.last_segments=[]
        self.handover_error=None
        self.protocol_failures=[]
        self._feedback_id=0
        self._published={}

    def _protocol_reject(self,reason,plan):
        self.protocol_failures.append(dict(reason=reason,sequence=plan.sequence,sample_s=self.sample_s))
        return reason

    def accept(self,plan,received_s):
        if self.target_rejected: return 'TARGET_REJECTED'
        if plan.valid_start_s <= received_s+1e-9: return 'LATE_DELIVERY'
        if plan.source_feedback_s>self.sample_s+1e-9 or plan.sequence<=self.sequence: return 'STALE_SOURCE'
        if self.pending is not None: return 'PENDING_FULL'
        published=self._published.get(plan.source_feedback_id)
        if (published is None or published!=(plan.source_feedback_s,tuple(plan.source_vector),plan.source_plan_sequence) or plan.source_plan_sequence!=self.sequence):
            return self._protocol_reject('SOURCE_IDENTITY_MISMATCH',plan)
        if received_s-plan.source_feedback_s>.15+1e-9:
            return self._protocol_reject('SOURCE_EXPIRED',plan)
        if len(plan.times)!=len(plan.controls)+1 or len(plan.times)<2 or plan.times[0]!=0 or any(b<=a for a,b in zip(plan.times,plan.times[1:])):
            return 'INVALID_PLAN'
        if np.asarray(plan.controls).shape!=(len(plan.times)-1,3): return 'INVALID_PLAN'
        if not np.isfinite([plan.valid_start_s,plan.source_feedback_s,*plan.times]).all() or not np.isfinite(plan.controls).all(): return 'INVALID_PLAN'
        expected=predict_feedback(self._snapshot(),plan.valid_start_s,self.limits)[:5]
        if np.asarray(plan.predicted_initial).shape!=(5,) or not np.allclose(expected,plan.predicted_initial,atol=1e-6,rtol=0):
            return self._protocol_reject('PREDICTION_MISMATCH',plan)
        self.pending=plan
        self.sequence=plan.sequence
        return 'ACCEPTED'

    def reject_target(self):
        self.target_rejected=True
        self.active=self.pending=None

    def advance(self,duration):
        end=self.sample_s+duration
        self.last_segments=[]
        while self.sample_s<end-1e-10:
            if self.pending and self.pending.valid_start_s<=self.sample_s+1e-9:
                self.active,self.pending=self.pending,None
                self.handover_error=(dict(sequence=self.active.sequence,sample_s=self.sample_s,
                    difference=(self.vector[:5]-self.active.predicted_initial).tolist()) if self.active.predicted_initial else None)
                if self.handover_error and not np.allclose(self.vector[:5],self.active.predicted_initial,atol=1e-6,rtol=0):
                    self._protocol_reject('HANDOVER_MISMATCH',self.active)
                    self.active=None
            if self.active and self.active.end_s<=self.sample_s+1e-9: self.active=None
            slot=(math.floor((self.sample_s+1e-9)/.05)+1)*.05
            boundary=min(end,slot,self.pending.valid_start_s if self.pending else end)
            if self.active:
                plan=self.active
                i=int(np.searchsorted(plan.times,self.sample_s-plan.valid_start_s+1e-9,side='right')-1)
                control=np.array(plan.controls[i])
                boundary=min(boundary,plan.valid_start_s+plan.times[i+1])
                source,epoch,version,mode=plan.source,plan.epoch,plan.route_version,plan.mode
            else:
                state=VehicleState(*self.vector[:5])
                brake=brake_candidate(state,SimpleNamespace(epoch=0),SimpleNamespace(route_version=0,progress_s=self.vector[5]),self.limits)
                control=brake.controls[0] if len(brake.controls) else np.zeros(3)
                if len(brake.controls): boundary=min(boundary,self.sample_s+brake.times[1])
                source,epoch,version,mode='brake',0,0,brake.mode
            dt=boundary-self.sample_s
            before=self.vector.copy()
            self.vector=step(before,control,dt)
            self.last_segments.append(dict(start_s=self.sample_s,end_s=boundary,before=before.tolist(),after=self.vector.tolist(),control=control.tolist(),source=source,epoch=epoch,route_version=version,mode=mode))
            self.sample_s=boundary
            self.last_source=source

    def _snapshot(self):
        return dict(sample_s=self.sample_s,vector=self.vector.tolist(),active=asdict(self.active) if self.active else None,pending=asdict(self.pending) if self.pending else None,sequence=self.sequence,target_rejected=self.target_rejected,handover_error=self.handover_error,protocol_failures=list(self.protocol_failures))

    def feedback(self):
        self._feedback_id+=1
        result=dict(feedback_id=self._feedback_id,**self._snapshot())
        self._published[self._feedback_id]=(self.sample_s,tuple(self.vector),self.sequence)
        # 仅保留最近8次实际发布，明确限制来源历史，不保存无限反馈队列。
        if len(self._published)>8: del self._published[min(self._published)]
        return result


def predict_feedback(feedback,target_s,limits):
    if target_s<feedback['sample_s']: raise ValueError('prediction before sample')
    model=PlantModel(VehicleState(*feedback['vector'][:5]),limits)
    model.vector=np.array(feedback['vector']); model.sample_s=feedback['sample_s']
    model.active=TimedPlan(**feedback['active']) if feedback['active'] else None
    model.pending=TimedPlan(**feedback['pending']) if feedback['pending'] else None
    model.advance(target_s-model.sample_s)
    return model.vector


def prepare_handover(feedback,now_s,limits,last_target_s=-1.,lead_s=.1,slot_s=.05):
    """纯父端预测；执行tick不受pending发布门禁影响，发布方另行检查。"""
    if now_s<feedback['sample_s']-1e-9: raise ValueError('clock before measured sample')
    if lead_s<=0 or slot_s<=0: raise ValueError('positive lead and slot required')
    target=math.ceil((now_s+lead_s)/slot_s-1e-9)*slot_s
    if target<=last_target_s+1e-9: return None
    predicted=predict_feedback(feedback,target,limits)
    return dict(source_feedback_s=feedback['sample_s'],prediction_target_s=target,
                state=VehicleState(*predicted[:5]),predicted_vector=predicted)


def plan_from_candidate(candidate,sequence,source_feedback_s,target_s,hold_s=.05,*,feedback=None):
    """已认证剩余段转消息；静止空控制明确发有限hold，到期由plant制动。"""
    times=tuple(candidate.times); controls=tuple(map(tuple,candidate.controls))
    if not controls:
        if abs(candidate.states[-1,3])>1e-10 or abs(candidate.states[-1,4])>1e-10:
            raise ValueError('empty moving trajectory')
        if hold_s<=0: raise ValueError('positive finite hold required')
        times=(0.,hold_s); controls=((0.,0.,0.),)
    return TimedPlan(sequence,source_feedback_s,target_s,times,controls,candidate.map_epoch,
                     candidate.route_version,candidate.mode,candidate.source,tuple(candidate.states[0,:5]),
                     feedback['feedback_id'] if feedback else -1,feedback['sequence'] if feedback else -1,
                     tuple(feedback['vector']) if feedback else ())


def _plant_main(commands,feedback,state,limits,max_steps):
    model=PlantModel(state,limits)
    origin=perf_counter()
    feedback.put(dict(kind='READY',origin_wall_s=origin,**model.feedback()))
    for index in range(max_steps):
        deadline=origin+(index+1)*.05
        while perf_counter()<deadline:
            try: message=commands.get(timeout=min(.005,max(0.,deadline-perf_counter())))
            except queue.Empty: continue
            if message['kind']=='STOP': return
            if message['kind']=='REJECT_TARGET':
                model.reject_target(); feedback.put(dict(kind='TARGET_REJECTED',received_wall_s=perf_counter()))
            else:
                received=perf_counter()
                reason=model.accept(message['plan'],received-origin)
                feedback.put(dict(kind='PLAN_ACK',sequence=message['plan'].sequence,reason=reason,received_wall_s=received,plan=asdict(message['plan'])))
        wake=perf_counter()
        model.advance(.05)
        feedback.put(dict(kind='FEEDBACK',step=index+1,origin_wall_s=origin,wake_wall_s=wake,published_wall_s=perf_counter(),lateness_s=wake-deadline,segments=model.last_segments,**model.feedback()))


class IndependentPlant:
    def __init__(self,state,limits,max_steps):
        context=mp.get_context('spawn')
        self.commands=context.Queue(); self.feedback=context.Queue()
        self.process=context.Process(target=_plant_main,args=(self.commands,self.feedback,state,limits,max_steps))
        self.process.start()
    def send(self,plan): self.commands.put(dict(kind='PLAN',plan=plan))
    def reject_target(self): self.commands.put(dict(kind='REJECT_TARGET'))
    def poll(self):
        rows=[]
        while True:
            try: rows.append(self.feedback.get_nowait())
            except queue.Empty: return rows
    def close(self):
        self.commands.put(dict(kind='STOP'))
        rows=[]; deadline=perf_counter()+2.
        while self.process.is_alive() and perf_counter()<deadline:
            rows.extend(self.poll()); self.process.join(.01)
        if self.process.is_alive(): self.process.terminate(); self.process.join(.5)
        rows.extend(self.poll())
        return rows


@dataclass(frozen=True)
class WorkerDelay:
    seconds: float
    def __call__(self,runner,request):
        start=perf_counter(); sleep(self.seconds)
        result=runner(request)
        result['injected_delay_s']=self.seconds
        result['delay_start_wall_s']=start
        result['worker_start_wall_s']=start
        return result
