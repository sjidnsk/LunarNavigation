"""测试身份账仅接受Runtime单写者的结构化TASK_ACCEPTED；不猜跨topic顺序。"""
from collections import namedtuple
import json
import math
import re

AcceptedGoal=namedtuple('AcceptedGoal','task_id generation goal observed_at')

class AcceptanceLedger:
    def __init__(self):
        self.bindings={};self.generations={};self.finished=set();self.errors=[];self._current=None;self._maximum=0;self.observed_ids=set();self.last_frontend_pose=None

    @property
    def ok(self):return not self.errors
    @property
    def current(self):
        return self.get(self._current) if self._current not in self.finished else None
    def get(self,task_id):return self.bindings.get(task_id) if self.ok else None
    def observe_uuid(self,task_id):
        self.observed_ids.add(task_id)
        return self.get(task_id)  # wire事件只能查现有exact绑定，不能创建它。
    def observe_frontend_goal(self,pose):
        self.last_frontend_pose=tuple(pose)  # 仅旁观显示，无UUID，不进入身份账。
    def finish(self,task_id):
        self.finished.add(task_id)
        if self._current==task_id:self._current=None
    def record(self,payload,observed_at):
        try:
            uid=payload['task_id'];generation=payload['accepted_generation'];goal=tuple(float(v) for v in payload['goal'])
            if not isinstance(uid,str) or not re.fullmatch('[0-9a-f]{32}',uid) or type(generation) is not int or generation<=0 or len(goal)!=3 or not all(math.isfinite(v) for v in goal):raise ValueError('invalid TASK_ACCEPTED identity')
            existing=self.bindings.get(uid)
            if existing:
                if (existing.generation,existing.goal)!=(generation,goal):raise ValueError('conflicting UUID acceptance')
                return existing
            if generation in self.generations and self.generations[generation]!=uid:raise ValueError('conflicting accepted generation')
            item=AcceptedGoal(uid,generation,goal,float(observed_at));self.bindings[uid]=item;self.generations[generation]=uid
            if generation>self._maximum:
                self._maximum=generation;self._current=uid if uid not in self.finished else None
            return item
        except (ValueError,KeyError,TypeError) as exc:
            self.errors.append(str(exc));return None

    def parse(self,line,observed_at):
        marker='TASK_ACCEPTED: '
        if marker not in line:return None
        try:return self.record(json.loads(line.split(marker,1)[1]),observed_at)
        except (ValueError,TypeError) as exc:self.errors.append(str(exc));return None

def progress_transitions(lines):
    """按同一 launch 进程的日志行序记录当前目标到达后接受新观察位姿。"""
    transitions=[]
    writers={}
    for line_number,line in enumerate(lines,1):
        match=re.match(r'^\[(navigation_task_node-\d+)\].*? (TASK_ACCEPTED|TASK_STATE): (.*)$',line)
        if not match:
            continue
        writer,kind,raw=match.groups()
        payload=json.loads(raw)
        ledger,terminal=writers.setdefault(writer,(AcceptanceLedger(),None))
        current=ledger.current
        if kind=='TASK_STATE':
            if (current and payload.get('task_id')==current.task_id and payload.get('generation')==current.generation
                    and (payload.get('phase'),payload.get('outcome'),payload.get('reason_code'))==('FINISHED','GOAL_REACHED','GOAL_REACHED')):
                writers[writer]=(ledger,line_number)
            continue
        if ledger.get(payload.get('task_id')) is not None:
            continue
        accepted=ledger.record(payload,line_number)
        if terminal and current and accepted and accepted.generation>current.generation:
            yaw_delta=accepted.goal[2]-current.goal[2]
            yaw_distance=abs(math.atan2(math.sin(yaw_delta),math.cos(yaw_delta)))
            if math.dist(current.goal[:2],accepted.goal[:2])>1e-9 or yaw_distance>1e-9:
                transitions.append(dict(writer=writer,reached_uuid=current.task_id,reached_generation=current.generation,
                                        reached_line=terminal,next_uuid=accepted.task_id,next_generation=accepted.generation,
                                        next_line=line_number))
        writers[writer]=(ledger,None)
    return transitions
