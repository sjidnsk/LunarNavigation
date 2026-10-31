"""MODEL_SIMULATION only. Reuse coordinator, buffer, command proof and dynamics."""
import argparse
from collections import Counter
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, Route, VehicleState, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.dynamics import step
from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
from scripts.navigation.one_cm.nav1cm.reference_buffer import _at, _project
from scripts.navigation.one_cm.nav1cm.trajectory_buffer import FeedbackTolerance
from scripts.navigation.one_cm.nav1cm.maps import build_distance_field


def identity(path):
    path=Path(path).resolve()
    return dict(path=str(path),sha256=sha256(path.read_bytes()).hexdigest())


def load_planner(path,name):
    spec=importlib.util.spec_from_file_location('scripts.navigation.one_cm.nav1cm._comparison_'+name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    return module.MpccPlanner,identity(path)


class PendingGlobal:
    busy=True
    def poll(self): return None
    def close(self): pass


class ScheduledLocal:
    ready=True
    def __init__(self,planner,delay,field_backend=None): self.planner=planner;self.delay=delay;self.now=0.;self.pending=None;self.field_backend=field_backend
    @property
    def busy(self): return self.pending is not None
    def submit(self,r):
        options={}
        if self.field_backend is not None:
            options=dict(distance_field=build_distance_field(r['grid'],backend=self.field_backend),field_grid_digest=r['grid'].digest())
        p=self.planner.plan(r['state'],r['grid'],r['reference'],initial_stopped_angular_rps=r['initial_stopped_angular_rps'],initial_travel_direction=r.get('initial_travel_direction',''),**options)
        self.pending=(self.now+self.delay,dict(request_identity=r['identity'],candidate=p.candidate,reason=p.reason,attempts=p.attempts,timings_ms=p.timings_ms))
    def poll(self):
        if self.pending is None or self.now+1e-9<self.pending[0]: return None
        result=self.pending[1];self.pending=None;return result
    def close(self,timeout_s=2.): pass


def target(source,state,floor,interval):
    phase=_project(source,np.array([state.x,state.y,state.yaw,state.v,state.w]),floor,FeedbackTolerance())
    return _at(source,min(float(source.times[-1]),phase+interval))[0][3:5]


def summarize(rows):
    def dist(key):
        a=np.array([r[key] for r in rows if r.get(key) is not None])
        return None if not len(a) else dict(p50=np.percentile(abs(a),50,axis=0).tolist(),p95=np.percentile(abs(a),95,axis=0).tolist(),max=abs(a).max(axis=0).tolist())
    reasons=Counter(r['diagnostics']['command_reject_reason'] for r in rows if r['diagnostics'].get('command_reject_reason'))
    events=Counter(e['kind']+(':'+str(e['reason']) if 'reason' in e else '') for r in rows for e in r['events'])
    return dict(issued_jump=dist('issued_jump'),raw_jump=dist('raw_jump'),handover_target_jump=dist('handover_target_jump'),zero_insertions=sum(r['zero_insertion'] for r in rows),nearzero_recovery_proxies=sum(r['restart'] for r in rows),strict_zero_insertions=sum(r['strict_zero_insertion'] for r in rows),initial_start_completed=sum(e['kind']=='INITIAL_START_COMPLETED' for r in rows for e in r['events']),command_rejections=dict(reasons),events=dict(events))


def simulate(planner,scenario,*,duration=12.,dt=.05,plan_period=.1,result_delay=0.,feedback_delay=0.,refresh_period=2.,command_hold_s=1.2,backend=None,field_backend=None):
    worker=ScheduledLocal(planner,result_delay,field_backend)
    c=AsyncNavigationCoordinator(local_worker=worker,global_runner=PendingGlobal(),limits=planner.limits,planning_period_s=plan_period,execution_period_s=dt,revalidate_current_map=True,backend=backend,field_backend=field_backend)
    c.velocity_reference=True
    points={'straight':[[0.,0.],[8.,0.]],'curve':[[0.,0.],[1.,0.],[2.,.3],[3.,.9],[5.,2.],[8.,3.]],'spin':[[0.,0.],[0.,8.]],'refresh':[[0.,0.],[8.,0.]]}[scenario]
    goal=tuple(points[-1]);c._goal=goal
    state=VehicleState(0.,0.,0.,0.,0.);previous=np.zeros(2);last_raw=None;rows=[];history=[(0.,state)]
    moving=False;dipped=False;was_near=True;was_strict=True;version=1
    try:
        for k in range(int(round(duration/dt))):
            now=k*dt;epoch=1+int(now/refresh_period) if scenario=='refresh' else 1
            wanted=1+int(now/(2*refresh_period)) if scenario=='refresh' else 1
            if c.route is None or wanted!=version:
                version=wanted;c.route=Route(version,epoch,points,goal);c.progress=None
            world=WorldSnapshot(epoch,(),(CellRect(-1200,-1200,1200,1200),))
            observed=next(value for stamp,value in reversed(history) if stamp<=max(0.,now-feedback_delay)+1e-9)
            old=c.buffer.source;old_floor=c.buffer.source_phase_s;worker.now=now
            result=c.tick(world,observed,goal,now,need_action=False)
            accepted=any(e['kind']=='LOCAL_ACCEPTED' for e in result.events)
            jump=None
            if accepted and old is not None and c.buffer.source is not old:
                jump=(target(c.buffer.source,observed,0.,planner.dt)-target(old,observed,old_floor,planner.dt)).tolist()
            diagnostics={}
            issued=np.array(certified_velocity_command(result,observed,planner.limits,reference_interval_s=planner.dt,last_command=previous,elapsed_s=dt,command_hold_s=command_hold_s,backend=backend,diagnostics=diagnostics))
            raw=diagnostics.get('target_raw_vw');near=bool(np.max(np.abs(issued))<=.005)
            insertion=bool(moving and near and not was_near);restart=bool(dipped and not near)
            if insertion:dipped=True
            if restart:dipped=False
            if not near:moving=True
            rows.append(dict(time_s=now,actual_state=vars(state),feedback_state=vars(observed),epoch=epoch,route_version=version,status=result.status,source_digest=None if c.buffer.source is None else c.buffer.source.digest(),source_phase_s=c.buffer.source_phase_s,accepted=accepted,handover_target_jump=jump,issued=issued.tolist(),issued_jump=(issued-previous).tolist(),raw_jump=None if raw is None or last_raw is None else (np.array(raw)-last_raw).tolist(),zero_insertion=insertion,strict_zero_insertion=bool(moving and np.max(abs(issued))<1e-7 and not was_strict),restart=restart,diagnostics=diagnostics,events=list(result.events)))
            if raw is not None:last_raw=np.array(raw)
            x=np.array([state.x,state.y,state.yaw,state.v,state.w,0.])
            x=step(x,[(issued[0]-state.v)/dt,(issued[1]-state.w)/dt,0.],dt)
            state=VehicleState(*x[:5]);history.append((now+dt,state));previous=issued;was_near=near;was_strict=bool(np.max(abs(issued))<1e-7)
    finally:c.close()
    return dict(scenario=scenario,summary=summarize(rows),rows=rows)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('baseline-module','warm-module','build-dir','output','limits-json'):parser.add_argument('--'+key,required=True)
    parser.add_argument('--duration',type=float,default=12.)
    parser.add_argument('--feedback-delay',type=float,default=0.)
    parser.add_argument('--result-delay',type=float,default=0.)
    parser.add_argument('--n',type=int,default=20)
    parser.add_argument('--command-hold',type=float,default=1.2)
    parser.add_argument('--geometry-lib')
    parser.add_argument('--field-lib')
    parser.add_argument('--scenarios',nargs='+',choices=['straight','curve','spin','refresh'],default=['straight','curve','spin','refresh'])
    a=parser.parse_args(argv)
    if a.duration<=0 or min(a.feedback_delay,a.result_delay)<0 or a.command_hold<=0:parser.error('invalid duration/delay')
    result=dict(evidence='MODEL_SIMULATION',plant='issued velocity reached over one dt using existing midpoint step; no vehicle actuator latency model',comparison='same initial conditions and scheduled map/routes; each arm feeds back its own commands, not identical state replay',feedback_delay_s=a.feedback_delay,result_delivery_delay_s=a.result_delay,n=a.n,arms={},shared={})
    for name in ('async_coordinator','coordinator','reference_buffer','execution','certification'):
        result['shared'][name]=identity(ROOT/'scripts/navigation/one_cm/nav1cm'/(name+'.py'))
    limits=WheelLimits(**json.loads(Path(a.limits_json).read_text(encoding='utf-8')))
    result.update(limits=vars(limits),limits_file=identity(a.limits_json),command_hold_s=a.command_hold,nearzero_analysis_threshold=.005,strict_zero_threshold=1e-7,restart_semantics='nearzero recovery proxy, not INITIAL_START_COMPLETED')
    backend=field_backend=None
    if a.geometry_lib:
        from scripts.navigation.one_cm.nav1cm.geometry_native import NativeGeometry
        backend=NativeGeometry(a.geometry_lib);result['geometry_library']=identity(a.geometry_lib)
    if a.field_lib:
        from scripts.navigation.one_cm.nav1cm.field_native import NativeDistanceField
        field_backend=NativeDistanceField(a.field_lib);result['field_library']=identity(a.field_lib)
    for arm in ('baseline','warm'):
        cls,info=load_planner(getattr(a,arm+'_module'),arm)
        runs=[simulate(cls(limits,Path(a.build_dir)/arm/s,n=a.n,dt=.2),s,duration=a.duration,result_delay=a.result_delay,feedback_delay=a.feedback_delay,command_hold_s=a.command_hold,backend=backend,field_backend=field_backend) for s in a.scenarios]
        result['arms'][arm]=dict(module=info,runs=runs)
    path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(json.dumps(result,ensure_ascii=False,indent=2).encode('utf-8'))
    print(json.dumps({arm:[r['summary'] for r in value['runs']] for arm,value in result['arms'].items()},ensure_ascii=False))


if __name__=='__main__':main()
