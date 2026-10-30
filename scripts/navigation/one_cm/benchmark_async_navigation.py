"""有界异步验证：logical为确定局部交付时间，wall为真实worker和独立车辆进程。"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import platform
from time import perf_counter, sleep
import numpy as np
from nav1cm.async_coordinator import AsyncNavigationCoordinator
from nav1cm.verification_evidence import tick_global_evidence, formal_global_evidence, execution_coverage
from nav1cm.contracts import WheelLimits, VehicleState, Trajectory
from nav1cm.certification import certify
from nav1cm.maps import extract_window
from nav1cm.seeds import brake_candidate
from nav1cm.fixtures import scenarios
from nav1cm.geometry_native import NativeGeometry
from nav1cm.field_native import NativeDistanceField
from nav1cm.local_worker import LocalWorker, _initialize, run_request
from nav1cm.verification_plant import PlantModel, IndependentPlant, prepare_handover, plan_from_candidate, WorkerDelay


class LogicalWorker:
    """仅离线逻辑模型：submit同步求解，结果在逻辑deadline交付；不声称非阻塞墙钟。"""
    def __init__(self,limits,build,field,delay):
        _initialize(limits,build,dict(n=20,dt=.1),field)
        self.now=0.; self.delay=delay; self.result=None
    @property
    def busy(self): return self.result is not None
    def submit(self,request):
        self.result=run_request(request); self.deadline=self.now+self.delay
    def poll(self):
        if self.result is None or self.now+1e-9<self.deadline: return None
        result,self.result=self.result,None
        return result
    def close(self): pass


def _percentiles(values):
    return dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100])))) if values else {}


def audit_feedback(rows,scenario,limits,backend):
    """执行后独立认证；绝不替换实际动作，失败保留并使总结果失败。"""
    audited=[]
    if scenario.updated_world is not None and (type(scenario.update_step) is not int or scenario.update_step<0):
        raise ValueError('map event must align to nonnegative integer 50ms plant slot')
    event_s=scenario.update_step*.05 if scenario.updated_world is not None else float('inf')
    def audit_state(vector,time_s,world,kind):
        state=VehicleState(*vector[:5]); grid=extract_window(world,state,scenario.allow_unknown)
        brake=brake_candidate(state,grid,SimpleNamespace(route_version=0,progress_s=vector[5]),limits)
        cert=certify(brake,grid,limits,0,backend=backend)
        audited.append(dict(kind=kind,start_s=time_s,end_s=time_s,valid=cert.valid,reason=cert.reason.value,world_epoch=world.epoch,source='state_and_stop'))
    samples=[row for row in rows if row['kind']=='FEEDBACK']
    for row in rows:
        if row['kind']!='FEEDBACK': continue
        for segment in row['segments']:
            if segment['start_s']+1e-9<event_s<segment['end_s']-1e-9:
                audited.append(dict(kind='MAP_EVENT',start_s=event_s,end_s=event_s,valid=False,reason='EVENT_INSIDE_UNSPLIT_SEGMENT',world_epoch=scenario.updated_world.epoch,source='protocol'))
            world=scenario.updated_world if scenario.updated_world and segment['start_s']+1e-9>=scenario.update_step*.05 else scenario.world
            before=VehicleState(*segment['before'][:5])
            grid=extract_window(world,before,scenario.allow_unknown)
            traj=Trajectory(world.epoch,segment['route_version'],[0.,segment['end_s']-segment['start_s']],
                [segment['before'],segment['after']],[segment['control']],segment['mode'],segment['source'])
            cert=certify(traj,grid,limits,traj.route_version,backend=backend)
            audited.append(dict(start_s=segment['start_s'],end_s=segment['end_s'],valid=cert.valid,reason=cert.reason.value,world_epoch=world.epoch,source=segment['source']))
            if abs(segment['end_s']-event_s)<1e-9:
                audit_state(segment['after'],event_s,scenario.updated_world,'MAP_EVENT_STATE')
            elif abs(segment['start_s']-event_s)<1e-9:
                audit_state(segment['before'],event_s,scenario.updated_world,'MAP_EVENT_STATE')
    if samples:
        last=samples[-1]; time_s=last['sample_s']
        world=scenario.updated_world if time_s+1e-9>=event_s else scenario.world
        audit_state(last['vector'],time_s,world,'TERMINAL_STATE')
    return audited


def continuity_assessment(busy_evidence,audit,protocol_errors,tail_inflight,*,execution_coverage=None):
    covered=any(row['feedback_count']>=2 and row['displacement_m']>1e-6 for row in busy_evidence)
    return dict(delayed_motion_covered=covered,
        continuity_pass=covered and bool(execution_coverage and execution_coverage['complete']) and bool(audit) and all(row['valid'] for row in audit) and not protocol_errors,
        tail_inflight_uncovered=bool(tail_inflight),completed_busy_intervals=len(busy_evidence))


def classify_protocol_events(records):
    """归并累计反馈和ACK对同一次裁决的报告；过期拒收属于鲁棒性观测。"""
    events={}
    for row in records:
        for failure in row.get('protocol_failures',[]):
            key=(failure['sequence'],failure['reason'],failure.get('sample_s'))
            events.setdefault(key,dict(failure))
    ignored={'ACCEPTED','PENDING_FULL','TARGET_REJECTED'}
    for row in records:
        if row['kind']!='PLAN_ACK' or row['reason'] in ignored: continue
        received=row.get('received_wall_s')
        matches=[event for event in events.values() if event['sequence']==row['sequence'] and event['reason']==row['reason']
                 and event.get('received_wall_s',received)==received]
        if len(matches)==1: event=matches[0]
        else:
            key=(row['sequence'],row['reason'],('wall',received))
            event=events.setdefault(key,dict(sequence=row['sequence'],reason=row['reason']))
        event['received_wall_s']=received
        event['source_feedback_s']=row.get('plan',{}).get('source_feedback_s')
    freshness=[event for event in events.values() if event['reason'] in ('SOURCE_EXPIRED','LATE_DELIVERY')]
    fatal=[event for event in events.values() if event['reason'] not in ('SOURCE_EXPIRED','LATE_DELIVERY')]
    return freshness,fatal


def session_outcome(records,audit,goal,busy_evidence,tail_inflight,base_status='STEP_LIMIT',*,initial_anchor=None):
    """纯后处理：实际运动结果、完整性、审计与连续性分别判定。"""
    freshness,errors=classify_protocol_events(records)
    feedback=[row for row in records if row['kind']=='FEEDBACK']
    last=feedback[-1] if feedback else None
    rejected=any(row['kind']=='TARGET_REJECTED' or row.get('target_rejected',False) or
                 (row['kind']=='TICK' and row.get('status') in ('GOAL_IN_OBSTACLE','GOAL_CLEARANCE')) for row in records)
    state=last['vector'] if last else None
    stopped=state is not None and abs(state[3])<=.01 and abs(state[4])<=.01
    arrived=bool(stopped and not rejected and np.linalg.norm(np.array(state[:2])-goal)<=.1)
    motion=('SAFE_STOP_GOAL_REJECTED' if stopped else 'GOAL_REJECTED_BRAKING') if rejected else ('ARRIVED' if arrived else ('STOPPED_NOT_ARRIVED' if stopped else 'NOT_ARRIVED'))
    coverage=execution_coverage(records,audit,initial_anchor=initial_anchor)
    formal=formal_global_evidence(records)
    audit_pass=coverage['complete'] and bool(audit) and all(row['valid'] for row in audit)
    # 仅协议/审计与运动分类可由本次证据重算，其他执行终止原因不可被到达覆盖。
    motion_statuses={'ARRIVED','SAFE_STOP_GOAL_REJECTED','GOAL_REJECTED_BRAKING',
                     'STOPPED_NOT_ARRIVED','NOT_ARRIVED'}
    recomputable=motion_statuses|{'FAILED_PROTOCOL','FAILED_EXECUTION_AUDIT','FAILED_EXECUTION_COVERAGE','STEP_LIMIT'}
    if errors:
        status='FAILED_PROTOCOL'
    elif base_status not in recomputable:
        status=base_status
    elif not coverage['structure_complete']:
        status='FAILED_EXECUTION_COVERAGE'
    elif not audit_pass:
        status='FAILED_EXECUTION_AUDIT'
    elif arrived or rejected or base_status in motion_statuses:
        status=motion
    else:
        status='STEP_LIMIT'
    continuity=continuity_assessment(busy_evidence,audit,errors,tail_inflight,execution_coverage=coverage)
    continuity['continuity_pass']=continuity['continuity_pass'] and audit_pass
    return dict(status=status,navigation_arrived=arrived,motion_outcome=motion,integrity_pass=not errors,
        execution_coverage=coverage,formal_global_evidence=formal,
        **{'pass':bool(status=='ARRIVED' and arrived and audit_pass and continuity['continuity_pass'] and formal['pass'])},
        all_executed_actions_audited=audit_pass,protocol_errors=errors,freshness_drops=freshness,
        freshness_drop_count=len(freshness),**continuity)


def reanalyze_session(identity,raw_jsonl,audit,original_summary,case,*,goal,analysis_sha256,initial_anchor=None):
    """纯只读重分析；输入JSON/JSONL原始bytes或文本，不读取或覆盖任何文件。"""
    inputs=dict(identity=identity,raw_jsonl=raw_jsonl,audit=audit,original_summary=original_summary)
    encoded={key:(value.encode('utf-8') if isinstance(value,str) else value) for key,value in inputs.items()}
    original=json.loads(encoded['original_summary']); source_identity=json.loads(encoded['identity'])
    records=[json.loads(line) for line in encoded['raw_jsonl'].splitlines() if line.strip()]
    audited=json.loads(encoded['audit'])
    prior=original['cases'][case]
    result=dict(prior)
    result.update(session_outcome(records,audited,goal,prior.get('worker_busy_evidence',[]),
        prior.get('tail_inflight_uncovered',False),prior['status'],initial_anchor=initial_anchor))
    return dict(identity=source_identity,cases={case:result},original_status=prior['status'],
        execution_rerun=False,analysis_sha256=analysis_sha256,original_summary=original,
        analysis_initial_anchor=initial_anchor,
        input_sha256={key:hashlib.sha256(value).hexdigest() for key,value in encoded.items()},
        analysis_goal=list(goal),analysis_goal_source='explicit caller-provided goal; original execution identity preserved')


def cadence(times):
    periods=np.diff(times).tolist()
    return dict(count=len(times),period_s=_percentiles(periods),
        observed_hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else None)


def run(args):
    out=Path(args.output_dir); out.mkdir(parents=True,exist_ok=False)
    limits=WheelLimits(linear_speed_mps=.5,angular_speed_rps=.5)
    backend=NativeGeometry(args.geometry_lib)
    field=NativeDistanceField(args.distance_field_lib) if args.distance_field_lib else None
    selected=[s for s in scenarios() if not args.case or s.name in args.case]
    if not selected: raise ValueError('no selected cases')
    summaries={}
    identity=dict(mode=args.mode,delay_ms=args.delay_ms,limits=asdict(limits),platform=platform.platform(),
        clock='perf_counter shared monotonic host clock; sample_s relative to plant origin',
        geometry_sha256=hashlib.sha256(Path(args.geometry_lib).read_bytes()).hexdigest(),
        distance_field_sha256=hashlib.sha256(Path(args.distance_field_lib).read_bytes()).hexdigest() if args.distance_field_lib else None,
        source_sha256={str(p.relative_to(Path(__file__).parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.rglob('*.py')},
        not_run=['ROS','DDS','physical sensors/actuators','real vehicle'],
        logical_boundary='local submit solves synchronously offline, fixed logical result delivery; no wall performance claim',
        wall_boundary='fixed-step model, independent spawn wakeups recorded; catchup steps are not stable 20Hz proof; audits after execution never filter controls')
    (out/'identity.json').write_text(json.dumps(identity,indent=2),encoding='utf-8')
    for scenario in selected:
        if scenario.name in ('new-obstacle','goal-later-blocked'): scenario=replace(scenario,update_step=40)
        count=min(args.max_steps or scenario.max_steps,scenario.max_steps)
        cold_start=perf_counter()
        worker=(LocalWorker(limits,args.build_dir,planner_options=dict(n=20,dt=.1),field_backend=field,
                request_wrapper=WorkerDelay(args.delay_ms/1000)) if args.mode=='wall' else
                LogicalWorker(limits,args.build_dir,args.distance_field_lib,args.delay_ms/1000))
        cold={}
        if args.mode=='wall':
            while not worker.ready:
                ready=worker.poll_ready()
                if ready: cold=ready
                if perf_counter()-cold_start>180: worker.close(); raise TimeoutError('warmup >180s')
                sleep(.01)
        cold['preclock_wall_ms']=(perf_counter()-cold_start)*1000
        core=AsyncNavigationCoordinator(local_worker=worker,limits=limits,backend=backend,field_backend=field,
            allow_unknown=scenario.allow_unknown,clearance_radius_m=scenario.clearance_radius_m)
        plant=IndependentPlant(scenario.start,limits,count) if args.mode=='wall' else PlantModel(scenario.start,limits)
        records=[]; ticks=[]; feedback=None; awaiting=None; sequence=0; last_target=-1.; status='STEP_LIMIT'
        origin=None; start=perf_counter()
        stream=(out/(scenario.name+'.jsonl')).open('x',encoding='utf-8')
        def record(row):
            stream.write(json.dumps(row)+'\n'); stream.flush(); records.append(row)
        try:
            if args.mode=='logical':
                record(dict(kind='READY',segments=[],**plant.feedback()))
            for iteration in range(count*1000):
                if args.mode=='wall':
                    messages=plant.poll()
                    for row in messages:
                        record(row)
                        if row['kind'] in ('READY','FEEDBACK'): feedback=row; origin=row['origin_wall_s']
                        if row['kind']=='PLAN_ACK' and row['sequence']==awaiting:
                            # ACK alone does not contain state; wait for a newer feedback containing pending.
                            if row['reason']!='ACCEPTED': awaiting=None
                    if feedback and awaiting is not None and feedback['sequence']>=awaiting: awaiting=None
                    if not feedback: sleep(.002); continue
                    now=perf_counter()-origin
                    if feedback.get('step',0)>=count: break
                    if perf_counter()-start>count*.05+15: status='WALL_TIMEOUT'; break
                else:
                    if iteration>=count: break
                    feedback=plant.feedback(); now=feedback['sample_s']
                prepared=prepare_handover(feedback,now,limits,last_target)
                if prepared is None:
                    if args.mode=='wall': sleep(.002)
                    else:
                        plant.advance(.05)
                        record(dict(kind='FEEDBACK',step=iteration+1,segments=plant.last_segments,**plant.feedback()))
                    continue
                target=prepared['prediction_target_s']
                if args.mode=='logical': worker.now=target
                world=scenario.updated_world if scenario.updated_world and now>=scenario.update_step*.05 else scenario.world
                goal=scenario.updated_goal if scenario.updated_goal and now>=scenario.update_step*.05 else scenario.goal
                before=prepared['state']; began=perf_counter()
                result=core.tick(world,before,goal,target)
                tick=dict(kind='TICK',sample_s=feedback['sample_s'],source_feedback_id=feedback['feedback_id'],source_feedback_sequence=feedback['sequence'],
                    prediction_target_s=target,predicted_state=asdict(before),status=result.status,events=result.events,
                    tick_start_wall_s=began,tick_finish_wall_s=perf_counter(),timings_ms=result.timings_ms,
                    local_busy=worker.busy,buffer_remaining_s=core.buffer.remaining_s(target),
                    **tick_global_evidence(result,goal,world.epoch,core.route))
                ticks.append(tick); record(tick); last_target=target
                if result.status in ('GOAL_IN_OBSTACLE','GOAL_CLEARANCE'):
                    plant.reject_target()
                if result.action is None: status='FAILED_NO_CERTIFIED_ACTION'; break
                candidate=result.candidate
                if awaiting is None and feedback['pending'] is None and result.status not in ('GOAL_IN_OBSTACLE','GOAL_CLEARANCE'):
                    sequence+=1
                    plan=plan_from_candidate(candidate,sequence,feedback['sample_s'],target,feedback=feedback)
                    record(dict(kind='PLAN_SENT',sent_wall_s=perf_counter(),plan=asdict(plan)))
                    if args.mode=='wall': plant.send(plan); awaiting=sequence
                    else:
                        reason=plant.accept(plan,now)
                        record(dict(kind='PLAN_ACK',sequence=sequence,reason=reason,received_wall_s=perf_counter(),plan=asdict(plan)))
                else:
                    record(dict(kind='PLAN_PUBLICATION_SKIPPED',target_s=target,reason='AWAITING_ACK_OR_PENDING' if result.status not in ('GOAL_IN_OBSTACLE','GOAL_CLEARANCE') else 'TARGET_REJECTED'))
                if args.mode=='logical':
                    plant.advance(.05)
                    record(dict(kind='FEEDBACK',step=iteration+1,segments=plant.last_segments,**plant.feedback()))
                actual=feedback['vector']
                stopped=abs(actual[3])<=.01 and abs(actual[4])<=.01
                if result.status in ('GOAL_IN_OBSTACLE','GOAL_CLEARANCE') and stopped: status='SAFE_STOP_GOAL_REJECTED'; break
                if np.linalg.norm(np.array(actual[:2])-goal)<=.1 and stopped: status='ARRIVED'; break
        finally:
            tail_inflight=worker.busy
            if args.mode=='wall':
                for row in plant.close(): record(row)
                for row in plant.poll(): record(row)
            core.close(); stream.close()
        audit=audit_feedback(records,scenario,limits,backend)
        (out/(scenario.name+'-audit.json')).write_text(json.dumps(audit,indent=2),encoding='utf-8')
        if any(not a['valid'] for a in audit): status='FAILED_EXECUTION_AUDIT'
        samples=[r for r in records if r['kind']=='FEEDBACK']
        _,protocol_errors=classify_protocol_events(records)
        intervals=[(e['worker_start_wall_s'],e['worker_finish_wall_s']) for t in ticks for e in t['events'] if e['kind']=='LOCAL_RESULT' and e.get('worker_start_wall_s') is not None]
        busy_evidence=[]
        for begin,end in intervals:
            inside=[r for r in samples if begin<=r.get('published_wall_s',-1)<=end]
            busy_evidence.append(dict(start_wall_s=begin,end_wall_s=end,feedback_count=len(inside),displacement_m=float(np.linalg.norm(np.array(inside[-1]['vector'][:2])-inside[0]['vector'][:2])) if len(inside)>1 else 0.))
        summary=dict(status=status,navigation_arrived=status=='ARRIVED',cold_start=cold,steps=len(samples),ticks=len(ticks),
            final_feedback=samples[-1] if samples else feedback,all_executed_actions_audited=bool(audit) and all(a['valid'] for a in audit),
            worker_busy_evidence=busy_evidence,tick_ms=_percentiles([t['timings_ms']['total'] for t in ticks]),
            plant_lateness_ms=_percentiles([r.get('lateness_s',0)*1000 for r in samples]),
            ack_reasons={reason:sum(r['kind']=='PLAN_ACK' and r['reason']==reason for r in records) for reason in set(r['reason'] for r in records if r['kind']=='PLAN_ACK')})
        observed_sequences={e['request_identity']['sequence'] for t in ticks for e in t['events'] if e['kind']=='LOCAL_RESULT' and e.get('request_identity')}
        summary['unobserved_result_requests']=[e for t in ticks for e in t['events'] if e['kind']=='LOCAL_SUBMITTED' and e['sequence'] not in observed_sequences]
        summary['protocol_errors']=protocol_errors
        summary['schedules']=dict(plant_step=cadence([r['sample_s'] for r in samples]),
            feedback_publication_wall=cadence([r['published_wall_s'] for r in samples if 'published_wall_s' in r]),
            core_tick_wall=cadence([r['tick_start_wall_s'] for r in ticks]),
            core_target=cadence([r['prediction_target_s'] for r in ticks]),
            plan_sent_wall=cadence([r['sent_wall_s'] for r in records if r['kind']=='PLAN_SENT']),
            plan_ack_wall=cadence([r['received_wall_s'] for r in records if r['kind']=='PLAN_ACK']),
            local_submitted_wall=cadence([e['submitted_wall_s'] for t in ticks for e in t['events'] if e['kind']=='LOCAL_SUBMITTED']),
            local_submitted_target=cadence([e['submitted_s'] for t in ticks for e in t['events'] if e['kind']=='LOCAL_SUBMITTED']))
        summary['publication_contract']='single pending: core target schedule 20Hz independent of lower command publication; local request target up to10Hz when ready/not busy; all actual rates reported separately'
        summary.update(session_outcome(records,audit,goal,busy_evidence,tail_inflight,status))
        summaries[scenario.name]=summary
        (out/'summary.json').write_text(json.dumps(dict(identity=identity,cases=summaries),indent=2),encoding='utf-8')
        points=' '.join(f'{100+r["vector"][0]*50:.2f},{350-r["vector"][1]*50:.2f}' for r in samples)
        (out/(scenario.name+'.svg')).write_text('<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="700"><rect width="100%" height="100%" fill="white"/><polyline points="'+points+'" fill="none" stroke="red" stroke-width="2"/></svg>',encoding='utf-8')
        print(json.dumps({scenario.name:summary}),flush=True)
    return summaries


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--build-dir',required=True); parser.add_argument('--output-dir',required=True)
    parser.add_argument('--geometry-lib',required=True); parser.add_argument('--distance-field-lib')
    parser.add_argument('--case',action='append',default=[]); parser.add_argument('--delay-ms',type=float,default=150.)
    parser.add_argument('--mode',choices=('logical','wall'),default='wall'); parser.add_argument('--max-steps',type=int)
    args=parser.parse_args()
    if not math.isfinite(args.delay_ms) or args.delay_ms<0: parser.error('delay must be finite and nonnegative')
    run(args)
