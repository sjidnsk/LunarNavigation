"""Bounded, resumable evaluation processes with independent DDS and progress files."""
from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from . import course_two_control as ctl
from .processes import _LAUNCH
from .evaluation_error_policy import enabled, counted_failure

MAX_WORKERS=16
EVALUATION_DOMAIN_BASE=216


def validate_slot(slot):
    if type(slot) is not int or not 0<=slot<MAX_WORKERS:
        raise ValueError('worker slot must be in 0..15')
    return slot


def evaluation_domain(slot):
    return EVALUATION_DOMAIN_BASE+validate_slot(slot)


def worker_count(settings,default=8):
    settings=Path(settings)
    workers=ctl.read(settings).get('workers',default) if settings.exists() else default
    if type(workers) is not int or not 1<=workers<=MAX_WORKERS:
        raise ValueError('workers must be an integer in 1..16')
    return workers


def evaluation_proof_matches(root, output, result, proof, identity):
    """An audited lifecycle-only repair may adopt an unchanged baseline file.

    Preserve its original runtime attribution and bind approval to every byte,
    actor, config and case. New results always record the current runtime.
    """
    actual=result.get('proof',{})
    if actual==proof:return True
    try:key=str(Path(output).resolve().relative_to(Path(root).resolve()))
    except ValueError:return False
    adopted=identity.get('evaluation_cache_adoptions',{}).get(key)
    return (isinstance(actual,dict) and adopted==dict(sha=ctl.sha(output),
        from_runtime=actual.get('runtime'),to_runtime=proof.get('runtime'))
        and dict(actual,runtime=proof.get('runtime'))==proof)


def cached_result(root,output,proof):
    output=Path(output)
    if not output.exists():return None
    result=ctl.read(output);actual=result.get('proof',{})
    row=result.get('row',{})
    if (not evaluation_proof_matches(root,output,result,proof,ctl.read(Path(root)/'identity.json')) or row.get('case_id')!=proof['case']['case_id']
            or row.get('kind')!=proof['case']['kind']):
        raise ValueError('cached evaluation identity changed: '+str(output))
    if result.get('status')!='finished' or row.get('error'):
        # DDS lease refusal is a single-case communication startup failure.
        # Keep the raw result/proof; no simulator observation means unknown collision.
        if enabled(root) and result.get('status')=='failed' and isinstance(row.get('error'),str) and row['error'].startswith('DomainBusy:'):
            return dict(result,row=counted_failure(dict(row,collisions=None)))
        if enabled(root) and result.get('status')=='failed' and isinstance(row.get('error'),str) and row['error'].startswith(('InfrastructureError:','WorkerExit:')):
            return dict(result,row=counted_failure(row))
        raise RuntimeError('evaluation infrastructure error: '+str(row.get('error')))
    return result


def evaluation_command(root,request,output,*,slot,progress,job_id):
    return [sys.executable,'-m','lunar_drl_exploration.course_two_control','evaluate-one',
        '--root',str(root),'--request',str(request),'--output',str(output),
        '--slot',str(slot),'--progress',str(progress),'--job-id',job_id]


def interrupt_children(controller):
    for slot,process in list(controller.evaluation_children.items()):
        if process.poll() is None and slot not in controller.evaluation_signalled:
            try:process.send_signal(signal.SIGINT)
            except ProcessLookupError:pass
            controller.evaluation_signalled.add(slot)


def screen(controller,actor,cases,label):
    root=controller.root;actor=Path(actor).resolve()
    settings=root/'evaluation-settings.json'
    workers=worker_count(settings,default=1)
    ids=[c['case_id'] for c in cases]
    if len(set(ids))!=len(ids) or any(Path(cid).name!=cid or cid in ('.','..') for cid in ids):
        raise ValueError('invalid or duplicate evaluation cases')
    if Path(label).name!=label or label in ('.','..'):raise ValueError('invalid evaluation label')
    directory=controller.state.get('evaluation_directory','evaluation')
    if Path(directory).name!=directory or directory in ('.','..'):raise ValueError('invalid evaluation directory')
    folder=root/directory/label;folder.mkdir(parents=True,exist_ok=True)
    common=dict(actor_sha=ctl.sha(actor),config_sha=ctl.sha(root/'config.json'),runtime=ctl.runtime_digest())
    results={};pending=deque();jobs={}
    for case in cases:
        result=cached_result(root,folder/(case['case_id']+'.json'),dict(common,case=case))
        if result is None:pending.append(case)
        else:results[case['case_id']]=result
    resumed=len(results);started=time.time()

    def publish():
        controller.save(phase='evaluation',evaluation_label=label,evaluation_done=len(results),
            evaluation_total=len(cases),evaluation_concurrency=workers,
            evaluation_cached=resumed,evaluation_started_unix=started,
            evaluation_queued=len(pending),evaluation_workers=[jobs[i]['record'] for i in sorted(jobs)],
            child_pid=None,child_label='eval-'+label+'-parallel')

    publish()
    try:
        while pending or jobs:
            if controller.stopping:break
            for slot in range(workers):
                if not pending or controller.stopping:break
                if slot in jobs:continue
                case=pending.popleft();cid=case['case_id'];job_id=uuid.uuid4().hex
                live=root/'evaluation-live'/label/cid/job_id
                request=live/'request.json';progress=live/'progress.json';output=folder/(cid+'.json')
                ctl.write(request,dict(actor=str(actor),case=case))
                args=evaluation_command(root,request,output,slot=slot,progress=progress,job_id=job_id)
                log=root/'logs'/('eval-'+label+'-'+cid+'.log')
                with log.open('ab') as stream:
                    # Parent-death binding also removes the worker's native ROS children
                    # if the controller dies. Each native child has the same protection.
                    process=subprocess.Popen([sys.executable,'-c',_LAUNCH,str(os.getpid()),*args],
                        stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,
                        env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1'))
                controller.evaluation_children[slot]=process
                record=dict(slot=slot,pid=process.pid,case_id=cid,domain_id=evaluation_domain(slot),job_id=job_id,
                    request=str(request.relative_to(root)),progress=str(progress.relative_to(root)),started_unix=time.time())
                jobs[slot]=dict(process=process,record=record,case=case,output=output)
                publish()
                if controller.stopping:interrupt_children(controller)
            for slot,job in list(jobs.items()):
                code=job['process'].poll()
                if code is None:continue
                if controller.stopping:break
                cid=job['case']['case_id']
                if code and not enabled(root):raise RuntimeError(f'eval-{label}-{cid} exited {code}; see logs/eval-{label}-{cid}.log')
                if code and not job['output'].exists():
                    record=job['record'];progress_path=root/record['progress']
                    progress=ctl.read(progress_path) if progress_path.exists() else {}
                    matched=all(progress.get(k)==record.get(k) for k in ('case_id','slot','job_id')) and progress.get('actor')==str(actor)
                    case=job['case'];row=dict(case_id=cid,kind=case['kind'],capability=case.get('capability'),completed=False,
                        error=f'WorkerExit: exited {code}; see logs/eval-{label}-{cid}.log',collisions=None,
                        initial_completed=False,decisions=progress.get('decisions') if matched else None,
                        final_coverage=progress.get('coverage') if matched else None,coverage_source='last_observed' if matched else 'unavailable')
                    ctl.write(job['output'],dict(status='failed',proof=dict(common,case=case),row=counted_failure(row),
                        wall_s=time.time()-record['started_unix'],execution=record))
                result=cached_result(root,job['output'],dict(common,case=job['case']))
                if result is None:raise RuntimeError('evaluation exited without a result: '+cid)
                if code and not result['row'].get('error'):raise RuntimeError('nonzero evaluation exit contradicts successful result: '+cid)
                results[cid]=result
                del jobs[slot];del controller.evaluation_children[slot]
                controller.evaluation_signalled.discard(slot)
                publish()
            if pending or jobs:time.sleep(.1)
    finally:
        interrupt_children(controller)
        deadline=time.monotonic()+30
        forced=[]
        for slot,process in list(controller.evaluation_children.items()):
            try:process.wait(timeout=max(.01,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                process.kill();process.wait(timeout=5);forced.append(process.pid)
        controller.evaluation_children.clear();controller.evaluation_signalled.clear();jobs.clear()
        # Capture any atomic result that finished at the pause boundary.
        for case in cases:
            if case['case_id'] not in results:
                output=folder/(case['case_id']+'.json')
                if output.exists():
                    try:
                        result=cached_result(root,output,dict(common,case=case))
                        if result is not None:results[case['case_id']]=result
                    except (ValueError,RuntimeError):pass  # Remains an error on the next verification.
        publish()
        if forced:raise RuntimeError('evaluation cleanup required forced termination: '+str(forced))
    if controller.stopping:return None
    rows=[results[cid]['row'] for cid in ids]
    summary=folder/'summary.json'
    existing=ctl.read(summary) if summary.exists() else None
    if not existing or existing.get('status')!='finished' or existing.get('actor_sha')!=common['actor_sha'] or existing.get('rows')!=rows:
        ctl.write(summary,dict(status='finished',actor_sha=common['actor_sha'],rows=rows,
            runtime_by_case={cid:results[cid]['proof']['runtime'] for cid in ids}))
    return rows
