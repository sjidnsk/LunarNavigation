"""Retry checks operate on temporary artifacts and owned fake subprocesses."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from unittest.mock import patch

import pytest

SCRIPT = Path(__file__).with_name('pressure_retry.py')


@pytest.fixture
def retry(tmp_path):
    assert SCRIPT.exists(), 'isolated serial retry helper is missing'
    spec=importlib.util.spec_from_file_location('pressure_retry',SCRIPT)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    cases=mod.pressure.build_cases()
    proof={key:'test-'+key for key in mod.pressure.IDENTITY}
    protocol=dict(cases=cases,modes=['baseline','remote'],frozen=proof)
    (tmp_path/'protocol.json').write_text(json.dumps(protocol))
    owner=dict(pid=9999999,start_ticks=42,ppid=1)
    (tmp_path/'handoff.json').write_text(json.dumps(dict(parallel_owner=owner)))
    state=dict(status='running',pid=owner['pid'],requested_episodes=40,terminal_episodes=27,active=[])
    mod.pressure.write_json(tmp_path/'run-state.json',state)
    for case in cases:
        for mode in case['mode_order']:
            output=tmp_path/case['name']/mode;output.mkdir(parents=True)
            mod.pressure.write_json(output/'dispatch.json',dict(case=case,mode=mode,frozen=proof))
            row=dict(status='finished',ended=1,error=None,mode=mode,family=case['family'],
                extent_m=case['extent'],seed=case['seed'],budget=case['budget'],provenance=proof,completed=True)
            if case['name']=='cave200-r2' and mode=='remote':
                row.update(status='error',ended=0,completed=False,error='native initialization timeout',
                           error_category='infrastructure',infrastructure_failures=1)
            mod.pressure.write_json(output/'summary.json',row)
    case=next(row for row in cases if row['name']=='cave200-r2')
    return mod,tmp_path,protocol,owner,state,case


@pytest.mark.parametrize('changes,want',[({},'queued'),({'status':'error'},'blocked'),
    ({'status':'finished','terminal_episodes':39},'blocked'),
    ({'status':'finished','terminal_episodes':40,'active':[dict(pid=123)]},'blocked')])
def test_formal_gate_never_starts_unfinished_or_failed_batch(retry,changes,want):
    mod,root,protocol,owner,state,_=retry;state.update(changes)
    mod.pressure.write_json(root/'run-state.json',state)
    assert mod.batch_gate(root,protocol,owner)[0]==want
    assert not (root/'retests').exists()


def test_formal_complete_waits_for_registered_owner_exit(retry):
    mod,root,protocol,_,state,_=retry
    process=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    try:
        reader=mod.ProcReader();stat=reader.stat(process.pid)
        owner=dict(pid=process.pid,start_ticks=stat['start_ticks'],ppid=stat['ppid'])
        state.update(status='finished_with_errors',pid=process.pid,terminal_episodes=40)
        mod.pressure.write_json(root/'run-state.json',state)
        assert mod.batch_gate(root,protocol,owner)[0]=='queued'
        process.terminate();process.wait(timeout=2)
        assert mod.batch_gate(root,protocol,owner)[0]=='ready'
    finally:
        if process.poll() is None: process.kill();process.wait()


def test_pid_reuse_and_missing_terminal_block_retry(retry):
    mod,root,protocol,owner,state,_=retry
    process=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    try:
        owner=dict(pid=process.pid,start_ticks=0,ppid=1)
        state.update(status='finished',pid=process.pid,terminal_episodes=40)
        mod.pressure.write_json(root/'run-state.json',state)
        assert mod.batch_gate(root,protocol,owner)[0]=='blocked'
    finally: process.terminate();process.wait(timeout=2)
    owner=dict(pid=9999999,start_ticks=42,ppid=1);state['pid']=owner['pid']
    mod.pressure.write_json(root/'run-state.json',state)
    (root/'moon050-r1'/'baseline'/'summary.json').unlink()
    assert mod.batch_gate(root,protocol,owner)[0]=='blocked'


def test_command_preserves_scene_and_frozen_model_arguments(retry):
    mod,root,_,_,_,case=retry;output=root/'retests'/'new'/'evaluation'
    command=mod.evaluation_command(root,case,'remote',output)
    values={command[i]:command[i+1] for i in range(len(command)-1) if command[i].startswith('--')}
    assert values==dict(zip(['--mode','--config','--actor','--output','--family','--extent','--seed','--budget','--max-remote-positions'],
        ['remote',str(root/'config.json'),str(root/'actor.pt'),str(output),'cave','200','2026100633','1536','32']))


def test_duplicate_retry_label_and_original_error_are_preserved(retry):
    mod,root,protocol,owner,_,case=retry
    original=(root/'cave200-r2'/'remote'/'summary.json').read_bytes()
    job=mod.create_job(root,protocol,owner,case,'remote','cave200-r2-A2-retry-01')
    request=(job/'request.json').read_bytes()
    with pytest.raises(FileExistsError):
        mod.create_job(root,protocol,owner,case,'remote','cave200-r2-A2-retry-01')
    assert (job/'request.json').read_bytes()==request
    assert (root/'cave200-r2'/'remote'/'summary.json').read_bytes()==original


def test_failed_fake_worker_is_infrastructure_error_not_zero_coverage(retry):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','failed-owned-worker')
    with patch.object(mod,'evaluation_command',return_value=[sys.executable,'-c','raise SystemExit(7)']):
        result=mod.evaluate_retry(root,job,protocol,case,'remote',poll_interval=.02)
    assert result['status']=='error' and result['ended']==0
    assert result['error_category']=='infrastructure' and result['exit_code']==7
    assert 'final_coverage' not in result
    with pytest.raises(FileExistsError):
        mod.evaluate_retry(root,job,protocol,case,'remote',poll_interval=.02)


def test_owned_fake_worker_writes_terminal_then_exits_without_touching_original(retry):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','finished-owned-worker')
    summary=dict(status='finished',ended=1,error=None,mode='remote',family='cave',extent_m=200,
        seed=2026100633,budget=1536,provenance=protocol['frozen'],completed=True,final_coverage=.991)
    code=('import json,os,sys; row=json.loads(sys.argv[1]);row["pid"]=os.getpid();'
          'open(sys.argv[2],"w").write(json.dumps(row))')
    with patch.object(mod,'evaluation_command',return_value=[sys.executable,'-c',code,json.dumps(summary),str(job/'evaluation'/'summary.json')]):
        result=mod.evaluate_retry(root,job,protocol,case,'remote',poll_interval=.02)
    assert result['status']=='finished' and result['final_coverage']==.991
    assert json.loads((root/'cave200-r2'/'remote'/'summary.json').read_text())['status']=='error'


def test_interrupted_retry_closes_exact_owned_fake_worker(retry):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','interrupted-owned-worker')
    with patch.object(mod,'evaluation_command',return_value=[sys.executable,'-c','import time;time.sleep(30)']), \
         patch.object(mod.ProcReader,'table',side_effect=KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):
            mod.evaluate_retry(root,job,protocol,case,'remote',poll_interval=.02)
    worker=json.loads((job/'retry-state.json').read_text())['worker_pid']
    with pytest.raises((OSError,ValueError)):
        mod.ProcReader().stat(worker)


def test_dry_run_writes_nothing_and_does_not_start_worker(retry,monkeypatch,capsys):
    mod,root,_,_,_,_=retry
    before={str(path.relative_to(root)):path.read_bytes() for path in root.rglob('*') if path.is_file()}
    monkeypatch.setattr(sys,'argv',['pressure_retry','--root',str(root),'--case','cave200-r2',
        '--mode','remote','--label','read-only','--dry-run'])
    with patch.object(mod.pressure,'verify_frozen'):
        mod.main()
    after={str(path.relative_to(root)):path.read_bytes() for path in root.rglob('*') if path.is_file()}
    assert before==after
    assert json.loads(capsys.readouterr().out)['status']=='queued'


def test_queued_request_mutation_blocks_before_native_launch(retry,monkeypatch):
    mod,root,_,_,state,_=retry;label='changed-request';job=root/'retests'/label
    monkeypatch.setattr(sys,'argv',['pressure_retry','--root',str(root),'--case','cave200-r2',
        '--mode','remote','--label',label])
    def finish_batch(_):
        request=json.loads((job/'request.json').read_text());request['authorization']='changed'
        mod.pressure.write_json(job/'request.json',request)
        state.update(status='finished_with_errors',terminal_episodes=40)
        mod.pressure.write_json(root/'run-state.json',state)
    with patch.object(mod.pressure,'verify_frozen'),patch.object(mod.time,'sleep',side_effect=finish_batch), \
         patch.object(mod,'evaluate_retry',return_value={}):
        with pytest.raises(ValueError,match='request identity'):
            mod.main()
    assert json.loads((job/'retry-state.json').read_text())['status']=='blocked'


def test_queued_request_mutation_blocks_while_formal_batch_still_runs(retry,monkeypatch):
    mod,root,_,_,_,_=retry;label='changed-while-queued';job=root/'retests'/label
    monkeypatch.setattr(sys,'argv',['pressure_retry','--root',str(root),'--case','cave200-r2',
        '--mode','remote','--label',label]);calls=[]
    def modify_request(_):
        calls.append(1)
        if len(calls)>1: raise RuntimeError('kept waiting after request mutation')
        request=json.loads((job/'request.json').read_text());request['authorization']='changed'
        mod.pressure.write_json(job/'request.json',request)
    with patch.object(mod.pressure,'verify_frozen'),patch.object(mod.time,'sleep',side_effect=modify_request):
        with pytest.raises(ValueError,match='request identity'):
            mod.main()
    assert json.loads((job/'retry-state.json').read_text())['status']=='blocked'


def test_unverifiable_proc_owner_is_not_treated_as_exited(retry):
    mod,root,protocol,owner,state,_=retry
    state.update(status='finished_with_errors',terminal_episodes=40)
    mod.pressure.write_json(root/'run-state.json',state)
    with patch.object(mod.ProcReader,'stat',side_effect=PermissionError('proc denied')):
        assert mod.batch_gate(root,protocol,owner)[0]=='blocked'


def test_state_write_failure_after_popen_still_closes_owned_worker(retry):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','failed-state-write')
    processes=[];real_popen=mod.subprocess.Popen
    def popen(*args,**kwargs):
        process=real_popen(*args,**kwargs);processes.append(process);return process
    try:
        with patch.object(mod,'evaluation_command',return_value=[sys.executable,'-c','import time;time.sleep(30)']), \
             patch.object(mod.subprocess,'Popen',side_effect=popen),patch.object(mod,'retry_state',side_effect=RuntimeError('state write failed')):
            with pytest.raises(RuntimeError,match='state write'):
                mod.evaluate_retry(root,job,protocol,case,'remote',poll_interval=.02)
        assert processes[0].poll() is not None
    finally:
        for process in processes:
            if process.poll() is None: process.kill();process.wait()


@pytest.mark.parametrize('parent_exit',[False,True])
def test_stuck_registered_native_is_closed_without_rewriting_valid_summary(retry,parent_exit):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','native-parent-'+str(parent_exit))
    marker=job/'native.pid'
    row=dict(status='finished',ended=1,error=None,mode='remote',family='cave',extent_m=200,
        seed=2026100633,budget=1536,provenance=protocol['frozen'],completed=True,final_coverage=.992)
    code=('import subprocess,json,os,sys,time; '
        'p=subprocess.Popen([sys.executable,"-c","import time;time.sleep(30)"]);'
        'open(sys.argv[1],"w").write(str(p.pid));time.sleep(.1);'
        'r=json.loads(sys.argv[2]);r["pid"]=os.getpid();open(sys.argv[3],"w").write(json.dumps(r));'
        + ('time.sleep(.05)' if parent_exit else 'time.sleep(30)'))
    try:
        with patch.object(mod,'evaluation_command',return_value=[sys.executable,'-c',code,str(marker),json.dumps(row),str(job/'evaluation'/'summary.json')]):
            result=mod.evaluate_retry(root,job,protocol,case,'remote',poll_interval=.02,native_timeout=.08)
        assert result['status']=='finished' and result['ended']==1 and result['final_coverage']==.992
        assert json.loads((job/'retry-state.json').read_text())['status']=='attention'
        with pytest.raises((OSError,ValueError)):
            mod.ProcReader().stat(int(marker.read_text()))
    finally:
        if marker.exists():
            pid=int(marker.read_text())
            try: os.kill(pid,signal.SIGKILL)
            except ProcessLookupError: pass


def test_cleanup_does_not_signal_reused_or_unrelated_pid(retry):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','reused-native-identity')
    process=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    try:
        reader=mod.ProcReader();start=reader.stat(process.pid)['start_ticks']
        mod.close_natives(job,reader,{process.pid:start+1})
        assert process.poll() is None
        assert json.loads((job/'native-cleanup.json').read_text())['actions']==[]
    finally: process.terminate();process.wait(timeout=2)


def test_native_exit_between_discovery_and_pidfd_open_is_clean(retry):
    mod,root,protocol,owner,_,case=retry
    job=mod.create_job(root,protocol,owner,case,'remote','native-natural-exit')
    process=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    try:
        reader=mod.ProcReader();start=reader.stat(process.pid)['start_ticks']
        def already_exited(_):
            process.terminate();process.wait(timeout=2)
            raise ProcessLookupError('native exited during discovery')
        with patch.object(mod.os,'pidfd_open',side_effect=already_exited):
            mod.close_natives(job,reader,{process.pid:start})
        assert json.loads((job/'native-cleanup.json').read_text())['remaining']==[]
    finally:
        if process.poll() is None: process.kill();process.wait()
