"""Real subprocess scheduling: overlap, bounded ownership, interruption and provenance."""
import json
from pathlib import Path
import signal
import sys
import threading

import pytest

from lunar_drl_exploration import course_two_control as ctl


@pytest.fixture
def pool(tmp_path, monkeypatch):
    from lunar_drl_exploration import course_two_parallel as parallel
    ctl.write(tmp_path/'state.json', dict(status='running', stage='A1'))
    ctl.write(tmp_path/'identity.json', dict(runtime='test-runtime'))
    ctl.write(tmp_path/'config.json', {})
    ctl.write(tmp_path/'evaluation-settings.json', dict(workers=2))
    (tmp_path/'logs').mkdir()
    actor=tmp_path/'actor.pt';actor.write_bytes(b'frozen actor')
    monkeypatch.setattr(ctl, 'runtime_digest', lambda:'test-runtime')
    worker=tmp_path/'worker.py'
    worker.write_text('''import hashlib,json,pathlib,sys,time
request,output=map(pathlib.Path,sys.argv[1:3])
r=json.loads(request.read_text());case=r['case'];root=pathlib.Path(sys.argv[3])
started=time.time();cid=case['case_id']
(root/(cid+'.started')).write_text(json.dumps(dict(start=started,pid=__import__('os').getpid(),request=r)))
time.sleep(.1 if cid=='bad' else .35)
if cid=='bad':sys.exit(7)
row=dict(case_id=cid,kind=case['kind'],completed=True,collisions=0,error=None,start=started,end=time.time())
proof=dict(actor_sha=hashlib.sha256(pathlib.Path(r['actor']).read_bytes()).hexdigest(),case=case,
 config_sha=hashlib.sha256((root/'config.json').read_bytes()).hexdigest(),runtime='test-runtime')
output.write_text(json.dumps(dict(status='finished',proof=proof,row=row)))
''')
    commands=[]
    def command(root, request, output, *, slot, progress, job_id):
        commands.append(dict(slot=slot, request=request, output=output, progress=progress, job_id=job_id))
        return [sys.executable,str(worker),str(request),str(output),str(root)]
    monkeypatch.setattr(parallel, 'evaluation_command', command)
    cases=[dict(case_id=f'c{i}', kind='new') for i in range(6)]
    return parallel, ctl.Controller(tmp_path), actor, cases, commands


def assert_reaped(root):
    for path in root.glob('*.started'):
        assert ctl.process_ticks(ctl.read(path)['pid']) is None


def test_cases_overlap_with_a_bound_and_resume_skips_verified_results(pool):
    _, controller, actor, cases, commands=pool
    rows=controller.screen(actor,cases,'round')
    assert [r['case_id'] for r in rows]==[c['case_id'] for c in cases]
    events=sorted([(r['start'],1) for r in rows]+[(r['end'],-1) for r in rows])
    active=peak=0
    for _, change in events:
        active+=change;peak=max(peak,active)
    assert peak==2
    assert {c['slot'] for c in commands}=={0,1}
    for key in ('request','progress','job_id'):
        assert len({str(c[key]) for c in commands})==len(cases)
    snapshots={p:p.read_bytes() for p in (controller.root/'evaluation').rglob('*.json')}
    commands.clear()
    assert controller.screen(actor,cases,'round')==rows
    assert not commands
    assert snapshots=={p:p.read_bytes() for p in snapshots}
    assert controller.state['evaluation_done']==6
    assert controller.state['evaluation_workers']==[]
    assert_reaped(controller.root)


def test_pause_stops_all_children_without_inventing_finished_cases(pool):
    _, controller, actor, cases, _=pool
    timer=threading.Timer(.15,controller.interrupt);timer.start()
    try:
        assert controller.screen(actor,cases,'round') is None
    finally:timer.cancel()
    assert not (controller.root/'evaluation/round/summary.json').exists()
    assert controller.state['evaluation_workers']==[]
    assert_reaped(controller.root)
    controller.stopping=False
    assert len(controller.screen(actor,cases,'round'))==6


def test_infrastructure_failure_stops_other_children_and_never_writes_summary(pool):
    _, controller, actor, cases, _=pool
    cases[1]['case_id']='bad'
    with pytest.raises(RuntimeError,match='exited 7'):
        controller.screen(actor,cases,'round')
    assert not (controller.root/'evaluation/round/summary.json').exists()
    assert_reaped(controller.root)


def test_old_protocol_results_cannot_be_reused_as_repaired_evaluation(pool):
    parallel, controller, actor, cases, _=pool
    path=controller.root/'evaluation/old/c0.json'
    expected=dict(actor_sha=ctl.sha(actor),case=cases[0],config_sha=ctl.sha(controller.root/'config.json'),runtime='test-runtime')
    record=dict(status='finished',proof=dict(expected,runtime='old-runtime'),row=dict(case_id='c0',kind='new'))
    ctl.write(path,record)
    with pytest.raises(ValueError,match='identity'):
        parallel.cached_result(controller.root,path,expected)
    record['proof']=dict(expected);ctl.write(path,record)
    assert parallel.cached_result(controller.root,path,expected)==record
    with pytest.raises(ValueError,match='identity'):
        parallel.cached_result(controller.root,path,dict(expected,actor_sha='other'))
    record['proof']['case']=cases[1];ctl.write(path,record)
    with pytest.raises(ValueError,match='identity'):
        parallel.cached_result(controller.root,path,expected)


@pytest.mark.parametrize('workers',[0,17,True,1.5])
def test_invalid_parallelism_cannot_launch(pool,workers):
    _, controller, actor, cases, commands=pool
    ctl.write(controller.root/'evaluation-settings.json',dict(workers=workers))
    with pytest.raises(ValueError,match='workers'):
        controller.screen(actor,cases,'round')
    assert not commands


def test_sixteen_evaluation_workers_use_valid_distinct_domains(pool):
    parallel, controller, actor, _, commands = pool
    ctl.write(controller.root/'evaluation-settings.json',dict(workers=16))
    cases=[dict(case_id=f'scaled-{i}',kind='new') for i in range(16)]
    rows=controller.screen(actor,cases,'scaled')
    assert len(rows)==16
    assert {c['slot'] for c in commands}==set(range(16))
    domains={parallel.evaluation_domain(i) for i in range(16)}
    assert domains==set(range(216,232))
    assert controller.state['evaluation_concurrency']==16
    assert_reaped(controller.root)


def test_worker_counts_and_slots_accept_sixteen_but_reject_seventeen(tmp_path):
    from lunar_drl_exploration.course_two_parallel import worker_count, validate_slot
    path=tmp_path/'settings.json';ctl.write(path,dict(workers=16))
    assert worker_count(path)==16 and validate_slot(15)==15
    with pytest.raises(ValueError,match='slot'):validate_slot(16)


def test_explicit_hash_bound_baseline_adoption_preserves_original_proof(tmp_path):
    from lunar_drl_exploration.course_two_parallel import cached_result
    case=dict(case_id='c0',kind='simple')
    old=dict(actor_sha='actor',config_sha='config',runtime='before',case=case)
    proof=dict(old,runtime='after');path=tmp_path/'evaluation/baseline/c0.json'
    result=dict(status='finished',proof=old,row=dict(case_id='c0',kind='simple',error=None))
    ctl.write(path,result);ctl.write(tmp_path/'identity.json',{})
    with pytest.raises(ValueError):cached_result(tmp_path,path,proof)
    adoption=dict(sha=ctl.sha(path),from_runtime='before',to_runtime='after')
    ctl.write(tmp_path/'identity.json',dict(evaluation_cache_adoptions={
        'evaluation/baseline/c0.json':adoption}))
    assert cached_result(tmp_path,path,proof)==result
    assert ctl.read(path)['proof']['runtime']=='before'
    with pytest.raises(ValueError):cached_result(tmp_path,path,dict(proof,actor_sha='different'))
    ctl.write(path,dict(result,wall_s=4.))
    with pytest.raises(ValueError):cached_result(tmp_path,path,proof)


@pytest.fixture
def domain_busy_result(pool):
    parallel, controller, actor, cases, commands = pool
    proof = dict(actor_sha=ctl.sha(actor), config_sha=ctl.sha(controller.root/'config.json'),
                 runtime='test-runtime', case=cases[0])
    record = dict(status='failed', proof=proof, row=dict(
        case_id='c0', kind='new', completed=False, initial_completed=False,
        collisions=0, error='DomainBusy: evaluation domain lease is occupied',
        trajectory=[]))
    path = controller.root/'evaluation/round/c0.json'
    ctl.write(path, record)
    return parallel, controller, actor, cases, commands, path, record


def test_domain_busy_counts_failure_and_continues_without_rewriting_result(domain_busy_result):
    _, controller, actor, cases, commands, path, record = domain_busy_result
    ctl.write(controller.root/'evaluation-settings.json', dict(workers=1, error_policy='count_failure'))
    before = path.read_bytes()
    rows = controller.screen(actor, cases, 'round')
    assert [row['case_id'] for row in rows] == [case['case_id'] for case in cases]
    failed = rows[0]
    assert failed['completed'] is False
    assert failed['error'] == record['row']['error']
    assert failed['evaluation_failure_kind'] == 'infrastructure'
    assert failed['truncation_reason'] == 'INFRASTRUCTURE_ERROR'
    assert failed['final_coverage'] is None
    assert failed['coverage_source'] == 'unavailable'
    assert failed['collisions'] is None
    assert sum(row.get('evaluation_failure_kind') == 'infrastructure' for row in rows) == 1
    assert sum(row['completed'] for row in rows) == 5
    assert controller.state['evaluation_done'] == 6
    assert (controller.root/'c5.started').exists()
    assert len(commands) == 5
    assert path.read_bytes() == before
    assert ctl.read(path) == record
    assert ctl.read(path.parent/'summary.json')['rows'] == rows
    assert_reaped(controller.root)
    snapshots = {p: p.read_bytes() for p in path.parent.glob('*.json')}
    commands.clear()
    assert controller.screen(actor, cases, 'round') == rows
    assert not commands
    assert {p: p.read_bytes() for p in snapshots} == snapshots


def test_domain_busy_stop_policy_still_raises(domain_busy_result):
    _, controller, actor, cases, commands, path, _ = domain_busy_result
    before = path.read_bytes()
    ctl.write(controller.root/'evaluation-settings.json', dict(workers=1, error_policy='stop'))
    with pytest.raises(RuntimeError, match='DomainBusy'):
        controller.screen(actor, cases, 'round')
    assert not commands
    assert path.read_bytes() == before
    assert not (path.parent/'summary.json').exists()


@pytest.mark.parametrize('field', ['actor_sha', 'config_sha', 'runtime', 'case'])
def test_domain_busy_proof_errors_still_raise(domain_busy_result, field):
    parallel, controller, _, cases, _, path, record = domain_busy_result
    ctl.write(controller.root/'evaluation-settings.json', dict(workers=1, error_policy='count_failure'))
    expected = dict(record['proof'])
    expected[field] = cases[1] if field == 'case' else 'foreign'
    before = path.read_bytes()
    with pytest.raises(ValueError, match='identity'):
        parallel.cached_result(controller.root, path, expected)
    assert path.read_bytes() == before


@pytest.mark.parametrize('error', [
    'ValueError: damaged record', 'AssertionError: invalid checkpoint',
    'DomainBusy without prefix delimiter', 'DomainBusyOther: unrelated error',
])
def test_non_domain_busy_data_failures_still_raise(domain_busy_result, error):
    parallel, controller, _, _, _, path, record = domain_busy_result
    ctl.write(controller.root/'evaluation-settings.json', dict(workers=1, error_policy='count_failure'))
    record['row']['error'] = error
    ctl.write(path, record)
    before = path.read_bytes()
    with pytest.raises(RuntimeError, match='evaluation infrastructure error'):
        parallel.cached_result(controller.root, path, record['proof'])
    assert path.read_bytes() == before
