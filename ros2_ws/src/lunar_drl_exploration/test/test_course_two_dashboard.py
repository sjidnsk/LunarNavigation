"""Monitor evidence must belong to the displayed evaluation and remain read-only."""
import importlib.util
import hashlib
import json
from pathlib import Path

import pytest


@pytest.fixture
def dashboard(monkeypatch):
    path = Path(__file__).resolve().parents[4] / 'scripts/drl/course_two_dashboard.py'
    monkeypatch.syspath_prepend(str(path.parent))
    spec = importlib.util.spec_from_file_location('course_two_dashboard_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def evaluation_state():
    return dict(status='running', stage='A1', phase='evaluation', stage_block=3,
        streak=0, pilot_checked=True, stage_start=254976, target_transitions=287744,
        evaluation_label='A1-block4', evaluation_done=38, evaluation_total=110,
        live_pid=123, child_pid=456, child_label='eval-A1-block4-A1-new-038',
        updated_unix=990, checkpoint=dict(path='/run/resume.pt', age_s=30),
        evaluation=dict(case_id='A1-new-038', actor='/run/A1-block4.pt',
            decisions=80, coverage=.97, updated_unix=999, file_age_s=1),
        progress=dict(transitions=287744, updates=71680,
            transitions_per_s=1.9, updates_per_s=.48, file_age_s=6000,
            environments={'0':dict(steps=444, curriculum_task='c2_A1',
                state='EXECUTING', reference_coverage=.15, budget=512)}))


@pytest.mark.parametrize('width,height', [(80,24),(124,34)])
def test_evaluation_progress_is_above_the_fold_without_stale_training_workers(dashboard, width, height):
    # An evaluation must not be buried below sixteen inactive training slots.
    lines = dashboard.layout(evaluation_state(),
        dict(baseline_transitions=254976, baseline_updates=63488),
        dict(environments=16), {}, width=width, height=height)
    visible = '\n'.join(text for text, _ in lines[:height-3])
    assert '38/110' in '\n'.join(text for text, _ in lines[:4])
    assert 'A1-new-038' in visible
    assert '444/512' not in visible
    assert '1.90 条/s' not in visible  # Last training speed is not live evaluation speed.
    assert all(dashboard.cells(text) < width for text, _ in lines)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


@pytest.fixture
def evaluation_root(tmp_path):
    root = tmp_path
    actor = root/'A1-block4.pt'
    actor.write_bytes(b'frozen actor four')
    write_json(root/'config.json', {'environments':16})
    write_json(root/'identity.json', {'runtime':'runtime-one'})
    cases = [dict(case_id=f'n{i}', kind='new', case=dict(family='moon',
        sensor=dict(range_m=10, fov_deg=90), episode_budget=512,
        roi_width_m=20, roi_height_m=30, bearing_bin=i)) for i in range(4)]
    cases += [dict(case_id='old0', kind='old', spec=dict(episode_budget=512)),
        dict(case_id='D1', kind='diagnostic', budget=32),
        dict(case_id='v0', kind='visible', case=dict(episode_budget=512))]
    write_json(root/'cases-A1.json', cases)
    for index, visible, completed in [(0,False,True),(1,True,True),(2,False,False),(5,False,True)]:
        case = cases[index]
        row = dict(case_id=case['case_id'], kind=case['kind'], completed=completed,
            collisions=0, error=None, initial_roi_visible=visible,
            first_roi=None if visible else {'decisions':2}, decisions=20 if completed else 512,
            final_coverage=1.0 if completed else .98,
            truncation_reason=None if completed else 'DECISION_BUDGET')
        proof = dict(actor_sha=hashlib.sha256(actor.read_bytes()).hexdigest(), case=case,
            config_sha=hashlib.sha256((root/'config.json').read_bytes()).hexdigest(), runtime='runtime-one')
        write_json(root/f'evaluation/A1-block4/{case["case_id"]}.json',
            dict(status='finished', row=row, proof=proof, wall_s=30))
    write_json(root/'evaluation-request.json', dict(actor=str(actor), case=cases[3]))
    state = evaluation_state()
    state.update(evaluation_done=4, evaluation_total=7, child_label='eval-A1-block4-n3')
    state['evaluation'].update(case_id='n3', actor=str(actor), decisions=17, coverage=.4)
    return root, state


def test_partial_results_have_separate_complete_and_initially_blind_denominators(dashboard, evaluation_root):
    root, state = evaluation_root
    before = {p: p.read_bytes() for p in root.rglob('*') if p.is_file()}
    panel = dashboard.EvaluationReader(root).snapshot(state, now=1000)
    assert panel['done'] == 4 and panel['total'] == 7
    assert not panel['complete']
    new = panel['groups']['new']
    assert (new['done'], new['total'], new['success'], new['failed']) == (3,4,2,1)
    assert (new['blind'], new['approached'], new['initial_visible']) == (2,2,1)
    assert panel['failures'] == {'DECISION_BUDGET':1}
    assert panel['groups']['old']['done'] == 0
    assert panel['current']['live']['decisions'] == 17
    assert panel['current']['budget'] == 512
    assert before == {p:p.read_bytes() for p in root.rglob('*') if p.is_file()}


@pytest.mark.parametrize('mismatch', ['case','actor','time'])
def test_previous_case_or_actor_progress_is_never_presented_as_current(dashboard, evaluation_root, mismatch):
    root, state = evaluation_root
    if mismatch == 'case': state['evaluation']['case_id'] = 'n2'
    elif mismatch == 'actor': state['evaluation']['actor'] = str(root/'A1-block3.pt')
    else: state['evaluation']['updated_unix'] = 989
    panel = dashboard.EvaluationReader(root).snapshot(state, now=1000)
    assert panel['current']['case_id'] == 'n3'
    assert panel['current']['live'] is None


def test_changed_or_broken_results_are_not_silently_counted_or_cached_forever(dashboard, evaluation_root):
    root, state = evaluation_root
    reader = dashboard.EvaluationReader(root)
    assert reader.snapshot(state, now=1000)['done'] == 4
    path = root/'evaluation/A1-block4/n0.json'
    record = json.loads(path.read_text())
    record['proof']['actor_sha'] = 'different-actor'
    write_json(path, record)
    panel = reader.snapshot(state, now=1000)
    assert panel['done'] == 3 and panel['warnings']
    path.write_text('{')
    panel = reader.snapshot(state, now=1000)
    assert panel['done'] == 3 and panel['warnings']
    assert not panel['complete']


def test_old_live_snapshot_keeps_its_age_warning(dashboard, evaluation_root):
    root, state = evaluation_root
    panel = dashboard.EvaluationReader(root).snapshot(state, now=1060)
    assert panel['current']['live']['decisions'] == 17
    assert panel['current']['age_s'] == 61
    assert panel['current']['stale']


def test_missing_evaluation_files_do_not_invent_success(dashboard, tmp_path):
    panel = dashboard.EvaluationReader(tmp_path).snapshot(evaluation_state(), now=1000)
    assert panel['done'] == 0
    assert not panel['complete'] and panel['warnings']


def test_finished_results_keep_the_recorded_gate_without_promoting_on_display(dashboard, evaluation_root):
    root, state = evaluation_root
    cases = json.loads((root/'cases-A1.json').read_text())
    template = json.loads((root/'evaluation/A1-block4/n0.json').read_text())
    for i in (3,4,6):
        record = json.loads(json.dumps(template))
        record['proof']['case'] = cases[i]
        record['row'].update(case_id=cases[i]['case_id'], kind=cases[i]['kind'])
        write_json(root/f'evaluation/A1-block4/{cases[i]["case_id"]}.json', record)
    rows = [json.loads((root/f'evaluation/A1-block4/{c["case_id"]}.json').read_text())['row'] for c in cases]
    summary = dict(status='finished', actor_sha=template['proof']['actor_sha'], rows=rows)
    write_json(root/'evaluation/A1-block4/summary.json', summary)
    state.update(status='needs_attention', phase='stage_budget_exhausted', stage_block=4,
        history=[dict(stage='A1', block=4, actor_sha=template['proof']['actor_sha'],
            passed=False, checks={'initial_visibility':False, 'coverage':True})])
    reader = dashboard.EvaluationReader(root)
    panel = reader.snapshot(state, now=1000)
    assert panel['complete'] and panel['done'] == 7
    assert panel['gate']['passed'] is False
    assert panel['history'][-1]['groups']['new']['success'] == 3
    assert panel['current']['live'] is None
    state['evaluation_panel'] = panel
    text = '\n'.join(line for line,_ in dashboard.layout(state,
        dict(baseline_transitions=254976, baseline_updates=63488),
        dict(environments=16), {}, width=124, height=34))
    assert '本轮晋级：未通过' in text and '初始可见性分类' in text
    # A stale aggregate cannot certify the current actor's results.
    summary['actor_sha'] = 'actor-from-another-round'
    write_json(root/'evaluation/A1-block4/summary.json', summary)
    panel = reader.snapshot(state, now=1000)
    assert not panel['complete'] and panel['gate'] is None


def test_last_evaluation_uses_its_own_stage_after_training_advances(dashboard, evaluation_root):
    root, state = evaluation_root
    state.update(stage='A2', phase='training')
    panel = dashboard.EvaluationReader(root).snapshot(state, now=1000)
    assert panel['stage'] == 'A1' and panel['done'] == 4
    assert panel['current']['live'] is None


def test_paused_evaluation_stays_visible_and_manual_training_view_is_available(dashboard):
    state = evaluation_state()
    state.update(status='paused', phase='saved_exit')
    assert dashboard.selected_view(state) == 'evaluation'
    assert dashboard.selected_view(state, 'training') == 'training'


def test_parallel_progress_keeps_each_request_and_job_separate(dashboard,evaluation_root):
    root,state=evaluation_root;cases=json.loads((root/'cases-A1.json').read_text())
    workers=[]
    for slot,case in enumerate((cases[3],cases[4])):
        request=f'live/{slot}/request.json';progress=f'live/{slot}/progress.json'
        write_json(root/request,dict(actor=str(root/'A1-block4.pt'),case=case))
        write_json(root/progress,dict(actor=str(root/'A1-block4.pt'),case_id=case['case_id'],
            slot=slot,job_id=f'job-{slot}',domain_id=220+slot,decisions=10+slot,coverage=.4,
            updated_unix=999))
        workers.append(dict(slot=slot,case_id=case['case_id'],pid=800+slot,domain_id=220+slot,
            request=request,progress=progress,job_id=f'job-{slot}',started_unix=990))
    state.update(evaluation_workers=workers,evaluation_concurrency=8,updated_unix=1000)
    reader=dashboard.EvaluationReader(root);panel=reader.snapshot(state,now=1001)
    assert len(panel['workers'])==2
    assert [w['live']['decisions'] for w in panel['workers']]==[10,11]
    assert [w['age_s'] for w in panel['workers']]==[2,2]
    state['evaluation_panel']=panel
    text='\n'.join(line for line,_ in dashboard.layout(state,
        dict(baseline_transitions=254976,baseline_updates=63488),dict(environments=16),{},width=124,height=34))
    assert '并行 2/8' in text and 'old0' in text and 'n3' in text
    wrong=json.loads((root/workers[0]['progress']).read_text());wrong['job_id']='previous-job'
    write_json(root/workers[0]['progress'],wrong)
    panel=reader.snapshot(state,now=1001)
    assert panel['workers'][0]['live'] is None
    assert panel['workers'][1]['live']['decisions']==11


def test_reassessment_never_counts_old_protocol_outputs(dashboard,evaluation_root):
    root,state=evaluation_root
    cases=json.loads((root/'cases-A1.json').read_text())
    write_json(root/'cases-A1-v2.json',cases)
    state.update(evaluation_directory='evaluation-v2',evaluation_case_file='cases-A1-v2.json',
        reevaluation=dict(labels=['baseline','A1-block4'],next=0),evaluation_workers=[])
    panel=dashboard.EvaluationReader(root).snapshot(state,now=1001)
    assert panel['done']==0 and not panel['complete']
