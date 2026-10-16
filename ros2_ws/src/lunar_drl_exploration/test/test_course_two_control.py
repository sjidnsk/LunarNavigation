"""Lifecycle ownership, finite budgets, and complete evaluation gates."""
import json
import os
import pytest


def test_remaining_budget_survives_pause_and_never_restarts_the_block():
    from lunar_drl_exploration.course_two_control import remaining_budget
    assert remaining_budget(257024,255000)==2024
    assert remaining_budget(257024,257024)==0
    with pytest.raises(ValueError):remaining_budget(257024,257025)


@pytest.mark.parametrize('transitions,credit,remaining,dispatch', [
    (7, '1/4', 3, True), (10, '3/4', 0, False), (10, '5/4', 0, True)])
def test_training_resume_accepts_fractional_credit_without_resetting_it(
        tmp_path, monkeypatch, transitions, credit, remaining, dispatch):
    from lunar_drl_exploration import course_two_control as ctl
    ctl.write(tmp_path/'state.json', dict(stage='A1', target_transitions=10))
    ctl.write(tmp_path/'identity.json', {})
    controller=ctl.Controller(tmp_path)
    checkpoints=iter([dict(transitions=transitions, credit=credit),
        dict(transitions=10, credit='1/4')])
    monkeypatch.setattr(ctl, 'schedule', lambda root: next(checkpoints))
    calls=[]
    def spawn(args, label, **env):
        calls.append((args, label, env))
        return True
    monkeypatch.setattr(controller, 'spawn', spawn)
    assert controller.train()
    assert bool(calls) is dispatch
    if dispatch:
        args, _, env=calls[0]
        assert args[args.index('--max-transitions')+1]==str(remaining)
        assert '--resume' in args and env['DRL_COURSE_TWO_STAGE']=='A1'


def test_lock_rejects_another_owner(tmp_path):
    from lunar_drl_exploration.course_two_control import root_lock
    with root_lock(tmp_path):
        with pytest.raises(RuntimeError,match='already'):
            with root_lock(tmp_path):pass


def test_stale_pid_is_never_signalled(tmp_path):
    from lunar_drl_exploration.course_two_control import request_stop
    (tmp_path/'owner.json').write_text(json.dumps(dict(pid=os.getpid(),start_ticks='wrong',root=str(tmp_path))))
    with pytest.raises(RuntimeError,match='identity'):request_stop(tmp_path)


def test_validation_pool_has_all_directions_and_sensor_combinations():
    from lunar_drl_exploration.course_two_control import validation_cases
    cases=validation_cases('C')
    current=[c for c in cases if c['kind']=='new' and c['case']['visibility_class']!='V']
    assert len(current)==200
    pairs={(c['case']['sensor']['range_m'],c['case']['sensor']['fov_deg']) for c in current}
    assert len(pairs)==25
    for pair in pairs:
        rows=[c for c in current if (c['case']['sensor']['range_m'],c['case']['sensor']['fov_deg'])==pair]
        assert {r['case']['bearing_bin'] for r in rows}==set(range(8))
        assert sum(r['case']['visibility_class']=='U' for r in rows)>=4
    assert sum(c['kind']=='old' for c in cases)==20
    assert sum(c['kind']=='diagnostic' for c in cases)==8


def test_partial_or_initial_only_results_cannot_pass():
    from lunar_drl_exploration.course_two_control import promotion_gate
    with pytest.raises(ValueError,match='incomplete'):
        promotion_gate([],[],stage='A1')
    row=dict(kind='new',case_id='n0',completed=True,collisions=0,error=None,
        initial_roi_visible=True,first_roi=None,decisions=0)
    with pytest.raises(ValueError,match='incomplete'):
        promotion_gate([row],[],stage='A1')


def test_range_anchor_gate_requires_all_easy_cases_and_real_approach():
    from lunar_drl_exploration.course_two_control import range_cases,range_gate
    cases=range_cases(40.)
    assert len(cases)==24
    assert all(c['case']['blind_gap_m']<=5 for c in cases)
    rows=[dict(case_id=c['case_id'],completed=True,collisions=0,error=None,
        initial_roi_visible=False,first_roi={'decisions':1}) for c in cases]
    assert range_gate(rows,40.)['passed']
    with pytest.raises(ValueError,match='incomplete'):range_gate(rows[:-1],40.)
    rows[0]['initial_roi_visible']=True
    assert not range_gate(rows,40.)['passed']


def test_unverified_sensor_stage_never_autostarts(tmp_path,monkeypatch):
    from lunar_drl_exploration import course_two_control as ctl
    monkeypatch.setattr(ctl,'runtime_digest',lambda:'v1')
    assert not ctl.sensor_stage_ready(tmp_path,'B')
    ctl.write(tmp_path/'sensor-readiness.json',dict(runtime='v1',ready_stages=['A1','A2']))
    assert ctl.sensor_stage_ready(tmp_path,'A1')
    assert not ctl.sensor_stage_ready(tmp_path,'B')
    monkeypatch.setattr(ctl,'runtime_digest',lambda:'v2')
    assert not ctl.sensor_stage_ready(tmp_path,'A1')


def test_a1_approach_pool_is_balanced_and_keeps_fov_checks_separate():
    from collections import Counter
    from lunar_drl_exploration.course_two_control import validation_cases
    cases=validation_cases('A1')
    approach=[c['case'] for c in cases if c['kind']=='new']
    assert len(approach)==80
    assert {c['visibility_class'] for c in approach}=={'U'}
    assert Counter((c['bearing_bin'],c['family']) for c in approach)==Counter({
        (direction,family):5 for direction in range(8) for family in ('moon','cave')})
    orientation=[c['case'] for c in cases if c['kind']=='visible' and c['case']['visibility_class']=='F']
    assert len(orientation)==16
    assert {(c['bearing_bin'],c['family']) for c in orientation}=={
        (direction,family) for direction in range(8) for family in ('moon','cave')}


def test_corrected_gate_can_pass_and_still_rejects_initially_visible_approach():
    from lunar_drl_exploration.course_two_control import validation_cases,promotion_gate
    cases=validation_cases('A1');rows=[]
    for c in cases:
        row=dict(case_id=c['case_id'],kind=c['kind'],completed=True,collisions=0,error=None,
            decisions=4,initial_roi_visible=c['kind']=='visible',initial_coverage=.1 if c['kind']=='visible' else 0.,
            first_roi=None if c['kind']=='visible' else dict(decisions=2))
        if c['kind']=='new':
            spec=c['case'];row.update(bearing_bin=spec['bearing_bin'],range_m=spec['sensor']['range_m'],fov_deg=spec['sensor']['fov_deg'])
        rows.append(row)
    baseline=[dict(r) for r in rows if r['kind'] in ('old','diagnostic')]
    gate=promotion_gate(rows,baseline,stage='A1')
    assert gate['passed'] and gate['approach_rate']==1.
    rows[0]['initial_roi_visible']=True
    assert not promotion_gate(rows,baseline,stage='A1')['passed']
    rows[0]['initial_roi_visible']=False
    rows[0]['kind']='visible'
    with pytest.raises(ValueError,match='identity'):
        promotion_gate(rows,baseline,stage='A1')


def test_policy_start_contract_reports_invalid_evaluation_not_policy_failure():
    from lunar_drl_exploration.course_two_eval import validate_policy_start
    validate_policy_start(dict(case_id='u',kind='new',case={'visibility_class':'U'}),0.)
    with pytest.raises(ValueError,match='initialization'):
        validate_policy_start(dict(case_id='u',kind='new',case={'visibility_class':'U'}),.01)
    validate_policy_start(dict(case_id='f',kind='visible',case={'visibility_class':'F'}),.25)


def test_reassessment_uses_only_frozen_actors_and_never_auto_trains_or_promotes(tmp_path,monkeypatch):
    from lunar_drl_exploration import course_two_control as ctl
    ctl.write(tmp_path/'identity.json',{})
    ctl.write(tmp_path/'state.json',dict(status='running',stage='A1',phase='evaluation',streak=0,
        evaluation_case_file='cases-A1-v2.json',evaluation_directory='evaluation-v2',history=[],
        reevaluation=dict(labels=['baseline','A1-block1','A1-block2'],next=0)))
    cases=ctl.validation_cases('A1');ctl.write(tmp_path/'cases-A1-v2.json',cases)
    (tmp_path/'frozen').mkdir();(tmp_path/'frozen/block31.pt').write_bytes(b'baseline')
    for label in ('A1-block1','A1-block2'):(tmp_path/(label+'.pt')).write_bytes(label.encode())
    controller=ctl.Controller(tmp_path);calls=[]
    def screen(actor,requested,label):
        calls.append(label)
        assert requested==cases and actor.is_file()
        return [dict(case_id=c['case_id']) for c in requested]
    monkeypatch.setattr(controller,'screen',screen)
    monkeypatch.setattr(controller,'train',lambda:pytest.fail('reassessment must not train'))
    monkeypatch.setattr(ctl,'promotion_gate',lambda *a,**k:dict(passed=True,checks={}))
    controller.reevaluate()
    assert calls==['baseline','A1-block1','A1-block2']
    assert controller.state['status']=='needs_attention'
    assert controller.state['phase']=='reevaluation_complete'
    assert controller.state['stage']=='A1' and controller.state['streak']==0
