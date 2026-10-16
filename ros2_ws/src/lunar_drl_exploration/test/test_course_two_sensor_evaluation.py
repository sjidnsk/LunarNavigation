from copy import deepcopy
from collections import Counter
from dataclasses import replace
import pytest
from test_course_two_distance_sensor import config,PROFILE
from lunar_drl_exploration.course_two_simple import development_cases,TASKS
from test_course_two_trial_dashboard import ui,root,write,sha


def test_development_retains_original_scenes_and_covers_sensor_combinations():
    cases=development_cases(config());old=development_cases(replace(config(),course_two_scene_profile=''))
    previous={c['case_id']:c for c in old}
    assert len(cases)==60 and len({c['spec']['seed'] for c in cases})==60
    for task in TASKS:
        group=[c for c in cases if c['capability']==task]
        assert Counter(c['evaluation_role'] for c in group)==dict(regression=5,adaptation=15)
        for c in group:
            assert c['scene_profile']==PROFILE
            if c['evaluation_role']=='regression':assert c['spec']==previous[c['case_id']]['spec']
        varied=[c for c in group if c['evaluation_role']=='adaptation']
        assert {(c['spec']['sensor']['range_m'],c['spec']['sensor']['fov_deg']) for c in varied}=={
            (r,f) for r in (10.,20.,30.) for f in (60.,90.,120.,180.,360.)}
        if task=='moon_outside':assert all(c['spec']['outside_distance_min_m']==10. for c in varied)


def records():
    return [dict(case_id=c['case_id'],kind='simple',capability=c['capability'],
        scene_profile=PROFILE,evaluation_role=c['evaluation_role'],completed=True,
        collisions=0,error=None,decisions=10,initial_completed=False,
        sensor=c['spec'].get('sensor',dict(range_m=10.,fov_deg=90.))) for c in development_cases(config())]


def test_initial_completion_excluded_from_policy_denominator_and_pair_efficiency():
    from lunar_drl_exploration.course_two_trial import gate
    rows=records()
    for r in rows:
        if r['sensor']['range_m']==30.:r.update(initial_completed=True,decisions=0)
    result=gate(rows,deepcopy(rows));assert result['passed']
    for g in result['development'].values():
        assert g['total']==15 and g['success']==15 and g['initial_completed']==5
        assert g['median_decision_ratio']==1.
    changed=deepcopy(rows);changed[0]['completed']=False
    assert gate(changed,rows)['stop'], 'one lost original scene must fail regression retention'
    changed=deepcopy(rows);changed[0]['initial_completed']=True
    assert gate(changed,rows)['stop'], 'changed initialization cannot be credited as learning'


def test_all_initially_complete_is_not_evidence_of_policy_success():
    from lunar_drl_exploration.course_two_trial import gate
    rows=records()
    for r in rows:r.update(initial_completed=True,decisions=0)
    result=gate(rows,deepcopy(rows))
    assert not result['passed']
    assert all(g['total']==0 and g['success']==0 for g in result['development'].values())


def test_evaluation_metadata_keeps_sensor_scope_even_when_reset_fails():
    # This helper is used before reset; a failed reset must retain its planned group.
    from lunar_drl_exploration.course_two_eval import case_metadata
    case=development_cases(config())[7]
    assert case_metadata(case)==dict(capability=case['capability'],scene_profile=PROFILE,evaluation_role='adaptation')


def test_panel_separates_initial_completion_and_shows_sensor_progress(ui,root):
    write(root/'identity.json',dict(runtime='runtime-one',sampling='scene_episode_v1',scene_profile=PROFILE))
    cases=development_cases(config())[:3];write(root/'cases.json',cases)
    for i,c in enumerate(cases):
        row=dict(case_id=c['case_id'],kind='simple',capability=c['capability'],scene_profile=PROFILE,
            evaluation_role=c['evaluation_role'],completed=i<2,initial_completed=i==0,
            initial_roi_visible=i==0,first_roi=dict(decisions=2) if i==1 else None,
            initial_coverage=1. if i==0 else 0.,collisions=0,error=None,decisions=0 if i==0 else 10,
            sensor=dict(range_m=30. if i==0 else 10.,fov_deg=360. if i==0 else 90.))
        proof=dict(runtime='runtime-one',actor_sha=sha(root/'frozen/block31.pt'),config_sha=sha(root/'config.json'),case=c)
        write(root/'evaluation/baseline'/(c['case_id']+'.json'),dict(status='finished',proof=proof,row=row,wall_s=3))
    s=ui.Reader(root,lambda _:None).snapshot();g=s['baseline']['groups']['moon_outside']
    assert (g['done'],g['success'],g['failed'],g['initial_completed'],g['policy_total'])==(3,1,1,1,2)
    text='\n'.join(t for t,_ in ui.evaluation_table(s,144,'baseline'))
    assert '50.0%' in text and '初始完成' in text and '360' in text
