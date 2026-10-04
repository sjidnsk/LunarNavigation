from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts/drl/task_frontier'))
from course_controller import default_plan, validate_plan, course_cases, promotion_gate, remaining_training

def rows(task, success=4, loop=0):
    return [dict(curriculum_task=task, completed=i<success, decisions=100, zero_gain_two_point_loop_steps=loop, collisions=0) for i in range(5)]

def test_gate_requires_success_and_loops_and_old_retention():
    plan=default_plan()
    assert promotion_gate(rows('moon_inside'),0,plan)['passed']
    assert not promotion_gate(rows('moon_inside',3),0,plan)['passed']
    assert not promotion_gate(rows('moon_inside',4,21),0,plan)['passed']
    assert not promotion_gate(rows('cave_inside')+rows('moon_inside',3),1,plan)['passed']
    assert promotion_gate(rows('cave_inside')+rows('moon_inside'),1,plan)['passed']

def test_cases_five_current_five_previous_and_modes_frozen():
    plan=default_plan()
    assert len(course_cases(0,1,plan))==5
    a=course_cases(3,1,plan);b=course_cases(3,2,plan)
    assert len(a)==10
    assert [r['budget'] for r in a[:5]]==[256,288,352,448,512]
    assert all(r['start_mode']=='outside' and r['family']=='cave' for r in a[:5])
    assert {r['curriculum_task'] for r in a[5:]}=={'moon_inside','cave_inside','moon_outside'}
    assert a==course_cases(3,1,plan)
    assert a[:5]==b[:5] and a[5:]!=b[5:]

def test_limits_and_plan_validation():
    p=default_plan();assert validate_plan(p)==p
    p['max_blocks']=[20,0,20,10]
    with pytest.raises(ValueError):validate_plan(p)

def test_admission_boundary_resumes_credit_without_more_collection():
    assert remaining_training(9216,2048,dict(transitions=9000,updates=1900,credit='0'))==216
    assert remaining_training(9216,2048,dict(transitions=9216,updates=2047,credit='1'))==0
    assert remaining_training(9216,2048,dict(transitions=9216,updates=2048,credit='0')) is None
    with pytest.raises(ValueError):remaining_training(9216,2048,dict(transitions=9217,updates=2048,credit='0'))

def test_course_evaluation_rejects_start_mode_cache_confusion():
    from controller import validate_evaluation
    from test_task_frontier_controller import paired_fixture
    cases,record=paired_fixture();cases=cases[:1];record['cases']=record['cases'][:1]
    cases[0].update(start_mode='inside',curriculum_task='moon_inside')
    record['cases'][0].update(start_mode='outside',curriculum_task='moon_inside')
    with pytest.raises(ValueError,match='start_mode'):
        validate_evaluation(record,actor_sha='actor',semantic_id='semantic',cases=cases)

def test_promotion_is_two_consecutive_passes_and_limit_saves():
    from course_controller import initial_state, advance_state
    plan=default_plan();s=initial_state(plan)
    good=dict(passed=True);bad=dict(passed=False)
    advance_state(s,good,plan,'a');assert s['phase']==0 and s['streak']==1
    advance_state(s,bad,plan,'b');assert s['phase']==0 and s['streak']==0
    advance_state(s,good,plan,'c');advance_state(s,good,plan,'d')
    assert s['phase']==1 and s['phase_blocks']==0 and s['blocks']==4 and s['streak']==0
    plan['max_blocks'][1]=1
    advance_state(s,bad,plan,'e');assert s['status']=='needs_attention' and s['phase']==1

def test_monitor_shows_coverage_and_durable_successes(tmp_path):
    from course_dashboard import lines
    progress=dict(transitions=100,updates=1,environments={'0':dict(curriculum_task='moon_inside',extent_m=50.,coverage_ratio=.75,steps=12,budget=256,state='READY')})
    course=dict(phase=0,by_task={'moon_inside':dict(transitions=100,samples=64,ended=2,successes=1,episode_steps=80)},phase_transitions={'moon_inside':100},phase_samples={'moon_inside':64})
    value='\n'.join(lines(tmp_path,dict(course_phase=0,phase='training'),progress,course,{}))
    assert '75.00%' in value and '12/256' in value and '100.0%' in value and '1/2' in value

def test_monitor_does_not_label_previous_phase_shares_as_current(tmp_path):
    from course_dashboard import lines
    course=dict(phase=0,by_task={},phase_transitions={'moon_inside':100},phase_samples={'moon_inside':64})
    progress=dict(course=course,environments={})
    value='\n'.join(lines(tmp_path,dict(course_phase=1,phase='training'),progress,course,{}))
    assert '等待本阶段统计' in value and '100.0%' not in value

def test_review_gate_binds_course_plan_before_first_run(tmp_path,monkeypatch):
    import controller as c
    root=tmp_path/'owned';root.mkdir()
    c.write(root/'config.json',dict(ordered_curriculum=True))
    c.write(root/'course-plan.json',default_plan())
    c.write(root/'review-gate.json',dict(source_commit='commit',config_sha=c.sha(root/'config.json'),
        native_smoke=dict(status='passed',source_commit='commit'),independent_audit=dict(status='passed',source_commit='commit'),
        course_plan_sha=c.sha(root/'course-plan.json')))
    monkeypatch.setattr(c,'source_commit',lambda:'commit')
    monkeypatch.setattr(c.subprocess,'check_output',lambda *args,**kwargs:'')
    c.verify_gate(root)
    plan=c.read(root/'course-plan.json');plan['max_blocks'][0]=99;c.write(root/'course-plan.json',plan)
    with pytest.raises(ValueError,match='course plan'):c.verify_gate(root)
