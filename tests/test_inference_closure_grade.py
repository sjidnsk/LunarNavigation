"""只检查实际闭环证据；不导入策略、ROS或训练。"""
import importlib.util
from pathlib import Path
from copy import deepcopy
import pytest


def grade(events, summary):
    path=Path(__file__).resolve().parents[1]/'scripts/drl/inference_closure_grade.py'
    assert path.is_file(), 'closed-loop acceptance gate missing'
    spec=importlib.util.spec_from_file_location('closure_grade',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.grade(events,summary)


def evidence():
    events=[]
    for i in range(1,4):
        goal=[float(i),0.,0.]
        events += [dict(kind='decision',index=i,stamp_ns=i*100,revision=i,selected_goal=goal),
                   dict(kind='goal',serial=i,goal=goal),
                   dict(kind='accepted',serial=i,session_id='uuid'+str(i),goal=goal)]
        if i<3:
            events.append(dict(kind='finish',serial=i,session_id='uuid'+str(i),accepted=True,goal=goal,
                outcome=0,reason_code='GOAL_REACHED',position_error_m=.03,yaw_error_rad=.02,
                velocity=[0.,0.],pose_age_s=.1,stamp_ns=i*100+10,revision=i))
    events += [dict(kind='cancel',serial=3,session_id='uuid3',stamp_ns=310),
        dict(kind='finish',serial=3,session_id='uuid3',accepted=True,goal=[3.,0.,0.],outcome=5,
             reason_code='CANCELED_STOPPED',velocity=[0.,0.],pose_age_s=.1,stamp_ns=320,revision=3),
        dict(kind='decision',index=4,stamp_ns=330,revision=4,selected_goal=[4.,0.,0.]),
        dict(kind='goal',serial=4,goal=[4.,0.,0.]),
        dict(kind='accepted',serial=4,session_id='uuid4',goal=[4.,0.,0.]),
        dict(kind='reset',task_id='reset-task',serial=4,session_id='uuid4',stamp_ns=335,accepted=True),
        dict(kind='cancel',serial=4,session_id='uuid4',stamp_ns=340),
        dict(kind='finish',serial=4,session_id='uuid4',accepted=True,goal=[4.,0.,0.],outcome=5,
             reason_code='CANCELED_STOPPED',velocity=[0.,0.],pose_age_s=.1,stamp_ns=350,revision=4)]
    summary=dict(frozen_unchanged=True,guard_attempts=[],stopped=True,inflight=False,state='CANCELED',
                 child_exits=dict(plant=0,mapper=0,planner=0,follower=0),forced_exit=False,workers_remaining=[])
    summary['guard_expected_processes']=[dict(pid=i,starttime=str(i*10)) for i in range(1,5)]
    summary['guard_startups']=[dict(kind='startup',pid=i,detail=dict(starttime=str(i*10))) for i in range(1,5)]
    return events,summary


def test_accepts_complete_feedback_chain():
    e,s=evidence();r=grade(e,s)
    assert r['ok'],r
    assert r['completed_goals']==2 and r['decisions']==4


def test_repeated_result_cannot_replace_second_goal():
    e,s=evidence();e=[x for x in e if not(x['kind']=='finish' and x['serial']==2)]
    e.append(deepcopy(next(x for x in e if x['kind']=='finish' and x['serial']==1)))
    assert not grade(e,s)['ok']


@pytest.mark.parametrize('change',[dict(outcome=5,reason_code='CANCELED_STOPPED'),dict(session_id='stale'),
 dict(position_error_m=.11),dict(yaw_error_rad=.06),dict(velocity=[.02,0.]),dict(pose_age_s=1.1),
 dict(position_error_m=float('nan')),dict(action_status=5),dict(goal=[999.,0.,0.])])
def test_invalid_success_is_not_arrival(change):
    e,s=evidence();next(x for x in e if x['kind']=='finish').update(change)
    assert not grade(e,s)['ok']


@pytest.mark.parametrize('change',['no_next','stale_stamp','no_revision','bad_cancel','bad_reset','missing_field'])
def test_incomplete_feedback_is_rejected(change):
    e,s=evidence()
    if change=='no_next': e=[x for x in e if not(x['kind']=='decision' and x['index']>=3)]
    if change=='stale_stamp':next(x for x in e if x['kind']=='decision' and x['index']>=3)['stamp_ns']=210
    if change=='no_revision':
        for x in e:x['revision']=1
    if change=='bad_cancel':next(x for x in e if x['kind']=='cancel')['session_id']='old'
    if change=='bad_reset':next(x for x in e if x['kind']=='reset')['stamp_ns']=1
    if change=='missing_field':del next(x for x in e if x['kind']=='finish')['velocity']
    assert not grade(e,s)['ok']


@pytest.mark.parametrize('field,value',[('frozen_unchanged',False),('guard_attempts',['replay']),('stopped',False),
 ('inflight',True),('state','RUNNING'),('child_exits',dict(plant=1)),('forced_exit',True),('workers_remaining',[12])])
def test_summary_integrity_is_required(field,value):
    e,s=evidence();s[field]=value
    assert not grade(e,s)['ok']


def test_missing_summary_field_fails_closed():
    e,s=evidence();del s['guard_attempts']
    assert not grade(e,s)['ok']


def test_reset_requires_its_own_cancel_completion():
    e,s=evidence();e=[x for x in e if not(x['kind']=='finish' and x['serial']==4)]
    assert not grade(e,s)['ok']


def test_goal_must_come_from_real_actor_decision():
    e,s=evidence();next(x for x in e if x['kind']=='decision')['selected_goal']=[-100.,0.,0.]
    assert not grade(e,s)['ok']


def test_first_feedback_also_requires_fresh_next_observation():
    e,s=evidence();next(x for x in e if x['kind']=='decision' and x['index']==2)['stamp_ns']=100
    assert not grade(e,s)['ok']


def test_reset_decision_cannot_replace_original_task_next_decision():
    e,s=evidence();e=[x for x in e if not(x['kind']=='decision' and x['index']==3)]
    assert not grade(e,s)['ok']


def test_well_formed_process_and_task_metadata_are_accepted():
    e,s=evidence()
    e.insert(0,dict(kind='process',name='plant',pid=123,command=['python','plant.py']))
    e.insert(1,dict(kind='task_published',command=1,task_id='task-one'))
    e.append(dict(kind='task_published',command=4,task_id='reset-task'))
    assert grade(e,s)['ok']


@pytest.mark.parametrize('event',[
    dict(kind='process',name='',pid=123,command=['python']),
    dict(kind='process',name='plant',pid=True,command=['python']),
    dict(kind='process',name='plant',pid=0,command=['python']),
    dict(kind='process',name='plant',pid=123,command='python'),
    dict(kind='process',name='plant',pid=123,command=[]),
    dict(kind='process',name='plant',pid=123,command=['python',None]),
    dict(kind='task_published',command=True,task_id='task'),
    dict(kind='task_published',command=2,task_id='task'),
    dict(kind='task_published',command=1,task_id=''),
    dict(kind='task_published',task_id='task'),
    dict(kind='unrecognized_metadata',name='plant'),
])
def test_malformed_or_unknown_metadata_remains_rejected(event):
    e,s=evidence();e.append(event)
    assert not grade(e,s)['ok']


@pytest.mark.parametrize('change',['missing_expected','missing_startups','lost_hook','wrong_starttime','duplicate_expected'])
def test_guard_requires_live_process_startup_coverage(change):
    e,s=evidence()
    if change=='missing_expected':del s['guard_expected_processes']
    if change=='missing_startups':del s['guard_startups']
    if change=='lost_hook':s['guard_startups'].pop()
    if change=='wrong_starttime':s['guard_startups'][-1]['detail']['starttime']='stale'
    if change=='duplicate_expected':s['guard_expected_processes']=[s['guard_expected_processes'][0]]*4
    assert not grade(e,s)['ok']
