from dataclasses import replace
import copy
import pytest
import torch
from test_sac import transitions
from test_model import scene
from lunar_drl_exploration.config import ModelConfig,LearningConfig
from lunar_drl_exploration.sac import SACLearner


def test_update_unchanged_matches_legacy(monkeypatch):
    import lunar_drl_exploration.sac as module
    calls={'obs':0,'truth':0}
    pack_obs,pack_truth=module.pack_observations,module.pack_privileged
    # Clone the pre-change update into a function as an independent oracle.
    import ast,pathlib
    oracle=pathlib.Path(__file__).with_name('legacy_sac_update.py')
    namespace=dict(vars(module));exec(compile(oracle.read_text(),str(oracle),'exec'),namespace)
    torch.manual_seed(41)
    cfg=LearningConfig(microbatch_size=8,entropy_mode='position_heading')
    a=SACLearner(ModelConfig(width=16,heads=2,layers=1),cfg,device='cpu')
    b=SACLearner(a.model_config,cfg,device='cpu');b.load_state_dict(a.state_dict())
    samples=transitions();registry={'s':scene()}
    old=namespace['legacy_update'](a,samples,registry)
    def counted_obs(*args,**kw):calls['obs']+=1;return pack_obs(*args,**kw)
    def counted_truth(*args,**kw):calls['truth']+=1;return pack_truth(*args,**kw)
    monkeypatch.setattr(module,'pack_observations',counted_obs)
    monkeypatch.setattr(module,'pack_privileged',counted_truth)
    new=b.update(samples,registry)
    # The rejected reuse experiment is disabled; original packing cadence remains.
    assert calls=={'obs':24,'truth':24}
    for key in old:assert new[key]==pytest.approx(old[key],rel=1e-6,abs=1e-7)
    from test_training_fork import assert_state_equal
    assert_state_equal(a.state_dict(),b.state_dict())


def test_dispatch_candidates_fill_slots_and_reselect_after_reservation():
    from lunar_drl_exploration.training import dispatch_candidates
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    course=SimpleCourse();course.active={0:'moon_outside',1:'moon_outside',2:'moon_inside',3:'moon_inside',4:'cave_inside',5:'cave_inside'}
    pending={i:i+100 for i in range(6)};reservations={};seen=[]
    for env,token in dispatch_candidates(pending,reservations,course):
        seen.append(env);reservations[env]=token
    assert seen==[0,2,4,1,3,5]
    assert pending=={i:i+100 for i in range(6)}
    assert len(set(seen))==6


def test_dispatch_preserves_bootstrap_missing_and_credit_budget():
    from lunar_drl_exploration.training import dispatch_candidates,within_admission_budget
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    from lunar_drl_exploration.schedule import UpdateSchedule
    c=SimpleCourse();c.active={0:'moon_inside',1:'moon_outside',2:'cave_inside',3:'moon_outside'}
    assert [e for e,_ in dispatch_candidates(dict.fromkeys(range(4),1),{},c,bootstrap_missing=['moon_outside'])]==[1,3]
    schedule=UpdateSchedule(0,.25,1);pending={i:i for i in range(4)};reservations={};seen=[]
    for e,t in dispatch_candidates(pending,reservations,c):
        if not within_admission_budget(schedule.transitions,schedule.inflight,baseline=0,maximum=2):break
        if not schedule.can_dispatch():break
        schedule.reserve_dispatch();reservations[e]=t;seen.append(e)
    assert len(seen)==2 and schedule.inflight==2
    assert [e for e,_ in dispatch_candidates(pending,{0:1},None)]==[1,2,3]



def test_dispatch_credit_backpressure_stops_at_exact_reservation_limit():
    from lunar_drl_exploration.training import dispatch_candidates
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    from lunar_drl_exploration.schedule import UpdateSchedule
    course=SimpleCourse();course.active={i:'moon_outside' for i in range(8)}
    schedule=UpdateSchedule(0,.25,1);reservations={}
    for env,token in dispatch_candidates({i:i for i in range(8)},reservations,course):
        if not schedule.can_dispatch():break
        schedule.reserve_dispatch();reservations[env]=token
    assert len(reservations)==4 and schedule.inflight==4
    assert not schedule.can_dispatch()
