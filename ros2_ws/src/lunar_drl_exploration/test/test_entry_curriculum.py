from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig,resume_semantics

MODE='near_to_far_v1'

def test_guided_mix_promotions_use_actual_entry_and_efficiency():
    from lunar_drl_exploration.entry_curriculum import EntryCurriculum
    c=EntryCurriculum(MODE)
    original=dict(curriculum_task='moon_outside',outside_distance_min_m=10.,outside_distance_max_m=50.)
    specs=[c.apply(original,i) for i in range(10)]
    assert sum(s['outside_distance_min_m']==2 for s in specs)==7
    assert original['outside_distance_min_m']==10
    for _ in range(49):c.observe(0,50,3.,dict(decisions=2,distance_m=4.),False)
    assert c.stage==0
    c.observe(0,50,3.,dict(decisions=2,distance_m=4.),False)
    assert c.stage==1 and c.promotions[-1]['reason']=='mastery'
    # Earlier-stage episodes finishing late do not advance the new stage.
    for _ in range(60):c.observe(0,50,3.,dict(decisions=2,distance_m=4.),False)
    assert c.stats['1']['count']==0
    assert sum(c.apply(original,i)['outside_distance_min_m']==5 for i in range(10))==5
    for _ in range(50):c.observe(1,100,10.,dict(decisions=5,distance_m=15.),False)
    assert c.stage==2 and c.apply(original,0)==original

def test_seeing_region_or_slow_entry_does_not_pass_gate_and_caps_are_explicit():
    from lunar_drl_exploration.entry_curriculum import EntryCurriculum
    c=EntryCurriculum(MODE)
    for _ in range(50):c.observe(0,100,3.,None,False)
    assert c.stage==0
    for _ in range(50):c.observe(0,100,3.,dict(decisions=50,distance_m=80.),False)
    assert c.stage==0
    c.advance(8192);assert c.stage==1 and c.promotions[-1]['reason']=='admission_cap'
    c.advance(32768);assert c.stage==2 and c.promotions[-1]['reason']=='admission_cap'

def test_restore_is_exact_and_rejects_corrupt_or_wrong_mode():
    from lunar_drl_exploration.entry_curriculum import EntryCurriculum
    c=EntryCurriculum(MODE)
    for _ in range(12):c.observe(0,10,3.,dict(decisions=2,distance_m=4.),False)
    c.advance(100);saved=c.state_dict();assert EntryCurriculum(MODE,saved).state_dict()==saved
    bad=deepcopy(saved);bad['stage']=3
    with pytest.raises(ValueError):EntryCurriculum(MODE,bad)
    bad=deepcopy(saved);bad['stats']['0']['recent'][0]['entered']=False
    with pytest.raises(ValueError):EntryCurriculum(MODE,bad)
    c.observe(0,0,3.,None,False);assert c.stats['0']['count']==12

def test_collision_after_entry_counts_entry_but_not_mastery():
    from lunar_drl_exploration.entry_curriculum import EntryCurriculum
    c=EntryCurriculum(MODE)
    c.observe(0,10,3.,dict(decisions=2,distance_m=4.),True)
    assert c.stats['0']==dict(count=1,entered=1,efficient=0,recent=[dict(entered=True,efficient=False)])

def test_training_integrates_guidance_without_changing_fixed_scene_generation():
    from lunar_drl_exploration.region_relation_run import make_config
    from lunar_drl_exploration.training import Curriculum
    from lunar_drl_exploration.course_two_simple import fresh_scene
    from lunar_drl_exploration.region_relation_prepare import _requests
    c=make_config('/tmp/entry-fixture','a'*64)
    assert c.moon_entry_curriculum==MODE
    legacy=replace(c,moon_entry_curriculum='')
    assert resume_semantics(c)!=resume_semantics(legacy)
    course=Curriculum(c,np.random.default_rng(1));spec=course.next(0,0)
    assert spec['curriculum_task']=='moon_outside' and spec['outside_distance_min_m']==2.
    assert spec['sensor']==fresh_scene(c,'moon_outside',0)['sensor']
    assert _requests(c,'moon_outside',3,'validation')==_requests(legacy,'moon_outside',3,'validation')
    saved=course.snapshot();restored=Curriculum(c,np.random.default_rng(2),saved)
    assert restored.next(0,0)==spec
    with pytest.raises(ValueError):Curriculum(legacy,np.random.default_rng(2),saved)


def test_course_counts_actual_entry_only_preserves_reset_tag_and_excludes_zero_actions():
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    c=SimpleCourse(tasks=('moon_outside','cave_inside'),entry_mode=MODE)
    spec=dict(curriculum_task='moon_outside',outside_distance_min_m=2.,outside_distance_max_m=5.)
    c.activate(0,'moon_outside');c.bind_entry_spec(0,spec);c.reset_episode(0)
    c.admitted(0);c.record_progress(0,dict(first_roi={'decisions':1},start_boundary_distance_m=3.,first_entry=None))
    c.observe(0,terminated=False,truncated=True,coverage=.3,collision=False)
    assert c.entry.stats['0']['count']==1 and c.entry.stats['0']['entered']==0
    c.activate(0,'moon_outside');c.bind_entry_spec(0,spec)
    c.observe(0,terminated=True,truncated=False,coverage=1.,collision=False)
    assert c.entry.stats['0']['count']==1
    for _ in range(50):
        c.activate(0,'moon_outside');c.bind_entry_spec(0,spec);c.admitted(0)
        c.record_progress(0,dict(start_boundary_distance_m=3.,first_entry={'decisions':1,'distance_m':4.}))
        c.observe(0,terminated=True,truncated=False,coverage=.99,collision=False)
    assert c.entry.stage==1
    assert SimpleCourse(c.state_dict(),tasks=c.tasks,entry_mode=MODE).state_dict()==c.state_dict()
