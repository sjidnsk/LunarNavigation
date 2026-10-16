from collections import Counter
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig,config_record,resume_semantics,training_config_from_record
TWO=('moon_outside','cave_inside')
def config():
    return TrainingConfig(course_two=True,course_two_variant='C3',course_two_task_scope='whole_cave_v1',course_two_sampling='scene_episode_v1',course_two_scene_tasks=TWO,environments=8)
def test_two_scene_dispatch_and_exact_restore():
    from lunar_drl_exploration.training import Curriculum
    c=config();course=Curriculum(c,np.random.default_rng(1));specs=[course.next(i,0) for i in range(8)]
    assert Counter(s['curriculum_task'] for s in specs)==dict.fromkeys(TWO,4)
    restored=Curriculum(c,np.random.default_rng(2),course.snapshot())
    assert [restored.next(i,0) for i in range(8)]==specs
    assert course.course.weights==dict.fromkeys(TWO,.5)
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    with pytest.raises(ValueError):SimpleCourse(course.course.state_dict())
def test_two_scene_semantics_and_json_roundtrip():
    c=config();assert training_config_from_record(config_record(c))==c
    assert resume_semantics(c)!=resume_semantics(replace(c,course_two_scene_tasks=('moon_outside','moon_inside','cave_inside')))
    assert resume_semantics(c)['course_two']['tasks']==list(TWO)
    profiled=replace(c,course_two_scene_profile='distance_sensor_10_30_v1')
    assert resume_semantics(profiled)['course_two']['scene']=='two_ordinary_distance_sensor_10_30_v1'
@pytest.mark.parametrize('tasks',[(),('moon_inside','cave_inside'),('moon_outside',),('moon_outside','moon_outside')])
def test_invalid_scene_scope_rejected(tasks):
    with pytest.raises(ValueError):replace(config(),course_two_scene_tasks=tasks)
def test_two_scene_replay_quota_diversity_exclusion_and_restore():
    from lunar_drl_exploration.replay import ReplayBuffer
    from test_course_two_replay import add_episode
    replay=ReplayBuffer(128*1024**2)
    for task in TWO:
        for i in range(8):
            rows=add_episode(replay,task,f'{task}{i}',count=8)
            if task=='cave_inside':
                key=rows[0].privileged.scene_id
                replay._scenes[key]=replace(replay._scenes[key],generator_descriptor=dict(curriculum_task=task,family='cave',task_scope='whole_cave_v1'))
    add_episode(replay,'moon_inside','excluded',count=8)
    kw=dict(task_scope='whole_cave_v1',sampling='scene_episode_v1',scene_tasks=TWO)
    index=replay.enable_course_two_sampling(**kw)
    assert index.status()['ready'] and index.status()['phase_quotas']==dict.fromkeys(TWO,'32')
    assert index.status()['scope_excluded_transitions']==8
    for n in range(4):
        batch=replay.sample_course_two(np.random.default_rng(n))
        assert Counter(replay.scenes[t.privileged.scene_id].generator_descriptor['curriculum_task'] for t in batch)==dict.fromkeys(TWO,32)
        assert max(Counter(t.episode_id for t in batch).values())<=4
    with pytest.raises(ValueError):replay.enable_course_two_sampling(task_scope='whole_cave_v1',sampling='scene_episode_v1')
    restored=ReplayBuffer.from_snapshot(replay.snapshot(32*1024**2,required_indices=index.required_indices()),replay.max_bytes)
    assert restored.enable_course_two_sampling(**kw).status()['ready']
    add_episode(restored,'moon_inside','excluded-new',count=1)
    assert restored.course_two_index.status()['scope_excluded_transitions']>=1

def test_two_scene_manifest_and_macro_gate():
    from test_region_relation_evaluation import fixture
    from lunar_drl_exploration.region_relation_evaluation import summarize,validate_manifest
    manifest,rows=fixture();manifest['active_tasks']=list(TWO)
    manifest['cases']=[c for c in manifest['cases'] if c['capability'] in TWO];rows=[r for r in rows if r['capability'] in TWO]
    assert len(validate_manifest(manifest))==120
    summary=summarize(manifest,rows);assert summary['candidate_gate_passed'] and set(summary['by_task'])==set(TWO)
    del manifest['active_tasks']
    with pytest.raises(ValueError):validate_manifest(manifest)
def test_new_preparation_uses_only_two_tasks_and_preserves_request_seeds(tmp_path,monkeypatch):
    from lunar_drl_exploration import region_relation_prepare as prep
    from lunar_drl_exploration.course_two_control import read
    monkeypatch.setattr(prep,'runtime_digest',lambda:'fixture')
    protocol=tmp_path/'protocol.json';protocol.write_text('{}');root=tmp_path/'run';pending=prep.prepare(root,protocol)
    assert len(pending['request_ids'])==384
    for split,count in [('validation',60),('test',100)]:
        requests=read(root/f'{split}-requests.json')['requests']
        assert Counter(r['case']['capability'] for r in requests)==dict.fromkeys(TWO,count)
        for task in TWO:
            actual=training_config_from_record(read(root/'config.json'))
            expected=prep._requests(replace(actual,course_two_scene_tasks=('moon_outside','moon_inside','cave_inside'),moon_entry_curriculum=''),task,count,split)
            assert [r for r in requests if r['case']['capability']==task]==expected

def test_full_checkpoint_two_scene_identity_rejects_three_scene_restore():
    import torch
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.training import Curriculum
    from lunar_drl_exploration.checkpoint import TrainingState,UpdateBoundary
    c=replace(config(),model=ModelConfig(width=16,heads=2,layers=1))
    learner=SACLearner(c.model,c.learning)
    state=TrainingState.capture(learner=learner,config=c,replay=ReplayBuffer(),schedule=UpdateSchedule(warmup=c.warmup),
        boundary=UpdateBoundary(),replay_rng=np.random.default_rng(1),curriculum_rng=np.random.default_rng(2),
        curriculum=Curriculum(c,np.random.default_rng(3)).snapshot(),
        collector_state={'policy_rng':torch.Generator().manual_seed(4).get_state(),'actor_record':learner.actor_state(),'actor_version':0},counters={})
    state.validate(c)
    with pytest.raises(ValueError,match='resume semantics'):
        state.validate(replace(c,course_two_scene_tasks=('moon_outside','moon_inside','cave_inside')))
