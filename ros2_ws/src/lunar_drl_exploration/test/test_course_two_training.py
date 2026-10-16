"""New curriculum preserves learned state and actually admits/samples new tasks."""
from dataclasses import replace
import hashlib
import numpy as np
import pytest
from test_ordered_extent_fork import ordered_checkpoint
from test_training_fork import assert_state_equal


def completed_source(fixture, tmp_path):
    import torch
    from lunar_drl_exploration.config import config_record, resume_semantics
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    from lunar_drl_exploration.replay import loads_transport, dumps_transport
    config, _, state = fixture
    config = replace(config, policy_graph_mode='a2')
    state.record.update(config=config_record(config), semantics=resume_semantics(config))
    curriculum = loads_transport(state.record['curriculum'])
    course = curriculum['course']
    for phase in (1,2,3): course=OrderedCourse(phase, state=course).state_dict()
    curriculum['course']=course
    state.record['curriculum']=dumps_transport(curriculum)
    path=tmp_path/'completed.pt'
    torch.save(state.record,path)
    return config,path,state


def test_explicit_course_two_fork_preserves_full_learning_and_old_replay(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.replay import loads_transport
    config,path,state=completed_source(ordered_checkpoint,tmp_path)
    before=hashlib.sha256(path.read_bytes()).hexdigest()
    dest=replace(config,course_two=True,ordered_curriculum=False,
        output_dir=str(tmp_path/'second/run'),artifact_root=str(tmp_path/'second'))
    fork=TrainingState.fork_from(path,dest,experiment='course-two')
    fork.validate(dest)
    for key in ('learner','replay','schedule','rng','collector_state'):
        assert_state_equal(state.record[key],fork.record[key])
    curriculum=loads_transport(fork.record['curriculum'])
    assert curriculum['course']['task']=='c2_A1'
    assert curriculum['course_one_history']['phase']==3
    assert curriculum['episodes_issued']==9
    assert fork.record['fork']['source_checkpoint_sha256']==before
    assert hashlib.sha256(path.read_bytes()).hexdigest()==before
    with pytest.raises(ValueError,match='incompatible'):
        TrainingState.fork_from(path,replace(dest,completion_bonus=17),experiment='course-two')
    with pytest.raises(ValueError,match='incompatible'):
        TrainingState.fork_from(path,replace(dest,environments=config.environments+1),experiment='course-two')


def test_quota_admission_and_batch_do_not_silently_use_only_old_data():
    from lunar_drl_exploration.course_two import CourseTwo
    c=CourseTwo('A1')
    for env,task in enumerate(c.weights): c.activate(env,task)
    for _ in range(100):
        env=c.select_pending(set(c.active),{})
        assert env is not None
        c.admitted(env)
    assert c.phase_transitions['c2_A1']==80
    assert sum(c.phase_transitions[k] for k in c.weights if not k.startswith('c2_'))==20
    counts={k:3 for k in c.weights if not k.startswith('c2_')}
    assert not c.ready_for_learning(counts)
    counts['c2_A1']=64
    assert c.ready_for_learning(counts)
    assert c.sampling_weights(counts)['c2_A1']==pytest.approx(.8)
    restored=CourseTwo('A1',state=c.state_dict())
    assert restored.phase_transitions==c.phase_transitions
    later=CourseTwo('A2',state=c.state_dict())
    assert later.weights['c2_A2']==pytest.approx(.6)
    assert later.weights['c2_A1']==pytest.approx(.2)


@pytest.mark.parametrize('environments', [8, 12])
def test_short_new_episodes_do_not_give_their_parallel_slots_to_slow_rehearsal(environments):
    from collections import Counter
    from lunar_drl_exploration.course_two import CourseTwo
    from lunar_drl_exploration.ordered_curriculum import TASKS
    c=CourseTwo('A1')
    for env in range(environments):c.activate(env,c.next_task())
    assert Counter(c.active.values())==dict(c2_A1=environments-4,moon_inside=1,
        cave_inside=1,moon_outside=1,cave_outside=1)
    # A1 briefly leads the admission quota while long old episodes are active.
    # Restarting short A1 episodes must not duplicate those waiting old tasks.
    c.phase_transitions.update({task:4 for task in TASKS},c2_A1=80)
    fast_slots=[env for env,task in c.active.items() if task=='c2_A1']
    for _ in range(3):
        for env in fast_slots:
            c.release(env);c.activate(env,c.next_task())
    assert Counter(c.active.values())==dict(c2_A1=environments-4,moon_inside=1,
        cave_inside=1,moon_outside=1,cave_outside=1)
    # Once all labels catch up, admission still maintains the original 80/20.
    c.phase_transitions=dict.fromkeys(c.weights,0)
    for _ in range(100):c.admitted(c.select_pending(set(c.active),{}))
    assert c.phase_transitions==dict(c2_A1=80,moon_inside=5,
        cave_inside=5,moon_outside=5,cave_outside=5)


@pytest.fixture
def unbalanced_course_two_checkpoint(ordered_checkpoint, tmp_path, monkeypatch):
    from copy import deepcopy
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.training import Curriculum
    from lunar_drl_exploration.replay import loads_transport, dumps_transport
    from lunar_drl_exploration.reward_diagnostics import RewardDiagnostics
    from test_replay_checkpoint import transition
    monkeypatch.setenv('DRL_COURSE_TWO_STAGE', 'A1')
    fixture_config, fixture_path, fixture_state = ordered_checkpoint
    config, path, _ = completed_source((replace(fixture_config, reward_diagnostics=True),
        fixture_path, fixture_state), tmp_path)
    config = replace(config, course_two=True, ordered_curriculum=False,
        output_dir=str(tmp_path/'second/run'), artifact_root=str(tmp_path/'second'),
        reward_diagnostics=True)
    state = TrainingState.fork_from(path, config, experiment='course-two')
    curr = Curriculum(config, np.random.default_rng(50), loads_transport(state.record['curriculum']))
    for env in range(8): curr.next(env, 7)
    saved = curr.snapshot()
    # Reproduce a saved allocation with one new-task worker and seven old ones.
    for env in (5, 6, 7):
        saved['active_specs'][str(env)] = deepcopy(saved['active_specs']['1'])
        saved['active_specs'][str(env)]['seed'] += 100+env
    state.record['curriculum'] = dumps_transport(saved)
    counters = loads_transport(state.record['counters'])
    counters['course'] = deepcopy(saved['course'])
    diag = RewardDiagnostics(8, config.completion_bonus, config.success_coverage)
    diag.observe(6, transition(), before_coverage=.96, after_coverage=.97,
        family='moon', extent_m=60., seed=19)
    counters['reward_diagnostics'] = diag.state_dict()
    state.record['counters'] = dumps_transport(counters)
    return config, state


def test_explicit_resize_rebalances_scenes_without_losing_learning_or_diagnostics(
        unbalanced_course_two_checkpoint, tmp_path):
    import torch
    from collections import Counter
    from copy import deepcopy
    from lunar_drl_exploration.course_two import reconfigure_checkpoint
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.training import Curriculum
    from lunar_drl_exploration.replay import loads_transport
    from lunar_drl_exploration.reward_diagnostics import RewardDiagnostics
    from lunar_drl_exploration.sac import SACLearner
    config, state = unbalanced_course_two_checkpoint
    before = deepcopy(state.record)
    expanded = replace(config, environments=12)
    migrated, audit = reconfigure_checkpoint(state, expanded)
    assert_state_equal(state.record, before)
    for key in ('learner', 'replay', 'schedule', 'rng', 'collector_state', 'fork'):
        assert_state_equal(before[key], migrated.record[key])
    old_curr = loads_transport(before['curriculum'])
    curr = loads_transport(migrated.record['curriculum'])
    assert Counter(s['curriculum_task'] for s in curr['active_specs'].values()) == dict(
        c2_A1=8, moon_inside=1, cave_inside=1, moon_outside=1, cave_outside=1)
    for env in range(5):
        assert curr['active_specs'][str(env)] == old_curr['active_specs'][str(env)]
    assert set(audit['retired_specs']) == {'5', '6', '7'}
    assert set(audit['issued_specs']) == {'5', '6', '7', '8', '9', '10', '11'}
    assert curr['episodes_issued'] == old_curr['episodes_issued']+7
    assert len({spec['seed'] for spec in curr['active_specs'].values()}) == 12
    assert curr['course'] == old_curr['course']
    old_counts = loads_transport(before['counters'])
    counts = loads_transport(migrated.record['counters'])
    for key in old_counts.keys()-{'reward_diagnostics'}:
        assert counts[key] == old_counts[key]
    old_diag = old_counts['reward_diagnostics']
    diag = counts['reward_diagnostics']
    assert diag == dict(old_diag, environments=12, active=old_diag['active']+[None]*4)
    restored_diag = RewardDiagnostics(12, config.completion_bonus, config.success_coverage, diag).state_dict()
    assert restored_diag['totals'] == old_diag['totals']
    assert restored_diag['segments_closed_by_reason']['RESUME'] == 1
    # Exercise the real serialized restore path, including optimizers and RNG.
    path = tmp_path/'expanded.pt'
    torch.save(migrated.record, path)
    learner = SACLearner(expanded.model, expanded.learning)
    restored = TrainingState(torch.load(path, weights_only=False)).restore(learner, expanded)
    assert_state_equal(before['learner'], learner.state_dict())
    resumed = Curriculum(expanded, restored.curriculum_rng, restored.curriculum)
    for env in reversed(range(12)):
        assert resumed.next(env, 7) == curr['active_specs'][str(env)]
    assert resumed.state['episodes_issued'] == curr['episodes_issued']
    with pytest.raises(ValueError, match='incompatible'):
        state.validate(expanded)


def test_resize_rejects_learning_changes_shrink_and_stage_change(unbalanced_course_two_checkpoint, monkeypatch):
    from lunar_drl_exploration.course_two import reconfigure_checkpoint
    config, state = unbalanced_course_two_checkpoint
    for changed in (replace(config, environments=7), replace(config, environments=12, completion_bonus=17),
            replace(config, environments=12, learning=replace(config.learning, gamma=.9))):
        with pytest.raises(ValueError): reconfigure_checkpoint(state, changed)
    monkeypatch.setenv('DRL_COURSE_TWO_STAGE', 'A2')
    with pytest.raises(ValueError, match='stage'):
        reconfigure_checkpoint(state, replace(config, environments=12))


def test_curriculum_restores_pending_scene_without_changing_sensor(monkeypatch):
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import Curriculum
    from copy import deepcopy
    monkeypatch.setenv('DRL_COURSE_TWO_STAGE','A1')
    config=replace(TrainingConfig(),course_two=True)
    rng=np.random.default_rng(50)
    curr=Curriculum(config,rng)
    spec=curr.next(0,0)
    assert spec['course_two_case']['sensor']['range_m']==10
    state=curr.snapshot()
    recovered=Curriculum(config,np.random.default_rng(),state)
    same=recovered.next(0,0)
    assert same==spec
    assert recovered.state['episodes_issued']==1
    assert recovered.next(1,0)['seed']!=spec['seed']


def test_last_stage_can_dispatch_with_fewer_slots_than_labels():
    from lunar_drl_exploration.course_two import CourseTwo
    c=CourseTwo('D')
    c.activate(0,'c2_D');c.activate(1,'moon_inside')
    for _ in range(30):
        env=c.select_pending({0,1},{})
        assert env is not None
        c.admitted(env)


def test_new_stage_starts_new_tasks_instead_of_restoring_six_old_episodes(monkeypatch):
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import Curriculum
    monkeypatch.setenv('DRL_COURSE_TWO_STAGE','A1')
    config=replace(TrainingConfig(),course_two=True,environments=6)
    current=Curriculum(config,np.random.default_rng(50))
    old=[current.next(env,0) for env in range(6)]
    saved=current.snapshot()
    monkeypatch.setenv('DRL_COURSE_TWO_STAGE','A2')
    new=Curriculum(config,np.random.default_rng(51),saved)
    scenes=[new.next(env,0) for env in range(6)]
    assert 'c2_A2' in {s['curriculum_task'] for s in scenes}
    assert not {s['seed'] for s in old}&{s['seed'] for s in scenes}
    for _ in range(107):new.course.admitted(new.course.select_pending(set(new.course.active),{}))
    assert new.course.ready_for_learning({task:1 for task in new.course.weights})


def test_first_region_observation_counted_once_and_not_from_initial_visibility():
    from lunar_drl_exploration.course_two import CourseTwo
    course=CourseTwo('A1');course.activate(0,'c2_A1')
    course.record_progress(0,{'first_roi':None})
    assert course.by_task['c2_A1']['first_roi']==0
    for _ in range(2):course.record_progress(0,{'first_roi':{'decision':3}})
    assert course.by_task['c2_A1']['first_roi']==1


def test_zero_action_initial_completion_is_separate_from_policy_success():
    from lunar_drl_exploration.course_two import CourseTwo
    course=CourseTwo('A1');course.activate(0,'c2_A1')
    course.observe(0,terminated=True,truncated=False,coverage=1.,collision=False)
    assert course.by_task['c2_A1']['successes']==0
    assert course.by_task['c2_A1']['initial_completed']==1
    assert course.by_task['c2_A1']['zero_decisions']==1
    assert course.by_task['c2_A1']['ended']==1


def test_range_curriculum_uses_only_explicitly_enabled_easy_anchor(monkeypatch):
    from lunar_drl_exploration.training import Curriculum
    from lunar_drl_exploration.config import TrainingConfig
    monkeypatch.setenv('DRL_COURSE_TWO_STAGE','B')
    monkeypatch.setenv('DRL_COURSE_TWO_RANGE_LIMIT','40')
    monkeypatch.setenv('DRL_COURSE_TWO_RANGE_EASY','1')
    c=Curriculum(replace(TrainingConfig(),course_two=True,environments=6),np.random.default_rng(1))
    scene=c.next(0,0)['course_two_case']
    assert scene['sensor']['range_m']==40.
    assert scene['blind_gap_m']<=5.
    state=c.snapshot()
    monkeypatch.delenv('DRL_COURSE_TWO_RANGE_LIMIT')
    monkeypatch.delenv('DRL_COURSE_TWO_RANGE_EASY')
    resume=Curriculum(c.config,np.random.default_rng(1),state)
    assert resume.next(0,0)['course_two_case']==scene
    monkeypatch.setenv('DRL_COURSE_TWO_RANGE_LIMIT','70')
    d=Curriculum(c.config,np.random.default_rng(1),state)
    assert d.next(0,0)['course_two_case']['sensor']['range_m']==70.
