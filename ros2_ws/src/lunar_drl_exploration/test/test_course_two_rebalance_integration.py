"""Learner/admission separation and genuine zero-update preparation recovery."""
from collections import Counter
from dataclasses import replace
import pytest


@pytest.mark.parametrize('variant', ['C0','C1','C2','C3'])
def test_admission_half_old_independent_of_learning(variant):
    from lunar_drl_exploration.course_two_rebalance import RebalancedCourse
    c=RebalancedCourse(variant)
    for env in range(16):c.activate(env,c.next_task())
    slots=Counter(c.active.values())
    assert sum(n for task,n in slots.items() if not task.startswith('c2_'))==8
    if variant in ('C2','C3'):
        assert slots['c2_A1_cover']==4 and slots['c2_A1_approach']==2 and slots['c2_A1_finish']==2
    for _ in range(128):c.admitted(c.select_pending(set(c.active),{}))
    assert sum(n for t,n in c.phase_transitions.items() if not t.startswith('c2_'))==64
    assert RebalancedCourse(variant,state=c.state_dict()).phase_transitions==c.phase_transitions
    with pytest.raises(ValueError,match='variant'):
        RebalancedCourse('C1' if variant!='C1' else 'C2',state=c.state_dict())


def test_c0_c1_c2_change_learning_not_admissions():
    from lunar_drl_exploration.course_two_rebalance import RebalancedCourse
    for variant,expected in [('C0',.8),('C1',.5),('C2',.5)]:
        course=RebalancedCourse(variant)
        counts={task:(900 if task.endswith('cover') else 100) for task in course.weights}
        weights=course.sampling_weights(counts)
        assert sum(v for k,v in weights.items() if k.startswith('c2_'))==pytest.approx(expected)
        if variant=='C2':assert weights['c2_A1_cover']/weights['c2_A1_finish']==pytest.approx(9)


def test_prefill_is_admitted_without_optimizer_credit_and_survives_restore():
    from lunar_drl_exploration.schedule import UpdateSchedule
    schedule=UpdateSchedule(warmup=0,ratio=.25,max_credit=2)
    for _ in range(32):
        schedule.reserve_dispatch(prefill=True)
        schedule.collected(reserved=True,prefill=True)
    assert schedule.transitions==32 and schedule.updates==0 and schedule.credit==0
    restored=UpdateSchedule.from_state_dict(schedule.state_dict())
    assert restored.prefill_transitions==32
    for _ in range(4):restored.collected()
    assert restored.credit==1
    restored.updated()
    assert restored.credit==0 and restored.updates==1


def test_rebalance_fork_changes_only_authorized_config_and_curriculum(ordered_checkpoint,tmp_path):
    from test_course_two_training import completed_source
    from test_training_fork import assert_state_equal
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.replay import loads_transport
    config,path,original=completed_source(ordered_checkpoint,tmp_path)
    dest=replace(config,course_two=True,ordered_curriculum=False,course_two_variant='C3',
        environments=16,course_two_catalog='/qualified.json',course_two_catalog_sha='a'*64,
        output_dir=str(tmp_path/'C3/run'),artifact_root=str(tmp_path/'C3'))
    fork=TrainingState.fork_from(path,dest,experiment='course-two-rebalance')
    for key in ('learner','replay','schedule','rng','collector_state'):
        assert_state_equal(fork.record[key],original.record[key])
    assert loads_transport(fork.record['curriculum'])['course']['variant']=='C3'
    with pytest.raises(ValueError,match='incompatible'):
        TrainingState.fork_from(path,replace(dest,completion_bonus=77),experiment='course-two-rebalance')


# Reuse the existing full-state fixture, including target and optimizer state.
from test_ordered_extent_fork import ordered_checkpoint


@pytest.mark.parametrize('simple',[False,True])
def test_single_c3_controller_initializes_directly_from_full_baseline(ordered_checkpoint,tmp_path,simple):
    import shutil
    from test_course_two_training import completed_source
    from test_training_fork import assert_state_equal
    from lunar_drl_exploration.config import config_record
    from lunar_drl_exploration.course_two_control import write
    from lunar_drl_exploration.course_two_trial import Trial, checkpoint
    from lunar_drl_exploration.replay import loads_transport
    config, source, original = completed_source(ordered_checkpoint,tmp_path)
    root = tmp_path/'single'
    (root/'frozen').mkdir(parents=True)
    shutil.copyfile(source,root/'frozen/source-resume.pt')
    write(root/'identity.json',dict(sampling='scene_episode_v1',task_scope='whole_cave_v1') if simple else {})
    write(root/'state.json',dict(status='ready',results={}))
    write(root/'config.json',config_record(config))
    if not simple:write(root/'rebuilt-catalog.json',dict(specs=[]))
    write(root/'training-settings.json',dict(workers=12))
    trial = Trial(root)
    destination = trial.initialize_arm('C3')
    assert destination.environments == 12
    restored = checkpoint(root/'arms/C3/training/run/resume.pt')
    restored.validate(destination)
    for key in ('learner','replay','schedule','rng','collector_state'):
        assert_state_equal(restored.record[key],original.record[key])
    course=loads_transport(restored.record['curriculum'])['course']
    if simple:
        assert course['schema']=='three_scene_course_v1'
        assert destination.course_two_sampling=='scene_episode_v1'
        assert not destination.course_two_catalog
    else:assert course['variant']=='C3'
    assert not (root/'arms/C2').exists()
    with pytest.raises(ValueError,match='C3'):
        trial.initialize_arm('C0')


def test_fresh_training_scenes_are_paired_bounded_and_never_recycle():
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.course_two_rebalance import fresh_spec
    config=TrainingConfig(course_two=True,course_two_variant='C2',course_two_catalog='/catalog',course_two_catalog_sha='a'*64)
    templates=[dict(role='finish',family='moon',seed=1,roi_side_interval_m=[32.,64.],
        planned_policy_start='outside',bearing_bin=0,sensor=dict(range_m=10.,fov_deg=90.),
        terrain_template='internal_occlusion',request_id='template')]
    first=fresh_spec(config,'c2_A1_finish',16,templates)
    paired=fresh_spec(replace(config,course_two_variant='C3'),'c2_A1_finish',16,templates)
    assert first==paired
    assert first!=fresh_spec(config,'c2_A1_finish',17,templates)
    case=first['course_two_case']
    assert case['role']=='return' and case['qualification_request']['split']=='train_continuation'
    assert case['sensor']['range_m']==10


def test_c2_new_half_is_uniform_over_transitions_not_fixed_role_rounding():
    import numpy as np
    from lunar_drl_exploration.replay import ReplayBuffer
    from test_course_two_replay import full_replay
    replay=full_replay()
    batches=[replay.sample_course_two_control(np.random.default_rng(i)) for i in range(6)]
    from lunar_drl_exploration.replay import transition_task
    counts=[Counter(transition_task(t,replay.scenes) for t in batch) for batch in batches]
    for count in counts:
        assert sum(count[t] for t in ('moon_inside','cave_inside','moon_outside','cave_outside'))==32
        assert len(count)>=5
    # The new half is sampled jointly; its role counts need not repeat exactly.
    assert len({tuple(sorted(c.items())) for c in counts})>1


def test_real_collector_prefill_saves_and_resumes_without_updates(tmp_path,monkeypatch):
    import numpy as np
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training,state_checksum
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration import course_two_rebalance as c2
    config=TrainingConfig(course_two=True,course_two_variant='C3',course_two_catalog='/test-catalog',
        course_two_catalog_sha='a'*64,environments=8,warmup=0,output_dir=str(tmp_path/'run'),
        system_reserve_bytes=0,save_interval_s=300.)
    catalog={task:[dict(seed=123+i,family='moon',extent=40.,start_mode='inside',
        curriculum_task=task,episode_budget=512)]*4 for i,task in enumerate(c2.RebalancedCourse('C3').weights)}
    monkeypatch.setattr(c2,'load_catalog',lambda _: (catalog,[]))
    kwargs=dict(collect_only=True,device='cpu',env_factory='worker_fixtures:ControlledCourseEnv',domain_base=157)
    result=run_training(config,max_transitions=16,**kwargs)
    before=CheckpointManager(config.output_dir).load(config)
    assert result['updates']==0 and before.record['schedule']['prefill_transitions']==16
    assert before.record['schedule']['credit']=='0'
    resumed=run_training(config,resume=True,max_transitions=8,**kwargs)
    after=CheckpointManager(config.output_dir).load(config)
    assert resumed['updates']==0 and after.record['schedule']['prefill_transitions']==24
    assert state_checksum(before.record['learner'])==state_checksum(after.record['learner'])


def test_slow_native_prefix_reset_survives_ordinary_watchdog_and_is_cancelable():
    import multiprocessing as mp,pickle,time,torch
    from lunar_drl_exploration.collector import collector_main
    from lunar_drl_exploration.config import TrainingConfig,config_record
    from lunar_drl_exploration.sac import SACLearner
    config=TrainingConfig(environments=1)
    actor=SACLearner().actor_state();parent,child=mp.get_context('spawn').Pipe()
    process=mp.get_context('spawn').Process(target=collector_main,
        args=(child,config_record(config),pickle.dumps(dict(actor_record=actor,actor_version=0,
            policy_rng=torch.get_rng_state(),sequences={}))),
        kwargs=dict(env_factory='worker_fixtures:ControlledSlowCourseResetEnv',worker_timeout_s=2.))
    process.start();child.close();seen=False
    spec=dict(seed=1,family='moon',extent=40,episode_budget=512,curriculum_task='c2_A1_finish',
        course_two_case=dict(version='region_sensor_rebalanced_scene_v2',prelude_goals=[[1,2,3]]))
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and not seen:
            if not parent.poll(.1):continue
            message=parent.recv()
            assert message['kind'] not in ('RECOVERY','FATAL'),message
            if message['kind']=='NEED_RESET':parent.send(dict(kind='RESET',env=0,spec=spec))
            if message['kind']=='SCENE':seen=True
        assert seen
        parent.send(dict(kind='RESET',env=0,spec=spec));time.sleep(.4)
        started=time.monotonic();parent.send(dict(kind='STOP'))
        stopped=False
        while time.monotonic()-started<5:
            if parent.poll(.1) and parent.recv()['kind']=='STOPPED':stopped=True;break
        assert stopped
        process.join(3);assert process.exitcode==0
    finally:
        if process.is_alive():process.kill()
        process.join(3);parent.close()
