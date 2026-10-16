"""A2 changes experience, while the complete learned SAC history survives."""
from dataclasses import replace
from fractions import Fraction
import copy
import hashlib

import numpy as np
import pytest

from test_training_fork import assert_state_equal


@pytest.fixture
def a2_source(tmp_path,request):
    import torch
    from lunar_drl_exploration.config import TrainingConfig, ModelConfig
    from lunar_drl_exploration.checkpoint import CheckpointManager, UpdateBoundary, TrainingState
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.schedule import UpdateSchedule
    from test_replay_checkpoint import scene, transition
    torch.set_num_threads(1)
    model = ModelConfig() if getattr(request,'param',None) == 'full' else ModelConfig(width=16,heads=2,layers=1)
    config = replace(TrainingConfig(), model=model,
        learning=replace(TrainingConfig().learning, entropy_mode='position_heading'),
        ordered_curriculum=True, ordered_rehearsal_by_phase=(.2,.5,.2,.2),
        environments=3, warmup=0, system_reserve_bytes=0,
        curriculum_extents_m=((30.,100.),)*3,
        output_dir=str(tmp_path/'source/run'), artifact_root=str(tmp_path/'source'))
    learner = SACLearner(config.model, config.learning)
    replay = ReplayBuffer(config.replay_max_bytes)
    replay.add(transition(), {'s':replace(scene(), generator_descriptor={'curriculum_task':'moon_inside'})})
    schedule = UpdateSchedule(0)
    for _ in range(11): schedule.collected()
    learner.update(replay.sample(64,np.random.default_rng(2)),replay.scenes)
    schedule.updated()
    with torch.no_grad():
        learner.log_alpha.fill_(-25.)
        learner.log_alpha_heading.fill_(-26.)
    course = OrderedCourse(1,.5)
    course.by_task['moon_inside']['transitions']=5
    course.by_task['moon_inside']['ended']=1
    course.by_task['moon_inside']['episode_steps']=5
    course.by_task['cave_inside']['transitions']=6
    course.phase_transitions.update(moon_inside=5,cave_inside=6)
    course.coverage_stats['moon_inside']=dict(count=1,sum=.8,recent=[.8])
    state = TrainingState.capture(learner=learner,config=config,replay=replay,
        schedule=schedule,boundary=UpdateBoundary(),replay_rng=np.random.default_rng(2),
        curriculum_rng=np.random.default_rng(3),curriculum=dict(episodes_issued=9,course=course.state_dict()),
        collector_state=dict(actor_record=learner.actor_state(),actor_version=learner.updates,
            policy_rng=torch.Generator().manual_seed(4).get_state(),sequences={0:11,1:7,2:3}),
        counters=dict(transitions=11,last_admitted={0:11,1:7,2:3},course=course.state_dict()))
    source=CheckpointManager(config.output_dir).save(state)
    return config,source,state


def destination(config,tmp_path,**changes):
    return replace(config,output_dir=str(tmp_path/'a2/run'),artifact_root=str(tmp_path/'a2'),
        policy_graph_mode='a2',**changes)


def test_ordered_a2_fork_keeps_all_learned_state_and_discards_only_replay(a2_source,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState, CheckpointManager
    from lunar_drl_exploration.replay import loads_transport, snapshot_sample_count
    from lunar_drl_exploration.sac import SACLearner
    config,source,original=a2_source
    changed=destination(config,tmp_path)
    sha=hashlib.sha256(source.read_bytes()).hexdigest()
    fork=TrainingState.fork_from(source,changed,experiment='ordered-a2')
    for key in ('learner','schedule','rng','curriculum','collector_state'):
        assert_state_equal(original.record[key],fork.record[key])
    counters=loads_transport(fork.record['counters'])
    for key,value in loads_transport(original.record['counters']).items():
        assert_state_equal(value,counters[key])
    bootstrap=counters['replay_bootstrap']
    assert bootstrap['active'] is True and bootstrap['admitted']==0 and bootstrap['required']==64
    assert bootstrap['source_credit']=='7/4' and bootstrap['source_checkpoint_sha256']==sha
    assert snapshot_sample_count(fork.record['replay'])==0
    assert fork.record['fork']['source_checkpoint_sha256']==sha
    assert set(fork.record['fork']['changes'])=={'artifact_root','policy_graph_mode'}
    CheckpointManager(changed.output_dir).save(fork)
    restored=CheckpointManager(changed.output_dir).load(changed).restore(SACLearner(changed.model,changed.learning),changed)
    assert len(restored.replay)==0 and restored.schedule.credit==Fraction(7,4)
    assert_state_equal(original.record['learner'],fork.record['learner'])
    assert hashlib.sha256(source.read_bytes()).hexdigest()==sha
    with pytest.raises(ValueError,match='resume.*policy_graph'): original.validate(changed)
    with pytest.raises(ValueError,match='resume.*policy_graph'): fork.validate(config)


def test_default_fine_config_still_restores_pre_a2_checkpoint(a2_source):
    from lunar_drl_exploration.config import resume_semantics
    config,_,state=a2_source
    state.record['config'].pop('policy_graph_mode',None)
    state.record['config'].pop('remote_frontier_positions',None)
    assert 'policy_graph' not in resume_semantics(config)
    state.validate(config)


@pytest.mark.parametrize('field,value', [('warmup',5),('completion_bonus',99.),('max_update_credit',33),
    ('ordered_rehearsal_by_phase',(.2,.4,.2,.2)),('remote_frontier_positions',31),('snapshot_max_bytes',1000000)])
def test_a2_fork_allowlist_rejects_unrelated_or_unaccepted_changes(a2_source,tmp_path,field,value):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_=a2_source
    with pytest.raises(ValueError,match='fork|ordered-a2'):
        TrainingState.fork_from(source,destination(config,tmp_path,**{field:value}),experiment='ordered-a2')


def test_empty_replay_credit_exception_needs_valid_a2_bootstrap(a2_source,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.replay import dumps_transport,loads_transport,ReplayBuffer
    config,source,_=a2_source
    fine=copy.deepcopy(a2_source[2])
    fine.record['replay']=ReplayBuffer(config.replay_max_bytes).snapshot(config.snapshot_max_bytes)
    with pytest.raises(ValueError,match='credit'): fine.validate(config)
    changed=destination(config,tmp_path)
    fork=TrainingState.fork_from(source,changed,experiment='ordered-a2')
    for mutation in ('missing','source_credit','admitted','active','source_checkpoint_sha256','required_tasks'):
        corrupt=copy.deepcopy(fork)
        counters=loads_transport(corrupt.record['counters'])
        if mutation=='missing': del counters['replay_bootstrap']
        else: counters['replay_bootstrap'][mutation]={'source_credit':'0','admitted':1,'active':False,
            'source_checkpoint_sha256':'0'*64,'required_tasks':['moon_inside']}[mutation]
        corrupt.record['counters']=dumps_transport(counters)
        with pytest.raises(ValueError,match='bootstrap|credit'): corrupt.validate(changed)


def test_empty_bootstrap_capture_requires_explicit_fork_provenance(a2_source,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState, UpdateBoundary
    from lunar_drl_exploration.sac import SACLearner
    config,source,_=a2_source
    changed=destination(config,tmp_path)
    fork=TrainingState.fork_from(source,changed,experiment='ordered-a2')
    learner=SACLearner(changed.model,changed.learning)
    restored=fork.restore(learner,changed)
    kwargs=dict(learner=learner,config=changed,replay=restored.replay,schedule=restored.schedule,
        boundary=UpdateBoundary(),replay_rng=restored.replay_rng,curriculum_rng=restored.curriculum_rng,
        curriculum=restored.curriculum,collector_state=restored.collector_state,counters=restored.counters)
    with pytest.raises(ValueError,match='bootstrap.*provenance'):
        TrainingState.capture(**kwargs)
    saved=TrainingState.capture(**kwargs,fork=fork.record['fork'])
    saved.validate(changed)
    assert_state_equal(fork.record['learner'],saved.record['learner'])
    assert_state_equal(fork.record['rng'],saved.record['rng'])


@pytest.mark.parametrize('a2_source',['full'],indirect=True)
def test_a2_bootstrap_stops_and_resumes_then_repays_before_new_dispatch(a2_source,tmp_path,monkeypatch):
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration.replay import loads_transport
    import json
    config,source,original=a2_source
    changed=destination(config,tmp_path)
    # Real IPC collector, reservation ledger, replay, SAC and persistence; only
    # navigation is the established controllable external environment double.
    monkeypatch.setenv('DRL_COURSE_PHASE','2')
    kwargs=dict(device='cpu',env_factory='test_ordered_curriculum:OrderedControlledEnv',
        probe_extent=40,probe_budget=5)
    first=run_training(changed,fork_from=source,fork_experiment='ordered-a2',max_transitions=31,**kwargs)
    assert (first['new_transitions'],first['updates'])==(31,1)
    state=CheckpointManager(changed.output_dir).load(changed)
    counters=loads_transport(state.record['counters'])
    assert counters['replay_bootstrap']['admitted']==31 and counters['replay_bootstrap']['active']
    assert state.record['schedule']['credit']=='19/2'
    assert_state_equal(original.record['learner'],state.record['learner'])
    with pytest.raises(ValueError,match='bootstrap.*zero|zero.*bootstrap'):
        run_training(changed,resume=True,max_transitions=0,**kwargs)
    second=run_training(changed,resume=True,max_transitions=34,**kwargs)
    assert second['new_transitions']==34 and second['updates']==19
    record=CheckpointManager(changed.output_dir).load(changed).record
    bootstrap=loads_transport(record['counters'])['replay_bootstrap']
    assert bootstrap['admitted']==64 and bootstrap['active'] is False
    assert record['schedule']['transitions']==76 and record['schedule']['credit']=='0'
    assert record['schedule']['unfinished_reservations']==0
    course=loads_transport(record['curriculum'])['course']
    assert all(course['by_task'][task]['samples']>0 for task in ('moon_inside','cave_inside','moon_outside'))
    events=[json.loads(line) for line in (tmp_path/'a2/run/metrics.jsonl').read_text().splitlines()]
    complete=[row for row in events if row['event']=='replay_bootstrap_complete']
    assert len(complete)==1
    assert complete[0]['transitions']==75 and complete[0]['updates']==1 and complete[0]['credit']=='71/4'
    repaid=[row for row in events if row['event']=='replay_bootstrap_repaid']
    assert len(repaid)==1 and repaid[0]['transitions']==75 and repaid[0]['updates']==18
    assert repaid[0]['credit']=='3/4'


def test_cli_requires_explicit_ordered_a2_source():
    from lunar_drl_exploration.cli import parser,load_config
    args=parser().parse_args(['train','--fork-from','/saved/resume.pt','--fork-experiment','ordered-a2'])
    assert args.fork_experiment=='ordered-a2'
    with pytest.raises(ValueError,match='requires --fork-from'):
        load_config(parser().parse_args(['train','--fork-experiment','ordered-a2']))
