"""An explicit extent-only fork preserves the complete ordered SAC state."""
from dataclasses import replace
import hashlib
from pathlib import Path

import numpy as np
import pytest

from lunar_drl_exploration.config import TrainingConfig, ModelConfig
from test_training_fork import assert_state_equal


@pytest.fixture
def ordered_checkpoint(tmp_path):
    torch = pytest.importorskip('torch')
    from lunar_drl_exploration.checkpoint import CheckpointManager, UpdateBoundary
    from lunar_drl_exploration.config import config_record, resume_semantics
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    from lunar_drl_exploration.replay import ReplayBuffer, dumps_transport
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.schedule import UpdateSchedule
    from test_replay_checkpoint import capture_state, scene, transition
    torch.set_num_threads(1)
    config = replace(TrainingConfig(), ordered_curriculum=True,
        model=ModelConfig(width=16, heads=2, layers=1), warmup=0,
        curriculum_extents_m=((50.,100.),)*3, output_dir=str(tmp_path/'source/run'),
        artifact_root=str(tmp_path/'source'))
    learner = SACLearner(config.model, config.learning)
    replay = ReplayBuffer(1000000)
    replay.add(transition(), {'s': replace(scene(), generator_descriptor={'curriculum_task':'moon_inside','start_mode':'inside'})})
    schedule = UpdateSchedule(warmup=0)
    course = OrderedCourse(0)
    course.activate(0,'moon_inside')
    for _ in range(7):
        schedule.collected()
        course.admitted(0)
    boundary = UpdateBoundary()
    with boundary.update():
        learner.update(replay.sample(64,np.random.default_rng(2)), replay.scenes)
        schedule.updated()
        course.sampled(['moon_inside']*64)
    state = capture_state(learner,config,replay,schedule,boundary)
    state.record['curriculum'] = dumps_transport(dict(episodes_issued=9,course=course.state_dict()))
    state.record['counters'] = dumps_transport(dict(transitions=7,course=course.state_dict()))
    path = CheckpointManager(config.output_dir).save(state)
    return config,path,state


def expanded(config,tmp_path,**changes):
    return replace(config,output_dir=str(tmp_path/'expanded/run'),artifact_root=str(tmp_path/'expanded'),
        curriculum_extents_m=((30.,100.),)*3,**changes)


def test_extent_fork_changes_only_identity_and_preserves_all_training_state(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import CheckpointManager, TrainingState
    from lunar_drl_exploration.sac import SACLearner
    config,source,original = ordered_checkpoint
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    destination = expanded(config,tmp_path)
    fork = TrainingState.fork_from(source,destination,experiment='ordered-extent')
    fork.validate(destination)
    for key in ('learner','replay','schedule','rng','curriculum','collector_state','counters'):
        assert_state_equal(original.record[key],fork.record[key])
    assert fork.record['config']['curriculum_extents_m'] == [[30.,100.]]*3
    assert fork.record['semantics']['curriculum_extents_m'] == [[30.,100.]]*3
    proof = fork.record['fork']
    assert proof['experiment']=='ordered-extent'
    assert proof['source_checkpoint_sha256']==digest
    assert proof['source_checkpoint']==str(source.resolve())
    assert proof['source_transitions']==7 and proof['source_updates']==1
    assert set(proof['changes'])=={'artifact_root','curriculum_extents_m'}
    assert not Path(destination.output_dir).exists()
    CheckpointManager(destination.output_dir).save(fork)
    saved = CheckpointManager(destination.output_dir).load(destination)
    learner = SACLearner(destination.model,destination.learning)
    restored = saved.restore(learner,destination)
    assert restored.schedule.transitions==7 and restored.schedule.updates==1
    assert restored.curriculum['course']['phase_transitions']=={'moon_inside':7}
    assert restored.counters['course']['by_task']['moon_inside']['samples']==64
    assert_state_equal(original.record['learner'],learner.state_dict())
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    CheckpointManager(config.output_dir).load(config)


@pytest.mark.parametrize('bounds', [((40.,100.),)*3,((50.,100.),)*3,((30.,101.),)*3,
    ((30.,100.),(50.,100.),(50.,100.)),((30.,100.),)*2])
def test_extent_fork_rejects_any_other_extent_change(ordered_checkpoint,tmp_path,bounds):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_ = ordered_checkpoint
    changed = replace(expanded(config,tmp_path),curriculum_extents_m=bounds)
    with pytest.raises(ValueError,match='ordered-extent.*extent'):
        TrainingState.fork_from(source,changed,experiment='ordered-extent')
    assert not Path(changed.output_dir).exists()


@pytest.mark.parametrize('field,value', [('completion_bonus',3.),('success_coverage',.95),
    ('warmup',8),('ordered_rehearsal_fraction',.1),('snapshot_max_bytes',1024**2),
    ('stagnation_window_steps',32),('reward_diagnostics',True),('save_interval_s',30.)])
def test_extent_fork_rejects_other_training_or_reward_changes(ordered_checkpoint,tmp_path,field,value):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_ = ordered_checkpoint
    changed = expanded(config,tmp_path,**{field:value})
    with pytest.raises(ValueError,match='fork.*'+field):
        TrainingState.fork_from(source,changed,experiment='ordered-extent')


def test_extent_fork_rejects_learning_and_budget_changes(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_ = ordered_checkpoint
    for changed in (replace(expanded(config,tmp_path),learning=replace(config.learning,gamma=.99)),
            expanded(config,tmp_path,ordered_budget_rule=dict(mode='linear_area',min_extent_m=30.,max_extent_m=100.,
                round_to=32,min_decisions=256,max_decisions=512))):
        with pytest.raises(ValueError,match='fork.*(learning|ordered_budget_rule)'):
            TrainingState.fork_from(source,changed,experiment='ordered-extent')


def test_extent_fork_requires_ordered_source_and_destination(ordered_checkpoint,tmp_path):
    import torch
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.config import config_record,resume_semantics
    config,source,original = ordered_checkpoint
    with pytest.raises(ValueError,match='ordered-extent.*ordered_curriculum'):
        TrainingState.fork_from(source,expanded(config,tmp_path,ordered_curriculum=False),experiment='ordered-extent')
    mixed = replace(config,ordered_curriculum=False)
    original.record['config']=config_record(mixed)
    original.record['semantics']=resume_semantics(mixed)
    mixed_source=tmp_path/'mixed.pt'
    torch.save(original.record,mixed_source)
    with pytest.raises(ValueError,match='ordered-extent.*ordered_curriculum'):
        TrainingState.fork_from(mixed_source,expanded(config,tmp_path),experiment='ordered-extent')


def test_ordinary_resume_and_collection_fork_do_not_silently_expand_extents(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,original=ordered_checkpoint
    changed=expanded(config,tmp_path)
    with pytest.raises(ValueError,match='resume.*curriculum'):
        original.validate(changed)
    with pytest.raises(ValueError,match='fork.*curriculum_extents_m'):
        TrainingState.fork_from(source,changed)


def test_cli_extent_experiment_requires_explicit_fork_source():
    from lunar_drl_exploration.cli import parser,load_config
    args=parser().parse_args(['train','--fork-from','/saved/resume.pt','--fork-experiment','ordered-extent'])
    assert args.fork_experiment=='ordered-extent'
    for options in ([],['--resume']):
        with pytest.raises(ValueError,match='fork-experiment.*fork-from'):
            load_config(parser().parse_args(['train','--fork-experiment','ordered-extent',*options]))
