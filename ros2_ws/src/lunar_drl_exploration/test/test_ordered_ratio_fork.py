from dataclasses import replace
import hashlib
import pytest
from test_ordered_extent_fork import ordered_checkpoint
from test_training_fork import assert_state_equal


def phase_one_source(fixture,tmp_path):
    import torch
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    from lunar_drl_exploration.replay import loads_transport,dumps_transport
    config,_,state=fixture
    curriculum=loads_transport(state.record['curriculum'])
    course=OrderedCourse(1,state=curriculum['course']).state_dict()
    course['phase_transitions']['moon_inside']=7
    course['phase_samples']['moon_inside']=64
    curriculum['course']=course
    state.record['curriculum']=dumps_transport(curriculum)
    counters=loads_transport(state.record['counters']);counters['course']=course
    state.record['counters']=dumps_transport(counters)
    path=tmp_path/'phase-one.pt';torch.save(state.record,path)
    return config,path,state


def ratio_config(config,tmp_path,**kw):
    return replace(config,output_dir=str(tmp_path/'ratio/run'),artifact_root=str(tmp_path/'ratio'),
        ordered_rehearsal_by_phase=(.2,.5,.2,.2),**kw)


def test_ratio_fork_preserves_learning_and_rebases_only_allocation(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.replay import loads_transport
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    config,source,original=phase_one_source(ordered_checkpoint,tmp_path)
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    changed=ratio_config(config,tmp_path)
    fork=TrainingState.fork_from(source,changed,experiment='ordered-ratio')
    fork.validate(changed)
    for key in ['learner','replay','schedule','rng','collector_state']:
        assert_state_equal(original.record[key],fork.record[key])
    old=loads_transport(original.record['curriculum']);new=loads_transport(fork.record['curriculum'])
    assert new['episodes_issued']==old['episodes_issued']
    c=new['course'];assert c['rehearsal']==.5
    assert c['phase_transitions']=={'moon_inside':0,'cave_inside':0}
    assert c['phase_samples']=={'moon_inside':0,'cave_inside':0}
    for key in ['by_task','coverage_stats','bootstrap_updates']:
        assert_state_equal(old['course'][key],c[key])
    course=OrderedCourse(1,.5,state=c)
    assert course.weights=={'moon_inside':.5,'cave_inside':.5}
    assert course.sampling_weights({'moon_inside':2,'cave_inside':2})==course.weights
    for phase in [0,2,3]:assert changed.ordered_rehearsal_for_phase(phase)==.2
    next_course=OrderedCourse(2,.2,state=c)
    assert next_course.weights['moon_outside']==.8
    assert fork.record['fork']['source_checkpoint_sha256']==before
    assert fork.record['fork']['allocation_before']['phase_transitions']==old['course']['phase_transitions']
    assert hashlib.sha256(source.read_bytes()).hexdigest()==before
    with pytest.raises(ValueError,match='resume'):
        original.validate(changed)
    with pytest.raises(ValueError,match='incompatible fork'):
        TrainingState.fork_from(source,changed)


@pytest.mark.parametrize('field,value',[('completion_bonus',99.),('success_coverage',.95),('ordered_rehearsal_fraction',.4),('save_interval_s',50.)])
def test_ratio_fork_rejects_unrelated_changes(ordered_checkpoint,tmp_path,field,value):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_=phase_one_source(ordered_checkpoint,tmp_path)
    with pytest.raises(ValueError,match='fork'):
        TrainingState.fork_from(source,ratio_config(config,tmp_path,**{field:value}),experiment='ordered-ratio')


def test_ratio_scope_invalid_phase_overrides_rejected(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_=phase_one_source(ordered_checkpoint,tmp_path)
    changed=replace(ratio_config(config,tmp_path),ordered_rehearsal_by_phase=(.2,.5,.5,.2))
    with pytest.raises(ValueError,match='ordered-ratio'):
        TrainingState.fork_from(source,changed,experiment='ordered-ratio')


@pytest.mark.parametrize('fractions',[.5,(.2,.5),(.2,1.,.2,.2),(.2,float('nan'),.2,.2)])
def test_phase_fraction_validation(fractions):
    from lunar_drl_exploration.config import TrainingConfig
    with pytest.raises(ValueError,match='four fractions'):
        replace(TrainingConfig(),ordered_rehearsal_by_phase=fractions)


def test_ratio_fork_rejects_network_reward_budget_and_learning_changes(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_=phase_one_source(ordered_checkpoint,tmp_path)
    base=ratio_config(config,tmp_path)
    for changed in [replace(base,model=replace(base.model,layers=2)),
                    replace(base,learning=replace(base.learning,gamma=.99)),
                    replace(base,ordered_budget_rule={'unexpected':256})]:
        with pytest.raises(ValueError,match='incompatible fork'):
            TrainingState.fork_from(source,changed,experiment='ordered-ratio')


def test_cli_ratio_requires_explicit_fork_source():
    from lunar_drl_exploration.cli import parser,load_config
    args=parser().parse_args(['train','--fork-from','/saved/resume.pt','--fork-experiment','ordered-ratio'])
    assert args.fork_experiment=='ordered-ratio'
    with pytest.raises(ValueError,match='fork-experiment'):
        load_config(parser().parse_args(['train','--fork-experiment','ordered-ratio']))


def test_ratio_fork_rejects_wrong_source_phase(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config,source,_=ordered_checkpoint
    with pytest.raises(ValueError,match='current cave_inside phase'):
        TrainingState.fork_from(source,ratio_config(config,tmp_path),experiment='ordered-ratio')


def test_ratio_resume_keeps_new_quota_progress_and_balances_admissions(ordered_checkpoint,tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.replay import loads_transport
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    config,source,_=phase_one_source(ordered_checkpoint,tmp_path)
    fork=TrainingState.fork_from(source,ratio_config(config,tmp_path),experiment='ordered-ratio')
    c=OrderedCourse(1,.5,state=loads_transport(fork.record['curriculum'])['course'])
    c.activate(0,'moon_inside');c.activate(1,'cave_inside')
    for _ in range(100):c.admitted(c.select_pending({0,1},{}))
    assert c.phase_transitions=={'moon_inside':50,'cave_inside':50}
    c.sampled(['moon_inside']*32+['cave_inside']*32)
    restored=OrderedCourse(1,.5,state=c.state_dict())
    assert restored.phase_transitions==c.phase_transitions
    assert restored.phase_samples==c.phase_samples
