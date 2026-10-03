"""Regressions for world-metre diagnostics and resumable best-new selection."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
import torch
from test_task_frontier_controller import module, paired_fixture


def graph_observation(polygon, point):
    from lunar_drl_exploration.contracts import Pose, SensorSpec, TaskSpec
    from lunar_drl_exploration.decision import DecisionCore
    from test_task_analysis import snapshot
    task = TaskSpec('task', 'map', polygon)
    core = DecisionCore(task, SensorSpec(range_m=10))
    grid = np.ones((12, 12), np.uint8)
    obs, _ = core.observe(snapshot(grid, pose=Pose(*point, 0), origin=(point[0]-6, point[1]-6, 0)))
    return obs, task


@pytest.mark.parametrize('offset', [0, 100])
@pytest.mark.parametrize('point,want', [((50,50),'inside'), ((5,50),'boundary'), ((0,50),'boundary'), ((-5,50),'outside')])
def test_categories_use_world_graph_positions(offset, point, want):
    p = module('fixed_probes')
    polygon = np.array([[0,0],[100,0],[100,100],[0,100]]) + offset
    obs, task = graph_observation(polygon, np.array(point)+offset)
    assert p.state_category(obs, 10, task.polygon) == want


def test_category_far_boundary_is_measured_in_metres():
    obs, task = graph_observation([[-100,-100],[100,-100],[100,100],[-100,100]], (0,0))
    assert module('fixed_probes').state_category(obs, 10, task.polygon) == 'inside'


def test_observed_environment_hooks_count_real_graph_exit_return():
    e = module('eval_case')
    polygon = [[100,100],[200,100],[200,200],[100,200]]
    observations = [graph_observation(polygon, point) for point in ((150,150),(105,150),(95,150),(150,150))]
    class Env:
        def reset(self, *args, **kwargs):
            self.task = observations[0][1]
            self.terrain = SimpleNamespace(terrain_id='terrain')
            self.reference = SimpleNamespace(reference_id='reference', coverage_ratio=lambda _: .5)
            from lunar_drl_exploration.config import TrainingConfig
            self.config = TrainingConfig(); self.episode_metadata = {}; self.probe_initial_pose = [150,150,0]
            self.index = 0
            return observations[0][0], SimpleNamespace(observed=[])
        def step(self, action):
            self.index += 1
            return SimpleNamespace(next_observation=observations[self.index][0],
                next_privileged=SimpleNamespace(observed=[]), parts=SimpleNamespace(new_area_m2=0))
    telemetry = SimpleNamespace(state={'step':0,'zero':0})
    telemetry.update = lambda **kw: telemetry.state.update(kw)
    metric = e.DomainMetrics()
    observed = e.observed_env_class(Env, telemetry, metric, {}, {'pairing_config_digest':'pair'}, None)()
    observed.reset(1,'moon',100)
    for _ in range(3): observed.step(0)
    assert metric.record() == dict(outside_steps=1,max_outside_run=1,task_exits=1,task_returns=1)


@pytest.fixture
def selection_fixture(tmp_path):
    from lunar_drl_exploration.config import TrainingConfig, ModelConfig, config_record
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.checkpoint import CheckpointManager, UpdateBoundary
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.evaluation import export_actor
    from test_replay_checkpoint import transition, scene, capture_state
    c = module('controller'); s = module('selection')
    config = replace(TrainingConfig(), model=ModelConfig(width=16,heads=2,layers=1),
                     output_dir=str(tmp_path/'run'), artifact_root=str(tmp_path))
    from lunar_drl_exploration.artifacts import initialize_artifact_root
    initialize_artifact_root(tmp_path, config.total_output_max_bytes)
    c.write(tmp_path/'config.json',config_record(config))
    cases, baseline = paired_fixture()
    c.write(tmp_path/'cases.json', {'cases':cases})
    baseline.update(label='baseline',stage=0)
    for row in baseline['cases']:
        row.update(terminated=True,truncated=False,completed=True,final_coverage=1.,covered_area_m2=100.,
                   max_zero_gain_run=0,zero_gain_two_point_loop_steps=0)
    c.write(tmp_path/'baseline-summary.json',baseline)
    learner = SACLearner(config.model,config.learning)
    replay = ReplayBuffer(config.replay_max_bytes); replay.add(transition(),{'s':scene()})
    learner.update([transition()]*64,{'s':scene()})  # Populate real Adam states and dual temperatures.
    manager = CheckpointManager(tmp_path/'run',artifact_root=str(tmp_path))
    def stage(number, coverage=.5):
        learner.updates = number*2048
        schedule = UpdateSchedule(); schedule.transitions=1024+8192*number; schedule.updates=number*2048
        state = capture_state(learner,config,replay,schedule,UpdateBoundary())
        manager.save(state)
        actor = tmp_path/f'stage{number}.pt'; export_actor(tmp_path/'run/resume.pt',actor)
        c.write(actor.with_suffix('.json'),dict(actor_sha=c.sha(actor),checkpoint_sha=c.sha(tmp_path/'run/resume.pt'),updates=learner.updates))
        _, result = paired_fixture()
        from eval_case import effective_identity
        ids=effective_identity(replace(config,stagnation_window_steps=0))
        result.update(label=f'stage{number}',stage=number,actor_sha=c.sha(actor),semantic_id=ids['semantic_id'])
        for row in result['cases']:
            row.update(actor_sha=result['actor_sha'],semantic_id=result['semantic_id'],config_digest=ids['config_digest'],
                       final_coverage=coverage,covered_area_m2=coverage*100,max_zero_gain_run=0,zero_gain_two_point_loop_steps=0)
        c.write(tmp_path/f'stage{number}-summary.json', result)
        return result, actor
    return c,s,tmp_path,baseline,stage,config


def test_best_new_survives_later_resume_overwrite_baseline_win_and_tie(selection_fixture):
    c,s,root,baseline,stage,config=selection_fixture
    selection=s.StageSelection(root); selection.start(baseline)
    first, actor=stage(1,.8); selection.complete(first,actor)
    selected_sha=c.sha(root/'best-new.pt')
    second, actor=stage(2,.4); selection.complete(second,actor)
    third, actor=stage(3,.8); selection.complete(third,actor)
    saved=c.read(root/'selection.json')
    assert saved['best_label']=='baseline' and saved['best_new']['label']=='stage1'
    assert saved['status']=='unfinished' and len(saved['completed'])==3
    assert c.sha(root/'best-new.pt')==selected_sha != c.sha(root/'run/resume.pt')
    assert c.read(root/'comparison.json')['best_new_label']=='stage1'
    resumed=s.StageSelection(root); resumed.verify()
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.sac import SACLearner
    learner=SACLearner(config.model,config.learning)
    restored=TrainingState(torch.load(root/'best-new.pt',weights_only=False)).restore(learner,config)
    assert learner.updates==2048 and restored.schedule.transitions==9216 and len(restored.replay)==1
    assert learner.actor_optimizer.state_dict()['state']
    exported=torch.load(root/'stage1.pt',weights_only=True)
    for key,value in learner.actor.state_dict().items(): assert torch.equal(value,exported['state_dict'][key])
    assert torch.load(root/'run/resume.pt',weights_only=False)['schedule']['updates']==6144


@pytest.mark.parametrize('bad', ['partial','error','semantic','checkpoint','actor'])
def test_selection_rejects_incomplete_or_unbound_stage(selection_fixture,bad):
    c,s,root,baseline,stage,_=selection_fixture
    selection=s.StageSelection(root); selection.start(baseline)
    result,actor=stage(1)
    if bad=='partial':result['cases'].pop()
    if bad=='error':result['cases'][0]['error']='failed'
    if bad=='semantic':result['semantic_id']='wrong'
    if bad=='checkpoint':(root/'run/resume.pt').write_bytes(b'invalid')
    if bad=='actor':actor.write_bytes(b'wrong')
    c.write(root/'stage1-summary.json',result)
    with pytest.raises((ValueError,RuntimeError)):selection.complete(result,actor)
    assert c.read(root/'selection.json')['best_new'] is None
    assert c.read(root/'selection.json')['status']=='unfinished'
    assert not (root/'best-new.pt').exists()


@pytest.mark.parametrize('failure', ['copy','selection','comparison','clear'])
def test_promotion_fault_is_recoverable_before_any_later_training(selection_fixture,monkeypatch,failure):
    c,s,root,baseline,stage,_=selection_fixture
    selection=s.StageSelection(root); selection.start(baseline)
    result,actor=stage(1)
    original=s.write
    if failure=='copy':
        def fail(*args,**kwargs):raise OSError('injected copy failure')
        monkeypatch.setattr(selection,'_copy_checkpoint',fail)
    else:
        def fail(path,value):
            if (path.name==failure+'.json' and (failure!='comparison' or value['best_new_label'])
                    or failure=='clear' and path.name=='selection-pending.json' and value is None):
                raise OSError('injected record failure')
            return original(path,value)
        monkeypatch.setattr(s,'write',fail)
    with pytest.raises(OSError):selection.complete(result,actor)
    monkeypatch.setattr(s,'write',original)
    recovered=s.StageSelection(root); recovered.verify()
    assert c.read(root/'selection.json')['best_new']['label']=='stage1'
    assert c.read(root/'comparison.json')['status']=='unfinished'
    assert c.read(root/'selection-pending.json') is None
    assert len(list(root.glob('best*.pt')))==1


@pytest.mark.parametrize('bad', ['checkpoint','actor','score','summary'])
def test_resume_refuses_selection_checkpoint_or_summary_inconsistency(selection_fixture,bad):
    c,s,root,baseline,stage,_=selection_fixture
    selection=s.StageSelection(root);selection.start(baseline)
    result,actor=stage(1);selection.complete(result,actor)
    if bad=='checkpoint':(root/'best-new.pt').write_bytes(b'wrong')
    if bad=='actor':actor.write_bytes(b'wrong')
    if bad=='score':
        saved=c.read(root/'selection.json');saved['best_new']['score']=[20,0,20];c.write(root/'selection.json',saved)
    if bad=='summary':
        result['cases'][0]['final_coverage']=.1;c.write(root/'stage1-summary.json',result)
    with pytest.raises(ValueError):s.StageSelection(root).verify()


def test_selection_promotes_using_completion_then_loop_failures_then_coverage(selection_fixture):
    c,s,root,baseline,stage,_=selection_fixture
    selected=s.StageSelection(root);selected.start(baseline)
    result,actor=stage(1,.9)
    result['cases'][0].update(max_zero_gain_run=128,zero_gain_two_point_loop_steps=1)
    c.write(root/'stage1-summary.json',result);selected.complete(result,actor)
    result,actor=stage(2,.4);selected.complete(result,actor)
    assert c.read(root/'selection.json')['best_new']['label']=='stage2'
    result,actor=stage(3,.2)
    result['cases'][0].update(completed=True,terminated=True,truncated=False,final_coverage=1.,covered_area_m2=100.)
    c.write(root/'stage3-summary.json',result);selected.complete(result,actor)
    assert c.read(root/'selection.json')['best_new']['label']=='stage3'


@pytest.mark.parametrize('when',['before_replace','after_replace'])
def test_second_promotion_fault_keeps_old_or_recovers_new(selection_fixture,monkeypatch,when):
    c,s,root,baseline,stage,_=selection_fixture
    selected=s.StageSelection(root);selected.start(baseline)
    result,actor=stage(1,.4);selected.complete(result,actor)
    old=c.sha(root/'best-new.pt')
    result,actor=stage(2,.8)
    from lunar_drl_exploration import artifacts
    original=artifacts.atomic_write
    def fail(path,writer,**kwargs):
        if path==root/'best-new.pt':
            if when=='after_replace':original(path,writer,**kwargs)
            else:
                def interrupted(stream):stream.write(b'partial');raise OSError('injected partial copy')
                return original(path,interrupted,**kwargs)
            raise OSError('injected after replacement')
        return original(path,writer,**kwargs)
    monkeypatch.setattr(artifacts,'atomic_write',fail)
    with pytest.raises(OSError):selected.complete(result,actor)
    assert c.read(root/'selection.json')['best_new']['label']=='stage1'
    if when=='before_replace':assert c.sha(root/'best-new.pt')==old
    monkeypatch.setattr(artifacts,'atomic_write',original)
    s.StageSelection(root).verify()
    assert c.read(root/'selection.json')['best_new']['label']=='stage2'
    assert c.sha(root/'best-new.pt')==c.sha(root/'run/resume.pt')
    assert not list(root.glob('.*.tmp.pt'))


def test_controller_selects_before_next_updates_and_resume_never_rewinds(selection_fixture,monkeypatch):
    c,s,root,baseline,stage,_=selection_fixture
    import curves, fixed_probes
    monkeypatch.setattr(curves,'CurveRecorder',lambda root:SimpleNamespace(start=lambda:None,close=lambda:None,fixed_checkpoint=lambda *args:None))
    monkeypatch.setattr(c,'verify_gate',lambda root:None)
    monkeypatch.setattr(c,'freeze_cases',lambda *args:None)
    monkeypatch.setattr(fixed_probes,'freeze_index',lambda *args:None)
    monkeypatch.setattr(c.signal,'signal',lambda *args:None)
    control=c.Controller.__new__(c.Controller)
    control.root=root;control.identity={'run_id':'controlled'};control.child=None;control.stop=False
    control.manifest={'status':'running','actuals':{},'exit_code':None}
    control.announce_ready=lambda *args:None
    def screen(label,actor,base=None):return baseline if label=='baseline' else c.read(root/f'{label}-summary.json')
    control.screen=screen
    admissions=[]
    def train(command,label,**environment):
        number=int(environment['DRL_STAGE']);admissions.append(number)
        before=c.read(root/'selection.json')
        assert len(before['completed'])==max(0,number-1)
        if number>1:
            assert before['best_new']['label']=='stage1'
            assert torch.load(root/'run/resume.pt',weights_only=False)['schedule']['updates']==2048*(number-1)
            assert '--resume' in command
        stage(number,.8 if number==1 else .4)
        return True
    control.spawn=train
    control.run()
    assert admissions==list(range(11))
    assert c.read(root/'selection.json')['status']=='finished'
    assert c.read(root/'comparison.json')['status']=='finished'
    assert c.read(root/'manifest.json')['actuals']=={'transitions':82944,'updates':20480}
    admissions.clear()
    control.run()
    assert admissions==[]
    assert torch.load(root/'run/resume.pt',weights_only=False)['schedule']['updates']==20480


def test_consistent_actor_hash_cannot_substitute_different_weights(selection_fixture):
    c,s,root,baseline,stage,_=selection_fixture
    selected=s.StageSelection(root);selected.start(baseline)
    result,actor=stage(1)
    exported=torch.load(actor,weights_only=True)
    key=next(iter(exported['state_dict']));exported['state_dict'][key]=exported['state_dict'][key]+1
    torch.save(exported,actor)
    sidecar=c.read(actor.with_suffix('.json'));sidecar['actor_sha']=c.sha(actor);c.write(actor.with_suffix('.json'),sidecar)
    result['actor_sha']=c.sha(actor)
    for row in result['cases']:row['actor_sha']=c.sha(actor)
    c.write(root/'stage1-summary.json',result)
    with pytest.raises(ValueError,match='Actor binding'):selected.complete(result,actor)
    assert c.read(root/'selection.json')['best_new'] is None


def test_interrupted_promotion_refuses_overwritten_unpreserved_source(selection_fixture,monkeypatch):
    c,s,root,baseline,stage,_=selection_fixture
    selected=s.StageSelection(root);selected.start(baseline)
    result,actor=stage(1)
    def fail(*args):raise OSError('before copy')
    monkeypatch.setattr(selected,'_copy_checkpoint',fail)
    with pytest.raises(OSError):selected.complete(result,actor)
    stage(2)
    with pytest.raises(ValueError,match='checkpoint hash'):s.StageSelection(root).verify()
    assert c.read(root/'selection.json')['best_new'] is None
    assert c.read(root/'selection.json')['status']=='unfinished'
