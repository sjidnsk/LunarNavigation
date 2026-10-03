"""Task-frontier orchestration contracts, with no native/production launches."""
import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = ROOT / 'scripts/drl/task_frontier'


def module(name):
    path = SCRIPTS / (name + '.py')
    assert path.exists(), f'missing Task7 module: {name}'
    sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_budget_resumes_partial_warmup_and_drains_before_screen():
    c = module('controller')
    assert c.stage_target(0) == (1024, 0)
    assert c.stage_target(10) == (82944, 20480)
    assert c.next_training(0, {'transitions': 300, 'updates': 0, 'credit': '0'}) == 724
    assert c.next_training(1, {'transitions': 9216, 'updates': 2047, 'credit': '1'}) == 0
    assert c.next_training(1, {'transitions': 9216, 'updates': 2048, 'credit': '0'}) is None
    with pytest.raises(ValueError):
        c.next_training(1, {'transitions': 108452, 'updates': 26856, 'credit': '1'})
    with pytest.raises(ValueError):
        c.next_training(1, {'transitions': 9216, 'updates': 2047, 'credit': '0'})


def paired_fixture():
    cases = [dict(case_index=i, family=('moon', 'cave')[i % 2], seed=100+i//2,
                  extent_m=50., budget=512, terrain_id=f't{i}', reference_id=f'r{i}',
                  pairing_digest='pair', config_digest='config') for i in range(20)]
    rows = [dict(case, actor_sha='actor', semantic_id='semantic', error=None,
                 terminated=False, truncated=True, collisions=0, completed=False,
                 decisions=512, final_coverage=.5, covered_area_m2=50., coverable_area_m2=100.,
                 run_id='run', pid=123, exit_code=0, provenance_digest='proof') for case in cases]
    return cases, dict(status='finished', exit_code=0, actor_sha='actor',
                       semantic_id='semantic', cases=rows)


@pytest.mark.parametrize('bad', ['actor', 'semantic', 'scene', 'reference', 'config', 'count', 'error', 'terminal', 'collision_success', 'missing_proof'])
def test_evaluation_cache_never_skips_wrong_or_incomplete_results(bad):
    c = module('controller')
    cases, record = paired_fixture()
    c.validate_evaluation(record, actor_sha='actor', semantic_id='semantic', cases=cases)
    broken = copy.deepcopy(record)
    if bad == 'actor': broken['cases'][4]['actor_sha'] = 'other'
    elif bad == 'semantic': broken['semantic_id'] = 'other'
    elif bad == 'scene': broken['cases'][0]['terrain_id'] = 'other'
    elif bad == 'reference': broken['cases'][0]['reference_id'] = 'other'
    elif bad == 'config': broken['cases'][0]['config_digest'] = 'other'
    elif bad == 'count': broken['cases'].pop()
    elif bad == 'error': broken['cases'][0]['error'] = 'reset failed'
    elif bad == 'terminal': broken['cases'][0]['truncated'] = False
    elif bad == 'collision_success': broken['cases'][0].update(completed=True, collisions=1)
    elif bad == 'missing_proof': broken['cases'][0].pop('provenance_digest')
    with pytest.raises(ValueError):
        c.validate_evaluation(broken, actor_sha='actor', semantic_id='semantic', cases=cases)


def test_cases_freeze_original_float_values_and_order(tmp_path):
    c = module('controller')
    source = tmp_path / 'old.json'
    expected = [dict(family=family, seed=2026100100+i, extent_m=50.+i*50./9.)
                for i in range(10) for family in ('moon', 'cave')]
    source.write_text(json.dumps(dict(cases=expected)))
    frozen = c.freeze_cases(source, tmp_path/'cases.json')
    assert frozen[4]['extent_m'] == 61.111111111111114
    assert [r['family'] for r in frozen[:4]] == ['moon', 'cave', 'moon', 'cave']
    expected[4]['extent_m'] = 61.11111111111111
    source.write_text(json.dumps(dict(cases=expected)))
    with pytest.raises(ValueError): c.freeze_cases(source, tmp_path/'other.json')


def test_monitor_rejects_stale_pid_run_case_and_clips_small_windows():
    d = module('dashboard')
    manifest = dict(phase='evaluation', status='running', stage=1, run_id='r', eval_pid=42, case_index=2)
    telemetry = dict(updated_unix=100., pid=42, run_id='r', case_index=2, coverage=.75)
    assert d.telemetry_valid(manifest, telemetry, now=101.)
    for key, value in [('updated_unix', 0), ('pid', 43), ('run_id', 'old'), ('case_index', 1)]:
        assert not d.telemetry_valid(manifest, dict(telemetry, **{key:value}), now=101.)
    output = d.fit_lines(['中'*40]*20, height=5, width=20)
    assert len(output) <= 4
    assert all(sum(2 if ord(char)>127 else 1 for char in line) <= 19 for line in output)


def test_measured_probe_rebuilds_exact_graph_with_precover_and_sensor_history(tmp_path):
    import numpy as np
    from lunar_drl_exploration.contracts import SensorSpec, Pose
    from lunar_drl_exploration.decision import DecisionCore
    from test_task_analysis import snapshot, task
    p = module('fixed_probes')
    b = np.zeros((9,9), np.uint8); b[2:7,2:7] = 1
    core = DecisionCore(task(0,0,9,9), SensorSpec(range_m=3))
    s = snapshot(b,b,pose=Pose(4.5,4.5,0))
    core.record_observation(Pose(4.5,4.5,0), [[4,4]])
    core.observe(s)
    core.record_observation(Pose(4.5,4.5,.8), [[4,5]])
    before = p.capture_before_observe(core,s,(.2,.3))
    original, report = core.observe(s,(.2,.3))
    bundle = p.finish_observe(before,core,original,report)
    rebuilt = p.rebuild_fixed_observation(bundle)
    p.validate_exact_alignment(rebuilt,original)
    assert len(bundle['history']['xy']) == 2
    assert bundle['snapshot']['origin'] == s.origin
    assert np.array_equal(bundle['pre_builder']['base_points'],before['pre_builder']['base_points'])
    original_hash = p.content_hash(bundle)
    # Later live mutation cannot change the frozen measurement bundle.
    core.history._records.clear(); core.builder._base_points[:] = -100
    assert p.content_hash(bundle) == original_hash
    broken = copy.deepcopy(bundle); broken['history']['xy'] = np.empty((0,2)); broken['history']['heading'] = np.empty(0,np.int64)
    with pytest.raises(ValueError): p.validate_exact_alignment(p.rebuild_fixed_observation(broken),original)


def test_curve_buckets_keep_samples_gaps_and_write_three_png_csv_classes(tmp_path):
    c = module('curves')
    data = c.empty_history()
    c.ingest(data,[dict(learning=dict(updates=1,actor_loss=2.,alpha_position=.0001,
        alpha_heading=.00005,position_entropy=.3,position_entropy_target=.2,
        heading_conditional_entropy=.4,heading_entropy_target=.5)),
        dict(learning=dict(updates=1,actor_loss=99.)),
        dict(learning=dict(updates=201,actor_loss=4.))])
    assert data['buckets']['0']['metrics']['actor_loss']['sum'] == 2.
    assert '1' not in data['buckets']
    assert data['buckets']['0']['metrics']['position_entropy_gap']['sum'] == pytest.approx(.1)
    assert data['buckets']['0']['metrics']['position_alpha_at_cap']['sum'] == 1
    recorder = c.CurveRecorder(tmp_path)
    recorder.data = data
    recorder.persist()
    for name in ('learning','fixed','evaluation','reward'):
        assert (tmp_path/'curves'/f'{name}.csv').read_text(encoding='utf-8')
        assert (tmp_path/'curves'/f'{name}.png').read_bytes().startswith(b'\x89PNG')


def test_root_ownership_and_lock_refuse_foreign_files_and_second_controller(tmp_path):
    c=module('controller')
    foreign=tmp_path/'foreign';foreign.mkdir();(foreign/'keep.txt').write_text('owned elsewhere')
    with pytest.raises(ValueError,match='adopt'):c.prepare(foreign,ROOT/'config/drl_task_frontier.yaml')
    owned=tmp_path/'owned';record=c.prepare(owned,ROOT/'config/drl_task_frontier.yaml')
    assert c.identity(owned)==record
    with c.root_lock(owned):
        with pytest.raises(BlockingIOError):
            with c.root_lock(owned):pass
    assert not c.process_matches(dict(pid=__import__('os').getpid(),root=str(owned),run_id=record['run_id']))


def test_single_case_cache_checks_runtime_proof_and_pair_before_reuse(tmp_path):
    e=module('eval_case');c=module('controller');cases,result=paired_fixture()
    wrapper=tmp_path/'wrapper';wrapper.write_text('wrapper')
    request=dict(cases[0],actor_sha='actor',config_sha='file',wrapper=str(wrapper),source_commit='commit',run_id='run')
    current=dict(config_digest='config',semantic_id='semantic',pairing_config_digest='pair-config',runtime={'native':'hash'})
    proof=dict(current,case={k:request[k] for k in c.CASE_KEYS},actor_sha='actor',config_sha='file',
        wrapper_sha=c.sha(wrapper),source_commit='commit',run_id='run',pid=123,
        terrain_id='t0',reference_id='r0',pairing_digest='pair')
    result['cases']=result['cases'][:1];result['provenance']=proof
    result['cases'][0]['provenance_digest']=c.digest(proof)
    e.validate_cached(result,request,current)
    with pytest.raises(ValueError):e.validate_cached(result,request,dict(current,runtime={'native':'changed'}))
    with pytest.raises(ValueError):e.validate_cached(result,dict(request,paired_with={'reference_id':'other'}),current)
    result['cases'][0]['terrain_id']='fabricated'
    with pytest.raises(ValueError):e.validate_cached(result,request,current)


def test_domain_metrics_distinguish_exit_return_from_area_zero():
    e=module('eval_case');metric=e.DomainMetrics()
    metric.previous=True
    for inside in (False,False,False,True,False,True):metric.observe(inside)
    assert metric.record()==dict(outside_steps=4,max_outside_run=3,task_exits=2,task_returns=2)


def test_final_review_gate_binds_source_and_config(tmp_path,monkeypatch):
    c=module('controller');(tmp_path/'config.json').write_text('{}')
    monkeypatch.setattr(c,'source_commit',lambda:'current')
    gate=dict(source_commit='current',config_sha='wrong',native_smoke={'status':'passed','source_commit':'current'},
              independent_audit={'status':'passed','source_commit':'current'})
    c.write(tmp_path/'review-gate.json',gate)
    with pytest.raises(ValueError,match='gate'):c.verify_gate(tmp_path)


def test_stage_terminal_keeps_new_absolute_targets():
    from lunar_drl_exploration import terminal
    assert hasattr(terminal,'format_stage_status'),'new stage terminal contract missing'
    lines=terminal.format_stage_status(dict(transitions=9216,updates=2047,credit=1),stage=1)
    assert any('9,216' in line and '2,047' in line for line in lines)
    assert any('1' in line and '额度' in line for line in lines)


def test_fixed_probe_save_reload_hash_and_actor_q_scoring(tmp_path):
    import numpy as np
    import torch
    from dataclasses import replace
    from lunar_drl_exploration.contracts import SensorSpec,Pose
    from lunar_drl_exploration.decision import DecisionCore
    from lunar_drl_exploration.config import ModelConfig,TrainingConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.checkpoint import TrainingState,UpdateBoundary
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from test_task_analysis import snapshot,task
    from test_replay_checkpoint import transition,scene
    p=module('fixed_probes');c=module('curves')
    b=np.zeros((9,9),np.uint8);b[2:7,2:7]=1
    core=DecisionCore(task(0,0,9,9),SensorSpec(range_m=3))
    s=snapshot(b,b,pose=Pose(4.5,4.5,0))
    before=p.capture_before_observe(core,s,(0.,0.));obs,report=core.observe(s)
    bundle=p.finish_observe(before,core,obs,report)
    item=transition(obs,obs);scenes={'s':scene()}
    row=p.save_probe(tmp_path/'probes',bundle,obs,item.privileged,scenes,{'category':'boundary'})
    index=p.freeze_index(tmp_path/'probes')
    assert index['missing_categories']==['inside','outside']
    restored,probes=p.load_probes(tmp_path/'probes')
    assert probes[0][0]['probe_hash']==row['probe_hash']
    with pytest.raises(ValueError,match='frozen'):p.save_probe(tmp_path/'probes',bundle,obs,item.privileged,scenes,{})
    config=replace(TrainingConfig(),model=ModelConfig(width=16,heads=2,layers=1))
    learner=SACLearner(config.model,config.learning)
    record=TrainingState.capture(learner=learner,config=config,replay=ReplayBuffer(1000000),schedule=UpdateSchedule(),
        boundary=UpdateBoundary(),replay_rng=np.random.default_rng(1),curriculum_rng=np.random.default_rng(2),
        curriculum={},collector_state=dict(actor_version=0,actor_record=learner.actor_state(),policy_rng=torch.get_rng_state()),counters={}).record
    checkpoint=tmp_path/'checkpoint.pt';actor=tmp_path/'actor.pt'
    torch.save(record,checkpoint);torch.save(learner.actor_state(),actor)
    values=c.fixed_scores(checkpoint,actor,tmp_path/'probes','semantic')
    assert len(values)==len(obs.goals)>1
    assert {r['probe_set_hash'] for r in values}=={restored['probe_set_hash']}
    assert sorted(r['actor_rank'] for r in values)==list(range(1,len(obs.goals)+1))
    assert sum(r['probability'] for r in values)==pytest.approx(1.)
    # Corruption never becomes a freshly selected/reconstructed replacement input.
    path=tmp_path/'probes'/row['inputs'];path.write_bytes(path.read_bytes()+b'changed')
    with pytest.raises(ValueError,match='changed'):p.load_probes(tmp_path/'probes')


def capture_factory_child(connection,root):
    """A spawned worker substitutes only ROS transport; real capture/core execute."""
    import os
    import numpy as np
    from types import SimpleNamespace
    from lunar_drl_exploration import ros_env
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.contracts import Pose,SensorSpec
    from lunar_drl_exploration.decision import DecisionCore
    from test_task_analysis import snapshot,task
    from test_replay_checkpoint import transition,scene
    sys.path.insert(0,str(SCRIPTS))
    import train_live
    os.environ['DRL_ROOT']=str(root);os.environ['DRL_RUN_ID']='spawn-test'
    calls=[];observe=DecisionCore.observe
    def counted(core,*args,**kwargs):calls.append(1);return observe(core,*args,**kwargs)
    DecisionCore.observe=counted
    class Transport:
        def __init__(self,config,env_id,**kwargs):self.config=config
        def reset(self,*args,**kwargs):
            return self.reset_scene(None,Pose(4.5,4.5,0),task(0,0,9,9),descriptor={})
        def reset_scene(self,terrain,pose,task_spec,*,descriptor=None):
            b=np.zeros((9,9),np.uint8);b[2:7,2:7]=1
            self.core=DecisionCore(task(0,0,9,9),SensorSpec(range_m=3))
            self.core.record_observation(Pose(4.5,4.5,.8),[[4,4]])
            obs,self.report=self.core.observe(snapshot(b,b,pose=Pose(4.5,4.5,0)))
            self.static_scenes={'s':scene()};self.episode_metadata={'episode_id':'episode'}
            self.terrain=SimpleNamespace(terrain_id='terrain');self.reference=SimpleNamespace(reference_id='reference')
            self.adapter=SimpleNamespace(processed_stamp_ns=123);self._latest_sensor_ns=123
            return obs,transition(obs,obs).privileged
        def progress(self):return {'sensor_frames':2}
    ros_env.RosExplorationEnv=Transport
    try:
        env=train_live.captured_env(TrainingConfig(),0,training=True)
        env.reset(1,'moon',50.)
        connection.send(dict(calls=len(calls),torch_loaded='torch' in sys.modules,
                             probes=len(list((Path(root)/'fixed-probes').glob('probe-*.json'))),
                             initial_pose=json.loads(next((Path(root)/'fixed-probes').glob('probe-*.json')).read_text())['provenance'].get('initial_pose')))
    except Exception as exc:connection.send({'error':repr(exc)})
    finally:connection.close()


def test_spawn_worker_factory_captures_normal_observe_without_loading_torch(tmp_path):
    import multiprocessing
    ctx=multiprocessing.get_context('spawn');parent,child=ctx.Pipe()
    process=ctx.Process(target=capture_factory_child,args=(child,str(tmp_path)))
    process.start();child.close()
    try:
        assert parent.poll(30),'spawned capture timed out'
        result=parent.recv()
        assert result==dict(calls=1,torch_loaded=False,probes=1,initial_pose=[4.5,4.5,0])
    finally:
        process.join(10)
        if process.is_alive():process.terminate();process.join(10)
        parent.close()
    assert process.exitcode==0


def test_controller_progress_subprocess_and_owned_stop_keep_persistent_logs(tmp_path):
    import os
    import signal
    import threading
    c=module('controller');root=tmp_path/'run';c.prepare(root,ROOT/'config/drl_task_frontier.yaml')
    control=c.Controller(root);control.record(phase='training')
    script="import json; print('TASK_FRONTIER_PROGRESS '+json.dumps(dict(transitions=300,updates=0,credit=0)),flush=True)"
    assert control.spawn([sys.executable,'-c',script],'fixture')
    assert c.read(root/'manifest.json')['actuals']==dict(transitions=300,updates=0)
    assert (root/'fixture.log').read_bytes().startswith(b'TASK_FRONTIER_PROGRESS')
    stop_script="import signal,time; signal.signal(signal.SIGINT,lambda *args:exit(0)); print('ready',flush=True); time.sleep(60)"
    def stop_when_ready():
        import time
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            if (root/'stop.log').exists() and b'ready' in (root/'stop.log').read_bytes():
                control.interrupt();return
            time.sleep(.01)
    thread=threading.Thread(target=stop_when_ready);thread.start()
    try:assert control.spawn([sys.executable,'-c',stop_script],'stop') is False
    finally:thread.join(12)
    assert control.child is None and control.stop


def test_controller_screen_orders_twenty_exact_cases_and_rejects_scene_mismatch(tmp_path):
    c=module('controller');root=tmp_path/'owned';c.prepare(root,ROOT/'config/drl_task_frontier.yaml')
    control=c.Controller(root);control.record(stage=1)
    actor=root/'stage1.pt';actor.write_bytes(b'controlled frozen actor')
    c.write(actor.with_suffix('.json'),dict(checkpoint_sha='checkpoint',actor_sha=c.sha(actor)))
    _,baseline=paired_fixture();observed=[]
    def external_child(command,label,**environment):
        request=c.read(root/'evaluation-request.json');observed.append(request)
        index=request['case_index'];row=dict(baseline['cases'][index],**{key:request[key] for key in c.CASE_KEYS})
        row.update(actor_sha=c.sha(actor))
        c.write(Path(command[-1]),dict(status='finished',exit_code=0,actor_sha=c.sha(actor),semantic_id='semantic',cases=[row]))
        return True
    control.spawn=external_child
    result=control.screen('stage1',actor,baseline)
    assert len(result['cases'])==20
    assert [row['family'] for row in observed[:4]]==['moon','cave','moon','cave']
    assert observed[4]['extent_m']==61.111111111111114
    assert c.read(root/'manifest.json')['eval_groups']==10
    baseline['cases'][0]['terrain_id']='different native terrain'
    # External fixture keeps its old scene, while the required pair identity changes.
    saved_spawn=control.spawn
    def wrong_scene(command,label,**environment):
        value=saved_spawn(command,label,**environment)
        payload=c.read(Path(command[-1]));payload['cases'][0]['terrain_id']='t0';c.write(Path(command[-1]),payload)
        return value
    control.spawn=wrong_scene
    with pytest.raises(ValueError,match='terrain_id'):control.screen('stage1',actor,baseline)


def test_controller_does_not_publish_partial_evaluation_as_finished(tmp_path):
    c=module('controller');root=tmp_path/'owned';c.prepare(root,ROOT/'config/drl_task_frontier.yaml')
    control=c.Controller(root);actor=root/'stage1.pt';actor.write_bytes(b'actor')
    c.write(actor.with_suffix('.json'),dict(checkpoint_sha='checkpoint',actor_sha=c.sha(actor)))
    control.spawn=lambda *args,**kwargs:False
    assert control.screen('stage1',actor) is None
    assert not (root/'stage1-summary.json').exists()


def test_owned_run_rejects_effective_config_drift(tmp_path):
    c=module('controller');root=tmp_path/'owned';c.prepare(root,ROOT/'config/drl_task_frontier.yaml')
    config=c.read(root/'config.json');config['warmup']=4;c.write(root/'config.json',config)
    with pytest.raises(ValueError,match='config'):c.identity(root)


@pytest.fixture
def startup_fixture(tmp_path,monkeypatch):
    """Real child entry/locks/pipe; substitute only gate and long-running job body."""
    import os
    import select
    import subprocess
    import threading
    c=module('controller');root=tmp_path/'owned';c.prepare(root,ROOT/'config/drl_task_frontier.yaml')
    monkeypatch.setattr(c,'verify_gate',lambda root:None)
    children=[];entered=threading.Event();release=threading.Event()
    options={'mode':'ready','hold_parent':False};real_popen=subprocess.Popen
    code='''import os,signal,sys
sys.path.insert(0,sys.argv[1])
import controller as c
marker=int(sys.argv[2]);mode=sys.argv[3]
c.verify_gate=lambda root:None
if mode=='exit':
    os.write(marker,b'E');os.close(marker);sys.exit(7)
def finite_job(self,*,ready_fd=None,startup_token=None,lock_fd=None):
    signal.signal(signal.SIGINT,lambda *args:sys.exit(0))
    signal.signal(signal.SIGTERM,lambda *args:sys.exit(0))
    os.write(marker,b'L');os.close(marker)
    if mode!='timeout':
        self.announce_ready(ready_fd,'wrong-token' if mode=='identity' else startup_token,lock_fd)
    while True:signal.pause()
c.Controller.run=finite_job
c.main(sys.argv[5:])
'''
    def controlled_popen(command,**kwargs):
        reader,writer=os.pipe()
        inherited=tuple(kwargs.pop('pass_fds',()))
        process=real_popen([*command[:2],'python','-c',code,str(SCRIPTS),str(writer),options['mode'],
            str(SCRIPTS/'controller.py'),*command[2:]],pass_fds=(*inherited,writer),**kwargs)
        children.append(process);os.close(writer)
        try:
            assert select.select([reader],[],[],10)[0],'child never reached lock or exit'
            event=os.read(reader,1)
            # This wait happens INSIDE the parent's execution-lock context.
            # L proves the inherited child acquired/adopted it before parent release.
            options['child_event']=event
            entered.set()
            if options['hold_parent']:assert release.wait(10),'test did not release launcher'
        finally:os.close(reader)
        return process
    monkeypatch.setattr(c.subprocess,'Popen',controlled_popen)
    try:yield c,root,children,options,entered,release
    finally:
        release.set()
        for process in children:
            if process.poll() is None:process.terminate()
            process.wait(timeout=10)


def test_background_entry_hands_owned_lock_to_child_before_parent_release(startup_fixture,capsys):
    import threading
    c,root,children,options,entered,release=startup_fixture
    options['hold_parent']=True;errors=[]
    def start():
        try:c.main(['start','--root',str(root)])
        except Exception as error:errors.append(error)
    launcher=threading.Thread(target=start);launcher.start()
    try:
        assert entered.wait(12)
        # Another concurrent startup cannot enter while either launcher or child owns it.
        with pytest.raises(BlockingIOError):c.main(['resume','--root',str(root)])
    finally:release.set();launcher.join(12)
    assert not launcher.is_alive() and not errors
    assert options['child_event']==b'L','child must hold execution lock before launcher releases it'
    ready=json.loads(capsys.readouterr().out)
    assert ready['ready'] is True and ready['pid']==children[0].pid
    assert ready['run_id']==c.identity(root)['run_id'] and ready['root']==str(root)
    assert c.process_matches(ready)
    owned=c.identity(root)
    for key,value in (('pid',ready['pid']+1),('process_start','wrong'),('run_id','wrong'),
                      ('root',str(root/'other')),('lock_inode',ready['lock_inode']+1)):
        assert not c.startup_identity_matches(dict(ready,**{key:value}),children[0],root,owned,ready['startup_token'])
    assert c.read(root/'manifest.json')['startup_token']==ready['startup_token']
    with pytest.raises(BlockingIOError):
        with c.root_lock(root):pass


@pytest.mark.parametrize('mode,pattern',[('exit','7'),('identity','identity'),('timeout','timed out')])
def test_background_entry_rejects_early_exit_bad_identity_and_timeout(startup_fixture,capsys,mode,pattern,monkeypatch):
    c,root,children,options,entered,release=startup_fixture
    options['mode']=mode
    monkeypatch.setattr(c,'STARTUP_TIMEOUT',.5,raising=False)
    with pytest.raises(RuntimeError,match=pattern):c.main(['resume','--root',str(root)])
    assert not capsys.readouterr().out.strip(),'failed startup must not print successful PID result'
    assert children[0].poll() is not None
    # Failed startup leaves no lock-owning orphan and permits a later retry.
    with c.root_lock(root):pass
