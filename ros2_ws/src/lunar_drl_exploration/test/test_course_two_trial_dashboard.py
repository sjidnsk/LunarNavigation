"""Presentation must not fabricate liveness, denominators, or training progress."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import pytest


@pytest.fixture
def ui(monkeypatch):
    path=Path(__file__).resolve().parents[4]/'scripts/drl/course_two_trial_dashboard.py'
    monkeypatch.syspath_prepend(str(path.parent))
    spec=importlib.util.spec_from_file_location('trial_dashboard_test',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value),encoding='utf-8')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def root(tmp_path):
    write(tmp_path/'state.json',dict(status='running',phase='qualification',stage='A1'))
    write(tmp_path/'identity.json',dict(runtime='runtime-one',baseline_transitions=254976,baseline_updates=63488))
    write(tmp_path/'config.json',dict(environments=16))
    for kind in ('qualification','evaluation','training'):
        write(tmp_path/(kind+'-settings.json'),dict(workers=12))
    write(tmp_path/'frozen/training-requests.json',[])
    write(tmp_path/'frozen/development-requests.json',[])
    (tmp_path/'frozen/block31.pt').write_bytes(b'actor baseline')
    return tmp_path


def test_dead_owner_is_never_running_and_no_file_is_changed(ui,root):
    before={p:p.read_bytes() for p in root.rglob('*') if p.is_file()}
    s=ui.Reader(root,lambda _:None).snapshot(now=1000)
    assert s['status']=='interrupted'
    assert not s['active_training'] and s['eta'] is None
    assert '控制器已退出' in '\n'.join(t for t,_ in ui.frame(s)[0])
    assert before=={p:p.read_bytes() for p in root.rglob('*') if p.is_file()}
    assert s['limits']==dict(qualification=12,evaluation=12,training=12)


def test_corrupt_owner_is_reported_without_crashing(ui,root):
    def broken(_):
        raise ValueError('identity mismatch')
    s=ui.Reader(root,broken).snapshot()
    assert s['status']=='interrupted'
    assert 'identity mismatch' in s['warnings'][0]


def test_qualification_reuses_only_explicitly_adopted_results(ui,root):
    request=dict(request_id='q1',role='cover',split='train')
    write(root/'frozen/training-requests.json',[request])
    out=root/'qualification/rebuilt-q1.json'
    proof=dict(runtime='old-runtime',config_sha=sha(root/'config.json'),request=request,original=False)
    result=dict(status='passed',proof=proof,rejections=[],wall_s=3)
    write(out,result)
    reader=ui.Reader(root,lambda _:dict(pid=123))
    assert reader.snapshot()['qualification']['done']==0
    identity=json.loads((root/'identity.json').read_text())
    identity['qualification_cache_adoptions']={out.name:dict(sha=sha(out),from_runtime='old-runtime',to_runtime='runtime-one')}
    write(root/'identity.json',identity)
    assert reader.snapshot()['qualification']['done']==1
    result['wall_s']=4
    write(out,result)
    assert reader.snapshot()['qualification']['done']==0


@pytest.fixture
def evaluated_root(root):
    cases=[dict(case_id=f'n{i}',kind='new') for i in range(4)]
    cases += [dict(case_id='dev1',kind='capability',capability='cover')]
    write(root/'cases.json',cases)
    for i,case in enumerate(cases):
        row=dict(case_id=case['case_id'],kind=case['kind'],completed=i in (0,1),
            initial_roi_visible=i==1,first_roi={'decisions':2} if i in (0,2,3) else None,
            error='reset failed' if i in (3,4) else None,collisions=0,decisions=10)
        proof=dict(runtime='runtime-one',actor_sha=sha(root/'frozen/block31.pt'),
            config_sha=sha(root/'config.json'),case=case)
        write(root/'evaluation/baseline'/(case['case_id']+'.json'),
            dict(status='failed' if row['error'] else 'finished',proof=proof,row=row,wall_s=3))
    return root


def test_first_sight_has_its_own_denominator_and_errors_are_separate(ui,evaluated_root):
    s=ui.Reader(evaluated_root,lambda _:None).snapshot()
    e=s['baseline'];g=e['groups']['new']
    assert (g['done'],g['success'],g['failed'],g['errors'])==(4,2,1,1)
    assert (g['blind'],g['approached'],g['initial_visible'])==(2,2,1)
    assert e['groups']['cover']['errors']==1  # Failed reset lacks a row.capability.
    assert not e['complete']
    text='\n'.join(t for t,_ in ui.evaluation_table(s,140,'baseline'))
    assert '2/2 (100.0%)' in text and '66.7%' in text
    assert '剔除初始可见 1 / 错误 1' in text


def test_three_scene_panel_shows_dynamic_slots_and_only_current_groups(ui,root):
    from lunar_drl_exploration.course_two_simple import development_cases,SimpleCourse
    from lunar_drl_exploration.config import TrainingConfig
    write(root/'identity.json',dict(runtime='runtime-one',sampling='scene_episode_v1',task_scope='whole_cave_v1'))
    write(root/'cases.json',development_cases(TrainingConfig()))
    course=SimpleCourse();course.active={0:'moon_inside',1:'moon_outside',2:'cave_inside'}
    write(root/'arms/C3/training/run/progress.json',dict(course=course.summary()))
    s=ui.Reader(root,lambda _:None).snapshot()
    assert all(s['baseline']['groups'][t]['total']==20 for t in course.weights)
    text='\n'.join(t for t,_ in ui.protocol(s,140)+ui.task_table(s,140)+ui.replay_table(s,140)+ui.gate_rows(s,140))
    assert '动态' in text and '21/21/22' in text and '当前进程' in text
    assert '收尾' not in text and '各 50%' not in text and '72/80' not in text


def test_changed_actor_or_result_is_not_counted_from_cache(ui,evaluated_root):
    reader=ui.Reader(evaluated_root,lambda _:None)
    assert reader.snapshot()['baseline']['done']==5
    path=evaluated_root/'evaluation/baseline/n0.json'
    path.write_text('{broken',encoding='utf-8')
    assert reader.snapshot()['baseline']['done']==4
    (evaluated_root/'frozen/block31.pt').write_bytes(b'different actor')
    s=reader.snapshot()
    assert s['baseline']['done']==0 and s['warnings']


def test_old_job_progress_is_not_a_current_worker(ui,evaluated_root,monkeypatch):
    root=evaluated_root
    case=json.loads((root/'cases.json').read_text())[0]
    worker=dict(slot=0,pid=123,case_id='n0',job_id='current',request='live/request.json',
        progress='live/progress.json',started_unix=950)
    write(root/'live/request.json',dict(actor=str(root/'frozen/block31.pt'),case=case))
    write(root/'live/progress.json',dict(actor=str(root/'frozen/block31.pt'),case_id='n0',
        job_id='old',slot=0,updated_unix=999,decisions=511,coverage=.99))
    write(root/'state.json',dict(status='running',phase='evaluation',evaluation_label='baseline',evaluation_workers=[worker]))
    s=ui.Reader(root,lambda _:dict(pid=123)).snapshot(now=1000)
    assert s['evaluation']['workers'][0]['progress']=={}
    assert '511' not in '\n'.join(t for t,_ in ui.worker_rows(s,144))


def test_prefill_does_not_display_inherited_updates_as_new_learning(ui,root):
    write(root/'state.json',dict(status='paused',phase='prefill',arm='C3',target_transitions=259072))
    write(root/'arms/C3/training/run/progress.json',dict(collect_only=True,transitions=255076,
        updates=63488,new_transitions=100,wall_s=100,transitions_per_s=1,updates_per_s=0))
    s=ui.Reader(root,lambda _:None).snapshot()
    text='\n'.join(t for t,_ in ui.training_stats(s,144))
    assert '100/4,096' in text and '本阶段零优化器更新' in text
    assert '累计更新 63,488' in text and '本段更新' not in text
    assert '最近训练快照' in text


def test_training_success_excludes_zero_step_episodes(ui,root):
    s=ui.Reader(root,lambda _:None).snapshot()
    s['progress']=dict(course=dict(by_task=dict(c2_A1_cover=dict(
        ended=12,zero_decisions=2,successes=8,initial_completed=2,transitions=50,samples=200))))
    text='\n'.join(t for t,_ in ui.task_table(s,144))
    assert '8/10' in text and '80.0%' in text


def test_actual_finish_phase_includes_success_bucket_without_double_counting(ui,root):
    s=ui.Reader(root,lambda _:None).snapshot()
    s['progress']=dict(balanced_replay=dict(ready=True,phase_quotas={'new_finish':8},
        pool_counts={'new_finish':40},episode_counts={'new_finish':8},
        last_batch=dict(roles={'new_finish':4,'success_finish':4},phases={'new_finish':8},
            unique_transitions=64,episodes=20,max_per_episode=4)))
    text='\n'.join(t for t,_ in ui.replay_table(s,140))
    row=next(line for line in text.splitlines() if line.startswith('覆盖收尾'))
    assert row.split()[-2:]==['8','8']
    assert '成功末段专用抽样 4/4' in text


def test_gate_needs_complete_matching_evaluation(ui,root):
    write(root/'arms/C3/gate.json',dict(passed=True,stop=False,actor_sha='old-actor'))
    s=ui.Reader(root,lambda _:None).snapshot()
    assert not s['gate'] and any('门槛记录' in w for w in s['warnings'])


def test_stop_reason_comes_from_the_training_run_manifest(ui,root):
    write(root/'arms/C3/training/run/run.json',dict(observations=dict(
        final=dict(stop_reason='qualified replay prefill ready; zero optimizer updates',owned_children_closed=True))))
    s=ui.Reader(root,lambda _:None).snapshot()
    assert s['exit']['stop_reason'].startswith('qualified replay')
    assert s['exit']['owned_children_closed'] is True


@pytest.mark.parametrize('width,height',[(80,24),(126,34),(144,42),(160,44)])
def test_paging_keeps_controls_visible_and_every_line_reachable(ui,root,width,height):
    s=ui.Reader(root,lambda _:None).snapshot()
    s['state'].update(error='long-error-'*40)
    for view in ui.VIEWS:
        first,offset,total=ui.frame(s,view,width,height,0,log=['记录 '*100]*15)
        last,last_offset,_=ui.frame(s,view,width,height,100000,log=['记录 '*100]*15)
        assert len(first)==len(last)==height
        assert all(ui.cells(t)<width for t,_ in first+last)
        assert 'P 保存停止' in first[-1][0] and 'R 恢复' in first[-1][0]
        assert offset==0 and last_offset==max(0,total-(height-6))


def test_small_overview_prioritizes_live_workers(ui,root):
    s=ui.Reader(root,lambda _:dict(pid=1)).snapshot()
    s['qualification'].update(done=140,total=192,workers=[dict(slot=i,pid=i+1,split='training',
        alive=True,elapsed=80,age=1,request=dict(role='finish',family='moon'),
        progress=dict(phase='prelude',coverage=.85,decisions=40)) for i in range(12)])
    text='\n'.join(t for t,_ in ui.frame(s,width=80,height=24)[0])
    assert '140/192' in text and 'E00' in text and '按 2' in text


def test_countdown_resets_on_pause_restart_and_insufficient_samples(ui):
    from course_two_trial_panel import Countdown
    timer=Countdown()
    assert timer.estimate('q',10,100,100,True) is None
    assert timer.estimate('q',10,100,130,True) is None
    estimate=timer.estimate('q',12,100,160,True)
    assert estimate['seconds']==2640
    assert timer.estimate('q',12,100,170,False) is None
    assert timer.estimate('q',12,100,180,True) is None
    assert timer.estimate('eval',0,190,200,True) is None


def test_panel_accepts_only_hash_bound_adopted_baseline(ui,evaluated_root):
    root=evaluated_root;identity=json.loads((root/'identity.json').read_text())
    identity['runtime']='runtime-two';adoptions={}
    for path in (root/'evaluation/baseline').glob('*.json'):
        adoptions[str(path.relative_to(root))]=dict(sha=sha(path),
            from_runtime='runtime-one',to_runtime='runtime-two')
    identity['evaluation_cache_adoptions']=adoptions;write(root/'identity.json',identity)
    assert ui.Reader(root,lambda _:None).snapshot()['baseline']['done']==5
    path=root/'evaluation/baseline/n0.json';value=json.loads(path.read_text())
    value['wall_s']=99.;write(path,value)
    assert ui.Reader(root,lambda _:None).snapshot()['baseline']['done']==4


def reward_step(episode,env,step,transition,*,exploration=2.,approach=0.,completed=False):
    # The large cumulative turn includes initialization and MUST NOT be charged again.
    base=exploration-.002*5.-.005-.001+(20. if completed else 0.)
    return dict(event='transition',episode_id=episode,env=env,steps=step,transitions=transition,
        token=transition,seed=transition,curriculum_task='moon_outside',budget=256,
        reward=base+approach,base_reward=base,approach_reward=approach,
        last_execution=dict(distance_m=5.,turn_rad=math.pi),turn_rad=999.,
        reference_coverage=.99 if completed else .4,completed=completed,
        terminated=completed,truncated=False)


def reward_log(root,rows,*,append=False,environments=None):
    run=root/'arms/C3/training/run'
    write(root/'arms/C3/config.json',dict(completion_bonus=20.))
    write(root/'state.json',dict(status='paused',phase='training'))
    if environments is None:
        environments={str(row['env']):row for row in rows}
    write(run/'progress.json',dict(transitions=max((r['transitions'] for r in rows),default=0),
        environments=environments))
    with (run/'metrics.jsonl').open('a' if append else 'w',encoding='utf-8') as stream:
        for row in rows:stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    return run/'metrics.jsonl'


def test_scene_rewards_separate_interleaved_episodes_and_show_only_two_rewards(ui,root):
    rows=[reward_step('outside-a',0,1,1,approach=.5),
        reward_step('outside-b',1,1,2,exploration=4.,approach=-.7),
        reward_step('outside-a',0,2,3,exploration=3.,approach=-.2,completed=True)]
    reward_log(root,rows)
    reader=ui.Reader(root,lambda _:None)
    s=reader.snapshot();episodes=s.get('episode_rewards',{})
    assert 'current' in episodes, 'monitor needs per-episode rewards'
    active=episodes['current'][0];ended=episodes['recent'][0]
    assert active['episode_id']=='outside-b' and ended['episode_id']=='outside-a'
    assert active['reward_sum']==pytest.approx(3.284)
    assert ended['reward_sum']==pytest.approx(25.268)
    assert ended['approach_reward']==pytest.approx(.3)
    assert ended['complete'] and ended['recorded_steps']==ended['steps']==2
    text='\n'.join(t for t,_ in ui.body(s,'training',144))
    for token in ['每个场景的回合奖励','outside-a','outside-b','25.27','3.28','-0.70']:
        assert token in text
    assert '累计总奖励' not in text
    header=next(line for line in text.splitlines() if line.startswith('环境 '))
    assert header.split()==['环境','回合编号','任务','状态','步数','覆盖率','接近辅助','总奖励']
    before={p:p.read_bytes() for p in root.rglob('*') if p.is_file()}
    assert reader.snapshot()['episode_rewards']==episodes
    assert ui.Reader(root,lambda _:None).snapshot()['episode_rewards']==episodes
    assert before=={p:p.read_bytes() for p in root.rglob('*') if p.is_file()}


def test_scene_rewards_incremental_reads_wait_for_complete_lines_and_reset_on_new_episode(ui,root):
    first=reward_step('first',0,1,1,approach=-.5)
    path=reward_log(root,[first]);reader=ui.Reader(root,lambda _:None)
    initial=reader.snapshot().get('episode_rewards',{})
    assert 'current' in initial, 'monitor needs per-episode rewards'
    second=reward_step('first',0,2,2,completed=True)
    payload=json.dumps(second)+'\n'
    with path.open('a') as f:f.write(payload[:-10])
    assert reader.snapshot()['episode_rewards']['current'][0]['recorded_steps']==1
    with path.open('a') as f:f.write(payload[-10:])
    third=reward_step('second',0,1,3,exploration=6.)
    reward_log(root,[third],append=True)
    s=reader.snapshot()['episode_rewards']
    assert s['current'][0]['episode_id']=='second'
    assert s['current'][0]['reward_sum']==pytest.approx(5.984)
    assert s['recent'][0]['episode_id']=='first' and s['recent'][0]['recorded_steps']==2


def test_scene_rewards_survive_in_place_log_compaction_without_double_counting(ui,root):
    rows=[reward_step('long',0,i,i,approach=-.1) for i in range(1,5)]
    path=reward_log(root,rows[:3]);reader=ui.Reader(root,lambda _:None)
    assert 'current' in reader.snapshot().get('episode_rewards',{})
    # Rolling writer replaces the prefix in-place; inode and size can stay alike.
    reward_log(root,rows[1:])
    row=reader.snapshot()['episode_rewards']['current'][0]
    assert row['complete'] and row['recorded_steps']==4
    assert row['reward_sum']==pytest.approx(4*1.884)
    # A reopened monitor cannot invent the discarded first action.
    recovered=ui.Reader(root,lambda _:None).snapshot()
    row=recovered['episode_rewards']['current'][0]
    assert not row['complete'] and row['recorded_steps']==3
    text='\n'.join(t for t,_ in ui.body(recovered,'training',144))
    assert '记录不全' in text and '5.65' not in text


def test_scene_rewards_exclude_old_bound_and_do_not_transfer_old_episode_to_reset(ui,root):
    write(root/'identity.json',dict(reward_adjustment=dict(boundary=dict(transitions=100))))
    rows=[reward_step('old-bound',0,1,99,exploration=999.),
        reward_step('new-bound',0,1,101,approach=-.5)]
    reward_log(root,rows,environments={'0':dict(episode_id='brand-new',steps=0,
        curriculum_task='cave_inside',state='READY',initial_reference_coverage=.1)})
    s=ui.Reader(root,lambda _:None).snapshot()
    assert 'current' in s.get('episode_rewards',{})
    row=s['episode_rewards']['current'][0]
    assert row['episode_id']=='brand-new' and row['reward_sum']==0.
    assert row['complete'] and row['recorded_steps']==0
    assert all(r['episode_id']!='old-bound' for r in s['episode_rewards']['recent'])
    assert '999' not in '\n'.join(t for t,_ in ui.body(s,'training',144))


@pytest.mark.parametrize('bad', ['missing_auxiliary','step_gap','invalid_reward'])
def test_scene_rewards_never_present_incomplete_records_as_full_return(ui,root,bad):
    rows=[reward_step('incomplete',0,1,1),reward_step('incomplete',0,2,2)]
    if bad=='missing_auxiliary':rows[0].pop('approach_reward')
    elif bad=='step_gap':rows[1]['steps']=3
    else:rows[0]['reward']=float('nan')
    reward_log(root,rows)
    s=ui.Reader(root,lambda _:None).snapshot()
    assert 'current' in s.get('episode_rewards',{})
    assert not s['episode_rewards']['current'][0]['complete']
    assert '记录不全' in '\n'.join(t for t,_ in ui.body(s,'training',144))


@pytest.mark.parametrize('width,height',[(80,24),(154,46)])
def test_scene_reward_table_remains_readable_with_all_twelve_environments(ui,root,width,height):
    reward_log(root,[reward_step(f'episode-{i:02}',i,1,i+1,approach=-.5) for i in range(12)])
    s=ui.Reader(root,lambda _:None).snapshot()
    text='\n'.join(t for t,_ in ui.body(s,'training',width-1))
    assert '每个场景的回合奖励' in text
    assert all(f'E{i:02}' in text for i in range(12))
    first,_,_=ui.frame(s,'training',width,height)
    last,_,_=ui.frame(s,'training',width,height,100000)
    assert all(ui.cells(t)<width for t,_ in first+last)
    assert 'P 保存停止' in first[-1][0]
