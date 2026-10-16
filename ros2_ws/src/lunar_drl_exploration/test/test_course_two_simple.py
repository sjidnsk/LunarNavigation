from collections import Counter
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig

TASKS=('moon_outside','moon_inside','cave_inside')


def config():
    return TrainingConfig(course_two=True,course_two_variant='C3',course_two_task_scope='whole_cave_v1',
        course_two_sampling='scene_episode_v1',environments=12)


def test_three_scene_curriculum_generates_whole_episodes_without_catalog_or_prefix():
    from lunar_drl_exploration.training import Curriculum
    c=Curriculum(config(),np.random.default_rng(1))
    specs=[c.next(i,0) for i in range(12)]
    assert Counter(s['curriculum_task'] for s in specs)==dict.fromkeys(TASKS,4)
    assert all('course_two_case' not in s for s in specs)
    assert all(s['start_mode']=='inside' for s in specs if s['family']=='cave')
    assert all(s['task_scope']=='whole_cave_v1' for s in specs if s['family']=='cave')
    assert all(s['outside_distance_max_m']==20. for s in specs if s['curriculum_task']=='moon_outside')
    saved=c.snapshot();d=Curriculum(config(),np.random.default_rng(2),saved)
    assert [d.next(i,0) for i in range(12)]==specs
    assert d.course.scene_issued==c.course.scene_issued==dict.fromkeys(TASKS,4)


def test_three_scene_replay_ignores_phase_and_success_and_balances_episodes():
    from lunar_drl_exploration.replay import ReplayBuffer
    from test_course_two_replay import add_episode
    replay=ReplayBuffer(128*1024**2)
    # Attach the actual scene scope; no success terminal or late-coverage pool exists.
    for task in TASKS:
        for i in range(6):
            records=add_episode(replay,task,task+str(i),count=8,covered=0 if i%2 else 95)
            if task=='cave_inside':
                key=records[0].privileged.scene_id
                replay._scenes[key]=replace(replay._scenes[key],generator_descriptor=dict(
                    curriculum_task=task,family='cave',task_scope='whole_cave_v1'))
    add_episode(replay,'cave_outside','obsolete-outside')
    index=replay.enable_course_two_sampling(task_scope='whole_cave_v1',sampling='scene_episode_v1')
    assert index.status()['ready'] and set(index.status()['pool_counts'])==set(TASKS)
    rng=np.random.default_rng(52);extra=set()
    for _ in range(30):
        batch=replay.sample_course_two(rng)
        counts=Counter(replay.scenes[t.privileged.scene_id].generator_descriptor['curriculum_task'] for t in batch)
        assert sorted(counts.values())==[21,21,22]
        extra.update(k for k,v in counts.items() if v==22)
        assert len({(t.episode_id,t.step_index) for t in batch})==64
        assert max(Counter(t.episode_id for t in batch).values())<=4
    assert extra==set(TASKS)
    restored=ReplayBuffer.from_snapshot(replay.snapshot(32*1024**2,required_indices=index.required_indices()),replay.max_bytes)
    assert restored.enable_course_two_sampling(task_scope='whole_cave_v1',sampling='scene_episode_v1').status()['ready']


def test_simple_development_has_twenty_normal_episodes_per_type_and_no_cave_outside():
    from lunar_drl_exploration.course_two_simple import development_cases
    cases=development_cases(config())
    assert len(cases)==60 and Counter(c['capability'] for c in cases)==dict.fromkeys(TASKS,20)
    assert len({c['spec']['seed'] for c in cases})==60
    assert all('course_two_case' not in c['spec'] for c in cases)
    assert all(c['spec']['start_mode']=='inside' for c in cases if c['spec']['family']=='cave')


def test_slow_moon_gets_more_workers_without_blocking_ready_cave():
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    course=SimpleCourse()
    # Historical moon replay is full; it must not skew new scene quotas.
    course.set_supply({'episode_capacity':dict(moon_outside=100,moon_inside=100,cave_inside=0)})
    for env in range(12):course.activate(env,course.next_task())
    assert Counter(course.active.values())==dict.fromkeys(TASKS,4)
    # Cave episodes finish first even if individual actions take the same time.
    for env in [e for e,t in course.active.items() if t=='cave_inside']:
        course.release(env);course.activate(env,course.next_task())
    counts=Counter(course.active.values())
    assert counts['moon_outside']>counts['cave_inside']>=1
    assert counts['moon_inside']>counts['cave_inside']
    cave=next(e for e,t in course.active.items() if t=='cave_inside')
    course.phase_transitions['cave_inside']=1000
    assert course.select_pending({cave:'ready'},set())==cave
    restored=SimpleCourse(course.state_dict())
    assert restored.action_seconds==course.action_seconds
    assert restored.scene_issued==course.scene_issued
    for i in range(35):
        assert restored.next_task()==course.next_task()
        assert max(course.scene_issued.values())-min(course.scene_issued.values())<=1


@pytest.mark.parametrize('field,value',[
    ('phase_transitions',-1),('phase_samples',2.5),('scene_issued',-1),
    ('coverage_stats',dict(count=-1,sum=float('nan'),recent=[2.])),
])
def test_simple_restore_rejects_corrupt_counters(field,value):
    from lunar_drl_exploration.course_two_simple import SimpleCourse
    state=SimpleCourse().state_dict();state[field]['moon_inside']=value
    with pytest.raises(ValueError):SimpleCourse(state)


def test_bounded_moon_start_and_natural_cave_scope():
    from lunar_drl_exploration.scene import Scene,TerrainGrid
    from lunar_drl_exploration.course_two_task_scope import build_rehearsal
    from lunar_drl_exploration.geometry import polygon_mask
    from scipy.ndimage import distance_transform_edt
    for seed in (7,91,502):
        moon=Scene(seed,'moon',100.,platform=config().platform)
        terrain=TerrainGrid.from_scene(moon)
        pose=moon.initial_pose(terrain,'outside',outside_distance_max_m=20.)
        roi=polygon_mask(terrain.shape,terrain.origin,terrain.resolution_m,moon.task.polygon)
        x,y=terrain.world_to_cell(pose.x,pose.y)
        assert not roi[y,x] and distance_transform_edt(~roi)[y,x]*terrain.resolution_m<=20.
        original=TerrainGrid.from_scene(Scene(seed,'cave',60.,platform=config().platform))
        cave,pose,task,descriptor=build_rehearsal(seed,60.,'inside',config().platform)
        assert np.array_equal(cave.heights,original.heights)
        roi=polygon_mask(cave.shape,cave.origin,cave.resolution_m,task.polygon)
        assert not np.any((cave.heights==0)&~roi)
        x,y=cave.world_to_cell(pose.x,pose.y)
        assert roi[y,x] and cave.navigation[y,x]==1


def test_simple_gate_requires_complete_same_cases_and_retains_each_scene_type():
    from copy import deepcopy
    from lunar_drl_exploration.course_two_trial import gate
    baseline=[dict(case_id=c['case_id'],kind='simple',capability=c['capability'],
        completed=True,collisions=0,error=None,decisions=10) for c in
        __import__('lunar_drl_exploration.course_two_simple',fromlist=['development_cases']).development_cases(config())]
    assert gate(baseline,baseline)['passed']
    candidate=deepcopy(baseline)
    candidate[0]['completed']=candidate[1]['completed']=False
    assert gate(candidate,baseline)['stop']
    with pytest.raises(ValueError,match='identity'):gate(candidate[:-1],baseline)
