"""Approved distance/sensor extension; preserve the original scene protocol."""
from collections import Counter
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig, resume_semantics
from lunar_drl_exploration.course_two_simple import TASKS, fresh_scene

PROFILE='distance_sensor_10_30_v1'


def config():
    return TrainingConfig(course_two=True,course_two_variant='C3',course_two_task_scope='whole_cave_v1',
        course_two_sampling='scene_episode_v1',course_two_scene_profile=PROFILE,environments=12)


def boundary_distance(pose,polygon):
    point=np.asarray([pose.x,pose.y]);a=np.asarray(polygon,dtype=float);b=np.roll(a,-1,axis=0)
    v=b-a;t=np.clip(np.sum((point-a)*v,axis=1)/np.sum(v*v,axis=1),0,1)
    return float(np.min(np.linalg.norm(point-(a+t[:,None]*v),axis=1)))


def test_sensor_profile_is_explicit_and_bound_to_resume():
    c=config();old=replace(c,course_two_scene_profile='')
    assert resume_semantics(c)!=resume_semantics(old)
    contract=resume_semantics(c)['course_two']
    assert contract['outside_distance_m']==[10.,50.]
    assert contract['sensor_scope']=='immutable_scenario'
    assert contract['range_m']==[10.,30.]
    assert contract['scene']=='three_ordinary_distance_sensor_10_30_v1'
    with pytest.raises(ValueError):replace(c,course_two_scene_profile='unknown')
    with pytest.raises(ValueError):replace(c,course_two_sampling='')


def test_all_scene_types_receive_balanced_fixed_sensor_specs_and_original_anchor():
    for task in TASKS:
        specs=[fresh_scene(config(),task,i) for i in range(75)]
        assert specs==[fresh_scene(config(),task,i) for i in range(75)]
        assert len({s['seed'] for s in specs})==75
        anchors=[s['sensor'] for i,s in enumerate(specs) if i%5==0]
        assert anchors==[dict(range_m=10.,fov_deg=90.)]*15
        varied=[s['sensor'] for i,s in enumerate(specs) if i%5]
        assert all(10<=s['range_m']<=30 for s in varied)
        assert Counter(s['fov_deg'] for s in varied)==dict.fromkeys((60.,90.,120.,180.,360.),12)
        bins=Counter(0 if s['range_m']<20 else 1 for s in varied)
        assert bins=={0:30,1:30}
        assert len({s['range_m'] for s in varied})>45
        for s in specs:
            if task=='moon_outside':
                assert s['outside_distance_min_m']==10. and s['outside_distance_max_m']==50.
                assert s['context_margin_m']>=62.
            else:assert 'outside_distance_min_m' not in s


def test_saved_active_scenes_resume_with_identical_sensors():
    from lunar_drl_exploration.training import Curriculum
    c=Curriculum(config(),np.random.default_rng(1));specs=[c.next(i,0) for i in range(12)]
    restored=Curriculum(config(),np.random.default_rng(55),c.snapshot())
    assert [restored.next(i,0) for i in range(12)]==specs
    assert Counter(s['curriculum_task'] for s in specs)==dict.fromkeys(TASKS,4)


@pytest.mark.parametrize('seed',[7,91,502])
def test_expanded_small_moon_supports_actual_far_starts_connected_to_roi(seed):
    from lunar_drl_exploration.scene import Scene,TerrainGrid
    from lunar_drl_exploration.geometry import polygon_mask
    c=config();s=Scene(seed,'moon',30.,platform=c.platform,context_margin_m=62.)
    grid=TerrainGrid.from_scene(s)
    # Check the far end explicitly: merely widening a configured maximum is insufficient.
    p=s.initial_pose(grid,'outside',outside_distance_min_m=42.,outside_distance_max_m=50.)
    assert 42.<=boundary_distance(p,s.task.polygon)<=50.
    x,y=grid.world_to_cell(p.x,p.y);assert grid.navigation[y,x]==1
    roi=polygon_mask(grid.shape,grid.origin,grid.resolution_m,s.task.polygon)
    assert not roi[y,x] and np.any(grid.reachable(p)&roi)
    assert s.initial_pose(grid,'outside',outside_distance_min_m=42.,outside_distance_max_m=50.)==p
    legacy=Scene(seed,'moon',30.,platform=c.platform)
    assert np.array_equal(s.task.polygon,legacy.task.polygon)
    assert s.scene_id!=legacy.scene_id


@pytest.mark.parametrize('bounds',[(50.,10.),(-1.,50.),(10.,float('nan'))])
def test_invalid_distance_bands_rejected(bounds):
    from lunar_drl_exploration.scene import Scene,TerrainGrid
    s=Scene(8,'moon',30.,platform=config().platform)
    with pytest.raises(ValueError):
        s.initial_pose(TerrainGrid.from_scene(s),'outside',outside_distance_min_m=bounds[0],
            outside_distance_max_m=bounds[1])


@pytest.mark.parametrize('task',TASKS)
def test_sensor_override_reaches_reset_scene_for_all_three_types(monkeypatch,task):
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.contracts import SensorSpec
    env=RosExplorationEnv(config(),0,domain_base=None)
    seen={}
    def capture(terrain,pose,target,*,sensor=None,descriptor=None):
        seen.update(sensor=sensor,descriptor=descriptor,terrain=terrain,pose=pose,task=target)
        return 'observation','privileged'
    monkeypatch.setattr(env,'reset_scene',capture)
    spec=fresh_scene(config(),task,1);spec['sensor']=dict(range_m=30.,fov_deg=360.)
    assert env.reset(**spec)==('observation','privileged')
    assert seen['sensor']==SensorSpec(range_m=30.,fov_deg=360.)
    assert seen['descriptor']['sensor']['range_m']==30.
    assert seen['terrain'].generator_descriptor['sensor']['fov_deg']==360.
    if task=='moon_outside':
        actual=boundary_distance(seen['pose'],seen['task'].polygon)
        assert 10.<=actual<=50.
        assert seen['descriptor']['start_boundary_distance_m']==pytest.approx(actual)


def test_current_profile_replay_preserves_but_does_not_sample_legacy_episodes():
    from lunar_drl_exploration.replay import ReplayBuffer
    from test_course_two_replay import add_episode
    replay=ReplayBuffer(128*1024**2)
    for task in TASKS:
        add_episode(replay,task,'legacy-'+task)
        for i in range(6):
            records=add_episode(replay,task,f'new-{task}-{i}')
            key=records[0].privileged.scene_id
            replay._scenes[key]=replace(replay._scenes[key],generator_descriptor=dict(
                curriculum_task=task,task_scope='whole_cave_v1',scene_profile=PROFILE,
                sensor=dict(range_m=25.,fov_deg=360.)))
    before=len(replay)
    index=replay.enable_course_two_sampling(task_scope='whole_cave_v1',sampling='scene_episode_v1',scene_profile=PROFILE)
    assert index.status()['ready'] and len(replay)==before
    assert index.status()['scope_excluded_transitions']==24
    assert all(t.episode_id.startswith('new-') for t in replay.sample_course_two(np.random.default_rng(5)))
    # Validate admission after enabling the index, and its reconstruction on resume.
    add_episode(replay,'moon_inside','legacy-after-enable',count=1)
    snapshot=replay.snapshot(32*1024**2,required_indices=index.required_indices())
    restored=ReplayBuffer.from_snapshot(snapshot,replay.max_bytes)
    assert restored.enable_course_two_sampling(task_scope='whole_cave_v1',sampling='scene_episode_v1',
        scene_profile=PROFILE).status()['ready']
    with pytest.raises(ValueError):replay.enable_course_two_sampling(task_scope='whole_cave_v1',sampling='scene_episode_v1')


def test_profile_fork_preserves_full_baseline_and_rejects_old_resume(ordered_checkpoint,tmp_path):
    from test_course_two_training import completed_source
    from test_training_fork import assert_state_equal
    from lunar_drl_exploration.checkpoint import TrainingState
    source,path,original=completed_source(ordered_checkpoint,tmp_path)
    dest=replace(source,ordered_curriculum=False,course_two=True,course_two_variant='C3',
        course_two_task_scope='whole_cave_v1',course_two_sampling='scene_episode_v1',
        course_two_scene_profile=PROFILE,environments=12,
        output_dir=str(tmp_path/'sensors/run'),artifact_root=str(tmp_path/'sensors'))
    state=TrainingState.fork_from(path,dest,experiment='course-two-rebalance')
    for key in ('learner','replay','schedule','rng','collector_state'):
        assert_state_equal(state.record[key],original.record[key])
    state.validate(dest)
    with pytest.raises(ValueError,match='incompatible'):state.validate(replace(dest,course_two_scene_profile=''))


from test_ordered_extent_fork import ordered_checkpoint
