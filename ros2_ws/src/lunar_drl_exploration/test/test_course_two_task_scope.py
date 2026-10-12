"""Reject task masks that cut cave interiors or manufacture outside returns."""
from copy import deepcopy
from dataclasses import replace
import numpy as np
import pytest

from lunar_drl_exploration.config import load_platform_config, TrainingConfig
from lunar_drl_exploration.geometry import polygon_mask
from lunar_drl_exploration.course_two_rebalanced_scene import make_case, build_case
from test_course_two_rebalanced_scene import request

SCOPE = 'whole_cave_v1'


def test_scoped_finish_cannot_be_turned_into_an_outside_return():
    r = dict(request('finish', start='outside'), task_scope=SCOPE)
    with pytest.raises(ValueError, match='scope|inside'):
        make_case(r)


def test_scoped_cover_cannot_require_leaving_the_task():
    with pytest.raises(ValueError, match='scope|detour'):
        make_case(dict(request(template='outside_detour'), task_scope=SCOPE))


@pytest.mark.parametrize('bearing', [0, 2, 5, 7])
def test_every_cave_interior_cell_is_in_task_but_exterior_start_is_legal(bearing):
    r = dict(request('approach', family='cave', start='outside'), task_scope=SCOPE,
             bearing_bin=bearing, terrain_template='bounded_open_approach')
    case = make_case(r)
    terrain, pose, task, sensor, desc = build_case(case, load_platform_config())
    yy, xx = np.indices(terrain.shape)
    world_x = terrain.origin[0] + (xx+.5)*terrain.resolution_m
    world_y = terrain.origin[1] + (yy+.5)*terrain.resolution_m
    a = case['bearing_rad']; cx, cy = case['translation']
    local_x = (world_x-cx)*np.cos(a)+(world_y-cy)*np.sin(a)
    interior_floor = (terrain.heights == 0) & (local_x >= -case['roi_width_m']/2)
    roi = polygon_mask(terrain.shape, terrain.origin, terrain.resolution_m, task.polygon)
    assert np.any(interior_floor)
    assert not np.any(interior_floor & ~roi), 'cave floor was cut by ROI'
    x, y = terrain.world_to_cell(pose.x, pose.y)
    assert not roi[y, x] and terrain.navigation[y, x] == 1
    assert desc['task_scope'] == SCOPE and desc['cave_interior_outside_cells'] == 0


def test_legacy_cave_replay_is_retained_but_not_sampled_as_whole_cave():
    from lunar_drl_exploration.replay import ReplayBuffer
    from test_course_two_replay import add_episode
    replay = ReplayBuffer()
    add_episode(replay, 'cave_inside', 'legacy')
    add_episode(replay, 'moon_inside', 'moon')
    before = len(replay)
    index = replay.enable_course_two_sampling(task_scope=SCOPE)
    assert len(replay) == before
    assert 'cave_inside' not in index.status()['pool_counts']
    assert index.status()['pool_counts']['moon_inside'] == 8
    assert index.status()['scope_excluded_transitions'] == 8
    restored=ReplayBuffer.from_snapshot(replay.snapshot(1024**2),replay.max_bytes)
    assert restored.enable_course_two_sampling(task_scope=SCOPE).status()['scope_excluded_transitions']==8
    replay.evict_oldest()
    assert index.status()['scope_excluded_transitions']==7


def test_corrected_rehearsal_keeps_existing_cave_rooms_inside():
    from lunar_drl_exploration.course_two_task_scope import build_rehearsal
    for start in ('inside', 'outside'):
        terrain, pose, task, desc = build_rehearsal(42, 50., start, load_platform_config())
        roi = polygon_mask(terrain.shape, terrain.origin, terrain.resolution_m, task.polygon)
        xx = terrain.origin[0] + (np.arange(terrain.shape[1])+.5)*terrain.resolution_m
        cave_floor = (terrain.heights == 0) & (xx[None, :] >= task.polygon[:, 0].min())
        assert np.any(cave_floor) and not np.any(cave_floor & ~roi)
        x, y = terrain.world_to_cell(pose.x, pose.y)
        assert bool(roi[y, x]) == (start == 'inside')
        assert terrain.navigation[y, x] == 1 and desc['task_scope'] == SCOPE


def test_scoped_manifests_have_only_normal_phases_and_fully_inside_finish():
    from lunar_drl_exploration.course_two_task_scope import scene_requests
    training, development = scene_requests()
    assert len(training) == 128 and len(development) == 64
    assert {r['role'] for r in training} == {'old', 'cover', 'approach', 'finish'}
    assert {r['role'] for r in development} == {'cover', 'finish'}
    assert all(r['planned_policy_start']=='inside' for r in training+development if r['role']=='finish')
    assert all(r['terrain_template']!='outside_detour' for r in training+development)
    assert len({r['request_id'] for r in training+development}) == 192


def test_scope_is_preserved_after_catalog_exhaustion_and_bound_to_resume():
    from lunar_drl_exploration.course_two_rebalance import fresh_spec
    from lunar_drl_exploration.config import resume_semantics
    config=TrainingConfig(course_two=True,course_two_variant='C3',course_two_catalog='/catalog',
        course_two_catalog_sha='a'*64,course_two_task_scope=SCOPE)
    spec=fresh_spec(config,'cave_outside',200,[])
    assert spec['task_scope']==SCOPE and spec['start_mode']=='outside'
    assert resume_semantics(config)['course_two']['task_scope']==SCOPE


def test_scoped_gate_accepts_family_phase_groups_without_special_return_tests():
    from test_course_two_trial import rows
    from lunar_drl_exploration.course_two_trial import gate
    records=rows()
    names=dict(cover='moon_cover',finish='cave_cover',return_='moon_finish',detour='cave_finish')
    names['return']=names.pop('return_')
    for row in records:
        if row.get('capability'):row['capability']=names[row['capability']]
        if row['kind']=='diagnostic':
            row['promotion_required']=False;row['completed']=False
    baseline=deepcopy(records)
    for row in baseline:
        if row['kind']=='diagnostic':row['completed']=True
    assert gate(records,baseline)['passed']
    changed=deepcopy(records);changed[20]['promotion_required']=True
    with pytest.raises(ValueError,match='identity'):gate(changed,baseline)


def test_scoped_fork_keeps_learned_state_and_rejects_wrong_scope_on_resume(ordered_checkpoint,tmp_path):
    from test_course_two_training import completed_source
    from test_training_fork import assert_state_equal
    from lunar_drl_exploration.checkpoint import TrainingState
    config,path,original=completed_source(ordered_checkpoint,tmp_path)
    dest=replace(config,course_two=True,ordered_curriculum=False,course_two_variant='C3',
        environments=12,course_two_catalog='/qualified.json',course_two_catalog_sha='a'*64,
        course_two_task_scope=SCOPE,output_dir=str(tmp_path/'scoped/run'),artifact_root=str(tmp_path/'scoped'))
    fork=TrainingState.fork_from(path,dest,experiment='course-two-rebalance')
    for key in ('learner','replay','schedule','rng','collector_state'):
        assert_state_equal(fork.record[key],original.record[key])
    fork.validate(dest)
    with pytest.raises(ValueError,match='incompatible'):
        fork.validate(replace(dest,course_two_task_scope=''))


from test_ordered_extent_fork import ordered_checkpoint
