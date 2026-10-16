from copy import deepcopy
import importlib.util

import numpy as np
import pytest

from lunar_drl_exploration.config import load_platform_config
from lunar_drl_exploration.geometry import polygon_mask


def api():
    name='lunar_drl_exploration.course_two_rebalanced_scene'
    assert importlib.util.find_spec(name), 'missing rebalanced scene generator'
    return __import__(name,fromlist=['make_case'])


def request(role='cover', family='moon', template='internal_occlusion', start='inside'):
    return dict(seed=2026101100090, role=role, family=family, planned_policy_start=start,
        bearing_bin=2, roi_side_interval_m=[32.,40.], terrain_template=template,
        sensor=dict(range_m=10.,fov_deg=90.), request_id='test-request',
        visibility_after_native_initialization='U', blind_gap_m=[1.,5.],
        policy_start_coverage=dict(gt=0.,lt=.8), episode_decision_budget=512)


@pytest.mark.parametrize('family',['moon','cave'])
def test_new_coverage_starts_inside_with_real_internal_occlusion(family):
    m=api();case=m.make_case(request(family=family))
    terrain,pose,task,sensor,desc=m.build_case(case,load_platform_config())
    roi=polygon_mask(terrain.shape,terrain.origin,terrain.resolution_m,task.polygon)
    x,y=terrain.world_to_cell(pose.x,pose.y)
    assert roi[y,x] and terrain.navigation[y,x]==1
    assert np.any((terrain.heights>1)&roi)
    assert desc['start_mode']=='inside' and sensor.range_m==10
    assert desc['scenario_id']==case['scenario_id']


def test_required_detour_disconnects_roi_but_has_a_safe_outside_connection():
    m=api();case=m.make_case(request(template='outside_detour'))
    terrain,pose,task,_,desc=m.build_case(case,load_platform_config())
    assert desc['outside_detour_certified']
    assert desc['internal_obstacle_cells']>0
    assert len(desc['detour_witness_world'])==2


@pytest.mark.parametrize('visibility',['U','O'])
def test_approach_has_zero_initial_visibility_and_bounded_observation_route(visibility):
    m=api();r=request('approach',start='outside')
    r.update(visibility_after_native_initialization=visibility,
        terrain_template='bounded_open_approach' if visibility=='U' else 'short_occluded_approach')
    case=m.make_case(r)
    _,_,_,_,desc=m.build_case(case,load_platform_config())
    assert desc['initial_observable_roi_cells']==0
    assert 0 < desc['reference_path_m'] <=20
    assert desc['reference_endpoint_roi_cells']>0


def test_scene_sensor_and_role_are_bound_to_identity():
    m=api();case=m.make_case(request())
    changed=deepcopy(case);changed['sensor']['range_m']=20.
    with pytest.raises(ValueError,match='identity'):
        m.build_case(changed,load_platform_config())
    assert m.make_case(request())==case
    assert m.make_case(request(),attempt=1)['scenario_id']!=case['scenario_id']


def test_native_start_classifier_rejects_wrong_visibility_completion_or_position():
    m=api()
    m.validate_start('cover',.5,True)
    m.validate_start('approach',0.,False)
    m.validate_start('finish',.95,True)
    m.validate_start('return',.95,False)
    for role,cov,inside in [('approach',.01,False),('cover',1.,True),('finish',.5,True),('return',.95,True)]:
        with pytest.raises(ValueError):m.validate_start(role,cov,inside)
