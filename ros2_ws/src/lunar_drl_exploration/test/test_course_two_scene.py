"""A scene keeps its sensor identity; geometry teaches all approach directions."""
from dataclasses import replace
import math
import numpy as np
import pytest


def test_case_identity_is_stable_and_binds_the_sensor():
    from lunar_drl_exploration.course_two_scene import make_case, validate_case
    first = make_case(17, 'C', 0, range_m=100., fov_deg=360., visibility='U')
    assert first == make_case(17, 'C', 0, range_m=100., fov_deg=360., visibility='U')
    assert first['sensor']['range_m'] == 100.
    assert first['sensor']['fov_deg'] == 360.
    changed = dict(first, sensor=dict(first['sensor'], range_m=10.))
    with pytest.raises(ValueError, match='identity'):
        validate_case(changed)
    assert make_case(17, 'C', 0, range_m=10., fov_deg=360., visibility='U')['scenario_id'] != first['scenario_id']


def test_eight_bearings_and_finite_distance_are_generated():
    from lunar_drl_exploration.course_two_scene import make_case
    for stage in ('A1', 'A2', 'B', 'C', 'D'):
        cases = [make_case(30+i, stage, i, visibility='U') for i in range(8)]
        assert {c['bearing_bin'] for c in cases} == set(range(8))
        for c in cases:
            assert 1 <= c['blind_gap_m'] <= (5 if stage == 'A1' else 20)
            assert c['start_distance_m'] <= c['sensor']['range_m'] + 20
            assert c['start_distance_m'] <= 120
            assert c['episode_budget'] == 512


def test_omnidirectional_never_gets_an_out_of_fov_task():
    from lunar_drl_exploration.course_two_scene import make_case
    c = make_case(3, 'C', 1, range_m=10., fov_deg=360., visibility='F')
    assert c['visibility_class'] == 'U'


def test_initial_pose_uses_existing_cell_center_convention_without_changing_gap():
    from lunar_drl_exploration.course_two_scene import make_case,build_case
    from lunar_drl_exploration.config import load_platform_config
    c=make_case(27102028,'A1',2,range_m=10.,fov_deg=180.,visibility='U',family='moon')
    t,p,task,_,_=build_case(c,load_platform_config())
    assert np.allclose((p.x,p.y),t.cell_center(*t.world_to_cell(p.x,p.y)),atol=1e-9,rtol=0.)
    edge=(task.polygon[0]+task.polygon[3])/2
    assert np.linalg.norm(edge-np.array([p.x,p.y]))==pytest.approx(c['start_distance_m'])


@pytest.mark.parametrize('family', ['moon', 'cave'])
@pytest.mark.parametrize('visibility', ['U', 'F', 'V', 'O'])
def test_native_geometry_has_a_short_reachable_observation_path(family, visibility):
    from lunar_drl_exploration.course_two_scene import make_case, build_case
    from lunar_drl_exploration.config import load_platform_config
    from lunar_drl_exploration.geometry import polygon_mask
    from lunar_drl_exploration.sensor import SensorModel
    c = make_case(104, 'A2', 0, visibility=visibility, family=family)
    terrain, pose, task, sensor, descriptor = build_case(c, load_platform_config())
    x, y = terrain.world_to_cell(pose.x, pose.y)
    assert terrain.navigation[y, x] == 1
    assert descriptor['reference_path_m'] <= 40
    mask = polygon_mask(terrain.shape, terrain.origin, terrain.resolution_m, task.polygon)
    observed = SensorModel.observe(terrain, pose, sensor).mask
    effective = mask & (terrain.intrinsic != 0)
    assert bool(np.any(observed & effective)) == (visibility == 'V')
    endpoint = descriptor['reference_observation_pose']
    from lunar_drl_exploration.contracts import Pose
    assert np.any(SensorModel.observe(terrain, Pose(*endpoint), sensor).mask & effective)
    assert descriptor['sensor'] == c['sensor']
    assert descriptor['scenario_id'] == c['scenario_id']


def test_rebuilding_a_scene_preserves_reference_and_truth_identity():
    from lunar_drl_exploration.course_two_scene import make_case, build_case
    from lunar_drl_exploration.config import load_platform_config
    from lunar_drl_exploration.reference import CoverageReference
    c = make_case(512, 'A1', 3, visibility='U')
    outputs = [build_case(c, load_platform_config()) for _ in range(2)]
    refs = [CoverageReference.build(t, p, task, sensor) for t,p,task,sensor,_ in outputs]
    assert refs[0].reference_id == refs[1].reference_id
    assert np.array_equal(outputs[0][0].heights, outputs[1][0].heights)
    assert refs[0].cell_count > 0


def test_environment_routes_scene_sensor_to_all_consumers_and_restores_default(monkeypatch):
    from lunar_drl_exploration.course_two_scene import make_case, build_case
    from lunar_drl_exploration.config import TrainingConfig, load_platform_config
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.reference import CoverageReference
    c = make_case(512, 'C', 3, range_m=20., fov_deg=360., visibility='U')
    terrain, pose, task, sensor, descriptor = build_case(c, load_platform_config())
    env = RosExplorationEnv(TrainingConfig(), 0)
    checked = []
    # Stop at the external ROS startup boundary; actual sensor/reference/core
    # construction above it remains real and is what this test checks.
    def boundary():
        checked.append(env.sensor)
        assert env.core.sensor == env.sensor
        assert env.privileged_builder.sensor == env.sensor
        expected = CoverageReference.build(terrain, pose, task, env.sensor)
        assert env.reference.reference_id == expected.reference_id
        raise RuntimeError('external ROS boundary')
    monkeypatch.setattr(env, '_start_ros', boundary)
    with pytest.raises(RuntimeError, match='external ROS boundary'):
        env.reset_scene(terrain, pose, task, sensor=sensor, descriptor=descriptor)
    with pytest.raises(RuntimeError, match='external ROS boundary'):
        env.reset_scene(terrain, pose, task)
    assert checked == [sensor, env.config.sensor]
