"""Strict course starts must preserve native component and terrain identity."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig, resume_semantics, config_record, training_config_from_record
from lunar_drl_exploration.scene import Scene, TerrainGrid


@pytest.mark.parametrize('family', ['moon', 'cave'])
def test_course_start_classification_and_same_component(family):
    scene = Scene(23, family, 50, .5)
    terrain = TerrainGrid.from_scene(scene)
    original = scene.initial_pose(terrain)
    main = terrain.reachable(original)
    from lunar_drl_exploration.geometry import polygon_mask
    inside = polygon_mask(terrain.shape, terrain.origin, terrain.resolution_m, scene.task.polygon)
    for mode in ('inside', 'outside'):
        pose = scene.initial_pose(terrain, start_mode=mode)
        x, y = terrain.world_to_cell(pose.x, pose.y)
        assert bool(inside[y, x]) == (mode == 'inside')
        assert main[y, x] == terrain.navigation[y, x] == 1
        assert pose == scene.initial_pose(terrain, start_mode=mode)
        assert pose.yaw == original.yaw
    assert scene.initial_pose(terrain, start_mode='any') == original
    assert terrain.terrain_id == scene.scene_id


def test_invalid_or_unavailable_start_fails_without_fallback():
    scene = Scene(23, 'moon', 50, .5)
    terrain = TerrainGrid.from_scene(scene)
    with pytest.raises(ValueError, match='start_mode'):
        scene.initial_pose(terrain, start_mode='invalid')
    # Use a legal component entirely inside the hand-defined task polygon.
    fake = SimpleNamespace(terrain_id=scene.scene_id, navigation=np.ones((2, 2), np.uint8),
        shape=(2, 2), origin=(0., 0.), resolution_m=.5, cell_center=lambda x,y: (.25+.5*x,.25+.5*y))
    with pytest.raises(ValueError, match='outside'):
        scene.initial_pose(fake, start_mode='outside')
    fake.origin = (100., 100.)
    fake.cell_center = lambda x,y: (100.25+.5*x,100.25+.5*y)
    with pytest.raises(ValueError, match='inside'):
        scene.initial_pose(fake, start_mode='inside')


def test_course_config_roundtrip_and_legacy_resume_compatibility():
    default = TrainingConfig()
    assert default.ordered_curriculum is False
    assert 'ordered_curriculum' not in resume_semantics(default)
    course = replace(default, ordered_curriculum=True, ordered_budget_rule={'version': 'area_v1'})
    restored = training_config_from_record(config_record(course))
    assert restored == course
    semantics = resume_semantics(course)['ordered_curriculum']
    assert semantics['rehearsal_fraction'] == .2
    assert semantics['budget_rule'] == {'version': 'area_v1'}
    assert resume_semantics(replace(course, ordered_rehearsal_fraction=.3)) != resume_semantics(course)
    for kwargs in ({'ordered_curriculum': 1}, {'ordered_rehearsal_fraction': -1},
                   {'ordered_rehearsal_fraction': 1}, {'ordered_budget_rule': []}):
        with pytest.raises(ValueError):
            replace(default, **kwargs)


def test_reset_persists_static_task_and_start_descriptor_without_changing_terrain(monkeypatch):
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env = RosExplorationEnv(TrainingConfig(), 0)
    monkeypatch.setattr(env, 'close', lambda: None)
    captured = {}
    def capture(terrain, pose, task, *, descriptor):
        captured.update(terrain=terrain, pose=pose, task=task, descriptor=descriptor)
        return 'observation', 'privileged'
    monkeypatch.setattr(env, 'reset_scene', capture)
    assert env.reset(23, 'moon', 50, start_mode='inside', curriculum_task='moon_inside') == ('observation', 'privileged')
    assert captured['descriptor']['start_mode'] == 'inside'
    assert captured['descriptor']['curriculum_task'] == 'moon_inside'
    assert captured['terrain'].generator_descriptor['curriculum_task'] == 'moon_inside'
    assert captured['terrain'].generator_descriptor['start_mode'] == 'inside'
    assert captured['terrain'].terrain_id == Scene(23, 'moon', 50).scene_id
    with pytest.raises(ValueError, match='curriculum_task'):
        env.reset(23, 'moon', 50, start_mode='inside', curriculum_task='cave_outside')


def test_default_reset_retains_unclassified_legacy_replay(monkeypatch):
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env = RosExplorationEnv(TrainingConfig(), 0)
    monkeypatch.setattr(env, 'close', lambda: None)
    monkeypatch.setattr(env, 'reset_scene', lambda terrain, pose, task, **kwargs: terrain)
    terrain = env.reset(23, 'moon', 50)
    assert terrain.generator_descriptor.get('curriculum_task', 'unclassified') == 'unclassified'
