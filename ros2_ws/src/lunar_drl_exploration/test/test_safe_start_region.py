"""Catch unsafe initial support, hidden terrain edits and dataset identity drift."""
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig, config_record, training_config_from_record, resume_semantics, load_platform_config
from lunar_drl_exploration.scene import Scene, TerrainGrid


def assert_disk_free(terrain, pose, radius):
    # Independent disk-square intersection oracle, including boundary contact.
    ox,oy=terrain.origin;res=terrain.resolution_m
    yy,xx=np.indices(terrain.shape)
    dx=np.maximum(np.maximum(ox+xx*res-pose.x,pose.x-(ox+(xx+1)*res)),0)
    dy=np.maximum(np.maximum(oy+yy*res-pose.y,pose.y-(oy+(yy+1)*res)),0)
    touches=dx*dx+dy*dy<=radius*radius+1e-10
    assert np.all(terrain.navigation[touches]==1)
    assert ox+radius<pose.x<ox+terrain.shape[1]*res-radius
    assert oy+radius<pose.y<oy+terrain.shape[0]*res-radius


def test_safe_start_checks_cell_area_and_finite_map_boundary():
    from lunar_drl_exploration.scene import safe_start_mask
    navigation=np.ones((9,9),np.uint8);navigation[4,6]=2
    terrain=SimpleNamespace(navigation=navigation,resolution_m=1.)
    mask=safe_start_mask(terrain,1.6)
    assert not mask[4,4]  # blocked square is 1.5m away, not its 2m center
    assert not mask[0,0]  # outside finite map is not certified free
    assert mask[4,2]
    assert np.array_equal(safe_start_mask(terrain,0),navigation==1)


@pytest.mark.parametrize('family,mode',[('moon','outside'),('cave','inside')])
def test_new_start_guarantees_region_without_mutating_scene(family,mode):
    scene=Scene(23,family,40,platform=load_platform_config(),safe_start_radius_m=2.)
    terrain=TerrainGrid.from_scene(scene)
    before=(terrain.heights.tobytes(),terrain.navigation.tobytes(),terrain.terrain_id)
    old=scene.initial_pose(terrain,start_mode=mode)
    pose=scene.initial_pose(terrain,start_mode=mode,safe_start_radius_m=2.)
    assert_disk_free(terrain,pose,2.)
    assert pose==scene.initial_pose(terrain,start_mode=mode,safe_start_radius_m=2.)
    assert pose.yaw==old.yaw
    assert before==(terrain.heights.tobytes(),terrain.navigation.tobytes(),terrain.terrain_id)


def test_cave058_rejects_original_unsafe_start_and_preserves_terrain():
    from lunar_drl_exploration.course_two_task_scope import build_rehearsal
    from lunar_drl_exploration.cave_topology import manifest_hash
    kw=dict(cave_generator_version='cave_topology_v2',mother_graph_id='b6b307c25c6d897902ccca3278e9971e0ef73f4821ef40d55761934c364085b6',dataset_split='validation',cave_split_manifest_hash=manifest_hash())
    args=(2026104400058,93.,'inside',load_platform_config())
    old,old_pose,old_task,_=build_rehearsal(*args,**kw)
    terrain,pose,task,descriptor=build_rehearsal(*args,**kw,safe_start_radius_m=2.)
    assert (pose.x,pose.y)!=(old_pose.x,old_pose.y)
    assert_disk_free(terrain,pose,2.)
    assert not np.array_equal(old.heights,terrain.heights)
    assert descriptor['mother_graph_edges']==old.generator_descriptor['mother_graph_edges']
    assert terrain.terrain_id!=old.terrain_id
    assert task.task_id!=old_task.task_id
    assert descriptor['safe_start_region']['navigation_radius_m']==2.
    assert descriptor['safe_start_region']['generation_mode']=='reserved_start_region_v1'


def test_no_safe_start_fails_instead_of_carving_ground():
    scene=Scene(23,'cave',40,platform=load_platform_config());terrain=TerrainGrid.from_scene(scene)
    with pytest.raises(ValueError,match='safe start'):
        scene.initial_pose(terrain,start_mode='inside',safe_start_radius_m=1000.)


def test_safe_start_is_frozen_in_config_resume_and_dispatch():
    from lunar_drl_exploration.course_two_simple import fresh_scene
    c=TrainingConfig(course_two=True,course_two_variant='C3',course_two_task_scope='whole_cave_v1',course_two_sampling='scene_episode_v1',course_two_scene_tasks=('moon_outside','cave_inside'))
    safe=replace(c,safe_start_radius_m=2.)
    assert training_config_from_record(config_record(safe))==safe
    assert resume_semantics(safe)!=resume_semantics(c)
    assert 'safe_start_region' not in resume_semantics(c)
    for task in ('moon_outside','cave_inside'):
        assert fresh_scene(safe,task,0)['safe_start_radius_m']==2.
        assert 'safe_start_radius_m' not in fresh_scene(c,task,0)


@pytest.mark.parametrize('radius',[-1.,float('nan'),float('inf'),True])
def test_invalid_safe_start_radius_rejected(radius):
    with pytest.raises(ValueError,match='safe start'):
        replace(TrainingConfig(),safe_start_radius_m=radius)


def test_runtime_profile_keeps_navigation_settings_and_certifies_bound(tmp_path):
    import yaml
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    c=replace(TrainingConfig(),safe_start_radius_m=2.,artifact_root=str(tmp_path))
    env=RosExplorationEnv(c,0);env._safe_start_radius_m=2.
    source=Path(c.platform.capability_path).read_bytes()
    path=env._runtime_platform_config()
    assert yaml.safe_load(Path(path).read_text())==yaml.safe_load(source)
    assert Path(c.platform.capability_path).read_bytes()==source
    env._safe_start_radius_m=.8
    with pytest.raises(ValueError,match='cover.*patch'):
        env._runtime_platform_config()
    env._safe_start_radius_m=0.
    assert env._runtime_platform_config()==c.platform.capability_path


def test_preparation_binds_safe_region_and_rejects_resume_change(tmp_path):
    import json
    from lunar_drl_exploration.region_relation_prepare import prepare
    protocol=tmp_path/'protocol.json';protocol.write_text('{}')
    root=tmp_path/'run';prepare(root,protocol,safe_start_radius_m=2.)
    record=json.loads((root/'config.json').read_text())
    assert record['safe_start_radius_m']==2.
    for split in ('validation','test'):
        requests=json.loads((root/(split+'-requests.json')).read_text())['requests']
        assert all(s['safe_start_radius_m']==2. for r in requests for s in r['backup_specs'])
    with pytest.raises(ValueError,match='safe start radius changed'):
        prepare(root,protocol,safe_start_radius_m=5.)


def test_safe_scene_rejects_forged_certificate_before_ros():
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.contracts import Pose
    config=replace(TrainingConfig(),safe_start_radius_m=2.)
    terrain=TerrainGrid.from_heights(np.zeros((12,12),np.float32),.2,(0.,0.),config.platform)
    env=RosExplorationEnv(config,0)
    with pytest.raises(ValueError,match='safe start'):
        env.reset_scene(terrain,Pose(.1,.1,0.),None,
            descriptor={'safe_start_region':{'navigation_radius_m':2.}})


def test_legacy_config_cannot_enable_safe_mode_via_reset_spec():
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env=RosExplorationEnv(TrainingConfig(),0)
    with pytest.raises(ValueError,match='differs from frozen'):
        env.reset(23,'moon',40,safe_start_radius_m=2.)


def test_legacy_config_cannot_enable_safe_mode_via_scene_certificate(monkeypatch):
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.scene import safe_start_record
    from lunar_drl_exploration.contracts import Pose
    c=TrainingConfig();env=RosExplorationEnv(c,0)
    terrain=TerrainGrid.from_heights(np.zeros((60,60),np.float32),.2,(0.,0.),c.platform)
    pose=Pose(6.1,6.1,0.)
    def unexpected():raise AssertionError('must reject before starting ROS')
    monkeypatch.setattr(env,'_start_ros',unexpected)
    with pytest.raises(ValueError,match='configured safe start'):
        env.reset_scene(terrain,pose,None,descriptor=safe_start_record(terrain,pose,2.))


def test_runtime_rejects_profile_geometry_drift(tmp_path):
    import yaml
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    base=TrainingConfig();profile=yaml.safe_load(Path(base.platform.capability_path).read_text())
    path=tmp_path/'wheel.yaml';profile['capability']['footprint_xy_m']=[[1.,1.],[1.,-1.],[-1.,-1.],[-1.,1.]]
    path.write_text(yaml.safe_dump(profile))
    platform=replace(base.platform,capability_path=str(path))
    c=replace(base,platform=platform,safe_start_radius_m=2.,artifact_root=str(tmp_path/'run'))
    env=RosExplorationEnv(c,0);env._safe_start_radius_m=2.
    with pytest.raises(ValueError,match='capability.*frozen'):
        env._runtime_platform_config()


def test_reserved_cave_room_keeps_mother_topology_at_small_extent():
    from lunar_drl_exploration.cave_topology import mother_catalog,build_geometry,validate_geometry
    for mother in mother_catalog():
        geometry=build_geometry(mother,7,30.,start_room_radius_m=5.4)
        assert geometry['rooms'][0][2]>=5.4
        assert geometry['edges']==mother['edges']
        assert validate_geometry(geometry)


def test_runtime_rejects_top_level_margin_drift(tmp_path):
    import yaml
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    base=TrainingConfig();profile=yaml.safe_load(Path(base.platform.capability_path).read_text())
    path=tmp_path/'wheel.yaml';path.write_text(yaml.safe_dump(profile))
    platform=replace(base.platform,capability_path=str(path))
    c=replace(base,platform=platform,safe_start_radius_m=2.,artifact_root=str(tmp_path/'run'))
    env=RosExplorationEnv(c,0);env._safe_start_radius_m=2.
    env._runtime_platform_config()
    profile['start_blind_zone_margin_m']=3.
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError,match='profile.*frozen'):
        env._runtime_platform_config()


def test_native_profile_identity_enters_resume_semantics(tmp_path):
    import yaml
    base=TrainingConfig();profile=yaml.safe_load(Path(base.platform.capability_path).read_text())
    path=tmp_path/'wheel.yaml';path.write_text(yaml.safe_dump(profile))
    platform=replace(base.platform,capability_path=str(path))
    a=replace(base,platform=platform,safe_start_radius_m=2.)
    profile['start_blind_zone_margin_m']=3.;path.write_text(yaml.safe_dump(profile))
    b=replace(base,platform=platform,safe_start_radius_m=2.)
    assert resume_semantics(a)!=resume_semantics(b)
