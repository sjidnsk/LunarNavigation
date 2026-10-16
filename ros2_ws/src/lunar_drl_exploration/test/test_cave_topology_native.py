"""Real native wheel classification; no Python substitute for native derive."""
import pytest
from lunar_drl_exploration.cave_topology import VERSION,select_mother,manifest_hash
from lunar_drl_exploration.config import load_platform_config,TrainingConfig
from lunar_drl_exploration.scene import Scene,TerrainGrid
from lunar_drl_exploration.course_two_task_scope import build_rehearsal
from lunar_drl_exploration.geometry import polygon_mask


@pytest.mark.parametrize('split',['train','validation','test'])
def test_native_floor_contraction_explicit_anchor_and_whole_task(split):
    p=load_platform_config();mother=select_mother(split,2)
    kwargs=dict(cave_generator_version=VERSION,mother_graph_id=mother['mother_graph_id'],dataset_split=split)
    a=Scene(123,'cave',30.,platform=p,**kwargs)
    t=TerrainGrid.from_scene(a)
    q=t.generator_descriptor['native_wheeled_summary']
    assert q['native_qualification']=='passed'
    assert q['grid_topology_validation']=='room_corridor_contraction_passed'
    assert q['native_floor_free_cells']==q['native_reachable_floor_cells']
    terrain,pose,task,descriptor=build_rehearsal(123,30.,'inside',p,**kwargs)
    assert descriptor['mother_graph_id']==mother['mother_graph_id']
    assert descriptor['cave_split_manifest_hash']==manifest_hash()
    roi=polygon_mask(terrain.shape,terrain.origin,.2,task.polygon)
    assert not ((terrain.heights==0)&~roi).any()
    x,y=terrain.world_to_cell(pose.x,pose.y)
    assert terrain.navigation[y,x]==1 and terrain.heights[y,x]==0
    assert descriptor['reference_reachable_task_cells']>0


def test_native_nonqualified_resolution_and_outside_start_are_rejected():
    p=load_platform_config()
    with pytest.raises(ValueError,match='0.2'):
        TerrainGrid.from_scene(Scene(12,'cave',30.,.5,p,cave_generator_version=VERSION))
    with pytest.raises(ValueError,match='inside'):
        build_rehearsal(12,30.,'outside',p,cave_generator_version=VERSION)
