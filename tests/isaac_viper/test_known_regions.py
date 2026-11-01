"""固定known demo ROI合同测试；合成数组仅测试边界，不是场景/导航验收。"""
import hashlib
import importlib
from pathlib import Path
import numpy as np
import pytest
from tools.isaac_viper import viewport_paths as vp
from tools.isaac_viper import navigation_showcase as nav
from tools.isaac_viper.navigation_scene import SOURCE_SHA

FLAT=[-490.,-370.,-250.,-130.]
TERRAIN='/World/Environment/Terrain_0_1__250m/V02_TerrainMesh_0_1'


def module():
    assert importlib.util.find_spec('tools.isaac_viper.known_regions') is not None,'fixed known-region contract missing'
    return importlib.import_module('tools.isaac_viper.known_regions')


def test_only_fixed_flat_roi_is_permitted_and_c_is_rejected():
    m=module(); spec=m.region_spec('KNOWN_DEMO_FLAT')
    assert spec['bounds_xy_m']==FLAT and spec['terrain_path']==TERRAIN
    assert m.validate_region_map('KNOWN_DEMO_FLAT',FLAT,.2,(600,600))['region']=='KNOWN_DEMO_FLAT'
    assert m.region_spec('A')['bounds_xy_m']==[235.,335.,-315.,-215.]
    for region,bounds in [('C',FLAT),('UNKNOWN',FLAT),('KNOWN_DEMO_FLAT',[235.,335.,-315.,-215.]),('A',FLAT),('KNOWN_DEMO_FLAT',[-140.,100.,-420.,-180.])]:
        with pytest.raises(ValueError): m.validate_region_map(region,bounds,.2)
    spec['bounds_xy_m'][0]=0.
    assert m.region_spec('KNOWN_DEMO_FLAT')['bounds_xy_m']==FLAT


def test_flat_projector_accepts_exact_roi_and_rejects_label_swap(tmp_path):
    path=tmp_path/'map.npz'; z=np.full((600,600),7.,np.float32);z[0,0]=np.nan
    np.savez_compressed(path,elevation=z,bounds_xy_m=FLAT,resolution_m=.2)
    source=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),source_stage_sha256=SOURCE_SHA,region='KNOWN_DEMO_FLAT')
    projector=vp.TerrainProjector.from_source(source)
    assert projector.bounds==FLAT and np.isnan(projector.elevation[0,0])
    assert projector.segments([[-489.5,-249.5,0],[-489.3,-249.3,0]])[0][0][2]==7.12
    for region in ('A','C'):
        with pytest.raises(ValueError): vp.TerrainProjector.from_source(dict(source,region=region))


def test_native_index_order_uses_actual_grid_size_without_500_label():
    from tools.isaac_viper import native_policy
    assert hasattr(native_policy,'native_index_order'),'native v2 dynamic axis contract missing'
    assert native_policy.native_index_order((600,600))=='array[y,x] ascending world axes; cell centers origin+(index+0.5)*resolution; policy indices0..599 exactly source grid'
    assert native_policy.INDEX_ORDER.endswith('policy indices0..499 exactly source grid')


def test_map_only_receipt_cannot_send_goal_but_remains_publishable_known_map():
    assert hasattr(nav,'scene_has_endpoints'),'map-only phase health gate missing'
    scene=dict(region='KNOWN_DEMO_FLAT',bounds_xy_m=FLAT,resolution_m=.2,source_sha256=SOURCE_SHA,
        start_xy_m=None,goal_xy_m=None,goal_yaw_rad=None,phase='MAP_ONLY')
    assert not nav.scene_has_endpoints(scene)
    assert nav.scene_has_endpoints(dict(scene,phase='APPROXIMATE_FIXTURE',start_xy_m=[-489.,-249.],goal_xy_m=[-371.,-131.],goal_yaw_rad=0.))
