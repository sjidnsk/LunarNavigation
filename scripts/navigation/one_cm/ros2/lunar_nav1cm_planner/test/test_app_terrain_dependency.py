"""app原生依赖halo：部分源更新必须等价于相同最终高程的完整重建。"""
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from nav1cm.terrain_native import NativeTerrain,load_wheel_capability


def test_partial_support_update_reclassifies_remote_neighbor_and_matches_fresh():
    lib=Path(get_package_prefix('lunar_nav1cm_planner'))/'lib/libnav1cm_terrain.so'
    cap=load_wheel_capability(Path(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml')
    def apply(terrain,data):return terrain.apply(data,resolution_m=.05,origin_xy_m=(0.,0.))
    def merge(cells,update):
        for x0,y0,x1,y1,state in update.regions:
            cells[max(0,y0):min(30,y1),max(0,x0):min(30,x1)]=state
    flat=np.zeros((30,30),np.float32)
    flat[12:19,12:19]=np.nan;flat[15,15]=0.
    patch=np.full_like(flat,np.nan);patch[13:18,17:19]=0.
    complete=flat.copy();complete[13:18,17:19]=0.
    incremental=np.full((30,30),-1);expected=np.full_like(incremental,-1)
    with NativeTerrain(lib,capability=cap) as terrain:
        merge(incremental,apply(terrain,flat))
        assert incremental[11,15]==-1
        merge(incremental,apply(terrain,patch))
    with NativeTerrain(lib,capability=cap) as fresh:merge(expected,apply(fresh,complete))
    assert expected[11,15]==0
    np.testing.assert_array_equal(incremental,expected)
