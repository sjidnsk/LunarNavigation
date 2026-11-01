"""距离分组和厘米矩形面积统计的独立几何校验，不依赖ROS。"""
import numpy as np
from types import SimpleNamespace as R
from scripts.simulation.p3_map_audit.summarize import grouped, native_grid_counts


def test_ring_edges_are_left_closed_and_distance_uses_vehicle_pose():
    counts=grouped(np.array([1.,3.,6.,11.]),np.zeros(4),np.array([1,1,1,0],bool),.2,[1.,0.])
    assert [v['count'] for v in counts[:3]]==[1,1,1]
    assert sum(v['count'] for v in counts)==3
    assert np.isclose(sum(v['area_m2'] for v in counts),.12)


def test_centimeter_rectangles_match_explicit_cell_enumeration_across_ring():
    rect=R(x0=195,x1=205,y0=-3,y1=3)
    actual=native_grid_counts([rect],[0.,0.],[-3.,-3.,3.,3.])
    x,y=np.meshgrid((np.arange(195,205)+.5)*.01,(np.arange(-3,3)+.5)*.01)
    expected=grouped(x.ravel(),y.ravel(),np.ones(x.size,bool),.01,[0.,0.])
    assert actual==expected
    assert sum(v['count'] for v in actual)==60


def test_centimeter_rectangles_are_clipped_to_shared_source_footprint():
    actual=native_grid_counts([R(x0=-2,x1=5,y0=-2,y1=5)],[0.,0.],[0.,0.,.03,.02])
    assert sum(v['count'] for v in actual)==6
    assert np.isclose(sum(v['area_m2'] for v in actual),.0006)
