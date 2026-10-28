"""完整车体支撑不能直接使用 intrinsic 中心状态。"""
import importlib
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot


def support(world,x=0.,y=0.,allow=False,backend=None):
    module=importlib.import_module('scripts.navigation.one_cm.nav1cm.follower_map')
    return module.center_support(world,x,y,body_length_m=1.182,body_width_m=.818,allow_unknown=allow,backend=backend)


def test_full_body_corner_and_closed_cell_support():
    world=WorldSnapshot(1,(CellRect(58,39,60,41),),(CellRect(-400,-400,400,400),))
    grid=support(world)
    assert grid.cells.shape==(500,500)
    assert grid.cells[250,250]==1
    assert not grid.eligible[250,250]


def test_unknown_policy_keeps_provenance_and_window_halo():
    world=WorldSnapshot(1,(),(CellRect(-400,-400,400,400),))
    strict=support(world);permissive=support(world,allow=True)
    # 窗口边缘仍有外部已知支撑，不能先截窗再膨胀。
    assert strict.eligible[250,490]
    empty=WorldSnapshot(2,(),())
    a=support(empty);b=support(empty,allow=True)
    assert np.all(a.cells==-1) and np.all(b.cells==-1)
    assert not a.eligible.any() and b.eligible.all()
    assert not b.cells.flags.writeable


def test_known_obstacle_never_becomes_eligible_under_unknown_permission():
    world=WorldSnapshot(1,(CellRect(-1,-1,1,1),),())
    grid=support(world,allow=True)
    assert not grid.eligible[250,250]
    assert grid.cells[250,250]==1


def test_native_and_reference_support_agree():
    import os
    from scripts.navigation.one_cm.nav1cm.field_native import NativeDistanceField
    backend=NativeDistanceField(os.environ['NAV1CM_DISTANCE_FIELD_LIB'])
    world=WorldSnapshot(1,(CellRect(100,-4,105,4),),(CellRect(-400,-400,400,400),))
    a=support(world);b=support(world,backend=backend)
    np.testing.assert_array_equal(a.cells,b.cells)
    np.testing.assert_array_equal(a.eligible,b.eligible)
