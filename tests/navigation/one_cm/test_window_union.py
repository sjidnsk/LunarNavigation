"""窗口平移不能把已知自由并集的内部矩形接缝变为未知。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, CellState, VehicleState, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.maps import extract_window, build_distance_field


@pytest.mark.parametrize('axis', ['x', 'y'])
@pytest.mark.parametrize('phase', [-.0002747559, .0032901408, .005, 0.])
def test_fractional_window_preserves_free_union_across_partition(axis, phase):
    # 两个相接矩形的并集是覆盖整个窗口的已知平地，接缝无物理意义。
    regions = ((CellRect(-400, -400, 25, 400), CellRect(25, -400, 400, 400))
               if axis == 'x' else
               (CellRect(-400, -400, 400, 25), CellRect(-400, 25, 400, 400)))
    pose = VehicleState(phase, phase, 1.6524611, 0., 0.)
    grid = extract_window(WorldSnapshot(1, (), regions), pose, False)
    assert grid.origin_xy == (phase - 2.5, phase - 2.5)
    assert grid.cells.shape == (500, 500)
    assert np.all(grid.cells == CellState.FREE)
    assert build_distance_field(grid).sample(pose.x, pose.y)[0] > 2.48


@pytest.mark.parametrize('allow_unknown', [False, True])
def test_union_resampling_preserves_real_gap_and_blocked_overlap(allow_unknown):
    world = WorldSnapshot(1, (CellRect(-1, -1, 1, 1),),
                          (CellRect(-400, -400, 400, 25), CellRect(-400, 26, 400, 400)))
    grid = extract_window(world, VehicleState(.005, .003, 0., 0., 0.), allow_unknown)
    assert grid.cell_at_world(0., 0.) == CellState.BLOCKED
    assert grid.cell_at_world(0., .249) == CellState.UNKNOWN
    assert grid.cell_at_world(0., .259) == CellState.UNKNOWN
    assert grid.cell_at_world(0., .275) == CellState.FREE


def test_fractional_union_keeps_interior_unknown_hole():
    # 四块自由区域围住 1 cm² 未知孔；相邻并集接缝不能掩盖孔本身。
    regions = (CellRect(-400, -400, 400, 0), CellRect(-400, 1, 400, 400),
               CellRect(-400, 0, 0, 1), CellRect(1, 0, 400, 1))
    grid = extract_window(WorldSnapshot(2, (), regions), VehicleState(-.005, -.003, 0., 0., 0.), False)
    assert grid.cell_at_world(.002, .002) == CellState.UNKNOWN
    assert grid.cell_at_world(.012, .002) == CellState.UNKNOWN
    assert grid.cell_at_world(-.012, .002) == CellState.FREE
    assert grid.cell_at_world(.022, .002) == CellState.FREE
    assert np.count_nonzero(grid.cells == CellState.UNKNOWN) == 4
