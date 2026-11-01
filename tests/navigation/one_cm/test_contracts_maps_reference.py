import math

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm import (
    CellRect,
    CellState,
    GridSnapshot,
    ProgressToken,
    Route,
    VehicleState,
    WorldSnapshot,
    build_distance_field,
    extract_reference,
    extract_window,
)


def test_snapshots_own_read_only_numeric_buffers():
    cells = np.zeros((3, 3), dtype=np.int8)
    points = np.array([[0.0, 0.0], [1.0, 0.0]])
    grid = GridSnapshot(4, (0.0, 0.0), 0.01, cells, False)
    route = Route(7, 4, points, (1.0, 0.0))
    cells[1, 1] = 1
    points[0, 0] = 99.0
    assert grid.cells[1, 1] == CellState.FREE
    assert route.points[0, 0] == 0.0
    with pytest.raises(ValueError):
        grid.cells[1, 1] = 1
    with pytest.raises(ValueError):
        route.points[0, 0] = 5.0
    with pytest.raises(ValueError):
        grid.cells.setflags(write=True)


@pytest.mark.parametrize("bad", [
    np.array([[0.9]], dtype=np.float64),
    np.array([[256]], dtype=np.int16),
    np.array([[True]], dtype=np.bool_),
])
def test_grid_rejects_noninteger_or_out_of_range_raw_cells_before_conversion(bad):
    with pytest.raises(ValueError, match="cells"):
        GridSnapshot(4, (0.0, 0.0), 0.01, bad, False)


def test_window_preserves_raw_unknown_under_both_policies_and_outside_is_unknown():
    world = WorldSnapshot(3, (), (CellRect(-60, -42, 60, 42),))
    pose = VehicleState(0.0, 0.0, 0.0, 0.0, 0.0)
    strict = extract_window(world, pose, False)
    permissive = extract_window(world, pose, True)
    assert strict.cells.shape == (500, 500)
    assert np.array_equal(strict.cells, permissive.cells)
    assert strict.cells[250, 250] == CellState.FREE
    assert strict.cells[0, 0] == CellState.UNKNOWN
    assert permissive.cells[0, 0] == CellState.UNKNOWN
    assert strict.cell_at_world(3.0, 0.0) == CellState.UNKNOWN


def test_window_marks_any_overlap_with_blocked_centimeter_cell():
    world = WorldSnapshot(
        9,
        (CellRect(1, 0, 2, 1),),
        (CellRect(-250, -250, 250, 250),),
    )
    grid = extract_window(world, VehicleState(0.005, 0.0, 0.0, 0.0, 0.0), False)
    assert np.count_nonzero(grid.cells == CellState.BLOCKED) >= 2
    assert grid.cell_at_world(0.015, 0.005) == CellState.BLOCKED


@pytest.mark.parametrize("rect", [
    CellRect(-310, -10, -290, 10),
    CellRect(290, -10, 310, 10),
    CellRect(-10, -310, 10, -290),
    CellRect(-10, 290, 10, 310),
])
def test_wholly_outside_blocked_rectangles_never_wrap_into_window(rect):
    world = WorldSnapshot(1, (rect,), (CellRect(-1000, -1000, 1000, 1000),))
    grid = extract_window(world, VehicleState(0, 0, 0, 0, 0), False)
    assert np.count_nonzero(grid.cells == CellState.BLOCKED) == 0
    assert np.all(grid.cells == CellState.FREE)


@pytest.mark.parametrize("rect", [
    CellRect(-260, -10, -240, 10),
    CellRect(240, -10, 260, 10),
    CellRect(-10, -260, 10, -240),
    CellRect(-10, 240, 10, 260),
])
def test_partially_intersecting_rectangles_are_clipped_to_window(rect):
    world = WorldSnapshot(1, (rect,), (CellRect(-1000, -1000, 1000, 1000),))
    grid = extract_window(world, VehicleState(0, 0, 0, 0, 0), False)
    assert np.count_nonzero(grid.cells == CellState.BLOCKED) == 200
    assert np.count_nonzero(grid.cells == CellState.FREE) == 500 * 500 - 200


@pytest.mark.parametrize("rect", [
    CellRect(-310, -10, -290, 10),
    CellRect(290, -10, 310, 10),
    CellRect(-10, -310, 10, -290),
    CellRect(-10, 290, 10, 310),
])
def test_wholly_outside_known_free_rectangles_leave_unknown_untouched(rect):
    world = WorldSnapshot(1, (), (rect,))
    grid = extract_window(world, VehicleState(0, 0, 0, 0, 0), False)
    assert np.all(grid.cells == CellState.UNKNOWN)


def test_distance_field_is_conservative_to_cell_area_and_window_edge():
    cells = np.zeros((5, 5), dtype=np.int8)
    cells[2, 2] = CellState.BLOCKED
    field = build_distance_field(GridSnapshot(2, (0.0, 0.0), 0.01, cells, True))
    assert field.distance_m[2, 2] == 0.0
    assert 0.0 <= field.distance_m[2, 3] <= 0.005 + 1e-12
    assert field.distance_m[0, 0] <= 0.005 + 1e-12
    assert field.gradient_x.shape == (5, 5)
    assert not field.distance_m.flags.writeable


def test_distance_field_arbitrary_position_query_remains_a_lower_bound():
    cells = np.zeros((5, 5), dtype=np.int8)
    cells[2, 2] = CellState.BLOCKED
    field = build_distance_field(GridSnapshot(2, (0.0, 0.0), 0.01, cells, True))
    # 障碍右边界为 x=0.03；该点真实净空仅 0.000001 m。
    distance, _, _ = field.sample(0.030001, 0.025)
    assert 0.0 <= distance <= 0.000001 + 1e-12


def test_unknown_is_obstacle_only_for_strict_distance_field():
    cells = np.zeros((5, 5), dtype=np.int8)
    cells[2, 2] = CellState.UNKNOWN
    strict = build_distance_field(GridSnapshot(2, (0.0, 0.0), 0.01, cells, False))
    permissive = build_distance_field(GridSnapshot(2, (0.0, 0.0), 0.01, cells, True))
    assert strict.distance_m[2, 2] == 0.0
    assert permissive.distance_m[2, 2] > 0.0


def test_same_route_progress_cannot_jump_across_a_crossing():
    route = Route(7, 2, np.array([[-1.0, 0.0], [1.0, 0.0], [0.0, -1.0], [0.0, 1.0]]), (0.0, 1.0))
    grid = GridSnapshot(2, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), True)
    _, token = extract_reference(route, VehicleState(0.01, 0.01, 0.0, 0.0, 0.0), grid, ProgressToken(7, 0.9))
    assert 0.85 <= token.s <= 1.2


def test_same_route_progress_cannot_jump_to_nearby_return_leg():
    route = Route(7, 2, np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.2], [0.0, 0.2]]), (0.0, 0.2))
    grid = GridSnapshot(2, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), True)
    _, token = extract_reference(route, VehicleState(0.1, 0.19, 0.0, 0.0, 0.0), grid, ProgressToken(7, 0.1))
    assert token.s <= 0.35


def test_new_route_version_reprojects_and_reference_stops_at_first_exit():
    grid = GridSnapshot(2, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), True)
    route = Route(8, 2, np.array([[0.0, 0.0], [3.0, 0.0], [0.0, 0.0]]), (0.0, 0.0))
    reference, token = extract_reference(route, VehicleState(0.1, 0.0, 0.0, 0.0, 0.0), grid, ProgressToken(7, 99.0))
    assert token.route_version == 8
    assert token.s == pytest.approx(0.1)
    assert reference.points[-1, 0] == pytest.approx(2.5)
    assert reference.s[-1] == pytest.approx(2.5)
    assert np.all(np.diff(reference.s) > 0.0)
    assert len(reference.points) == 2


def test_reference_on_new_grid_retains_route_version_and_uses_current_grid_epoch():
    route = Route(1, 10, np.array([[0.0, 0.0], [1.0, 0.0]]), (1.0, 0.0))
    grid = GridSnapshot(11, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), True)
    reference, token = extract_reference(route, VehicleState(0.1, 0.0, 0.0, 0.0, 0.0), grid)
    assert reference.route_version == token.route_version == 1
    assert reference.map_epoch == 11
    assert token.s == pytest.approx(0.1)
