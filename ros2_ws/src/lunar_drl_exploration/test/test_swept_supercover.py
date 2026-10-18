"""Closed-segment collision semantics must match native VisitSupercoverCells."""
from types import SimpleNamespace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import Pose
from lunar_drl_exploration.plant import KinematicPlant
from lunar_drl_exploration.config import load_platform_config


def plant(origin=(-91.5, -95.5), blocked=(438, 463)):
    navigation = np.ones((512, 512), np.uint8)
    navigation[blocked[1], blocked[0]] = 2
    terrain = SimpleNamespace(navigation=navigation, origin=origin, resolution_m=.2)
    return KinematicPlant(terrain, Pose(-3.6970886293685803, -2.702398024919407, 0), load_platform_config())


def test_original_collision_interval_does_not_touch_blocked_side_cell():
    p = plant()
    a = Pose(-3.6970886293685803, -2.702398024919407, 2.3946085120771174)
    b = Pose(-3.7044260420115545, -2.695603734985446, a.yaw)
    assert p._swept(a, b)
    assert p._swept(b, a)
    q = Pose(a.x+(b.x-a.x)*.37486573284375535, a.y+(b.y-a.y)*.37486573284375535, a.yaw)
    assert p._swept(a, q) and p._swept(q, b)


@pytest.mark.parametrize('transpose', [False, True])
@pytest.mark.parametrize('reverse', [False, True])
def test_distinct_grid_crossings_are_not_an_exact_corner(transpose, reverse):
    origin = (.13, -1.17)
    grid = np.ones((8, 8), np.uint8)
    blocked = (2, 3) if transpose else (3, 2)
    grid[blocked[1], blocked[0]] = 2
    coords = [(2.99, 2.99), (3.01, 3.02)]
    if transpose:
        coords = [(y, x) for x, y in coords]
    points = [Pose(origin[0]+x*.2, origin[1]+y*.2, 0) for x, y in coords]
    if reverse:
        points.reverse()
    p = KinematicPlant(SimpleNamespace(navigation=grid, origin=origin, resolution_m=.2), points[0], load_platform_config())
    assert p._swept(*points)


def test_exact_corner_and_boundary_contact_still_reject_blocked_cell():
    p = plant()
    assert not p._swept(Pose(-3.695, -2.705, 0), Pose(-3.705, -2.695, 0))
    assert not p._swept(Pose(-3.7, -2.85, 0), Pose(-3.7, -2.75, 0))
    assert not p._swept(Pose(-3.695, -2.75, 0), Pose(-3.705, -2.75, 0))
