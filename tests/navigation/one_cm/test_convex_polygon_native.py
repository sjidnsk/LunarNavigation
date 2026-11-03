from test_p3_terrain_native import compiled, _api
import pytest


def area(polygons):
    return sum(abs(sum(x * polygon[(i + 1) % len(polygon)][1] - y * polygon[(i + 1) % len(polygon)][0] for i, (x, y) in enumerate(polygon))) / 2 for polygon in polygons)


def test_polygon_analytic_cases(compiled):
    square = ((0., 0.), (2., 0.), (2., 2.), (0., 2.))
    overlap = ((1., 0.), (3., 0.), (3., 2.), (1., 2.))
    outer = ((0., 0.), (4., 0.), (4., 4.), (0., 4.))
    inner = ((1., 1.), (3., 1.), (3., 3.), (1., 3.))
    touching = ((2., 0.), (4., 0.), (4., 2.), (2., 2.))
    with _api().NativeTerrain(compiled[0]) as terrain:
        assert area(terrain.polygon_intersection(square, overlap)) == pytest.approx(2.)
        assert area(terrain.polygon_difference(square, overlap)) == pytest.approx(2.)
        assert area(terrain.polygon_intersection(outer, inner)) == pytest.approx(4.)
        assert area(terrain.polygon_difference(outer, inner)) == pytest.approx(12.)
        diamond_subject = ((0., 0.), (2., 0.), (2., 2.), (0., 2.))
        diamond_clipper = ((2., 0.), (4., 1.), (2., 2.), (0., 1.))
        assert area(terrain.polygon_intersection(diamond_subject, diamond_clipper)) == pytest.approx(2.)
        assert area(terrain.polygon_difference(diamond_subject, diamond_clipper)) == pytest.approx(2.)
        assert terrain.polygon_intersection(square, touching) == ()


def test_polygon_validation_and_lifetime(compiled):
    square = ((0., 0.), (2., 0.), (2., 2.), (0., 2.))
    strip = ((1.99999, 0.), (3., 0.), (3., 2.), (1.99999, 2.))
    diamond = ((1., -1.), (3., 1.), (1., 3.), (-1., 1.))
    with _api().NativeTerrain(compiled[0]) as terrain:
        saved = terrain.polygon_intersection(square, strip)
        assert area(saved) == pytest.approx(.00002)
        assert area(terrain.polygon_intersection(square, diamond)) == pytest.approx(4.)
        assert area(terrain.polygon_intersection(tuple(reversed(square)), strip)) == pytest.approx(.00002)
        assert area(terrain.polygon_difference(square, strip, area_epsilon=5.)) == 0.
        for invalid in (((0., 0.), (1., 0.), (2., 0.)),
                        ((0., 0.), (2., 0.), (1., .5), (2., 2.), (0., 2.)),
                        ((0., 0.), (float('nan'), 0.), (0., 1.))):
            with pytest.raises(ValueError):
                terrain.polygon_intersection(invalid, square)
        with pytest.raises(ValueError):
            terrain.polygon_difference(square, strip, area_epsilon=-1.)
        assert area(saved) == pytest.approx(.00002)
    assert isinstance(saved, tuple) and isinstance(saved[0], tuple)


def test_geometry_preserves_measurement_bytes(compiled):
    import ctypes
    import numpy as np
    from scripts.navigation.one_cm.nav1cm.terrain_native import MEASUREMENT_DTYPE
    square = ((0., 0.), (2., 0.), (2., 2.), (0., 2.))
    with _api().NativeTerrain(compiled[0]) as terrain:
        terrain.apply(np.zeros((3, 3), dtype=np.float32), resolution_m=.1, origin_xy_m=(0., 0.))
        def snapshot():
            ptr, count = ctypes.c_void_p(), ctypes.c_size_t()
            assert terrain._measurements(terrain._handle, ctypes.byref(ptr), ctypes.byref(count)) == 0
            return ctypes.string_at(ptr, count.value * MEASUREMENT_DTYPE.itemsize)
        before = snapshot()
        terrain.polygon_intersection(square, square)
        with pytest.raises(ValueError):
            terrain.polygon_difference(square, square, area_epsilon=-1.)
        assert snapshot() == before


def test_polygon_combined_arithmetic_overflow_is_rejected(compiled):
    a = ((1e308, 0.), (1.0000000000000002e308, 0.),
         (1.0000000000000002e308, 1e-292), (1e308, 1e-292))
    b = ((-1e308, 0.), (-9.999999999999998e307, 0.),
         (-9.999999999999998e307, 1e-292), (-1e308, 1e-292))
    with _api().NativeTerrain(compiled[0]) as terrain:
        with pytest.raises(ValueError):
            terrain.polygon_intersection(a, b)
        with pytest.raises(ValueError):
            terrain.polygon_difference(a, b)

def test_clipped_result_can_be_used_as_subject(compiled):
    square = ((0., 0.), (2., 0.), (2., 2.), (0., 2.))
    triangle = ((0., 0.), (3., 1.), (1., 3.))
    with _api().NativeTerrain(compiled[0]) as terrain:
        part, = terrain.polygon_intersection(square, triangle)
        assert area(terrain.polygon_intersection(part, square)) == pytest.approx(area((part,)))
