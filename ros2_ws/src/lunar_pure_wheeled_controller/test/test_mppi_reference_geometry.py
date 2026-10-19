"""Catch stale reference caches and changes to local projection/tie semantics."""
import math
import numpy as np
import pytest
from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
from lunar_pure_wheeled_controller.path_geometry import Polyline
from lunar_pure_wheeled_controller.tracking import TrackingPolicy


def follower(points):
    executor = MppiPathFollower(TrackingPolicy())
    executor.set_path(points)
    return executor


def test_repeated_projection_reuses_read_only_geometry_and_replaces_stale_path():
    executor = follower([(0, 0, 0), (2, 0, 0)])
    poses = np.array([[[.3, .1, 0.]]])
    executor._reference_errors(poses)
    assert hasattr(executor, '_reference_geometry'), 'immutable geometry must be cached'
    first = executor._reference_geometry
    executor._reference_errors(poses)
    assert executor._reference_geometry is first
    assert all(not array.flags.writeable for array in first[1:])
    executor.set_path([(0, 1, 0), (2, 1, 0)])
    distance, heading = executor._reference_errors(poses)
    np.testing.assert_allclose(distance, [[.81]], rtol=0, atol=1e-15)
    np.testing.assert_array_equal(heading, [[0.]])
    assert executor._reference_geometry is not first
    executor.clear()
    assert executor._reference_geometry is None


@pytest.mark.parametrize('progress,expected_distance,expected_heading', [
    (0., .05, 0.), (.9, .01, math.pi / 2), (1.1, .01, math.pi / 2),
    (2., .50, math.pi / 2), (3., .65, math.pi / 2),
])
def test_local_window_clipping_and_endpoint_fallback(progress, expected_distance, expected_heading):
    executor = follower([(0, 0, 0), (1, 0, 0), (1, 1, 0)])
    executor.progress_m = progress
    distance, heading = executor._reference_errors(np.array([[[.9, .2, 0.]]]))
    np.testing.assert_allclose(distance, [[expected_distance]], rtol=0, atol=1e-15)
    np.testing.assert_array_equal(heading, [[expected_heading]])


def test_ties_keep_first_segment_and_single_point_keeps_final_yaw():
    executor = follower([(0, 0, 0), (.2, 0, 0), (.2, .2, 0)])
    distance, heading = executor._reference_errors(np.array([[[.1, .1, 0.]]]))
    np.testing.assert_allclose(distance, [[.01]], rtol=0, atol=1e-15)
    np.testing.assert_array_equal(heading, [[0.]])
    executor.set_path([(1, 2, 1.2), (1, 2, -.7)])
    distance, heading = executor._reference_errors(np.array([[[1.3, 2.4, 0.]]]))
    np.testing.assert_allclose(distance, [[.25]], rtol=0, atol=1e-15)
    np.testing.assert_array_equal(heading, [[-.7]])


def test_temporary_corner_path_does_not_pollute_restored_reference():
    executor = follower([(0, 0, 0), (1, 0, 0), (1, 1, 0)])
    route = executor.path
    poses = np.array([[[1., .4, 0.]]])
    executor.progress_m = .9
    before = executor._reference_errors(poses)
    executor.path = Polyline([(0, 0, 0), (1, 0, 0)])
    distance, heading = executor._reference_errors(poses)
    np.testing.assert_allclose(distance, [[.16]], rtol=0, atol=1e-15)
    executor.path = route
    after = executor._reference_errors(poses)
    for old, new in zip(before, after):
        np.testing.assert_array_equal(old, new)
