import math
import unittest
import numpy as np
from tools.isaac_viper import navigation_scene as ns
from tools.isaac_viper import navigation_showcase as nav
from tools.isaac_viper import viewport_paths as vp


class NavigationContracts(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(ns, "actual collision map module missing")
        self.assertIsNotNone(nav, "live navigation adapter missing")
        self.assertIsNotNone(vp, "viewport overlay module missing")
    def test_grid_map_reversed_axes_and_nan(self):
        grid = np.array([[1., np.nan, 3.], [4., 5., 6.]])
        data, dims = nav.grid_encoding(grid)
        np.testing.assert_allclose(data, [6, 5, 4, 3, np.nan, 1], equal_nan=True)
        self.assertEqual(dims, [('column_index', 2, 6), ('row_index', 3, 3)])

    def test_trace_uses_original_stamp_and_transform(self):
        trace = nav.ActualTrace(3)
        self.assertTrue(trace.append(10, [1, 2, 3], [0, 0, math.sqrt(.5), math.sqrt(.5)], [4, 5, 6]))
        np.testing.assert_allclose(trace.points[-1]['xyz'], [2, 6, 9], atol=1e-8)
        self.assertFalse(trace.append(10, [99, 99, 99], [0, 0, 0, 1], [0, 0, 0]))
        with self.assertRaises(ValueError):
            trace.append(9, [1, 2, 3], [0, 0, 0, 1], [0, 0, 0])
        self.assertEqual(trace.points[-1]['stamp_ns'], 10)

    def test_overlay_rejects_unbounded_or_nonfinite_paths(self):
        payload = dict(schema=vp.SCHEMA, frame='map', source='live_ros', sequence=1,
                       paths={'actual_trace':dict(stamp_ns=1, points=[[1, 2, 3]])})
        self.assertEqual(vp.validate_payload(payload)['sequence'], 1)
        payload['paths']['actual_trace']['points'] = [[1, 2, float('nan')]]
        with self.assertRaises(ValueError): vp.validate_payload(payload)
        payload['paths']['actual_trace']['points'] = [[1, 2, 3]]*10001
        with self.assertRaises(ValueError): vp.validate_payload(payload)

    def test_surface_keeps_rock_top_and_unknown(self):
        terrain = np.array([[[0,0,0],[1,0,0],[1,1,0]], [[0,0,0],[1,1,0],[0,1,0]]], float)
        rock = terrain.copy(); rock[:,:,2] = .7
        grid, relief = ns.rasterize_triangles(np.concatenate([terrain,rock]), [0, 2, 0, 1], .2)
        self.assertAlmostEqual(float(grid[2,2]), .7, places=5)
        self.assertAlmostEqual(float(relief[2,2]), .7, places=5)
        self.assertTrue(np.isnan(grid[2,7]))
        self.assertTrue(np.isnan(grid[2,5]), "zero-area boundary is not measured surface")

    def test_route_refuses_unknown_and_rock_footprint(self):
        grid = np.zeros((60,60)); relief = np.zeros_like(grid)
        grid[20:40,20:40] = np.nan
        start, goal = ns.select_route(grid, relief, [235,247,-315,-303], .2)
        self.assertTrue(3 <= math.dist(start, goal) <= 5)
        self.assertTrue(235 <= start[0] <= 247 and -315 <= start[1] <= -303)
        with self.assertRaises(ValueError): ns.select_route(np.full((40,40),np.nan), np.zeros((40,40)), [235,243,-315,-307], .2)

    def test_planning_requires_matching_live_reference(self):
        ref = dict(session_id='abc',segment_revision=2,traversability_revision=3,state=0,points=[[1,2,3]])
        diag = dict(session_id='abc',segment_revision='2',fine_traversability_revision='3',cycle_result='PLAN_FOUND',reason_code='PLAN_FOUND')
        self.assertTrue(nav.planning_evidence(diag,ref)['has_reference'])
        ref['traversability_revision'] = 4
        self.assertFalse(nav.planning_evidence(diag,ref)['has_reference'])

if __name__ == '__main__': unittest.main()

