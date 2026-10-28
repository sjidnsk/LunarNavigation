"""轻场景纯 CPU 与可由 SDK unittest 直接运行的实际 USD 回归。"""
import json
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.isaac_viper import build_showcase as showcase


class GeometryTests(unittest.TestCase):
    def test_piecewise_triangle_is_not_bilinear(self):
        z = np.array([[0., 0.], [0., 4.]])
        self.assertAlmostEqual(float(showcase.triangle_heights(z, [0, 1, 0, 1], .75, .25)), 1.)
        self.assertAlmostEqual(float(showcase.triangle_heights(z, [0, 1, 0, 1], .25, .75)), 1.)
        self.assertAlmostEqual(float(showcase.triangle_heights(z, [0, 1, 0, 1], 1, 1)), 4.)
        with self.assertRaises(ValueError):
            showcase.triangle_heights(z, [0, 1, 0, 1], 1.001, 0)

    def test_sampling_patch_and_uniform_mesh(self):
        base = np.zeros((5, 5))
        patch = {'bounds': [1, 3, 1, 3], 'step': .5, 'height': np.full((5, 5), 2.)}
        sampler = showcase.HeightSource(base, [0, 4, 0, 4], [patch])
        self.assertEqual(float(sampler.sample(0, 0)), 0)
        self.assertEqual(float(sampler.sample(1.25, 2.5)), 2)
        grid = showcase.uniform_grid(sampler, [0, 4, 0, 4], 1.)
        points, indices = showcase.grid_mesh(grid, [0, 4, 0, 4])
        self.assertEqual(points.shape, (25, 3))
        self.assertEqual(indices.shape, (32, 3))
        self.assertTrue(np.all(np.cross(points[indices[:, 1]]-points[indices[:, 0]],
                                       points[indices[:, 2]]-points[indices[:, 0]])[:, 2] > 0))
        np.testing.assert_equal(points[[0, -1], :2], [[0, 0], [4, 4]])
        with self.assertRaises(ValueError):
            showcase.uniform_grid(sampler, [0, 4, 0, 4], 1.3)

    def test_height_errors_are_full_source_samples(self):
        original = np.array([[0., 0., 0.], [0., 2., 0.], [0., 0., 0.]])
        source = showcase.HeightSource(original, [0, 2, 0, 2], [])
        coarse = showcase.uniform_grid(source, [0, 2, 0, 2], 2.)
        result = showcase.height_error_report(source, coarse, [0, 2, 0, 2])
        self.assertEqual(result['global']['samples'], 9)
        self.assertEqual(result['global']['max_abs_m'], 2)
        self.assertAlmostEqual(result['global']['rmse_m'], 2/3)

    def test_actual_triangle_support_differs_from_nearest_vertex(self):
        grid = np.array([[0.,0.],[0.,4.]])
        points, faces = showcase.grid_mesh(grid, [0,1,0,1])
        np.testing.assert_allclose(showcase.triangle_support(points,faces,[[.75,.25],[.25,.75]]), [1,1])
        with self.assertRaises(ValueError):
            showcase.triangle_support(points,faces,[[2,2]])

    def test_proxy_preserves_world_envelope(self):
        points, faces = showcase.rock_proxy([-2, 1, 3], [4, 5, 7])
        self.assertEqual(faces.shape, (80, 3))
        np.testing.assert_array_equal(points.min(axis=0), [-2, 1, 3])
        np.testing.assert_array_equal(points.max(axis=0), [4, 5, 7])
        for low, high in (([0, 0, 0], [0, 1, 1]), ([0, 0, 0], [1, float('nan'), 1])):
            with self.assertRaises(ValueError):
                showcase.rock_proxy(low, high)

    def test_region_input_is_authority_and_has_no_actor_truth(self):
        path = Path(__file__).parents[2]/'docs/simulation/isaac-polar-v02-midterm-regions.json'
        cfg = showcase.load_regions(path)
        self.assertEqual(cfg['regions'][2]['spawn_candidate_xy_m'], [-55, -320])
        cfg['regions'][0]['bounds_xy_m'][0] = -501
        with self.assertRaises(ValueError):
            showcase.validate_regions(cfg)

    def test_view_file_accepts_only_known_camera_and_does_not_repeat(self):
        from tools.isaac_viper.run_scene import ShowcaseViewChannel
        with tempfile.TemporaryDirectory() as tmp:
            request = Path(tmp)/'view-request.json'
            selected = []
            channel = ShowcaseViewChannel(request, {'vehicle':'/Vehicle','A':'/A'}, selected.append)
            request.write_text(json.dumps({'view':'A'}), encoding='utf-8')
            self.assertEqual(channel.poll(), 'A')
            self.assertIsNone(channel.poll())
            self.assertEqual(selected, ['/A'])
            request.write_text(json.dumps({'view':'/World/Arbitrary'}), encoding='utf-8')
            with self.assertRaises(ValueError):
                channel.poll()
            self.assertEqual(selected, ['/A'])
            request.write_text(json.dumps({'view':{}}), encoding='utf-8')
            with self.assertRaises(ValueError):
                channel.poll()

    def test_quality_and_existing_output_refused_before_sdk_import(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileExistsError):
                showcase.build_showcase('missing', 'missing', 'missing', 'missing', tmp)
        with self.assertRaises(ValueError):
            showcase.build_showcase('missing', 'missing', 'missing', 'missing', 'unused', quality='fast')


@unittest.skipUnless(os.environ.get('VIPER_SHOWCASE_STAGE'), 'requires generated actual PXR showcase stage')
class ActualShowcaseTests(unittest.TestCase):
    def test_actual_composition(self):
        result = showcase.validate_showcase(os.environ['VIPER_SHOWCASE_STAGE'],
                                            Path(__file__).parents[2]/'config/isaac_viper.yaml')
        self.assertTrue(result['passed'], result['errors'])

    def test_metadata_old_collider_and_annotation_mutations(self):
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        source = os.environ['VIPER_SHOWCASE_STAGE']
        for kind, expected in (('metadata','engineering_metadata'), ('collider','old environment'), ('annotation','annotation geometry')):
            stage = Usd.Stage.Open(source)
            stage.SetEditTarget(stage.GetSessionLayer())
            original_data = dict(stage.GetRootLayer().customLayerData)
            if kind == 'metadata':
                data = dict(stage.GetRootLayer().customLayerData)
                data.pop('engineering_metadata_json')
                stage.GetRootLayer().customLayerData = data
            elif kind == 'collider':
                stage.OverridePrim('/World/Environment').SetActive(True)
                cube = UsdGeom.Cube.Define(stage, '/World/Environment/LeakedCollider')
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            else:
                UsdGeom.Cube.Define(stage, '/World/ShowcaseViews/LeakedBoundary')
            result = showcase.validate_showcase(stage, Path(__file__).parents[2]/'config/isaac_viper.yaml')
            stage.GetRootLayer().customLayerData = original_data
            self.assertFalse(result['passed'], (kind, result))
            self.assertIn(expected, ' '.join(result['errors']))


if __name__ == '__main__':
    unittest.main()
