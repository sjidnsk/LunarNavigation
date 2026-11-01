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

    def test_default_quality_recovers_half_meter_detail(self):
        # 若默认采样退回 1 m，同一 4 m 平面只会输出 25 点/32 面。
        import yaml
        cfg = yaml.safe_load((Path(__file__).parents[2]/'config/isaac_viper_showcase.yaml').read_text(encoding='utf-8'))
        source = showcase.HeightSource(np.zeros((5, 5)), [0, 4, 0, 4], [])
        grid = showcase.uniform_grid(source, [0, 4, 0, 4], cfg['terrain_spacing_m'])
        points, faces = showcase.grid_mesh(grid, [0, 4, 0, 4])
        self.assertEqual(points.shape, (81, 3))
        self.assertEqual(faces.shape, (128, 3))
        spec = showcase.quality_spec(cfg)
        self.assertEqual((spec['terrain_points'], spec['terrain_triangles']), (4004001, 8000000))
        self.assertEqual((spec['rock_points'], spec['rock_triangles']), (162, 320))
        self.assertEqual(cfg['geometry_version'], 'uniform0p5m-rock320-v2')

    def test_quality_identity_rejects_inconsistent_or_arbitrary_counts(self):
        old = {'schema':'lunar.viper-showcase-quality.v1', 'quality':'light',
               'geometry_version':'uniform1m-rock80-v1', 'terrain_spacing_m':1., 'rock_max_triangles':128}
        spec = showcase.quality_spec(old)
        self.assertEqual((spec['terrain_points'], spec['terrain_triangles']), (1002001, 2000000))
        self.assertEqual((spec['rock_points'], spec['rock_triangles']), (42, 80))
        self.assertFalse(spec['smooth_terrain'])
        for patch in ({'geometry_version':'unknown'}, {'terrain_spacing_m':.5},
                      {'rock_max_triangles':512}, {'rock_triangles_each':320}, {'quality':'original'}):
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                showcase.quality_spec(dict(old, **patch))
        original = dict(old, quality='original', geometry_version='original-source-v1')
        self.assertIsNone(showcase.quality_spec(original)['terrain_points'])

    def test_gradient_normals_match_hand_computed_plane_including_edges(self):
        # z = 2*x - 3*y + 7，因此向上单位法线为 (-2, 3, 1)/sqrt(14)。
        x, y = np.meshgrid([-1., 0., 1., 2.], [4., 6., 8.])
        grid = 2*x-3*y+7
        normals = showcase.grid_normals(grid, [-1, 2, 4, 8])
        self.assertEqual(normals.shape, (12, 3))
        np.testing.assert_allclose(normals, np.tile([-2/14**.5, 3/14**.5, 1/14**.5], (12, 1)), atol=1e-7)
        np.testing.assert_allclose(showcase.grid_normals(np.zeros((2, 2)), [0, 1, 0, 1]),
                                   np.tile([0, 0, 1], (4, 1)))
        for bad in (np.zeros((1, 2)), np.array([[0., np.nan], [0., 0.]])):
            with self.assertRaises(ValueError):
                showcase.grid_normals(bad, [0, 1, 0, 1])

    def test_320_triangle_proxy_preserves_envelope_and_convex_closed_surface(self):
        points, faces = showcase.rock_proxy([-2, 1, 3], [4, 5, 7], triangles=320)
        self.assertEqual((len(points), len(faces)), (162, 320))
        np.testing.assert_array_equal(points.min(axis=0), [-2, 1, 3])
        np.testing.assert_array_equal(points.max(axis=0), [4, 5, 7])
        edges = {}
        for a, b, c in faces:
            for edge in ((a, b), (b, c), (c, a)):
                key = tuple(sorted(edge))
                edges[key] = edges.get(key, 0)+1
            normal = np.cross(points[b]-points[a], points[c]-points[a])
            self.assertTrue(np.all((points-points[a])@normal <= 1e-9))
        self.assertEqual(set(edges.values()), {2})
        for unsupported in (0, 128, 512, 1280):
            with self.assertRaises(ValueError):
                showcase.rock_proxy([0, 0, 0], [1, 1, 1], triangles=unsupported)

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

    def test_quality_metadata_cannot_relax_actual_topology(self):
        from pxr import Usd
        source = os.environ['VIPER_SHOWCASE_STAGE']
        for kind in ('unknown_version', 'spacing', 'rock_faces', 'version_mismatch', 'masquerade_original'):
            with self.subTest(kind=kind):
                stage = Usd.Stage.Open(source)
                original = dict(stage.GetRootLayer().customLayerData)
                data = json.loads(original['engineering_metadata_json'])
                info = data['showcase']
                if info['quality'] != 'light':
                    self.skipTest('light geometry identity mutations require a light stage')
                if kind == 'unknown_version':
                    info['geometry_version'] = info['quality_settings']['geometry_version'] = 'arbitrary-counts-v9'
                elif kind == 'spacing':
                    info['quality_settings']['terrain_spacing_m'] = .25
                elif kind == 'rock_faces':
                    info['rock_triangles_each'] = 1
                elif kind == 'masquerade_original':
                    info['quality'] = info['quality_settings']['quality'] = 'original'
                    info['geometry_version'] = info['quality_settings']['geometry_version'] = 'original-source-v1'
                    info['rock_triangles_each'], info['rock_count'] = None, 0
                    info['terrain_mesh_path'] = '/World/Environment/DetailTerrain_Rover_plain/DetailMesh_Rover_plain'
                    info['ground_material_root'] = '/World/Environment'
                else:
                    info['quality_settings']['geometry_version'] = 'original-source-v1'
                try:
                    stage.GetRootLayer().customLayerData = dict(original, engineering_metadata_json=json.dumps(data))
                    errors = []
                    showcase.validate_showcase_environment(stage, errors)
                    self.assertTrue(errors, kind)
                    self.assertIn('quality', ' '.join(errors))
                finally:
                    stage.GetRootLayer().customLayerData = original

    def test_actual_terrain_and_rock_topology_cannot_be_declared_away(self):
        from pxr import Gf, Usd, UsdGeom, Vt
        for kind in ('terrain_faces', 'terrain_spacing', 'rock_faces', 'rock_indices'):
            with self.subTest(kind=kind):
                stage = Usd.Stage.Open(os.environ['VIPER_SHOWCASE_STAGE'])
                info = json.loads(stage.GetRootLayer().customLayerData['engineering_metadata_json'])['showcase']
                if info['quality'] != 'light':
                    self.skipTest('light topology gate requires a light stage')
                stage.SetEditTarget(stage.GetSessionLayer())
                terrain = UsdGeom.Mesh(stage.GetPrimAtPath(showcase.TERRAIN))
                rock = UsdGeom.Mesh(stage.GetPrimAtPath(showcase.ENV+'/Rocks/Rock_0000'))
                if kind == 'terrain_faces':
                    terrain.GetFaceVertexCountsAttr().Set(Vt.IntArray([3]))
                elif kind == 'terrain_spacing':
                    points = Vt.Vec3fArray(terrain.GetPointsAttr().Get())
                    points[1] = points[1]+Gf.Vec3f(.125, 0, 0)
                    terrain.GetPointsAttr().Set(points)
                elif kind == 'rock_faces':
                    rock.GetFaceVertexCountsAttr().Set(Vt.IntArray([3]))
                else:
                    indices = Vt.IntArray(rock.GetFaceVertexIndicesAttr().Get())
                    indices[0] = indices[1]
                    rock.GetFaceVertexIndicesAttr().Set(indices)
                errors = []
                showcase.validate_showcase_environment(stage, errors)
                self.assertIn('quality terrain' if kind.startswith('terrain') else 'quality rock', ' '.join(errors))

    def test_legacy_missing_normals_allowed_but_v2_requires_them(self):
        from pxr import Usd, UsdGeom
        stage = Usd.Stage.Open(os.environ['VIPER_SHOWCASE_STAGE'])
        info = json.loads(stage.GetRootLayer().customLayerData['engineering_metadata_json'])['showcase']
        if info['quality'] != 'light':
            self.skipTest('original terrain has its own authored normals')
        stage.SetEditTarget(stage.GetSessionLayer())
        UsdGeom.Mesh(stage.GetPrimAtPath(showcase.TERRAIN)).GetNormalsAttr().Block()
        errors = []
        showcase.validate_showcase_environment(stage, errors)
        if info['geometry_version'] == 'uniform1m-rock80-v1':
            self.assertEqual(errors, [])
        else:
            self.assertIn('terrain smooth normals', ' '.join(errors))

    def test_v2_normals_material_and_collision_mutations(self):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
        source = os.environ['VIPER_SHOWCASE_STAGE']
        for kind, expected in (('normals_nonfinite', 'terrain smooth normals'),
                               ('normals_nonunit', 'terrain smooth normals'),
                               ('normals_wrong_gradient', 'terrain smooth normals'),
                               ('normals_count', 'terrain smooth normals'),
                               ('normals_interpolation', 'terrain smooth normals'),
                               ('matte_roughness', 'terrain matte material'),
                               ('matte_metallic', 'terrain matte material'),
                               ('matte_unbound', 'terrain matte material'),
                               ('terrain_collision', 'new terrain collision disabled')):
            with self.subTest(kind=kind):
                stage = Usd.Stage.Open(source)
                info = json.loads(stage.GetRootLayer().customLayerData['engineering_metadata_json'])['showcase']
                if info['geometry_version'] != 'uniform0p5m-rock320-v2':
                    self.skipTest('v2-only display contract; legacy stage remains supported')
                stage.SetEditTarget(stage.GetSessionLayer())
                mesh = UsdGeom.Mesh(stage.GetPrimAtPath(showcase.TERRAIN))
                if kind.startswith('normals_'):
                    if kind == 'normals_interpolation':
                        mesh.SetNormalsInterpolation('constant')
                    elif kind == 'normals_count':
                        mesh.GetNormalsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)]))
                    else:
                        normals = Vt.Vec3fArray(mesh.GetNormalsAttr().Get())
                        normals[0] = (Gf.Vec3f(float('nan'), 0, 1) if kind == 'normals_nonfinite' else
                                      Gf.Vec3f(0, 0, -1) if kind == 'normals_wrong_gradient' else Gf.Vec3f(0, 0, 2))
                        mesh.GetNormalsAttr().Set(normals)
                elif kind == 'terrain_collision':
                    UsdPhysics.CollisionAPI(mesh.GetPrim()).GetCollisionEnabledAttr().Set(False)
                else:
                    binding = UsdShade.MaterialBindingAPI(mesh.GetPrim())
                    material, _ = binding.ComputeBoundMaterial()
                    self.assertTrue(material)
                    if kind == 'matte_unbound':
                        binding.UnbindAllBindings()
                    else:
                        shader, _, _ = material.ComputeSurfaceSource()
                        shader.GetInput('roughness' if kind == 'matte_roughness' else 'metallic').Set(.2 if kind == 'matte_roughness' else 1.)
                errors = []
                showcase.validate_showcase_environment(stage, errors)
                self.assertIn(expected, ' '.join(errors))

    def test_metadata_old_collider_and_annotation_mutations(self):
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        source = os.environ['VIPER_SHOWCASE_STAGE']
        for kind, expected in (('metadata','engineering_metadata'), ('collider','old environment'), ('annotation','annotation geometry'),
                               ('disabled_rock','rock proxy collision disabled')):
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
            elif kind == 'disabled_rock':
                rock = stage.GetPrimAtPath('/World/ShowcaseEnvironment/Rocks/Rock_0000')
                self.assertTrue(rock and rock.HasAPI(UsdPhysics.CollisionAPI))
                UsdPhysics.CollisionAPI(rock).GetCollisionEnabledAttr().Set(False)
            else:
                UsdGeom.Cube.Define(stage, '/World/ShowcaseViews/LeakedBoundary')
            result = showcase.validate_showcase(stage, Path(__file__).parents[2]/'config/isaac_viper.yaml')
            stage.GetRootLayer().customLayerData = original_data
            self.assertFalse(result['passed'], (kind, result))
            self.assertIn(expected, ' '.join(result['errors']))


if __name__ == '__main__':
    unittest.main()
