"""真实 USD 合同测试；无 PXR 时显式跳过，不构成几何验收。"""
import importlib.util
import os
from pathlib import Path
import unittest

PXR = importlib.util.find_spec("pxr") is not None


@unittest.skipUnless(PXR and os.environ.get("VIPER_TEST_STAGE"),
                     "需要 PXR 和 VIPER_TEST_STAGE；几何验收 NOT_RUN")
class StageContractTest(unittest.TestCase):
    def test_actual_topology_mass_and_zero_initial_state(self):
        """阻止将原 447kg/17关节模型当作规范骨架交付。"""
        from pxr import Usd, UsdPhysics
        stage = Usd.Stage.Open(os.environ["VIPER_TEST_STAGE"])
        prims = [p for p in stage.Traverse() if str(p.GetPath()).startswith("/World/ProjectVIPER/")]
        bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
        self.assertEqual(len(bodies), 9)
        self.assertEqual(len(joints), 8)
        self.assertAlmostEqual(sum(UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in bodies), 50)
        for prim in bodies:
            body = UsdPhysics.RigidBodyAPI(prim)
            self.assertEqual(tuple(body.GetVelocityAttr().Get()), (0, 0, 0))
            self.assertEqual(tuple(body.GetAngularVelocityAttr().Get()), (0, 0, 0))

    def test_generated_geometry_and_material_contract(self):
        """缩放轮径、错误包络、悬空关节、遮挡或缺材质均必须失败。"""
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        profile = load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml")
        result = validate_stage(os.environ["VIPER_TEST_STAGE"], profile)
        self.assertTrue(result["passed"], result["errors"])
        self.assertAlmostEqual(result["mass_kg"], 50)
        self.assertAlmostEqual(result["steering_sweep_width_m"], 1.0216603474945676, places=6)
        self.assertGreater(result["visual_mesh_count"], 20)
        self.assertGreater(result["material_asset_count"], 20)

    def test_validator_rejects_physics_contract_mutations(self):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        profile = load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml")
        def redistribute(stage):
            UsdPhysics.MassAPI(stage.GetPrimAtPath("/World/ProjectVIPER/chassis")).GetMassAttr().Set(39)
            UsdPhysics.MassAPI(stage.GetPrimAtPath("/World/ProjectVIPER/fl_steer")).GetMassAttr().Set(1.5)
        mutations = {
            "redistributed_mass": redistribute,
            "steering_limit": lambda s: UsdPhysics.RevoluteJoint(s.GetPrimAtPath("/World/ProjectVIPER/fl_steering_joint")).GetUpperLimitAttr().Set(10),
            "local_joint_rotation": lambda s: UsdPhysics.RevoluteJoint(s.GetPrimAtPath("/World/ProjectVIPER/fl_rolling_joint")).GetLocalRot0Attr().Set(Gf.Quatf(.5, .5, .5, .5)),
            "disabled_collision": lambda s: UsdPhysics.CollisionAPI(s.GetPrimAtPath("/World/ProjectVIPER/fl_wheel/Collision")).GetCollisionEnabledAttr().Set(False),
            "scaled_collision": lambda s: UsdGeom.Xformable(s.GetPrimAtPath("/World/ProjectVIPER/fl_wheel/Collision")).AddScaleOp().Set(Gf.Vec3f(2)),
        }
        for name, mutation in mutations.items():
            with self.subTest(name=name):
                stage = Usd.Stage.Open(os.environ["VIPER_TEST_STAGE"])
                stage.SetEditTarget(stage.GetSessionLayer())
                mutation(stage)
                self.assertFalse(validate_stage(stage, profile)["passed"], name)

    def test_disabled_chassis_and_knuckle_collisions_are_rejected(self):
        """I1：所有规范刚体必须保有已启用的碰撞。"""
        from pxr import Usd, UsdPhysics
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        profile = load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml")
        for paths in (("chassis/CollisionUpper",), ("chassis/CollisionKeel",),
                      ("chassis/CollisionUpper", "chassis/CollisionKeel"),
                      *((corner + "_steer/Collision",) for corner in ("fl", "fr", "rl", "rr"))):
            with self.subTest(paths=paths):
                stage = Usd.Stage.Open(os.environ["VIPER_TEST_STAGE"])
                stage.SetEditTarget(stage.GetSessionLayer())
                for path in paths:
                    UsdPhysics.CollisionAPI(stage.GetPrimAtPath("/World/ProjectVIPER/" + path)).GetCollisionEnabledAttr().Set(False)
                result = validate_stage(stage, profile)
                self.assertFalse(result["passed"], paths)
                self.assertIn("collision_enabled", " ".join(result["errors"]))

    def test_knuckle_actual_size_transform_inertia_and_bounds(self):
        """I2：实际球体尺寸、变换、惯性、包络及遮挡均不得漏检。"""
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        profile = load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml")
        def sphere(stage):
            return stage.GetPrimAtPath("/World/ProjectVIPER/fl_steer/Collision")
        mutations = {
            "oversized": (lambda s: UsdGeom.Sphere(sphere(s)).GetRadiusAttr().Set(2), "nominal_visual_envelope"),
            "scaled": (lambda s: UsdGeom.Xformable(sphere(s)).AddScaleOp().Set(Gf.Vec3f(2, 1, 1)), "knuckle_collision_transform"),
            "translated": (lambda s: UsdGeom.Xformable(sphere(s)).AddTranslateOp().Set(Gf.Vec3d(.1, 0, 0)), "knuckle_collision_transform"),
            "forward_occluder": (lambda s: UsdGeom.Xformable(sphere(s)).AddTranslateOp().Set(Gf.Vec3d(.18, -.265, .9405)), "forward_fov_self_occlusion"),
            "wrong_inertia": (lambda s: UsdPhysics.MassAPI(s.GetPrimAtPath("/World/ProjectVIPER/fl_steer")).GetDiagonalInertiaAttr().Set(Gf.Vec3f(.01)), "sphere_inertia"),
        }
        for name, (mutation, error) in mutations.items():
            with self.subTest(name=name):
                stage = Usd.Stage.Open(os.environ["VIPER_TEST_STAGE"])
                stage.SetEditTarget(stage.GetSessionLayer())
                mutation(stage)
                result = validate_stage(stage, profile)
                self.assertFalse(result["passed"], name)
                self.assertIn(error, " ".join(result["errors"]))
                if name == "oversized":
                    self.assertGreater(result["nominal_visual_bounds_m"][1][1], 2)

    def test_unsupported_visible_boundable_is_explicitly_rejected(self):
        """I2：新出现的可见Boundable不能被空points静默丢弃。"""
        from pxr import Usd, UsdGeom
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        stage = Usd.Stage.Open(os.environ["VIPER_TEST_STAGE"])
        stage.SetEditTarget(stage.GetSessionLayer())
        cone = UsdGeom.Cone.Define(stage, "/World/ProjectVIPER/chassis/Visual/UnexpectedCone")
        cone.CreateRadiusAttr(2)
        cone.CreateHeightAttr(5)
        result = validate_stage(stage, load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml"))
        self.assertFalse(result["passed"])
        self.assertIn("unsupported_visible_boundable", " ".join(result["errors"]))

    def test_validator_rejects_old_scene(self):
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        old = os.environ.get("VIPER_OLD_STAGE")
        if not old:
            self.skipTest("未提供只读旧展示场景")
        result = validate_stage(old, load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml"))
        self.assertFalse(result["passed"])
        self.assertIn("rigid_body_count", " ".join(result["errors"]))
        self.assertIn("mass_kg", " ".join(result["errors"]))

    def test_validator_detects_real_usd_mutations(self):
        """内存会话层篡改实际碰撞、关节、初速、发射朝向，必须被发现。"""
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.validate_stage import validate_stage
        profile = load_profile(Path(__file__).parents[2] / "config/isaac_viper.yaml")
        mutations = [
            lambda s: UsdGeom.Cylinder(s.GetPrimAtPath("/World/ProjectVIPER/fl_wheel/Collision")).GetRadiusAttr().Set(.20),
            lambda s: UsdPhysics.RevoluteJoint(s.GetPrimAtPath("/World/ProjectVIPER/fl_rolling_joint")).GetBody1Rel().SetTargets([]),
            lambda s: UsdPhysics.RigidBodyAPI(s.GetPrimAtPath("/World/ProjectVIPER/chassis")).GetVelocityAttr().Set(Gf.Vec3f(1, 0, 0)),
            lambda s: UsdGeom.Xformable(s.GetPrimAtPath("/World/ProjectVIPER/chassis/FrontSensors/left_camera")).ClearXformOpOrder(),
        ]
        for mutation in mutations:
            stage = Usd.Stage.Open(os.environ["VIPER_TEST_STAGE"])
            stage.SetEditTarget(stage.GetSessionLayer())
            mutation(stage)
            self.assertFalse(validate_stage(stage, profile)["passed"])


@unittest.skipUnless(PXR and os.environ.get("VIPER_TEST_STAGE") and os.environ.get("VIPER_CONTRACT_OUTPUT"),
                     "需要PXR、VIPER_TEST_STAGE及新的VIPER_CONTRACT_OUTPUT；启动合同USD验收NOT_RUN")
class StartupStageContractTest(unittest.TestCase):
    """只读原v5；合法YAML变更及session变更都不得写回冻结源。"""

    def setUp(self):
        self.output = Path(os.environ['VIPER_CONTRACT_OUTPUT']) / self._testMethodName
        self.output.mkdir(parents=True, exist_ok=False)
        self.frozen = Path(os.environ['VIPER_TEST_STAGE']).resolve()
        import hashlib
        self.before = hashlib.sha256(self.frozen.read_bytes()).hexdigest()

    def tearDown(self):
        import hashlib
        self.assertEqual(hashlib.sha256(self.frozen.read_bytes()).hexdigest(), self.before)

    def profile(self, directory, change=None):
        import yaml
        from tools.isaac_viper.config import load_profile
        repo = Path(__file__).parents[2]
        raw = yaml.safe_load((repo/'config/isaac_viper.yaml').read_text(encoding='utf-8'))
        cap = yaml.safe_load((repo/'config/wheel_orin.yaml').read_text(encoding='utf-8'))
        if change:
            change(raw, cap)
        raw['sources'] = {'capability': 'capability.yaml',
                          'exploration': str(repo/'config/exploration_navigation.yaml')}
        directory.mkdir(exist_ok=False)
        (directory/'capability.yaml').write_text(yaml.safe_dump(cap), encoding='utf-8')
        path = directory/'profile.yaml'
        path.write_text(yaml.safe_dump(raw), encoding='utf-8')
        return load_profile(path)

    def save_result(self, name, result):
        import json
        (self.output/(name+'.json')).write_text(json.dumps(result, indent=2), encoding='utf-8')

    def test_frozen_stage_against_legally_loaded_profiles(self):
        import json
        from tools.isaac_viper.run_scene import require_stage_profile
        for field in ('original', 'diameter', 'wheelbase', 'track', 'extrinsic', 'baseline',
                      'focal_length_mm', 'horizontal_aperture_mm', 'vertical_aperture_mm'):
            with self.subTest(field=field):
                def change(p, c):
                    if field in ('diameter', 'wheelbase', 'track'):
                        key = {'diameter':'wheel_diameter_m', 'wheelbase':'wheelbase_m', 'track':'track_width_m'}[field]
                        c['capability'][key] *= .98
                    elif field == 'extrinsic':
                        for side in ('left', 'right'):
                            p['sensors']['stereo'][side+'_origin_m'][2] = 1.12
                    elif field == 'baseline':
                        p['sensors']['stereo'].update(baseline_m=.16,
                            left_origin_m=[.567,.08,1.10], right_origin_m=[.567,-.08,1.10])
                    elif field != 'original':
                        p['sensors']['stereo'][field] *= 2
                directory = self.output/field
                profile = self.profile(directory, change)
                report = directory/'contract.json'
                if field == 'original':
                    self.assertTrue(require_stage_profile(self.frozen, profile, report)['passed'])
                else:
                    with self.assertRaisesRegex(RuntimeError, 'stage/profile contract'):
                        require_stage_profile(self.frozen, profile, report)
                result = json.loads(report.read_text(encoding='utf-8'))
                self.assertEqual(result['passed'], field == 'original')
                if field.endswith('_mm'):
                    self.assertIn(field, ' '.join(result['errors']))

    def test_validator_reads_actual_camera_optics(self):
        from pxr import Usd
        from tools.isaac_viper.validate_stage import validate_stage
        profile = self.profile(self.output/'original')
        for attribute, value, error in (
                ('focalLength', 48., 'focal_length_mm'),
                ('horizontalAperture', 50., 'horizontal_aperture_mm'),
                ('verticalAperture', 40., 'vertical_aperture_mm'),
                ('horizontalApertureOffset', 1., 'aperture_offset'),
                ('projection', 'orthographic', 'perspective')):
            with self.subTest(attribute=attribute):
                stage = Usd.Stage.Open(str(self.frozen))
                stage.SetEditTarget(stage.GetSessionLayer())
                prim = stage.GetPrimAtPath('/World/ProjectVIPER/chassis/FrontSensors/left_camera')
                prim.GetAttribute(attribute).Set(value)
                result = validate_stage(stage, profile)
                self.save_result(attribute, result)
                self.assertFalse(result['passed'])
                self.assertIn(error, ' '.join(result['errors']))

    def test_validator_rejects_runtime_frame_and_unit_mismatch(self):
        import json
        from pxr import Usd, UsdGeom, UsdPhysics
        from tools.isaac_viper.validate_stage import validate_stage
        profile = self.profile(self.output/'original')
        for mutation, error in (('imu_rotation', 'body_aligned_orientation'),
                ('lidar_rotation', 'body_aligned_orientation'), ('metadata_com', 'metadata_base_link_com'),
                ('metadata_footprint', 'metadata_footprint_translation'), ('mass_units', 'kilograms_per_unit')):
            with self.subTest(mutation=mutation):
                stage = Usd.Stage.Open(str(self.frozen))
                stage.SetEditTarget(stage.GetSessionLayer())
                if mutation.endswith('_rotation'):
                    name = mutation.split('_')[0]
                    UsdGeom.Xformable(stage.GetPrimAtPath('/World/ProjectVIPER/chassis/FrontSensors/'+name)).AddRotateZOp().Set(30)
                elif mutation == 'mass_units':
                    UsdPhysics.SetStageKilogramsPerUnit(stage, .001)
                else:
                    # root metadata另建匿名层；不编辑或保存冻结源root。
                    source = stage
                    stage = Usd.Stage.CreateInMemory()
                    stage.GetRootLayer().subLayerPaths = [str(self.frozen)]
                    for key in ('metersPerUnit', 'upAxis', 'kilogramsPerUnit'):
                        stage.SetMetadata(key, source.GetMetadata(key))
                    custom = dict(source.GetRootLayer().customLayerData)
                    metadata = json.loads(custom['engineering_metadata_json'])
                    key = ('base_link_origin_in_footprint_m' if mutation == 'metadata_com'
                           else 'base_link_to_footprint_translation_m')
                    metadata[key][2] += .02
                    custom['engineering_metadata_json'] = json.dumps(metadata)
                    stage.GetRootLayer().customLayerData = custom
                result = validate_stage(stage, profile)
                self.save_result(mutation, result)
                self.assertFalse(result['passed'])
                self.assertIn(error, ' '.join(result['errors']))


if __name__ == "__main__":
    unittest.main()
