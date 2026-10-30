"""百米fixture是离线预检，不能当成ROS/Kit运行通过。"""
import math
from pathlib import Path
import unittest
import numpy as np
import yaml
from tools.isaac_viper import navigation_scene as ns
from tools.isaac_viper import viewport_paths as vp
from tools.isaac_viper import navigation_showcase as nav
from tools.isaac_viper.config import load_profile

ROOT=Path(__file__).resolve().parents[2]

class LongNavigation(unittest.TestCase):
    def test_demo_profile_isolated_and_preserves_safety(self):
        path=ROOT/'config/isaac_viper_navigation_demo.yaml'
        self.assertTrue(path.is_file(),'isolated navigation demo profile missing')
        demo=load_profile(path); original=load_profile(ROOT/'config/isaac_viper.yaml')
        self.assertTrue(demo['simulation_only'])
        self.assertEqual(demo['capability']['maximum_forward_speed_mps'],2.)
        self.assertEqual(demo['capability']['maximum_spin_rate_radps'],2.)
        for key,value in original['capability'].items():
            if key not in ('maximum_forward_speed_mps','maximum_spin_rate_radps'):
                self.assertEqual(demo['capability'][key],value,key)
        for key in ('sensors','ros','drive','simulation'):
            self.assertEqual(demo[key],original[key])
        cap=yaml.safe_load((ROOT/'config/wheel_orin.yaml').read_text(encoding='utf-8'))
        self.assertEqual(cap['capability']['maximum_forward_speed_mps'],.2)
        from tools.isaac_viper.drive import DriveController
        controller=DriveController(demo)
        self.assertTrue(controller.receive_twist(2.,0.,2.,0.))
        self.assertEqual(controller.target,(2.,.25),'2m/s turning remains constrained by .5m/s2 lateral limit')
        controller.receive_twist(0.,0.,2.,.1)
        self.assertEqual(controller.target,(0.,2.))

    def test_demo_capability_has_cpp_strict_root_schema(self):
        # C++ ValidateIdentity(surface_platform=true) 的有效根字段；真实parser另有gtest。
        capability=yaml.safe_load((ROOT/'config/wheel_isaac_navigation_demo.yaml').read_text(encoding='utf-8'))
        allowed={'platform','platform_id','platform_type','capability_version','base_frame_id','start_blind_zone_margin_m','capability'}
        self.assertEqual(set(capability)-allowed,set(),'unknown roots make real C++ navigator fail before a goal')
        self.assertEqual(capability['base_frame_id'],'base_footprint')
        profile=yaml.safe_load((ROOT/'config/isaac_viper_navigation_demo.yaml').read_text(encoding='utf-8'))
        self.assertTrue(profile['simulation_only'])
        self.assertEqual(profile['sources']['capability'],'wheel_isaac_navigation_demo.yaml')

    def test_long_route_bends_around_real_blocked_cells(self):
        self.assertTrue(hasattr(ns,'select_long_route'),'long route precheck missing')
        z=np.zeros((120,120)); relief=np.zeros_like(z)
        # 真实测试几何障碍的高度与起伏，不是规划输出。
        z[20:100,58:62]=1.; relief[20:100,58:62]=1.
        start,goal,audit=ns.select_long_route(z,relief,[0,120,0,120],1.,100.)
        self.assertGreaterEqual(math.dist(start,goal),100.)
        self.assertGreater(audit['path_length_m'],math.dist(start,goal))
        self.assertTrue(audit['line_of_sight_blocked'])
        self.assertTrue(audit['offline_only'])
        self.assertEqual(audit['connectivity'],'PASS')
        self.assertTrue(audit['blocked_cell_xy_m'])
        self.assertTrue(audit['endpoint_checks']['start']['footprint_clear'])
        # 只返回审计点/长度，绝不返回可被展示为导航路线的offline polyline。
        self.assertNotIn('path',audit)
        relief[:,58:62]=1.; z[:,58:62]=1.
        with self.assertRaises(ValueError):
            ns.select_long_route(z,relief,[0,120,0,120],1.,140.)

    def test_flat_or_unknown_map_is_not_obstacle_detour_fixture(self):
        self.assertTrue(hasattr(ns,'select_long_route'),'long route precheck missing')
        for z in (np.zeros((110,110)),np.full((110,110),np.nan)):
            with self.assertRaises(ValueError):
                ns.select_long_route(z,np.zeros_like(z),[0,110,0,110],1.,100.)

    def test_vector_raster_retains_partial_triangle_extrema_and_nan(self):
        self.assertTrue(hasattr(ns,'rasterize_triangles_batched'),'batched conservative raster missing')
        triangles=np.array([[[0,0,0],[1,0,2],[0,1,1]],[[.2,.2,3],[.8,.2,3],[.2,.8,3]]])
        expected,relief=ns.rasterize_triangles(triangles,[0,2,0,2],.2)
        actual,actual_relief=ns.rasterize_triangles_batched(triangles,[0,2,0,2],.2)
        np.testing.assert_allclose(actual,expected,atol=1e-6,equal_nan=True)
        np.testing.assert_allclose(actual_relief,relief,atol=1e-6,equal_nan=True)
        self.assertTrue(np.isnan(actual[0,5]))

    def test_only_verified_actual_navigator_global_length_meets_requirement(self):
        self.assertTrue(hasattr(nav,'long_route_evidence'),'actual navigator length evidence missing')
        session=nav.DisplayPlanSession(); sid='a'*32
        session.begin_goal(sid,100); session.accept_goal(sid)
        scene={'fixture_precheck':{'minimum_required_m':100.,'offline_only':True,'line_of_sight_blocked':True,'path_length_m':120.,'blocked_cell_xy_m':[55.,0.]}}
        self.assertFalse(nav.long_route_evidence(session,scene)['satisfied'])
        reference=dict(session_id=sid,segment_revision=1,traversability_revision=1,state=0,stamp_ns=0,points=[[0,0,0],[1,0,0]])
        session.observe_global(dict(stamp_ns=0,points=[[0,0,0],[110,0,0]]),101)
        session.observe_reference(reference,102)
        self.assertFalse(nav.long_route_evidence(session,scene)['satisfied'])
        session.observe_diagnostics(dict(session_id=sid,segment_revision='1',fine_traversability_revision='1',cycle_result='PLAN_FOUND',reason_code='PLAN_FOUND'))
        evidence=nav.long_route_evidence(session,scene)
        self.assertFalse(evidence['satisfied'],'straight global still crosses the known real blocker')
        self.assertEqual(evidence['navigator_global_length_m'],110.)
        session.begin_goal(sid,200); session.accept_goal(sid)
        session.observe_global(dict(stamp_ns=0,points=[[0,0,0],[55,10,0],[110,0,0]]),201)
        session.observe_reference(reference,202)
        session.observe_diagnostics(dict(session_id=sid,segment_revision='1',fine_traversability_revision='1',cycle_result='PLAN_FOUND',reason_code='PLAN_FOUND'))
        evidence=nav.long_route_evidence(session,scene)
        self.assertTrue(evidence['satisfied'])
        self.assertAlmostEqual(evidence['navigator_global_length_m'],111.80339887498948)
        self.assertEqual(evidence['session_id'],sid)
        session.begin_goal('b'*32,200)
        self.assertFalse(nav.long_route_evidence(session,scene)['satisfied'])

    def test_overview_scales_with_map_and_follow_uses_actual_pose(self):
        self.assertTrue(hasattr(vp,'navigation_camera_pose'),'adaptive camera missing')
        eye,target=vp.navigation_camera_pose([235,335,-315,-215],8.)
        self.assertEqual(target,[285.,-265.,8.25])
        self.assertGreater(eye[2]-target[2],80.)
        # 独立按相机投影检查非方形/高差场景所有八角点在画幅内。
        for bounds in ([235,335,-315,-215],[235,295,-315,-195]):
            eye,target=vp.navigation_camera_pose(bounds,20.,height_range=[6.,37.])
            forward=np.asarray(target)-eye; forward/=np.linalg.norm(forward)
            right=np.cross(forward,[0.,0.,1.]); right/=np.linalg.norm(right)
            up=np.cross(right,forward)
            for x in bounds[:2]:
                for y in bounds[2:]:
                    for z in (6.,39.):
                        delta=np.array([x,y,z])-eye; depth=np.dot(delta,forward)
                        self.assertGreater(depth,0.)
                        self.assertLess(abs(np.dot(delta,right))*24./depth,18.)
                        self.assertLess(abs(np.dot(delta,up))*24./depth,10.125)
        close_eye,close_target=vp.navigation_camera_pose([235,335,-315,-215],8.,actual_xyz=[300,-260,11])
        self.assertEqual(close_target,[300.,-260.,11.6])
        self.assertLess(math.dist(close_eye,close_target),15.)

if __name__=='__main__': unittest.main()
