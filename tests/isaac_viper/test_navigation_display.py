"""显示坐标/会话回归：不需要 ROS 或 Kit。"""
import copy
import json
from types import SimpleNamespace
import unittest
import numpy as np
from tools.isaac_viper import navigation_showcase as nav
from tools.isaac_viper import viewport_paths as vp
from tools.isaac_viper.known_regions import SOURCE_SHA

SESSION='a'*32
OTHER='b'*32

def ref(session=SESSION,state=0):
    return dict(session_id=session,segment_revision=2,traversability_revision=3,state=state,stamp_ns=0,points=[[.1,.1,0],[1.1,.1,0]])

def diag():
    return dict(session_id=SESSION,segment_revision='2',fine_traversability_revision='3',cycle_result='PLAN_FOUND',reason_code='PLAN_FOUND')

class DisplayRegressions(unittest.TestCase):
    def test_planner_zero_z_projects_to_varying_nonzero_map(self):
        self.assertTrue(hasattr(vp,'TerrainProjector'),'terrain display projection missing')
        projector=vp.TerrainProjector(np.array([[8.,9.,10.],[8.,9.,10.]]),[0,3,0,2],1.)
        raw=[[.1,.1,0],[1.1,.1,0]]; saved=copy.deepcopy(raw)
        segments=projector.segments(raw)
        self.assertAlmostEqual(segments[0][0][2],8.12)
        self.assertAlmostEqual(segments[0][-1][2],9.12)
        self.assertEqual(raw,saved)

    def test_unknown_and_outside_split_without_zero_fallback(self):
        self.assertTrue(hasattr(vp,'TerrainProjector'),'terrain display projection missing')
        projector=vp.TerrainProjector(np.array([[8.,np.nan,10.],[8.,np.nan,10.]]),[0,3,0,2],1.)
        segments=projector.segments([[-1,.1,0],[.1,.1,0],[2.9,.1,0],[4,.1,0]])
        self.assertTrue(segments)
        for segment in segments:
            self.assertTrue(all(0<=p[0]<3 and not 1<=p[0]<2 for p in segment))
            self.assertTrue(all(p[2]>8 for p in segment))
            self.assertLess(max(p[0] for p in segment)-min(p[0] for p in segment),1)

    def session(self):
        self.assertTrue(hasattr(nav,'DisplayPlanSession'),'accepted Action display binding missing')
        return nav.DisplayPlanSession()

    def verified(self):
        session=self.session(); session.begin_goal(SESSION,100)
        session.observe_global(dict(stamp_ns=0,points=[[.1,.1,0],[1.1,.1,0],[2.1,.1,0]]),101)
        session.observe_reference(ref(),102); session.accept_goal(SESSION); session.observe_diagnostics(diag())
        return session

    def test_retained_previous_session_never_live(self):
        session=self.session()
        self.assertFalse(session.observe_global(dict(stamp_ns=0,points=[[0,0,0],[1,0,0]]),1))
        session.begin_goal(SESSION,100); session.accept_goal(SESSION)
        session.observe_reference(ref(OTHER),102)
        session.observe_global(dict(stamp_ns=0,points=[[0,0,0],[1,0,0]]),99)
        session.observe_diagnostics(diag())
        self.assertEqual(session.snapshot()['paths'],{})

    def test_local_comes_from_current_reference_and_global_has_source_window(self):
        session=self.verified(); payload=session.snapshot()
        self.assertEqual(payload['paths']['local_path']['points'],ref()['points'])
        self.assertEqual(payload['paths']['local_path']['binding']['session_id'],SESSION)
        self.assertEqual(len(payload['paths']['global_route']['points']),3)
        self.assertEqual(payload['paths']['global_route']['binding']['transport_source_stamp_ns'],101)

    def test_invalidation_stops_live_and_preserves_verified_full_history(self):
        session=self.verified(); session.observe_reference(ref(state=1),103)
        payload=session.snapshot()
        self.assertEqual(payload['paths'],{})
        self.assertEqual(payload['goal_state'],'invalidated')
        self.assertEqual(len(payload['history_paths']['global_route']['points']),3)
        self.assertEqual(payload['history_paths']['global_route']['display_status'],'history')

    def test_terminal_and_new_goal_do_not_claim_history_live(self):
        session=self.verified(); session.finish('completed'); payload=session.snapshot()
        self.assertEqual(payload['paths'],{})
        self.assertTrue(payload['history_paths'])
        self.assertEqual(payload['goal_state'],'completed')
        session.begin_goal(OTHER,200); payload=session.snapshot()
        self.assertEqual(payload['paths'],{})
        self.assertEqual(payload['history_paths'],{})
        self.assertIsNone(payload['path_reference'])

    def test_viewport_rejects_mismatched_or_invalid_live_payload(self):
        payload=dict(schema=vp.SCHEMA,frame='map',source='live_ros',sequence=1,action_session_id=SESSION,
                     goal_accepted=True,goal_state='active',path_reference=ref(OTHER,state=1),
                     paths={'global_route':dict(stamp_ns=0,points=[[0,0,0],[1,0,0]])})
        with self.assertRaises(ValueError): vp.validate_payload(payload)

    def test_trace_reset_keeps_stamp_highwater(self):
        trace=nav.ActualTrace(); trace.append(10,[1,2,3],[0,0,0,1],[0,0,0])
        self.assertTrue(hasattr(trace,'reset'),'new goal must clear trace')
        trace.reset(); self.assertEqual(trace.points,[])
        self.assertFalse(trace.append(10,[99,99,99],[0,0,0,1],[0,0,0]))
        self.assertTrue(trace.append(11,[1,2,3],[0,0,0,1],[0,0,0]))

def test_overlay_default_hides_local_without_mutating_original_reference():
    session=DisplayRegressions().verified()
    assert hasattr(nav,'overlay_snapshot'),'display-only local filtering missing'
    actual={'actual_trace':dict(stamp_ns=9,points=[[0,0,0],[1,0,0]])}
    before=session.snapshot()
    filtered=nav.overlay_snapshot(session,actual)
    assert set(filtered['paths'])=={'global_route','actual_trace'}
    assert set(filtered['history_paths'])=={'global_route'}
    assert filtered['path_reference']==before['path_reference']
    assert session.snapshot()==before
    visible=nav.overlay_snapshot(session,actual,show_local_path=True)
    assert set(visible['paths'])=={'global_route','local_path','actual_trace'}
    assert set(visible['history_paths'])=={'global_route','local_path'}


def test_trace_capacity_preserves_full_travel_and_turns_with_audit():
    trace=nav.ActualTrace(limit=8)
    journey=([[float(x),0.,0.] for x in range(51)]
             +[[50.,float(y),0.] for y in range(1,51)]
             +[[float(x),50.,0.] for x in range(51,151)])
    for stamp,point in enumerate(journey,1):
        assert trace.append(stamp,point,[0,0,0,1],[0,0,0])
        assert len(trace.points)<=8
    assert trace.points[0]==dict(stamp_ns=1,xyz=[0.,0.,0.])
    assert trace.points[-1]==dict(stamp_ns=201,xyz=[150.,50.,0.])
    retained=[p['xyz'] for p in trace.points]
    assert [50.,0.,0.] in retained and [50.,50.,0.] in retained
    assert trace.highwater==201
    assert [p['stamp_ns'] for p in trace.points]==sorted(p['stamp_ns'] for p in trace.points)
    assert hasattr(trace,'audit'),'progressive simplification must have an audit'
    audit=trace.audit()
    assert audit['accepted_samples']==201 and audit['retained_samples']==len(trace.points)
    assert audit['compactions']>0 and audit['first_stamp_ns']==1 and audit['last_stamp_ns']==201
    assert audit['raw_samples']=='all preserved in actual_odom events; display trace only is simplified'
    assert not trace.append(201,[999,999,999],[0,0,0,1],[0,0,0])
    assert trace.audit()['accepted_samples']==201
    trace.reset()
    assert trace.highwater==201 and trace.points==[]
    assert trace.append(202,[150,50,0],[0,0,0,1],[0,0,0])
    assert trace.audit()['accepted_samples']==1


def test_viewport_poll_draws_projected_plan_and_original_truth_trace(tmp_path):
    session=nav.DisplayPlanSession(); session.begin_goal(SESSION,100)
    session.observe_global(dict(stamp_ns=0,points=[[.1,.1,0],[1.1,.1,0],[2.1,.1,0]]),101)
    session.observe_reference(ref(),102); session.accept_goal(SESSION); session.observe_diagnostics(diag())
    payload=dict(schema=vp.SCHEMA,run_id='test',frame='map',source='live_ros',sequence=1,
                 display_map={'sha256':'unit-map','region':'A','source_stage_sha256':SOURCE_SHA,'path':'cached-unit-map.npz'},**session.snapshot())
    payload['paths']['actual_trace']=dict(stamp_ns=9,points=[[.1,.1,11],[.2,.1,11]])
    path=tmp_path/'overlay.json'; path.write_text(json.dumps(payload),encoding='utf-8')
    captured=[]
    class Scene:
        def clear(self): captured.clear()
        def __enter__(self): return self
        def __exit__(self,*args): return False
    class Curve:
        CurveType=SimpleNamespace(LINEAR=0)
        def __init__(self,points,**kwargs): captured.append((points,kwargs))
    overlay=vp.ViewportPaths.__new__(vp.ViewportPaths)
    overlay.path=path; overlay.digest=None; overlay.sequence=-1; overlay.session=None
    overlay.map_identity=('A',SOURCE_SHA,'cached-unit-map.npz','unit-map')
    overlay.map_sha='unit-map'; overlay.projector=vp.TerrainProjector(np.array([[8.,9.,10.],[8.,9.,10.]]),[0,3,0,2],1.)
    overlay.view=SimpleNamespace(scene=Scene()); overlay.sc=SimpleNamespace(Curve=Curve); overlay.status_label=SimpleNamespace(text=''); overlay.legend_label=SimpleNamespace(text='')
    overlay.poll()
    global_curves=[points for points,style in captured if style['colors']==[vp.COLORS['global_route']]]
    actual=[points for points,style in captured if style['colors']==[vp.COLORS['actual_trace']]]
    assert global_curves and min(p[2] for p in global_curves[0])==8.12 and max(p[2] for p in global_curves[0])==10.12
    assert actual==[[[.1,.1,11.12],[.2,.1,11.12]]]
    assert all(style['thicknesses']==[8.] for _,style in captured if style['colors']==[vp.COLORS['global_route']])
    assert all(style['thicknesses']==[3.] for _,style in captured if style['colors']==[vp.COLORS['actual_trace']])
    assert all(p[2]==0 for p in json.loads(path.read_text())['paths']['global_route']['points'])
    assert 'HISTORY' in overlay.status_label.text
    payload['sequence']=2; payload['path_reference']=ref(OTHER,state=1)
    path.write_text(json.dumps(payload),encoding='utf-8')
    try: overlay.poll()
    except ValueError: pass
    else: raise AssertionError('invalid session/ref must be rejected by real poll')
    assert captured==[] and 'REJECTED' in overlay.status_label.text

if __name__=='__main__': unittest.main()
