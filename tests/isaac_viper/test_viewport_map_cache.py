"""缓存命中也不得绕过region/source身份；受控Scene仅代替本机不可用Kit绘制。"""
import copy
import hashlib
import json
from types import SimpleNamespace
import numpy as np
import pytest
from tools.isaac_viper import navigation_showcase as nav
from tools.isaac_viper import viewport_paths as vp
from tools.isaac_viper.known_regions import SOURCE_SHA


@pytest.mark.parametrize('mutation',['region_C','region_A','source_sha','path'])
def test_cached_flat_map_revalidates_identity_and_clears_stale_curves(tmp_path,monkeypatch,mutation):
    native_map=tmp_path/'flat.npz'
    np.savez_compressed(native_map,elevation=np.full((600,600),7.,np.float32),bounds_xy_m=[-490.,-370.,-250.,-130.],resolution_m=.2)
    sid='a'*32; points=[[-489.5,-249.5,0],[-489.3,-249.3,0]]
    session=nav.DisplayPlanSession();session.begin_goal(sid,100);session.accept_goal(sid)
    session.observe_global(dict(stamp_ns=0,points=points),101)
    session.observe_reference(dict(session_id=sid,segment_revision=2,traversability_revision=3,state=0,stamp_ns=0,points=points),102)
    session.observe_diagnostics(dict(session_id=sid,segment_revision='2',fine_traversability_revision='3',cycle_result='PLAN_FOUND',reason_code='PLAN_FOUND'))
    source=dict(path=str(native_map),sha256=hashlib.sha256(native_map.read_bytes()).hexdigest(),source_stage_sha256=SOURCE_SHA,region='KNOWN_DEMO_FLAT')
    payload=dict(schema=vp.SCHEMA,run_id='cached-map',frame='map',source='live_ros',sequence=1,display_map=source,**session.snapshot())
    captured=[]
    class Scene:
        def clear(self): captured.clear()
        def __enter__(self): return self
        def __exit__(self,*args): return False
    class Curve:
        CurveType=SimpleNamespace(LINEAR=0)
        def __init__(self,points,**kwargs): captured.append(points)
    overlay=vp.ViewportPaths.__new__(vp.ViewportPaths)
    overlay.path=tmp_path/'overlay.json';overlay.digest=None;overlay.sequence=-1;overlay.session=None;overlay.projector=None;overlay.map_sha=None;overlay.map_identity=None
    overlay.view=SimpleNamespace(scene=Scene());overlay.sc=SimpleNamespace(Curve=Curve)
    overlay.status_label=SimpleNamespace(text='');overlay.legend_label=SimpleNamespace(text='')
    original=vp.TerrainProjector.from_source; loaded=[]
    def load(cls,value): loaded.append(copy.deepcopy(value));return original(value)
    monkeypatch.setattr(vp.TerrainProjector,'from_source',classmethod(load))
    overlay.path.write_text(json.dumps(payload),encoding='utf-8');overlay.poll()
    assert captured and len(loaded)==1
    payload['sequence']=2;overlay.path.write_text(json.dumps(payload),encoding='utf-8');overlay.poll()
    assert captured and len(loaded)==1,'identical validated source may reuse cache'
    payload['sequence']=3
    if mutation=='region_C': payload['display_map']['region']='C'
    elif mutation=='region_A': payload['display_map']['region']='A'
    elif mutation=='source_sha': payload['display_map']['source_stage_sha256']='0'*64
    else: payload['display_map']['path']=str(tmp_path/'missing-map.npz')
    with pytest.raises(ValueError):
        overlay.path.write_text(json.dumps(payload),encoding='utf-8');overlay.poll()
    assert captured==[] and 'REJECTED' in overlay.status_label.text
