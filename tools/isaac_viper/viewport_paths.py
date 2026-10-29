"""真实 ROS 路径的 Kit UI 叠加；二维规划高度仅按核验 A 图投影。"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np

SCHEMA='lunar.viper-viewport-paths.v1'
MAX_BYTES=4*1024*1024
COLORS={'global_route':0xffff7733,'local_path':0xff33ddff,'actual_trace':0xffffff22}


def validate_points(points):
    if not isinstance(points,list) or len(points)>10000: raise ValueError('path point limit')
    for point in points:
        if not isinstance(point,list) or len(point)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>10000 for v in point): raise ValueError('invalid point')


def _reference_key(reference):
    return tuple(reference.get(k) for k in ('session_id','segment_revision','traversability_revision'))


def validate_payload(payload):
    if not isinstance(payload,dict) or payload.get('schema')!=SCHEMA or payload.get('frame')!='map' or payload.get('source')!='live_ros': raise ValueError('invalid overlay identity')
    if not isinstance(payload.get('sequence'),int) or payload['sequence']<0: raise ValueError('invalid sequence')
    for history,paths in ((False,payload.get('paths')),(True,payload.get('history_paths',{}))):
        if not isinstance(paths,dict) or set(paths)-set(COLORS): raise ValueError('unknown path kind')
        for name,path in paths.items():
            if not isinstance(path,dict) or not isinstance(path.get('stamp_ns'),int) or path['stamp_ns']<0: raise ValueError('invalid source stamp')
            validate_points(path.get('points'))
            if name=='actual_trace':
                if history: raise ValueError('trace is actual, never planner history')
                continue
            binding=path.get('binding',{}); reference=path.get('verified_reference',{})
            if not isinstance(binding,dict) or not isinstance(reference,dict): raise ValueError('invalid reference/binding object')
            validate_points(reference.get('points'))
            session=payload.get('action_session_id')
            if not isinstance(session,str) or len(session)!=32 or any(c not in '0123456789abcdef' for c in session): raise ValueError('invalid accepted Action UUID')
            if payload.get('goal_accepted') is not True or not payload.get('action_session_id'): raise ValueError('planner display requires accepted Action')
            if binding.get('session_id')!=payload['action_session_id'] or reference.get('session_id')!=payload['action_session_id']: raise ValueError('planner session mismatch')
            if reference.get('state')!=0 or not reference.get('points') or _reference_key(binding)!=_reference_key(reference): raise ValueError('planner requires matching ACTIVE reference/revision')
            if binding.get('reference_stamp_ns')!=reference.get('stamp_ns'): raise ValueError('reference stamp mismatch')
            if history:
                if path.get('display_status')!='history': raise ValueError('history must be visibly marked')
            else:
                current=payload.get('path_reference') or {}
                if not isinstance(current,dict): raise ValueError('invalid current reference')
                if payload.get('goal_state')!='active' or current.get('state')!=0 or _reference_key(current)!=_reference_key(reference) or current.get('stamp_ns')!=reference.get('stamp_ns'): raise ValueError('live reference is stale or invalidated')
                if path.get('display_status')!='live': raise ValueError('live status mismatch')
            if name=='local_path':
                if path['points']!=reference['points'] or path['stamp_ns']!=reference['stamp_ns']: raise ValueError('local display must be original reference path')
            else:
                request=binding.get('request_source_time_ns',-1); source=binding.get('transport_source_stamp_ns',-1); ref_source=binding.get('reference_transport_stamp_ns',-1)
                if not all(isinstance(v,int) and not isinstance(v,bool) for v in (request,source,ref_source)): raise ValueError('invalid transport source stamps')
                if not 0<request<=source<=ref_source or ref_source-source>1000000000: raise ValueError('global lacks current goal transport publication window')
    return payload


class TerrainProjector:
    """只生成显示点：原规划 XY 折线不变，已知图取高度；未知/越界断线。"""
    def __init__(self,elevation,bounds,resolution):
        self.elevation=np.asarray(elevation,float); self.bounds=list(bounds); self.resolution=float(resolution)
        if self.elevation.ndim!=2 or self.elevation.size>1000000 or np.isinf(self.elevation).any() or len(self.bounds)!=4 or not all(math.isfinite(v) for v in self.bounds) or not math.isfinite(self.resolution) or self.resolution<=0: raise ValueError('invalid display map')
        x0,x1,y0,y1=self.bounds
        if x1<=x0 or y1<=y0 or self.elevation.shape!=(round((y1-y0)/resolution),round((x1-x0)/resolution)): raise ValueError('display map dimensions mismatch')

    def segments(self,points):
        validate_points(points)
        samples=[]
        for i,point in enumerate(points):
            if i:
                previous=points[i-1]; steps=max(1,math.ceil(math.dist(previous[:2],point[:2])/(self.resolution/2)))
                if len(samples)+steps>50000: raise ValueError('display sample limit')
                for step in range(1,steps):
                    t=step/steps; samples.append([previous[0]+t*(point[0]-previous[0]),previous[1]+t*(point[1]-previous[1])])
            samples.append(point[:2])
        result=[]; segment=[]; x0,x1,y0,y1=self.bounds
        for x,y in samples:
            z=float('nan')
            if x0<=x<x1 and y0<=y<y1: z=float(self.elevation[int(math.floor((y-y0)/self.resolution)),int(math.floor((x-x0)/self.resolution))])
            if math.isfinite(z): segment.append([x,y,z+.12])
            else:
                if len(segment)>=2: result.append(segment)
                segment=[]
        if len(segment)>=2: result.append(segment)
        return result

    @classmethod
    def from_source(cls,source):
        from tools.isaac_viper.navigation_scene import SOURCE_SHA
        if not isinstance(source,dict) or source.get('region')!='A' or source.get('source_stage_sha256')!=SOURCE_SHA: raise ValueError('display map must be the current original A map')
        if not isinstance(source.get('path'),str) or not isinstance(source.get('sha256'),str): raise ValueError('invalid display map source')
        path=Path(source['path'])
        if path.stat().st_size>16*1024*1024 or hashlib.sha256(path.read_bytes()).hexdigest()!=source.get('sha256'): raise ValueError('display map hash/size mismatch')
        with np.load(path,allow_pickle=False) as archive:
            bounds=archive['bounds_xy_m'].tolist(); resolution=float(archive['resolution_m'])
            if not (235<=bounds[0]<bounds[1]<=335 and -315<=bounds[2]<bounds[3]<=-215 and resolution==.2): raise ValueError('display map outside authorized A bounds/resolution')
            return cls(archive['elevation'],bounds,resolution)


class ViewportPaths:
    """SceneView 跟随相机；路径不会进入 RTX/碰撞世界。"""
    def __init__(self,path):
        self.path=Path(path); self.digest=None; self.sequence=-1; self.session=None; self.projector=None; self.map_sha=None
        import omni.ui as ui
        from omni.ui import scene as sc
        from omni.kit.viewport.utility import get_active_viewport_window
        self.sc=sc; self.window=get_active_viewport_window()
        if self.window is None: raise RuntimeError('navigation overlay requires an active viewport window')
        with self.window.get_frame('lunar.navigation.showcase'):
            with ui.ZStack():
                self.view=sc.SceneView()
                with ui.VStack():
                    ui.Spacer(height=12)
                    ui.Label('  Global: blue | Local reference: yellow | Actual truth odom: cyan',height=24,style={'color':0xffffffff})
                    self.status_label=ui.Label('  Waiting for accepted Action and verified reference',height=24,style={'color':0xffffffff})
                    ui.Spacer()
        self.window.viewport_api.add_scene_view(self.view)

    def poll(self):
        if not self.path.is_file(): return None
        if self.path.stat().st_size>MAX_BYTES: raise ValueError('overlay file limit')
        raw=self.path.read_bytes(); digest=hashlib.sha256(raw).hexdigest()
        if digest==self.digest: return None
        try:
            payload=validate_payload(json.loads(raw)); session=payload.get('run_id')
            if session==self.session and payload['sequence']<=self.sequence: raise ValueError('overlay sequence not increasing')
            groups=(payload['paths'],payload.get('history_paths',{}))
            if any(name!='actual_trace' for group in groups for name in group):
                source=payload.get('display_map',{})
                if not isinstance(source,dict): raise ValueError('invalid display map metadata')
                if self.projector is None or source.get('sha256')!=self.map_sha:
                    self.projector=TerrainProjector.from_source(source); self.map_sha=source['sha256']
            self.view.scene.clear()
            with self.view.scene:
                for group in groups:
                    for name,path in group.items():
                        segments=([[[x,y,z+.12] for x,y,z in path['points']]] if name=='actual_trace' else self.projector.segments(path['points']))
                        for segment in segments:
                            if len(segment)>=2:
                                self.sc.Curve(segment,curve_type=self.sc.Curve.CurveType.LINEAR,colors=[COLORS[name]],thicknesses=[4. if name=='actual_trace' else 2. if path.get('display_status')=='history' else 3.])
            history=bool(payload.get('history_paths'))
            self.status_label.text='  '+payload.get('goal_state','waiting')+(' | initial verified plan: HISTORY, frozen' if history else '')+' | planner height: A collision map (display only)'
        except (ValueError,KeyError,OSError) as exc:
            self.view.scene.clear(); self.status_label.text='  REJECTED overlay; no live planner claim'; self.digest=digest
            raise ValueError('navigation overlay rejected: '+str(exc)) from exc
        self.digest=digest; self.sequence=payload['sequence']; self.session=session
        return {'sequence':self.sequence,'paths':{k:len(v['points']) for k,v in payload['paths'].items()},'history_paths':{k:len(v['points']) for k,v in payload.get('history_paths',{}).items()},'goal_state':payload.get('goal_state'),'display_height_source':payload.get('display_map'),'geometry':'UI_ONLY; RTX invisible'}
