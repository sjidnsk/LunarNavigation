"""真实 ROS 路径的 Kit UI 叠加；模块导入不需要 Kit，不写 USD。"""
import hashlib
import json
import math
from pathlib import Path

SCHEMA='lunar.viper-viewport-paths.v1'
MAX_BYTES=4*1024*1024
COLORS={'global_route':0xffff7733,'local_path':0xff33ddff,'actual_trace':0xffffff22}


def validate_payload(payload):
    if not isinstance(payload,dict) or payload.get('schema')!=SCHEMA or payload.get('frame')!='map' or payload.get('source')!='live_ros': raise ValueError('invalid overlay identity')
    if not isinstance(payload.get('sequence'),int) or payload['sequence']<0: raise ValueError('invalid sequence')
    paths=payload.get('paths')
    if not isinstance(paths,dict) or set(paths)-set(COLORS): raise ValueError('unknown path kind')
    for path in paths.values():
        if not isinstance(path,dict) or not isinstance(path.get('stamp_ns'),int) or path['stamp_ns']<0: raise ValueError('invalid source stamp')
        points=path.get('points')
        if not isinstance(points,list) or len(points)>10000: raise ValueError('path point limit')
        for point in points:
            if not isinstance(point,list) or len(point)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>10000 for v in point): raise ValueError('invalid point')
    return payload


class ViewportPaths:
    """SceneView 跟随 viewport 的相机矩阵；线条永不进入 RTX/碰撞世界。"""
    def __init__(self,path):
        self.path=Path(path); self.digest=None; self.sequence=-1; self.session=None
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
                    ui.Label('  Global: blue | Local: yellow | Actual truth odom: cyan',height=24,style={'color':0xffffffff})
                    ui.Spacer()
        self.window.viewport_api.add_scene_view(self.view)

    def poll(self):
        if not self.path.is_file(): return None
        if self.path.stat().st_size>MAX_BYTES: raise ValueError('overlay file limit')
        raw=self.path.read_bytes(); digest=hashlib.sha256(raw).hexdigest()
        if digest==self.digest: return None
        payload=validate_payload(json.loads(raw))
        session=payload.get('run_id')
        if session==self.session and payload['sequence']<=self.sequence: raise ValueError('overlay sequence not increasing')
        self.view.scene.clear()
        with self.view.scene:
            for name,path in payload['paths'].items():
                if len(path['points'])>=2:
                    # 仅显示抬高 .12m 以避免路面遮挡；原始 ROS xyz 完整保存在 JSON。
                    self.sc.Curve([[x,y,z+.12] for x,y,z in path['points']],curve_type=self.sc.Curve.CurveType.LINEAR,
                                  colors=[COLORS[name]],thicknesses=[4. if name=='actual_trace' else 3.])
        self.digest=digest; self.sequence=payload['sequence']; self.session=session
        return {'sequence':self.sequence,'paths':{k:len(v['points']) for k,v in payload['paths'].items()},'geometry':'UI_ONLY; RTX invisible'}
