"""原 A 区碰撞几何的 CPU 已知地图；不向 C/Actor 提供真值。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np

TERRAIN = '/World/Environment/DetailTerrain_Rockfield/DetailMesh_Rockfield'
SOURCE_SHA = 'e0573e650232211b6ef0c0281fe644028855a95f823eda7f9db9d51a9e5eb70e'
A_BOUNDS = [235.,335.,-315.,-215.]


def _clip(poly, axis, value, lower):
    result = []
    for a,b in zip(poly, poly[1:]+poly[:1]):
        ain, bin = (a[axis]>=value, b[axis]>=value) if lower else (a[axis]<=value,b[axis]<=value)
        if ain: result.append(a)
        if ain != bin: result.append(a+(b-a)*((value-a[axis])/(b[axis]-a[axis])))
    return result


def rasterize_triangles(triangles, bounds, resolution):
    """逐格裁剪真实三角；存最高绝对高程，保留格内 max-min 起伏。"""
    x0,x1,y0,y1 = bounds
    if resolution <= 0 or x1 <= x0 or y1 <= y0: raise ValueError('invalid raster bounds')
    nx,ny = round((x1-x0)/resolution),round((y1-y0)/resolution)
    high,low = np.full((ny,nx),-np.inf),np.full((ny,nx),np.inf)
    triangles = np.asarray(triangles,float)
    if triangles.ndim != 3 or triangles.shape[1:] != (3,3) or not np.isfinite(triangles).all(): raise ValueError('invalid triangles')
    for tri in triangles:
        lo,hi = tri.min(axis=0),tri.max(axis=0)
        if hi[0]<x0 or lo[0]>x1 or hi[1]<y0 or lo[1]>y1: continue
        ix0,ix1 = max(0,int(math.floor((lo[0]-x0)/resolution))),min(nx-1,int(math.floor((hi[0]-x0)/resolution)))
        iy0,iy1 = max(0,int(math.floor((lo[1]-y0)/resolution))),min(ny-1,int(math.floor((hi[1]-y0)/resolution)))
        for iy in range(iy0,iy1+1):
            for ix in range(ix0,ix1+1):
                poly = list(tri)
                for axis,value,lower in ((0,x0+ix*resolution,True),(0,x0+(ix+1)*resolution,False),(1,y0+iy*resolution,True),(1,y0+(iy+1)*resolution,False)):
                    if poly: poly = _clip(poly,axis,value,lower)
                area = 0.
                if len(poly)>=3:
                    origin=poly[0]
                    area=abs(sum((a[0]-origin[0])*(b[1]-origin[1])-(a[1]-origin[1])*(b[0]-origin[0]) for a,b in zip(poly[1:-1],poly[2:])))/2
                if area>1e-12:
                    z = [p[2] for p in poly]
                    high[iy,ix] = max(high[iy,ix],max(z)); low[iy,ix] = min(low[iy,ix],min(z))
    valid = np.isfinite(high)
    relief = np.full_like(high,np.nan); relief[valid] = high[valid]-low[valid]
    high[~valid] = np.nan
    return high.astype(np.float32),relief.astype(np.float32)


def select_route(elevation, relief, bounds, resolution):
    """选择 4m 直行候选，仅用于选起终点；真正路线由正式 Action 规划。"""
    elevation,relief = np.asarray(elevation),np.asarray(relief)
    if elevation.shape != relief.shape or min(elevation.shape)<40: raise ValueError('map must cover at least 8m')
    safe = np.isfinite(elevation)&np.isfinite(relief)&(relief<=.08)
    # wheel_orin 包络半径 .719m + 显式 .5m 安全余量；保守圆覆盖所有 yaw。
    radius = math.hypot(.591,.409)+.5
    cells = math.ceil(radius/resolution)
    good = safe.copy()
    for dy in range(-cells,cells+1):
        for dx in range(-cells,cells+1):
            if math.hypot(dx,dy)*resolution > radius+resolution: continue
            shifted = np.zeros_like(safe)
            ys,ye=max(0,-dy),min(len(safe),len(safe)-dy)
            xs,xe=max(0,-dx),min(safe.shape[1],safe.shape[1]-dx)
            shifted[ys:ye,xs:xe] = safe[ys+dy:ye+dy,xs+dx:xe+dx]
            good &= shifted
            # 包络内部最高/最低差限制，避免跨岩石/坑边与过大坡度。
            z = np.full_like(elevation,np.nan)
            z[ys:ye,xs:xe] = elevation[ys+dy:ye+dy,xs+dx:xe+dx]
            good &= np.abs(z-elevation) <= math.tan(math.radians(12))*max(resolution,math.hypot(dx,dy)*resolution)
    n = round(4/resolution)
    # 顺序按距地图中心排序，避免边缘落位。
    candidates = np.argwhere(good)
    candidates = sorted(candidates,key=lambda p: float(np.sum((p-np.array(good.shape)/2)**2)))
    for iy,ix in candidates:
        for dx,dy in ((1,0),(0,1),(-1,0),(0,-1)):
            xx,yy=ix+dx*n,iy+dy*n
            if 0<=xx<good.shape[1] and 0<=yy<good.shape[0] and all(good[iy+dy*k,ix+dx*k] for k in range(n+1)):
                x0,_,y0,_=bounds
                return [x0+(ix+.5)*resolution,y0+(iy+.5)*resolution],[x0+(xx+.5)*resolution,y0+(yy+.5)*resolution]
    raise ValueError('no 4m route with real terrain/rock footprint and .5m margin')


def collision_triangles(stage, bounds):
    from pxr import Usd,UsdGeom,UsdPhysics
    cache=UsdGeom.XformCache(); chunks=[]; audit=[]
    for prim in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh) or not str(prim.GetPath()).startswith('/World/Environment/'): continue
        collider=prim
        while collider and not collider.HasAPI(UsdPhysics.CollisionAPI): collider=collider.GetParent()
        if not collider or UsdPhysics.CollisionAPI(collider).GetCollisionEnabledAttr().Get() is False: continue
        mesh=UsdGeom.Mesh(prim); pts=np.asarray(mesh.GetPointsAttr().Get(),float)
        matrix=np.asarray(cache.GetLocalToWorldTransform(prim),float)
        pts=pts@matrix[:3,:3]+matrix[3,:3]
        low,high=pts.min(axis=0),pts.max(axis=0)
        x0,x1,y0,y1=bounds
        if high[0]<x0 or low[0]>x1 or high[1]<y0 or low[1]>y1: continue
        counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get(),int); ids=np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),int)
        triangles=[]; offset=0
        for count in counts:
            face=ids[offset:offset+count]; offset+=count
            for k in range(1,count-1): triangles.append([face[0],face[k],face[k+1]])
        tris=pts[np.asarray(triangles,int)]
        keep=(tris[:,:,0].max(axis=1)>=x0)&(tris[:,:,0].min(axis=1)<=x1)&(tris[:,:,1].max(axis=1)>=y0)&(tris[:,:,1].min(axis=1)<=y1)
        if np.any(keep):
            chunks.append(tris[keep]); audit.append(dict(path=str(prim.GetPath()),collider=str(collider.GetPath()),triangles=int(keep.sum())))
    if not chunks: raise ValueError('no actual collision triangles')
    return np.concatenate(chunks),audit


def build_scene(source_stage,output_dir,profile_path):
    from pxr import Usd,UsdGeom,UsdPhysics
    from tools.isaac_viper.build_stage import _placement,_transform
    from tools.isaac_viper.config import load_profile
    source_path=Path(source_stage).resolve()
    sha=hashlib.sha256(source_path.read_bytes()).hexdigest()
    if sha!=SOURCE_SHA: raise ValueError('source stage differs from frozen original A/B/C')
    output=Path(output_dir).resolve(); output.mkdir(parents=True,exist_ok=False)
    source=Usd.Stage.Open(str(source_path))
    from tools.isaac_viper.build_showcase import digest
    source_layers={str(Path(layer.realPath).resolve()):digest(layer.realPath) for layer in source.GetUsedLayers() if layer.realPath and Path(layer.realPath).is_file()}
    if not source.GetPrimAtPath(TERRAIN): raise ValueError('original A terrain missing')
    # 优先中心近场，不可通行时继续 A 内独立候选；每候选均实际重新裁剪。
    selected=None
    for cx,cy in ((285,-265),(265,-265),(305,-265),(285,-285),(285,-245),(255,-295),(315,-235)):
        bounds=[cx-8,cx+8,cy-8,cy+8]
        triangles,audit=collision_triangles(source,bounds)
        elevation,relief=rasterize_triangles(triangles,bounds,.2)
        try: start,goal=select_route(elevation,relief,bounds,.2)
        except ValueError: continue
        selected=(bounds,elevation,relief,start,goal,audit); break
    if selected is None: raise ValueError('A safe nearfield search exhausted; no fake fallback')
    bounds,elevation,relief,start,goal,audit=selected
    yaw=math.atan2(goal[1]-start[1],goal[0]-start[0])
    position,orientation,placement=_placement(source,load_profile(profile_path),TERRAIN,start,math.degrees(yaw))
    stage_path=output/'viper-navigation.usda'
    stage=Usd.Stage.CreateNew(str(stage_path)); stage.GetRootLayer().subLayerPaths=[str(source_path)]
    for key in ('metersPerUnit','upAxis','kilogramsPerUnit','timeCodesPerSecond','framesPerSecond','defaultPrim'):
        if source.HasAuthoredMetadata(key): stage.SetMetadata(key,source.GetMetadata(key))
    _transform(stage.OverridePrim('/World/ProjectVIPER'),position).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orientation)
    meta=json.loads(source.GetRootLayer().customLayerData['engineering_metadata_json'])
    meta.update(initial_position_m=position,initial_orientation_wxyz=[orientation.GetReal(),*orientation.GetImaginary()],placement=placement)
    meta['navigation_showcase']=dict(region='A',map_bounds_xy_m=bounds,start_xy_m=start,goal_xy_m=goal,goal_yaw_rad=yaw)
    data=dict(source.GetRootLayer().customLayerData); data['engineering_metadata_json']=json.dumps(meta,ensure_ascii=False)
    stage.GetRootLayer().customLayerData=data; stage.GetRootLayer().Save()
    np.savez_compressed(output/'map.npz',elevation=elevation,relief=relief,bounds_xy_m=bounds,resolution_m=.2)
    report=dict(map_sha256=digest(output/'map.npz'),stage_sha256=digest(stage_path),schema='lunar.viper-navigation-scene.v1',region='A',source_stage=str(source_path),source_sha256=sha,
        stage=str(stage_path),map_npz=str(output/'map.npz'),bounds_xy_m=bounds,resolution_m=.2,start_xy_m=start,goal_xy_m=goal,goal_yaw_rad=yaw,
        map_source='known_map_from_same_scene_collision_geometry; A only; forbidden for C/Actor',
        raster_method='exact triangle/cell polygon clipping; absolute max elevation; unknown NaN',
        elevation_approximation='conservative per-cell upper envelope, no terrain smoothing; spatial cell width .2m',
        source_layer_sha256=source_layers,
        elevation_cell_envelope_error_bound_m=dict(max=float(np.nanmax(relief)),p95=float(np.nanpercentile(relief,95)),
            meaning='per-cell max-minus-min over actual clipped triangles; conservative height quantization bound, including rocks'),
        collision_meshes=audit,collision_check=dict(footprint_half_extent_m=[.591,.409],safety_margin_m=.5,
        acceptance_slope_limit_deg=12,maximum_cell_relief_m=.08,route_length_m=math.dist(start,goal),
        static_corridor='PASS',dynamic_settle='NOT_RUN',collision_runtime='NOT_RUN'),placement=placement,
        NOT_RUN=['Kit','ROS','dynamic_settle','navigation','Orin','real_vehicle'])
    (output/'navigation-scene.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    if hashlib.sha256(source_path.read_bytes()).hexdigest()!=sha or any(digest(path)!=value for path,value in source_layers.items()): raise RuntimeError('source modified')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--source-stage',required=True); p.add_argument('--output-dir',required=True); p.add_argument('--profile',default='config/isaac_viper.yaml')
    a=p.parse_args(); print(json.dumps(build_scene(a.source_stage,a.output_dir,a.profile),ensure_ascii=False))

if __name__=='__main__': main()
