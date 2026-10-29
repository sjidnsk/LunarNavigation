"""冻结原场景固定区域的 CPU 已知地图；不向 C/Actor 提供真值。"""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path
import numpy as np

from tools.isaac_viper.known_regions import SOURCE_SHA,REGION_NAMES,region_spec,validate_region_map
TERRAIN = region_spec('A')['terrain_path']
A_BOUNDS = region_spec('A')['bounds_xy_m']


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


def rasterize_triangles_batched(triangles, bounds, resolution, batch_size=32768):
    """批量精确三角/格裁剪；与小图参考算法相同的最高高程与格内起伏。"""
    x0,x1,y0,y1=map(float,bounds)
    if resolution<=0 or x1<=x0 or y1<=y0: raise ValueError('invalid raster bounds')
    nx,ny=round((x1-x0)/resolution),round((y1-y0)/resolution)
    if nx*ny>1000000 or nx<1 or ny<1: raise ValueError('raster size limit')
    triangles=np.asarray(triangles,float)
    if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not np.isfinite(triangles).all(): raise ValueError('invalid triangles')
    high,low=np.full(nx*ny,-np.inf),np.full(nx*ny,np.inf)
    # 每块先做真实XY包围盒 broad phase，后续四次裁剪全部在NumPy批量进行。
    for base in range(0,len(triangles),2048):
        tris=triangles[base:base+2048]
        lo=tris[:,:,:2].min(axis=1); hi=tris[:,:,:2].max(axis=1)
        ix0=np.maximum(0,np.floor((lo[:,0]-x0)/resolution).astype(int)); ix1=np.minimum(nx-1,np.ceil((hi[:,0]-x0)/resolution).astype(int)-1)
        iy0=np.maximum(0,np.floor((lo[:,1]-y0)/resolution).astype(int)); iy1=np.minimum(ny-1,np.ceil((hi[:,1]-y0)/resolution).astype(int)-1)
        widths=np.maximum(0,ix1-ix0+1); counts=widths*np.maximum(0,iy1-iy0+1)
        cumulative=np.cumsum(counts); total=int(cumulative[-1])
        for first in range(0,total,batch_size):
            flat=np.arange(first,min(total,first+batch_size)); ids=np.searchsorted(cumulative,flat,side='right')
            offset=flat-(cumulative[ids]-counts[ids]); ix=ix0[ids]+offset%widths[ids]; iy=iy0[ids]+offset//widths[ids]
            polys=np.zeros((len(ids),8,3)); polys[:,:3]=tris[ids]; n=np.full(len(ids),3,int)
            for axis,values,lower in ((0,x0+ix*resolution,True),(0,x0+(ix+1)*resolution,False),(1,y0+iy*resolution,True),(1,y0+(iy+1)*resolution,False)):
                out=np.zeros_like(polys); out_n=np.zeros_like(n); rows=np.arange(len(ids))
                for k in range(7):
                    active=k<n; a=polys[:,k]; b=polys[rows,np.where(active,(k+1)%np.maximum(n,1),0)]
                    ain=a[:,axis]>=values if lower else a[:,axis]<=values
                    bin_=b[:,axis]>=values if lower else b[:,axis]<=values
                    keep=active&ain; rr=rows[keep]; out[rr,out_n[keep]]=a[keep]; out_n[keep]+=1
                    cross=active&(ain!=bin_); rr=rows[cross]
                    if len(rr):
                        t=(values[cross]-a[cross,axis])/(b[cross,axis]-a[cross,axis])
                        out[rr,out_n[cross]]=a[cross]+(b[cross]-a[cross])*t[:,None]; out_n[cross]+=1
                polys,n=out,out_n
            area=np.zeros(len(ids)); zhigh=np.full(len(ids),-np.inf); zlow=np.full(len(ids),np.inf); origin=polys[:,0]
            for k in range(8):
                active=k<n; nxt=polys[np.arange(len(ids)),np.where(active,(k+1)%np.maximum(n,1),0)]
                area+=np.where(active,(polys[:,k,0]-origin[:,0])*(nxt[:,1]-origin[:,1])-(polys[:,k,1]-origin[:,1])*(nxt[:,0]-origin[:,0]),0)
                zhigh=np.maximum(zhigh,np.where(active,polys[:,k,2],-np.inf)); zlow=np.minimum(zlow,np.where(active,polys[:,k,2],np.inf))
            valid=(n>=3)&(np.abs(area)>2e-12); cell=iy[valid]*nx+ix[valid]
            np.maximum.at(high,cell,zhigh[valid]); np.minimum.at(low,cell,zlow[valid])
    valid=np.isfinite(high); relief=np.full_like(high,np.nan); relief[valid]=high[valid]-low[valid]; high[~valid]=np.nan
    return high.reshape(ny,nx).astype(np.float32),relief.reshape(ny,nx).astype(np.float32)


def conservative_footprint_mask(elevation,relief,resolution,slope_limit_deg=12.,maximum_relief_m=.08):
    """车体外接圆+.5m校验全部yaw；实际构建从权威capability传入坡度/起伏门。"""
    elevation,relief=np.asarray(elevation),np.asarray(relief)
    if elevation.ndim!=2 or elevation.shape!=relief.shape or resolution<=0: raise ValueError('invalid footprint map')
    safe=np.isfinite(elevation)&np.isfinite(relief)&(relief<=maximum_relief_m)
    radius=math.hypot(.591,.409)+.5; cells=math.ceil(radius/resolution); good=safe.copy()
    for dy in range(-cells,cells+1):
        for dx in range(-cells,cells+1):
            distance=math.hypot(dx,dy)*resolution
            if distance>radius+resolution: continue
            ys,ye=max(0,-dy),min(len(safe),len(safe)-dy); xs,xe=max(0,-dx),min(safe.shape[1],safe.shape[1]-dx)
            shifted=np.zeros_like(safe); shifted[ys:ye,xs:xe]=safe[ys+dy:ye+dy,xs+dx:xe+dx]
            z=np.full_like(elevation,np.nan); z[ys:ye,xs:xe]=elevation[ys+dy:ye+dy,xs+dx:xe+dx]
            good &= shifted & (np.abs(z-elevation)<=math.tan(math.radians(slope_limit_deg))*max(resolution,distance))
    return good,safe


def _offline_distance(good,start,goal,resolution):
    """八邻域A*，禁止切过障碍角；只返回可达长度，不返回/发布规划折线。"""
    import heapq
    queue=[(math.dist(start,goal)*resolution,0.,start)]; costs={start:0.}
    ny,nx=good.shape
    while queue:
        _,cost,(y,x)=heapq.heappop(queue)
        if cost>costs.get((y,x),math.inf): continue
        if (y,x)==goal: return cost
        for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
            yy,xx=y+dy,x+dx
            if not (0<=yy<ny and 0<=xx<nx and good[yy,xx]): continue
            if dx and dy and not (good[y,xx] and good[yy,x]): continue
            value=cost+resolution*math.hypot(dx,dy)
            if value<costs.get((yy,xx),math.inf):
                costs[(yy,xx)]=value; heapq.heappush(queue,(value+math.hypot(goal[0]-yy,goal[1]-xx)*resolution,value,(yy,xx)))
    return None


def select_long_route(elevation,relief,bounds,resolution,minimum_length_m=100.,capability=None):
    """仅为真正导航选fixture：要求实际有限障碍阻断直线且保守绕行长度足够。"""
    if not math.isfinite(minimum_length_m) or minimum_length_m<=0: raise ValueError('invalid minimum route length')
    slope_limit_deg=12. if capability is None else math.degrees(float(capability['maximum_slope_rad']))
    maximum_relief_m=.08 if capability is None else float(capability['maximum_local_obstacle_relief_m'])
    good,safe=conservative_footprint_mask(elevation,relief,resolution,slope_limit_deg,maximum_relief_m)
    candidates=np.argwhere(good)
    if not len(candidates): raise ValueError('no safe endpoints on actual collision terrain')
    # 16方向边界点与其附近分位点，确定有界搜索，不随机改变fixture身份。
    selected=set()
    for angle in np.linspace(0,2*math.pi,16,endpoint=False):
        order=np.argsort(candidates[:,0]*math.sin(angle)+candidates[:,1]*math.cos(angle))
        for fraction in (0.,.02,.10): selected.add(tuple(map(int,candidates[order[min(len(order)-1,int(fraction*len(order)))]])))
    # 无切角8邻域与4邻域连通域相同；先分量过滤，避免反复搜不可达端点。
    from collections import deque
    labels=np.zeros(good.shape,np.int32); label=0
    for sy,sx in np.argwhere(good):
        if labels[sy,sx]: continue
        label+=1; labels[sy,sx]=label; queue=deque([(int(sy),int(sx))])
        while queue:
            y,x=queue.popleft()
            for yy,xx in ((y+1,x),(y-1,x),(y,x+1),(y,x-1)):
                if 0<=yy<good.shape[0] and 0<=xx<good.shape[1] and good[yy,xx] and not labels[yy,xx]:
                    labels[yy,xx]=label; queue.append((yy,xx))
    pairs=sorted(((math.dist(a,b)*resolution,a,b) for a in selected for b in selected if a<b and labels[a]==labels[b]),reverse=True)
    attempts=0
    for direct,start,goal in pairs:
        if direct<minimum_length_m*.65: continue
        steps=max(1,math.ceil(math.dist(start,goal)*3)); line=np.rint(np.linspace(start,goal,steps+1)).astype(int)
        # 未知/单纯车体margin不作为真实障碍来源；必须命中有高度的不可通行格。
        blocked=[tuple(map(int,p)) for p in line if not safe[tuple(p)] and np.isfinite(elevation[tuple(p)])]
        if not blocked: continue
        attempts+=1
        if attempts>96: break
        length=_offline_distance(good,start,goal,resolution)
        if length is None or length<minimum_length_m: continue
        def xy(p): return [float(bounds[0]+(p[1]+.5)*resolution),float(bounds[2]+(p[0]+.5)*resolution)]
        checks={}
        for name,p in (('start',start),('goal',goal)):
            checks[name]=dict(xy_m=xy(p),elevation_m=float(elevation[p]),footprint_clear=True,
                footprint_half_extent_m=[.591,.409],clearance_margin_m=.5,slope_limit_deg=slope_limit_deg,maximum_cell_relief_m=maximum_relief_m,
                acceptance='conservative all-yaw footprint mask on actual collision geometry; PhysX settle NOT_RUN')
        return xy(start),xy(goal),dict(offline_only=True,connectivity='PASS',path_length_m=float(length),net_displacement_m=float(direct),
            minimum_required_m=float(minimum_length_m),line_of_sight_blocked=True,blocked_cell_xy_m=xy(blocked[len(blocked)//2]),
            blocking_evidence='finite actual collision cell exceeds capability clipped-height relief threshold; unknown is never an obstacle substitute',
            threshold_source='explicit capability maximum_slope_rad / maximum_local_obstacle_relief_m' if capability is not None else 'legacy Task4 conservative candidate gates',
            endpoint_checks=checks,search_attempts=attempts,method='offline 8-neighbour A*; no corner cutting; fixture reachability only, never published/drawn as navigator output')
    raise ValueError('Fixed known-region search found no connected real-obstacle detour of required length; no synthetic fallback')


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
        if np.all(counts==3):
            triangles=ids.reshape(-1,3)
        else:
            triangles=[]; offset=0
            for count in counts:
                face=ids[offset:offset+count]; offset+=count
                for k in range(1,count-1): triangles.append([face[0],face[k],face[k+1]])
            triangles=np.asarray(triangles,int)
        tris=pts[triangles]
        keep=(tris[:,:,0].max(axis=1)>=x0)&(tris[:,:,0].min(axis=1)<=x1)&(tris[:,:,1].max(axis=1)>=y0)&(tris[:,:,1].min(axis=1)<=y1)
        if np.any(keep):
            chunks.append(tris[keep]); audit.append(dict(path=str(prim.GetPath()),collider=str(collider.GetPath()),triangles=int(keep.sum())))
    if not chunks: raise ValueError('no actual collision triangles')
    return np.concatenate(chunks),audit


def _write_map_only_scene(output,source_path,source_layers,profile_path,profile,spec,elevation,relief,audit,raster_wall_s,minimum_length,reason):
    """phase1只图回执：不生成PoseStage、不补造端点/路线/原生成功。"""
    from tools.isaac_viper.build_showcase import digest
    bounds=spec['bounds_xy_m']; output=Path(output)
    np.savez_compressed(output/'map.npz',elevation=elevation,relief=relief,bounds_xy_m=bounds,resolution_m=.2)
    cap_path=Path(profile_path).resolve().parent/profile['sources']['capability']
    receipt=dict(schema='lunar.viper-navigation-scene.v1',region=spec['region'],phase='MAP_ONLY',simulation_only=True,
        source_stage=str(source_path),source_sha256=SOURCE_SHA,source_layer_sha256=source_layers,
        stage=None,stage_sha256=None,map_npz=str(output/'map.npz'),map_sha256=digest(output/'map.npz'),bounds_xy_m=bounds,resolution_m=.2,
        placement_terrain_path=spec['terrain_path'],start_xy_m=None,goal_xy_m=None,goal_yaw_rad=None,
        map_source=spec['usage'],raster_method='NumPy batched exact triangle/cell polygon clipping; absolute max elevation; unknown NaN',
        raster_wall_s=raster_wall_s,collision_meshes=audit,
        capability_profile=str(Path(profile_path).resolve()),capability_path=str(cap_path),capability_sha256=digest(cap_path),
        profile_sha256=digest(profile_path),capability_limits=profile['capability'],
        fixture_precheck=dict(offline_only=True,connectivity='NOT_RUN',native_endpoint_validation='NOT_RUN',path_length_m=None,
            minimum_required_m=float(minimum_length),line_of_sight_blocked=None,selection_status=reason),
        NOT_RUN=['pose_stage','native_endpoint_validation','Kit','ROS','navigation','dynamic_settle','Orin','real_vehicle','C Actor'])
    (output/'navigation-scene.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    return receipt


def build_scene(source_stage,output_dir,profile_path,minimum_route_length_m=100.,native_policy_json=None,start_xy_m=None,goal_xy_m=None,native_profile_hash=None,region='A',map_only=False):
    from pxr import Usd,UsdGeom,UsdPhysics
    from tools.isaac_viper.build_stage import _placement,_transform
    from tools.isaac_viper.config import load_profile
    spec=region_spec(region); terrain_path=spec['terrain_path']
    if not math.isfinite(minimum_route_length_m) or minimum_route_length_m<=0: raise ValueError('minimum route length must be finite positive')
    if map_only and native_policy_json is not None: raise ValueError('map-only phase cannot consume native fixture endpoints')
    source_path=Path(source_stage).resolve()
    sha=hashlib.sha256(source_path.read_bytes()).hexdigest()
    if sha!=SOURCE_SHA: raise ValueError('source stage differs from frozen original A/B/C')
    output=Path(output_dir).resolve(); output.mkdir(parents=True,exist_ok=False)
    source=Usd.Stage.Open(str(source_path))
    from tools.isaac_viper.build_showcase import digest
    source_layers={str(Path(layer.realPath).resolve()):digest(layer.realPath) for layer in source.GetUsedLayers() if layer.realPath and Path(layer.realPath).is_file()}
    if not source.GetPrimAtPath(terrain_path).IsA(UsdGeom.Mesh): raise ValueError('fixed region original terrain Mesh missing')
    profile=load_profile(profile_path)
    if profile.get('simulation_only') is not True: raise ValueError('navigation demo requires an explicit simulation_only profile')
    if profile['capability']['maximum_forward_speed_mps']!=2. or profile['capability']['maximum_spin_rate_radps']!=2.: raise ValueError('navigation demo capability must be forward2/spin2')
    # 只用两个固定已知图演示区；没有任意bounds或C/Actor真值入口。
    bounds=spec['bounds_xy_m']; validate_region_map(region,bounds,.2); started=time.perf_counter()
    triangles,audit=collision_triangles(source,bounds)
    elevation,relief=rasterize_triangles_batched(triangles,bounds,.2)
    raster_wall_s=time.perf_counter()-started
    if not any(mesh['path']==terrain_path for mesh in audit): raise ValueError('fixed placement terrain is not an enabled collision source in this ROI')
    def map_receipt(reason):
        receipt=_write_map_only_scene(output,source_path,source_layers,profile_path,profile,spec,elevation,relief,audit,raster_wall_s,minimum_route_length_m,reason)
        if digest(source_path)!=sha or any(digest(path)!=value for path,value in source_layers.items()): raise RuntimeError('source modified')
        return receipt
    if map_only: return map_receipt('explicit map-only phase; approximate/native fixture NOT_RUN')
    if native_policy_json is not None:
        from tools.isaac_viper.native_policy import load_native_policy,select_native_route
        policy=load_native_policy(native_policy_json,profile,elevation,relief,bounds,.2,source_layers,native_profile_hash,digest(profile_path),region=region)
        start,goal,fixture=select_native_route(policy,elevation,relief,bounds,.2,start_xy_m,goal_xy_m,minimum_route_length_m,profile['capability'])
    else:
        if any(value is not None for value in (start_xy_m,goal_xy_m,native_profile_hash)): raise ValueError('explicit endpoints/hash require a native policy snapshot')
        try: start,goal,fixture=select_long_route(elevation,relief,bounds,.2,minimum_route_length_m,profile['capability'])
        except ValueError as exc: return map_receipt('approximate endpoint search failed: '+str(exc))
        fixture['native_endpoint_validation']='NOT_RUN; approximate candidate only, not native endpoint acceptance'
    blocker=fixture['blocked_cell_xy_m']; collision_sources=[]
    # 根据真实三角XY包围盒列出阻挡格的候选来源；精确高度/relief来自裁剪图。
    offset=0
    for mesh in audit:
        group=triangles[offset:offset+mesh['triangles']]; offset+=mesh['triangles']
        lo=group[:,:,:2].min(axis=1); hi=group[:,:,:2].max(axis=1)
        hit=(lo[:,0]<=blocker[0]+.1)&(hi[:,0]>=blocker[0]-.1)&(lo[:,1]<=blocker[1]+.1)&(hi[:,1]>=blocker[1]-.1)
        if np.any(hit):
            cell_bounds=[blocker[0]-.1,blocker[0]+.1,blocker[1]-.1,blocker[1]+.1]
            mesh_z,mesh_relief=rasterize_triangles_batched(group[hit],cell_bounds,.2)
            if np.isfinite(mesh_z[0,0]): collision_sources.append(dict(path=mesh['path'],collider=mesh['collider'],
                clipped_cell_max_elevation_m=float(mesh_z[0,0]),clipped_cell_relief_m=float(mesh_relief[0,0]),
                attribution='actual positive-area triangle/cell intersection; no bbox-only obstacle claim'))
    fixture['obstacle_collision_sources']=collision_sources
    yaw=math.atan2(goal[1]-start[1],goal[0]-start[0])
    position,orientation,placement=_placement(source,profile,terrain_path,start,math.degrees(yaw))
    stage_path=output/'viper-navigation.usda'
    stage=Usd.Stage.CreateNew(str(stage_path)); stage.GetRootLayer().subLayerPaths=[str(source_path)]
    for key in ('metersPerUnit','upAxis','kilogramsPerUnit','timeCodesPerSecond','framesPerSecond','defaultPrim'):
        if source.HasAuthoredMetadata(key): stage.SetMetadata(key,source.GetMetadata(key))
    _transform(stage.OverridePrim('/World/ProjectVIPER'),position).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orientation)
    meta=json.loads(source.GetRootLayer().customLayerData['engineering_metadata_json'])
    meta.update(initial_position_m=position,initial_orientation_wxyz=[orientation.GetReal(),*orientation.GetImaginary()],placement=placement)
    meta['navigation_showcase']=dict(region=region,map_bounds_xy_m=bounds,map_height_range_m=[float(np.nanmin(elevation)),float(np.nanmax(elevation))],start_xy_m=start,goal_xy_m=goal,goal_yaw_rad=yaw,fixture_precheck=fixture,simulation_only=True)
    data=dict(source.GetRootLayer().customLayerData); data['engineering_metadata_json']=json.dumps(meta,ensure_ascii=False)
    stage.GetRootLayer().customLayerData=data; stage.GetRootLayer().Save()
    np.savez_compressed(output/'map.npz',elevation=elevation,relief=relief,bounds_xy_m=bounds,resolution_m=.2)
    report=dict(map_sha256=digest(output/'map.npz'),stage_sha256=digest(stage_path),schema='lunar.viper-navigation-scene.v1',region=region,phase='NATIVE_FIXTURE' if native_policy_json else 'APPROXIMATE_FIXTURE',source_stage=str(source_path),source_sha256=sha,
        stage=str(stage_path),map_npz=str(output/'map.npz'),bounds_xy_m=bounds,resolution_m=.2,start_xy_m=start,goal_xy_m=goal,goal_yaw_rad=yaw,
        map_source=spec['usage'],placement_terrain_path=terrain_path,
        raster_method='NumPy batched exact triangle/cell polygon clipping; absolute max elevation; unknown NaN',
        raster_wall_s=raster_wall_s,fixture_precheck=fixture,simulation_only=True,
        capability_profile=str(Path(profile_path).resolve()),
        capability_path=str((Path(profile_path).resolve().parent/profile['sources']['capability']).resolve()),
        capability_sha256=digest(Path(profile_path).resolve().parent/profile['sources']['capability']),
        profile_sha256=digest(profile_path),capability_limits=profile['capability'],
        elevation_approximation='conservative per-cell upper envelope, no terrain smoothing; spatial cell width .2m',
        source_layer_sha256=source_layers,
        elevation_cell_envelope_error_bound_m=dict(max=float(np.nanmax(relief)),p95=float(np.nanpercentile(relief,95)),
            meaning='per-cell max-minus-min over actual clipped triangles; conservative height quantization bound, including rocks'),
        collision_meshes=audit,collision_check=dict(footprint_half_extent_m=[.591,.409],safety_margin_m=.5,
        acceptance_slope_limit_deg=math.degrees(profile['capability']['maximum_slope_rad']),maximum_cell_relief_m=profile['capability']['maximum_local_obstacle_relief_m'],offline_fixture_route_length_m=fixture['path_length_m'],net_displacement_m=math.dist(start,goal),
        static_corridor='captured native FREE corridor' if native_policy_json else 'approximate physical candidate only',native_endpoint_validation=fixture['native_endpoint_validation'],dynamic_settle='NOT_RUN',collision_runtime='NOT_RUN'),placement=placement,
        NOT_RUN=['Kit','ROS','dynamic_settle','navigation','Orin','real_vehicle'])
    (output/'navigation-scene.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    if hashlib.sha256(source_path.read_bytes()).hexdigest()!=sha or any(digest(path)!=value for path,value in source_layers.items()): raise RuntimeError('source modified')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--source-stage',required=True); p.add_argument('--output-dir',required=True); p.add_argument('--profile',default='config/isaac_viper_navigation_demo.yaml')
    p.add_argument('--region',choices=REGION_NAMES,default='A',help='固定已知地图演示区；KNOWN_DEMO_FLAT不是C探索')
    p.add_argument('--map-only',action='store_true',help='phase1只生成同源图/CPU回执，不生成PoseStage或端点')
    p.add_argument('--minimum-route-length-m',type=float,default=100.,help='仅离线fixture长度门；正式成功仍要求真实新导航规划>=此长度')
    p.add_argument('--native-policy-json',help='第二阶段：同源实际GetPolicyMap快照manifest；不启动导航器')
    p.add_argument('--native-policy-profile-hash',help='运行回执中的期望原生profile_hash，非YAML SHA')
    p.add_argument('--start-xy-m',type=float,nargs=2,metavar=('X','Y'))
    p.add_argument('--goal-xy-m',type=float,nargs=2,metavar=('X','Y'))
    a=p.parse_args()
    native_values=(a.native_policy_json,a.native_policy_profile_hash,a.start_xy_m,a.goal_xy_m)
    if any(value is not None for value in native_values) and not all(value is not None for value in native_values): p.error('native policy manifest, runtime profile hash and both cell-center endpoints are required together')
    if a.map_only and a.native_policy_json: p.error('--map-only and native endpoints are mutually exclusive')
    print(json.dumps(build_scene(a.source_stage,a.output_dir,a.profile,a.minimum_route_length_m,a.native_policy_json,a.start_xy_m,a.goal_xy_m,a.native_policy_profile_hash,a.region,a.map_only),ensure_ascii=False))

if __name__=='__main__': main()
