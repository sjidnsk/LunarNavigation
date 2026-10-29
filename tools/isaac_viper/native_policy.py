"""原生 GetPolicyMap 捕获快照的第二阶段fixture核验；不发布或绘制离线路线。"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np

INDEX_ORDER='array[y,x] ascending world axes; cell centers origin+(index+0.5)*resolution; policy indices0..499 exactly source grid'


def native_index_order(shape):
    ny,nx=shape
    suffix=f'policy indices0..{nx-1}' if nx==ny else f'policy indices x0..{nx-1}, y0..{ny-1}'
    return 'array[y,x] ascending world axes; cell centers origin+(index+0.5)*resolution; '+suffix+' exactly source grid'


def _digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_native_policy(manifest_path,profile,elevation,relief,bounds,resolution,source_layers,expected_profile_hash,expected_profile_sha256=None,region='A'):
    from tools.isaac_viper.known_regions import SOURCE_SHA,validate_region_map
    path=Path(manifest_path).resolve(); manifest=json.loads(path.read_text(encoding='utf-8'))
    schema=manifest.get('schema'); shape=tuple(np.asarray(elevation).shape)
    validate_region_map(region,list(bounds),resolution,shape)
    if schema=='native-policy-aligned500-v1':
        if region!='A' or manifest.get('region','A')!='A': raise ValueError('legacy aligned500 manifest is A only')
        expected_order=INDEX_ORDER
    elif schema=='native-policy-aligned-v2':
        if manifest.get('region')!=region: raise ValueError('native manifest region mismatch')
        expected_order=native_index_order(shape)
    else: raise ValueError('unknown native policy schema')
    if not isinstance(expected_profile_hash,str) or len(expected_profile_hash)!=16 or any(c not in '0123456789abcdef' for c in expected_profile_hash) or manifest.get('profile_hash')!=expected_profile_hash: raise ValueError('native profile hash mismatch; explicit runtime receipt hash required')
    if not isinstance(manifest.get('epoch'),str) or not manifest['epoch'] or any(type(manifest.get(k)) is not int or manifest[k]<1 for k in ('fine_revision','raw_elevation_revision')): raise ValueError('native epoch/revision unavailable')
    if manifest.get('wire')!={'unknown':0,'free':1,'blocked':2} or manifest.get('index_order')!=expected_order: raise ValueError('native wire/axis contract mismatch')
    response=path.parent/manifest['response_cdr']
    if response.stat().st_size>64*1024*1024 or _digest(response)!=manifest.get('response_sha256'): raise ValueError('native original service response SHA mismatch')
    if manifest.get('frame_id')!='map' or manifest.get('resolution_m')!=resolution or manifest.get('bounds_xy_m')!=list(bounds) or manifest.get('array_shape')!=list(shape) or manifest.get('origin')!=[bounds[0],bounds[2],0.]: raise ValueError('native manifest geometry mismatch')
    source_map=Path(manifest['source_map']); source_map=source_map if source_map.is_absolute() else path.parent/source_map
    native=path.parent/manifest['native_npz']
    if source_map.stat().st_size>32*1024*1024 or _digest(source_map)!=manifest.get('source_map_sha256'): raise ValueError('native source map SHA mismatch')
    if native.stat().st_size>32*1024*1024 or _digest(native)!=manifest.get('native_sha256'): raise ValueError('native snapshot SHA mismatch')
    receipt_path=source_map.with_name('navigation-scene.json'); receipt=json.loads(receipt_path.read_text(encoding='utf-8'))
    if receipt.get('schema')!='lunar.viper-navigation-scene.v1' or receipt.get('region')!=region or receipt.get('map_sha256')!=manifest['source_map_sha256'] or receipt.get('source_sha256')!=SOURCE_SHA or receipt.get('source_layer_sha256')!=source_layers: raise ValueError('native source scene geometry receipt mismatch')
    validate_region_map(receipt.get('region'),receipt.get('bounds_xy_m'),receipt.get('resolution_m'),shape)
    if receipt.get('capability_limits')!=profile['capability']: raise ValueError('native source capability semantics mismatch')
    if expected_profile_sha256 is not None and receipt.get('profile_sha256')!=expected_profile_sha256: raise ValueError('native source profile receipt SHA mismatch')
    with np.load(source_map,allow_pickle=False) as source:
        if source['bounds_xy_m'].tolist()!=list(bounds) or float(source['resolution_m'])!=resolution: raise ValueError('native source map geometry mismatch')
        if not np.array_equal(source['elevation'],elevation,equal_nan=True): raise ValueError('native source elevation differs from current real collision raster')
        if not np.array_equal(source['relief'],relief,equal_nan=True): raise ValueError('native source relief differs from current real collision raster')
    with np.load(native,allow_pickle=False) as arrays:
        if arrays['bounds_xy_m'].tolist()!=list(bounds) or float(arrays['resolution_m'])!=resolution or arrays['origin_xy'].tolist()!=[bounds[0],bounds[2]]: raise ValueError('native NPZ geometry mismatch')
        if not np.array_equal(arrays['elevation_m'],elevation,equal_nan=True): raise ValueError('native elevation differs from source/current collision raster')
        if not np.array_equal(arrays['source_relief_m'],relief,equal_nan=True): raise ValueError('native source relief mismatch')
        states={key:arrays[key].copy() for key in ('states','intrinsic_states','observed')}
    if any(value.shape!=shape or value.dtype!=np.uint8 or np.any(value>2) for value in states.values()): raise ValueError('native state shape/type/value mismatch')
    free=np.isfinite(elevation)
    blocked=np.isfinite(elevation)
    for value in states.values(): free=free&(value==1); blocked=blocked&(value==2)
    audit=dict(region=region,manifest_schema=schema,manifest=str(path),manifest_sha256=_digest(path),snapshot=str(native),snapshot_sha256=manifest['native_sha256'],
        source_map=str(source_map),source_map_sha256=manifest['source_map_sha256'],source_scene_receipt=str(receipt_path),source_scene_receipt_sha256=_digest(receipt_path),response_cdr=str(response),response_cdr_sha256=manifest['response_sha256'],
        response_verification='original captured CDR file SHA checked; not deserialized/reissued as live ROS evidence',
        epoch=manifest['epoch'],fine_revision=manifest['fine_revision'],raw_elevation_revision=manifest['raw_elevation_revision'],profile_hash=manifest['profile_hash'],
        expected_runtime_profile_hash=expected_profile_hash,profile_binding='explicit runtime hash matched; source scene capability deep-equal; native C++ hash not recomputed',
        source_geometry_layers_exact=True,source_and_native_elevation_equal_current_collision_raster=True,source_relief_equal=True,
        free_rule='states == intrinsic_states == observed == wire FREE(1); UNKNOWN never assumed free',free_cells=int(free.sum()),
        offline_only=True,live_endpoint_revalidation='required on new run; captured policy is not a live goal/settling certificate')
    return dict(free=free,blocked=blocked,states=states,audit=audit)


def select_native_route(policy,elevation,relief,bounds,resolution,start_xy,goal_xy,minimum_length_m,capability):
    from tools.isaac_viper.navigation_scene import conservative_footprint_mask,_offline_distance
    if not math.isfinite(minimum_length_m) or minimum_length_m<=0: raise ValueError('invalid route length requirement')
    good=policy['free']; ny,nx=good.shape
    def index(xy):
        if xy is None or len(xy)!=2 or not all(math.isfinite(v) for v in xy): raise ValueError('explicit finite native endpoints required')
        x=(float(xy[0])-bounds[0])/resolution-.5; y=(float(xy[1])-bounds[2])/resolution-.5
        ix,iy=round(x),round(y)
        if abs(x-ix)>1e-6 or abs(y-iy)>1e-6 or not (0<=ix<nx and 0<=iy<ny): raise ValueError('native endpoint must be an in-bounds cell center')
        if not good[iy,ix]: raise ValueError('native endpoint requires observed/intrinsic/inflated all FREE')
        return iy,ix
    start,goal=index(start_xy),index(goal_xy)
    slope=math.degrees(capability['maximum_slope_rad']); maximum_relief=capability['maximum_local_obstacle_relief_m']
    physical,_=conservative_footprint_mask(elevation,relief,resolution,slope,maximum_relief)
    if not physical[start] or not physical[goal]: raise ValueError('native endpoint failed additional physical footprint candidate check')
    length=_offline_distance(good,start,goal,resolution)
    if length is None: raise ValueError('native endpoints are not connected without corner cutting')
    if length<minimum_length_m: raise ValueError('native connected fixture length below requirement')
    line=np.rint(np.linspace(start,goal,max(2,math.ceil(math.dist(start,goal)*4)+1))).astype(int)
    blockers=[tuple(map(int,p)) for p in line if policy['blocked'][tuple(p)]]
    if not blockers: raise ValueError('native direct line has no finite observed/intrinsic/inflated BLOCKED actual terrain cell')
    blocker=max(blockers,key=lambda p:float(relief[p]))
    def xy(p): return [float(bounds[0]+(p[1]+.5)*resolution),float(bounds[2]+(p[0]+.5)*resolution)]
    checks={}
    for name,p in (('start',start),('goal',goal)):
        checks[name]=dict(xy_m=xy(p),elevation_m=float(elevation[p]),footprint_clear=True,
            footprint_half_extent_m=[.591,.409],clearance_margin_m=.5,slope_limit_deg=slope,maximum_cell_relief_m=maximum_relief,
            native_states={key:int(value[p]) for key,value in policy['states'].items()},physical_candidate_mask=True,
            acceptance='captured native observed/intrinsic/inflated FREE plus physical candidate footprint; new-run settle NOT_RUN')
    proof=dict(offline_only=True,connectivity='PASS',path_length_m=float(length),net_displacement_m=math.dist(start_xy,goal_xy),
        minimum_required_m=float(minimum_length_m),line_of_sight_blocked=True,blocked_cell_xy_m=xy(blocker),
        blocked_cell_native_states={key:int(value[blocker]) for key,value in policy['states'].items()},
        blocking_evidence='actual finite source terrain; captured native observed/intrinsic/inflated BLOCKED; exact per-mesh collision attribution follows',
        endpoint_checks=checks,native_endpoint_validation='PASS',native_policy=policy['audit'],
        threshold_source='captured native policy; no navigator threshold changed',
        method='offline A* on captured native all-FREE intersection, no corner cutting; fixture only, never published/drawn as navigator output')
    return xy(start),xy(goal),proof
