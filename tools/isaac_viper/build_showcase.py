"""只读源资产，构建新几何身份的 VIPER 三区展示；导入不需要 PXR/GPU。"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

BOUNDS = [-500, 500, -500, 500]
ENV = '/World/ShowcaseEnvironment'
TERRAIN = ENV + '/Terrain'
VIEWS = '/World/ShowcaseViews'


def quality_spec(settings):
    """只由受支持的版本推导拓扑，不能让回执声明任意数量放宽运行门。"""
    if settings.get('schema') != 'lunar.viper-showcase-quality.v1':
        raise ValueError('quality schema unsupported')
    version = settings.get('geometry_version')
    if version == 'original-source-v1':
        if settings.get('quality') != 'original':
            raise ValueError('quality/version mismatch')
        return {'terrain_points':None, 'terrain_triangles':None, 'smooth_terrain':False,
                'rock_points':None, 'rock_triangles':None}
    versions = {'uniform1m-rock80-v1': (1., 80, 42, 128, False),
                'uniform0p5m-rock320-v2': (.5, 320, 162, 512, True)}
    if version not in versions:
        raise ValueError('quality geometry_version unsupported')
    spacing, triangles, vertices, maximum, smooth = versions[version]
    if (settings.get('quality') != 'light' or settings.get('terrain_spacing_m') != spacing or
            settings.get('rock_max_triangles') != maximum or
            settings.get('rock_triangles_each', triangles) != triangles):
        raise ValueError('quality settings/version mismatch')
    nx, ny = round((BOUNDS[1]-BOUNDS[0])/spacing)+1, round((BOUNDS[3]-BOUNDS[2])/spacing)+1
    return {'terrain_points':nx*ny, 'terrain_triangles':2*(nx-1)*(ny-1),
            'terrain_shape':(ny, nx), 'smooth_terrain':smooth,
            'rock_points':vertices, 'rock_triangles':triangles}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def validate_regions(cfg):
    if cfg.get('schema') != 'lunar.isaac-polar-midterm-regions.v1':
        raise ValueError('unsupported region schema')
    if cfg['coordinate_system']['scene_bounds_xy_m'] != BOUNDS:
        raise ValueError('world bounds changed')
    if [r['id'] for r in cfg['regions']] != ['A', 'B', 'C']:
        raise ValueError('expected A/B/C regions')
    for r in cfg['regions']:
        x0, x1, y0, y1 = r['bounds_xy_m']
        if not (-500 <= x0 < x1 <= 500 and -500 <= y0 < y1 <= 500):
            raise ValueError('invalid region bounds')
        expected_polygon = [[x0,y0],[x1,y0],[x1,y1],[x0,y1]]
        if (r['polygon_xy_m'] != expected_polygon or r['size_xy_m'] != [x1-x0,y1-y0] or
                r['area_m2'] != (x1-x0)*(y1-y0)):
            raise ValueError('region polygon/area/size mismatch')
        if r['start_pose'] is not None or r['goal_pose'] is not None:
            raise ValueError('unverified start/goal must remain null')
    for index,a in enumerate(cfg['regions']):
        a0,a1,a2,a3 = a['bounds_xy_m']
        for b in cfg['regions'][index+1:]:
            b0,b1,b2,b3 = b['bounds_xy_m']
            if not (a1 <= b0 or b1 <= a0 or a3 <= b2 or b3 <= a2):
                raise ValueError('regions overlap')
    c = cfg['regions'][2]
    x, y = c['spawn_candidate_xy_m']
    x0, x1, y0, y1 = c['bounds_xy_m']
    if not (x0 < x < x1 and y0 < y < y1) or c.get('hidden_truth_to_actor') is not False:
        raise ValueError('invalid C candidate or actor truth policy')
    return cfg


def load_regions(path):
    return validate_regions(json.loads(Path(path).read_text(encoding='utf-8')))


def _grid_coordinates(array, bounds, x, y):
    x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
    x0, x1, y0, y1 = bounds
    if (not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)) or
            np.any(x < x0) or np.any(x > x1) or np.any(y < y0) or np.any(y > y1)):
        raise ValueError('sample outside terrain bounds')
    fx = (x-x0)/(x1-x0)*(array.shape[1]-1)
    fy = (y-y0)/(y1-y0)*(array.shape[0]-1)
    ix = np.minimum(fx.astype(int), array.shape[1]-2)
    iy = np.minimum(fy.astype(int), array.shape[0]-2)
    return ix, iy, fx-ix, fy-iy


def triangle_heights(array, bounds, x, y):
    """与 grid_mesh 的 LL->UR 对角线完全一致，不能用双线性代替碰撞三角面。"""
    ix, iy, u, v = _grid_coordinates(array, bounds, x, y)
    a, b = array[iy, ix], array[iy, ix+1]
    c, d = array[iy+1, ix], array[iy+1, ix+1]
    return np.where(u >= v, a*(1-u)+b*(u-v)+d*v, a*(1-v)+d*u+c*(v-u))


class HeightSource:
    def __init__(self, base, bounds, patches):
        self.base, self.bounds, self.patches = base, list(bounds), patches
        if base.ndim != 2 or min(base.shape) < 2 or not np.all(np.isfinite(base)):
            raise ValueError('invalid heightfield')
        for patch in patches:
            h = patch['height']
            if h.ndim != 2 or min(h.shape) < 2 or not np.all(np.isfinite(h)):
                raise ValueError('invalid detail heightfield')
            b = patch['bounds']
            if (not np.isclose((h.shape[1]-1)*patch['step'], b[1]-b[0]) or
                    not np.isclose((h.shape[0]-1)*patch['step'], b[3]-b[2])):
                raise ValueError('detail shape/spacing mismatch')

    @staticmethod
    def bilinear(array, bounds, x, y):
        ix, iy, u, v = _grid_coordinates(array, bounds, x, y)
        return (array[iy, ix]*(1-u)*(1-v)+array[iy, ix+1]*u*(1-v)+
                array[iy+1, ix]*(1-u)*v+array[iy+1, ix+1]*u*v)

    def sample(self, x, y):
        x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
        values = np.array(self.bilinear(self.base, self.bounds, x, y), copy=True)
        # 与原准备脚本一致：首个包含坐标的 detail patch 优先。
        assigned = np.zeros(x.shape, dtype=bool)
        for patch in self.patches:
            x0, x1, y0, y1 = patch['bounds']
            mask = (x >= x0)&(x <= x1)&(y >= y0)&(y <= y1)&~assigned
            if np.any(mask):
                values[mask] = self.bilinear(patch['height'], patch['bounds'], x[mask], y[mask])
                assigned[mask] = True
        return values


def uniform_grid(source, bounds, spacing=1.):
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError('invalid spacing')
    x0, x1, y0, y1 = bounds
    nx, ny = (x1-x0)/spacing, (y1-y0)/spacing
    if not np.isclose(nx, round(nx)) or not np.isclose(ny, round(ny)):
        raise ValueError('spacing must tile world exactly')
    x, y = np.meshgrid(np.linspace(x0, x1, round(nx)+1), np.linspace(y0, y1, round(ny)+1))
    return source.sample(x, y).astype(np.float32)


def grid_mesh(grid, bounds):
    x0, x1, y0, y1 = bounds
    x, y = np.meshgrid(np.linspace(x0, x1, grid.shape[1]), np.linspace(y0, y1, grid.shape[0]))
    points = np.column_stack((x.ravel(), y.ravel(), grid.ravel())).astype(np.float32)
    ll = (np.arange(grid.shape[0]-1)[:, None]*grid.shape[1]+np.arange(grid.shape[1]-1)).ravel()
    faces = np.empty((len(ll)*2, 3), dtype=np.int32)
    faces[0::2] = np.column_stack((ll, ll+1, ll+grid.shape[1]+1))
    faces[1::2] = np.column_stack((ll, ll+grid.shape[1]+1, ll+grid.shape[1]))
    return points, faces


def grid_normals(grid, bounds):
    """按实际高程及 XY 间距计算向上单位顶点法线，边缘使用单侧差分。"""
    grid = np.asarray(grid, dtype=np.float32)
    if grid.ndim != 2 or min(grid.shape) < 2 or not np.all(np.isfinite(grid)):
        raise ValueError('invalid normal heightfield')
    x0, x1, y0, y1 = bounds
    if not np.all(np.isfinite(bounds)) or x1 <= x0 or y1 <= y0:
        raise ValueError('invalid normal bounds')
    dy, dx = np.gradient(grid, (y1-y0)/(grid.shape[0]-1), (x1-x0)/(grid.shape[1]-1))
    normals = np.column_stack((-dx.ravel(), -dy.ravel(), np.ones(grid.size, np.float32)))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return normals


def rock_proxy(low, high, triangles=80):
    """80/320 面 ico 凸代理；默认保留旧调用，世界 AABB 不承诺占据等价。"""
    low, high = np.asarray(low, float), np.asarray(high, float)
    if low.shape != (3,) or high.shape != (3,) or not np.all(np.isfinite([low, high])) or np.any(high <= low):
        raise ValueError('invalid rock envelope')
    if triangles not in (80, 320):
        raise ValueError('unsupported rock topology; expected 80 or 320 triangles')
    phi = (1+5**.5)/2
    points = np.array([(-1,phi,0),(1,phi,0),(-1,-phi,0),(1,-phi,0),
                       (0,-1,phi),(0,1,phi),(0,-1,-phi),(0,1,-phi),
                       (phi,0,-1),(phi,0,1),(-phi,0,-1),(-phi,0,1)], float)
    points /= np.linalg.norm(points, axis=1)[:, None]
    faces = [(0,11,5),(0,5,1),(0,1,7),(0,7,10),(0,10,11),(1,5,9),(5,11,4),
             (11,10,2),(10,7,6),(7,1,8),(3,9,4),(3,4,2),(3,2,6),(3,6,8),(3,8,9),
             (4,9,5),(2,4,11),(6,2,10),(8,6,7),(9,8,1)]
    for _ in range(1 if triangles == 80 else 2):
        values, edges, refined = list(points), {}, []
        def middle(a, b):
            key = tuple(sorted((a, b)))
            if key not in edges:
                point = (values[a]+values[b])/2
                edges[key] = len(values)
                values.append(point/np.linalg.norm(point))
            return edges[key]
        for a, b, c in faces:
            ab, bc, ca = middle(a,b), middle(b,c), middle(c,a)
            refined.extend(((a,ab,ca),(b,bc,ab),(c,ca,bc),(ab,bc,ca)))
        points, faces = np.array(values), refined
    points = low+(points-points.min(axis=0))/np.ptp(points, axis=0)*(high-low)
    return points, np.asarray(faces, np.int32)


def height_error_report(source, grid, bounds):
    def compare(array, area):
        errors = []
        # 遍历每个源顶点，不随机抽样；逐行限制暂存内存。
        xs = np.linspace(area[0], area[1], array.shape[1])
        for y in np.linspace(area[2], area[3], array.shape[0]):
            errors.append(triangle_heights(grid, bounds, xs, y)-source.sample(xs, y))
        err = np.concatenate(errors)
        absolute = abs(err)
        return {'samples': int(err.size), 'mean_signed_m': float(err.mean()),
                'rmse_m': float(np.sqrt(np.mean(err*err))), 'max_abs_m': float(absolute.max()),
                'abs_percentiles_m': {str(q): float(np.percentile(absolute, q)) for q in (50,90,95,99)}}
    return {'comparison': 'new collision triangles vs source bilinear height at every source grid vertex',
            'global': compare(source.base, source.bounds),
            'details': {p.get('name', str(i)): compare(p['height'], p['bounds']) for i,p in enumerate(source.patches)},
            'traversability_equivalence': 'NOT_VERIFIED; new geometry version',
            'between_source_vertices_error_bound': 'NOT_ESTABLISHED; report evaluates every source grid vertex' }


def triangle_support(points, faces, xy):
    """实际世界三角面的垂直交点；重叠时选择最高支撑面，域外拒绝。"""
    tri = np.asarray(points, float)[np.asarray(faces, np.int32)]
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    den = (b[:,1]-c[:,1])*(a[:,0]-c[:,0])+(c[:,0]-b[:,0])*(a[:,1]-c[:,1])
    good = abs(den) > 1e-12
    if not np.any(good):
        raise ValueError('no nonvertical support triangles')
    a,b,c,den = a[good],b[good],c[good],den[good]
    result = []
    for x,y in np.asarray(xy, float):
        u = ((b[:,1]-c[:,1])*(x-c[:,0])+(c[:,0]-b[:,0])*(y-c[:,1]))/den
        v = ((c[:,1]-a[:,1])*(x-c[:,0])+(a[:,0]-c[:,0])*(y-c[:,1]))/den
        w = 1-u-v
        inside = (u >= -1e-9)&(v >= -1e-9)&(w >= -1e-9)
        if not np.any(inside):
            raise ValueError('no actual triangle beneath spawn/contact')
        result.append(float((u*a[:,2]+v*b[:,2]+w*c[:,2])[inside].max()))
    return np.asarray(result)


def _write_mesh(stage, path, points, faces, color, *, collision=True, normals=None):
    from pxr import Gf, UsdGeom, UsdPhysics, Vt
    mesh = UsdGeom.Mesh.Define(stage, path)
    points = np.asarray(points, dtype=np.float32)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces), 3, np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(faces, np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateOrientationAttr('rightHanded')
    mesh.CreateExtentAttr([Gf.Vec3f(*points.min(axis=0).tolist()), Gf.Vec3f(*points.max(axis=0).tolist())])
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if normals is not None:
        mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(normals, np.float32)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if collision:
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    return mesh


def _bind_regolith_material(stage, mesh):
    from pxr import Gf, Sdf, UsdShade
    material = UsdShade.Material.Define(stage, ENV+'/Materials/Regolith')
    shader = UsdShade.Shader.Define(stage, str(material.GetPath())+'/PreviewSurface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.34, .33, .31))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.95)
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    # 默认 allPurpose 只提供视觉；ENV 已有的 physics purpose 接触材质继续继承。
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def _source_statistics(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    result = {'visible_triangles':0, 'collider_triangles':0, 'mesh_count':0}
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh):
            result['mesh_count'] += 1
            counts = np.asarray(UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get(), dtype=np.int64)
            n = int(np.maximum(counts-2, 0).sum())
            if UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible':
                result['visible_triangles'] += n
            if prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
                result['collider_triangles'] += n
    return result


def _source_rocks(stage):
    from pxr import Usd, UsdGeom, UsdPhysics
    cache = UsdGeom.XformCache()
    rocks = {}
    # 生效组合中的实际岩石碰撞 mesh；不依据 Blender 旧清单复制失活岩石。
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if not path.startswith('/World/Environment/Rock_'):
            continue
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        local = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), float)
        m = np.asarray(cache.GetLocalToWorldTransform(prim), float)
        points = local@m[:3,:3]+m[3,:3]
        root = '/'.join(path.split('/')[:4])
        low, high = points.min(axis=0), points.max(axis=0)
        if root in rocks:
            low,high = np.minimum(rocks[root][0],low),np.maximum(rocks[root][1],high)
        rocks[root] = (low,high)
    if not rocks:
        raise ValueError('no active source rock collider meshes found')
    return [(path,*bounds) for path,bounds in sorted(rocks.items())]


def _cameras(stage, regions, source):
    from pxr import Gf, UsdGeom
    UsdGeom.Xform.Define(stage, VIEWS)
    views = {'overview': (0, 0, 1800)}
    for region in regions['regions']:
        x0,x1,y0,y1 = region['bounds_xy_m']
        x,y = np.meshgrid(np.arange(x0,x1+1), np.arange(y0,y1+1))
        views[region['id']] = ((x0+x1)/2, (y0+y1)/2,
                              float(source.sample(x,y).max())+region['top_camera_height_above_region_max_m'])
    for name, xyz in views.items():
        camera = UsdGeom.Camera.Define(stage, VIEWS+'/'+name)
        UsdGeom.Xformable(camera).AddTranslateOp().Set(Gf.Vec3d(*xyz))
        camera.CreateFocalLengthAttr(35 if name == 'overview' else 24)
        camera.CreateHorizontalApertureAttr(36)
        camera.CreateVerticalApertureAttr(20.25)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.1,10000))
    return {key:VIEWS+'/'+key for key in views}


def build_showcase(profile, region_config, source_scene, assets_dir, output_dir, quality='light') -> dict:
    if quality not in ('light', 'original'):
        raise ValueError('quality must be light or original')
    output = Path(output_dir).resolve()
    if output.exists():
        raise FileExistsError('refusing existing output directory: '+str(output))
    regions = load_regions(region_config)
    quality_config_path = Path(__file__).resolve().parents[2]/'config/isaac_viper_showcase.yaml'
    import yaml
    quality_config = yaml.safe_load(quality_config_path.read_text(encoding='utf-8'))
    spec = quality_spec(quality_config)
    source_path = Path(source_scene).resolve()
    if digest(source_path) != regions['source']['rover_entry_sha256']:
        raise ValueError('source scene SHA does not match region authority')
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    from tools.isaac_viper.build_stage import build_stage
    from tools.isaac_viper.config import load_profile
    p = load_profile(profile) if not isinstance(profile, dict) else profile
    data = Path(regions['source']['terrain_data_directory'])
    detail_path = data/'detail_patches.json'
    patches = json.loads(detail_path.read_text(encoding='utf-8'))
    for patch in patches:
        patch['height'] = np.load(data/(patch['name']+'_height.npy'), mmap_mode='r')
    base_path = data/'heightfield_m.npy'
    base = np.load(base_path, mmap_mode='r')
    if base.shape != (2001,2001):
        raise ValueError('expected 2001x2001 heightfield')
    heights = HeightSource(base, BOUNDS, patches)
    tracked = [source_path, base_path, detail_path]+[data/(v['name']+'_height.npy') for v in patches]
    hashes = {str(path):digest(path) for path in tracked}
    session = Sdf.Layer.CreateAnonymous()
    Sdf.CreatePrimInLayer(session, '/World/LunarRover/chassis/Sensors').active = False
    source = Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(source_path)), session)
    for layer in source.GetUsedLayers():
        if layer.realPath and Path(layer.realPath).is_file():
            hashes[str(Path(layer.realPath).resolve())] = digest(layer.realPath)
    hashes[str(quality_config_path)] = digest(quality_config_path)
    before = _source_statistics(source)
    rocks = _source_rocks(source) if quality == 'light' else []
    output.mkdir(parents=True, exist_ok=False)
    environment_path = output/'showcase-environment.usdc'
    environment = Usd.Stage.CreateNew(str(environment_path))
    environment.GetRootLayer().subLayerPaths = [str(source_path)]
    # 旧环境整树 inactive 同时移除显示和碰撞；新地形在独立根。
    environment.OverridePrim('/World/LunarRover').SetActive(False)
    environment.OverridePrim('/World/MidtermPreparation').SetActive(False)
    errors, bound_errors = None, []
    if quality == 'light':
        environment.OverridePrim('/World/Environment').SetActive(False)
        new_env = UsdGeom.Xform.Define(environment, ENV).GetPrim()
        ground = UsdShade.Material(environment.GetPrimAtPath('/World/Materials/GroundContact'))
        if not ground:
            raise ValueError('source ground physics material missing')
        UsdShade.MaterialBindingAPI.Apply(new_env).Bind(ground, UsdShade.Tokens.weakerThanDescendants, 'physics')
        grid = uniform_grid(heights, BOUNDS, quality_config['terrain_spacing_m'])
        points, faces = grid_mesh(grid, BOUNDS)
        terrain = _write_mesh(environment, TERRAIN, points, faces, (.34,.33,.31),
                              normals=grid_normals(grid, BOUNDS) if spec['smooth_terrain'] else None)
        if spec['smooth_terrain']:
            _bind_regolith_material(environment, terrain)
        for index,(path, low, high) in enumerate(rocks):
            points, faces = rock_proxy(low, high, triangles=spec['rock_triangles'])
            mesh = _write_mesh(environment, ENV+f'/Rocks/Rock_{index:04d}', points, faces, (.24,.23,.22))
            mesh.GetPrim().SetCustomDataByKey('source_collider_path', path)
            written = np.asarray(mesh.GetPointsAttr().Get(), float)
            bound_errors.append(float(max(abs(written.min(axis=0)-low).max(), abs(written.max(axis=0)-high).max())))
        errors = height_error_report(heights, grid, BOUNDS)
    environment.GetRootLayer().Save()
    stage_path = output/'viper-showcase.usda'
    result = build_stage(p, assets_dir, environment_path, stage_path,
                         terrain_mesh_path=TERRAIN if quality == 'light' else None,
                         spawn_xy_m=regions['regions'][2]['spawn_candidate_xy_m'])
    stage = Usd.Stage.Open(str(stage_path))
    metadata = json.loads(stage.GetRootLayer().customLayerData['engineering_metadata_json'])
    if any(digest(path) != sha for path,sha in hashes.items()):
        raise RuntimeError('source changed during showcase generation')
    camera_paths = _cameras(stage, regions, heights)
    quality_id = quality_config['geometry_version'] if quality == 'light' else 'original-source-v1'
    geometry = {'quality':quality, 'geometry_version':quality_id, 'source_hashes':hashes,
                'source_unchanged':True, 'quality_settings':dict(quality_config,quality=quality,geometry_version=quality_id), 'source_statistics':before, 'height_error_m':errors,
                'terrain_mesh_path':TERRAIN if quality == 'light' else '/World/Environment/DetailTerrain_Rover_plain/DetailMesh_Rover_plain',
                'ground_material_root':ENV if quality == 'light' else '/World/Environment',
                'rock_count':len(rocks), 'rock_triangles_each':spec['rock_triangles'] if rocks else None,
                'rock_world_aabb_max_error_m':max(bound_errors, default=0.),
                'rock_proxy_note':f"{spec['rock_triangles']}-triangle convex proxy; AABB preserved within float32 rounding; occupancy/traversability equivalence NOT_VERIFIED",
                'new_statistics':_source_statistics(stage), 'region_camera_paths':camera_paths,
                'sensor_annotation_geometry':'NONE; cameras only, region text in viewport UI',
                'lighting_note':regions['source']['lighting_note'],
                'NOT_RUN':['GPU','ROS','dynamic_settle','A/B known maps','Actor','new planner','navigation','Orin','real_vehicle']}
    metadata['showcase'] = geometry
    stage.GetRootLayer().customLayerData = {'engineering_metadata_json':json.dumps(metadata, ensure_ascii=False)}
    stage.SetTimeCodesPerSecond(source.GetTimeCodesPerSecond())
    stage.SetFramesPerSecond(source.GetFramesPerSecond())
    stage.GetRootLayer().Save()
    (output/'isaac-polar-v02-midterm-regions.json').write_bytes(Path(region_config).read_bytes())
    validation = validate_showcase(stage_path, p)
    result = {'stage':str(stage_path), 'geometry':geometry, 'validation':validation,
              'profile_semantic_sha256':hashlib.sha256(json.dumps(p,sort_keys=True,allow_nan=False).encode()).hexdigest()}
    (output/'showcase-receipt.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    if not validation['passed']:
        raise ValueError('showcase contract rejected: '+'; '.join(validation['errors']))
    return result


def validate_showcase_environment(stage, errors):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    try:
        metadata = json.loads(stage.GetRootLayer().customLayerData['engineering_metadata_json'])
        info = metadata['showcase']
        settings = info['quality_settings']
        spec = quality_spec(settings)
        if (info['geometry_version'] != settings['geometry_version'] or info['quality'] != settings['quality'] or
                info['rock_triangles_each'] != spec['rock_triangles']):
            raise ValueError('quality metadata/settings mismatch')
        if not info['source_unchanged']:
            errors.append('source_unchanged')
        if info['quality'] == 'original':
            original_env, light_env = stage.GetPrimAtPath('/World/Environment'), stage.GetPrimAtPath(ENV)
            if not original_env or not original_env.IsActive() or (light_env and light_env.IsActive()):
                errors.append('quality original source environment mismatch')
            if (info['terrain_mesh_path'] != '/World/Environment/DetailTerrain_Rover_plain/DetailMesh_Rover_plain' or
                    info['ground_material_root'] != '/World/Environment' or info['rock_count'] != 0):
                errors.append('quality original source metadata mismatch')
        if info['quality'] == 'light':
            old = stage.GetPrimAtPath('/World/Environment')
            if old and old.IsActive():
                errors.append('old environment remains active')
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(TERRAIN))
            counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get()) if mesh else np.array([])
            if len(counts) != spec['terrain_triangles'] or np.any(counts != 3):
                errors.append('quality terrain topology')
            if mesh:
                points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
                ny, nx = spec['terrain_shape']
                valid_points = points.shape == (spec['terrain_points'], 3) and np.all(np.isfinite(points))
                if valid_points:
                    coords = points.reshape(ny, nx, 3)
                    valid_points = (np.allclose(coords[:, :, 0], np.linspace(BOUNDS[0], BOUNDS[1], nx)[None, :], rtol=0, atol=1e-5) and
                                    np.allclose(coords[:, :, 1], np.linspace(BOUNDS[2], BOUNDS[3], ny)[:, None], rtol=0, atol=1e-5))
                if not valid_points:
                    errors.append('quality terrain world bounds/vertices/spacing')
                indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
                valid_indices = indices.shape == (spec['terrain_triangles']*3,)
                if valid_indices:
                    triangles = indices.reshape(-1, 3)
                    # 固定 LL->UR 拓扑；分块比较以免复制整个 800 万三角数组。
                    for begin in range(0, (ny-1)*(nx-1), 100000):
                        end = min(begin+100000, (ny-1)*(nx-1))
                        cells = np.arange(begin, end)
                        ll = cells//(nx-1)*nx+cells%(nx-1)
                        lower, upper = triangles[2*begin:2*end:2], triangles[2*begin+1:2*end:2]
                        if not (np.array_equal(lower[:, 0], ll) and np.array_equal(lower[:, 1], ll+1) and
                                np.array_equal(lower[:, 2], ll+nx+1) and np.array_equal(upper[:, 0], ll) and
                                np.array_equal(upper[:, 1], ll+nx+1) and np.array_equal(upper[:, 2], ll+nx)):
                            valid_indices = False
                            break
                if not valid_indices:
                    errors.append('quality terrain triangle indices')
                if spec['smooth_terrain']:
                    normals = np.asarray(mesh.GetNormalsAttr().Get(), dtype=np.float32)
                    valid_normals = (normals.shape == (spec['terrain_points'], 3) and
                                     mesh.GetNormalsInterpolation() == UsdGeom.Tokens.vertex and
                                     np.all(np.isfinite(normals)) and
                                     np.allclose(np.linalg.norm(normals, axis=1), 1., rtol=0, atol=1e-4))
                    if valid_normals and valid_points:
                        valid_normals = np.allclose(normals, grid_normals(coords[:, :, 2], BOUNDS), rtol=0, atol=1e-4)
                    if not valid_normals:
                        errors.append('terrain smooth normals missing/invalid')
                    material, _ = UsdShade.MaterialBindingAPI(mesh.GetPrim()).ComputeBoundMaterial()
                    shader = material.ComputeSurfaceSource()[0] if material else None
                    valid_material = bool(shader) and shader.GetIdAttr().Get() == 'UsdPreviewSurface'
                    if valid_material:
                        color = shader.GetInput('diffuseColor').Get()
                        roughness = shader.GetInput('roughness').Get()
                        metallic = shader.GetInput('metallic').Get()
                        valid_material = (color is not None and roughness is not None and metallic is not None and
                                          np.allclose(color, [.34, .33, .31], rtol=0, atol=1e-6) and
                                          np.isclose(roughness, .95, rtol=0, atol=1e-6) and
                                          np.isclose(metallic, 0., rtol=0, atol=1e-6))
                    if not valid_material:
                        errors.append('terrain matte material missing/invalid')
                    physics_material, _ = UsdShade.MaterialBindingAPI(mesh.GetPrim()).ComputeBoundMaterial(materialPurpose='physics')
                    if not physics_material or str(physics_material.GetPath()) != '/World/Materials/GroundContact':
                        errors.append('terrain physics material changed')
                if (not mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI) or
                        UsdPhysics.CollisionAPI(mesh.GetPrim()).GetCollisionEnabledAttr().Get() is not True):
                    errors.append('new terrain collision disabled')
            for prim in Usd.PrimRange(stage.GetPseudoRoot(),Usd.TraverseInstanceProxies()):
                path = str(prim.GetPath())
                if (prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() and
                        not path.startswith((ENV+'/', '/World/ProjectVIPER/'))):
                    errors.append('unexpected legacy collider: '+path)
            rock_prims = [q for q in stage.Traverse() if str(q.GetPath()).startswith(ENV+'/Rocks/') and q.IsA(UsdGeom.Mesh)]
            if len(rock_prims) != info['rock_count']:
                errors.append('rock proxy count')
            _, expected_rock_faces = rock_proxy([0, 0, 0], [1, 1, 1], triangles=spec['rock_triangles'])
            for prim in rock_prims:
                rock = UsdGeom.Mesh(prim)
                rock_counts = np.asarray(rock.GetFaceVertexCountsAttr().Get(), dtype=np.int32)
                rock_points = np.asarray(rock.GetPointsAttr().Get(), dtype=np.float32)
                rock_indices = np.asarray(rock.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
                if (rock_counts.shape != (spec['rock_triangles'],) or np.any(rock_counts != 3) or
                        rock_points.shape != (spec['rock_points'], 3) or not np.all(np.isfinite(rock_points)) or
                        rock_indices.shape != (spec['rock_triangles']*3,) or np.any(rock_indices < 0) or
                        np.any(rock_indices >= spec['rock_points']) or
                        not np.array_equal(rock_indices, expected_rock_faces.ravel())):
                    errors.append('quality rock proxy topology: '+str(prim.GetPath()))
                if (not prim.HasAPI(UsdPhysics.CollisionAPI) or
                        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not True):
                    errors.append('rock proxy collision disabled: '+str(prim.GetPath()))
        for root in ('/World/LunarRover','/World/MidtermPreparation','/Render'):
            prim = stage.GetPrimAtPath(root)
            if prim and prim.IsActive():
                errors.append('legacy active: '+root)
        views = stage.GetPrimAtPath(VIEWS)
        if not views:
            errors.append('missing showcase views')
        else:
            for prim in Usd.PrimRange(views):
                if prim.IsA(UsdGeom.Boundable) or prim.HasAPI(UsdPhysics.CollisionAPI):
                    errors.append('sensor annotation geometry leak: '+str(prim.GetPath()))
        pose = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/ProjectVIPER'))
        if np.max(abs(np.asarray(pose.ExtractTranslation())-metadata['initial_position_m'])) > 1e-6:
            errors.append('showcase pose metadata mismatch')
        if info['quality'] == 'light' and metadata['placement'].get('surface_query') != 'actual_triangle_barycentric':
            errors.append('placement must use actual new triangles')
    except Exception as exc:
        errors.append('showcase metadata/composition: '+type(exc).__name__+': '+str(exc))


def validate_showcase(stage_or_path, profile):
    from pxr import Usd
    from tools.isaac_viper.validate_stage import validate_stage
    stage = stage_or_path if isinstance(stage_or_path, Usd.Stage) else Usd.Stage.Open(str(stage_or_path))
    result = validate_stage(stage, profile)
    # validate_stage 对带 showcase 身份的实际组合调用同一个补充合同。
    text = stage.GetRootLayer().customLayerData.get('engineering_metadata_json')
    if not text:
        result['errors'].append('showcase metadata/composition: engineering_metadata_missing')
    elif 'showcase' not in json.loads(text):
        result['errors'].append('showcase metadata/composition: showcase identity missing')
    result['passed'] = not result['errors']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', default='config/isaac_viper.yaml')
    parser.add_argument('--regions', default='docs/simulation/isaac-polar-v02-midterm-regions.json')
    parser.add_argument('--source-scene', required=True)
    parser.add_argument('--assets-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--quality', choices=('light','original'), default='light')
    args = parser.parse_args()
    print(json.dumps(build_showcase(args.profile,args.regions,args.source_scene,args.assets_dir,args.output_dir,args.quality),ensure_ascii=False))


if __name__ == '__main__':
    main()
