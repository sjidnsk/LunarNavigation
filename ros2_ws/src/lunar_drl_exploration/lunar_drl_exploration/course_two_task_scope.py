"""Whole-cave curriculum metadata and opt-in rehearsal geometry.

Task bounds enclose the cave; they do not reveal its internal walls to the Actor.
The exterior apron is a real approach area, not an excluded cave passage.
"""
from copy import deepcopy
from types import MappingProxyType
import numpy as np

SCOPE = 'whole_cave_v1'
VERSION = 'region_sensor_rebalanced_scene_v3'
CAPABILITIES = ('moon_cover', 'cave_cover', 'moon_finish', 'cave_finish')


def scene_requests():
    from .course_two_scene import _identity
    sizes = ([16., 32.], [32., 64.], [32., 64.], [64., 96.])
    def row(split, ordinal, role, family, start, bearing, size):
        r = dict(design_version='course_two_task_scope_v3', task_scope=SCOPE,
            split=split, request_ordinal=ordinal,
            seed=(2026101300000 if split=='train' else 2026101400000)+ordinal,
            stage='A1', role=role, family=family, planned_policy_start=start,
            bearing_bin=bearing, roi_side_interval_m=list(size),
            sensor=dict(range_m=10., fov_deg=90.), sensor_immutable=True,
            max_geometry_attempts=100, qualification='NOT_RUN',
            terrain_template='internal_occlusion', prelude_required=role=='finish',
            policy_start_coverage=dict(ge=.9, le=.98) if role=='finish' else dict(gt=0., lt=.8))
        if role=='old':
            r.update(old_task=family+'_'+start, terrain_template='whole_cave_rehearsal' if family=='cave'
                else 'unchanged_old_scene_v5', old_nominal_extent_m=[30., 100.])
        if role=='approach':
            visibility='O' if bearing in (2, 6) else 'U'
            r.update(visibility_after_native_initialization=visibility,
                terrain_template='short_occluded_approach' if visibility=='O' else 'bounded_open_approach',
                blind_gap_m=[1.,5.] if visibility=='U' else None, policy_start_coverage=dict(eq=0.))
        return r
    training=[]
    for task in ('moon_inside','cave_inside','moon_outside','cave_outside'):
        family,start=task.split('_')
        for i in range(16):training.append(row('train',len(training),'old',family,start,i%8,sizes[i%4]))
    for role,count in (('cover',16),('approach',8),('finish',8)):
        for family in ('moon','cave'):
            for i in range(count):
                r=row('train',len(training),role,family,'outside' if role=='approach' else 'inside',i%8,sizes[i%4])
                if role=='cover' and i%4==0:r['terrain_template']='open'
                training.append(r)
    development=[]
    for capability in CAPABILITIES:
        family,role=capability.split('_')
        for i in range(16):
            r=row('development',len(development),role,family,'inside',i%8,sizes[i%4])
            r.update(capability=capability, used_for_checkpoint_selection=True, independent_publication_test=False)
            if role=='cover' and i%4==0:r['terrain_template']='open'
            development.append(r)
    return [[dict(r,request_id=_identity(r)) for r in rows] for rows in (training,development)]


def build_rehearsal(seed, extent, start_mode, platform):
    from .scene import Scene, TerrainGrid
    from .contracts import Pose, TaskSpec
    from .geometry import polygon_mask
    from .course_two_scene import _identity
    if start_mode not in ('inside','outside'):raise ValueError('whole cave requires explicit start')
    scene=Scene(seed,'cave',extent,platform=platform)
    original=TerrainGrid.from_scene(scene)
    heights=original.heights.copy();rows,cols=np.nonzero(heights==0.)
    resolution=original.resolution_m
    low=np.array(original.origin)+np.array([cols.min(),rows.min()])*resolution-.4
    high=np.array(original.origin)+(np.array([cols.max(),rows.max()])+1)*resolution+.4
    polygon=np.array([[low[0],low[1]],[high[0],low[1]],[high[0],high[1]],[low[0],high[1]]])
    x=original.origin[0]+(np.arange(original.shape[1])+.5)*resolution
    y=original.origin[1]+(np.arange(original.shape[0])+.5)*resolution
    mouth_x,mouth_y,_=scene._rooms[1]
    apron=x[None,:]<low[0]
    entrance=(x[None,:]<mouth_x)&(np.abs(y[:,None]-mouth_y)<1.8)
    # The current curriculum uses only inside starts: preserve the natural cave.
    # Outside geometry remains solely for the versioned historical diagnostic.
    if start_mode=='outside':heights[np.broadcast_to(apron,heights.shape)|entrance]=0.
    terrain=TerrainGrid.from_heights(heights,resolution,original.origin,platform)
    task=TaskSpec(_identity(dict(scope=SCOPE,seed=seed,extent=extent)),'map',polygon)
    anchor=Pose(mouth_x,mouth_y,scene._yaw)
    reachable=terrain.reachable(anchor)!=0
    roi=polygon_mask(terrain.shape,terrain.origin,resolution,polygon)
    candidates=reachable & (roi if start_mode=='inside' else ~roi)
    if start_mode=='outside':
        candidates&=(x[None,:]>=low[0]-5)&(np.abs(y[:,None]-mouth_y)<2.)
    ys,xs=np.nonzero(candidates)
    if not len(xs):raise ValueError('whole cave has no legal requested start')
    pick=int(np.random.default_rng(np.random.SeedSequence([seed,302])).integers(len(xs)))
    pose=Pose(*terrain.cell_center(int(xs[pick]),int(ys[pick])),scene._yaw)
    descriptor=dict(generator_version='whole_cave_rehearsal_v1',task_scope=SCOPE,
        seed=seed,family='cave',extent_m=extent,provided_bounds=scene.bounds,
        resolution_m=resolution,start_mode=start_mode,curriculum_task='cave_'+start_mode,
        cave_interior_outside_cells=int(np.count_nonzero((original.heights==0)&~roi)))
    if descriptor['cave_interior_outside_cells']:raise ValueError('cave task cuts interior')
    terrain.generator_descriptor=MappingProxyType(descriptor)
    return terrain,pose,task,descriptor


def evaluation_cases(legacy):
    """Re-freeze task-aligned cases; keep D geometry as labelled diagnostics."""
    from .course_two_scene import _identity
    from .course_two_rebalanced_scene import make_case
    result=deepcopy(legacy)
    for row in result:
        row['task_scope']=SCOPE
        if row['kind']=='new':
            old=row['case'];width=old['roi_width_m'];height=old['roi_height_m']
            request=dict(task_scope=SCOPE,seed=old['seed'],family=old['family'],role='approach',
                planned_policy_start='outside',bearing_bin=old['bearing_bin'],sensor=old['sensor'],
                roi_side_interval_m=[min(width,height),max(width,height)],
                terrain_template='bounded_open_approach' if old['visibility_class']=='U' else 'short_occluded_approach',
                visibility_after_native_initialization=old['visibility_class'],
                blind_gap_m=[old['blind_gap_m'],old['blind_gap_m']] if old['visibility_class']=='U' else None)
            request['request_id']=_identity(request)
            row['case']=make_case(request);row['case_id']='scope-v3-'+row['case_id']
        elif row['kind']=='old' and row['spec']['family']=='cave':
            row['spec']['task_scope']=SCOPE;row['case_id']='scope-v3-'+row['case_id']
        elif row['kind']=='visible' and row['case']['family']=='cave':
            case=row['case'];case.pop('scenario_id');case['task_scope']=SCOPE
            case['scenario_id']=_identity(case);row['case_id']='scope-v3-'+row['case_id']
        elif row['kind']=='diagnostic':
            row['task_scope']='historical_diagnostic';row['promotion_required']=False
    return result
