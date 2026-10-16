"""Deterministic bounded tasks; one immutable sensor per scenario identity.

Ground-truth path/visibility checks are sampler diagnostics only. The Actor gets
the ordinary measured graph, task polygon and sensor context, never this path.
"""
from dataclasses import asdict, replace
import hashlib
import json
import math
from types import MappingProxyType
import numpy as np
from .contracts import Pose, SensorSpec, TaskSpec

VERSION = 'region_sensor_course_two_scene_v1'
STAGES = ('A1', 'A2', 'B', 'C', 'D')
RANGES = (10., 20., 40., 70., 100.)
FOVS = (60., 90., 120., 180., 360.)


def _identity(record):
    return hashlib.sha256(json.dumps(record, sort_keys=True, allow_nan=False,
        separators=(',', ':')).encode()).hexdigest()


def make_case(seed, stage, ordinal, *, range_m=None, fov_deg=None,
              visibility=None, family=None, range_limit=100., easy=False):
    if stage not in STAGES or type(seed) is not int or seed < 0 or type(ordinal) is not int or ordinal < 0:
        raise ValueError('invalid course-two scene seed/stage/ordinal')
    rng = np.random.default_rng(np.random.SeedSequence([seed, 20261010]))
    allowed = [r for r in RANGES if r <= range_limit]
    if not allowed: raise ValueError('range limit below 10 m')
    r = float(range_m if range_m is not None else
        (10. if stage in ('A1', 'A2') else allowed[(ordinal // 8) % len(allowed)]))
    fov = float(fov_deg if fov_deg is not None else
        (90. if stage in ('A1', 'A2', 'B') else FOVS[(ordinal // 40) % 5]))
    if not 10 <= r <= 100 or not (60 <= fov <= 180 or fov == 360):
        raise ValueError('course-two sensor outside approved range/FOV')
    category = visibility
    if category is None:
        roll = int(rng.integers(100))
        limits = (80, 90, 100) if stage == 'A1' else ((70, 80, 90) if stage == 'A2' else (60, 70, 85))
        category = 'U' if roll < limits[0] else 'V' if roll < limits[1] else 'F' if roll < limits[2] else 'O'
    if category not in ('U', 'V', 'F', 'O'): raise ValueError('invalid visibility class')
    if category == 'F' and fov == 360: category = 'U'
    selected_family = family or ('moon' if (ordinal // 8) % 2 == 0 else 'cave')
    if selected_family not in ('moon', 'cave'): raise ValueError('invalid terrain family')
    size_bin = int(rng.choice(3, p=(.4, .4, .2)))
    low, high = (16., 32.) if stage == 'A1' else ((8., 16.), (16., 32.), (32., 80.))[size_bin]
    width, height = map(float, rng.uniform(low, high, 2))
    # Short occlusion tasks need a short physically feasible detour. Record the
    # actual size instead of pretending a large rejected rectangle was sampled.
    if category == 'O': width, height = min(width, 24.), min(height, 24.)
    gap_bin = int(rng.integers(3))
    gap_bounds = (1., 5.) if stage == 'A1' or easy else ((1., 5.), (5., 10.), (10., 20.))[gap_bin]
    gap = float(rng.uniform(*gap_bounds)) if category == 'U' else 0.
    distance = r + gap if category == 'U' else float(rng.uniform(2., min(8., .7*r)))
    angle = ordinal % 8 * math.pi/4 + float(rng.uniform(-math.pi/16, math.pi/16))
    yaw = angle + (math.pi if category == 'F' else 0. if category == 'V' else float(rng.uniform(-math.pi, math.pi)))
    translation=rng.uniform(-12,12,2)
    start=translation+(-width/2-distance)*np.array([math.cos(angle),math.sin(angle)])
    # Follow Scene.initial_pose's lattice-center convention. Move the entire
    # geometry together so g/d and the certified route remain exact; this also
    # avoids unvalidated sub-cell translations at episode initialization.
    translation+=.2*(np.floor(start/.2)+.5)-start
    record = dict(version=VERSION, seed=seed, stage=stage, ordinal=ordinal,
        family=selected_family, bearing_bin=ordinal % 8, bearing_rad=angle,
        sensor=asdict(SensorSpec(range_m=r, fov_deg=fov)), visibility_class=category,
        blind_gap_m=gap, start_distance_m=distance, roi_width_m=width,
        roi_height_m=height, translation=list(map(float, translation)),
        initial_yaw_rad=yaw, resolution_m=.2, episode_budget=512,
        approach_decision_budget=128, curriculum_task='c2_' + stage,easy=bool(easy))
    return dict(record, scenario_id=_identity(record))


def validate_case(case):
    if not isinstance(case, dict) or case.get('version') != VERSION:
        raise ValueError('invalid course-two scene version')
    record = dict(case)
    identity = record.pop('scenario_id', None)
    if identity != _identity(record): raise ValueError('scene sensor/task identity changed')
    sensor = SensorSpec(**record['sensor'])
    from .sensor import validate_sensor
    validate_sensor(sensor)
    if (record['stage'] not in STAGES or not 10 <= sensor.range_m <= 100
            or not (60 <= sensor.fov_deg <= 180 or sensor.fov_deg == 360)
            or record['visibility_class'] not in ('U', 'V', 'F', 'O')
            or not 0 <= record['blind_gap_m'] <= 20
            or not 0 <= record['start_distance_m'] <= sensor.range_m + 20
            or record['resolution_m'] != .2 or record['episode_budget'] != 512):
        raise ValueError('invalid bounded course-two scene')
    return sensor


def build_case(case, platform):
    """Build real terrain and certify a <=40 m navigation-free reference path."""
    from .scene import TerrainGrid, _segment_distance
    from .geometry import polygon_mask
    from .sensor import SensorModel
    sensor = validate_case(case)
    width, height, d = (case[k] for k in ('roi_width_m', 'roi_height_m', 'start_distance_m'))
    a = case['bearing_rad']
    rotation = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    translation = np.asarray(case['translation'])
    local_start = np.array([-width/2-d, 0.])
    polygon = np.array([[-width/2,-height/2], [width/2,-height/2],
        [width/2,height/2], [-width/2,height/2]]) @ rotation.T + translation
    xy = local_start @ rotation.T + translation
    pose = Pose(*xy, case['initial_yaw_rad'])
    context = np.vstack([polygon, xy])
    origin = np.floor((context.min(axis=0)-sensor.range_m-2)/16)*16
    high = np.ceil((context.max(axis=0)+sensor.range_m+2)/16)*16
    if np.any(high-origin > 512): raise ValueError('scene exceeds finite 512 m bound')
    resolution = case['resolution_m']
    nx, ny = np.rint((high-origin)/resolution).astype(int)
    x = origin[0]+(np.arange(nx)+.5)*resolution-translation[0]
    y = origin[1]+(np.arange(ny)+.5)*resolution-translation[1]
    xx = x[None,:]*math.cos(a)+y[:,None]*math.sin(a)
    yy = -x[None,:]*math.sin(a)+y[:,None]*math.cos(a)
    category = case['visibility_class']
    if category == 'O':
        wall_x = -width/2-d/2
        wall_half = height/2+1.
        route = [local_start, [wall_x-2.5, 0.], [wall_x-2.5, wall_half+3.],
                 [wall_x+2.5, wall_half+3.]]
    elif category == 'U':
        route = [local_start, local_start+np.array([case['blind_gap_m']+2., 0.])]
    else:
        route = [local_start]
    if case['family'] == 'cave':
        if case.get('task_scope')=='whole_cave_v1':
            floor=(np.abs(xx)<width/2-.6)&(np.abs(yy)<height/2-.6)
            floor|=(np.abs(yy)<2.8)&(xx>=-width/2)&(xx<-width/2+3)
            floor|=xx < -width/2
        else:
            floor = (np.abs(yy)<2.6) & (xx>local_start[0]-5) & (xx<width/2+5)
            floor |= (np.abs(xx)<width/2+3) & (np.abs(yy)<height/2+3)
            for ax, ay, bx, by in [(local_start[0]*.5, -10., local_start[0]*.5, 10.),
                    *[(*p, *q) for p,q in zip(route, route[1:])]]:
                floor |= _segment_distance(xx, yy, ax, ay, bx, by)<2.6
        heights = np.where(floor, 0., 3.).astype(np.float32)
    else:
        heights = (.02*np.sin(xx*.09+case['seed']%31)+.015*np.cos(yy*.11)).astype(np.float32)
        rng = np.random.default_rng(np.random.SeedSequence([case['seed'], 913]))
        # Sparse obstacles outside the certified approach corridor. Coordinates
        # rotate with the terrain; goal direction does not become a world axis.
        for _ in range(12):
            cx = float(rng.uniform(local_start[0]-5, width/2+5))
            cy = float(rng.choice([-1,1])*rng.uniform(height/2+7, height/2+15))
            heights[(xx-cx)**2+(yy-cy)**2<rng.uniform(.6,1.4)**2] = 1.5
    if category == 'O':
        heights[(np.abs(xx-wall_x)<.5) & (np.abs(yy)<wall_half)] = 3.
    terrain = TerrainGrid.from_heights(heights, resolution, tuple(origin), platform)
    task = TaskSpec(case['scenario_id'], 'map', polygon)
    roi = polygon_mask(terrain.shape, terrain.origin, resolution, task.polygon) & (terrain.intrinsic != 0)
    def visible(p, s):
        observation = SensorModel.observe(terrain, p, s)
        return bool(np.any(roi[observation.rows, observation.cols]))
    initially_visible = visible(pose, sensor)
    omnivisible = visible(pose, replace(sensor, fov_deg=360.))
    realized = 'V' if initially_visible else 'F' if omnivisible else 'U' if d>sensor.range_m else 'O'
    if realized != category: raise ValueError(f'visibility classification mismatch: {category} -> {realized}')
    length = sum(float(np.linalg.norm(np.asarray(q)-p)) for p,q in zip(route,route[1:]))
    if length > 40: raise ValueError('observation reference path exceeds 40 m')
    for p,q in zip(route,route[1:]):
        samples = np.linspace(p,q,max(2,math.ceil(np.linalg.norm(np.asarray(q)-p)/.1)+1)) @ rotation.T + translation
        cols = np.floor((samples[:,0]-origin[0])/resolution).astype(int)
        rows = np.floor((samples[:,1]-origin[1])/resolution).astype(int)
        if not np.all(terrain.navigation[rows,cols] == 1): raise ValueError('reference path is not navigable')
    sx, sy = terrain.world_to_cell(pose.x, pose.y)
    if terrain.navigation[sy,sx] != 1: raise ValueError('scene start is not navigable')
    endpoint_xy = np.asarray(route[-1]) @ rotation.T + translation
    endpoint = Pose(*endpoint_xy, math.atan2(translation[1]-endpoint_xy[1], translation[0]-endpoint_xy[0]))
    if not visible(endpoint, sensor): raise ValueError('reference endpoint cannot observe task')
    descriptor = dict(case, generator_version=VERSION, extent_m=float(max(high-origin)),
        provided_bounds=[*origin, *high], resolution_m=resolution,
        reference_path_m=length, reference_observation_pose=[endpoint.x, endpoint.y, endpoint.yaw],
        approach_distance_budget_m=max(20., 4*length), start_mode='outside')
    terrain.generator_descriptor = MappingProxyType(descriptor)
    return terrain, pose, task, sensor, descriptor
