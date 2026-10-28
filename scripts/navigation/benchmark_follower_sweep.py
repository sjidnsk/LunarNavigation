#!/usr/bin/env python3
"""离线跟随器配对比较；独立闭格圆弧 oracle，不启动 ROS 或正式队列。

--baseline 指向基线 mppi_path_follower.py；仅该模块替换，父类与当前相同。
--algorithm-shim 只替换导入所需 ROS 消息占位类，不验证消息/节点/ROS。
"""
import argparse
from collections import deque
from copy import deepcopy
from dataclasses import asdict
import hashlib
import importlib
import importlib.util
import json
import math
from pathlib import Path
import platform
import sys
from time import perf_counter_ns
import types

import numpy as np


def arc_endpoint(pose, v, w, dt):
    x, y, yaw = pose
    turn = w*dt
    if abs(w) < 1e-12:
        return (x+v*dt*math.cos(yaw), y+v*dt*math.sin(yaw), yaw+turn)
    return (x+v/w*(math.sin(yaw+turn)-math.sin(yaw)),
            y+v/w*(math.cos(yaw)-math.cos(yaw+turn)), yaw+turn)


def arc_hits_box(pose, v, w, dt, lo, hi):
    """解析常 twist 圆弧与闭矩形相交；独立于控制器地图实现。"""
    start = pose[:2]
    end = arc_endpoint(pose, v, w, dt)[:2]
    inside = lambda p: all(lo[i]-1e-12 <= p[i] <= hi[i]+1e-12 for i in (0, 1))
    if inside(start) or inside(end):
        return True
    if abs(w) < 1e-12 or abs(v) < 1e-15:
        enter, leave = 0., 1.
        for i in (0, 1):
            d = end[i]-start[i]
            if abs(d) < 1e-15:
                if start[i] < lo[i] or start[i] > hi[i]:
                    return False
            else:
                u, z = sorted(((lo[i]-start[i])/d, (hi[i]-start[i])/d))
                enter, leave = max(enter, u), min(leave, z)
        return enter <= leave+1e-12
    radius = v/w
    center = (pose[0]-radius*math.sin(pose[2]), pose[1]+radius*math.cos(pose[2]))
    initial = (start[0]-center[0], start[1]-center[1])
    turn = w*dt
    def on_arc(p):
        q = (p[0]-center[0], p[1]-center[1])
        angle = math.atan2(initial[0]*q[1]-initial[1]*q[0], initial[0]*q[0]+initial[1]*q[1])
        if turn >= 0:
            return angle % (2*math.pi) <= turn+1e-11
        return (-angle) % (2*math.pi) <= -turn+1e-11
    for axis in (0, 1):
        other = 1-axis
        for edge in (lo[axis], hi[axis]):
            square = radius*radius-(edge-center[axis])**2
            if square < -1e-12:
                continue
            root = math.sqrt(max(0., square))
            for value in (center[other]-root, center[other]+root):
                p = [0., 0.]
                p[axis], p[other] = edge, value
                if lo[other]-1e-12 <= value <= hi[other]+1e-12 and on_arc(p):
                    return True
    return False


def obstacle_hit(states, origin, resolution, pose, v, w, dt):
    """检查物理禁行 M2 和地图外；UNKNOWN/assumed 单独计授权越界。"""
    end = arc_endpoint(pose, v, w, dt)
    # 行程长给出圆弧的保守包围框；只影响候选格，不替代精确相交。
    reach = abs(v)*dt+1e-10
    low = np.floor((np.asarray(pose[:2])-reach-origin)/resolution).astype(int)
    high = np.floor((np.asarray(pose[:2])+reach-origin)/resolution).astype(int)
    h, width = states.shape
    touched = []
    for iy in range(low[1], high[1]+1):
        for ix in range(low[0], high[0]+1):
            lo = (origin[0]+ix*resolution, origin[1]+iy*resolution)
            hi = (lo[0]+resolution, lo[1]+resolution)
            if arc_hits_box(pose, v, w, dt, lo, hi):
                value = int(states[iy, ix]) if 0 <= iy < h and 0 <= ix < width else 2
                touched.append((iy, ix, value))
    return any(c[2] == 2 for c in touched), touched, end


def fixtures():
    cases = []
    def add(name, points, pose=(0., 0., 0.), states=None, origin=(-2., -2.), resolution=.1, **kw):
        cases.append(dict(name=name, points=points, pose=pose,
                          states=np.ones((60, 60), np.uint8) if states is None else states,
                          origin=origin, resolution=resolution, **kw))
    add('straight', [(0, 0, 0), (2, 0, 0)])
    add('corner', [(0, 0, 0), (.7, 0, math.pi/2), (.7, .7, math.pi/2)])
    add('sharp_turn', [(0, 0, 0), (.7, 0, 2.7), (.1, .3, 2.7)])
    corridor = np.full((9, 130), 2, np.uint8)
    corridor[1:8] = 1
    add('narrow', [(0, 0, 0), (2, 0, 0)], (0, 0, .06), corridor, (-.5, -.09), .02)
    blocked = np.ones((60, 60), np.uint8)
    blocked[:, 28] = 2
    add('blocked', [(0, 0, 0), (2, 0, 0)], states=blocked)
    add('map_change', [(0, 0, 0), (2, 0, 0)], event=(35, .8, 2))
    add('map_unknown_change', [(0, 0, 0), (2, 0, 0)], event=(35, .8, 0))
    unknown = np.ones((60, 60), np.uint8)
    unknown[:, 22:25] = 0
    add('unknown', [(0, 0, 0), (2, 0, 0)], states=unknown)
    assumed = np.zeros_like(unknown, dtype=bool)
    assumed[:, 22:25] = True
    add('assumed', [(0, 0, 0), (2, 0, 0)], states=unknown.copy(), assumed=assumed)
    brake = np.ones((60, 60), np.uint8)
    brake[:, 24] = 2
    add('braking', [(0, 0, 0), (2, 0, 0)], states=brake, velocity=(.2, 0.))
    mm = np.ones((12, 12), np.uint8)
    mm[1, 0] = 2
    pose = (8.9423022725, 4.9381833596, 2.450581676)
    direction = (math.cos(pose[2]), math.sin(pose[2]))
    add('case09_mm_constructed', [pose, (pose[0]+.4*direction[0], pose[1]+.4*direction[1], pose[2])],
        pose, mm, (8.940919534-.2, 4.940919534-.4), .2,
        note='constructed near-corner regression, not historical replay: original reference unavailable; west M2 cell')
    return cases


def load_api(baseline, algorithm_shim=False):
    root = Path(__file__).resolve().parents[2]
    package = root/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller'
    if algorithm_shim:
        pkg = types.ModuleType('lunar_pure_wheeled_controller')
        pkg.__path__ = [str(package)]
        sys.modules[pkg.__name__] = pkg
        msg = types.ModuleType('lunar_planning_msgs.msg')
        msg.MotionReference = type('MotionReference', (), {})
        parent = types.ModuleType('lunar_planning_msgs')
        parent.msg = msg
        sys.modules[parent.__name__] = parent
        sys.modules[msg.__name__] = msg
    else:
        sys.path.insert(0, str(package.parent))
    prefix = 'lunar_pure_wheeled_controller.'
    tracking = importlib.import_module(prefix+'tracking')
    mppi = importlib.import_module(prefix+'mppi')
    grid = importlib.import_module(prefix+'mppi_map').GridMap
    new = importlib.import_module(prefix+'mppi_path_follower').MppiPathFollower
    spec = importlib.util.spec_from_file_location(prefix+'_sweep_baseline', baseline)
    old_module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = old_module
    spec.loader.exec_module(old_module)
    return types.SimpleNamespace(TrackingPolicy=tracking.TrackingPolicy, TrackingState=tracking.TrackingState,
                                 MppiPolicy=mppi.MppiPolicy, GridMap=grid,
                                 classes={'baseline': old_module.MppiPathFollower, 'current': new})


def make_controller(api, case, backend, seed):
    policy = api.TrackingPolicy(goal_position_tolerance_m=.1, goal_yaw_tolerance_rad=.05)
    config = api.MppiPolicy(seed=seed)
    obj = api.classes[backend](policy, config)
    obj.set_path(case['points'], identity=(b'sweep_benchmark', 1))
    obj.set_map(api.GridMap(case['states'], case['resolution'], case['origin'], assumed=case.get('assumed')))
    return obj


def percentile(samples):
    return dict(n=len(samples), **{f'p{q}_ms': float(np.percentile(samples, q)) for q in (50, 95, 99)}, max_ms=max(samples))


def closed_loop(api, case, backend, ticks, seed):
    obj = make_controller(api, case, backend, seed)
    p = obj.policy
    dt = .05
    states = case['states'].copy()
    pose = case['pose']
    v, w = case.get('velocity', (0., 0.))
    delay = math.ceil(p.control_response_time_s/dt)
    queue = deque([(v, w)]*delay)
    if v or w:
        for j in range(delay+1):
            obj.note_command(v, w, stamp_s=(j-delay)*dt)
    trace, timings = [], []
    distance = 0.
    collision = violation = False
    complete = False
    reason = 'TIME_BUDGET'
    stopped_ticks = 0
    for tick in range(ticks):
        if case.get('event') and tick == case['event'][0]:
            _, x, value = case['event']
            states[:, int(math.floor((x-case['origin'][0])/case['resolution']))] = value
            obj.set_map(api.GridMap(states, case['resolution'], case['origin'], revision=2))
        state = api.TrackingState(*pose, v, w)
        start = perf_counter_ns()
        result = obj.update(state, dt, now_s=tick*dt)
        timings.append((perf_counter_ns()-start)/1e6)
        command = (result.command.linear_x_mps, result.command.angular_z_radps)
        obj.note_command(*command, stamp_s=tick*dt)
        queue.append(command)
        target_v, target_w = queue.popleft()
        target_v = float(np.clip(target_v, -p.max_reverse_mps, p.max_linear_mps))
        target_w = float(np.clip(target_w, -p.max_angular_radps, p.max_angular_radps))
        accel = p.max_linear_accel_mps2 if abs(target_v) > abs(v) else p.max_linear_decel_mps2
        v += float(np.clip(target_v-v, -accel*dt, accel*dt))
        bound = p.max_angular_accel_radps2*dt
        w += float(np.clip(target_w-w, -bound, bound))
        collision, touched, end = obstacle_hit(states, case['origin'], case['resolution'], pose, v, w, dt)
        if abs(v) > 1e-12 and any(c[2] == 0 for c in touched):
            violation = True
        distance += abs(v)*dt
        pose = end
        trace.append([tick*dt, *pose, v, w, *command, result.phase])
        if collision:
            reason = 'COLLISION'
            break
        stopped = abs(v) <= p.stopped_linear_mps and abs(w) <= p.stopped_angular_radps
        drained = all(abs(a) <= 1e-12 and abs(b) <= 1e-12 for a, b in queue)
        stopped_ticks = stopped_ticks+1 if stopped and drained and max(map(abs, command)) <= 1e-12 else 0
        if result.phase == 'COMPLETED' and stopped and drained:
            complete, reason = True, 'COMPLETED'
            break
        # 等待真实停车及队列排空，FAILED 也不提前截断制动验证。
        if stopped_ticks >= 12:
            reason = result.command.failure_reason or 'SAFE_STOP_NO_PROGRESS'
            break
    safe_stop = not collision and not complete and stopped_ticks >= 12
    return dict(case=case['name'], backend=backend, complete=complete, safe_stop=safe_stop,
                collision=collision, authorization_violation=violation, reason=reason,
                ticks=len(trace), distance_m=distance, progress_m=obj.progress_m,
                goal_distance_m=math.hypot(pose[0]-case['points'][-1][0], pose[1]-case['points'][-1][1]),
                final_pose=pose, execution_clearance_m=p.execution_clearance_m,
                control_response_time_s=p.control_response_time_s, compute_ms=percentile(timings),
                note=case.get('note'), trace=trace)


def timing_comparison(api, case, iterations, seed):
    """每次从同一未更新快照计时首轮求解/拒绝，预热副本随即丢弃。"""
    bases = {name: make_controller(api, case, name, seed) for name in api.classes}
    state = api.TrackingState(*case['pose'], *case.get('velocity', (0., 0.)))
    timed_states = case['states'].copy()
    event = case.get('event')
    map_update_phase = 'initial_map'
    revision = 1
    if event:
        _, x, value = event
        column = int(math.floor((x-case['origin'][0])/case['resolution']))
        timed_states[:, column] = value
        revision = 2
        map_update_phase = 'post_event_fixed_initial_state'
        for obj in bases.values():
            obj.set_map(api.GridMap(timed_states, case['resolution'], case['origin'], revision=revision,
                                   assumed=case.get('assumed')))
    snapshot = {name: dict(phase=obj.phase, failure=obj.failure,
                           selected=obj.diagnostics.get('selected'), map_revision=obj.map.revision)
                for name, obj in bases.items()}
    for obj in bases.values():
        for _ in range(3):
            deepcopy(obj).update(state, .05, now_s=.2)
    updates = {name: [] for name in bases}
    setters = {name: [] for name in bases}
    observed = {name: {} for name in bases}
    constructions = []
    for i in range(iterations):
        order = ('baseline', 'current') if i % 2 == 0 else ('current', 'baseline')
        for name in order:
            obj = deepcopy(bases[name])
            start = perf_counter_ns()
            result = obj.update(state, .05, now_s=.2)
            updates[name].append((perf_counter_ns()-start)/1e6)
            key = result.phase+'/'+str(obj.diagnostics.get('selected'))
            observed[name][key] = observed[name].get(key, 0)+1
            start = perf_counter_ns()
            grid = api.GridMap(timed_states, case['resolution'], case['origin'], revision=revision+i+1, assumed=case.get('assumed'))
            constructions.append((perf_counter_ns()-start)/1e6)
            start = perf_counter_ns()
            obj.set_map(grid)
            setters[name].append((perf_counter_ns()-start)/1e6)
    result = dict(case=case['name'], update={k: percentile(v) for k, v in updates.items()},
                  set_map={k: percentile(v) for k, v in setters.items()}, grid_copy=percentile(constructions),
                  observed_phase_and_selection=observed, input_snapshot=snapshot, state=asdict(state),
                  map_update_phase=map_update_phase,
                  timed_map_state_counts={str(int(value)): int(count) for value, count in
                                          zip(*np.unique(timed_states, return_counts=True))},
                  input_protocol='same initial state, path and seed; fresh pre-update controller snapshot; '
                                 'event cases use changed map at initial pose, not event-time closed-loop state',
                  comparison_boundary='phase/selection may differ: timings include first rejection versus solve; '
                                      'do not interpret differing output paths as equal optimizer work')
    difference = np.asarray(updates['current'])-updates['baseline']
    result['paired_current_minus_baseline_ms'] = dict(p50=float(np.percentile(difference, 50)),
                                                     p95=float(np.percentile(difference, 95)), mean=float(difference.mean()))
    result['p50_ratio'] = result['update']['current']['p50_ms']/max(result['update']['baseline']['p50_ms'], 1e-12)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timing-iterations', type=int, default=100)
    parser.add_argument('--closed-loop-ticks', type=int, default=350)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--algorithm-shim', action='store_true')
    parser.add_argument('--timing-only', action='store_true', help='skip closed loop; write timing-summary.json, preserving summary.json and traces')
    parser.add_argument('--cases', nargs='+')
    args = parser.parse_args()
    if args.timing_iterations < 100 or args.closed_loop_ticks < 1:
        parser.error('timing-iterations must be >=100; closed-loop-ticks must be positive')
    if sys.platform == 'win32' and args.output.resolve().drive.upper() != 'D:':
        parser.error('output must be an explicit managed D: directory')
    api = load_api(args.baseline.resolve(), args.algorithm_shim)
    args.output.mkdir(parents=True, exist_ok=True)
    cases = [c for c in fixtures() if not args.cases or c['name'] in args.cases]
    if not cases:
        parser.error('no matching cases')
    current = Path(__file__).resolve().parents[2]/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/mppi_path_follower.py'
    summary = dict(scope='algorithm only; ROS/Jazzy/Humble/Orin/vehicle NOT_RUN',
                   oracle='independent exact closed-cell constant-twist arc intersection, footprint-inflated M2 map; unknown translation separately counted',
                   timing_protocol='3 disposable-copy warmups; immutable pre-update snapshots, deepcopy outside timer; '
                                   'fixed initial state/path/seed, AB/BA alternating; event maps applied before snapshot; '
                                   'GridMap copy separate from set_map; phases and selection recorded per backend',
                   baseline_sha256=hashlib.sha256(args.baseline.read_bytes()).hexdigest(),
                   current_sha256=hashlib.sha256(current.read_bytes()).hexdigest(),
                   shared_dependencies_sha256={name: hashlib.sha256((current.parent/name).read_bytes()).hexdigest()
                                               for name in ('mppi.py', 'mppi_map.py', 'tracking.py')},
                   python=platform.python_version(), platform=platform.platform(), numpy=np.__version__,
                   seed=args.seed, mppi_policy=asdict(api.MppiPolicy(seed=args.seed)),
                   timing_only=args.timing_only,
                   closed_loop=[], timing=[])
    for case in cases:
        for backend in (() if args.timing_only else ('baseline', 'current')):
            result = closed_loop(api, case, backend, args.closed_loop_ticks, args.seed)
            (args.output/f"{case['name']}-{backend}.json").write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
            summary['closed_loop'].append({k: v for k, v in result.items() if k != 'trace'})
            print(json.dumps(summary['closed_loop'][-1], ensure_ascii=False), flush=True)
        timing = timing_comparison(api, case, args.timing_iterations, args.seed)
        summary['timing'].append(timing)
        summary_name = 'timing-summary.json' if args.timing_only else 'summary.json'
        (args.output/summary_name).write_text(json.dumps(summary, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print(json.dumps(timing), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
