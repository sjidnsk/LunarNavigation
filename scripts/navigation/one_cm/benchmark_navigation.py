"""真实 acados/native/spawn 的离线闭环，全部周期保留，失败不转为到达。"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import resource
import subprocess
from time import perf_counter, sleep
import numpy as np

from nav1cm.certification import certify
from nav1cm.contracts import WheelLimits
from nav1cm.coordinator import NavigationCoordinator
from nav1cm.execution import execute_action
from nav1cm.fixtures import scenarios
from nav1cm.geometry_native import NativeGeometry
from nav1cm.field_native import NativeDistanceField
from nav1cm.global_worker import GlobalWorker, restore_result
from nav1cm.local_mpcc import MpccPlanner


TIMING_POLICY = 'soft_period_certified_action'


def timing_observations(result, tick_wall_ms, independent):
    """迟到有效动作是软周期观测值，独立审计有效才计入。"""
    late_motion = (tick_wall_ms > 50. and result.action is not None
                   and result.action.source != 'brake' and independent is not None
                   and independent.valid)
    return {'decision_deadline_miss': result.deadline_miss,
            'deadline_miss': tick_wall_ms > 50.,
            'late_certified_motion': bool(late_motion)}


def percentiles(values):
    return dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(values, [50, 95, 99, 100])))) if values else {}


def classify_terminal(status, state, goal):
    """仅分类当前实测终态，不规划、不推进状态。"""
    stopped = abs(state.v)<=.01 and abs(state.w)<=.01
    if status in ('GOAL_IN_OBSTACLE', 'GOAL_CLEARANCE'):
        return 'SAFE_STOP_GOAL_REJECTED' if stopped else None
    if np.linalg.norm(np.array(goal)-[state.x,state.y])<=.1 and stopped:
        return 'ARRIVED'
    return None


def trajectory_data(value):
    if value is None: return None
    return {'times': value.times.tolist(), 'states': value.states.tolist(), 'controls': value.controls.tolist(),
            'mode': value.mode, 'source': value.source, 'digest': value.digest(),
            'map_epoch': value.map_epoch, 'route_version': value.route_version}


def draw_svg(path, scenario, rows, worlds):
    """完整接受路线和实际轨迹；不依赖额外绘图库。"""
    routes = [e['points'] for r in rows for e in r['events'] if e['kind'] == 'ROUTE_ACCEPTED']
    execution = [[r['state']['x'], r['state']['y']] for r in rows]
    if rows: execution.append([rows[-1]['after_state']['x'], rows[-1]['after_state']['y']])
    points = execution + [list(scenario.goal)] + [p for route in routes for p in route]
    if not points: return
    a = np.asarray(points); low = a.min(axis=0)-1.5; high = a.max(axis=0)+1.5
    scale = min(900/(high[0]-low[0]), 600/(high[1]-low[1]))
    def xy(p): return ((p[0]-low[0])*scale+40, (high[1]-p[1])*scale+40)
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="720" viewBox="0 0 1000 720">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="25" y="22">{scenario.name}: accepted global routes (blue), executed (red)</text>']
    for epoch, world in worlds.items():
        for rect in world.blocked:
            x, y = xy((rect.x0/100, rect.y1/100))
            svg.append(f'<rect x="{x}" y="{y}" width="{(rect.x1-rect.x0)/100*scale}" height="{(rect.y1-rect.y0)/100*scale}" fill="gray" opacity=".35"/>')
    for route in routes:
        coords = ' '.join(f'{x:.2f},{y:.2f}' for x,y in map(xy,route))
        svg.append(f'<polyline points="{coords}" fill="none" stroke="#3478ba" stroke-width="2" opacity=".6"/>')
    coords = ' '.join(f'{x:.2f},{y:.2f}' for x,y in map(xy,execution))
    svg.append(f'<polyline points="{coords}" fill="none" stroke="#cf3434" stroke-width="3"/></svg>')
    path.write_text('\n'.join(svg), encoding='utf-8')


def run(build_dir, output_dir, geometry_lib, selected=(), pace=False, max_steps=None, speed_profile='baseline', distance_field_lib=None):
    if not geometry_lib: raise RuntimeError('formal benchmark requires explicit native library')
    out = Path(output_dir); out.mkdir(parents=True, exist_ok=False)
    limits = WheelLimits(linear_speed_mps=.5, angular_speed_rps=.5) if speed_profile=='fast' else WheelLimits()
    backend = NativeGeometry(geometry_lib)
    field_backend = NativeDistanceField(distance_field_lib) if distance_field_lib else None
    started = perf_counter(); planner = MpccPlanner(limits, Path(build_dir)/speed_profile, n=20, dt=.1 if speed_profile=='fast' else .2)
    cold_ms = (perf_counter()-started)*1000
    summaries, all_rows = {}, []
    scenario_list = [s for s in scenarios() if not selected or s.name in selected]
    if speed_profile == 'fast':
        # .5 m/s 档在 2 s 时最大位移 .75 m，增障仍处于完整车身+刹停范围外。
        scenario_list = [replace(s, update_step=40) if s.name in ('new-obstacle','goal-later-blocked') else s for s in scenario_list]
    if not scenario_list: raise ValueError('no selected scenarios')
    with GlobalWorker() as worker:
        for scenario in scenario_list:
            coordinator = NavigationCoordinator(planner, runner=worker, limits=limits, backend=backend,
                allow_unknown=scenario.allow_unknown, clearance_radius_m=scenario.clearance_radius_m, field_backend=field_backend)
            world, state, goal = scenario.world, scenario.start, scenario.goal
            worlds = {world.epoch: world}; rows = []; status = 'STEP_LIMIT'; last_motion = 0; anchor = np.array([state.x,state.y,state.yaw])
            scene_start = perf_counter()
            with (out/(scenario.name+'.jsonl')).open('x', encoding='utf-8') as stream:
                for cycle in range(min(scenario.max_steps, max_steps or scenario.max_steps)):
                    cycle_start = perf_counter()
                    if cycle == scenario.update_step:
                        world = scenario.updated_world or world
                        goal = scenario.updated_goal or goal
                        worlds[world.epoch] = world
                    before = state
                    tick_start = perf_counter()
                    result = coordinator.tick(world, state, goal, cycle*.05)
                    tick_wall_ms = (perf_counter()-tick_start)*1000
                    audit_start = perf_counter()
                    independent = None
                    if result.action is not None:
                        independent = certify(result.action, result.grid, limits, result.action.route_version, backend=backend)
                        if not independent.matches(result.action, result.grid, limits, result.action.route_version):
                            raise RuntimeError('independent action audit rejected')
                        if not np.allclose(result.action.states[0,:5], list(vars(state).values()), atol=1e-6, rtol=0):
                            raise RuntimeError('independent initial state audit rejected')
                        state = execute_action(result.action, state)
                    audit_ms = (perf_counter()-audit_start)*1000
                    # 独立复核与离散物理推进属离线审计；运行时 action 门禁已包含在 tick.total。
                    raw_global = result.global_result
                    global_record = None
                    if raw_global:
                        global_record = {k:v for k,v in raw_global.items() if k != 'route'}
                        global_record['route'] = asdict(raw_global['route']) if raw_global['route'] else None
                        if global_record['route']: global_record['route']['points'] = raw_global['route'].points.tolist()
                    reentry = float(np.linalg.norm(result.reference.points[0]-[before.x,before.y])) if result.reference is not None else None
                    row = {'case': scenario.name, 'cycle': cycle, 'simulation_s': cycle*.05, 'world_epoch': world.epoch,
                        'goal': goal, 'state': asdict(before), 'after_state': asdict(state), 'status': result.status,
                        'route_version': coordinator.route.version if coordinator.route else None,
                        'route_map_epoch': coordinator.route.map_epoch if coordinator.route else None,
                        'grid_digest': result.grid.digest(), 'grid_origin': result.grid.origin_xy,
                        'allow_unknown': scenario.allow_unknown, 'candidate': trajectory_data(result.candidate),
                        'action': trajectory_data(result.action), 'certificate': asdict(result.certificate) if result.certificate else None,
                        'action_certificate': asdict(result.action_certificate) if result.action_certificate else None,
                        'audit_valid': independent.valid if independent else False, 'offline_audit_step_ms': audit_ms,
                        'events': result.events, 'timings_ms': result.timings_ms,
                        'tick_call_wall_ms': tick_wall_ms,
                        **timing_observations(result, tick_wall_ms, independent),
                        'global_result': global_record, 'progress_s': result.reference.progress_s, 'reentry_error_m': reentry,
                        'rss_peak_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                        'worker_rss_peak_kib': resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss}
                    stream.write(json.dumps(row)+'\n'); stream.flush(); rows.append(row)
                    if result.action is None:
                        status = 'FAILED_NO_CERTIFIED_ACTION'; break
                    stopped = abs(state.v)<=.01 and abs(state.w)<=.01
                    terminal = classify_terminal(result.status, state, goal)
                    if terminal is not None:
                        status = terminal; break
                    position = np.array([state.x,state.y,state.yaw])
                    if np.linalg.norm(position-anchor)>.02:
                        anchor, last_motion = position, cycle
                    if cycle-last_motion>=240 and stopped and cycle>scenario.update_step+20:
                        status = 'SAFE_STOP_NOT_ARRIVED'; break
                    # 首轮 spawn 与队列等待也是真实周期；无路线时给 worker 有界墙钟机会。
                    delay = max(0., .05-(perf_counter()-cycle_start)) if pace else (.005 if coordinator.route is None else 0.)
                    if delay: sleep(delay)
            tail_global = []
            drain_start = perf_counter()
            while worker.busy:
                raw = worker.poll()
                if raw is None:
                    sleep(.01)
                else:
                    result = restore_result(raw)
                    result['route'] = asdict(result['route']) if result['route'] else None
                    if result['route']: result['route']['points'] = result['route']['points'].tolist()
                    tail_global.append(result)
            (out/(scenario.name+'-global-tail.json')).write_text(json.dumps(tail_global, indent=2), encoding='utf-8')
            all_rows.extend(rows)
            global_records = [r['global_result'] for r in rows if r['global_result']]
            global_records.extend(tail_global)
            summaries[scenario.name] = {'status': status, 'navigation_arrived': status=='ARRIVED',
                'expected': scenario.expected, 'steps': len(rows), 'simulation_s': len(rows)*.05,
                'configured_update_step': scenario.update_step,
                'wall_s': perf_counter()-scene_start, 'state': asdict(state), 'goal': goal,
                'clearance_radius_m': scenario.clearance_radius_m,
                'timings_ms': percentiles([r['timings_ms']['total'] for r in rows]),
                'tick_call_wall_ms': percentiles([r['tick_call_wall_ms'] for r in rows]),
                'timing_policy': TIMING_POLICY,
                'late_certified_motion': sum(r['late_certified_motion'] for r in rows),
                'actual_peak_abs_v': max(abs(r['after_state']['v']) for r in rows),
                'actual_peak_abs_w': max(abs(r['after_state']['w']) for r in rows),
                'deadline_misses': sum(r['deadline_miss'] for r in rows),
                'timeout_rate': sum(r['deadline_miss'] for r in rows)/len(rows),
                'actions': dict(Counter(r['action']['source'] if r['action'] else 'NONE' for r in rows)),
                'local_reasons': dict(Counter(e['reason'] for r in rows for e in r['events'] if e['kind'] in ('LOCAL_RESULT','CANDIDATE_REJECTED','ACTION_REJECTED','BRAKE_REJECTED'))),
                'replans': dict(Counter(e['reason'] for r in rows for e in r['events'] if e['kind']=='GLOBAL_SUBMITTED')),
                'global_reasons': dict(Counter(r['reason_code'] for r in global_records)),
                'global_search_ms': percentiles([r['timing']['total_s']*1000 for r in global_records]),
                'global_queue_ms': percentiles([r['queue_ms'] for r in global_records]),
                'global_handoff_ms': percentiles([r['handoff_ms'] for r in global_records]),
                'worker_pids': sorted(set(r['worker_pid'] for r in global_records)),
                'worker_rss_peak_kib': max((r.get('worker_rss_peak_kib') or 0 for r in global_records), default=0),
                'post_scene_drain_ms': (perf_counter()-drain_start)*1000,
                'max_reentry_error_m': max(r['reentry_error_m'] or 0 for r in rows),
                'all_executed_actions_audited': all(r['audit_valid'] for r in rows if r['action']),
                'worlds': {epoch: asdict(w) for epoch,w in worlds.items()}}
            draw_svg(out/(scenario.name+'.svg'), scenario, rows, worlds)
            (out/'progress.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
            print(json.dumps({scenario.name:summaries[scenario.name]}, ensure_ascii=False), flush=True)
    sources = {str(p.relative_to(Path(__file__).parent)): hashlib.sha256(p.read_bytes()).hexdigest()
               for p in Path(__file__).parent.rglob('*') if p.suffix in ('.py','.cpp')}
    acados = os.environ.get('ACADOS_SOURCE_DIR')
    acados_sha = subprocess.check_output(['git','-C',acados,'rev-parse','HEAD'], text=True).strip() if acados else None
    summary = {'cases': summaries, 'cold_build_ms': cold_ms, 'parent_pid': os.getpid(), 'platform': platform.platform(),
        'distance_field_backend': 'native' if field_backend else 'scipy',
        'distance_field_library_path': str(field_backend.library_path) if field_backend else None,
        'distance_field_library_sha256': field_backend.library_sha256 if field_backend else None,
        'python': platform.python_version(), 'source_sha256': sources, 'acados_sha': acados_sha,
        'numpy': np.__version__, 'scipy': importlib.metadata.version('scipy'),
        'casadi': importlib.metadata.version('casadi'), 'geometry_library_sha256': hashlib.sha256(Path(geometry_lib).read_bytes()).hexdigest(),
        'pace': pace, 'simulation_dt_s': .05, 'limits': asdict(limits), 'all_samples': len(all_rows),
        'speed_profile': speed_profile, 'horizon_n': planner.N, 'horizon_dt_s': planner.dt,
        'horizon_s': planner.N*planner.dt, 'model_fingerprint': planner.model_fingerprint,
        'tick_call_wall_ms': percentiles([r['tick_call_wall_ms'] for r in all_rows]),
        'timing_policy': TIMING_POLICY,
        'late_certified_motion': sum(r['late_certified_motion'] for r in all_rows),
        'all_cycles_ms': percentiles([r['timings_ms']['total'] for r in all_rows]),
        'all_stages_ms': {key: percentiles([r['timings_ms'].get(key, 0.) for r in all_rows])
                          for key in sorted(set().union(*(r['timings_ms'].keys() for r in all_rows)))},
        'strict_50ms_pass': not any(r['deadline_miss'] for r in all_rows),
        'timeout_rate': sum(r['deadline_miss'] for r in all_rows)/len(all_rows),
        'rss_peak_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        'not_run': ['ROS 2 package build/interfaces','DDS','rosbag','physical sensors/actuators','real vehicle','P3','40 km route following'],
        'timing_contract': 'tick.total is completed-action decision time, includes map/SDF/plan/full certificate/action-prefix certificate/IPC/scheduling/brake; outer tick_call_wall_ms includes function return; strict_50ms_pass is only an observed performance result, never an action-selection gate; late_certified_motion counts independently audited non-brake actions above 50 ms; offline audit, model execution, JSON/SVG IO excluded and labelled; no removed warmup samples'}
    (out/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--build-dir', required=True); parser.add_argument('--output-dir', required=True)
    parser.add_argument('--geometry-lib', required=True); parser.add_argument('--case', action='append', default=[])
    parser.add_argument('--pace', action='store_true'); parser.add_argument('--max-steps', type=int)
    parser.add_argument('--distance-field-lib')
    parser.add_argument('--speed-profile', choices=('baseline','fast'), default='baseline')
    args = parser.parse_args()
    run(args.build_dir, args.output_dir, args.geometry_lib, args.case, args.pace, args.max_steps, args.speed_profile, args.distance_field_lib)
