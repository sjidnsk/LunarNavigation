"""离线判定原始 DDS/状态/进程证据；不从到达反推规划成功。"""
import csv
import json
import math
from pathlib import Path


def path_valid(path, *, local):
    if path['frame'] != 'map':
        return False
    previous = None
    for pose in path['poses']:
        if pose['frame'] != 'map' or not all(math.isfinite(x) for x in [pose['x'], pose['y'], pose['z'], *pose['q']]):
            return False
        if abs(sum(x*x for x in pose['q'])-1.) > 1e-6:
            return False
        if local and ((previous is None and pose['stamp_ns'] != path['stamp_ns']) or
                      (previous is not None and pose['stamp_ns'] <= previous)):
            return False
        previous = pose['stamp_ns']
    return True


def formal_acceptance(tick):
    raw = tick.get('global_result')
    if not raw or tuple(raw.get(k) for k in ('planning_outcome', 'reason_code', 'has_reference')) != (0, 'PLAN_FOUND', True):
        return False
    if raw.get('requested_goal') != tick.get('goal'):
        return False
    received = next((e for e in tick['events'] if e['kind']=='GLOBAL_RESULT_RECEIVED'), {})
    if (received.get('accepted_generation_at_submit') != tick.get('generation') or
            received.get('goal_revision_at_submit') != tick.get('goal_revision')):
        return False
    return any(e['kind']=='ROUTE_ACCEPTED' and e.get('version')==raw.get('requested_version') and
               e.get('map_epoch')==raw.get('requested_epoch') and e.get('points') and
               all(abs(a-b)<1e-8 for a,b in zip(e['points'][-1],tick['goal'])) for e in tick['events'])


def enclosed_failure(raw):
    return (raw.get('planning_outcome')==1 and raw.get('reason_code')=='DOMAIN_EXHAUSTED' and
            raw.get('has_reference') is False and raw.get('requested_goal')==[0., 0.])


def graph_valid(graph, case):
    expected = {
        '/Car/T3/mapping/grid_map': ('grid_map_msgs/msg/GridMap','nav1cm_bounded_fixture'),
        '/Car/T3/localization/odometry': ('nav_msgs/msg/Odometry','nav1cm_bounded_path_consumer'),
        '/Car/T4/navigation/goal': ('geometry_msgs/msg/PoseStamped','nav1cm_bounded_fixture'),
        '/Car/T4/planning/global_route': ('nav_msgs/msg/Path','lunar_nav1cm_planner'),
        '/Car/T4/planning/local_path': ('nav_msgs/msg/Path','lunar_nav1cm_planner'),
        '/P4/input/map_to_odom' if case=='legacy_tf' else '/tf_static': ('tf2_msgs/msg/TFMessage','nav1cm_bounded_fixture'),
    }
    if case=='empty_smoke':
        expected = {k:v for k,v in expected.items() if k.endswith(('odometry','local_path'))}
        expected['/Car/T4/planning/local_path'] = ('nav_msgs/msg/Path','nav1cm_bounded_fixture')
    return all(graph.get('types',{}).get(topic)==[type_name] and
               any(p['node_name']==publisher and p['topic_type']==type_name for p in graph.get('publishers',{}).get(topic,[]))
               for topic,(type_name,publisher) in expected.items())


def load_rows(folder, role):
    path = Path(folder) / (role+'.jsonl')
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()] if path.exists() else []


def apply_supervision_gate(summary, folder, error=None):
    """监督失败是独立否决证据；保留其他评分及原始错误供在线/离线共用。"""
    error = error if error is not None else summary.get('supervisor_error')
    evidence = Path(folder) / 'supervisor-error.txt'
    if error is None and evidence.exists():
        try:
            error = evidence.read_text(encoding='utf-8', errors='replace') or 'supervisor-error.txt exists (empty)'
        except OSError as exc:
            error = f'supervisor-error.txt unreadable: {exc}'
    summary.setdefault('checks', {})['supervisor_integrity'] = error is None
    summary['supervisor_error'] = error
    summary['pass_'] = bool(summary.get('pass_', False) and error is None)
    return summary


def held_terminal(rows, goal, lo, hi):
    start = None
    for row in rows:
        t = row['elapsed_s']
        valid = (lo <= t <= hi and math.hypot(row['x']-goal[0], row['y']-goal[1]) <= .1 and
                 abs(math.atan2(math.sin(row['yaw']-goal[2]), math.cos(row['yaw']-goal[2]))) <= .05 and
                 abs(row['v']) <= .01 and abs(row['w']) <= .01)
        start = t if valid and start is None else start if valid else None
        if start is not None and t-start >= .3:
            return True
    return False


def analyze(folder, case, start, process_results, stop_s):
    planner, consumer, fixture = [load_rows(folder, name) for name in ('planner', 'consumer', 'fixture')]
    with (Path(folder)/'state.csv').open(encoding='utf-8') as stream:
        states = [{k: float(v) if k not in ('mode', 'collision_free') else v for k,v in row.items()} for row in csv.DictReader(stream)]
    paths = [r for r in consumer if r['kind'] == 'PATH_RECEIVED']
    local = [r for r in paths if r['stream'] == 'local']
    global_paths = [r for r in paths if r['stream'] == 'global' and r['message']['poses']]
    ticks = [r for r in planner if r['kind'] == 'TICK']
    notices = [r for r in planner if r['kind'] == 'NOTICE']
    formal = [r for r in ticks if formal_acceptance(r)]
    checks = dict(process_exit=all(p['returncode']==0 and not p['escalated'] and p['remaining_group_pids']==[] for p in process_results),
                  dds_graph_types=any(r['kind']=='GRAPH' and graph_valid(r,case) for r in consumer),
                  independent_consumer=any(r['kind']=='EXIT' and r['forbidden_modules']==[] for r in consumer),
                  collision_free=bool(states) and all(r['collision_free']=='True' for r in states),
                  numeric_motion_bounds=bool(states) and all(abs(r['v'])<=.200001 and abs(r['w'])<=1.000001 and
                      abs(r['a'])<=.500001 and abs(r['alpha'])<=.500001 and
                      (abs(r['w'])<=abs(r['v'])+1e-7 or abs(r['v'])<1e-8) for r in states))
    if case != 'empty_smoke':
        closes = [r for r in planner if r['kind']=='CLOSE']
        checks.update(installed_only=any(r['kind']=='INSTALL' and '/install/' in r['nav1cm_file'] for r in planner),
                      workers_clean=bool(closes) and not closes[-1]['remaining_children'] and
                          not closes[-1]['result']['threads_alive'] and not closes[-1]['forbidden_modules'] and
                          all(not x['terminated_pids'] and not x['surviving_pids'] for x in closes[-1]['result']['workers'].values() if x),
                      path_contract=bool(local) and all(path_valid(r['message'],local=r['stream']=='local') for r in paths),
                      formal_global=bool(formal),
                      startup_empty=any(not r['message']['poses'] and r['monotonic_s']-start<2. for r in local),
                      local_grid=any(r['notice']=='LOCAL_GRID' and json.loads(r['detail'])['resolution_m']==.01 and
                                     json.loads(r['detail'])['shape']==[500,500] for r in notices),
                      no_runtime_error=not any(r['notice'] in ('TICK_ERROR','MAP_REJECTED') for r in notices))
    if case in ('contract','legacy_tf'):
        cutoff = 12. if case=='contract' else 5.
        checks.update(zero_tf=all(r['stamp_ns']==0 for r in fixture if r['kind']=='TF_SENT'),
                      nonidentity_tf_before=any(r['message']['poses'] and abs(r['message']['poses'][0]['x']-.15)<1e-6 and
                                               abs(r['message']['poses'][0]['y']+.08)<1e-6 for r in local if r['monotonic_s']-start<cutoff),
                      same_zero_tf_updated=any(r['message']['poses'] and abs(r['message']['poses'][0]['x']-.18)<1e-6 and
                                              abs(r['message']['poses'][0]['y']+.05)<1e-6 for r in local if r['monotonic_s']-start>cutoff))
        if case=='contract':
            applied = [json.loads(r['detail'])['source_resolution_m'] for r in notices if r['notice']=='MAP_APPLIED']
            checks['source_resolutions_applied'] = all(any(abs(x-y)<1e-10 for x in applied) for y in (.2,.05,.03,.01,.005))
    if case=='lifecycle':
        sent = {r['name']:r for r in fixture if r['kind']=='GOAL_SENT'}
        checks['only_once_A_then_B'] = sum(r['kind']=='GOAL_SENT' and r.get('name')=='A' for r in fixture)==1
        checks['invalid_B_retains_A'] = (any(r['notice']=='GOAL_REJECTED' for r in notices) and
            any(sent['B-invalid']['ros_stamp_ns'] < r['message']['stamp_ns'] < sent['enclosed']['ros_stamp_ns'] and
                math.hypot(r['message']['poses'][-1]['x']+2.6,r['message']['poses'][-1]['y']+.9)<1e-6 for r in global_paths))
        no_path = [r['global_result'] for r in ticks if r['global_result'] and r['monotonic_s']-start<17. and
                   r['goal']==[0.,0.] and r['global_result']['has_reference'] is False]
        checks['enclosed_explicit_failure'] = any(enclosed_failure(r) for r in no_path)
        checks['no_old_route_for_enclosed_goal'] = not any(
            r['message']['stamp_ns']>sent['enclosed']['ros_stamp_ns']+300_000_000 and
            r['monotonic_s']-start<17. for r in global_paths)
        checks['goal_retained_after_open'] = (sum(r['kind']=='GOAL_SENT' and r.get('name')=='enclosed' for r in fixture)==1 and
             any(r['monotonic_s']-start>18. and math.hypot(r['message']['poses'][-1]['x'],r['message']['poses'][-1]['y'])<1e-6 for r in global_paths))
    if case in ('forward_spin','reverse'):
        forward = case=='forward_spin'
        checks['translated_at_least_30cm'] = (max(r['x'] for r in states)>.3 if forward else min(r['x'] for r in states)<-.3)
        checks['position_yaw_stopped_hold'] = held_terminal(states,(.6 if forward else -.6,0.,0.),3.,10. if forward else 14.)
        if not forward:
            checks['negative_velocity_and_body_yaw'] = min(r['v'] for r in states)<-.1 and any(
                len(r['message']['poses'])>1 and r['message']['poses'][-1]['x']<r['message']['poses'][0]['x']-.1 and
                abs(r['message']['poses'][0]['q'][2])<.05 for r in local)
        else:
            checks['terminal_spin_hold'] = held_terminal(states,(.6,0.,.6),10.,17.)
            phase = [r for r in states if 10.1<=r['elapsed_s']<17.]
            checks['actual_spin_without_translation'] = bool(phase) and max(r['yaw'] for r in phase)-min(r['yaw'] for r in phase)>=.5 and max(math.hypot(r['x']-phase[0]['x'],r['y']-phase[0]['y']) for r in phase)<=.02
            checks['repeated_xy_spin_poses'] = any(any(math.hypot(a['x']-b['x'],a['y']-b['y'])<1e-9 and abs(a['q'][2]-b['q'][2])>1e-4 for a,b in zip(r['message']['poses'],r['message']['poses'][1:])) for r in local)
            delay = load_rows(folder,'delay')
            if len(delay)==2:
                begin,end = [r['monotonic_s'] for r in delay]
                during = [r for r in states if begin<=r['monotonic_s']<=end]
                checks['dds_250ms_buffer_continuation'] = (end-begin>=.25 and
                    any(r['monotonic_s']<begin and any(e['kind']=='LOCAL_ACCEPTED' for e in r['events']) for r in ticks) and
                    sum(begin<=r['monotonic_s']<=end and bool(r['message']['poses']) for r in local)>=2 and
                    any(begin<=r['monotonic_s']<=end and any(e['kind']=='BUFFER_EXECUTING' and e['local_busy'] for e in r['events']) for r in ticks) and
                    bool(during) and math.hypot(during[-1]['x']-during[0]['x'],during[-1]['y']-during[0]['y'])>.001)
            else:
                checks['dds_250ms_buffer_continuation'] = False
        before = min(states,key=lambda r:abs(r['monotonic_s']-stop_s))
        final = states[-1]
        distance = math.hypot(final['x']-before['x'],final['y']-before['y'])
        checks['moving_at_planner_sigint'] = abs(before['v'])>.03
        checks['freshness_brake_after_sigint'] = (abs(final['v'])<.01 and abs(final['w'])<.01 and
            final['path_age_s']>.35 and distance<=abs(before['v'])*.40+before['v']**2+.02)
    if case=='empty_smoke':
        first_empty = next((r['monotonic_s'] for r in local if not r['message']['poses']),None)
        at_empty = min(states,key=lambda r:abs(r['monotonic_s']-first_empty)) if first_empty else states[0]
        checks['standard_empty_received_while_moving'] = first_empty is not None and abs(at_empty['v'])>.1
        checks['empty_controlled_brake'] = states[-1]['v']==0. and states[-1]['x']>at_empty['x'] and states[-1]['x']-at_empty['x']<=.06
    return apply_supervision_gate(dict(case=case, pass_=all(checks.values()), checks=checks, processes=process_results,
                runtime_s=max((r['elapsed_s'] for r in states),default=0.), formal_results=len(formal),
                path_counts=dict(local=len(local),global_nonempty=len(global_paths)),
                state_rows=len(states), final_state=states[-1] if states else None,
                observed_failure_reasons=sorted({r['global_result']['reason_code'] for r in ticks if r['global_result'] and not r['global_result']['has_reference']})), folder)
