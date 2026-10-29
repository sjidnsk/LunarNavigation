"""Humble短路径bench与独立判据；只发布一次Path和可选固定TF，不发布Twist。

可直接在Humble cp310运行本文件；纯判据及--analyze不需要ROS/Isaac。
"""
import argparse
import bisect
import json
import math
import os
from pathlib import Path
import time

CMD = '/Car/T5/Car_Cmd_Vel'
ODOM = '/Car/T3/localization/odometry'
PATH = '/Car/T4/planning/local_path'


def yaw_of(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('invalid quaternion')
    x, y, z, w = q
    if abs(sum(v*v for v in q)-1.) > .01:
        raise ValueError('quaternion is not normalized')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def body_vector(q, v):
    x, y, z, w = q
    # R(q).T @ world_vector，明确处理坡面上的完整姿态。
    return [(1-2*(y*y+z*z))*v[0]+2*(x*y+z*w)*v[1]+2*(x*z-y*w)*v[2],
            2*(x*y-z*w)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z+x*w)*v[2],
            2*(x*z+y*w)*v[0]+2*(y*z-x*w)*v[1]+(1-2*(x*x+y*y))*v[2]]


def identity_tf_event(components, wall, stamp):
    """校验直接identity TF；非法原值以可严格JSON保存的表示留证。"""
    values = list(components)
    finite = [isinstance(v, (int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values]
    event = {'event':'tf','wall':wall,'stamp':stamp,
             'transform_xyz_xyzw':[v if ok else repr(v) for v,ok in zip(values,finite)]}
    if len(values) != 7 or not all(finite):
        event.update(event='error',message='nonfinite or invalid map->odom components')
    elif any(abs(v-want)>1e-6 for v,want in zip(values,(0.,0.,0.,0.,0.,0.,1.))):
        event.update(event='error',message='bench requires truth-consistent identity map->odom')
    return event


def new_sample_gap(odometry, start_wall, end_wall):
    """按接收时间比较真实新stamp；缓存重发不能刷新采样时间锚点。"""
    for sample in odometry:
        if not all(math.isfinite(sample[key]) for key in ('wall','stamp')):
            raise ValueError('nonfinite odometry sample time')
    last_stamp, anchor, gaps, reversed_stamp = None, start_wall, [], False
    for sample in sorted(odometry, key=lambda e:e['wall']):
        stamp, wall = sample['stamp'], sample['wall']
        if last_stamp is not None:
            if stamp < last_stamp:
                reversed_stamp = True
                continue
            if stamp == last_stamp:
                continue
        last_stamp = stamp
        if wall >= start_wall:
            gaps.append(wall-anchor)
        # 路径前最后一个新样本也作为起点freshness依据；不计更早的准备期gap。
        anchor = wall
    gaps.append(end_wall-anchor)
    return max(gaps), reversed_stamp


def evaluate(events, odom_timeout=10., wheel_radius=.1595):
    """保守检查最终连续尾窗；不选历史最优帧，重复stamp不累积仿真时长。"""
    result = {'passed': False, 'reasons': [], 'controller_odom_timeout_s': odom_timeout,
              'controller_tf_timeout_s': .5, 'drive_command_timeout_s': .5}
    try:
        return _evaluate(events, result, odom_timeout, wheel_radius)
    except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
        result['reasons'].append('invalid_evidence: '+str(exc))
        return result


def _evaluate(events, result, odom_timeout, wheel_radius):
    def require(condition, reason):
        if not condition:
            result['reasons'].append(reason)

    require(not any(e['event'] == 'error' for e in events), 'recorded_bench_error')
    paths = [e for e in events if e['event'] == 'path']
    require(len(paths) == 1 and len(paths[0]['poses']) >= 2, 'path_not_published_exactly_once')
    if result['reasons']:
        return result
    path = paths[0]
    start, goal = path['poses'][0], path['poses'][-1]
    if not all(math.isfinite(v) for p in path['poses'] for v in p):
        raise ValueError('nonfinite path')
    groups = {}
    for e in events:
        e = dict(e)
        if e['event'] == 'tf' and 'transform_xyz_xyzw' in e:
            checked = identity_tf_event(e['transform_xyz_xyzw'],e['wall'],e['stamp'])
            if checked['event'] == 'error':
                raise ValueError(checked['message'])
        if e['event'] in ('twist','rejected_twist'):
            e['wall'] = e['rx_wall']
        if e['event'] == 'physics':
            e['wall'] = e['state_publish_done_wall']
            q = e['root_pose_xyzw'][3:]
            e['pose'] = e['root_pose_xyzw'][:2]+[yaw_of(q)]
            linear = body_vector(q, e['root_world_velocity'][:3])
            angular = body_vector(q, e['root_world_velocity'][3:])
            e['velocity'] = linear[:2]+[angular[2]]
            e['stamp'] = e['time']
        if e.get('wall', -1) >= path['wall']:
            groups.setdefault(e['event'], []).append(e)
    for group in groups.values():
        group.sort(key=lambda e: e['wall'])
    cmd, odom, physics, tf, clocks = [groups.get(k, []) for k in ('cmd','odom','physics','tf','clock')]
    require(all((cmd, odom, physics, tf, clocks)), 'missing_command_or_physical_feedback')
    graphs = groups.get('graph', [])
    require(bool(graphs) and all(g['command_publishers'] == ['lunar_pure_wheeled_controller'] and
            'viper_engineering_bridge' in g['command_subscribers'] for g in graphs), 'command_graph_not_isolated')
    if not all((cmd, odom, physics, tf, clocks)):
        return result
    for e in cmd+odom+physics:
        values = e['values'] if e['event'] == 'cmd' else e['pose']+e['velocity']
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite command/state')
    require(any(abs(c['values'][0]) > 1e-6 for c in cmd), 'no_linear_command')
    require(any(abs(c['values'][5]) > 1e-6 for c in cmd), 'no_angular_command')
    require(all(len(c['values']) == 6 and all(abs(c['values'][i]) <= 1e-6 for i in (1,2,3,4))
                and abs(c['values'][0]) <= .200001 and abs(c['values'][5]) <= .500001 for c in cmd), 'command_outside_bench_limits')

    def error(e):
        return math.hypot(e['pose'][0]-goal[0], e['pose'][1]-goal[1]), abs(wrap(e['pose'][2]-goal[2]))

    def stopped(e):
        return max(error(e)[0]/.2, error(e)[1]/.2,
                   math.hypot(*e['velocity'][:2])/.01, abs(e['velocity'][2])/.03) <= 1.+1e-9

    result.update(final_position_error_m=error(odom[-1])[0], final_yaw_error_rad=error(odom[-1])[1],
                  root_final_position_error_m=error(physics[-1])[0], root_final_yaw_error_rad=error(physics[-1])[1],
                  forward_displacement_m=(odom[-1]['pose'][0]-start[0])*math.cos(start[2])+(odom[-1]['pose'][1]-start[1])*math.sin(start[2]),
                  actual_yaw_change_rad=wrap(odom[-1]['pose'][2]-start[2]),
                  peak_odom_speed_mps=max(math.hypot(*e['velocity'][:2]) for e in odom),
                  peak_odom_yaw_rate_radps=max(abs(e['velocity'][2]) for e in odom),
                  max_cross_track_m=max(abs(-(e['pose'][0]-start[0])*math.sin(start[2])+(e['pose'][1]-start[1])*math.cos(start[2])) for e in odom),
                  wall_elapsed_s=max(e['wall'] for e in cmd+odom+physics)-path['wall'],
                  sim_elapsed_s=odom[-1]['stamp']-odom[0]['stamp'])
    require(result['forward_displacement_m'] >= .5, 'insufficient_actual_displacement')
    joint_peak = max(abs(v) for e in physics for v in e['dof_velocities'])
    result['peak_measured_joint_speed_radps'] = joint_peak
    require(math.isfinite(joint_peak) and joint_peak > .1, 'no_measured_joint_response')
    result['runtime_command_timeout_samples'] = sum(e['targets']['reason'] == 'command_timeout' for e in physics)
    rejected = len(groups.get('rejected_twist', []))+sum(e.get('accepted') is not True for e in groups.get('twist', []))
    result['runtime_rejected_receipts'] = rejected
    require(rejected == 0, 'runtime_rejected_twist')
    # 从最后一次违规之后算最终尾窗；原始root与odom都必须保持目标容差与停稳。
    bad = [path['wall']]
    bad.extend(e['wall'] for e in cmd if any(abs(v)>1e-6 for v in e['values']))
    bad.extend(e['wall'] for e in odom if not stopped(e))
    for e in physics:
        joints = e['dof_velocities']
        if (not stopped(e) or len(joints) != 8 or not all(math.isfinite(v) and abs(v) <= .01/wheel_radius for v in joints)
                or e['targets']['reason'] in ('command_timeout','invalid_command','invalid_feedback_or_time','no_command')):
            bad.append(e['wall'])
    begin = max(bad)
    tail = {k:[e for e in groups[k] if e['wall'] > begin] for k in ('cmd','odom','physics','clock')}
    # 相同stamp可明确重发，但不作为新样本；倒退一律失败。
    for k in ('odom','physics','clock'):
        stream = tail[k]
        require(all(b['stamp'] >= a['stamp'] for a,b in zip(stream,stream[1:])), k+'_stamp_reversed')
        tail[k] = [e for i,e in enumerate(stream) if i == 0 or e['stamp'] > stream[i-1]['stamp']]
    tc, to, tp, tclock = [tail[k] for k in ('cmd','odom','physics','clock')]
    require(len(tc) >= 20, 'fewer_than_20_stopped_commands')
    require(all(len(stream) >= 3 and stream[-1]['stamp']-stream[0]['stamp'] >= 1.-1e-9
                for stream in (to,tp,tclock)), 'insufficient_new_physics_sim_window')
    require(bool(tc) and tc[-1]['wall']-tc[0]['wall'] >= 1.-1e-9, 'insufficient_wall_stop_window')
    gaps = [b['wall']-a['wall'] for a,b in zip(tc,tc[1:])]
    require(bool(gaps) and max(gaps) < .5, 'controller_command_gap')
    max_ages = {}
    for name, stream, timeout in [('odom',odom,odom_timeout), ('tf',tf,.5)]:
        times = [e['wall'] for e in stream]
        ages = []
        for e in tc:
            i = bisect.bisect_right(times,e['wall'])-1
            ages.append(e['wall']-times[i] if i >= 0 else timeout+1.)
        max_ages[name] = max(ages,default=None)
        require(bool(ages) and max(ages) <= timeout, name+'_stale_in_stop_window')
    end = max(e['wall'] for e in odom+physics+cmd)
    require(bool(tc) and end-tc[-1]['wall'] < .5, 'controller_stream_ended_before_feedback')
    renders = [e for e in events if e['event'] == 'render_timing']
    require(bool(tp) and any(tp[0]['wall'] <= e['render_start_wall'] < e['render_done_wall'] <= tp[-1]['wall']
                for e in renders), 'no_render_crossed_in_stop_window')
    for stream in (to,tp):
        # 两种独立状态必须描述相同仿真步；不把遥测旧样本和新odom拼接。
        other = tp if stream is to else to
        stamps = [e['stamp'] for e in other]
        require(all(any(abs(e['stamp']-s)<1e-6 for s in stamps) for e in stream), 'odom_root_stamp_mismatch')
    result['max_feedback_gap_wall_s'], reversed_stamp = new_sample_gap([e for e in events if e['event'] == 'odom'], path['wall'], end)
    require(not reversed_stamp, 'odom_stamp_reversed')
    require(result['max_feedback_gap_wall_s'] is not None and result['max_feedback_gap_wall_s'] < odom_timeout, 'feedback_gap_exceeded')
    result['stop_window'] = {'begin_wall':begin, 'commands':len(tc), 'distinct_odom_samples':len(to),
        'distinct_physics_samples':len(tp), 'max_command_gap_s':max(gaps,default=None), 'max_receive_age_s':max_ages,
        'wall_s':tc[-1]['wall']-tc[0]['wall'] if tc else 0.,
        'sim_s':to[-1]['stamp']-to[0]['stamp'] if to else 0., 'joint_speed_limit_radps':.01/wheel_radius}
    result['passed'] = not result['reasons']
    return result


def sampled_stamp(seconds):
    """仅转写已经观测到的仿真clock；render停顿时保持同一真实stamp。"""
    if isinstance(seconds, bool) or not isinstance(seconds, (int,float)) or not math.isfinite(seconds) or seconds < 0:
        raise ValueError('valid observed simulation clock required')
    return divmod(round(seconds*1e9),1_000_000_000)


def run_bench(args):
    if any(os.environ.get(k) != v for k,v in [('ROS_DOMAIN_ID','117'),('ROS_LOCALHOST_ONLY','1'),('ROS_DISTRO','humble')]):
        raise RuntimeError('required domain117/localhost1/Humble')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry, Path as RosPath
    from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    events, latest, initial = [], {}, []
    log = (output/'events.jsonl').open('w',encoding='utf-8',buffering=1)
    telemetry = Path(args.telemetry).open(encoding='utf-8')
    telemetry.seek(0,2)  # 不把本run之前的矩阵/物理采样当新闭环。
    rclpy.init(args=[])
    node = rclpy.create_node('isaac_viper_short_path_verifier')
    sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(RosPath,PATH,durable)
    tfpub = node.create_publisher(TFMessage,'/tf',10) if args.fixed_identity_tf else None

    def record(e):
        events.append(e)
        log.write(json.dumps(e,allow_nan=False)+'\n')

    def stamp(s):
        return s.sec+s.nanosec*1e-9

    def odometry(m):
        if m.header.frame_id != 'odom' or m.child_frame_id != 'base_link':
            raise ValueError('odometry frame mismatch')
        q, p, v = m.pose.pose.orientation, m.pose.pose.position, m.twist.twist
        e = dict(event='odom',wall=time.monotonic(),stamp=stamp(m.header.stamp),
                 pose=[p.x,p.y,yaw_of([q.x,q.y,q.z,q.w])],velocity=[v.linear.x,v.linear.y,v.angular.z])
        if not all(math.isfinite(v) for v in e['pose']+e['velocity']):
            raise ValueError('nonfinite initial/live odometry')
        latest['odom'] = e
        if not initial or e['stamp']>initial[-1]['stamp']:
            initial.append(e)
        record(e)

    def transform(m):
        for t in m.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                q,p = t.transform.rotation,t.transform.translation
                event = identity_tf_event([p.x,p.y,p.z,q.x,q.y,q.z,q.w],time.monotonic(),stamp(t.header.stamp))
                record(event)
                if event['event'] == 'error':
                    raise ValueError(event['message'])

    subscriptions = [node.create_subscription(Odometry,ODOM,odometry,sensor),
        node.create_subscription(TFMessage,'/tf',transform,sensor),
        node.create_subscription(Twist,CMD,lambda m:record(dict(event='cmd',wall=time.monotonic(),
            values=[m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z])),100),
        node.create_subscription(Clock,'/clock',lambda m:(latest.update(clock=stamp(m.clock)),
            record(dict(event='clock',wall=time.monotonic(),stamp=stamp(m.clock)))),sensor)]
    def path_received(m):
        if m.header.frame_id != 'map':
            raise ValueError('path frame mismatch')
        route = []
        for pose in m.poses:
            p,q = pose.pose.position,pose.pose.orientation
            route.append([p.x,p.y,yaw_of([q.x,q.y,q.z,q.w])])
        record(dict(event='path',wall=time.monotonic(),stamp=stamp(m.header.stamp),poses=route))

    subscriptions.append(node.create_subscription(RosPath,PATH,path_received,durable))
    started, sent, last_tf, last_check, last_graph = time.monotonic(), False, 0., 0., 0.
    pending = ''
    graph = {}
    outcome = None
    try:
        while time.monotonic()-started < args.wall_timeout:
            rclpy.spin_once(node,timeout_sec=.01)
            now = time.monotonic()
            if tfpub and 'clock' in latest and now-last_tf >= .05:
                t = TransformStamped()
                t.header.stamp.sec,t.header.stamp.nanosec = sampled_stamp(latest['clock'])
                t.header.frame_id,t.child_frame_id = 'map','odom'
                t.transform.rotation.w = 1.
                tfpub.publish(TFMessage(transforms=[t]))
                last_tf = now
            pending += telemetry.read()
            while '\n' in pending:
                line,pending = pending.split('\n',1)
                e = json.loads(line)
                if e['event'] in ('physics','render_timing','twist','rejected_twist'):
                    record(e)
            if now-last_graph >= 1.:
                graph = dict(event='graph',wall=now,
                    command_publishers=[x.node_name for x in node.get_publishers_info_by_topic(CMD)],
                    command_subscribers=[x.node_name for x in node.get_subscriptions_info_by_topic(CMD)])
                record(graph)
                last_graph = now
            ready = (len(initial)>=3 and initial[-1]['stamp']-initial[-3]['stamp']>0 and
                     'clock' in latest and latest['clock']>initial[0]['stamp'] and
                     now-latest['odom']['wall']<.5 and pub.get_subscription_count() >= 2 and
                     graph.get('command_publishers') == ['lunar_pure_wheeled_controller'] and
                     'viper_engineering_bridge' in graph.get('command_subscribers',[]))
            if not sent and ready:
                if not all(math.hypot(*e['velocity'][:2])<=.01 and abs(e['velocity'][2])<=.03 for e in initial[-3:]):
                    continue
                x,y,a = latest['odom']['pose']
                route = [[x,y,a],[x+.8*math.cos(a),y+.8*math.sin(a),a+.35]]
                p = RosPath()
                p.header.stamp.sec,p.header.stamp.nanosec = sampled_stamp(latest['clock'])
                p.header.frame_id = 'map'
                for px,py,yaw in route:
                    s = PoseStamped()
                    s.header = p.header
                    s.pose.position.x,s.pose.position.y = px,py
                    s.pose.orientation.z,s.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
                    p.poses.append(s)
                pub.publish(p)
                record(dict(event='fixture',wall=now,poses=route))
                sent = True
                print(json.dumps({'published_once':route}),flush=True)
            if sent and now-last_check >= 1.:
                outcome = evaluate(events,args.odometry_timeout,args.wheel_radius)
                last_check = now
                if outcome['passed']:
                    break
        outcome = evaluate(events,args.odometry_timeout,args.wheel_radius)
        outcome.update(wall_budget_s=args.wall_timeout,wall_total_s=time.monotonic()-started,
                       mode='wall-scheduled controller bench',use_sim_time=False,
                       fixed_identity_tf=args.fixed_identity_tf,
                       NOT_RUN=['SLAM','planning_PLAN_FOUND','Orin','real_vehicle','real_calibration'])
    except Exception as exc:
        record(dict(event='error',wall=time.monotonic(),message=repr(exc)))
        outcome = {'passed':False,'reasons':['bench_exception: '+repr(exc)]}
    finally:
        # 不发空Path，不停controller，不改变Kit；退出后恢复/停车由操作者单独执行并留证。
        telemetry.close()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    (output/'result.json').write_text(json.dumps(outcome,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(outcome,allow_nan=False),flush=True)
    return 0 if outcome['passed'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analyze',help='只读重算events.jsonl，不加载ROS')
    parser.add_argument('--output-dir')
    parser.add_argument('--telemetry',help='当前唯一Kit的telemetry.jsonl')
    parser.add_argument('--fixed-identity-tf',action='store_true',help='已确认世界一致的固定map->odom，墙钟20Hz；Kit原边也必须为identity')
    parser.add_argument('--wall-timeout',type=float,default=240.)
    parser.add_argument('--odometry-timeout',type=float,default=10.)
    parser.add_argument('--wheel-radius',type=float,default=.1595,help='从当前wheel_orin.yaml核对的轮半径，m')
    args = parser.parse_args()
    if args.analyze:
        events = [json.loads(s) for s in Path(args.analyze).read_text(encoding='utf-8').splitlines()]
        result = evaluate(events,args.odometry_timeout,args.wheel_radius)
        print(json.dumps(result,indent=2,allow_nan=False))
        return 0 if result['passed'] else 2
    if not args.output_dir or not args.telemetry:
        parser.error('live bench requires --output-dir and --telemetry')
    if not 0 < args.wall_timeout <= 240 or not 0 < args.odometry_timeout <= 10 or args.wheel_radius <= 0:
        parser.error('invalid bench bounds')
    return run_bench(args)


if __name__ == '__main__':
    raise SystemExit(main())
