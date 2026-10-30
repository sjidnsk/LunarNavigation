"""P3延迟观测上的无控制导航/前沿联调；不模拟到达，不发车辆指令。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

PREFIX='/p4_p3_debug'

def launch_arguments(source,build_dir,*,profile=None,allow_unknown=False):
    topics=dict(action_name='navigate_to_pose',goal_topic='goal',global_route_topic='global_route',
        local_path_topic='local_path',execution_reference_topic='execution_reference',tracking_status_topic='tracking_status',
        exploration_map_topic='exploration_map',policy_map_service='get_policy_map',task_topic='exploration/task',
        status_topic='exploration/status',current_goal_topic='exploration/current_goal',frontiers_topic='exploration/frontiers',
        task_boundary_topic='exploration/task_boundary',task_map_markers_topic='exploration/task_map_markers',
        exploration_diagnostics_topic='exploration/diagnostics',diagnostics_topic='control/diagnostics')
    result=dict({k:PREFIX+'/'+v for k,v in topics.items()},
        exploration_strategy='frontier',enable_controller='false',use_sim_time='false',
        platform_config=str(Path(source)/'config/wheel.yaml'),build_dir=str(build_dir),
        map_topic='/Car/T3/mapping/grid_map',odometry_topic='/Car/T3/localization/odometry',
        tf_topic='/tf',tf_static_topic='/tf_static',map_qos_reliability='reliable',
        map_qos_durability='transient_local',command_topic=PREFIX+'/disabled_cmd_vel',
        allow_unknown='true' if allow_unknown else 'false')
    if profile is not None:
        allowed={'odometry_timeout_s','p3_map_timeout_s','mppi_time_steps','mppi_compute_envelope_s'}
        if not isinstance(profile,dict) or set(profile)-allowed:
            raise ValueError('调试profile只能覆盖既有输入/预测/计算预算')
        result.update({k:str(v) for k,v in profile.items()})
    return result

def goal_candidates(grid,x,y):
    if grid is None:return []
    r=grid.info.resolution;o=grid.info.origin.position;w=grid.info.width
    candidates=[]
    for i,value in enumerate(grid.data):
        if value!=0:continue
        px=o.x+(i%w+.5)*r;py=o.y+(i//w+.5)*r;d=math.hypot(px-x,py-y)
        if .75<=d<=6.:candidates.append((d,px,py))
    return [(px,py) for _,px,py in sorted(candidates)]

def formal_plan(log,goal,*,after_offset=0):
    for line in log.read_bytes()[after_offset:].decode(errors='replace').splitlines():
        if 'GLOBAL_RESULT:' not in line:continue
        try:record=json.loads(line[line.index('{'):])
        except (ValueError,json.JSONDecodeError):continue
        requested=record.get('requested_goal') or []
        if (record.get('planning_outcome')==0 and record.get('reason_code')=='PLAN_FOUND'
                and record.get('has_reference') is True
                and record.get('accepted_generation')==record.get('current_accepted_generation') and len(requested)>=2
                and math.hypot(requested[0]-goal[0],requested[1]-goal[1])<1e-6):return record
    return None

def cleanup_goal(send_future,handle,spin_once,*,timeout_s=5.,now=time.monotonic,result_future=None):
    """待接受目标也必须有界解析并确认终态；失败由调用方停止整组。"""
    deadline=now()+timeout_s
    result=dict(ok=False,terminal=False,cancel_accepted=False)
    def wait(future):
        while not future.done() and now()<deadline:spin_once()
        return future.done()
    try:
        if handle is None and send_future is not None:
            if not wait(send_future):return dict(result,reason='GOAL_ACCEPTANCE_UNRESOLVED')
            handle=send_future.result()
        if handle is None or not handle.accepted:return dict(result,ok=True,terminal=True,reason='NO_ACTIVE_GOAL')
        terminal=result_future if result_future is not None else handle.get_result_async()
        if terminal.done():
            status=int(terminal.result().status)
            return dict(result,ok=status in (4,5,6),terminal=status in (4,5,6),status=status)
        cancel=handle.cancel_goal_async()
        if not wait(cancel):return dict(result,reason='CANCEL_RESPONSE_TIMEOUT')
        response=cancel.result()
        accepted=any(bytes(g.goal_id.uuid)==bytes(handle.goal_id.uuid) for g in response.goals_canceling)
        result['cancel_accepted']=accepted
        # 已终止目标的取消响应可能先到；仍等待真正终态，不能仅凭拒绝判清理失败。
        if not wait(terminal):return dict(result,reason='GOAL_TERMINAL_TIMEOUT' if accepted else 'CANCEL_REJECTED_WITHOUT_TERMINAL')
        status=int(terminal.result().status)
        return dict(result,ok=status in (4,5,6),terminal=status in (4,5,6),status=status)
    except Exception as exc:return dict(result,reason=repr(exc))


def observation_report(result,health,*,tf,tf_changed,stay_fault=None):
    report=dict(result,input_health=health,tf=tf,tf_changed=tf_changed)
    if stay_fault is not None:report['stay_fault']=stay_fault
    report['ok']=bool(result.get('ok') and all(v['ready'] for v in health.values()) and not stay_fault)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--artifact',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=120.);parser.add_argument('--stay',action='store_true')
    parser.add_argument('--observe-frontiers',action='store_true',help='只观察前沿与规划输入，不要求自主选点通过')
    parser.add_argument('--tcp-profile',type=Path,help='显式复用TCP输入预算配置，不启用控制')
    parser.add_argument('--allow-unknown',action='store_true',help='与TCP入口相同的显式UNKNOWN策略')
    args=parser.parse_args();artifact=args.artifact.resolve();artifact.mkdir(parents=True,exist_ok=False)
    profile=None
    if args.tcp_profile is not None:
        import yaml
        profile=yaml.safe_load(args.tcp_profile.read_text(encoding='utf-8'))

    # 所有ROS导入放在main，纯入口/目标选择测试不需要ROS。
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
    from grid_map_msgs.msg import GridMap
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import Point32
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker,MarkerArray
    from lunar_planning_msgs.action import NavigateToPose
    from lunar_planning_msgs.srv import GetPolicyMap
    from lunar_pure_exploration_msgs.msg import PureExplorationTask,PureExplorationStatus
    from p3_input_timing import record_sample,timing_health
    os.environ['ROS_DOMAIN_ID']='57'
    rclpy.init();node=rclpy.create_node('p4_p3_planning_observer')
    q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state=dict(samples={},odom=None,map=None,grid=None,global_paths=0,local_paths=0,statuses=[],tf=None,tf_changed=False)
    # 观察使用单一源有效期；接收/推进年龄只按同一期限作失效诊断。
    # 重复戳不能更新源时间，故不会延长有效期；这不是车辆执行预算。
    budgets=dict(map=dict(max_source_age_s=8.,receive_timeout_s=8.,progress_timeout_s=8.),
        odom=dict(max_source_age_s=6.,receive_timeout_s=6.,progress_timeout_s=6.))
    def received(kind,message):
        stamp=message.header.stamp;source=stamp.sec*10**9+stamp.nanosec
        state['samples'][kind]=record_sample(state['samples'].get(kind),source,node.get_clock().now().nanoseconds,time.monotonic())
        state[kind]=message
    def transforms(message):
        for t in message.transforms:
            if t.header.frame_id!='map' or t.child_frame_id!='odom':continue
            p=t.transform.translation;q=t.transform.rotation
            value=(p.x,p.y,p.z,q.x,q.y,q.z,q.w)
            if state['tf'] is not None and value!=state['tf']:state['tf_changed']=True
            state['tf']=value
    def health():
        now=node.get_clock().now().nanoseconds;mono=time.monotonic()
        report={k:timing_health(state['samples'].get(k),now,mono,**v) for k,v in budgets.items()}
        # 本次实测是固定单位变换；发生变化需历史TF配准，不默默采用latest补偿。
        identity=(0.,0.,0.,0.,0.,0.,1.)
        report['tf']=dict(ready=state['tf']==identity and not state['tf_changed'],reason='FIXED_IDENTITY' if state['tf']==identity and not state['tf_changed'] else 'WAITING_OR_CHANGED_TF')
        return report
    node.create_subscription(GridMap,'/Car/T3/mapping/grid_map',lambda m:received('map',m),q)
    node.create_subscription(Odometry,'/Car/T3/localization/odometry',lambda m:received('odom',m),q)
    node.create_subscription(TFMessage,'/tf',transforms,q)
    node.create_subscription(TFMessage,'/tf_static',transforms,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(OccupancyGrid,PREFIX+'/exploration_map',lambda m:state.update(grid=m),latched)
    def route(kind,m):
        if m.poses:state[kind]+=1
    node.create_subscription(RosPath,PREFIX+'/global_route',lambda m:route('global_paths',m),10)
    node.create_subscription(RosPath,PREFIX+'/local_path',lambda m:route('local_paths',m),10)
    task_id='p3-planning-observation'
    def status(m):
        if m.task_id==task_id:
            state['statuses'].append(dict(state=m.state,reason=m.reason_code,frontiers=m.frontier_cluster_count,candidates=m.candidate_count,reachable=m.reachable_candidate_count,plan_id=m.current_plan_id,completed=m.completed_goal_count))
            if len(state['statuses'])>256:del state['statuses'][:-256]
    node.create_subscription(PureExplorationStatus,PREFIX+'/exploration/status',status,10)
    task_pub=node.create_publisher(PureExplorationTask,PREFIX+'/exploration/task',10)
    health_pub=node.create_publisher(String,PREFIX+'/input_timing',10)
    health_markers=node.create_publisher(MarkerArray,PREFIX+'/input_status_markers',10)
    client=ActionClient(node,NavigateToPose,PREFIX+'/navigate_to_pose')
    maps=node.create_client(GetPolicyMap,PREFIX+'/get_policy_map')
    log_path=artifact/'launch.log';log=log_path.open('w')
    command=['ros2','launch','lunar_pure_exploration_ros','nav1cm_exploration.launch.py']
    command += [k+':='+v for k,v in launch_arguments(args.source,os.environ['NAV1CM_BUILD_DIR'],profile=profile,allow_unknown=args.allow_unknown).items()]
    process=subprocess.Popen(command,cwd=artifact,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (artifact/'pids.json').write_text(json.dumps(dict(observer=os.getpid(),launch=process.pid)))
    result=dict(ok=False,scope='P3 delayed observation planning only; no controller or vehicle bridge',budgets=budgets,manual_attempts=[])
    stopped=[False];signal.signal(signal.SIGTERM,lambda *a:stopped.__setitem__(0,True));signal.signal(signal.SIGINT,lambda *a:stopped.__setitem__(0,True))
    deadline=time.monotonic()+args.duration;guarded=False;handle=None;send_future=None;result_future=None;last_health=0.;stay_fault=None;last_pause=0.
    def spin():
        nonlocal last_health
        rclpy.spin_once(node,timeout_sec=.05)
        report=health()
        if time.monotonic()-last_health>1.:
            health_pub.publish(String(data=json.dumps(report)));last_health=time.monotonic()
            marker=Marker();marker.header.frame_id='base_link';marker.ns='p3_input_observation';marker.id=0
            marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD;marker.pose.orientation.w=1.;marker.lifetime.sec=3
            marker.pose.position.z=2.;marker.scale.z=.3
            marker.color.r=1.;marker.color.g=.9;marker.color.b=.3;marker.color.a=1.
            def age(kind):
                value=report[kind]['source_age_s'];return '?' if value is None else f'{value:.1f}s'
            latest=state['statuses'][-1] if state['statuses'] else {}
            marker.text=(f'P3 PLANNING OBSERVATION - NO CONTROL\nmap age {age("map")} / odom age {age("odom")}\n'
                f'frontiers {latest.get("frontiers",0)} / candidates {latest.get("candidates",0)}\n'
                f'map: {report["map"]["reason"]} / odom: {report["odom"]["reason"]}')
            health_markers.publish(MarkerArray(markers=[marker]))
        if stopped[0]:raise KeyboardInterrupt()
        if process.poll() is not None:raise RuntimeError('planning launch exited')
        if guarded and not all(v['ready'] for v in report.values()):
            result['input_fault']=report
            raise RuntimeError('input unavailable: '+json.dumps(report))
    def wait(predicate,seconds):
        end=min(deadline,time.monotonic()+seconds)
        while time.monotonic()<end:
            spin()
            if predicate():return
        raise TimeoutError('bounded planning observation wait expired')
    def task(command):
        message=PureExplorationTask();message.header.frame_id='map';message.header.stamp=node.get_clock().now().to_msg()
        message.task_id=task_id;message.command=command
        if state['odom'] is not None:
            p=state['odom'].pose.pose.position
            message.boundary.points=[Point32(x=float(p.x+dx),y=float(p.y+dy),z=0.) for dx,dy in ((-16,-16),(16,-16),(16,16),(-16,16))]
        task_pub.publish(message)
    def cancel_manual():
        nonlocal handle,send_future,result_future
        cleaned=cleanup_goal(send_future,handle,lambda:rclpy.spin_once(node,timeout_sec=.05),result_future=result_future)
        result.setdefault('manual_cleanup',[]).append(cleaned)
        if cleaned['ok']:handle=None;send_future=None;result_future=None
        else:result['ok']=False;result['cleanup_failed']=cleaned
        return cleaned['ok']
    def manual_phase():
        nonlocal handle,send_future,result_future
        p=state['odom'].pose.pose.position
        for goal in goal_candidates(state['grid'],p.x,p.y)[:4]:
            g=NavigateToPose.Goal(target_x_m=goal[0],target_y_m=goal[1],has_target_yaw=False)
            offset=log_path.stat().st_size
            item=dict(goal=goal,accepted=None,feedback_reasons={});result['manual_attempts'].append(item)
            def feedback(message,item=item):
                reason=message.feedback.reason_code
                item['feedback_reasons'][reason]=item['feedback_reasons'].get(reason,0)+1
            f=client.send_goal_async(g,feedback_callback=feedback);send_future=f;wait(f.done,5.);handle=f.result()
            item['accepted']=handle.accepted
            if not handle.accepted:
                if not cancel_manual():raise RuntimeError('manual goal cleanup failed')
                continue
            done=handle.get_result_async();result_future=done;before=state['local_paths']
            wait(lambda:formal_plan(log_path,goal,after_offset=offset) is not None or done.done(),25.)
            item['formal_plan']=formal_plan(log_path,goal,after_offset=offset)
            if item['formal_plan'] is not None:
                wait(lambda:state['local_paths']>before or done.done() or 'NO_CERTIFIED_ACTION' in item['feedback_reasons'],20.)
                result['manual_plan_found']=True;result['manual_local_reference']=state['local_paths']>before
                result['manual_no_certified_action']='NO_CERTIFIED_ACTION' in item['feedback_reasons']
                if not cancel_manual():raise RuntimeError('manual goal cleanup failed')
                break
            if done.done():item['reason']=done.result().result.reason_code
            if not cancel_manual():raise RuntimeError('manual goal cleanup failed')
    try:
        wait(lambda:all(v['ready'] for v in health().values()) and state['grid'] is not None and client.server_is_ready() and maps.service_is_ready(),40.)
        guarded=True
        # 此服务另含停车支撑认证；无控制规划不能用它替代地图/规划自身判据。
        # 只请求一次完整快照，避免反复导出大地图干扰同域的实际输入。
        f=maps.call_async(GetPolicyMap.Request());wait(f.done,10.)
        response=f.result()
        result['policy_map_diagnostic']=dict(ready=bool(response.ready),reason=response.reason_code,
            frame=response.frame_id,fine_revision=int(response.fine_revision),tiles=len(response.tiles),
            stationary_support_ready=bool(response.stationary_support_ready),
            start_connection_status=int(response.start_connection_status))
        anchor=response.anchor_pose.position
        cell_x=math.floor((anchor.x-response.origin.x)/response.resolution_m) if response.resolution_m>0 else 0
        cell_y=math.floor((anchor.y-response.origin.y)/response.resolution_m) if response.resolution_m>0 else 0
        center_state=None
        for tile in response.tiles:
            if tile.tile_x==cell_x//256 and tile.tile_y==cell_y//256:
                center_state=int(tile.states[(cell_y%256)*256+cell_x%256])
        result['policy_map_diagnostic']['anchor_center_state']=center_state
        for recovery in range(0 if args.observe_frontiers else 3):
            result.pop('input_fault',None)
            try:
                manual_phase()
                break
            except RuntimeError:
                fault=result.get('input_fault')
                if fault is None or recovery==2:raise
                guarded=False
                if not cancel_manual():raise RuntimeError('manual goal cleanup failed')
                task(PureExplorationTask.PAUSE)
                watermark={k:state['samples'][k]['high_water_source_ns']
                    for k in ('map','odom') if not fault[k]['ready']}
                result.setdefault('input_recoveries',[]).append(dict(fault=fault,source_watermark=watermark))
                wait(lambda:all(v['ready'] for v in health().values()) and
                    all(state['samples'][k]['high_water_source_ns']>stamp for k,stamp in watermark.items()),30.)
                guarded=True
        result.pop('input_fault',None)
        wait(lambda:task_pub.get_subscription_count()>0,5.)
        task(PureExplorationTask.START)
        wait(lambda:any(s['frontiers']>0 and (args.observe_frontiers or s['plan_id']) for s in state['statuses']),30.)
        result['frontiers_detected']=max((s['frontiers'] for s in state['statuses']),default=0)
        result['exploration_goal_selected']=any(bool(s['plan_id']) for s in state['statuses'])
        task(PureExplorationTask.PAUSE)
        wait(lambda:any(s['state']==PureExplorationStatus.PAUSED for s in state['statuses']),10.)
        result['paused']=True
        result['observation_ok']=bool(result.get('manual_plan_found') and result.get('exploration_goal_selected') and result['paused'])
        result['ok']=bool(result.get('manual_plan_found') and result.get('manual_local_reference') and result.get('exploration_goal_selected') and result['paused'])
    except BaseException as exc:result['error']=repr(exc)
    finally:
        guarded=False;cancel_manual();task(PureExplorationTask.PAUSE)
        result.update(tf=state['tf'],tf_changed=state['tf_changed'],input_health=health(),global_paths=state['global_paths'],local_paths=state['local_paths'],exploration_statuses=state['statuses'])
        result['vehicle_command_publishers']=[p.node_name for p in node.get_publishers_info_by_topic('/Car/T5/Car_Cmd_Vel')]
        result['disabled_command_publishers']=[p.node_name for p in node.get_publishers_info_by_topic(PREFIX+'/disabled_cmd_vel')]
        result['ok']=bool(result['ok'] and not result['disabled_command_publishers'])
        result['initial_probe_ok']=result['ok']
        result['phase']='PAUSED_OBSERVING' if args.stay and not result.get('cleanup_failed') else 'PROBE_FINISHED'
        result=observation_report(result,health(),tf=state['tf'],tf_changed=state['tf_changed'])
        (artifact/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    try:
        if args.stay and not result.get('cleanup_failed') and process.poll() is None:
            # 暂停状态保留图形；仍监测实际输入，不触发新的探索目标。
            while not stopped[0]:
                spin()
                if not all(v['ready'] for v in health().values()):
                    stay_fault=stay_fault or json.dumps(health())
                    if time.monotonic()-last_pause>1.:task(PureExplorationTask.PAUSE);last_pause=time.monotonic()
    except KeyboardInterrupt:
        result['operator_stopped']=True
    finally:
        task(PureExplorationTask.CANCEL)
        for _ in range(10):rclpy.spin_once(node,timeout_sec=.05)
        final_health=health()
        result['observation_ended_ros_ns']=node.get_clock().now().nanoseconds
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12.)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5.);result['forced_shutdown']=True
        result=observation_report(result,final_health,tf=state['tf'],tf_changed=state['tf_changed'],stay_fault=stay_fault)
        result['phase']='FINISHED'
        result['launch_returncode']=process.returncode
        result['ok']=bool(result['ok'] and process.returncode==0 and not result.get('forced_shutdown'))
        (artifact/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown();log.close()
    return 0 if result['ok'] else 1

if __name__=='__main__':raise SystemExit(main())
