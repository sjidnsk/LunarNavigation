#!/usr/bin/env python3
"""真实公共launch + 独立Twist植物；仅策略task命令，不发布goal/Path。"""
import argparse
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from fixtures import CASES,DECLARATION,measurement_mask,declaration
from grade import grade

def helper(name):
    spec=importlib.util.spec_from_file_location("independent_"+name,HERE.parent/"p3_path_validation"/(name+".py"))
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module);return module
motion=helper("motion")
spec=importlib.util.spec_from_file_location("independent_clearance",HERE.parent/"path_follower_validation/clearance.py")
clearance=importlib.util.module_from_spec(spec);spec.loader.exec_module(clearance)

PREFIX="/nav1cm_task3"
TOPICS={k:PREFIX+"/"+k for k in ("grid","odom","tf","tf_static","navigate","goal_disabled","global","local","exploration_map","policy","task","status","cmd","diag")}

def pace_cycle(started):
    """独立测量周期不随ROS反馈队列唤醒频率加速。"""
    time.sleep(max(0., .02-(time.monotonic()-started)))

def service_cycle(started, spin):
    """同一单调20ms窗口内处理事件，测量发布周期不随事件频率加速。"""
    deadline=started+.02
    while time.monotonic()<deadline:
        spin(min(.002,max(0.,deadline-time.monotonic())))
    pace_cycle(started)

def uuid_values(value):
    """ROS uint8数组转成生成消息setter接受的Python整数。"""
    return list(bytes(value))

def process_rows(group):
    rows=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            pid=int(path.name)
            if os.getpgid(pid)!=group:continue
            maps=(path/'maps').read_text();libs={}
            for line in maps.splitlines():
                target=Path(line.split()[-1])
                if ('libnav1cm_' in str(target) or 'liblunar_' in str(target)) and target.is_file():
                    libs[str(target)]=hashlib.sha256(target.read_bytes()).hexdigest()
            rows.append(dict(pid=pid,cmdline=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace'),native=libs))
        except (OSError,ProcessLookupError,PermissionError):pass
    return rows

def measured_stopped_before(rows, terminal_time):
    return any(0.<=terminal_time-r['monotonic_s']<=.15 and abs(r['v'])<=.01 and abs(r['w'])<=.01 for r in rows)

def held(rows,pose,terminal_time):
    start=None;best=None
    for row in rows:
        if row['monotonic_s']>terminal_time:break
        xy=math.hypot(row['x']-pose[0],row['y']-pose[1]);yaw=motion.wrap(row['yaw']-pose[2])
        valid=xy<=.1+1e-9 and abs(yaw)<=.05+1e-9 and abs(row['v'])<=.01+1e-9 and abs(row['w'])<=.01+1e-9
        if valid:
            if start is None:start=row['monotonic_s']
            candidate=dict(xy_error=xy,yaw_error=yaw,v=row['v'],w=row['w'],hold_s=row['monotonic_s']-start,measured_at=row['monotonic_s'])
            if terminal_time-row['monotonic_s']<=.5 and (best is None or candidate['hold_s']>best['hold_s']):best=candidate
        else:start=None
    return best or dict(xy_error=math.inf,yaw_error=math.inf,v=math.inf,w=math.inf,hold_s=0.)

def run(args):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.convert import message_to_ordereddict
    from nav_msgs.msg import Odometry,Path as PathMessage,OccupancyGrid
    from geometry_msgs.msg import Twist,TransformStamped,Point32
    from grid_map_msgs.msg import GridMap
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String,Float32MultiArray,MultiArrayDimension
    from action_msgs.msg import GoalStatusArray
    from unique_identifier_msgs.msg import UUID
    from lunar_planning_msgs.action import NavigateToPose
    from lunar_planning_msgs.srv import GetPolicyMap
    from lunar_pure_exploration_msgs.msg import PureExplorationTask,PureExplorationStatus
    assert os.environ.get('ROS_DOMAIN_ID')=='92' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DISTRO')=='humble'
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    (output/'declaration.json').write_text(json.dumps(declaration(args.fixture_version),ensure_ascii=False,indent=2))
    (output/'harness-identity.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for folder in (HERE,HERE.parent/'p3_path_validation',HERE.parent/'path_follower_validation') for f in folder.glob('*.py')},indent=2))
    stream=(output/'plant.jsonl').open('x',encoding='utf-8')
    records=[];states=[];results=[];feedback=[];goals=[];pending={};graphs=[];policy=[];observations=[];diagnostics=[]
    def emit(kind,**values):
        row=dict(kind=kind,monotonic_s=time.monotonic(),**values);records.append(row)
        stream.write(json.dumps(row,ensure_ascii=False,allow_nan=True)+'\n');stream.flush();return row
    strategy='none' if args.case=='large_source_preflight' else 'frontier'
    launch=['ros2','launch','lunar_pure_exploration_ros','nav1cm_exploration.launch.py',
        'exploration_strategy:='+strategy,'enable_controller:=true','build_dir:='+str(Path(args.build_dir).resolve()),'coarse_resolution_m:=0.2',
        'navigation_map_wait_timeout_s:='+('2.0' if args.case=='blocked_wait_reselect' else '30.0')]
    names={'map_topic':'grid','odometry_topic':'odom','tf_topic':'tf','tf_static_topic':'tf_static','action_name':'navigate','goal_topic':'goal_disabled',
           'global_route_topic':'global','local_path_topic':'local','exploration_map_topic':'exploration_map','policy_map_service':'policy',
           'task_topic':'task','status_topic':'status','command_topic':'cmd','diagnostics_topic':'diag'}
    launch += [name+':='+TOPICS[key] for name,key in names.items()]
    launch += [name+':='+PREFIX+'/'+name for name in ('current_goal_topic','frontiers_topic','task_boundary_topic','task_map_markers_topic','exploration_diagnostics_topic')]
    emit('COMMAND_CONTRACT',argv=launch,domain=os.environ['ROS_DOMAIN_ID'],localhost=os.environ['ROS_LOCALHOST_ONLY'])
    console=(output/'launch.console.log').open('x')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=Node('nav1cm_task3_independent_plant')
    reliable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    command=[0.,0.];state=motion.State();start=time.monotonic();last=start;cancel_at=None;cancel_moving=False;sent=False;scan=False;obstacle=None
    last_status=None;next_map=next_tf=next_graph=next_policy=0.;observed_count=0
    def receive_cmd(m):
        command[:]=[m.linear.x,m.angular.z];emit('COMMAND',v=command[0],w=command[1])
    def status(m):
        nonlocal last_status
        last_status=m;emit('STATUS',message=message_to_ordereddict(m))
    def get_result(uuid):
        uid=bytes(uuid).hex()
        if uid in pending:return
        request=NavigateToPose.Impl.GetResultService.Request();request.goal_id=UUID(uuid=uuid_values(uuid))
        pending[uid]=result_client.call_async(request);emit('UUID',uuid=uid,pose=(list(goals[-1]['pose']) if goals else None))
    def action_feedback(m):
        row=emit('ACTION_FEEDBACK',uuid=bytes(m.goal_id.uuid).hex(),message=message_to_ordereddict(m.feedback));feedback.append(row)
        get_result(m.goal_id.uuid)
    def action_status(m):
        for s in m.status_list:get_result(s.goal_info.goal_id.uuid)
    def exploration(m):emit('EXPLORATION_MAP',resolution=m.info.resolution,counts={str(i):m.data.count(i) for i in (-1,0,100)})
    def diag(m):
        row=emit('DIAGNOSTIC',data=json.loads(m.data));diagnostics.append(row)
    def goal(m):
        p=m.pose.position;q=m.pose.orientation;pose=(p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
        if not goals or any(abs(a-b)>1e-9 for a,b in zip(pose,goals[-1]['pose'])):goals.append(emit('FRONTEND_GOAL',pose=pose))
    from geometry_msgs.msg import PoseStamped
    node.create_subscription(Twist,TOPICS['cmd'],receive_cmd,100)
    node.create_subscription(PureExplorationStatus,TOPICS['status'],status,transient)
    node.create_subscription(OccupancyGrid,TOPICS['exploration_map'],exploration,transient)
    node.create_subscription(String,TOPICS['diag'],diag,100)
    node.create_subscription(PoseStamped,PREFIX+'/current_goal_topic',goal,transient)
    node.create_subscription(NavigateToPose.Impl.FeedbackMessage,TOPICS['navigate']+'/_action/feedback',action_feedback,100)
    node.create_subscription(GoalStatusArray,TOPICS['navigate']+'/_action/status',action_status,transient)
    for name in ('local','global'):
        node.create_subscription(PathMessage,TOPICS[name],lambda m,n=name:emit('PATH',stream=n,points=[(p.pose.position.x,p.pose.position.y) for p in m.poses],stamp=message_to_ordereddict(m.header.stamp)),100)
    result_client=node.create_client(NavigateToPose.Impl.GetResultService,TOPICS['navigate']+'/_action/get_result')
    policy_client=node.create_client(GetPolicyMap,TOPICS['policy']);policy_future=None
    odom_pub=node.create_publisher(Odometry,TOPICS['odom'],1);map_pub=node.create_publisher(GridMap,TOPICS['grid'],reliable)
    tf_pub=node.create_publisher(TFMessage,TOPICS['tf'],10);static_pub=node.create_publisher(TFMessage,TOPICS['tf_static'],transient)
    task_pub=node.create_publisher(PureExplorationTask,TOPICS['task'],10)
    def cancel(t):
        nonlocal cancel_at,cancel_moving
        if cancel_at is not None:return
        m=PureExplorationTask();m.task_id='task3-'+args.case;m.command=m.CANCEL;task_pub.publish(m)
        cancel_at=time.monotonic();cancel_moving=abs(state.v)>.03 or abs(state.w)>.03
        emit('TASK_CANCEL',t=t,state=asdict(state),moving=cancel_moving)
    resolution=.01 if args.case=='large_source_preflight' else .1
    length=5. if args.case=='large_source_preflight' else 10.;n=round(length/resolution)
    coords=length/2-(np.arange(n)+.5)*resolution;x,y=np.meshgrid(coords,coords,indexing='ij')
    measured=np.ones((n,n),bool) if args.case=='large_source_preflight' else measurement_mask(x,y,args.case,args.fixture_version)
    physical_boundary=[(-5.2,-5.2,-5.,5.2),(5.,-5.2,5.2,5.2),(-5.,-5.2,5.,-5.),(-5.,5.,5.,5.2)]
    exception=None;termination=[];process_identity=[]
    proc=subprocess.Popen(launch,stdout=console,stderr=subprocess.STDOUT,start_new_session=True,cwd=output)
    try:
        while time.monotonic()-start<CASES[args.case]:
            now=time.monotonic();remaining=now-last;t=now-start
            while remaining>1e-9:
                dt=min(.01,remaining);a=max(-.5,min(.5,(command[0]-state.v)/dt));alpha=max(-.5,min(.5,(command[1]-state.w)/dt))
                before=state;state=motion.integrate(state,a,alpha,dt);remaining-=dt
                boxes=physical_boundary+([] if obstacle is None else [obstacle])
                row=emit('STATE',t=t-remaining,x=state.x,y=state.y,yaw=state.yaw,v=state.v,w=state.w,a=a,alpha=alpha,
                         clear=motion.swept_clear(before,state,boxes));states.append(row)
            last=now
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.position.x=state.x;odom.pose.pose.position.y=state.y;odom.pose.pose.orientation.z=math.sin(state.yaw/2);odom.pose.pose.orientation.w=math.cos(state.yaw/2)
            odom.twist.twist.linear.x=state.v;odom.twist.twist.angular.z=state.w;odom_pub.publish(odom)
            emit("ODOMETRY",t=t,stamp=message_to_ordereddict(odom.header.stamp))
            if t>=next_tf:
                tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
                msg=TFMessage(transforms=[tf]);tf_pub.publish(msg);static_pub.publish(msg);next_tf=t+.1;emit("TF",t=t,zero_stamp=True)
            for uid,future in list(pending.items()):
                if future is None or not future.done():continue
                result=future.result();pose=next((r['pose'] for r in records if r['kind']=='UUID' and r['uuid']==uid),None)
                row=emit('ACTION_RESULT',uuid=uid,status=result.status,outcome=result.result.outcome,reason=result.result.reason_code,pose=pose)
                results.append(row);pending[uid]=None
            if args.case=='frontier_progress' and not scan and any(r['outcome']==0 and r['status']==4 for r in results):
                measured |= (x-state.x)**2+(y-state.y)**2<=3.2**2;scan=True
                emit('SCAN_NEW',pose=asdict(state),observed_cells=int(measured.sum()))
            if args.case=='blocked_wait_reselect' and obstacle is None and goals and t>=5.:
                gx,gy,_=goals[0]['pose']
                if math.hypot(gx-state.x,gy-state.y)>=1.6:
                    obstacle=(gx-.3,gy-.3,gx+.3,gy+.3);emit('OBSTACLE_ADDED',t=t,box=obstacle,state=asdict(state))
            if t>=next_map:
                height=np.where(measured,0.,np.nan).astype(np.float32)
                if obstacle is not None:
                    x0,y0,x1,y1=obstacle;height[(x>=x0)&(x<=x1)&(y>=y0)&(y<=y1)]=2.
                msg=GridMap();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg();msg.info.resolution=resolution
                msg.info.length_x=msg.info.length_y=length;msg.info.pose.orientation.w=1.;msg.layers=['elevation']
                layer=Float32MultiArray();layer.layout.dim=[MultiArrayDimension(label='column_index',size=n,stride=n*n),MultiArrayDimension(label='row_index',size=n,stride=n)]
                layer.data=height.ravel(order='F').tolist();msg.data=[layer];map_pub.publish(msg)
                count=int(np.isfinite(height).sum());observations.append(emit('MAP',t=t,resolution=resolution,extent=length,cells=n*n,observed_cells=count,obstacle=obstacle))
                if not (output/'source-initial.npy').exists():np.save(output/'source-initial.npy',height)
                np.save(output/'source-final.npy',height);next_map=t+.5
            if strategy=='frontier' and t>=3. and not sent and task_pub.get_subscription_count()>0:
                m=PureExplorationTask();m.task_id='task3-'+args.case;m.command=m.START;m.header.frame_id='map'
                m.boundary.points=[Point32(x=float(a),y=float(b)) for a,b in [(-4.6,-4.6),(4.6,-4.6),(4.6,4.6),(-4.6,4.6)]]
                task_pub.publish(m);sent=True;emit('TASK_START',boundary=message_to_ordereddict(m.boundary))
            if args.case=='task_cancel_stop' and t>=10. and (abs(state.v)>.03 or abs(state.w)>.03 or t>=20.):cancel(t)
            if strategy=='frontier' and t>=CASES[args.case]-8.:cancel(t)
            if t>=next_policy and policy_future is None and policy_client.service_is_ready():
                policy_future=policy_client.call_async(GetPolicyMap.Request());next_policy=t+2.
            if policy_future is not None and policy_future.done():
                value=policy_future.result();row=emit('POLICY',ready=value.ready,reason=value.reason_code,revision=value.fine_revision,
                    processed_stamp=message_to_ordereddict(value.processed_stamp));policy.append(row);policy_future=None
            if t>=next_graph:
                types=dict(node.get_topic_names_and_types());publishers={topic:[dict(node_name=p.node_name,topic_type=p.topic_type,reliability=str(p.qos_profile.reliability),durability=str(p.qos_profile.durability)) for p in node.get_publishers_info_by_topic(topic)] for topic in types}
                subscriptions={topic:[dict(node_name=p.node_name,topic_type=p.topic_type,reliability=str(p.qos_profile.reliability),durability=str(p.qos_profile.durability)) for p in node.get_subscriptions_info_by_topic(topic)] for topic in types}
                graphs.append(emit('GRAPH',types=types,publishers=publishers,subscriptions=subscriptions,nodes=node.get_node_names()));next_graph=t+5.
                process_identity=process_rows(proc.pid);emit('PROCESS_IDENTITY',rows=process_identity)
            if proc.poll() is not None:raise RuntimeError('public launch premature exit '+str(proc.returncode))
            service_cycle(now,lambda timeout:rclpy.spin_once(node,timeout_sec=timeout))
    except BaseException as exc:
        exception=repr(exc);emit('EXCEPTION',error=exception)
    finally:
        if strategy=='frontier':cancel(time.monotonic()-start)
        if proc.poll() is None:proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=8.)
        except subprocess.TimeoutExpired:
            termination.append('SIGTERM');os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=2.)
            except subprocess.TimeoutExpired:termination.append('SIGKILL');os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2.)
        remaining=process_rows(proc.pid)
        emit('CLEANUP',returncode=proc.returncode,escalated=termination,remaining=remaining)
        node.destroy_node();rclpy.shutdown();console.close();stream.close()
    text=(output/'launch.console.log').read_text(errors='replace');formal=[];applied=[]
    for line in text.splitlines():
        for key,rows in [('GLOBAL_RESULT: ',formal),('MAP_APPLIED: ',applied)]:
            if key in line:
                try:rows.append(json.loads(line.split(key,1)[1]))
                except json.JSONDecodeError:pass
    reached=[]
    for row in results:
        if row['outcome']==0 and row['status']==4 and row['pose']:
            reached.append(dict(uuid=row['uuid'],status=row['status'],outcome=row['outcome'],**held(states,row['pose'],row['monotonic_s'])))
    obstacle_event=next((r for r in records if r['kind']=='OBSTACLE_ADDED'),None)
    boxes=lambda t:physical_boundary+([] if obstacle_event is None or t<obstacle_event['t'] else [obstacle_event['box']])
    clear=clearance.measure(states,boxes,.2)
    graph_ok=bool(graphs) and all(not r['publishers'].get('/Car/T5/Car_Cmd_Vel') for r in graphs)
    last_graph=graphs[-1] if graphs else {};pubs=last_graph.get('publishers',{})
    graph_ok &= len(pubs.get(TOPICS['local'],[]))==1 and len(pubs.get(TOPICS['cmd'],[]))==1 and not pubs.get(TOPICS['goal_disabled']) and len(pubs.get(TOPICS['navigate']+'/_action/feedback',[]))==1
    # 安装身份由本次进程原始cmdline/native映射及外部逐文件receipt补证。
    installed=bool(process_identity) and any('/install/lunar_nav1cm_planner/' in r['cmdline'] for r in process_identity) and any('/install/lunar_pure_wheeled_controller/' in r['cmdline'] for r in process_identity)
    no_exceptions=exception is None and not any(s in text for s in ('Traceback (most recent call last)','TICK_ERROR','process has died'))
    odom_times=[r["t"] for r in records if r["kind"]=="ODOMETRY"]
    span=odom_times[-1]-odom_times[0] if len(odom_times)>1 else 0.
    timing=dict(duration_s=span,odometry_count=len(odom_times),odometry_hz=(len(odom_times)-1)/span if span>0 else 0.,map_count=len(observations),tf_count=sum(r["kind"]=="TF" for r in records),state_count=len(states),max_integration_dt_s=.01)
    timing_ok=(abs(span-CASES[args.case])<.5 and 35.<=timing["odometry_hz"]<=55.)
    common=dict(sensor_timing=timing_ok,graph=graph_ok,formal=any(r.get('planning_outcome')==0 and r.get('reason_code')=='PLAN_FOUND' and r.get('has_reference') is True for r in formal),
      motion=bool(states) and max(math.hypot(r['x'],r['y']) for r in states)>.3,
      bounds=bool(states) and all(all(math.isfinite(r[k]) for k in ('x','y','yaw','v','w','a','alpha')) and abs(r['v'])<=.200001 and abs(r['w'])<=1.000001 and abs(r['a'])<=.500001 and abs(r['alpha'])<=.500001 and (abs(r['v'])<1e-8 or abs(r['w'])<=abs(r['v'])+1e-7) for r in states),
      clearance=clear['pass'],cleanup=proc.returncode==0 and not termination and not remaining,
      installed=installed,no_exceptions=no_exceptions,
      source_received=bool(applied) and any(r['ready'] for r in policy) and any(r['data'].get('map_revision',0)>0 for r in diagnostics))
    evidence=dict(case=args.case,fixture_version=args.fixture_version,common=common,sensor_timing=timing,reached=reached,distinct_goals=len(goals),observed_growth=(observations[-1]['observed_cells']-observations[0]['observed_cells']) if observations else 0,
       waits=sum(r['message']['reason_code'] in ('WAITING_FOR_MAP','WAITING_FOR_PATH') for r in feedback),canceled_stopped=sum(r['reason']=='CANCELED_STOPPED' and r['status']==5 and measured_stopped_before(states,r['monotonic_s']) for r in results),
       cancel_while_moving=cancel_moving,no_goals_after_cancel=cancel_at is not None and not any(r['monotonic_s']>cancel_at+.1 for r in goals),
       formal=formal,applied=applied,clearance=clear,goals=goals,results=results,exception=exception,returncode=proc.returncode,termination=termination,
       maxima={k:max((abs(r[k]) for r in states),default=0.) for k in ('v','w','a','alpha')},final_state=asdict(state),
       source_resolution_m=resolution,source_extent_m=length,source_cells=n*n,process_identity=process_identity)
    evidence['independent_plant_modules']=[n for n in sys.modules if n=='nav1cm' or n.startswith(('nav1cm.','lunar_nav1cm_planner.','lunar_pure_wheeled_controller.'))]
    evidence['common']['installed'] &= not evidence['independent_plant_modules']
    summary={**grade(evidence),**evidence};(output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=True))
    print(json.dumps({k:summary[k] for k in ('case','pass_','checks','distinct_goals','observed_growth','waits','canceled_stopped','exception')},ensure_ascii=False),flush=True)
    return 0 if summary['pass_'] else 1

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True,choices=CASES);p.add_argument('--fixture-version',default="v1",choices=("v1","v2"));p.add_argument('--output',required=True);p.add_argument('--build-dir',required=True)
    return run(p.parse_args())
if __name__=='__main__':sys.exit(main())
