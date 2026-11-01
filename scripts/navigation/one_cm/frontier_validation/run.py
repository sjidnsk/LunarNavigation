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
from fixtures import CASES,DECLARATION,measurement_mask,declaration,OBSTACLE_UPDATE_DECLARATION
from grade import grade
from acceptance import AcceptanceLedger, progress_transitions

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

def latest_before(rows, terminal_time):
    available=[r for r in rows if r['monotonic_s']<=terminal_time]
    return available[-1] if available else None

def measured_stopped_before(rows, terminal_time):
    latest=latest_before(rows,terminal_time)
    return bool(latest is not None and terminal_time-latest['monotonic_s']<=.15 and abs(latest['v'])<=.01 and abs(latest['w'])<=.01)

def held(rows,pose,terminal_time,yaw_tolerance_rad=.05):
    start=None;endpoint=None
    for row in rows:
        if row['monotonic_s']>terminal_time:break
        xy=math.hypot(row['x']-pose[0],row['y']-pose[1]);yaw=motion.wrap(row['yaw']-pose[2])
        valid=xy<=.1+1e-9 and abs(yaw)<=yaw_tolerance_rad+1e-9 and abs(row['v'])<=.01+1e-9 and abs(row['w'])<=.01+1e-9
        start=(row['monotonic_s'] if start is None else start) if valid else None
        endpoint=dict(xy_error=xy,yaw_error=yaw,v=row['v'],w=row['w'],hold_s=0. if start is None else row['monotonic_s']-start,measured_at=row['monotonic_s'])
    if endpoint is None:return dict(xy_error=math.inf,yaw_error=math.inf,v=math.inf,w=math.inf,hold_s=0.,sample_age_s=math.inf)
    endpoint['sample_age_s']=terminal_time-endpoint['measured_at']
    if endpoint['sample_age_s']>.15:endpoint['hold_s']=0.
    return endpoint

def validation_settings(args):
    """独立验证器声明实际后端与平台边界，不改变产品配置。"""
    if args.case in ('blocked_wait_reselect','blocked_obstacle_update'):
        raise ValueError('旧取消重选合同已废弃；当前等待保留目标，由状态机回归验证')
    backend=getattr(args,'control_backend','path_mppi')
    if backend not in ('path_mppi','mpcc_cmd_vel'):raise ValueError('invalid control backend')
    profile=getattr(args,'platform_config',None)
    limits=dict(forward=.2,reverse=.2,angular=1.,acceleration=.5,braking=.5,yaw_acceleration=.5)
    if profile:
        import yaml
        profile=str(Path(profile).resolve())
        document=yaml.safe_load(Path(profile).read_text(encoding='utf-8'))
        cap=document['capability']
        if (document.get('platform')!='wheel' or cap.get('motion_model')!='differential_drive'
                or cap.get('body_extent_m')!=[1.182,.818,1.29996]
                or cap.get('footprint_xy_m')!=[[.591,.409],[.591,-.409],[-.591,-.409],[-.591,.409]]
                or cap.get('minimum_clearance_m')!=.2):
            raise ValueError('platform geometry incompatible with independent wheel plant')
        for name,key in [('forward','maximum_forward_speed_mps'),('reverse','maximum_reverse_speed_mps'),('angular','maximum_spin_rate_radps'),('acceleration','maximum_acceleration_mps2'),('braking','maximum_braking_deceleration_mps2'),('yaw_acceleration','maximum_yaw_acceleration_radps2')]:
            value=float(cap[key])
            if not math.isfinite(value) or value<=0:raise ValueError('invalid platform capability '+key)
            limits[name]=value
    strategy='none' if args.case=='large_source_preflight' else 'frontier'
    return dict(strategy=strategy,allow_unknown=False,initial_travel_direction='auto',control_backend=backend,platform_config=profile,limits=limits,goal_yaw_tolerance_rad=math.radians(5.))


def launch_command(args,settings):
    strategy=settings['strategy']
    launch=['ros2','launch','lunar_pure_exploration_ros','nav1cm_exploration.launch.py',
        'exploration_strategy:='+strategy,'enable_controller:=true','build_dir:='+str(Path(args.build_dir).resolve()),'coarse_resolution_m:=0.2', 'control_backend:='+settings['control_backend']]
    if settings['control_backend']=='mpcc_cmd_vel':launch += ['command_hold_s:=1.2','global_period_s:=15.0']
    if settings['platform_config']:launch += ['platform_config:='+settings['platform_config']]
    names={'map_topic':'grid','odometry_topic':'odom','tf_topic':'tf','tf_static_topic':'tf_static','action_name':'navigate','goal_topic':'goal_disabled',
           'global_route_topic':'global','local_path_topic':'local','exploration_map_topic':'exploration_map','policy_map_service':'policy',
           'task_topic':'task','status_topic':'status','command_topic':'cmd','diagnostics_topic':'diag'}
    launch += [name+':='+TOPICS[key] for name,key in names.items()]
    launch += [name+':='+PREFIX+'/'+name for name in ('current_goal_topic','frontiers_topic','task_boundary_topic','task_map_markers_topic','exploration_diagnostics_topic')]
    return launch


def backend_evidence(backend,identities,applied,policy,diagnostics,records,graph):
    """根据实际参与节点核对安装与输入证据；direct不需要MPPI诊断。"""
    planner=any('/install/lunar_nav1cm_planner/' in row['cmdline'] for row in identities)
    follower=any('/install/lunar_pure_wheeled_controller/' in row['cmdline'] for row in identities)
    source=bool(applied) and any(row['ready'] for row in policy)
    expected='lunar_nav1cm_planner' if backend=='mpcc_cmd_vel' else 'lunar_pure_wheeled_controller'
    nodes=graph.get('nodes',[])
    publishers=graph.get('publishers',{}).get(TOPICS['cmd'],[])
    owner=expected in nodes and [p['node_name'] for p in publishers]==[expected]
    if backend=='mpcc_cmd_vel':
        return planner and owner and not follower and 'lunar_pure_wheeled_controller' not in nodes,source and any(row['kind']=='EXPLORATION_MAP' and row['counts'].get('0',0)>0 for row in records)
    return planner and follower and owner and 'lunar_nav1cm_planner' in nodes,source and any(row['data'].get('map_revision',0)>0 for row in diagnostics)


def commands_in_bounds(records,limits):
    for row in records:
        if row['kind']=='COMMAND':
            v,w=row['v'],row['w']
            if not (math.isfinite(v) and math.isfinite(w) and
                    -limits['reverse']-1e-6<=v<=limits['forward']+1e-6 and abs(w)<=limits['angular']+1e-6):
                return False
    return True


def run(args):
    settings=validation_settings(args)
    strategy=settings["strategy"]
    limits=settings["limits"]
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
    declared=declaration(args.fixture_version)
    declared.update(limits=settings["limits"],wait_policy="retain current navigation goal; no timed cancellation",validation_settings=settings,tf_mode=args.tf_mode,identity_contract='exact TASK_ACCEPTED task_id/generation/map goal only; missing/conflicting binding fails qualification and forbids obstacle',terminal_contract='latest measurement before result receipt; continuous valid suffix to latest endpoint; freshness<=.15s; cancel uses latest sample')
    if args.case=='blocked_obstacle_update':
        if args.fixture_version!='v1':raise ValueError('blocked_obstacle_update uses qualified progress v1 initial source')
        declared.pop('blocked',None)
        declared['progress_scan']='disabled; only actual obstacle event changes source'
        declared['case_baseline']='qualified progress initial source without unknown slit'
        declared.update(obstacle_update=OBSTACLE_UPDATE_DECLARATION)
    (output/'declaration.json').write_text(json.dumps(declared,ensure_ascii=False,indent=2))
    (output/'harness-identity.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for folder in (HERE,HERE.parent/'p3_path_validation',HERE.parent/'path_follower_validation') for f in folder.glob('*.py')},indent=2))
    stream=(output/'plant.jsonl').open('x',encoding='utf-8')
    records=[];states=[];results=[];feedback=[];goals=[];pending={};graphs=[];policy=[];observations=[];diagnostics=[]
    def emit(kind,**values):
        row=dict(kind=kind,monotonic_s=time.monotonic(),**values);records.append(row)
        stream.write(json.dumps(row,ensure_ascii=False,allow_nan=True)+'\n');stream.flush();return row
    launch=launch_command(args,settings)
    emit('COMMAND_CONTRACT',argv=launch,domain=os.environ['ROS_DOMAIN_ID'],localhost=os.environ['ROS_LOCALHOST_ONLY'])
    console=(output/'launch.console.log').open('x')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=Node('nav1cm_task3_independent_plant')
    reliable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    transient=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    command=[0.,0.];state=motion.State();start=time.monotonic();last=start;cancel_at=None;cancel_moving=False;sent=False;scan=False;obstacle=None
    accepted_goals=[];accepted_ids=set();active_status={};ledger=AcceptanceLedger();notice_position=0;notice_partial=""
    last_status=None;next_map=next_tf=next_graph=next_policy=0.;observed_count=0
    def receive_cmd(m):
        command[:]=[m.linear.x,m.angular.z];emit('COMMAND',v=command[0],w=command[1])
    def status(m):
        nonlocal last_status
        last_status=m;emit('STATUS',message=message_to_ordereddict(m))
    def receive_result(uid,future):
        result=future.result()
        results.append(emit('ACTION_RESULT',uuid=uid,status=result.status,outcome=result.result.outcome,reason=result.result.reason_code))
        ledger.finish(uid)
    def get_result(uuid):
        uid=bytes(uuid).hex();ledger.observe_uuid(uid)
        if uid in pending:return
        request=NavigateToPose.Impl.GetResultService.Request();request.goal_id=UUID(uuid=uuid_values(uuid))
        future=result_client.call_async(request);pending[uid]=future
        future.add_done_callback(lambda done,u=uid:receive_result(u,done))
        emit('UUID',uuid=uid)  # 跨topic的current_goal不能提供身份绑定。
    def action_feedback(m):
        row=emit('ACTION_FEEDBACK',uuid=bytes(m.goal_id.uuid).hex(),message=message_to_ordereddict(m.feedback));feedback.append(row)
        get_result(m.goal_id.uuid)
    def action_status(m):
        for item in m.status_list:
            uid=bytes(item.goal_info.goal_id.uuid).hex();active_status[uid]=int(item.status)
            if int(item.status) in (4,5,6):ledger.finish(uid)
            get_result(item.goal_info.goal_id.uuid)
    def read_acceptances():
        nonlocal notice_position,notice_partial
        with (output/'launch.console.log').open(encoding='utf-8',errors='replace') as source:
            source.seek(notice_position);notice_partial+=source.read();notice_position=source.tell()
        complete=notice_partial.split('\n');notice_partial=complete.pop()
        for line in complete:
            item=ledger.parse(line,time.monotonic())
            if item is not None and item.task_id not in accepted_ids:
                accepted_ids.add(item.task_id)
                accepted_goals.append(emit('ACTION_LOGICAL_ACCEPTED',uuid=item.task_id,generation=item.generation,
                    pose=list(item.goal),logical_accepted=True,binding_source='TASK_ACCEPTED',current_at_receipt=ledger.current is item,
                    notice_line=line))
    def exploration(m):emit('EXPLORATION_MAP',stamp=message_to_ordereddict(m.header.stamp),resolution=m.info.resolution,counts={str(i):m.data.count(i) for i in (-1,0,100)})
    def diag(m):
        row=emit('DIAGNOSTIC',data=json.loads(m.data));diagnostics.append(row)
    def goal(m):
        p=m.pose.position;q=m.pose.orientation;pose=(p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
        ledger.observe_frontend_goal(pose)
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
    tf_pub=(node.create_publisher(TFMessage,TOPICS['tf'],10) if args.tf_mode=='paired' else None)
    static_pub=node.create_publisher(TFMessage,TOPICS['tf_static'],transient)
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
    if args.case=='blocked_obstacle_update':
        import io
        initial=np.where(measured,0.,np.nan).astype(np.float32);wire=io.BytesIO();np.save(wire,initial)
        assert hashlib.sha256(wire.getvalue()).hexdigest()=='70d26980e9c32fd604f8578f5bac3895f79860c49a08be2c9625e8b1f707889f'
        emit('INITIAL_SUPPORT_BINDING',source_sha256='70d26980e9c32fd604f8578f5bac3895f79860c49a08be2c9625e8b1f707889f',baseline='qualified progress initial source')
    physical_boundary=[(-5.2,-5.2,-5.,5.2),(5.,-5.2,5.2,5.2),(-5.,-5.2,5.,-5.),(-5.,5.,5.,5.2)]
    exception=None;termination=[];process_identity=[]
    if args.tf_mode=='static_only_latched':
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
        static_pub.publish(TFMessage(transforms=[tf]))
        emit('TF',t=time.monotonic()-start,zero_stamp=True,mode=args.tf_mode,before_launch=True)
    proc=subprocess.Popen(launch,stdout=console,stderr=subprocess.STDOUT,start_new_session=True,cwd=output)
    launch_started=emit('LAUNCH_STARTED',pid=proc.pid)
    try:
        while time.monotonic()-start<CASES[args.case]:
            now=time.monotonic();remaining=now-last;t=now-start
            while remaining>1e-9:
                dt=min(.01,remaining);a=max(-limits['braking'],min(limits['acceleration'],(command[0]-state.v)/dt));alpha=max(-limits['yaw_acceleration'],min(limits['yaw_acceleration'],(command[1]-state.w)/dt))
                before=state;state=motion.integrate(state,a,alpha,dt);remaining-=dt
                boxes=physical_boundary+([] if obstacle is None else [obstacle])
                row=emit('STATE',t=t-remaining,x=state.x,y=state.y,yaw=state.yaw,v=state.v,w=state.w,a=a,alpha=alpha,
                         clear=motion.swept_clear(before,state,boxes));states.append(row)
            last=now
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.position.x=state.x;odom.pose.pose.position.y=state.y;odom.pose.pose.orientation.z=math.sin(state.yaw/2);odom.pose.pose.orientation.w=math.cos(state.yaw/2)
            odom.twist.twist.linear.x=state.v;odom.twist.twist.angular.z=state.w;odom_pub.publish(odom)
            emit("ODOMETRY",t=t,stamp=message_to_ordereddict(odom.header.stamp))
            if args.tf_mode=='paired' and t>=next_tf:
                tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
                msg=TFMessage(transforms=[tf]);tf_pub.publish(msg);static_pub.publish(msg);next_tf=t+.1;emit("TF",t=t,zero_stamp=True)
            read_acceptances()
            if args.case=='frontier_progress' and not scan and any(r['outcome']==0 and r['status']==4 and ledger.get(r['uuid']) is not None for r in results):
                measured |= (x-state.x)**2+(y-state.y)**2<=3.2**2;scan=True
                emit('SCAN_NEW',pose=asdict(state),observed_cells=int(measured.sum()))
            if args.case=='blocked_wait_reselect' and obstacle is None and ledger.current is not None and t>=5.:
                gx,gy,_=ledger.current.goal
                if math.hypot(gx-state.x,gy-state.y)>=1.6:
                    obstacle=(gx-.3,gy-.3,gx+.3,gy+.3);emit('OBSTACLE_ADDED',t=t,box=obstacle,state=asdict(state))
            if args.case=='blocked_obstacle_update' and obstacle is None and ledger.current is not None and active_status.get(ledger.current.task_id) in (1,2):
                accepted=ledger.current;pose=accepted.goal
                distance=math.hypot(pose[0]-state.x,pose[1]-state.y)
                if distance>=1.6:
                    gx,gy,_=pose;obstacle=(gx-.3,gy-.3,gx+.3,gy+.3)
                    emit('OBSTACLE_ADDED',t=t,box=obstacle,state=asdict(state),uuid=accepted.task_id,pose=pose,
                         distance_m=distance,accepted_at=accepted.observed_at,generation=accepted.generation,
                         binding_source='TASK_ACCEPTED',active=True)
            if t>=next_map:
                height=np.where(measured,0.,np.nan).astype(np.float32)
                if obstacle is not None:
                    x0,y0,x1,y1=obstacle;height[(x>=x0)&(x<=x1)&(y>=y0)&(y<=y1)]=2.
                msg=GridMap();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg();msg.info.resolution=resolution
                msg.info.length_x=msg.info.length_y=length;msg.info.pose.orientation.w=1.;msg.layers=['elevation']
                layer=Float32MultiArray();layer.layout.dim=[MultiArrayDimension(label='column_index',size=n,stride=n*n),MultiArrayDimension(label='row_index',size=n,stride=n)]
                layer.data=height.ravel(order='F').tolist();msg.data=[layer];map_pub.publish(msg)
                count=int(np.isfinite(height).sum());observations.append(emit('MAP',t=t,resolution=resolution,extent=length,cells=n*n,observed_cells=count,obstacle=obstacle,stamp=message_to_ordereddict(msg.header.stamp),source_sha256=hashlib.sha256(height.tobytes()).hexdigest()))
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
        read_acceptances()
        node.destroy_node();rclpy.shutdown();console.close();stream.close()
    text=(output/'launch.console.log').read_text(errors='replace');formal=[];applied=[]
    for line in text.splitlines():
        for key,rows in [('GLOBAL_RESULT: ',formal),('MAP_APPLIED: ',applied)]:
            if key in line:
                try:rows.append(json.loads(line.split(key,1)[1]))
                except json.JSONDecodeError:pass
    # 派生归因只按权威通知exactID；原始ACTION_RESULT不猜测或回填topic最近坐标。
    results=[dict(r,pose=(list(ledger.get(r['uuid']).goal) if ledger.get(r['uuid']) else None),
                  accepted_generation=(ledger.get(r['uuid']).generation if ledger.get(r['uuid']) else None)) for r in results]
    reached=[]
    for row in results:
        if row['outcome']==0 and row['status']==4 and row['pose']:
            reached.append(dict(uuid=row['uuid'],status=row['status'],outcome=row['outcome'],**held(states,row['pose'],row['monotonic_s'],settings['goal_yaw_tolerance_rad'])))
    obstacle_event=next((r for r in records if r['kind']=='OBSTACLE_ADDED'),None)
    boxes=lambda t:physical_boundary+([] if obstacle_event is None or t<obstacle_event['t'] else [obstacle_event['box']])
    clear=clearance.measure(states,boxes,.2)
    graph_ok=bool(graphs) and all(not r['publishers'].get('/Car/T5/Car_Cmd_Vel') for r in graphs)
    last_graph=graphs[-1] if graphs else {};pubs=last_graph.get('publishers',{})
    graph_ok &= len(pubs.get(TOPICS['local'],[]))==1 and len(pubs.get(TOPICS['cmd'],[]))==1 and not pubs.get(TOPICS['goal_disabled']) and len(pubs.get(TOPICS['navigate']+'/_action/feedback',[]))==1
    # 安装身份由本次进程原始cmdline/native映射及外部逐文件receipt补证。
    installed,source_received=backend_evidence(settings['control_backend'],process_identity,applied,policy,diagnostics,records,last_graph)
    no_exceptions=exception is None and not any(s in text for s in ('Traceback (most recent call last)','TICK_ERROR','process has died'))
    odom_times=[r["t"] for r in records if r["kind"]=="ODOMETRY"]
    span=odom_times[-1]-odom_times[0] if len(odom_times)>1 else 0.
    timing=dict(duration_s=span,odometry_count=len(odom_times),odometry_hz=(len(odom_times)-1)/span if span>0 else 0.,map_count=len(observations),tf_count=sum(r["kind"]=="TF" for r in records),state_count=len(states),max_integration_dt_s=.01)
    timing_ok=(abs(span-CASES[args.case])<.5 and 35.<=timing["odometry_hz"]<=55.)
    required_ids={r['uuid'] for r in feedback if r['message']['reason_code']=='GOAL_ACCEPTED'} | {r['uuid'] for r in results if r['reason'] in ('GOAL_REACHED','CANCELED_STOPPED')}
    identity_ok=ledger.ok and all(ledger.get(uid) is not None for uid in required_ids)
    for r in formal:
        uid=ledger.generations.get(r.get('accepted_generation'));binding=ledger.get(uid)
        if binding is None or r.get('requested_goal') is None or any(abs(a-b)>1e-9 for a,b in zip(binding.goal[:2],r['requested_goal'])):identity_ok=False
    common=dict(authoritative_identity=identity_ok,sensor_timing=timing_ok,graph=graph_ok,formal=any(r.get('planning_outcome')==0 and r.get('reason_code')=='PLAN_FOUND' and r.get('has_reference') is True for r in formal),
      motion=bool(states) and max(math.hypot(r['x'],r['y']) for r in states)>.3,
      bounds=bool(states) and commands_in_bounds(records,limits) and all(all(math.isfinite(r[k]) for k in ('x','y','yaw','v','w','a','alpha')) and -limits['reverse']-1e-6<=r['v']<=limits['forward']+1e-6 and abs(r['w'])<=limits['angular']+1e-6 and -limits['braking']-1e-6<=r['a']<=limits['acceleration']+1e-6 and abs(r['alpha'])<=limits['yaw_acceleration']+1e-6 for r in states),
      clearance=clear['pass'],cleanup=proc.returncode==0 and not termination and not remaining,
      installed=installed,no_exceptions=no_exceptions,
      source_received=source_received)
    evidence=dict(progress_transitions=progress_transitions(text.splitlines()),validation_settings=settings,goal_yaw_tolerance_rad=settings["goal_yaw_tolerance_rad"],case=args.case,tf_mode=args.tf_mode,fixture_version=args.fixture_version,common=common,sensor_timing=timing,reached=reached,distinct_goals=len({tuple(r['pose']) for r in accepted_goals}),observed_growth=(observations[-1]['observed_cells']-observations[0]['observed_cells']) if observations else 0,
       waits=sum(r['message']['reason_code'] in ('WAITING_FOR_MAP','WAITING_FOR_PATH') for r in feedback),canceled_stopped=sum(r['reason']=='CANCELED_STOPPED' and r['status']==5 and measured_stopped_before(states,r['monotonic_s']) for r in results),
       cancel_while_moving=cancel_moving,no_goals_after_cancel=cancel_at is not None and not any(r['monotonic_s']>cancel_at+.1 for r in accepted_goals),
       formal=formal,applied=applied,clearance=clear,goals=accepted_goals,current_goal_observations=goals,results=results,acceptance_errors=ledger.errors,required_task_ids=sorted(required_ids),exception=exception,returncode=proc.returncode,termination=termination,
       maxima={k:max((abs(r[k]) for r in states),default=0.) for k in ('v','w','a','alpha')},final_state=asdict(state),
       source_resolution_m=resolution,source_extent_m=length,source_cells=n*n,process_identity=process_identity)
    if args.case=='blocked_obstacle_update':
        event=next((r for r in records if r['kind']=='OBSTACLE_ADDED'),None);sequence=None
        if event:
            initial=observations[0]['source_sha256']
            source=next((r for r in observations if r['monotonic_s']>=event['monotonic_s'] and r['source_sha256']!=initial),None)
            before=[r for r in records if r['kind']=='EXPLORATION_MAP' and r['monotonic_s']<event['monotonic_s']]
            occupied=max((r['counts']['100'] for r in before),default=0)
            changed=next((r for r in records if source and r['kind']=='EXPLORATION_MAP' and r['monotonic_s']>=source['monotonic_s'] and r['counts']['100']>occupied),None)
            wait=next((r for r in feedback if changed and r['monotonic_s']>=changed['monotonic_s'] and r['uuid']==event['uuid'] and r['message']['reason_code'] in ('WAITING_FOR_PATH','WAITING_FOR_MAP')),None)
            terminal=next((r for r in results if wait and r['monotonic_s']>=wait['monotonic_s'] and r['uuid']==event['uuid'] and r['reason']=='CANCELED_STOPPED'),None)
            nxt=next((r for r in accepted_goals if terminal and r['monotonic_s']>terminal['monotonic_s'] and r['uuid']!=event['uuid'] and r['generation']>event['generation'] and r['current_at_receipt'] and r['pose'] and math.hypot(r['pose'][0]-event['pose'][0],r['pose'][1]-event['pose'][1])>.1),None)
            sequence=dict(event=event,source=None if source is None else dict(source,changed=True),map=None if changed is None else dict(changed,changed=True),
                          wait=None if wait is None else dict(wait,reason=wait['message']['reason_code']),terminal=None if terminal is None else dict(terminal,stopped_before=measured_stopped_before(states,terminal['monotonic_s'])),next=nxt)
        evidence['obstacle_sequence']=sequence
        evidence['obstacle_event_count']=sum(r['kind']=='OBSTACLE_ADDED' for r in records)
        evidence['logical_accepted_goals']=accepted_goals
    if args.tf_mode=='static_only_latched':
        tf_rows=[r for r in records if r['kind']=='TF']
        evidence['static_tf_contract']=bool(len(tf_rows)==1 and tf_rows[0]['monotonic_s']<launch_started['monotonic_s'] and tf_pub is None and not pubs.get(TOPICS['tf']))
    evidence['independent_plant_modules']=[n for n in sys.modules if n=='nav1cm' or n.startswith(('nav1cm.','lunar_nav1cm_planner.','lunar_pure_wheeled_controller.'))]
    evidence['common']['installed'] &= not evidence['independent_plant_modules']
    summary={**grade(evidence),**evidence};(output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=True))
    print(json.dumps({k:summary[k] for k in ('case','pass_','checks','distinct_goals','observed_growth','waits','canceled_stopped','exception')},ensure_ascii=False),flush=True)
    return 0 if summary['pass_'] else 1

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',required=True,choices=CASES);p.add_argument('--fixture-version',default="v1",choices=("v1","v2"));p.add_argument('--output',required=True);p.add_argument('--build-dir',required=True);p.add_argument('--tf-mode',default='paired',choices=('paired','static_only_latched'))
    p.add_argument('--control-backend',default='path_mppi',choices=('path_mppi','mpcc_cmd_vel'));p.add_argument('--platform-config')
    return run(p.parse_args())
if __name__=='__main__':sys.exit(main())
