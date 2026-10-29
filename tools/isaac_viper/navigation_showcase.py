"""Humble cp310 原 A 已知图助手：真实 Action/odom/TF/规划输出；绝不发布车辆控制。"""
import argparse
import copy
import hashlib
from array import array
import json
import math
from pathlib import Path
import time
import uuid
import numpy as np
from tools.isaac_viper.viewport_paths import SCHEMA,validate_payload,validate_points


def configure_grid_info(info,bounds,resolution):
    """NPZ/JSON 数值进入生成的 ROS double setter 前必须转原生 float。"""
    x0,x1,y0,y1=(float(value) for value in bounds)
    info.resolution=float(resolution)
    info.length_x=float(x1-x0); info.length_y=float(y1-y0)
    info.pose.position.x=float((x0+x1)/2); info.pose.position.y=float((y0+y1)/2)
    info.pose.position.z=0.0
    info.pose.orientation.x=0.0; info.pose.orientation.y=0.0; info.pose.orientation.z=0.0; info.pose.orientation.w=1.0


def configure_goal(goal,target_xy,yaw):
    goal.target_x_m=float(target_xy[0]); goal.target_y_m=float(target_xy[1])
    goal.has_target_yaw=True; goal.target_yaw_rad=float(yaw)


def grid_encoding(elevation):
    grid=np.asarray(elevation,dtype=np.float32)
    if grid.ndim!=2 or min(grid.shape)==0 or np.isinf(grid).any(): raise ValueError('invalid elevation grid')
    ny,nx=grid.shape
    return array('f',grid[::-1,::-1].ravel()),[('column_index',ny,nx*ny),('row_index',nx,nx)]


def transform_point(point,rotation,translation):
    p,q,t=np.asarray(point,float),np.asarray(rotation,float),np.asarray(translation,float)
    if p.shape!=(3,) or q.shape!=(4,) or t.shape!=(3,) or not all(np.isfinite(v).all() for v in (p,q,t)) or abs(np.linalg.norm(q)-1)>1e-3: raise ValueError('invalid pose/TF')
    xyz=q[:3]; return (p+2*np.cross(xyz,np.cross(xyz,p)+q[3]*p)+t).tolist()


class ActualTrace:
    def __init__(self,limit=10000): self.limit=limit; self.points=[]; self.highwater=-1
    def reset(self):
        self.points.clear()  # 新 goal 清轨迹，但不退回真实 stamp 高水位。
    def append(self,stamp_ns,xyz,rotation,translation):
        if not isinstance(stamp_ns,int) or stamp_ns<0: raise ValueError('invalid odom stamp')
        if stamp_ns<self.highwater: raise ValueError('odom stamp rollback')
        if stamp_ns==self.highwater: return False
        point=transform_point(xyz,rotation,translation)
        self.highwater=stamp_ns; self.points.append(dict(stamp_ns=stamp_ns,xyz=point))
        if len(self.points)>self.limit: self.points.pop(0)
        return True


def planning_evidence(diag,reference):
    try:
        matched=(diag.get('cycle_result')=='PLAN_FOUND' and diag.get('reason_code')=='PLAN_FOUND' and reference.get('state')==0 and bool(reference.get('points'))
            and diag.get('session_id')==reference.get('session_id') and int(diag['segment_revision'])==reference['segment_revision']
            and int(diag['fine_traversability_revision'])==reference['traversability_revision'])
    except (KeyError,ValueError,TypeError): matched=False
    return dict(planning_outcome=0 if matched else None,has_reference=bool(matched),derivation='matching original DiagnosticArray PLAN_FOUND and nonempty ACTIVE PathReference; not Action result')


class DisplayPlanSession:
    """显示限定本 Action：local 取带 UUID 的 reference，global 只关联新 volatile 样本。"""
    def __init__(self):
        self.begin_goal(None,0)

    def begin_goal(self,session_id,request_source_time_ns):
        self.session_id=session_id; self.request_source_time_ns=request_source_time_ns
        self.accepted=False; self.goal_state='pending' if session_id else 'waiting'; self.terminal=False
        self.reference=None; self.reference_transport_stamp_ns=0; self.invalidated=set()
        self.candidates=[]; self.diagnostics={}; self.live={}; self.history={}

    def accept_goal(self,session_id):
        if session_id!=self.session_id: raise ValueError('accepted goal UUID mismatch')
        self.accepted=True; self.goal_state='active'; self._refresh()

    def observe_global(self,path,transport_source_stamp_ns):
        validate_points(path['points'])
        if self.terminal or not self.session_id or transport_source_stamp_ns<self.request_source_time_ns or transport_source_stamp_ns<=0: return False
        # 订阅是 VOLATILE；source_timestamp 是 DDS 原始发布时刻，不伪造 Path header。
        self.candidates.append((transport_source_stamp_ns,copy.deepcopy(path)))
        self.candidates=self.candidates[-64:]; self._refresh(); return True

    def observe_reference(self,reference,transport_source_stamp_ns):
        if self.terminal or reference['session_id']!=self.session_id: return False
        key=(reference['session_id'],reference['segment_revision'],reference['traversability_revision'])
        if transport_source_stamp_ns<self.reference_transport_stamp_ns: return False
        if reference['state']==0 and key in self.invalidated: return False
        if self.reference and (reference['segment_revision']<self.reference['segment_revision'] or
            (reference['segment_revision']==self.reference['segment_revision'] and reference['traversability_revision']<self.reference['traversability_revision'])): return False
        self.reference=copy.deepcopy(reference); self.reference_transport_stamp_ns=transport_source_stamp_ns
        if reference['state']!=0:
            self.invalidated.add(key); self.live={}; self.goal_state='invalidated'; return True
        self.goal_state='active' if self.accepted else 'pending'; self._refresh(); return True

    def observe_diagnostics(self,diagnostic):
        if diagnostic.get('session_id')!=self.session_id: return False
        key=(diagnostic['session_id'],int(diagnostic['segment_revision']),int(diagnostic['fine_traversability_revision']))
        self.diagnostics[key]=copy.deepcopy(diagnostic); self._refresh(); return True

    def _refresh(self):
        self.live={}
        ref=self.reference
        if self.terminal or not self.accepted or not ref or ref['state']!=0 or not ref['points']: return
        key=(ref['session_id'],ref['segment_revision'],ref['traversability_revision'])
        if not planning_evidence(self.diagnostics.get(key,{}),ref)['has_reference']: return
        binding=dict(session_id=ref['session_id'],segment_revision=ref['segment_revision'],traversability_revision=ref['traversability_revision'],reference_stamp_ns=ref['stamp_ns'])
        self.live['local_path']=dict(stamp_ns=ref['stamp_ns'],points=copy.deepcopy(ref['points']),binding=copy.deepcopy(binding),verified_reference=copy.deepcopy(ref),display_status='live')
        for source,path in reversed(self.candidates):
            if 0<=self.reference_transport_stamp_ns-source<=1000000000:
                if path['points']:
                    global_binding=dict(binding,request_source_time_ns=self.request_source_time_ns,transport_source_stamp_ns=source,reference_transport_stamp_ns=self.reference_transport_stamp_ns,
                        association='volatile sample after this goal request, immediately before matching ACTIVE reference and PLAN_FOUND; Path has no native UUID')
                    self.live['global_route']=dict(copy.deepcopy(path),binding=global_binding,verified_reference=copy.deepcopy(ref),display_status='live')
                break
        # 每类只冻结首份已验证几何，完整初始全局路线不随末端短段消失。
        for name,path in self.live.items():
            if name not in self.history:
                self.history[name]=copy.deepcopy(path); self.history[name]['display_status']='history'

    def finish(self,status):
        if status not in ('completed','failed','timeout','cancelled'): raise ValueError('invalid terminal display state')
        self.terminal=True; self.goal_state=status; self.live={}

    def snapshot(self):
        return dict(action_session_id=self.session_id,goal_accepted=self.accepted,goal_state=self.goal_state,
                    path_reference=copy.deepcopy(self.reference),paths=copy.deepcopy(self.live),history_paths=copy.deepcopy(self.history))


def long_route_evidence(display,scene):
    """实际新navigator global长度与fixture分列；没有原消息绑定时保持未满足。"""
    fixture=scene.get('fixture_precheck',{}); minimum=float(fixture.get('minimum_required_m',100.))
    value=dict(satisfied=False,minimum_required_m=minimum,navigator_global_length_m=None,
        source='verified original navigator global_route; accepted Action + matching PLAN_FOUND/ACTIVE reference',
        offline_fixture_length_m=fixture.get('path_length_m'),goal_reached_is_separate=True)
    path=display.history.get('global_route')
    if not display.accepted or not path or path.get('binding',{}).get('session_id')!=display.session_id: return value
    points=path['points']; length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
    value.update(navigator_global_length_m=length,session_id=display.session_id,
        segment_revision=path['binding']['segment_revision'],traversability_revision=path['binding']['traversability_revision'],
        fixture_straight_line_real_obstacle_blocked=fixture.get('line_of_sight_blocked') is True)
    blocker=fixture.get('blocked_cell_xy_m'); distance=None
    if blocker is not None and len(points)>=2:
        distance=math.inf
        for a,b in zip(points,points[1:]):
            dx,dy=b[0]-a[0],b[1]-a[1]; norm=dx*dx+dy*dy
            t=0. if norm==0 else max(0.,min(1.,((blocker[0]-a[0])*dx+(blocker[1]-a[1])*dy)/norm))
            distance=min(distance,math.hypot(blocker[0]-a[0]-t*dx,blocker[1]-a[1]-t*dy))
    required_clearance=math.hypot(.591,.409)+.5+float(scene.get('resolution_m',.2))*math.sqrt(.5)
    detour=distance is not None and distance>=required_clearance
    value.update(blocker_to_navigator_polyline_clearance_m=distance,required_blocker_clearance_m=required_clearance,
        detour_around_fixture_obstacle=detour,collision_runtime='NOT_RUN; this is a static XY display/fixture check')
    value['satisfied']=length>=minimum and fixture.get('line_of_sight_blocked') is True and detour
    return value


def _stamp_ns(stamp): return int(stamp.sec)*1000000000+int(stamp.nanosec)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-json',required=True); parser.add_argument('--output-dir',required=True); parser.add_argument('--overlay-file',required=True)
    parser.add_argument('--send-goal',action='store_true',help='显式发送一次真实 NavigateToPose；缺省仅发布图和监听')
    parser.add_argument('--wall-timeout',type=float,default=300); parser.add_argument('--sim-timeout',type=float,default=150)
    parser.add_argument('--freshness-wall',type=float,default=2.,help='发送目标前真实 odom/TF 接收年龄上限；不改变控制器阈值')
    args=parser.parse_args()
    if min(args.wall_timeout,args.sim_timeout,args.freshness_wall)<=0: parser.error('timeouts must be positive')
    scene_path=Path(args.scene_json).resolve(); scene=json.loads(scene_path.read_text(encoding='utf-8'))
    if scene.get('schema')!='lunar.viper-navigation-scene.v1' or scene.get('region')!='A' or scene.get('resolution_m')!=.2: raise ValueError('only explicit original A known map is authorized')
    if hashlib.sha256(Path(scene['map_npz']).read_bytes()).hexdigest()!=scene['map_sha256']: raise ValueError('known map hash mismatch')
    archive=np.load(scene['map_npz'],allow_pickle=False); elevation=archive['elevation']; bounds=archive['bounds_xy_m'].tolist(); resolution=float(archive['resolution_m'])
    if bounds!=scene['bounds_xy_m'] or resolution!=.2 or elevation.shape!=(round((bounds[3]-bounds[2])/resolution),round((bounds[1]-bounds[0])/resolution)): raise ValueError('map/scene identity mismatch')
    data,dims=grid_encoding(elevation)
    output=Path(args.output_dir).resolve(); output.mkdir(parents=True,exist_ok=False)
    overlay=Path(args.overlay_file).resolve()
    if overlay.exists(): raise FileExistsError('overlay belongs to an existing session')
    overlay.parent.mkdir(parents=True,exist_ok=True)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from grid_map_msgs.msg import GridMap
    from std_msgs.msg import Float32MultiArray,MultiArrayDimension
    from nav_msgs.msg import Odometry,Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from diagnostic_msgs.msg import DiagnosticArray
    from lunar_planning_msgs.msg import PathReference
    from lunar_planning_msgs.action import NavigateToPose
    from unique_identifier_msgs.msg import UUID
    from rosidl_runtime_py.convert import message_to_ordereddict
    rclpy.init(); node=rclpy.create_node('viper_navigation_showcase')
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    route_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    best=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    map_pub=node.create_publisher(GridMap,'/Car/T3/mapping/grid_map',latched)
    trace_pub=node.create_publisher(RosPath,'/Car/T4/showcase/actual_trace',latched)
    action=ActionClient(node,NavigateToPose,'/Car/T4/navigation/navigate_to_pose')
    run_id=uuid.uuid4().hex; events=(output/'events.jsonl').open('w',encoding='utf-8',buffering=1)
    state=dict(run_id=run_id,scene=scene,argv=vars(args),clock_ns=None,first_clock_ns=None,map_publish_count=0,
               goal_sent=False,goal_accepted=False,goal_reached=False,planning_evidence=dict(planning_outcome=None,has_reference=False),NOT_RUN=['Orin','real_vehicle','C Actor'])
    display=DisplayPlanSession(); paths={}; trace=ActualTrace(); pending={}; transforms={}; latest={}; references={}; diagnostics={}; sequence=0
    wall_start=time.monotonic(); last_map=last_overlay=-1.; goal_handle=None; send_future=result_future=cancel_future=None
    def record(kind,**values):
        events.write(json.dumps(dict(event=kind,wall_elapsed_s=time.monotonic()-wall_start,**values),ensure_ascii=False,allow_nan=False)+'\n')
    def raw(kind,msg): record(kind,message=message_to_ordereddict(msg))
    def save(): (output/'result.json').write_text(json.dumps(state,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    def stamp_message(stamp,ns): stamp.sec=ns//1000000000; stamp.nanosec=ns%1000000000
    def match():
        for key,ref in references.items():
            diag=diagnostics.get(key)
            if not diag or ref['session_id']!=state.get('action_session_id'): continue
            evidence=planning_evidence(diag,ref)
            if evidence['has_reference'] and not state['planning_evidence']['has_reference']:
                state['planning_evidence']=dict(evidence,diagnostics=diag,path_reference=ref)
                record('planning_evidence',**state['planning_evidence']); save()
    def path_value(msg):
        if msg.header.frame_id!='map': raise ValueError('planner path frame must be map')
        points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]
        value=dict(stamp_ns=_stamp_ns(msg.header.stamp),points=points)
        validate_points(points)
        return value
    def on_path(name,msg,info=None):
        value=path_value(msg); raw(name,msg)
        if name=='global_route':
            source=int(getattr(info,'source_timestamp',0))
            accepted=display.observe_global(value,source)
            record('global_display_candidate',transport_source_stamp_ns=source,accepted=accepted)
    def consume(stamp):
        if stamp not in pending or stamp not in transforms: return
        msg=pending.pop(stamp); q,t=transforms[stamp]; p=msg.pose.pose.position
        o=msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (o.x,o.y,o.z,o.w)) or abs(math.sqrt(o.x*o.x+o.y*o.y+o.z*o.z+o.w*o.w)-1)>1e-3: raise ValueError('invalid odom orientation')
        if not trace.append(stamp,[p.x,p.y,p.z],q,t): return
        latest['odom_wall']=time.monotonic(); latest['odom_stamp']=stamp
        state['actual_trace_samples']=len(trace.points); state['last_odom_stamp_ns']=stamp; state['last_actual_xyz']=trace.points[-1]['xyz']
        paths['actual_trace']=dict(stamp_ns=stamp,points=[p['xyz'] for p in trace.points])
        raw('odometry',msg)
        record('actual_odom',stamp_ns=stamp,xyz=trace.points[-1]['xyz'],tf_stamp_ns=stamp)
    def on_odom(msg):
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link': raise ValueError('odom frame mismatch')
        ns=_stamp_ns(msg.header.stamp)
        if ns<trace.highwater: raise ValueError('odom stamp rollback')
        if ns==trace.highwater: return
        pending[ns]=msg
        if len(pending)>200: raise RuntimeError('odom lacks exact-stamp direct map to odom TF')
        consume(ns)
    def on_tf(msg):
        for tf in msg.transforms:
            if tf.header.frame_id!='map' or tf.child_frame_id!='odom': continue
            ns=_stamp_ns(tf.header.stamp); q=tf.transform.rotation; t=tf.transform.translation
            rotation=[q.x,q.y,q.z,q.w]; translation=[t.x,t.y,t.z]; transform_point([0,0,0],rotation,translation)
            if max(abs(v) for v in translation+rotation[:3])>1e-6 or abs(abs(rotation[3])-1)>1e-6: raise ValueError('original scene requires identity map to odom')
            raw('map_to_odom',tf)
            transforms[ns]=(rotation,translation)
            latest['tf_wall']=time.monotonic(); latest['tf_stamp']=ns
            consume(ns)
        if len(transforms)>2000:
            for ns in sorted(transforms)[:-1000]: transforms.pop(ns)
    def on_clock(msg):
        ns=_stamp_ns(msg.clock)
        if state['clock_ns'] is not None and ns<state['clock_ns']: raise ValueError('simulation clock rollback')
        state['clock_ns']=ns
        if state['first_clock_ns'] is None: state['first_clock_ns']=ns
    def on_reference(msg,info):
        value=path_value(msg.path); ref=dict(session_id=bytes(msg.session_id.uuid).hex(),segment_revision=int(msg.segment_revision),traversability_revision=int(msg.traversability_revision),state=int(msg.state),**value)
        key=(ref['session_id'],ref['segment_revision'],ref['traversability_revision'])
        if ref['state']==0 or key not in references: references[key]=ref
        state['latest_path_reference']=ref
        display.observe_reference(ref,int(getattr(info,'source_timestamp',0)))
        raw('path_reference',msg); match()
    def on_diagnostics(msg):
        raw('diagnostics',msg)
        for status in msg.status:
            values={v.key:v.value for v in status.values}
            if all(k in values for k in ('session_id','segment_revision','fine_traversability_revision','cycle_result','reason_code')):
                key=(values['session_id'],int(values['segment_revision']),int(values['fine_traversability_revision']))
                diagnostics[key]=dict(values,source_stamp_ns=_stamp_ns(msg.header.stamp))
                display.observe_diagnostics(diagnostics[key])
        match()
    subscriptions=[node.create_subscription(Odometry,'/Car/T3/localization/odometry',on_odom,best),node.create_subscription(TFMessage,'/tf',on_tf,best),node.create_subscription(Clock,'/clock',on_clock,best),
        node.create_subscription(RosPath,'/Car/T4/planning/global_route',lambda m,info:on_path('global_route',m,info),route_qos),node.create_subscription(RosPath,'/Car/T4/planning/local_path',lambda m:on_path('local_path',m),latched),
        node.create_subscription(PathReference,'/Car/T4/planning/path_reference',on_reference,latched),node.create_subscription(DiagnosticArray,'/Car/T4/planning/diagnostics',on_diagnostics,10)]
    def write_overlay():
        nonlocal sequence
        sequence+=1; snapshot=display.snapshot(); snapshot['paths'].update(paths)
        state['long_route_evidence']=long_route_evidence(display,scene)
        payload=dict(schema=SCHEMA,run_id=run_id,sequence=sequence,frame='map',source='live_ros',clock_ns=state['clock_ns'],
            display_map=dict(path=scene['map_npz'],sha256=scene['map_sha256'],source_stage_sha256=scene['source_sha256'],region='A',
                usage='display-only projection of original planner XY; never changes raw ROS geometry; unknown/outside split'),**snapshot)
        validate_payload(payload); tmp=overlay.with_suffix(overlay.suffix+'.tmp'); tmp.write_text(json.dumps(payload,allow_nan=False),encoding='utf-8'); tmp.replace(overlay)
        state['display_session']=dict(goal_state=snapshot['goal_state'],action_session_id=snapshot['action_session_id'],history_paths=list(snapshot['history_paths']))
    record('start',run_id=run_id,scene=scene,argv=vars(args)); save()
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02); now=time.monotonic()
            ns=state['clock_ns']
            if ns is not None and state['map_publish_count']<2 and now-last_map>=1:
                msg=GridMap(); msg.header.frame_id='odom'; stamp_message(msg.header.stamp,ns)
                configure_grid_info(msg.info,bounds,resolution)
                msg.layers=['elevation']; msg.basic_layers=['elevation']; layer=Float32MultiArray(); layer.data=data
                layer.layout.dim=[MultiArrayDimension(label=label,size=size,stride=stride) for label,size,stride in dims]
                msg.data=[layer]; msg.outer_start_index=0; msg.inner_start_index=0
                map_pub.publish(msg); state['map_publish_count']+=1; last_map=now
                record('known_map_publish',stamp_ns=ns,frame='odom',resolution=resolution,source=scene['source_sha256'])
            healthy=(ns is not None and state['map_publish_count']>=2 and len(trace.points)>=2 and all(now-latest.get(k,-math.inf)<=args.freshness_wall for k in ('odom_wall','tf_wall'))
                and 0<=ns-latest.get('odom_stamp',-10**18)<=500000000
                and math.dist(trace.points[-1]['xyz'][:2],scene['start_xy_m'])<=.5)
            if args.send_goal and not state['goal_sent'] and healthy and action.server_is_ready():
                goal=NavigateToPose.Goal(); configure_goal(goal,scene['goal_xy_m'],scene['goal_yaw_rad'])
                state['goal_sent']=True; record('goal_request',target_xy_m=scene['goal_xy_m'],target_yaw_rad=scene['goal_yaw_rad'],health=dict(latest),map_count=state['map_publish_count'])
                session_id=uuid.uuid4().hex; display.begin_goal(session_id,time.time_ns()); trace.reset(); paths.clear()
                state['action_session_id']=session_id
                record('goal_display_epoch',session_id=session_id,request_source_time_ns=display.request_source_time_ns)
                send_future=action.send_goal_async(goal,goal_uuid=UUID(uuid=list(bytes.fromhex(session_id))),feedback_callback=lambda m:raw('action_feedback',m.feedback))
            if send_future is not None and send_future.done() and goal_handle is None:
                goal_handle=send_future.result(); state['goal_accepted']=bool(goal_handle.accepted); state['action_session_id']=bytes(goal_handle.goal_id.uuid).hex()
                record('goal_response',accepted=state['goal_accepted'],session_id=state['action_session_id']); match()
                if not goal_handle.accepted: state['stop_reason']='GOAL_REJECTED'; break
                display.accept_goal(state['action_session_id'])
                result_future=goal_handle.get_result_async()
            if result_future is not None and result_future.done():
                result=result_future.result(); raw('action_result',result.result)
                state['action_result']=dict(status=int(result.status),message=message_to_ordereddict(result.result))
                state['goal_reached']=result.result.outcome==0 and result.result.reason_code=='GOAL_REACHED'
                # 留观测尾窗由 root 控制；Action 到达与严格停车分别验收。
                display.finish('completed' if state['goal_reached'] else 'failed')
                state['stop_reason']='ACTION_RESULT'; break
            if now-last_overlay>=.2:
                if trace.points:
                    path=RosPath(); path.header.frame_id='map'; stamp_message(path.header.stamp,trace.highwater)
                    for item in trace.points:
                        pose=PoseStamped(); pose.header.frame_id='map'; stamp_message(pose.header.stamp,item['stamp_ns'])
                        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=(float(value) for value in item['xyz']); pose.pose.orientation.w=1.0; path.poses.append(pose)
                    trace_pub.publish(path)
                write_overlay()
                last_overlay=now; save()
            sim_elapsed=0 if ns is None else (ns-state['first_clock_ns'])/1e9
            if now-wall_start>=args.wall_timeout or sim_elapsed>=args.sim_timeout:
                state['stop_reason']='WALL_TIMEOUT' if now-wall_start>=args.wall_timeout else 'SIM_TIMEOUT'; break
    except BaseException as exc:
        state['stop_reason']='ERROR'; state['error']=repr(exc); record('error',detail=repr(exc)); raise
    finally:
        try:
            if goal_handle is None and send_future is not None:
                deadline=time.monotonic()+5
                while not send_future.done() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
                if send_future.done():
                    goal_handle=send_future.result()
                    state['goal_accepted']=bool(goal_handle.accepted)
                    if goal_handle.accepted: result_future=goal_handle.get_result_async()
                else: state['pending_goal_response']='UNRESOLVED; root must inspect action server'
            if goal_handle is not None and goal_handle.accepted and result_future is not None and not result_future.done():
                cancel_future=goal_handle.cancel_goal_async(); deadline=time.monotonic()+5
                while not cancel_future.done() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
                state['cancel_response_received']=cancel_future.done()
                if cancel_future.done(): raw('cancel_response',cancel_future.result())
                while not result_future.done() and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.05)
                if result_future.done():
                    result=result_future.result(); raw('action_result_after_cancel',result.result)
                    state['action_result']=dict(status=int(result.status),message=message_to_ordereddict(result.result))
        except BaseException as exc:
            state['cancel_error']=repr(exc)
            record('cancel_error',detail=repr(exc))
        if not display.terminal:
            display.finish('timeout' if state.get('stop_reason') in ('WALL_TIMEOUT','SIM_TIMEOUT') else 'cancelled' if state.get('stop_reason')!='ERROR' else 'failed')
        try: write_overlay()
        except (ValueError,OSError) as exc:
            state['final_overlay_error']=repr(exc); record('final_overlay_error',detail=repr(exc))
        state['wall_elapsed_s']=time.monotonic()-wall_start
        state['sim_elapsed_s']=0 if state['clock_ns'] is None else (state['clock_ns']-state['first_clock_ns'])/1e9
        state['strict_stop']='NOT_RUN; root must inspect actual odom/physics/sole-controller command tail'
        save(); events.close(); node.destroy_node(); rclpy.shutdown()
    print(json.dumps(state,ensure_ascii=False,allow_nan=False))
    return 0 if (not args.send_goal or (state['planning_evidence']['has_reference'] and state.get('long_route_evidence',{}).get('satisfied',False))) else 2

if __name__=='__main__': raise SystemExit(main())
