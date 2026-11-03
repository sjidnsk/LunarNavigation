"""216/P4 有界 TCP 导航检查；--preflight-only 不获取车辆控制权。"""
import argparse
from p3_input_timing import record_sample,runtime_input_health,timing_statistics
from p4_tcp_runtime import (CONTROL_DOMAIN, TcpCommandRelay, load_tcp_platform_profile,
                            profile_command_limits, launch_settings, runtime_environment, runtime_source)
import os,json,time,math,signal,subprocess,collections
from pathlib import Path
import yaml
from p4_topics import load_topic_contract
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
from grid_map_msgs.msg import GridMap
from lunar_planning_msgs.msg import TrackingStatus
from lunar_planning_msgs.action import NavigateToPose
from lunar_car_ctrl.msg import VehicleTelemetry
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--artifact',type=Path,required=True)
parser.add_argument('--preflight-only',action='store_true')
options=parser.parse_args()
ROOT=Path('/home/yanfa/P4');OUT=options.artifact.resolve();PREFIX='/p4_tcp_navigation_check'
assert OUT.is_relative_to(ROOT/'run'),'产物仅允许P4/run'
OUT.mkdir(exist_ok=False)
class PreflightOnlyComplete(Exception):pass
samples={}

start=time.monotonic();result={'scope':'one real UE TCP attempt using P3 map and odometry','allow_unknown':True,'reached':False};events=(OUT/'events.jsonl').open('w',buffering=1)
def event(kind,**kw):events.write(json.dumps(dict(elapsed=time.monotonic()-start,kind=kind,**kw))+'\n')
# 本轮独立命名空间；只管理自身launch和租约。
SOURCE=runtime_source(ROOT)
profile=load_tcp_platform_profile(SOURCE)
limits=profile_command_limits(profile)
result['speed_limits']=dict(v=limits[0],reverse=limits[1],w=limits[2])
config=OUT/'test-wheel.yaml';config.write_text(yaml.safe_dump(profile,sort_keys=False))
feedback_budget=yaml.safe_load((SOURCE/'config/tcp_p3_feedback.yaml').read_text())
contract=load_topic_contract(SOURCE);vehicle=contract['vehicle']
args=launch_settings(SOURCE,ROOT/'.cache/nav1cm',config,feedback_budget,
                     prefix=PREFIX,contract=contract)
result['feedback_budget']=feedback_budget
env=runtime_environment(ROOT,OUT)
launch_source=(Path(get_package_share_directory('lunar_pure_exploration_ros'))/'launch/nav1cm_exploration.launch.py').read_text(encoding='utf-8')
needle='return Node(package=package, executable=executable, output="screen", parameters=[{**params,**clock}])'
assert launch_source.count(needle)==1
launch_source=launch_source.replace(needle,'return Node(package=package, executable=executable, namespace="/p4_tcp_navigation_check", output="screen", parameters=[{**params,**clock}])')
private_launch=OUT/'isolated.launch.py';private_launch.write_text(launch_source)
command=['ros2','launch',str(private_launch)]+[k+':='+str(v) for k,v in args.items()]
contexts=[];nodes=[];executors=[];subs=[];process=None;handle=None;terminal=None;nonzero=0;latest={};counts=collections.Counter();reasons=collections.Counter();positions=[];feedback=[];reference_accepted=False;initial_pose=None;goal=None
for domain in (CONTROL_DOMAIN,):
 c=rclpy.Context();rclpy.init(context=c,domain_id=domain);contexts.append(c);n=rclpy.create_node('p4_tcp_closedloop_'+str(domain),context=c);nodes.append(n);e=SingleThreadedExecutor(context=c);e.add_node(n);executors.append(e)
control_node=nodes[0];vel=control_node.create_publisher(Twist,vehicle['command_topic'],10);lease=control_node.create_publisher(Bool,vehicle['lease_topic'],10);mode=control_node.create_publisher(String,vehicle['mode_topic'],10)
relay=TcpCommandRelay(profile,publish_velocity=vel.publish,
 publish_mode=lambda value:mode.publish(String(data=value)),
 publish_lease=lambda value:lease.publish(Bool(data=value)),make_zero=Twist)
q=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
def spin(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:
  for e in executors:e.spin_once(timeout_sec=.005)
def callback(kind,msg):
 global reference_accepted
 counts[kind]+=1;latest[kind]=(time.monotonic(),msg)
 if kind in ('odom','map'):
  stamp=msg.header.stamp;source=stamp.sec*10**9+stamp.nanosec
  samples[kind]=record_sample(samples.get(kind),source,control_node.get_clock().now().nanoseconds,time.monotonic())
  event('input_timing',input=kind,source_ns=source,received_ros_ns=samples[kind]['received_ros_ns'],received_mono_s=samples[kind]['received_mono_s'])
 if kind=='odom':
  p=msg.pose.pose.position;positions.append((p.x,p.y,p.z));event(kind,position=[p.x,p.y,p.z],source_age=time.time()-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9))
 elif kind=='diag':
  try:d=json.loads(msg.data)
  except Exception:d={'raw':msg.data}
  reasons[str(d.get('reason',d.get('phase',d.get('event'))))]+=1;event(kind,data=d)
 elif kind=='tracking':
  reference_accepted=bool(handle is not None and handle.accepted and bytes(msg.session_id.uuid)==bytes(handle.goal_id.uuid) and msg.reference_accepted);event(kind,state=msg.state,reason=msg.reason,reference_accepted=msg.reference_accepted,cross_track=msg.cross_track_m,progress=msg.progress_m)
 elif kind in ('local_path','global_path'):event(kind,poses=len(msg.poses));counts[kind+'_nonempty']+=int(bool(msg.poses))
 elif kind=='telemetry':
  relay.update_telemetry(msg);event(kind,vx=msg.vx,vy=msg.vy,wz=msg.wz)
for topic,cls,kind in [(args['odometry_topic'],Odometry,'odom'),(args['map_topic'],GridMap,'map'),(args['exploration_map_topic'],OccupancyGrid,'exploration_map'),(args['diagnostics_topic'],String,'diag'),(args['tracking_status_topic'],TrackingStatus,'tracking'),(args['local_path_topic'],RosPath,'local_path'),(args['global_route_topic'],RosPath,'global_path')]:
 subs.append(control_node.create_subscription(cls,topic,lambda msg,k=kind:callback(k,msg),map_qos if kind=='map' else q))
subs.append(control_node.create_subscription(VehicleTelemetry,vehicle['telemetry_topic'],lambda msg:callback('telemetry',msg),q))
subs.append(control_node.create_subscription(Bool,vehicle['connected_topic'],relay.update_connected,map_qos))
def command_input(msg):
 global nonzero
 counts['controller_commands']+=1;event('controller_command',v=msg.linear.x,w=msg.angular.z)
 nonzero+=int(abs(msg.linear.x)+abs(msg.angular.z)>=1e-7)
 was_armed=relay.armed
 forwarded=relay.receive(msg)
 if forwarded and not was_armed:event('auto_mode_entered')
subs.append(control_node.create_subscription(Twist,args['command_topic'],command_input,q))
client=ActionClient(control_node,NavigateToPose,args['action_name'])
def params(name,keys):
 c=control_node.create_client(GetParameters,PREFIX+'/'+name+'/get_parameters');assert c.wait_for_service(timeout_sec=10.),name;r=GetParameters.Request();r.names=keys;f=c.call_async(r);end=time.monotonic()+10
 while not f.done() and time.monotonic()<end:spin(.05)
 assert f.done(),name;values={}
 for k,v in zip(keys,f.result().values):values[k]=v.bool_value if v.type==1 else v.integer_value if v.type==2 else v.double_value if v.type==3 else v.string_value
 return values
try:
 with (OUT/'launch.log').open('w') as log:process=subprocess.Popen(command,cwd=OUT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (OUT/'pids.json').write_text(json.dumps({'supervisor':os.getpid(),'launch':process.pid}));spin(5)
 result['planner_params']=params('lunar_nav1cm_planner',['allow_unknown','linear_speed_mps','angular_speed_rps','execution_contract_enabled','control_backend','enable_controller','command_topic','odometry_topic','odometry_timeout_s'])
 assert result['planner_params']['allow_unknown'] is True
 assert result['planner_params']['control_backend']=='mpcc_cmd_vel' and result['planner_params']['enable_controller']
 assert not result['planner_params']['execution_contract_enabled']
 assert result['planner_params']['command_topic']==args['command_topic']
 assert result['planner_params']['odometry_topic']==args['odometry_topic']
 assert abs(result['planner_params']['linear_speed_mps']-min(limits[:2]))<1e-9 and abs(result['planner_params']['angular_speed_rps']-limits[2])<1e-9
 end=time.monotonic()+15
 while not all(k in latest for k in ('odom','exploration_map','telemetry')) and time.monotonic()<end:spin(.1)
 assert all(k in latest for k in ('odom','exploration_map','telemetry')),'missing inputs'
 # 使用实际参数回读，定位源龄/接收龄、direct_p3地图接收龄与运行时一致。
 measured_budget=dict(odometry_timeout_s=result['planner_params']['odometry_timeout_s'],
                      p3_map_timeout_s=feedback_budget['p3_map_timeout_s'])
 result['planner_pose_budget']=params('lunar_nav1cm_planner',['odometry_timeout_s'])
 assert result['planner_pose_budget']['odometry_timeout_s']==measured_budget['odometry_timeout_s']
 result['preflight_time_contract']=dict(measured_budget,map_source_age_gate=False,extra_recovery_wait_s=0.,odometry_continuity_check=True,odometry_source_age_gate=False)
 end=time.monotonic()+45;last_report=0.
 while time.monotonic()<end:
  spin(.1);ros_now=control_node.get_clock().now().nanoseconds;mono_now=time.monotonic()
  report={k:runtime_input_health(k,samples.get(k),ros_now,mono_now,**measured_budget) for k in ('odom','map')}
  result['final_preflight_health']=report
  if mono_now-last_report>=1.:
   event('preflight_health',inputs=report);last_report=mono_now
  if all(v['ready'] for v in report.values()):break
 else:raise RuntimeError('P3_INPUTS_NOT_READY_WITHIN_RUNTIME_BUDGET')
 result['preflight_current_inputs']=True
 if options.preflight_only:raise PreflightOnlyComplete()
 tel=latest.get('telemetry',(None,None))[1]
 assert relay.measured_stopped(),'telemetry not fresh and stopped'
 od=latest['odom'][1];p=od.pose.pose.position;initial_pose=(p.x,p.y,p.z);positions.clear();positions.append(initial_pose)
 q0=od.pose.pose.orientation;yaw=math.atan2(2*(q0.w*q0.z+q0.x*q0.y),1-2*(q0.y*q0.y+q0.z*q0.z))
 goal=(p.x+.5*math.cos(yaw),p.y+.5*math.sin(yaw));result['goal']=goal;result['initial_pose']=initial_pose;result['goal_policy']='0.5m forward in explicitly authorized UNKNOWN; known BLOCKED still vetoed by planner/controller'
 gui_publishers=[i.node_name for i in control_node.get_publishers_info_by_topic(vehicle['command_topic'])];assert all(x in ('lunar_car_gui_node','p4_tcp_closedloop_10') for x in gui_publishers),gui_publishers;result['vehicle_command_publishers']=gui_publishers
 relay.acquire();spin(.3)
 assert client.wait_for_server(timeout_sec=5.);request=NavigateToPose.Goal();request.target_x_m=goal[0];request.target_y_m=goal[1];request.has_target_yaw=False
 f=client.send_goal_async(request,feedback_callback=lambda m:(feedback.append(m.feedback.reason_code),event('navigation_feedback',reason=m.feedback.reason_code)));end=time.monotonic()+8
 while not f.done() and time.monotonic()<end:relay.heartbeat();spin(.1)
 assert f.done(),'goal acceptance unresolved';handle=f.result();result['goal_accepted']=handle.accepted;assert handle.accepted;terminal=handle.get_result_async();attempt=time.monotonic();last_progress=0
 while time.monotonic()-attempt<75:
  was_armed=relay.armed
  ready=relay.heartbeat()
  if was_armed and not ready:event('relay_stopped_or_disconnected')
  spin(.1)
  if process.poll() is not None:raise RuntimeError('navigation launch exited')
  if positions and math.dist(initial_pose,positions[-1])>.8:raise RuntimeError('test displacement exceeded 0.8m bound')
  if terminal.done():
   m=terminal.result();result['action_result']={'status':m.status,'outcome':m.result.outcome,'reason_code':m.result.reason_code};result['reached']=m.result.outcome==NavigateToPose.Result.GOAL_REACHED;break
  elapsed=time.monotonic()-attempt
  if elapsed-last_progress>5:print(json.dumps({'event':'test_progress','elapsed':round(elapsed,1),'nonzero_commands':nonzero,'forwarded_nonzero':relay.forwarded_nonzero,'paths':counts['local_path_nonempty'],'diagnostics':dict(reasons)}),flush=True);last_progress=elapsed
  if elapsed>25 and relay.forwarded_nonzero==0:result['test_stop_reason']='NO_START_WITHIN_25_SECONDS';break
 result['test_elapsed_s']=time.monotonic()-attempt
except PreflightOnlyComplete:result['navigation']='NOT_RUN_PREFLIGHT_ONLY'
except BaseException as exc:result['error']=repr(exc)
finally:
 relay.begin_stop()
 if handle is not None and handle.accepted and terminal is not None and not terminal.done():
  cancel=handle.cancel_goal_async();end=time.monotonic()+6
  while (not cancel.done() or not terminal.done()) and time.monotonic()<end:relay.heartbeat();spin(.05)
  result['cancel_response_received']=cancel.done();result['terminal_confirmed']=terminal.done()
  if terminal.done():m=terminal.result();result['cleanup_action_result']={'status':m.status,'outcome':m.result.outcome,'reason_code':m.result.reason_code}
 # 收尾保持原租约直到零速/park发送完成；平台组件禁止新回调重新auto。
 for i in range(15):relay.heartbeat();spin(.1)
 result['measured_stopped']=relay.measured_stopped()
 result['control_authority_acquired']=relay.leased
 relay.release()
 spin(.2)
 if process is not None and process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:result['launch_shutdown_timeout']=True
 result['launch_returncode']=None if process is None else process.poll()
 result['input_timing_statistics']={k:timing_statistics(samples.get(k),control_node.get_clock().now().nanoseconds,time.monotonic()) for k in ('odom','map')}
 counts['forwarded_zero']=relay.forwarded_zero
 result.update(counts=dict(counts),diagnostic_reasons=dict(reasons),controller_nonzero_commands=nonzero,forwarded_nonzero_commands=relay.forwarded_nonzero,auto_mode_entered=relay.forwarded_nonzero>0,reference_accepted=reference_accepted)
 if positions:result['final_p3_pose']=positions[-1]
 if initial_pose is not None and positions:result['p3_displacement_max_m']=max(math.dist(initial_pose,p) for p in positions)
 if goal and positions:result['final_p3_goal_error_m']=math.hypot(positions[-1][0]-goal[0],positions[-1][1]-goal[1])
 result['stage_results']={'start':('NOT_RUN' if handle is None else 'PASS' if relay.forwarded_nonzero and result.get('p3_displacement_max_m',0)>.02 else 'FAIL'),'tracking':'PASS' if counts['local_path_nonempty'] and relay.forwarded_nonzero else 'NOT_RUN','arrival':'PASS' if result['reached'] else 'NOT_REACHED','stop':'PASS' if result['measured_stopped'] else 'UNCONFIRMED'}
 (OUT/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True);events.close()
 for e in executors:e.shutdown()
 for n in nodes:n.destroy_node()
 for c in contexts:rclpy.shutdown(context=c)
