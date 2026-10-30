"""216/P4 低速有界 TCP 导航检查；--preflight-only 不获取车辆控制权。"""
import argparse
from p3_input_timing import record_sample,runtime_input_health,timing_statistics
from p4_tcp_runtime import CONTROL_DOMAIN
import os,json,time,math,signal,subprocess,importlib.util,collections
from pathlib import Path
import yaml
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

start=time.monotonic();result={'scope':'one real UE TCP attempt using P3 map and odometry','speed_limits':{'v':.05,'w':.1},'allow_unknown':True,'reached':False};events=(OUT/'events.jsonl').open('w',buffering=1)
def event(kind,**kw):events.write(json.dumps(dict(elapsed=time.monotonic()-start,kind=kind,**kw))+'\n')
# 本轮独立命名空间；只管理自身launch和租约。
profile=yaml.safe_load((ROOT/'source/config/wheel.yaml').read_text());profile['capability'].update(maximum_forward_speed_mps=.05,maximum_reverse_speed_mps=.05,maximum_spin_rate_radps=.1);config=OUT/'test-wheel.yaml';config.write_text(yaml.safe_dump(profile,sort_keys=False))
spec=importlib.util.spec_from_file_location('p3_original_launcher',ROOT/'source/scripts/simulation/debug_p3_planning.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
args=m.launch_arguments(ROOT/'source',ROOT/'.cache/nav1cm');args={k:(v.replace('/p4_p3_debug',PREFIX) if isinstance(v,str) else v) for k,v in args.items()};args.update(enable_controller='true',allow_unknown='true',exploration_strategy='none',platform_config=str(config),command_topic=PREFIX+'/cmd_vel')
feedback_budget=yaml.safe_load((ROOT/'source/config/tcp_p3_feedback.yaml').read_text());args.update({k:str(v) for k,v in feedback_budget.items()});result['feedback_budget']=feedback_budget
env=dict(os.environ,ROS_DOMAIN_ID=str(CONTROL_DOMAIN),ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_HOME=str(ROOT/'run/ros-home'),ROS_LOG_DIR=str(OUT/'ros-log'),TMPDIR=str(ROOT/'tmp'),XDG_CACHE_HOME=str(ROOT/'.cache'),PYTHONDONTWRITEBYTECODE='1',ACADOS_SOURCE_DIR=str(ROOT/'deps/acados-v040'),NAV1CM_BUILD_DIR=str(ROOT/'.cache/nav1cm'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
env['PATH']=str(ROOT/'.venv/bin')+':'+env['PATH'];env['LD_LIBRARY_PATH']=str(ROOT/'deps/acados-v040/lib')+':'+env.get('LD_LIBRARY_PATH','')
launch_source=(ROOT/'ros2-humble/install/lunar_pure_exploration_ros/share/lunar_pure_exploration_ros/launch/nav1cm_exploration.launch.py').read_text()
needle='return Node(package=package, executable=executable, output="screen", parameters=[{**params,**clock}])'
assert launch_source.count(needle)==1
launch_source=launch_source.replace(needle,'return Node(package=package, executable=executable, namespace="/p4_tcp_navigation_check", output="screen", parameters=[{**params,**clock}])')
private_launch=OUT/'isolated.launch.py';private_launch.write_text(launch_source)
command=['ros2','launch',str(private_launch)]+[k+':='+str(v) for k,v in args.items()]
contexts=[];nodes=[];executors=[];subs=[];process=None;handle=None;terminal=None;armed=False;leased=False;forwarded_nonzero=0;nonzero=0;latest={};counts=collections.Counter();reasons=collections.Counter();positions=[];feedback=[];telemetry_positions=[];reference_accepted=False;initial_pose=None;goal=None;last_command=None
for domain in (CONTROL_DOMAIN,):
 c=rclpy.Context();rclpy.init(context=c,domain_id=domain);contexts.append(c);n=rclpy.create_node('p4_tcp_closedloop_'+str(domain),context=c);nodes.append(n);e=SingleThreadedExecutor(context=c);e.add_node(n);executors.append(e)
control_node=nodes[0];vel=control_node.create_publisher(Twist,'/car/cmd_vel',10);lease=control_node.create_publisher(Bool,'/car/external_control_lease',10);mode=control_node.create_publisher(String,'/car/set_mode',10)
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
 elif kind=='telemetry':telemetry_positions.append((msg.x,msg.y,msg.z));event(kind,vx=msg.vx,vy=msg.vy,wz=msg.wz,wheels=[msg.wheel_speed_fl,msg.wheel_speed_fr,msg.wheel_speed_rl,msg.wheel_speed_rr])
for topic,cls,kind in [('/Car/T3/localization/odometry',Odometry,'odom'),('/Car/T3/mapping/grid_map',GridMap,'map'),(PREFIX+'/exploration_map',OccupancyGrid,'exploration_map'),(PREFIX+'/control/diagnostics',String,'diag'),(PREFIX+'/tracking_status',TrackingStatus,'tracking'),(PREFIX+'/local_path',RosPath,'local_path'),(PREFIX+'/global_route',RosPath,'global_path')]:
 subs.append(control_node.create_subscription(cls,topic,lambda msg,k=kind:callback(k,msg),map_qos if kind=='map' else q))
subs.append(control_node.create_subscription(VehicleTelemetry,'/car/telemetry',lambda msg:callback('telemetry',msg),q))
def command_input(msg):
 global armed,nonzero,forwarded_nonzero,last_command
 last_command=time.monotonic();counts['controller_commands']+=1;v=msg.linear.x;w=msg.angular.z;event('controller_command',v=v,w=w)
 if abs(v)+abs(w)<1e-7:
  if leased:vel.publish(msg);counts['forwarded_zero']+=1
  return
 nonzero+=1
 if not all(math.isfinite(x) for x in (v,w)) or abs(v)>.050001 or abs(w)>.100001:raise RuntimeError('controller exceeded low-speed bound')
 if handle is None or not handle.accepted or not reference_accepted:return
 if 'telemetry' not in latest or time.monotonic()-latest['telemetry'][0]>.5:raise RuntimeError('telemetry stale before command')
 if 'odom' not in latest:return
 od=latest['odom'][1];age=time.time()-od.header.stamp.sec-od.header.stamp.nanosec*1e-9
 if not runtime_input_health('odom',samples.get('odom'),control_node.get_clock().now().nanoseconds,time.monotonic(),odometry_timeout_s=float(feedback_budget['odometry_timeout_s']),p3_map_timeout_s=float(feedback_budget['p3_map_timeout_s']))['ready']:raise RuntimeError('odometry stream stopped advancing before motion')
 if not armed:mode.publish(String(data='auto'));armed=True;event('auto_mode_entered')
 vel.publish(msg);forwarded_nonzero+=1
subs.append(control_node.create_subscription(Twist,PREFIX+'/cmd_vel',command_input,q))
client=ActionClient(control_node,NavigateToPose,PREFIX+'/navigate_to_pose')
def params(name,keys):
 c=control_node.create_client(GetParameters,PREFIX+'/'+name+'/get_parameters');assert c.wait_for_service(timeout_sec=10.),name;r=GetParameters.Request();r.names=keys;f=c.call_async(r);end=time.monotonic()+10
 while not f.done() and time.monotonic()<end:spin(.05)
 assert f.done(),name;values={}
 for k,v in zip(keys,f.result().values):values[k]=v.bool_value if v.type==1 else v.integer_value if v.type==2 else v.double_value if v.type==3 else v.string_value
 return values
try:
 with (OUT/'launch.log').open('w') as log:process=subprocess.Popen(command,cwd=OUT,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 (OUT/'pids.json').write_text(json.dumps({'supervisor':os.getpid(),'launch':process.pid}));spin(5)
 result['planner_params']=params('lunar_nav1cm_planner',['allow_unknown','linear_speed_mps','angular_speed_rps','execution_contract_enabled'])
 result['controller_params']=params('lunar_pure_wheeled_controller',['allow_unknown','mppi_compute_envelope_s','odometry_timeout_s','p3_map_timeout_s','mppi_time_steps','execution_contract_enabled','platform_config'])
 assert result['planner_params']['allow_unknown'] is True and result['controller_params']['allow_unknown'] is True
 assert result['planner_params']['execution_contract_enabled'] and result['controller_params']['execution_contract_enabled']
 assert abs(result['planner_params']['linear_speed_mps']-.05)<1e-9 and abs(result['planner_params']['angular_speed_rps']-.1)<1e-9
 end=time.monotonic()+15
 while not all(k in latest for k in ('odom','exploration_map','telemetry')) and time.monotonic()<end:spin(.1)
 assert all(k in latest for k in ('odom','exploration_map','telemetry')),'missing inputs'
 # 使用实际参数回读，定位源龄/接收龄、direct_p3地图接收龄与运行时一致。
 measured_budget=dict(odometry_timeout_s=result['controller_params']['odometry_timeout_s'],
                      p3_map_timeout_s=result['controller_params']['p3_map_timeout_s'])
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
 assert tel is not None and time.monotonic()-latest['telemetry'][0]<.5 and max(abs(tel.vx),abs(tel.vy))<.01,'telemetry not fresh and stopped'
 od=latest['odom'][1];p=od.pose.pose.position;initial_pose=(p.x,p.y,p.z)
 q0=od.pose.pose.orientation;yaw=math.atan2(2*(q0.w*q0.z+q0.x*q0.y),1-2*(q0.y*q0.y+q0.z*q0.z))
 goal=(p.x+.5*math.cos(yaw),p.y+.5*math.sin(yaw));result['goal']=goal;result['initial_pose']=initial_pose;result['goal_policy']='0.5m forward in explicitly authorized UNKNOWN; known BLOCKED still vetoed by planner/controller'
 gui_publishers=[i.node_name for i in control_node.get_publishers_info_by_topic('/car/cmd_vel')];assert all(x in ('lunar_car_gui_node','p4_tcp_closedloop_10') for x in gui_publishers),gui_publishers;result['vehicle_command_publishers']=gui_publishers
 lease.publish(Bool(data=True));leased=True;mode.publish(String(data='park'));vel.publish(Twist());spin(.3)
 assert client.wait_for_server(timeout_sec=5.);request=NavigateToPose.Goal();request.target_x_m=goal[0];request.target_y_m=goal[1];request.has_target_yaw=False
 f=client.send_goal_async(request,feedback_callback=lambda m:(feedback.append(m.feedback.reason_code),event('navigation_feedback',reason=m.feedback.reason_code)));end=time.monotonic()+8
 while not f.done() and time.monotonic()<end:lease.publish(Bool(data=True));spin(.1)
 assert f.done(),'goal acceptance unresolved';handle=f.result();result['goal_accepted']=handle.accepted;assert handle.accepted;terminal=handle.get_result_async();attempt=time.monotonic();last_progress=0
 while time.monotonic()-attempt<75:
  lease.publish(Bool(data=True))
  if not armed:vel.publish(Twist())
  elif last_command is None or time.monotonic()-last_command>.5:
   vel.publish(Twist());mode.publish(String(data='park'));armed=False;event('relay_command_gap_park')
  spin(.1)
  if process.poll() is not None:raise RuntimeError('navigation launch exited')
  if telemetry_positions and math.dist(telemetry_positions[0],telemetry_positions[-1])>.8:raise RuntimeError('test displacement exceeded 0.8m bound')
  if terminal.done():
   m=terminal.result();result['action_result']={'status':m.status,'outcome':m.result.outcome,'reason_code':m.result.reason_code};result['reached']=m.result.outcome==NavigateToPose.Result.GOAL_REACHED;break
  elapsed=time.monotonic()-attempt
  if elapsed-last_progress>5:print(json.dumps({'event':'test_progress','elapsed':round(elapsed,1),'nonzero_commands':nonzero,'forwarded_nonzero':forwarded_nonzero,'paths':counts['local_path_nonempty'],'diagnostics':dict(reasons)}),flush=True);last_progress=elapsed
  if elapsed>25 and forwarded_nonzero==0:result['test_stop_reason']='NO_START_WITHIN_25_SECONDS';break
 result['test_elapsed_s']=time.monotonic()-attempt
except PreflightOnlyComplete:result['navigation']='NOT_RUN_PREFLIGHT_ONLY'
except BaseException as exc:result['error']=repr(exc)
finally:
 if handle is not None and handle.accepted and terminal is not None and not terminal.done():
  cancel=handle.cancel_goal_async();end=time.monotonic()+6
  while (not cancel.done() or not terminal.done()) and time.monotonic()<end:spin(.05)
  result['cancel_response_received']=cancel.done();result['terminal_confirmed']=terminal.done()
  if terminal.done():m=terminal.result();result['cleanup_action_result']={'status':m.status,'outcome':m.result.outcome,'reason_code':m.result.reason_code}
 # Explicit zero/park before releasing the GUI lease, then confirm from telemetry.
 for i in range(15):
  if leased:mode.publish(String(data='park'));vel.publish(Twist());lease.publish(Bool(data=True))
  spin(.1)
 tel=latest.get('telemetry',(None,None))[1]
 result['measured_stopped']=bool(tel is not None and time.monotonic()-latest['telemetry'][0]<.5 and max(abs(tel.vx),abs(tel.vy))<.01 and max(abs(getattr(tel,k)) for k in ('wheel_speed_fl','wheel_speed_fr','wheel_speed_rl','wheel_speed_rr'))<.05)
 if leased:lease.publish(Bool(data=False));mode.publish(String(data='park'))
 spin(.2)
 if process is not None and process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:result['launch_shutdown_timeout']=True
 result['launch_returncode']=None if process is None else process.poll()
 result['control_authority_acquired']=leased
 result['input_timing_statistics']={k:timing_statistics(samples.get(k),control_node.get_clock().now().nanoseconds,time.monotonic()) for k in ('odom','map')}
 result.update(counts=dict(counts),diagnostic_reasons=dict(reasons),controller_nonzero_commands=nonzero,forwarded_nonzero_commands=forwarded_nonzero,auto_mode_entered=armed,reference_accepted=reference_accepted)
 if positions:result['p3_displacement_max_m']=max(math.dist(positions[0],p) for p in positions);result['final_p3_pose']=positions[-1]
 if telemetry_positions:result['tcp_displacement_max_m']=max(math.dist(telemetry_positions[0],p) for p in telemetry_positions)
 if goal and positions:result['final_p3_goal_error_m']=math.hypot(positions[-1][0]-goal[0],positions[-1][1]-goal[1])
 result['stage_results']={'start':('NOT_RUN' if handle is None else 'PASS' if forwarded_nonzero and result.get('tcp_displacement_max_m',0)>.02 else 'FAIL'),'tracking':'PASS' if counts['local_path_nonempty'] and forwarded_nonzero else 'NOT_RUN','arrival':'PASS' if result['reached'] else 'NOT_REACHED','stop':'PASS' if result['measured_stopped'] else 'UNCONFIRMED'}
 (OUT/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True);events.close()
 for e in executors:e.shutdown()
 for n in nodes:n.destroy_node()
 for c in contexts:rclpy.shutdown(context=c)
