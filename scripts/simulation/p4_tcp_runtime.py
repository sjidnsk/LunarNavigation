"""P4控制域10入口：P3单向输入桥、导航启动及同域TCP租约适配。"""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

CONTROL_DOMAIN=10
P3_SOURCE_DOMAIN=57
PREFIX='/p4_p3_debug'
TRANSPORT_TIMEOUT_S=.5


def launch_settings(source,build_dir,profile_path,budgets,*,prefix=PREFIX,goal_topic="/goal_pose"):
    try:from .debug_p3_planning import launch_arguments
    except ImportError:from debug_p3_planning import launch_arguments
    args=launch_arguments(source,build_dir,profile=budgets,allow_unknown=True)
    args={key:(prefix+value[len(PREFIX):] if isinstance(value,str) and value.startswith(PREFIX+'/') else value)
          for key,value in args.items()}
    args.update(enable_controller='true',exploration_strategy='none',
                platform_config=str(profile_path),command_topic=prefix+'/cmd_vel',goal_topic=goal_topic,
                initial_travel_direction='forward')
    return args


def input_bridge(source,target,classes,qos):
    # 回调直接交付原消息，禁止重标源时间、反向转发或控制转发。
    handles=[]
    for topic,kind,key in [('/Car/T3/mapping/grid_map','map','map'),
                           ('/Car/T3/localization/odometry','odom','odom'),
                           ('/tf','tf','tf'),('/tf_static','tf','tf_static')]:
        publisher=target.create_publisher(classes[kind],topic,qos.get(key+'_output',qos[key]))
        subscription=source.create_subscription(classes[kind],topic,publisher.publish,qos[key])
        handles.extend((publisher,subscription))
    return handles


class NavigationObserver:
    """只记录源定位，不参与导航决策。"""
    def __init__(self, make_pose, make_path, publish_start, publish_goal, publish_path):
        self.make_pose, self.make_path = make_pose, make_path
        self.publish_start, self.publish_goal, self.publish_path = publish_start, publish_goal, publish_path
        self.latest = None
        self.stamps = {}
        self.active = False
        self.start = None
        self.poses = []

    def goal(self, msg):
        self.active = True
        self.start = deepcopy(self.latest)
        self.poses = [] if self.start is None else [deepcopy(self.start)]
        self.publish_goal(msg)
        # 空 frame 的消息清除上一次起点；不虚构当前位置。
        self.publish_start(self.start if self.start is not None else self.make_pose())
        self._path()

    def odometry(self, msg):
        h, p = msg.header, msg.pose.pose
        stamp = (h.stamp.sec, h.stamp.nanosec)
        values = (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
        if (not h.frame_id or stamp <= (0,0) or not all(math.isfinite(v) for v in values)
                or sum(v*v for v in values[3:]) < 1e-12): return
        if stamp <= self.stamps.get(h.frame_id,(0,0)): return
        self.stamps[h.frame_id] = stamp
        pose = self.make_pose(); pose.header = deepcopy(h); pose.pose = deepcopy(p)
        self.latest = pose
        if not self.active: return
        if self.start is None:
            self.start = deepcopy(pose); self.publish_start(self.start)
        if self.poses and self.poses[-1].header.frame_id != h.frame_id:
            self.poses = []
        self.poses.append(pose)
        self.poses = self.poses[-2000:]
        self._path()

    def _path(self):
        path = self.make_path()
        if self.poses: path.header = deepcopy(self.poses[-1].header)
        path.poses = deepcopy(self.poses)
        self.publish_path(path)


def attach_navigation_observer(source, target, classes, qos):
    pubs = [target.create_publisher(classes[kind], PREFIX+topic, qos['display']) for kind,topic in
            [('pose','/start_pose'),('pose','/goal_pose_display'),('path','/actual_path')]]
    observer = NavigationObserver(classes['pose'],classes['path'],*(p.publish for p in pubs))
    subs = [source.create_subscription(classes['odom'],'/Car/T3/localization/odometry',observer.odometry,qos['odom']),
            target.create_subscription(classes['pose'],'/goal_pose',observer.goal,qos['goal'])]
    return [observer,*pubs,*subs]


def command_ready(command_age,telemetry_age):
    # 平台只检查传输活性，不重新解释规划参考的接管状态。
    return 0<=command_age<=TRANSPORT_TIMEOUT_S and 0<=telemetry_age<=TRANSPORT_TIMEOUT_S


def profile_command_limits(profile):
    capability=profile['capability']
    values=tuple(float(capability[name]) for name in
        ('maximum_forward_speed_mps','maximum_reverse_speed_mps','maximum_spin_rate_radps'))
    if any(not math.isfinite(v) or v<=0 for v in values):raise ValueError('invalid command limits')
    return values


def low_speed_profile(profile,*,linear_mps=.1,angular_rps=.1950770225306774):
    if any(not math.isfinite(v) or v<=0 for v in (linear_mps,angular_rps)):
        raise ValueError('invalid low-speed profile')
    forward,reverse,angular=profile_command_limits(profile)
    result=deepcopy(profile)
    result['capability'].update(maximum_forward_speed_mps=min(forward,linear_mps),
        maximum_reverse_speed_mps=min(reverse,linear_mps),maximum_spin_rate_radps=min(angular,angular_rps))
    return result


def runtime_environment(root,out):
    root,out=Path(root),Path(out)
    env=dict(os.environ,ROS_DOMAIN_ID=str(CONTROL_DOMAIN),ROS_LOCALHOST_ONLY='0',
        RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_HOME=str(out/'ros-home'),ROS_LOG_DIR=str(out/'ros-log'),
        TMPDIR=str(root/'tmp'),XDG_CACHE_HOME=str(root/'.cache'),PYTHONDONTWRITEBYTECODE='1',
        ACADOS_SOURCE_DIR=str(root/'deps/acados-v040'),NAV1CM_BUILD_DIR=str(root/'.cache/nav1cm'),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    env['PATH']=str(root/'.venv/bin')+os.pathsep+env['PATH']
    env['LD_LIBRARY_PATH']=str(root/'deps/acados-v040/lib')+':'+env.get('LD_LIBRARY_PATH','')
    return env


class TcpCommandRelay:
    """两入口共用现有车辆转发状态；只负责协议、租约和真实传输活性。"""
    def __init__(self,profile,*,publish_velocity,publish_mode,publish_lease,make_zero,clock=time.monotonic):
        self.limits=profile_command_limits(profile)
        self._velocity,self._mode,self._lease=publish_velocity,publish_mode,publish_lease
        self._zero,self._clock=make_zero,clock
        self.leased=self.armed=self._stopping=False
        self._command_s=self._telemetry=None
        self.forwarded_nonzero=self.forwarded_zero=0

    def acquire(self):
        self.leased=True;self._stopping=False
        self.park();self._lease(True)

    def park(self):
        if self.leased:self._velocity(self._zero());self._mode('park')
        self.armed=False

    def begin_stop(self):
        self._stopping=True;self.park()

    def release(self):
        self.begin_stop()
        if self.leased:self._lease(False)
        self.leased=False

    def update_telemetry(self,message):
        self._telemetry=(self._clock(),message)

    def receive(self,message):
        # 收尾/未接管时不再处理运动输入，非法队列消息也不得打断释放与结果保存。
        if not self.leased:return False
        if self._stopping:self.park();return False
        v,w=message.linear.x,message.angular.z
        forward,reverse,angular=self.limits
        if not all(math.isfinite(x) for x in (v,w)) or not -reverse-1e-6<=v<=forward+1e-6 or abs(w)>angular+1e-6:
            self.park();raise ValueError('invalid_or_over_limit_controller_command')
        self._command_s=self._clock()
        if abs(v)+abs(w)<1e-7:
            self._velocity(message);self.forwarded_zero+=1;return False
        if self._telemetry is None or not command_ready(0.,self._clock()-self._telemetry[0]):
            self.park();return False
        if not self.armed:self._mode('auto');self.armed=True
        self._velocity(message);self.forwarded_nonzero+=1
        return True

    def heartbeat(self):
        if not self.leased:return False
        self._lease(True);now=self._clock()
        ready=(self._command_s is not None and self._telemetry is not None and
               command_ready(now-self._command_s,now-self._telemetry[0]))
        if self._stopping or not ready:self.park();return False
        return True

    def measured_stopped(self):
        if self._telemetry is None:return False
        received,tel=self._telemetry
        velocity=(tel.vx,tel.vy)
        wheels=tuple(getattr(tel,name) for name in ('wheel_speed_fl','wheel_speed_fr','wheel_speed_rl','wheel_speed_rr'))
        return (0<=self._clock()-received<TRANSPORT_TIMEOUT_S and
                all(math.isfinite(v) for v in (*velocity,*wheels)) and
                max(map(abs,velocity))<.01 and max(map(abs,wheels))<.05)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('/home/yanfa/P4'))
    parser.add_argument('--artifact',type=Path,required=True)
    parser.add_argument('--rviz-config',type=Path)
    parser.add_argument('--check-only',action='store_true',help='只接入输入，不启动导航、RViz、租约或车控')
    parser.add_argument('--duration',type=float,default=0.,help='运行上限，0表示等待停止信号')
    options=parser.parse_args();root=options.root.resolve();out=options.artifact.resolve()
    if root!=Path('/home/yanfa/P4') or not out.is_relative_to(root/'run'):
        parser.error('运行产物必须位于/home/yanfa/P4/run')
    out.mkdir(parents=True,exist_ok=False)
    import yaml
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist,PoseStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from grid_map_msgs.msg import GridMap
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool,String
    from lunar_car_ctrl.msg import VehicleTelemetry
    contexts=[];nodes=[];executors=[];children=[];logs=[];stopping=[False]
    for domain,name in [(P3_SOURCE_DOMAIN,'p4_p3_input_reader'),(CONTROL_DOMAIN,'p4_tcp_runtime')]:
        context=rclpy.Context();rclpy.init(context=context,domain_id=domain)
        node=rclpy.create_node(name,context=context);executor=SingleThreadedExecutor(context=context)
        executor.add_node(node);contexts.append(context);nodes.append(node);executors.append(executor)
    source,target=nodes
    qos=dict(map=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL),
             odom=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT),
             tf=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT),
             tf_output=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE),
             odom_output=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE),
             tf_static=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    handles=input_bridge(source,target,dict(map=GridMap,odom=Odometry,tf=TFMessage),qos)
    if options.rviz_config is not None and not options.check_only:
        qos['display']=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos['goal']=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
        handles.extend(attach_navigation_observer(source,target,dict(pose=PoseStamped,path=RosPath,odom=Odometry),qos))
    source_counts={};relay=None;lease=None;mode=None;vel=None
    for topic,cls in [('/Car/T3/mapping/grid_map',GridMap),('/Car/T3/localization/odometry',Odometry),('/tf',TFMessage),('/tf_static',TFMessage)]:
        def received(msg,key=topic):source_counts[key]=source_counts.get(key,0)+1
        key='map' if cls is GridMap else 'odom' if cls is Odometry else 'tf_static' if topic=='/tf_static' else 'tf'
        handles.append(target.create_subscription(cls,topic,received,qos[key]))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            for executor in executors:executor.spin_once(timeout_sec=.005)
    def stop(*unused):stopping[0]=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    started=time.monotonic();last_report=0.;result=dict(control_domain=CONTROL_DOMAIN,p3_source_domain=P3_SOURCE_DOMAIN,check_only=options.check_only)
    try:
        if not options.check_only:
            profile=low_speed_profile(yaml.safe_load((root/'source/config/wheel.yaml').read_text()))
            config=out/'test-wheel.yaml';config.write_text(yaml.safe_dump(profile,sort_keys=False))
            budgets=yaml.safe_load((root/'source/config/tcp_p3_feedback.yaml').read_text())
            args=launch_settings(root/'source',root/'.cache/nav1cm',config,budgets)
            env=runtime_environment(root,out)
            commands=[('navigation',['ros2','launch','lunar_pure_exploration_ros','nav1cm_exploration.launch.py']+[k+':='+str(v) for k,v in args.items()])]
            if options.rviz_config is not None:
                commands.append(('rviz',['rviz2','-d',str(options.rviz_config),'--ros-args','-r','__node:=p4_planning_rviz']))
            for name,command in commands:
                log=(out/(name+'.log')).open('w');logs.append(log)
                child=subprocess.Popen(command,env=env,cwd=out,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                children.append((name,child))
            (out/'launch-arguments.json').write_text(json.dumps(args,indent=2))
            vel=target.create_publisher(Twist,'/car/cmd_vel',10)
            lease=target.create_publisher(Bool,'/car/external_control_lease',10)
            mode=target.create_publisher(String,'/car/set_mode',10)
            relay=TcpCommandRelay(profile,publish_velocity=vel.publish,
                publish_mode=lambda value:mode.publish(String(data=value)),
                publish_lease=lambda value:lease.publish(Bool(data=value)),make_zero=Twist)
            handles.append(target.create_subscription(VehicleTelemetry,'/car/telemetry',relay.update_telemetry,qos['odom']))
            handles.append(target.create_subscription(Twist,PREFIX+'/cmd_vel',relay.receive,qos['odom']))
            relay.acquire();spin(.3)
        (out/'pids.json').write_text(json.dumps(dict(operator=os.getpid(),children={name:c.pid for name,c in children})))
        while not stopping[0] and (options.duration<=0 or time.monotonic()-started<options.duration):
            spin(.05)
            if any(c.poll() is not None for name,c in children if name=='navigation'):
                raise RuntimeError('navigation_group_exited')
            if relay is not None:relay.heartbeat()
            if time.monotonic()-last_report>=5.:
                last_report=time.monotonic()
                (out/'live.json').write_text(json.dumps(dict(result,source_counts=source_counts,forwarded_nonzero=0 if relay is None else relay.forwarded_nonzero,armed=False if relay is None else relay.armed)))
    except BaseException as exc:
        result['error']=repr(exc)
    finally:
        if relay is not None:
            relay.begin_stop()
            for _ in range(10):relay.heartbeat();spin(.1)
            relay.release();spin(.2)
            result['measured_stopped']=relay.measured_stopped()
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:result[name+'_shutdown_timeout']=True
        result.update(source_counts=source_counts,forwarded_nonzero=0 if relay is None else relay.forwarded_nonzero,children={name:c.poll() for name,c in children})
        (out/'result.json').write_text(json.dumps(result,indent=2))
        for log in logs:log.close()
        for executor in executors:executor.shutdown()
        for node in nodes:node.destroy_node()
        for context in contexts:rclpy.shutdown(context=context)
    return int('error' in result)


if __name__=='__main__':raise SystemExit(main())
