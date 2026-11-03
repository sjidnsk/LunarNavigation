"""P4 ROS域继承入口：P3按需跨域输入、导航启动及TCP租约适配。"""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
from collections import deque

CONTROL_DOMAIN=57
P3_SOURCE_DOMAIN=57
PREFIX='/Car/T4'
TRANSPORT_TIMEOUT_S=.5


try:
    from .p4_topics import load_topic_contract, launch_topic_arguments, render_rviz_config
except ImportError:
    from p4_topics import load_topic_contract, launch_topic_arguments, render_rviz_config


def launch_settings(source,build_dir,profile_path,budgets,*,prefix=None,goal_topic=None,exploration_strategy="frontier",contract=None):
    if exploration_strategy not in ("none", "frontier"):
        raise ValueError("exploration_strategy must be none or frontier")
    allowed={'odometry_timeout_s','p3_map_timeout_s','mppi_time_steps','mppi_compute_envelope_s',
             'command_hold_s','global_period_s','command_spin_prepare_s','global_soft_clearance_m','global_budget_s','coverage_target','policy_resolution_m','coarse_resolution_m'}
    if not isinstance(budgets,dict) or set(budgets)-allowed:
        raise ValueError('TCP profile只能覆盖既有输入/预测/计算预算')
    contract=load_topic_contract(source) if contract is None else contract
    args=launch_topic_arguments(source,namespace=prefix,contract=contract)
    args.update({k:str(v) for k,v in budgets.items()})
    args.update(use_sim_time='false',build_dir=str(build_dir),allow_unknown='true',
                map_qos_reliability='reliable',map_qos_durability='transient_local',
                command_spin_prepare_s=str(budgets.get('command_spin_prepare_s',0.)),
                global_period_s=str(budgets.get('global_period_s',15.)),command_hold_s=str(budgets['command_hold_s']),
                horizon_steps='80',horizon_dt_s='0.2',command_prediction_enabled='true',
                prediction_command_topic=contract['vehicle']['command_topic'],
                control_backend='mpcc_cmd_vel',enable_controller='true',exploration_strategy=exploration_strategy,
                platform_config=str(profile_path),initial_travel_direction='forward')
    if goal_topic is not None:args['goal_topic']=goal_topic
    return args


def control_domain():
    return int(os.environ.get('ROS_DOMAIN_ID') or CONTROL_DOMAIN)


def input_bridge(source,target,classes,qos,contract=None,*,source_domain=P3_SOURCE_DOMAIN,target_domain=None):
    if target_domain is None:target_domain=control_domain()
    # 同域直接订阅源话题，禁止同话题重发形成反馈环。
    if source_domain==target_domain:return []
    topics=(load_topic_contract() if contract is None else contract)['launch']
    # 回调直接交付原消息，禁止重标源时间、反向转发或控制转发。
    handles=[]
    for topic,kind,key in [(topics['map_topic'],'map','map'),
                           (topics['odometry_topic'],'odom','odom'),
                           (topics['tf_topic'],'tf','tf'),(topics['tf_static_topic'],'tf','tf_static')]:
        publisher=target.create_publisher(classes[kind],topic,qos.get(key+'_output',qos[key]))
        subscription=source.create_subscription(classes[kind],topic,publisher.publish,qos[key])
        handles.extend((publisher,subscription))
    return handles


class NavigationObserver:
    """只在显示线程处理真实源观测；绝不参与导航。"""
    def __init__(self, make_pose, make_path, publish_start, publish_goal, publish_path, timeout, defer_path=False):
        self.make_pose, self.make_path = make_pose, make_path
        self.publish_start, self.publish_goal, self.publish_path = publish_start, publish_goal, publish_path
        self.timeout = float(timeout)
        self.defer_path = defer_path; self.pending_header = None
        self.latest = self.last_advance = self.high_stamp = self.path_header = None
        self.active = False
        self.start = None
        self.poses = deque(maxlen=2000)

    def goal(self, msg, received):
        self.active = True
        fresh = self.latest is not None and 0 <= received-self.last_advance <= self.timeout
        self.start = deepcopy(self.latest) if fresh else None
        self.poses.clear()
        if self.start is not None: self.poses.append(deepcopy(self.start))
        self.publish_start(self.start)  # None 明确发送 Marker DELETE。
        self.publish_goal(msg)
        self._path(msg.header)

    def odometry(self, msg, received):
        h, p = msg.header, msg.pose.pose
        stamp = (h.stamp.sec, h.stamp.nanosec)
        values = (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
        if (not h.frame_id or stamp <= (0,0) or not 0 <= stamp[1] < 1000000000
                or not all(math.isfinite(v) for v in values)
                or sum(v*v for v in values[3:]) < 1e-12): return
        if self.high_stamp is not None and stamp <= self.high_stamp: return
        if self.last_advance is not None and received < self.last_advance: return
        self.high_stamp, self.last_advance = stamp, received
        pose = self.make_pose(); pose.header = deepcopy(h); pose.pose = deepcopy(p)
        self.latest = pose
        if not self.active: return
        if self.start is None:
            self.start = deepcopy(pose); self.publish_start(self.start)
        if self.poses and self.poses[-1].header.frame_id != h.frame_id: self.poses.clear()
        self.poses.append(pose)
        self._path(h)

    def _path(self, fallback_header):
        if self.defer_path:
            self.pending_header = fallback_header
            return
        self._publish_path(fallback_header)

    def flush(self):
        if self.pending_header is not None:
            header = self.pending_header; self.pending_header = None
            self._publish_path(header)

    def _publish_path(self, fallback_header):
        path = self.make_path()
        header = self.poses[-1].header if self.poses else (self.path_header or fallback_header)
        if not header.frame_id: return
        path.header = deepcopy(header)
        self.path_header = deepcopy(header)
        path.poses = deepcopy(list(self.poses))
        self.publish_path(path)


class DisplayWorker:
    """两个单槽邮箱：主线程仅交换引用，显示工作不持锁。"""
    def __init__(self, observer, timeout=4.5):
        self.observer = observer
        self.timeout = timeout; self.latest_offer = None; self.offer_high_stamp = None; self.waiting_goal = None
        self.lock = threading.Lock(); self.wake = threading.Event()
        self.sequence = 0; self.pending = {}; self.closing = False
        self.thread = threading.Thread(target=self._run, name='p4-display', daemon=True)

    def start(self): self.thread.start()
    def offer_odom(self, msg, received=None): self._offer('odom',msg,received)
    def offer_goal(self, msg, received=None): self._offer('goal',msg,received)
    def _offer(self, kind, msg, received):
        received = time.monotonic() if received is None else received
        with self.lock:
            self.sequence += 1
            if kind == 'odom':
                # 固定大小的合法性检查与锁存，不复制轨迹或 ROS 消息。
                try:
                    h, p = msg.header, msg.pose.pose
                    stamp = (h.stamp.sec,h.stamp.nanosec)
                    values = (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)
                    valid = (bool(h.frame_id) and stamp > (0,0) and 0 <= stamp[1] < 1000000000
                             and all(math.isfinite(v) for v in values) and sum(v*v for v in values[3:]) >= 1e-12
                             and (self.offer_high_stamp is None or stamp > self.offer_high_stamp)
                             and (self.latest_offer is None or received >= self.latest_offer[1]))
                except (AttributeError,TypeError,ValueError): valid = False
                if not valid: return
                self.offer_high_stamp = stamp; self.latest_offer = (msg,received)
                if self.waiting_goal is not None and self.waiting_goal[0] is None:
                    self.waiting_goal[0] = (msg,received)
                self.pending[kind] = (self.sequence,msg,received)
            else:
                pre = self.latest_offer
                if pre is not None and not 0 <= received-pre[1] <= self.timeout: pre = None
                waiting = [None] if pre is None else None
                self.waiting_goal = waiting
                self.pending[kind] = (self.sequence,msg,received,pre,waiting)
            self.wake.set()

    def _run(self):
        deadline = 0.
        while True:
            self.wake.wait()
            while True:
                with self.lock:
                    remaining = deadline-time.monotonic()
                    if self.closing or remaining <= 0:
                        batch = sorted((item[0],kind,item) for kind,item in self.pending.items())
                        waiting = self.waiting_goal
                        first = waiting[0] if waiting is not None else None
                        if first is not None: self.waiting_goal = None
                        self.pending.clear(); self.wake.clear(); closing = self.closing
                        break
                    self.wake.clear()
                self.wake.wait(remaining)
            if closing: return
            has_goal = any(kind=='goal' for _,kind,_ in batch)
            if first is not None and not has_goal: self.observer.odometry(*first)
            for _,kind,item in batch:
                _,msg,received,*extra = item
                if kind=='goal':
                    pre, goal_waiting = extra
                    if pre is not None: self.observer.odometry(*pre)
                    self.observer.goal(msg,received)
                    if first is not None and goal_waiting is waiting: self.observer.odometry(*first)
                else:
                    self.observer.odometry(msg,received)
            flush = getattr(self.observer,'flush',None)
            if flush is not None: flush()
            deadline = time.monotonic()+.2

    def stop(self):
        with self.lock: self.closing = True; self.wake.set()
        self.thread.join()


def attach_navigation_observer(source, target, classes, qos, timeout, contract=None):
    contract=load_topic_contract() if contract is None else contract
    topics=contract['launch']; visualization=contract['visualization']
    pubs = [target.create_publisher(classes[kind], visualization[key], qos['display']) for kind,key in
            [('marker','start_marker'),('marker','goal_marker'),('path','actual_path')]]
    start_header = [None]
    def marker(pose, index, color):
        m = classes['marker'](); m.ns='p4_navigation'; m.id=index
        if pose is None:
            if start_header[0] is None: return None
            m.header=deepcopy(start_header[0]); m.action=m.DELETE
        else:
            m.header=deepcopy(pose.header); m.pose=deepcopy(pose.pose)
            if index==0: start_header[0]=deepcopy(pose.header)
            m.type=m.ARROW; m.action=m.ADD
            m.scale.x=.8; m.scale.y=.18; m.scale.z=.18
            m.color.r,m.color.g,m.color.b=color; m.color.a=1.
        return m
    def publish_start(pose):
        m=marker(pose,0,(0.,1.,0.))
        if m is not None: pubs[0].publish(m)
    observer = NavigationObserver(classes['pose'],classes['path'],
        publish_start,
        lambda p:pubs[1].publish(marker(p,1,(1.,0.,0.))),pubs[2].publish,timeout,defer_path=True)
    worker = DisplayWorker(observer,timeout); worker.start()
    subs = [source.create_subscription(classes['odom'],topics['odometry_topic'],worker.offer_odom,qos['odom']),
            target.create_subscription(classes['pose'],topics['goal_topic'],worker.offer_goal,qos['goal'])]
    return worker,[*pubs,*subs]


def profile_command_limits(profile):
    capability=profile['capability']
    values=tuple(float(capability[name]) for name in
        ('maximum_forward_speed_mps','maximum_reverse_speed_mps','maximum_spin_rate_radps'))
    if any(not math.isfinite(v) or v<=0 for v in values):raise ValueError('invalid command limits')
    return values


def tcp_platform_profile(base, speed_override):
    """TCP 专用速度配置仅覆盖速度，其余车辆能力继承公共 profile。"""
    keys = {'maximum_forward_speed_mps', 'maximum_reverse_speed_mps', 'maximum_spin_rate_radps'}
    if not isinstance(speed_override, dict) or set(speed_override) != keys:
        raise ValueError('invalid TCP speed override fields')
    result = deepcopy(base)
    result['capability'].update(speed_override)
    profile_command_limits(result)
    return result


def load_tcp_platform_profile(source):
    """两个 TCP 入口读取同一基础能力和速度覆盖。"""
    import yaml
    config = Path(source) / 'config'
    return tcp_platform_profile(
        yaml.safe_load((config / 'wheel.yaml').read_text(encoding='utf-8')),
        yaml.safe_load((config / 'tcp_wheel_speed.yaml').read_text(encoding='utf-8')))


def low_speed_profile(profile,*,linear_mps=.1,angular_rps=.1950770225306774):
    if any(not math.isfinite(v) or v<=0 for v in (linear_mps,angular_rps)):
        raise ValueError('invalid low-speed profile')
    forward,reverse,angular=profile_command_limits(profile)
    result=deepcopy(profile)
    result['capability'].update(maximum_forward_speed_mps=min(forward,linear_mps),
        maximum_reverse_speed_mps=min(reverse,linear_mps),maximum_spin_rate_radps=min(angular,angular_rps))
    return result


def runtime_source(root,environ=None):
    """从选定正式构建读取同版脚本/配置；旧入口默认仍为P4/source。"""
    root=Path(root).resolve();environ=os.environ if environ is None else environ
    source=Path(environ.get('P4_SOURCE_DIR') or root/'source').resolve()
    if not source.is_relative_to(root) or not source.is_dir():
        raise ValueError('selected source must exist inside P4')
    return source


def runtime_environment(root,out):
    root,out=Path(root),Path(out)
    env=dict(os.environ,ROS_DOMAIN_ID=str(control_domain()),ROS_LOCALHOST_ONLY='0',
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
        self._telemetry=None
        self._connected=None
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

    def update_connected(self,message):
        self._connected=bool(message.data)
        if self._connected is False:self.park()

    def receive(self,message):
        # 收尾/未接管时不再处理运动输入，非法队列消息也不得打断释放与结果保存。
        if not self.leased:return False
        if self._stopping:self.park();return False
        v,w=message.linear.x,message.angular.z
        forward,reverse,angular=self.limits
        if not all(math.isfinite(x) for x in (v,w)) or not -reverse-1e-6<=v<=forward+1e-6 or abs(w)>angular+1e-6:
            self.park();raise ValueError('invalid_or_over_limit_controller_command')
        if abs(v)+abs(w)<1e-7:
            self._velocity(message);self.forwarded_zero+=1;return False
        if self._connected is False or (not self.armed and (self._telemetry is None or not 0<=self._clock()-self._telemetry[0]<=1.)):
            self.park();return False
        # 既有租约唯一 owner 重申幂等 auto，使底盘 watchdog 停车后可随新输入恢复。
        self._mode('auto');self.armed=True
        self._velocity(message);self.forwarded_nonzero+=1
        return True

    def heartbeat(self):
        if not self.leased:return False
        self._lease(True)
        # 指令间隔不参与停车；真实断连与明确停止由各自事件处理。
        return not self._stopping and self._connected is not False

    def measured_stopped(self):
        if self._telemetry is None:return False
        received,tel=self._telemetry
        velocity=(tel.vx,tel.vy,tel.wz)
        return (0<=self._clock()-received<TRANSPORT_TIMEOUT_S and
                all(math.isfinite(v) for v in velocity) and
                max(map(abs,velocity))<.01)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('/home/yanfa/P4'))
    parser.add_argument('--artifact',type=Path,required=True)
    parser.add_argument('--rviz-config',type=Path)
    parser.add_argument('--exploration-strategy',choices=('none','frontier'),default='frontier',help='选择探索节点；frontier仍等待显式区域START任务')
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
    from visualization_msgs.msg import Marker
    from grid_map_msgs.msg import GridMap
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool,String
    from lunar_car_ctrl.msg import VehicleTelemetry
    source_root=runtime_source(root)
    contract=load_topic_contract(source_root);topics=contract['launch'];vehicle=contract['vehicle']
    selected_domain=control_domain()
    contexts=[];nodes=[];executors=[];children=[];logs=[];stopping=[False]
    for domain,name in [(P3_SOURCE_DOMAIN,'p4_p3_input_reader'),(selected_domain,'p4_tcp_runtime')]:
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
    handles=input_bridge(source,target,dict(map=GridMap,odom=Odometry,tf=TFMessage),qos,contract=contract,target_domain=selected_domain)
    display_worker=None
    source_counts={};relay=None;lease=None;mode=None;vel=None
    for topic,cls in [(topics['map_topic'],GridMap),(topics['odometry_topic'],Odometry),(topics['tf_topic'],TFMessage),(topics['tf_static_topic'],TFMessage)]:
        def received(msg,key=topic):source_counts[key]=source_counts.get(key,0)+1
        key='map' if cls is GridMap else 'odom' if cls is Odometry else 'tf_static' if topic==topics['tf_static_topic'] else 'tf'
        handles.append(target.create_subscription(cls,topic,received,qos[key]))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            for executor in executors:executor.spin_once(timeout_sec=.005)
    def stop(*unused):stopping[0]=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    started=time.monotonic();last_report=0.;result=dict(control_domain=selected_domain,p3_source_domain=P3_SOURCE_DOMAIN,check_only=options.check_only,
        source_root=str(source_root),formal_build=os.environ.get('P4_FORMAL_BUILD'),
        source_commit=os.environ.get('P4_FORMAL_COMMIT'))
    try:
        if not options.check_only:
            profile=load_tcp_platform_profile(source_root)
            config=out/'test-wheel.yaml';config.write_text(yaml.safe_dump(profile,sort_keys=False))
            budgets=yaml.safe_load((source_root/'config/tcp_p3_feedback.yaml').read_text())
            if options.rviz_config is not None and not options.check_only:
                qos['display']=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                qos['goal']=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
                display_worker,display_handles=attach_navigation_observer(source,target,dict(pose=PoseStamped,path=RosPath,odom=Odometry,marker=Marker),qos,budgets['odometry_timeout_s'],contract=contract)
                handles.extend(display_handles)
            args=launch_settings(source_root,root/'.cache/nav1cm',config,budgets,exploration_strategy=options.exploration_strategy,contract=contract)
            env=runtime_environment(root,out)
            commands=[('navigation',['ros2','launch','lunar_pure_exploration_ros','nav1cm_exploration.launch.py']+[k+':='+str(v) for k,v in args.items()])]
            if options.rviz_config is not None:
                rviz_config=out/'p4_navigation.rviz'
                rviz_config.write_text(yaml.safe_dump(render_rviz_config(yaml.safe_load(options.rviz_config.read_text(encoding='utf-8')),contract),sort_keys=False),encoding='utf-8')
                commands.append(('rviz',['rviz2','-d',str(rviz_config),'--ros-args','-r','__node:=p4_planning_rviz']))
            for name,command in commands:
                log=(out/(name+'.log')).open('w');logs.append(log)
                child=subprocess.Popen(command,env=env,cwd=out,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                children.append((name,child))
            (out/'launch-arguments.json').write_text(json.dumps(args,indent=2))
            vel=target.create_publisher(Twist,vehicle['command_topic'],10)
            lease=target.create_publisher(Bool,vehicle['lease_topic'],10)
            mode=target.create_publisher(String,vehicle['mode_topic'],10)
            relay=TcpCommandRelay(profile,publish_velocity=vel.publish,
                publish_mode=lambda value:mode.publish(String(data=value)),
                publish_lease=lambda value:lease.publish(Bool(data=value)),make_zero=Twist)
            handles.append(target.create_subscription(VehicleTelemetry,vehicle['telemetry_topic'],relay.update_telemetry,qos['odom']))
            handles.append(target.create_subscription(Bool,vehicle['connected_topic'],relay.update_connected,qos['map']))
            handles.append(target.create_subscription(Twist,args['command_topic'],relay.receive,qos['odom']))
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
        if display_worker is not None:display_worker.stop()
        for executor in executors:executor.shutdown()
        for node in nodes:node.destroy_node()
        for context in contexts:rclpy.shutdown(context=context)
    return int('error' in result)


if __name__=='__main__':raise SystemExit(main())
