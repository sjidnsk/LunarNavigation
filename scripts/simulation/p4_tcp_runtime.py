"""P4控制域10入口：P3单向输入桥、导航启动及同域TCP租约适配。"""
import argparse
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


def launch_settings(source,build_dir,profile_path,budgets):
    try:from .debug_p3_planning import launch_arguments
    except ImportError:from debug_p3_planning import launch_arguments
    args=launch_arguments(source,build_dir,profile=budgets,allow_unknown=True)
    args.update(enable_controller='true',exploration_strategy='none',
                platform_config=str(profile_path),command_topic=PREFIX+'/cmd_vel',goal_topic='/goal_pose',
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


def command_ready(command_age,telemetry_age,status_age,accepted,state):
    return (0<=command_age<=.5 and 0<=telemetry_age<=.5 and
            0<=status_age<=1. and accepted and state!=6)


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
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from grid_map_msgs.msg import GridMap
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import Bool,String
    from lunar_planning_msgs.msg import TrackingStatus
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
    source_counts={};latest={};armed=False;lease=None;mode=None;vel=None;forwarded=0
    for topic,cls in [('/Car/T3/mapping/grid_map',GridMap),('/Car/T3/localization/odometry',Odometry),('/tf',TFMessage),('/tf_static',TFMessage)]:
        def received(msg,key=topic):source_counts[key]=source_counts.get(key,0)+1
        key='map' if cls is GridMap else 'odom' if cls is Odometry else 'tf_static' if topic=='/tf_static' else 'tf'
        handles.append(target.create_subscription(cls,topic,received,qos[key]))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            for executor in executors:executor.spin_once(timeout_sec=.005)
    def park():
        nonlocal armed
        if vel is not None:vel.publish(Twist());mode.publish(String(data='park'))
        armed=False
    def stop(*unused):stopping[0]=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    started=time.monotonic();last_report=0.;result=dict(control_domain=CONTROL_DOMAIN,p3_source_domain=P3_SOURCE_DOMAIN,check_only=options.check_only)
    try:
        if not options.check_only:
            profile=yaml.safe_load((root/'source/config/wheel.yaml').read_text())
            # 仅本次调试profile，沿用此前获准的低速上限。
            profile['capability'].update(maximum_forward_speed_mps=.1,maximum_reverse_speed_mps=.1,
                                         maximum_spin_rate_radps=.1950770225306774)
            config=out/'test-wheel.yaml';config.write_text(yaml.safe_dump(profile,sort_keys=False))
            budgets=yaml.safe_load((root/'source/config/tcp_p3_feedback.yaml').read_text())
            args=launch_settings(root/'source',root/'.cache/nav1cm',config,budgets)
            env=dict(os.environ,ROS_DOMAIN_ID=str(CONTROL_DOMAIN),ROS_LOCALHOST_ONLY='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_LOG_DIR=str(out/'ros-log'))
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
            def store(key,msg):latest[key]=(time.monotonic(),msg)
            handles.append(target.create_subscription(TrackingStatus,PREFIX+'/tracking_status',lambda msg:store('tracking',msg),qos['odom']))
            handles.append(target.create_subscription(VehicleTelemetry,'/car/telemetry',lambda msg:store('telemetry',msg),qos['odom']))
            def command(msg):
                nonlocal armed,forwarded
                v,w=msg.linear.x,msg.angular.z;now=time.monotonic()
                if not all(math.isfinite(x) for x in (v,w)) or abs(v)>.100001 or abs(w)>.195078:
                    park();raise RuntimeError('invalid_or_over_limit_controller_command')
                store('command',msg)
                if abs(v)+abs(w)<1e-7:vel.publish(msg);return
                telemetry=latest.get('telemetry');status=latest.get('tracking')
                if not telemetry or not status or not command_ready(0.,now-telemetry[0],now-status[0],status[1].reference_accepted,status[1].state):
                    park();return
                if not armed:mode.publish(String(data='auto'));armed=True
                vel.publish(msg);forwarded+=1
            handles.append(target.create_subscription(Twist,PREFIX+'/cmd_vel',command,qos['odom']))
            # GUI既有1.5s租约机制；本进程失联后GUI和TCP节点仍自行停车。
            park();lease.publish(Bool(data=True));spin(.3)
        (out/'pids.json').write_text(json.dumps(dict(operator=os.getpid(),children={name:c.pid for name,c in children})))
        while not stopping[0] and (options.duration<=0 or time.monotonic()-started<options.duration):
            spin(.05)
            if any(c.poll() is not None for name,c in children if name=='navigation'):
                raise RuntimeError('navigation_group_exited')
            if lease is not None:
                lease.publish(Bool(data=True));now=time.monotonic()
                cmd=latest.get('command');tel=latest.get('telemetry');status=latest.get('tracking')
                if not (cmd and tel and status and command_ready(now-cmd[0],now-tel[0],now-status[0],status[1].reference_accepted,status[1].state)):
                    park()
            if time.monotonic()-last_report>=5.:
                last_report=time.monotonic()
                (out/'live.json').write_text(json.dumps(dict(result,source_counts=source_counts,forwarded_nonzero=forwarded,armed=armed)))
    except BaseException as exc:
        result['error']=repr(exc)
    finally:
        if lease is not None:
            for _ in range(10):park();lease.publish(Bool(data=True));spin(.1)
            lease.publish(Bool(data=False));spin(.2)
            tel=latest.get('telemetry')
            result['measured_stopped']=bool(tel and time.monotonic()-tel[0]<.5 and max(abs(tel[1].vx),abs(tel[1].vy))<.01 and max(abs(getattr(tel[1],x)) for x in ('wheel_speed_fl','wheel_speed_fr','wheel_speed_rl','wheel_speed_rr'))<.05)
        for name,child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:result[name+'_shutdown_timeout']=True
        result.update(source_counts=source_counts,forwarded_nonzero=forwarded,children={name:c.poll() for name,c in children})
        (out/'result.json').write_text(json.dumps(result,indent=2))
        for log in logs:log.close()
        for executor in executors:executor.shutdown()
        for node in nodes:node.destroy_node()
        for context in contexts:rclpy.shutdown(context=context)
    return int('error' in result)


if __name__=='__main__':raise SystemExit(main())
