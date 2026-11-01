#!/usr/bin/env python3
"""固定既有 Actor 的合成接口短窗；不创建训练器、优化器或 replay。"""
import argparse
from collections.abc import Mapping
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path,value):
    with Path(path).open('x',encoding='utf-8') as stream:
        json.dump(value,stream,ensure_ascii=False,indent=2,default=lambda v:v.tolist() if hasattr(v,'tolist') else dict(v) if isinstance(v,Mapping) else vars(v))


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--artifact',type=Path,required=True)
    p.add_argument('--actor',type=Path)
    p.add_argument('--config',type=Path)
    p.add_argument('--platform',type=Path)
    p.add_argument('--backend',choices=('incremental_v2','nav1cm'),default='nav1cm')
    p.add_argument('--duration',type=float,default=45.)
    p.add_argument('--declare-only',action='store_true')
    p.add_argument('--run',action='store_true')
    p.add_argument('--plant',action='store_true',help=argparse.SUPPRESS)
    return p


def declaration(args, *, maximum_duration_s=90.):
    if not all((args.actor,args.config,args.platform)) or not 0<args.duration<=maximum_duration_s:
        raise ValueError(f'explicit actor/config/platform and duration in (0,{maximum_duration_s:g}] required')
    from lunar_drl_exploration.runtime import ActorPolicy
    from lunar_drl_exploration.inference_node import read_policy_settings
    policy=ActorPolicy.load(args.actor)
    read_policy_settings(args.config,policy.actor.config)
    record=json.loads(args.config.read_text(encoding='utf-8'))
    # 新合成传感器配置，与原文件原字节分离；策略只收这些真实量测的地图。
    record['sensor']=dict(range_m=3.,fov_deg=360.,offset_x_m=0.,offset_y_m=0.,offset_yaw_rad=0.)
    package=Path(__import__('lunar_drl_exploration.runtime',fromlist=['x']).__file__).parent
    return dict(schema='synthetic_nav1cm_inference_v1',backend=args.backend,duration_s=args.duration,
        actor=str(args.actor),actor_sha256=digest(args.actor),config=str(args.config),config_sha256=digest(args.config),
        platform=str(args.platform),platform_sha256=digest(args.platform),
        script_sha256=digest(__file__),source_files={p.name:digest(p) for p in sorted(package.glob('*.py'))},
        inference_config=record,namespace='/lunar_training/task4_'+args.backend,
        scene=dict(extent_m=12.,resolution_m=.2,origin=[-6.,-6.],flat_height_m=0.,start=[.1,.1,0.],
                   task_polygon=[[-5.,-5.],[5.,-5.],[5.,5.],[-5.,5.]]),
        domain=92,localhost_only=True,training=False,optimizer=False,replay=False,
        criteria=dict(actor_schema='task_graph_v5',current_action_feedback=True,
            physical_motion='distance_m > .05 or turn_rad > .05',observed_map_revision_update=True,
            formal_planning='planning_outcome=0, reason_code=PLAN_FOUND, has_reference=true',
            canceled_current_action=True,measured_stop=True,clean_worker_exit=True,
            efficacy_claim=False,arrival_claim=False))


def plant_worker(art):
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist,TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from grid_map_msgs.msg import GridMap
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from lunar_drl_exploration.config import load_platform_config
    from lunar_drl_exploration.contracts import Pose,SensorSpec
    from lunar_drl_exploration.scene import TerrainGrid
    from lunar_drl_exploration.plant import KinematicPlant
    from lunar_drl_exploration.sensor import SensorModel
    from lunar_drl_exploration.ros_messages import MeasurementBuffer,grid_message,odometry_message
    d=json.loads((art/'declaration.json').read_text());ns=d['namespace'];scene=d['scene']
    platform=load_platform_config(Path(d['platform']))
    terrain=TerrainGrid.from_heights(np.zeros((60,60),np.float32),.2,(-6.,-6.),platform)
    plant=KinematicPlant(terrain,Pose(*scene['start']),platform)
    sensor=SensorSpec(**d['inference_config']['sensor']);buffer=MeasurementBuffer(terrain.origin,.2)
    rclpy.init();node=rclpy.create_node('task4_command_plant',namespace=ns)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom=node.create_publisher(Odometry,ns+'/odom',10)
    grid=node.create_publisher(GridMap,ns+'/grid',qos)
    tf=node.create_publisher(TFMessage,ns+'/tf_static',qos)
    transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom';transform.transform.rotation.w=1.
    tf.publish(TFMessage(transforms=[transform]))
    command=[0.,0.];last=[time.monotonic()];frames=[0];stopped=[False]
    signal.signal(signal.SIGINT,lambda *a:stopped.__setitem__(0,True))
    def on_command(msg):command[:]=[float(msg.linear.x),float(msg.angular.z)];last[0]=time.monotonic()
    node.create_subscription(Twist,ns+'/cmd_vel',on_command,10)
    stream=(art/'plant.jsonl').open('x',buffering=1)
    def tick():
        current=command if time.monotonic()-last[0]<.5 else [0.,0.]
        plant.advance(*current,.05)
        stamp=node.get_clock().now().nanoseconds
        odom.publish(odometry_message(plant.pose,plant.linear_mps,plant.angular_radps,stamp))
        stream.write(json.dumps(dict(wall=time.monotonic(),stamp=stamp,pose=vars(plant.pose),
            velocity=[plant.linear_mps,plant.angular_radps],command=current[:],distance_m=plant.distance_m,
            turn_rad=plant.turn_rad,simulation_s=plant.simulation_s,collision=plant.collision))+'\n')
        frames[0]+=1
        if frames[0]%5==0:
            tf.publish(TFMessage(transforms=[transform]))
            measurements=SensorModel.observe(terrain,plant.pose,sensor)
            buffer.add(measurements,stamp)
            # 旧地图输入契约是 odom；此处 map→odom 为恒等变换，物理量测不变。
            frame='odom' if d['backend']=='incremental_v2' else 'map'
            origin,planes=buffer.payload();grid.publish(grid_message(planes,origin,.2,stamp,frame=frame))
    node.create_timer(.05,tick)
    try:
        while not stopped[0] and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        write_json(art/'plant-summary.json',dict(pose=vars(plant.pose),distance_m=plant.distance_m,
            turn_rad=plant.turn_rad,velocity=[plant.linear_mps,plant.angular_radps],collision=plant.collision,
            sensor_frames=frames[0]//5,observed_cells=len(buffer._cells)))
        stream.close();node.destroy_node();rclpy.shutdown()


def run(args,d):
    if os.environ.get('ROS_DOMAIN_ID')!='92' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('isolated domain 92 and localhost-only required')
    art=args.artifact;ns=d['namespace'];children=[];logs=[];runtime=None;node=None
    import rclpy
    from rclpy.parameter import Parameter
    from lunar_drl_exploration.inference_node import attach_node
    from lunar_drl_exploration.navigation_backend import owned_nav1cm_specs
    from lunar_drl_exploration.processes import ros_arguments
    from lunar_drl_exploration.contracts import TaskSpec
    from lunar_drl_exploration.ros_messages import velocity_is_stopped
    from ament_index_python.packages import get_package_prefix
    write_json(art/'inference-config.json',d['inference_config'])
    def spawn(name,command):
        log=(art/(name+'.log')).open('x');logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,child));return child
    trace=None;summary={};start=time.monotonic()
    try:
        spawn('plant',[sys.executable,str(Path(__file__).resolve()),'--plant','--artifact',str(art)])
        specs=owned_nav1cm_specs(ns,platform_config=d['platform'],build_dir=str(art/'planner-cache'),use_sim_time=False)
        if d['backend']=='incremental_v2':
            common=dict(platform_config=d['platform'],odometry_topic=ns+'/odom',tf_topic=ns+'/tf_static',use_sim_time=False)
            specs=[('lunar_incremental_navigation_ros','lunar_incremental_navigation_node',dict(common,
                local_map_topic=ns+'/grid',action_name=ns+'/navigate',path_reference_topic=ns+'/reference',
                local_path_topic=ns+'/path',global_route_topic=ns+'/route',diagnostics_topic=ns+'/diagnostics',
                exploration_map_topic=ns+'/map',policy_map_service=ns+'/get_map',tracking_status_topic=ns+'/tracking',enable_tracking_feedback=True)),
                ('lunar_pure_wheeled_controller','lunar_pure_wheeled_controller_node.py',dict(common,
                input_mode='incremental_reference',path_reference_topic=ns+'/reference',tracking_status_topic=ns+'/tracking',
                reference_topic=ns+'/wheel_reference',path_topic=ns+'/path',command_topic=ns+'/cmd_vel',execution_cancel_topic=ns+'/cancel'))]
        for i,(package,entry,params) in enumerate(specs):
            command=[str(Path(get_package_prefix(package))/'lib'/package/entry),*ros_arguments(ns,params)]
            command[command.index('--log-level')+1]='info';spawn('node'+str(i),command)
        rclpy.init()
        values=dict(actor_path=d['actor'],actor_config_path=str(art/'inference-config.json'),
            navigation_backend=d['backend'],platform_config=d['platform'],task_topic=ns+'/task',status_topic=ns+'/status',
            action_name=ns+'/navigate',policy_map_service=ns+'/get_map',odometry_topic=ns+'/odom',
            tf_topic=ns+'/tf',tf_static_topic=ns+'/tf_static',global_route_topic=ns+'/route')
        node=rclpy.create_node('task4_attached_inference',namespace=ns,
            parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
        trace=(art/'runtime.jsonl').open('x',buffering=1)
        runtime=attach_node(node)
        # legacy 默认诊断参数只用于旧后端；私有端点明确配置。
        if d['backend']=='incremental_v2':
            runtime.adapter.close()
            from lunar_drl_exploration.navigation_backend import create_navigation_adapter
            runtime.adapter=create_navigation_adapter(node,action_name=ns+'/navigate',policy_map_service=ns+'/get_map',
                odometry_topic=ns+'/odom',diagnostics_topic=ns+'/diagnostics',path_reference_topic=ns+'/reference',tf_topic=ns+'/tf_static')
        deadline=time.monotonic()+30.
        while not (runtime.adapter.ready and runtime.adapter.pose is not None):
            if time.monotonic()>deadline:raise RuntimeError('input readiness timeout')
            for name,child in children:
                if child.poll() is not None:raise RuntimeError(name+' exited')
            rclpy.spin_once(node,timeout_sec=.05)
        write_json(art/'graph.json',dict(topics=node.get_topic_names_and_types(),
            command_publishers=[dict(node_name=v.node_name,namespace=v.node_namespace,topic_type=v.topic_type)
                for v in node.get_publishers_info_by_topic(ns+'/cmd_vel')],pids={n:c.pid for n,c in children}))
        runtime.start(TaskSpec('synthetic-task4','map',d['scene']['task_polygon']))
        end=time.monotonic()+d['duration_s'];last_serial=0
        while time.monotonic()<end:
            for name,child in children:
                if child.poll() is not None:raise RuntimeError(name+' exited')
            rclpy.spin_once(node,timeout_sec=.05)
            a=runtime.adapter
            if a._goal_serial!=last_serial and runtime.last_observation is not None:
                last_serial=a._goal_serial
                write_json(art/('observation-'+str(last_serial)+'.json'),vars(runtime.last_observation))
            trace.write(json.dumps(dict(wall=time.monotonic(),status=runtime.status(),goal=a.sent_goal,
                serial=a._goal_serial,execution=a.execution_state(),pose=vars(a.pose) if a.pose else None,
                velocity=a.velocity,revision=a.snapshot.revision if a.snapshot else None,
                processed_stamp=a.processed_stamp_ns,map_reason=a.map_reason,
                last_result=vars(runtime.last_result) if runtime.last_result else None,
                observation_schema=getattr(runtime.last_observation,'schema',None),policy_graph=runtime.policy_graph_stats),
                default=lambda v:vars(v))+'\n')
            if runtime.last_error:raise RuntimeError(runtime.last_error)
        runtime.cancel();deadline=time.monotonic()+15.
        while runtime.state!='CANCELED' and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        summary=dict(runtime=runtime.status(),execution=runtime.adapter.last_goal_execution,
            last_result=vars(runtime.last_result) if runtime.last_result else None,
            stopped=velocity_is_stopped(runtime.adapter.velocity),velocity=runtime.adapter.velocity,
            inflight=runtime.adapter.inflight,wall_s=time.monotonic()-start)
    except BaseException as exc:
        summary=dict(error=repr(exc),wall_s=time.monotonic()-start)
        if runtime is not None:
            runtime.cancel();deadline=time.monotonic()+10.
            while runtime.adapter.inflight and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
        raise
    finally:
        if runtime is not None:runtime.adapter.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if trace is not None:trace.close()
        exits={}
        # 先停止导航/跟踪工作线程，plant仍继续记录最后的制动命令。
        for name,child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5.);summary['forced_exit']=True
            exits[name]=child.returncode
        for log in logs:log.close()
        summary['child_exits']=exits;summary['frozen_after']={key:digest(d[key]) for key in ('actor','config','platform')}
        write_json(art/'summary.json',summary)


def main():
    args=parser().parse_args()
    if args.plant:return plant_worker(args.artifact)
    d=declaration(args)
    if args.declare_only:
        args.artifact.mkdir(parents=True,exist_ok=True);write_json(args.artifact/'declaration.json',d)
        print(json.dumps(dict(declaration=str(args.artifact/'declaration.json'),sha256=digest(args.artifact/'declaration.json'))));return
    if not args.run:raise ValueError('use --declare-only before --run')
    previous=json.loads((args.artifact/'declaration.json').read_text())
    if previous!=d:raise ValueError('frozen declaration/source/input drift')
    return run(args,d)


if __name__=='__main__':main()