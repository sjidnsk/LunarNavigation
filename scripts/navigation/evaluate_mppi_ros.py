#!/usr/bin/env python3
"""Public ROS controller A/B with an explicit fine-map service and true plant.

This validates ROS transport/controller integration using controlled maps.
The separate environment experiment validates the actual navigation producer.
"""
import argparse
from array import array
from collections import deque
import json,math,os,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,Twist,TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from lunar_planning_msgs.msg import PathReference,TrackingStatus,PolicyMapTile
from lunar_planning_msgs.srv import GetPolicyMap
from lunar_drl_exploration.processes import OwnedProcesses
from lunar_drl_exploration.domain_lease import DomainLease
from lunar_drl_exploration.ros_messages import odometry_message,ros_time
from lunar_drl_exploration.plant import KinematicPlant,GeometryFailure
from lunar_drl_exploration.contracts import Pose
from lunar_drl_exploration.config import load_platform_config
from evaluate_mppi import fixtures
from types import SimpleNamespace


def run(case,backend,domain,pace,max_ticks,controller_python=None,drain_io=False,trace_diagnostics=True):
    lease=DomainLease.acquire(domain);owned=OwnedProcesses();node=None
    os.environ['ROS_DOMAIN_ID']=str(lease.domain_id)
    command=(0.,0.);statuses=[];diagnostics=[];requests=0;states=case['states'].copy();revision=1;trace=[]
    terrain=SimpleNamespace(navigation=states,origin=(*case['origin'],0.),resolution_m=case['resolution'])
    plant=KinematicPlant(terrain,Pose(*case['pose']),load_platform_config())
    pending=deque([(0.,0.)]*2)
    def tiles():
        h,w=states.shape;out=[]
        for y in range((h+255)//256):
            for x in range((w+255)//256):
                s=np.zeros((256,256),np.uint8);part=states[y*256:(y+1)*256,x*256:(x+1)*256];s[:part.shape[0],:part.shape[1]]=part
                t=PolicyMapTile();t.tile_x=x;t.tile_y=y;t.states=array('B',s.ravel());t.costs=array('f',np.zeros(65536,np.float32));out.append(t)
        return out
    wire_tiles=tiles()
    def on_map(req,res):
        nonlocal requests
        requests+=1;res.ready=True;res.reason_code='READY';res.epoch='mppi-benchmark';res.fine_revision=revision;res.raw_elevation_revision=revision
        res.full_snapshot=req.since_revision!=revision;res.base_revision=req.since_revision
        res.frame_id='map';res.resolution_m=case['resolution'];res.origin.x,res.origin.y=case['origin']
        res.profile_hash='paired-controller-fixture';res.processed_stamp=ros_time(sim_ns)
        res.goal_position_tolerance_m=.1;res.goal_yaw_tolerance_rad=.05
        res.anchor_pose.position.x=float(plant.pose.x);res.anchor_pose.position.y=float(plant.pose.y);res.anchor_pose.orientation.w=1.
        if res.full_snapshot:res.tiles=wire_tiles
        return res
    def on_command(msg):
        nonlocal command
        command=(msg.linear.x,msg.angular.z)
        diagnostics.append(dict(event='receive_command',now_ns=sim_ns,command=list(command)))
    start=time.monotonic();sim_ns=1_000_000_000;complete=False;collision=False;reason='TIME_BUDGET';sent=False;start_tick=None
    try:
        rclpy.init();node=rclpy.create_node('mppi_ros_evaluation',namespace='/lunar_training/mppi_benchmark',parameter_overrides=[Parameter('use_sim_time',value=True)])
        ns=node.get_namespace();qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ref=node.create_publisher(PathReference,ns+'/reference',qos);clock=node.create_publisher(Clock,'/clock',10)
        odom=node.create_publisher(Odometry,ns+'/odom',10);tf=node.create_publisher(TFMessage,ns+'/tf',10)
        node.create_subscription(Twist,ns+'/cmd_vel',on_command,10);node.create_subscription(TrackingStatus,ns+'/tracking',lambda m:statuses.append(m),10)
        node.create_service(GetPolicyMap,ns+'/get_map',on_map)
        node.create_subscription(String,ns+'/mppi_diagnostics',lambda m:diagnostics.append(json.loads(m.data)),10)
        params=dict(platform_config=str(Path(__file__).resolve().parents[2]/'config/wheel.yaml'),use_sim_time=True,input_mode='incremental_reference',controller_backend=backend,
                    path_reference_topic=ns+'/reference',tracking_status_topic=ns+'/tracking',odometry_topic=ns+'/odom',tf_topic=ns+'/tf',command_topic=ns+'/cmd_vel',policy_map_service=ns+'/get_map',
                    mppi_diagnostics_topic=ns+'/mppi_diagnostics',mppi_trace_diagnostics=trace_diagnostics,goal_position_tolerance_m=.1,goal_yaw_tolerance_rad=.05,max_linear_mps=.2,max_angular_radps=.5,control_rate_hz=20.)
        child_env=dict(os.environ)
        if controller_python:
            child_env['PYTHONPATH']=str(controller_python)+os.pathsep+child_env.get('PYTHONPATH','')
        owned.start('lunar_pure_wheeled_controller','lunar_pure_wheeled_controller_node.py',ns,params,child_env)
        message=PathReference();message.session_id.uuid=[19]*16;message.segment_revision=1;message.traversability_revision=1;message.reaches_final_goal=True;message.path.header.frame_id='map'
        for x,y,yaw in case['points']:
            point=PoseStamped();point.header.frame_id='map';point.pose.position.x=float(x);point.pose.position.y=float(y);point.pose.orientation.z=math.sin(yaw*.5);point.pose.orientation.w=math.cos(yaw*.5);message.path.poses.append(point)
        for tick in range(max_ticks+400):
            owned.check()
            turn=time.monotonic()
            for _ in range(8):rclpy.spin_once(node,timeout_sec=0.)
            if ref.get_subscription_count() and not sent:ref.publish(message);sent=True;start_tick=tick
            if sent and not statuses and tick%10==0:ref.publish(message)
            if sent:
                pending.append(command);applied=pending.popleft()
                try:plant.advance(*applied,.05)
                except GeometryFailure:collision=True;reason='COLLISION';break
                p=plant.pose;trace.append([(tick-start_tick)*.05,p.x,p.y,p.yaw,plant.linear_mps,plant.angular_radps,*command])
            sim_ns+=50_000_000;stamp=ros_time(sim_ns)
            clock.publish(Clock(clock=stamp));odom.publish(odometry_message(plant.pose,plant.linear_mps,plant.angular_radps,sim_ns))
            transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom';transform.header.stamp=stamp;transform.transform.rotation.w=1.;tf.publish(TFMessage(transforms=[transform]))
            rclpy.spin_once(node,timeout_sec=.001)
            if statuses and statuses[-1].state==TrackingStatus.FAILED:reason=statuses[-1].reason;break
            if statuses and statuses[-1].state==TrackingStatus.COMPLETED and abs(plant.linear_mps)<=.01 and abs(plant.angular_radps)<=.03:complete=True;reason='COMPLETED';break
            if sent and tick-start_tick>=max_ticks:break
            if not sent and time.monotonic()-start>15:raise RuntimeError('controller discovery timeout')
            if drain_io:
                while time.monotonic()-turn<pace:
                    rclpy.spin_once(node,timeout_sec=min(.001,max(0.,pace-(time.monotonic()-turn))))
            else:
                time.sleep(max(0.,pace-(time.monotonic()-turn)))
        p=plant.pose;goal=case['points'][-1]
        return dict(case=case['name'],backend=backend,complete=complete,collision=collision,reason=reason,simulation_s=len(trace)*.05,wall_s=time.monotonic()-start,map_requests=requests,
                    position_error_m=math.hypot(p.x-goal[0],p.y-goal[1]),yaw_error_rad=abs(math.atan2(math.sin(p.yaw-goal[2]),math.cos(p.yaw-goal[2]))),diagnostics=diagnostics,trace=trace,stderr=owned.error_tail())
    finally:
        owned.close()
        if node is not None:node.destroy_node();rclpy.shutdown()
        lease.close()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--cases',nargs='+',default=['straight','reverse','wall_-0.079','corner_False','final_yaw']);ap.add_argument('--backends',nargs='+',default=['pure_pursuit','mppi']);ap.add_argument('--domain',type=int,default=176);ap.add_argument('--pace',type=float,default=.05);ap.add_argument('--max-ticks',type=int,default=1000);ap.add_argument('--controller-python',type=Path);ap.add_argument('--drain-io',action='store_true');ap.add_argument('--no-trace',action='store_true');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);summary=[]
    for case in fixtures():
        if case['name'] not in args.cases:continue
        for backend in args.backends:
            result=run(case,backend,args.domain,args.pace,args.max_ticks,args.controller_python,args.drain_io,not args.no_trace);(args.output/(case['name']+'-'+backend+'.json')).write_text(json.dumps(result,indent=2)+'\n')
            summary.append({k:v for k,v in result.items() if k not in ('trace','stderr','diagnostics')});(args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary[-1]),flush=True)
if __name__=='__main__':main()
