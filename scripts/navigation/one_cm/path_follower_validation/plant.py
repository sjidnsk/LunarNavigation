from .qos import profile_data,entity_rows
"""独立真实Twist植物：有界速度跟随，完整时钟细分积分；不读取Path控制。"""
import math
import sys
import time
from dataclasses import asdict
from nav_msgs.msg import Odometry,Path as PathMessage
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from grid_map_msgs.msg import GridMap
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String,Float32MultiArray,MultiArrayDimension
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from .evidence import Recorder,TOPICS,CMD,DIAG,path_data,legacy,obstacles,goal
# 仅复用解析SAT和运动积分函数，绝不构造旧PathFollower。
geometry=legacy('motion')
State,integrate,swept_clear=geometry.State,geometry.integrate,geometry.swept_clear

def run(args):
    rec=Recorder(args.output,'plant')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=Node('nav1cm_joint_plant')
    state=State()
    command=[0.,0.]
    last=time.monotonic()
    def receive(msg):
        command[:]=[msg.linear.x,msg.angular.z]
        rec.emit('COMMAND',v=command[0],w=command[1])
    node.create_subscription(Twist,CMD,receive,100)
    node.create_subscription(String,DIAG,lambda m:rec.emit('DIAGNOSTIC',data=m.data),100)
    for stream,topic in [('local',TOPICS['local']),('global',TOPICS['global_route'])]:
        node.create_subscription(PathMessage,topic,lambda m,s=stream:rec.emit('PATH',stream=s,message=path_data(m)),100)
    odom_pub=node.create_publisher(Odometry,TOPICS['odom'],1)
    map_pub=node.create_publisher(GridMap,TOPICS['grid'],1)
    goal_pub=node.create_publisher(PoseStamped,TOPICS['goal'],10)
    path_pub=node.create_publisher(PathMessage,TOPICS['local'],1) if args.case=='empty_smoke' else None
    tf_pub=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    rec.emit('ENTITY_QOS',entities=entity_rows(node))
    sent=False
    resumed=False
    next_map=next_tf=next_graph=next_path=0.
    map_stage=None
    cached=None
    rec.emit('PLANT_CONTRACT',body=[1.182,.818],max_dt=.01,response='bounded slew .5, exact midpoint substeps; command only')
    try:
        while time.monotonic()-args.start<args.duration:
            # 先把旧指令作用至真实当前时刻，再消费新指令，避免回溯施加新命令。
            now=time.monotonic()
            remaining=now-last
            while remaining>1e-9:
                dt=min(.01,remaining)
                a=max(-.5,min(.5,(command[0]-state.v)/dt))
                alpha=max(-.5,min(.5,(command[1]-state.w)/dt))
                before=state
                state=integrate(state,a,alpha,dt)
                remaining-=dt
                sample=now-remaining
                rec.emit('STATE',t=sample-args.start,**asdict(state),a=a,alpha=alpha,
                         clear=swept_clear(before,state,obstacles(args.case,sample-args.start)))
            last=now
            t=now-args.start
            odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link'
            odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.position.x=state.x;odom.pose.pose.position.y=state.y
            odom.pose.pose.orientation.z=math.sin(state.yaw/2);odom.pose.pose.orientation.w=math.cos(state.yaw/2)
            odom.twist.twist.linear.x=state.v;odom.twist.twist.angular.z=state.w
            odom_pub.publish(odom)
            if t>=.5 and t>=next_tf:
                tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
                tf_pub.publish(TFMessage(transforms=[tf]))
                rec.emit('TF',stamp=0)
                next_tf=t+1.
            if t>=1. and t>=next_map:
                resolution=.05 if args.case!='mapchange' or t<17. else .03
                stage=(resolution,tuple(obstacles(args.case,t)))
                if stage!=map_stage:
                    length=8.;n=round(length/resolution);length=n*resolution
                    coords=length/2-(np.arange(n)+.5)*resolution
                    x,y=np.meshgrid(coords,coords,indexing='ij');h=np.zeros((n,n),np.float32)
                    for x0,y0,x1,y1 in stage[1]:h[(x>=x0)&(x<x1)&(y>=y0)&(y<y1)]=2.
                    cached=GridMap();cached.header.frame_id='map';cached.info.resolution=resolution
                    cached.info.length_x=cached.info.length_y=length;cached.info.pose.orientation.w=1.
                    cached.layers=['elevation'];layer=Float32MultiArray()
                    layer.layout.dim=[MultiArrayDimension(label='column_index',size=n,stride=n*n),
                                      MultiArrayDimension(label='row_index',size=n,stride=n)]
                    layer.data=h.ravel(order='F').tolist();cached.data=[layer];map_stage=stage
                cached.header.stamp=node.get_clock().now().to_msg();map_pub.publish(cached)
                rec.emit('MAP',resolution=resolution,obstacles=stage[1]);next_map=t+.5
            if t>=3. and not sent:
                xyz=goal(args.case);msg=PoseStamped();msg.header.frame_id='map'
                msg.header.stamp=node.get_clock().now().to_msg()
                msg.pose.position.x=xyz[0];msg.pose.position.y=xyz[1]
                msg.pose.orientation.z=math.sin(xyz[2]/2);msg.pose.orientation.w=math.cos(xyz[2]/2)
                if path_pub is None:goal_pub.publish(msg)
                rec.emit('GOAL',pose=xyz,injected=path_pub is not None);sent=True
            if args.case=='forward_spin' and t>=38. and not resumed:
                msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
                msg.pose.position.x=2.4;msg.pose.position.y=.7
                msg.pose.orientation.z=math.sin(.3);msg.pose.orientation.w=math.cos(.3)
                goal_pub.publish(msg);rec.emit('RESUME_GOAL',pose=[2.4,.7,.6]);resumed=True
            if path_pub is not None and t>=3. and t>=next_path:
                msg=PathMessage();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
                if t<6. or t>=8.:
                    for index,x in enumerate([0.,.8]):
                        p=PoseStamped();p.header.frame_id='map'
                        ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec+index*50_000_000
                        p.header.stamp.sec=ns//1_000_000_000;p.header.stamp.nanosec=ns%1_000_000_000
                        p.pose.position.x=x;p.pose.orientation.w=1.
                        msg.poses.append(p)
                path_pub.publish(msg);rec.emit('INJECTED_PATH',empty=not msg.poses);next_path=t+.05
            if t>=5. and t>=next_graph:
                types=dict(node.get_topic_names_and_types())
                rec.emit('GRAPH',types=types,publishers={topic:[dict(node_name=p.node_name,topic_type=p.topic_type,qos=profile_data(p.qos_profile),qos_text=str(p.qos_profile))
                    for p in node.get_publishers_info_by_topic(topic)] for topic in types},
                    subscriptions={topic:[dict(node_name=p.node_name,topic_type=p.topic_type,qos=profile_data(p.qos_profile),qos_text=str(p.qos_profile))
                    for p in node.get_subscriptions_info_by_topic(topic)] for topic in types})
                next_graph=t+5.
            rclpy.spin_once(node,timeout_sec=.005)
    except KeyboardInterrupt:pass
    finally:
        rec.emit('EXIT',state=asdict(state),forbidden=[n for n in sys.modules if n.startswith(('nav1cm','lunar_pure_wheeled_controller'))])
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        rec.close()
