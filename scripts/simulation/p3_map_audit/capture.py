"""只订阅P3，保存单帧CDR及最接近源时刻的真实定位；不发布ROS消息。"""
import argparse
import hashlib
import json
from pathlib import Path
import time

def main():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from grid_map_msgs.msg import GridMap
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    parser=argparse.ArgumentParser();parser.add_argument('output');parser.add_argument('--duration',type=float,default=14.)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    rclpy.init();node=rclpy.create_node('p3_map_layer_capture')
    odoms=[];maps=[];transforms={};tf_changed=[]
    def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    def tf(m):
        for t in m.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                v=t.transform
                key=tuple(getattr(v.translation,k) for k in 'xyz')+tuple(getattr(v.rotation,k) for k in 'xyzw')
                if transforms and key not in transforms:tf_changed.append(key)
                transforms[key]=t
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(GridMap,'/Car/T3/mapping/grid_map',lambda m:maps.append((m,node.get_clock().now().nanoseconds)),qos)
    node.create_subscription(Odometry,'/Car/T3/localization/odometry',lambda m:odoms.append(m),100)
    node.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    end=time.monotonic()+args.duration
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    if not maps or not odoms or len(transforms)!=1 or tf_changed:raise RuntimeError('missing input or changing map->odom; refusing unaligned replay')
    pairs=[(m,receipt,min(odoms,key=lambda o:abs(ns(o)-ns(m)))) for m,receipt in maps]
    aligned=[v for v in pairs if abs(ns(v[2])-ns(v[0]))<=250000000]
    m,receipt,o=max(aligned or pairs,key=lambda v:ns(v[0]));t=next(iter(transforms.values()))
    tfkey=next(iter(transforms))
    if any(abs(v)>1e-10 for v in tfkey[:6]) or abs(abs(tfkey[6])-1)>1e-10:raise RuntimeError('this audit requires observed fixed identity map->odom')
    meta=dict(map_stamp_ns=ns(m),odom_stamp_ns=ns(o),odom_minus_map_s=(ns(o)-ns(m))/1e9,
        map_receipt_ns=receipt,map_age_at_receipt_s=(receipt-ns(m))/1e9,map_frame=m.header.frame_id,
        odom_frame=o.header.frame_id,odom_child=o.child_frame_id,pose=[o.pose.pose.position.x,o.pose.pose.position.y],
        map_resolution_m=m.info.resolution,map_length_m=[m.info.length_x,m.info.length_y],layers=list(m.layers),tf_identity=list(tfkey),
        odom_samples=len(odoms),odometry_nearest_sample=True,interpolated=False)
    meta['map_samples']=len(maps)
    meta['alignment_ok']=abs(meta['odom_minus_map_s'])<=.25
    for name,value in [('map',m),('odom',o),('tf',TFMessage(transforms=[t]))]:
        data=serialize_message(value);(out/(name+'.cdr')).write_bytes(data);meta[name+'_sha256']=hashlib.sha256(data).hexdigest()
    (out/'capture.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta))
    node.destroy_node();rclpy.shutdown()
    if not meta['alignment_ok']:raise RuntimeError('nearest actual odometry >250ms from map; evidence saved, no synchronized statistics claimed')
if __name__=='__main__':main()
