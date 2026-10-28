"""真实 GridMap/TF/Goal 生成进程；不读取规划器内部状态。"""
import math
import time

import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage

from .common import TOPICS, Recorder, obstacles, transform


def grid_message(case, elapsed, resolution, folder, label):
    length = 8. if case == 'lifecycle' else 6.
    n = round(length / resolution)
    # grid_map 原生 column-major：buffer index 0 在地图正向上边界。
    coords = length / 2 - (np.arange(n) + .5) * resolution
    xx, yy = np.meshgrid(coords, coords, indexing='ij')
    height = np.zeros((n, n), dtype=np.float32)
    for x0, y0, x1, y1 in obstacles(case, elapsed):
        height[(xx >= x0) & (xx < x1) & (yy >= y0) & (yy < y1)] = 2.
    message = GridMap()
    message.header.frame_id = 'map'
    message.info.resolution = resolution
    message.info.length_x = message.info.length_y = length
    message.info.pose.orientation.w = 1.
    message.layers = ['elevation']
    layer = Float32MultiArray()
    layer.layout.dim = [MultiArrayDimension(label='column_index', size=n, stride=n*n),
                        MultiArrayDimension(label='row_index', size=n, stride=n)]
    layer.data = height.ravel(order='F').tolist()
    message.data = [layer]
    np.savez_compressed(folder / (label + '.npz'), elevation=height, resolution=resolution,
                        length=length, origin_pose=np.array([0., 0., 0.]),
                        outer_start_index=0, inner_start_index=0)
    return message


def run(args):
    rec = Recorder(args.output, 'fixture')
    rclpy.init(args=[])
    node = Node('nav1cm_bounded_fixture')
    pub_map = node.create_publisher(GridMap, TOPICS['grid'], 1)
    pub_goal = node.create_publisher(PoseStamped, TOPICS['goal'], 10)
    static = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tf_topic = '/P4/input/map_to_odom' if args.case == 'legacy_tf' else '/tf_static'
    pub_tf = node.create_publisher(TFMessage, tf_topic, 10 if args.case == 'legacy_tf' else static)
    pub_path = node.create_publisher(PathMessage, TOPICS['local'], 10) if args.case == 'empty_smoke' else None
    sent, cached, next_map, next_tf, next_path = set(), None, 0., 0., 0.
    label = None

    def goal(name, xyz):
        msg = PoseStamped()
        # 故意缺省 frame，故意非单位四元数：生产兼容契约。
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = xyz[:2]
        msg.pose.orientation.z, msg.pose.orientation.w = 3 * math.sin(xyz[2]/2), 3 * math.cos(xyz[2]/2)
        pub_goal.publish(msg)
        rec.emit('GOAL_SENT', name=name, pose=xyz, ros_stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec)
        sent.add(name)

    try:
        while time.monotonic() - args.start < args.duration:
            t = time.monotonic() - args.start
            if t >= 1.5 and t >= next_tf:
                tx, ty, yaw = transform(args.case, t)
                tf = TransformStamped()
                tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
                tf.transform.translation.x, tf.transform.translation.y = tx, ty
                tf.transform.rotation.z, tf.transform.rotation.w = 2*math.sin(yaw/2), 2*math.cos(yaw/2)
                pub_tf.publish(TFMessage(transforms=[tf]))
                rec.emit('TF_SENT', topic=tf_topic, stamp_ns=0, transform=[tx, ty, yaw])
                next_tf = t + .5
            if t >= 2. and t >= next_map and args.case != 'empty_smoke':
                stage = min(4, max(0, int((t - 2.) // 5.))) if args.case == 'contract' else int(t >= 17. and args.case == 'lifecycle')
                resolution = [.2, .05, .03, .01, .005][stage] if args.case == 'contract' else .05
                current = f'map-{stage}-{resolution}'
                if current != label:
                    cached = grid_message(args.case, t, resolution, rec.folder, current)
                    label = current
                cached.header.stamp = node.get_clock().now().to_msg()
                pub_map.publish(cached)
                rec.emit('MAP_SENT', label=label, resolution=resolution, length=cached.info.length_x,
                         obstacles=obstacles(args.case, t), npz=label + '.npz')
                next_map = t + 1.
            if args.case in ('contract', 'legacy_tf') and t >= 3. and 'A' not in sent:
                goal('A', (.75, -.05, .12))
            if args.case == 'lifecycle':
                if t >= 4. and 'C' not in sent:
                    goal('C', (-2.6, 1., 0.))
                if t >= 7. and 'A' not in sent:
                    # 同一个发布者连续 A→无效 B，中间无重发 A。
                    goal('A', (-2.6, -.9, 0.))
                    goal('B-invalid', (1.3, 0., 0.))
                if t >= 11. and 'enclosed' not in sent:
                    goal('enclosed', (0., 0., 0.))
            if args.case in ('forward_spin', 'reverse'):
                if t >= 3. and 'move' not in sent:
                    goal('move', (.6 if args.case == 'forward_spin' else -.6, 0., 0.))
                if args.case == 'forward_spin' and t >= 10. and 'spin' not in sent:
                    goal('spin', (.6, 0., .6))
                if t >= (17. if args.case == 'forward_spin' else 14.) and 'resume' not in sent:
                    # 停发验证前再次真实运动，SIGINT 后依新鲜度受限停车。
                    goal('resume', (.6+.7*math.cos(.6), .7*math.sin(.6), .6) if args.case == 'forward_spin' else (-1.3, 0., 0.))
            if pub_path is not None and t >= 1. and t >= next_path:
                msg = PathMessage()
                msg.header.frame_id = 'map'
                msg.header.stamp = node.get_clock().now().to_msg()
                if t < 3.:
                    for x in (0., .8):
                        pose = PoseStamped()
                        pose.header = msg.header
                        pose.pose.position.x, pose.pose.orientation.w = x, 1.
                        msg.poses.append(pose)
                pub_path.publish(msg)
                rec.emit('SYNTHETIC_STANDARD_PATH', empty=not msg.poses)
                next_path = t + .05
            rclpy.spin_once(node, timeout_sec=.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        rec.emit('EXIT')
        rec.close()
