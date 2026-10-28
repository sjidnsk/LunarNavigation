"""独立 Path 消费者/运动模型：Path 是唯一控制输入；Goal 仅供离线判据。"""
import csv
import math
import sys
import time

from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.node import Node

from common import TOPICS, Recorder, obstacles, path_data, transform, yaw_of
from motion import PathFollower, State, swept_clear


def run(args):
    rec = Recorder(args.output, 'consumer')
    rclpy.init(args=[])
    node = Node('nav1cm_bounded_path_consumer')
    follower = PathFollower()
    rec.emit('CONTROL_CONTRACT', source='Path geometry only; independent retiming',
             limits=dict(v=.2, w=1., a=.5, alpha=.5, curvature=1., body=[1.182, .818]),
             freshness_s=follower.freshness_s, integration_dt_max_s=.02)
    moving = args.case in ('forward_spin', 'reverse', 'empty_smoke')
    state = State(x=-2.6) if args.case == 'lifecycle' else State()
    odom_pub = node.create_publisher(Odometry, TOPICS['odom'], 1)

    def receive(msg, kind):
        rec.emit('PATH_RECEIVED', stream=kind, message=path_data(msg))
        if kind == 'local':
            follower.offer([(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in msg.poses], time.monotonic())

    node.create_subscription(PathMessage, TOPICS['local'], lambda msg: receive(msg, 'local'), 100)
    node.create_subscription(PathMessage, TOPICS['global_route'], lambda msg: receive(msg, 'global'), 100)
    stream = (rec.folder / 'state.csv').open('x', newline='', encoding='utf-8')
    csvout = csv.writer(stream)
    csvout.writerow(['monotonic_s', 'elapsed_s', 'x', 'y', 'yaw', 'v', 'w', 'a', 'alpha', 'mode', 'path_age_s', 'collision_free'])
    last, next_tick = time.monotonic(), time.monotonic()
    graph_recorded = False
    try:
        while time.monotonic() - args.start < args.duration:
            rclpy.spin_once(node, timeout_sec=.001)
            now = time.monotonic()
            if now < next_tick:
                continue
            dt, previous = min(.02, now-last), state
            # 物理时钟的全部间隔逐步覆盖，不跳过 scheduler 延迟。
            remaining = now-last
            a = alpha = 0.
            mode, collision_free = 'static', True
            while remaining > 1e-10:
                dt = min(.02, remaining)
                if moving:
                    state, a, alpha, mode = follower.step(state, now-remaining, dt)
                collision_free &= swept_clear(previous, state, obstacles(args.case, now-args.start))
                assert abs(a) <= .5+1e-9 and abs(alpha) <= .5+1e-9
                assert abs(state.v) <= .2+1e-8 and abs(state.w) <= 1.+1e-8
                assert abs(state.w) <= abs(state.v)+1e-8 or abs(state.v) < 1e-8, 'curvature exceeded'
                assert collision_free, 'independent SAT sweep collision'
                previous = state
                remaining -= dt
            t = now-args.start
            if t>=4.5 and not graph_recorded:
                types = dict(node.get_topic_names_and_types())
                rec.emit('GRAPH', types=types, publishers={topic:[dict(node_name=p.node_name, topic_type=p.topic_type)
                           for p in node.get_publishers_info_by_topic(topic)] for topic in types})
                graph_recorded = True
            csvout.writerow([now, t, state.x, state.y, state.yaw, state.v, state.w, a, alpha, mode,
                             now-follower.received_s if math.isfinite(follower.received_s) else 1e9, collision_free])
            stream.flush()
            tf_x, tf_y, tf_yaw = transform(args.case, t)
            # 静态合同测试刻意让 raw odom 为原点，以核对非恒等零 stamp TF。
            if args.case in ('contract', 'legacy_tf'):
                ox = oy = oyaw = 0.
            else:
                dx, dy = state.x-tf_x, state.y-tf_y
                ox, oy = math.cos(tf_yaw)*dx+math.sin(tf_yaw)*dy, -math.sin(tf_yaw)*dx+math.cos(tf_yaw)*dy
                oyaw = state.yaw-tf_yaw
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = ox, oy
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = 3*math.sin(oyaw/2), 3*math.cos(oyaw/2)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = state.v, state.w
            odom_pub.publish(odom)
            last, next_tick = now, now+.01
    except KeyboardInterrupt:
        pass
    finally:
        rec.emit('EXIT', state=state, forbidden_modules=[n for n in sys.modules if n.startswith('nav1cm') or 'verification_plant' in n])
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
        rec.close()
