"""真实ROS调度：接收不等待控制计算，旧输入也不重新标时。"""
import time
from threading import Event
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode


def test_feedback_is_received_while_control_group_is_blocked(monkeypatch):
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path'])
    node=PureWheeledControllerNode();peer=rclpy.create_node('feedback_ingress_probe')
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node);executor.add_node(peer)
    entered=Event();release=Event()
    def block():
        entered.set();release.wait(2.)
    monkeypatch.setattr(node,'_tick',block)
    publisher=peer.create_publisher(Odometry,node.get_parameter('odometry_topic').value,10)
    message=Odometry();message.header.frame_id='map';message.pose.pose.orientation.w=1.
    try:
        end=time.monotonic()+3.
        while not entered.is_set() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        assert entered.is_set()
        end=time.monotonic()+1.
        while time.monotonic()<end:
            message.header.stamp=peer.get_clock().now().to_msg();publisher.publish(message)
            executor.spin_once(timeout_sec=.02)
            if getattr(node,'_feedback_pending',None) is not None:break
        pending=getattr(node,'_feedback_pending',None)
        assert pending is not None, 'feedback callback blocked by control calculation'
        assert node._odometry is None, 'receive callback mutated executor input state'
        received=pending[1]
        release.set()
        node._drain_standard_feedback()
        assert node._odometry_received_at==received
        assert node._standard_state_ns==int(pending[0].header.stamp.sec)*10**9+pending[0].header.stamp.nanosec
        time.sleep(.01)
        node._drain_standard_feedback()
        assert node._odometry_received_at==received, 'draining restamped old feedback'
    finally:
        release.set();executor.shutdown();node.destroy_node();peer.destroy_node();rclpy.shutdown()


def test_tf_only_update_does_not_refresh_old_odometry_receipt_or_source():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path'])
    node=PureWheeledControllerNode()
    try:
        odom=Odometry();odom.header.frame_id='odom';odom.header.stamp.sec=7
        odom.pose.pose.orientation.w=1.
        node._receive_odometry(odom)
        pending=node._feedback_pending
        node._drain_standard_feedback()
        assert node._odometry is None
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='odom'
        transform.header.stamp.sec=8;transform.transform.rotation.w=1.
        node._receive_tf(TFMessage(transforms=[transform]));node._drain_standard_feedback()
        assert node._odometry_received_at==pending[1]
        assert node._standard_state_ns==7_000_000_000
        node._receive_tf(TFMessage(transforms=[transform]));node._drain_standard_feedback()
        assert node._odometry_received_at==pending[1] and node._standard_state_ns==7_000_000_000
    finally:node.destroy_node();rclpy.shutdown()
