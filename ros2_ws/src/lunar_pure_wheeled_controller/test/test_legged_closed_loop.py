"""ROS topic loop with an ideal planar plant, not a gait or hardware model."""
import math
from pathlib import Path as FilePath
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from lunar_planning_msgs.msg import PathReference, TrackingStatus
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TFMessage

from lunar_pure_wheeled_controller.node import PureWheeledControllerNode, incremental_path_qos
from test_node import make_path, make_odometry, make_direct_map_from_odom

ROOT = FilePath(__file__).resolve().parents[4]


def test_legged_forward_turn_topic_loop_and_fail_closed_inputs():
    rclpy.init(args=['--ros-args', '--params-file', str(ROOT / 'config/orin_legged_controller.yaml'),
                    '-p', f'platform_config:={ROOT}/config/legged_orin.yaml',
                    '-p', 'command_topic:=/legged_test/cmd',
                    '-p', 'odometry_topic:=/legged_test/odom',
                    '-p', 'tf_topic:=/legged_test/tf',
                    '-p', 'path_reference_topic:=/legged_test/reference',
                    '-p', 'tracking_status_topic:=/legged_test/status'])
    controller = None
    plant = None
    executor = SingleThreadedExecutor()
    try:
        controller = PureWheeledControllerNode()
        # Advance one explicit 50 ms control sample per plant sample. DDS stays real;
        # this avoids wall-clock sleeps without pretending to model robot dynamics.
        for timer in controller.timers:
            timer.cancel()
        plant = rclpy.create_node('legged_test_ideal_plant')
        commands, statuses = [], []
        plant.create_subscription(Twist, '/legged_test/cmd', commands.append, 10)
        plant.create_subscription(TrackingStatus, '/legged_test/status', statuses.append, 10)
        odom_pub = plant.create_publisher(Odometry, '/legged_test/odom', 10)
        tf_pub = plant.create_publisher(TFMessage, '/legged_test/tf', 10)
        reference_pub = plant.create_publisher(PathReference, '/legged_test/reference', incremental_path_qos())
        executor.add_node(controller)
        executor.add_node(plant)

        def wait(predicate):
            deadline = time.monotonic() + 3.
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.001)
            assert predicate()

        wait(lambda: odom_pub.get_subscription_count() == 1 and
             tf_pub.get_subscription_count() == 1 and reference_pub.get_subscription_count() == 1 and
             plant.count_publishers('/legged_test/cmd') == 1)
        reference = PathReference()
        reference.session_id.uuid = [11] * 16
        reference.segment_revision = 1
        reference.reaches_final_goal = True
        reference.path = make_path(frame_id='map', points=[(0., 0., 0.), (.4, 0., 0.), (.4, .4, math.pi)])
        reference_pub.publish(reference)
        wait(lambda: controller._active is not None)
        x = y = yaw = v = w = 0.
        phases = set()
        for step in range(1800):
            odometry = make_odometry(x=x, y=y, yaw=yaw)
            odometry.header.stamp.sec = step + 1
            odometry.twist.twist.linear.x = v
            odometry.twist.twist.angular.z = w
            odom_pub.publish(odometry)
            tf_pub.publish(make_direct_map_from_odom())
            wait(lambda: controller._odometry is not None and
                 controller._odometry.header.stamp.sec == step + 1 and controller._map_from_odom is not None)
            count, feedback_count = len(commands), len(statuses)
            controller._last_control_ns = controller.get_clock().now().nanoseconds - 50_000_000
            controller._tick()
            wait(lambda: len(commands) > count and len(statuses) > feedback_count)
            command, status = commands[-1], statuses[-1]
            assert 0. <= command.linear.x <= .1 + 1e-9
            assert abs(command.angular.z) <= .15 + 1e-9
            assert command.linear.y == command.linear.z == command.angular.x == command.angular.y == 0.
            assert bytes(status.session_id.uuid) == bytes([11] * 16)
            assert status.segment_revision == 1
            assert status.state != TrackingStatus.FAILED, status.reason
            phases.add(status.state)
            v, w = command.linear.x, command.angular.z
            if status.state == TrackingStatus.COMPLETED:
                assert math.hypot(x - .4, y - .4) <= .1
                assert abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))) <= .05
                assert v == w == 0.
                break
            x += v * math.cos(yaw) * .05
            y += v * math.sin(yaw) * .05
            yaw += w * .05
        else:
            pytest.fail(f'closed loop did not complete: {x}, {y}, {yaw}, {statuses[-1]}')
        assert {TrackingStatus.TRACKING, TrackingStatus.ALIGNING, TrackingStatus.FINAL_ALIGN} <= phases

        # A replacement segment reuses the same identity protocol.
        reference.segment_revision = 2
        reference.path = make_path(frame_id='map', points=[(x, y, yaw), (x - .5, y, math.pi)])
        reference_pub.publish(reference)
        wait(lambda: controller._reference_identity[1] == 2)
        for stale in ['_odometry_received_at', '_map_from_odom_received_at']:
            controller._odometry_received_at = controller._map_from_odom_received_at = time.monotonic()
            setattr(controller, stale, time.monotonic() - 10.)
            count, feedback_count = len(commands), len(statuses)
            controller._tick()
            wait(lambda: len(commands) > count)
            assert commands[-1].linear.x == commands[-1].angular.z == 0.
            assert len(statuses) == feedback_count
        reference.state = PathReference.INVALIDATED
        reference_pub.publish(reference)
        wait(lambda: controller._active is None)
        reference.state = PathReference.ACTIVE
        reference_pub.publish(reference)
        # Drain callbacks: invalidated revision cannot reactivate.
        for _ in range(10):
            executor.spin_once(timeout_sec=.001)
        assert controller._active is None
    finally:
        executor.shutdown()
        if controller is not None:
            controller.destroy_node()
        if plant is not None:
            plant.destroy_node()
        rclpy.shutdown()
