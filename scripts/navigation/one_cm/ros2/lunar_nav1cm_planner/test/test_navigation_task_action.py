"""真实 Humble Action 客户端；隔离 worker，但使用真实任务/会话/Runtime。"""
from importlib import util
from pathlib import Path
import sys
import time
import math
import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
sys.path.insert(0, str(Path(__file__).parents[3]))


def test_action_client_goal_feedback_cancel_replace_and_shutdown():
    assert util.find_spec('lunar_nav1cm_planner.task_node') is not None, '可选任务Action入口尚未实现'
    rclpy = pytest.importorskip('rclpy')
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.node import Node
    from lunar_planning_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from lunar_nav1cm_planner.task_node import NavigationTaskNode
    from test_runtime import build_runtime

    rclpy.init()
    executor = SingleThreadedExecutor()
    def factory(*args):
        _, session, _, _ = build_runtime()
        return session, None
    prefix = '/nav1cm_task1_test'
    node = NavigationTaskNode(resource_factory=factory, parameter_overrides=[
        Parameter('build_dir', value='/tmp/nav1cm-unused-test'),
        Parameter('action_name', value=prefix+'/navigate'),
        Parameter('odometry_topic', value=prefix+'/odom'),
        Parameter('goal_topic', value=prefix+'/goal'),
        Parameter('local_path_topic', value=prefix+'/local'),
        Parameter('global_route_topic', value=prefix+'/global')])
    client_node = Node('nav1cm_task1_client')
    executor.add_node(node); executor.add_node(client_node)
    client = ActionClient(client_node, NavigateToPose, prefix+'/navigate')
    odom = client_node.create_publisher(Odometry, prefix+'/odom', 10)
    manual = client_node.create_publisher(PoseStamped, prefix+'/goal', 10)
    observations = []
    measured = [0., 0., 0., 0., 0.]
    def publish_state():
        message = Odometry(); message.header.frame_id = 'map'; message.child_frame_id = 'base_link'
        message.pose.pose.position.x, message.pose.pose.position.y = measured[:2]
        message.pose.pose.orientation.z = math.sin(measured[2]/2)
        message.pose.pose.orientation.w = math.cos(measured[2]/2)
        message.twist.twist.linear.x = measured[3]; message.twist.twist.angular.z = measured[4]
        odom.publish(message)
    timer = client_node.create_timer(.03, publish_state)
    def until(predicate, timeout=5.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        assert predicate(), 'Action客户端有界等待超时'
    def send(x, y, yaw=0., has_yaw=True):
        goal = NavigateToPose.Goal(target_x_m=float(x), target_y_m=float(y),
                                   target_yaw_rad=float(yaw), has_target_yaw=has_yaw)
        future = client.send_goal_async(goal, feedback_callback=lambda msg: observations.append(msg.feedback))
        until(future.done)
        handle = future.result(); assert handle.accepted
        return handle, handle.get_result_async()
    try:
        assert client.wait_for_server(timeout_sec=3.)
        active, result = send(0, 3)
        until(lambda: len(observations)>0)
        bad, bad_result = send(2.6, 0)
        until(bad_result.done)
        assert bad_result.result().result.outcome == NavigateToPose.Result.INVALID_GOAL
        assert bad_result.result().status == 6
        assert not result.done()
        replacement, replacement_result = send(0, 3)
        until(result.done)
        assert result.result().result.reason_code == 'GOAL_REPLACED'
        assert result.result().status == 6
        late = active.cancel_goal_async(); until(late.done)
        assert not replacement_result.done()
        measured[3] = .2
        until(lambda: any(e.reason_code == 'WAITING_FOR_PATH' for e in observations))
        for _ in range(10): executor.spin_once(timeout_sec=.02)
        cancel = replacement.cancel_goal_async(); until(cancel.done)
        for _ in range(10): executor.spin_once(timeout_sec=.02)
        assert not replacement_result.done()
        measured[3] = 0.
        until(replacement_result.done)
        assert replacement_result.result().result.reason_code == 'CANCELED_STOPPED'
        assert replacement_result.result().status == 5
        turning, turning_result = send(0, 0, 1.)
        for _ in range(20): executor.spin_once(timeout_sec=.02)
        assert not turning_result.done()
        measured[2] = 1.; measured[3] = .1
        for _ in range(20): executor.spin_once(timeout_sec=.02)
        assert not turning_result.done()
        measured[3] = 0.
        until(turning_result.done)
        assert turning_result.result().result.outcome == NavigateToPose.Result.GOAL_REACHED
        assert turning_result.result().status == 4
        missing, missing_result = send(0, 0, has_yaw=False)
        until(missing_result.done)
        assert missing_result.result().result.outcome == NavigateToPose.Result.GOAL_REACHED
        managed, managed_result = send(0, 3)
        pose = PoseStamped(); pose.header.frame_id='map'; pose.pose.position.y=-3.; pose.pose.orientation.w=1.
        manual.publish(pose)
        until(managed_result.done)
        assert managed_result.result().result.reason_code == 'GOAL_REPLACED'
        last, last_result = send(0, 3)
        node.task_api.close()
        until(last_result.done)
        assert last_result.result().result.reason_code == 'SHUTDOWN'
        assert not node.task_api.pending
    finally:
        node.task_api.close()
        result = node.stop_runtime()
        assert not result['threads_alive']
        client.destroy(); node.destroy_node(); client_node.destroy_node()
        executor.shutdown(); rclpy.shutdown()
