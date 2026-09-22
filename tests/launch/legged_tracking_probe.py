#!/usr/bin/env python3
"""Actual navigator/controller DDS loop against a flat map and ideal planar plant.

Run in a dedicated localhost ROS_DOMAIN_ID with the built overlay sourced.
No real robot or gait is represented. Artifacts must be outside the repository.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from grid_map_msgs.msg import GridMap
from lunar_planning_msgs.action import NavigateToPose
from lunar_planning_msgs.msg import TrackingStatus, PathReference
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from tf2_msgs.msg import TFMessage
import yaml

ROOT = Path(__file__).resolve().parents[2]
PREFIX = '/legged_tracking_probe'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    artifact = args.artifact_dir.resolve()
    if artifact == ROOT or ROOT in artifact.parents:
        raise ValueError('artifacts must be outside the repository')
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise ValueError('set a dedicated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    artifact.mkdir(parents=True, exist_ok=True)
    settings = yaml.safe_load((ROOT / 'config/orin_hardware.yaml').read_text())
    nav = dict(settings['navigation'])
    nav.update(platform_type='legged', platform_config=str(ROOT / 'config/legged_orin.yaml'),
               local_map_topic=PREFIX+'/map', odometry_topic=PREFIX+'/odom', tf_topic=PREFIX+'/tf',
               action_name=PREFIX+'/navigate', path_reference_topic=PREFIX+'/reference',
               tracking_status_topic=PREFIX+'/status', exploration_map_topic=PREFIX+'/exploration_map',
               global_route_topic=PREFIX+'/route', local_path_topic=PREFIX+'/path',
               diagnostics_topic=PREFIX+'/diagnostics', local_fine_map_topic=PREFIX+'/fine',
               local_map_qos_reliability='reliable', local_map_qos_durability='transient_local')
    params = artifact / 'navigation.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': nav}}))
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node('legged_tracking_probe')
    state = dict(x=0., y=0., yaw=0., v=0., w=0., received=0.)
    last_command = dict(v=0., w=0., received=0.)
    statuses, references, results = [], [], []
    maximum = dict(v=0., w=0.)
    try:
        def launch(command, name):
            log = (artifact / name).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True))
        launch(['ros2', 'run', 'lunar_incremental_navigation_ros', 'lunar_incremental_navigation_node',
                '--ros-args', '--params-file', str(params)], 'navigator.log')
        launch(['ros2', 'launch', 'lunar_pure_wheeled_controller', 'pure_legged_controller.launch.py',
                f'config_file:={ROOT}/config/orin_legged_controller.yaml',
                f'platform_config:={ROOT}/config/legged_orin.yaml',
                f'command_topic:={PREFIX}/cmd', f'odometry_topic:={PREFIX}/odom',
                f'tf_topic:={PREFIX}/tf', f'path_reference_topic:={PREFIX}/reference',
                f'tracking_status_topic:={PREFIX}/status'], 'controller.log')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom_pub = node.create_publisher(Odometry, PREFIX+'/odom', 10)
        tf_pub = node.create_publisher(TFMessage, PREFIX+'/tf', 10)
        map_pub = node.create_publisher(GridMap, PREFIX+'/map', qos)
        def command(message):
            assert abs(message.linear.x) <= .100001
            assert abs(message.angular.z) <= .150001
            assert message.linear.y == 0.
            state.update(v=message.linear.x, w=message.angular.z, received=time.monotonic())
            last_command.update(v=message.linear.x, w=message.angular.z, received=time.monotonic())
            maximum['v'] = max(maximum['v'], abs(message.linear.x))
            maximum['w'] = max(maximum['w'], abs(message.angular.z))
        node.create_subscription(Twist, PREFIX+'/cmd', command, 10)
        node.create_subscription(TrackingStatus, PREFIX+'/status', statuses.append, 100)
        node.create_subscription(PathReference, PREFIX+'/reference', references.append, qos)
        client = ActionClient(node, NavigateToPose, PREFIX+'/navigate')
        grid = GridMap()
        grid.header.frame_id = 'odom'
        grid.info.resolution = .2
        grid.info.length_x = grid.info.length_y = 12.
        grid.info.pose.orientation.w = 1.
        grid.layers = grid.basic_layers = ['elevation']
        layer = Float32MultiArray()
        layer.layout.dim = [MultiArrayDimension(label='column_index', size=60, stride=3600),
                            MultiArrayDimension(label='row_index', size=60, stride=60)]
        layer.data = [0.] * 3600
        grid.data = [layer]
        previous = time.monotonic()
        last_map = 0.
        def pump():
            nonlocal previous, last_map
            assert all(p.poll() is None for p in processes), 'child process exited'
            now = time.monotonic()
            dt, previous = min(now - previous, .1), now
            if now - state['received'] > .3:
                state['v'] = state['w'] = 0.
            state['x'] += state['v'] * math.cos(state['yaw']) * dt
            state['y'] += state['v'] * math.sin(state['yaw']) * dt
            state['yaw'] += state['w'] * dt
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.pose.pose.position.x, odom.pose.pose.position.y = state['x'], state['y']
            odom.pose.pose.orientation.z = math.sin(state['yaw']/2)
            odom.pose.pose.orientation.w = math.cos(state['yaw']/2)
            odom.twist.twist.linear.x, odom.twist.twist.angular.z = state['v'], state['w']
            odom_pub.publish(odom)
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
            transform.header.stamp = odom.header.stamp
            transform.transform.rotation.w = 1.
            tf_pub.publish(TFMessage(transforms=[transform]))
            if now-last_map > .5:
                grid.header.stamp = odom.header.stamp
                map_pub.publish(grid)
                last_map = now
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.01)
        def wait(predicate, timeout=45.):
            deadline = time.monotonic()+timeout
            while not predicate() and time.monotonic() < deadline:
                pump()
            assert predicate(), f'timed out at {state}'
        wait(client.server_is_ready, 15.)
        # Give the first finite map time to reach the planning worker.
        startup = time.monotonic()
        wait(lambda: time.monotonic()-startup > 1.)
        for x, y, yaw in [(.6, 0., math.pi/2), (.6, .6, 0.)]:
            future = client.send_goal_async(NavigateToPose.Goal(target_x_m=x, target_y_m=y,
                                           has_target_yaw=True, target_yaw_rad=yaw))
            wait(future.done)
            handle = future.result()
            assert handle.accepted
            done = handle.get_result_async()
            wait(done.done)
            result = done.result().result
            results.append(dict(outcome=result.outcome, reason=result.reason_code,
                                segment_revision=result.last_segment_revision, pose=dict(state)))
            assert result.outcome == NavigateToPose.Result.GOAL_REACHED, str(result)
            assert math.hypot(state['x']-x, state['y']-y) <= .1
            assert abs(math.atan2(math.sin(state['yaw']-yaw), math.cos(state['yaw']-yaw))) <= .05
            assert any(bytes(s.session_id.uuid) == bytes(handle.goal_id.uuid) and
                       s.state == TrackingStatus.COMPLETED and
                       s.segment_revision == result.last_segment_revision for s in statuses)
        # Cancel a live third goal and require navigation acknowledgment plus zero command.
        future = client.send_goal_async(NavigateToPose.Goal(target_x_m=state['x']+1., target_y_m=state['y']))
        wait(future.done)
        handle = future.result()
        assert handle.accepted
        wait(lambda: state['v'] > .01)
        cancel_started = time.monotonic()
        cancel = handle.cancel_goal_async()
        wait(cancel.done)
        assert cancel.result().goals_canceling
        done = handle.get_result_async()
        wait(done.done)
        assert done.result().result.outcome == NavigateToPose.Result.CANCELED
        wait(lambda: last_command['received'] > cancel_started and
             last_command['v'] == last_command['w'] == 0.)
        report = dict(results=results, canceled=True, maximum_command=maximum,
                      status_samples=len(statuses), references=len(references),
                      boundary='localhost Jazzy DDS, flat synthetic map, ideal planar plant; no real driver')
        (artifact/'result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        trace = dict(pose=state, results=results,
                     statuses=[dict(session=[int(v) for v in s.session_id.uuid], revision=s.segment_revision,
                                    state=s.state, reason=s.reason, progress=s.progress_m,
                                    cross_track=s.cross_track_m, heading=s.heading_error_rad,
                                    v=s.linear_speed_mps, w=s.angular_speed_radps) for s in statuses[-20:]],
                     references=[dict(session=[int(v) for v in r.session_id.uuid], revision=r.segment_revision,
                                      state=r.state, final=r.reaches_final_goal,
                                      points=[dict(x=p.pose.position.x, y=p.pose.position.y,
                                                   yaw=2*math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                                              for p in r.path.poses]) for r in references])
        (artifact/'trace.json').write_text(json.dumps(trace, indent=2)+'\n')
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
