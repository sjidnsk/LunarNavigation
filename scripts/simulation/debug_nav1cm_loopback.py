"""有界TCP回环+OBJ+共享nav1cm调试；仅写显式artifact，不连接外部UE。"""
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
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import yaml
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path as RosPath
from lunar_planning_msgs.action import NavigateToPose
from lunar_planning_msgs.msg import PathExecutionReference, TrackingStatus
from lunar_obj_tcp_sim.local_vehicle import LocalVehicle
from lunar_obj_tcp_sim.prepare import prepare_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=120.)
    args = parser.parse_args()
    artifact = args.artifact.resolve()
    artifact.mkdir(parents=True, exist_ok=False)
    obj = artifact/'flat.obj'
    obj.write_text('v -10 -10 0\nv 10 -10 0\nv 10 10 0\nv -10 10 0\nf 1 2 3\nf 1 3 4\n')
    prepare_map(obj, artifact/'map', center=(0., 0.), size=12., resolution=.2, halo=2., scale=1., axes='x,y,z')
    vehicle = LocalVehicle((0., 0., 0.))
    prefix = '/p4_tcp_debug'
    cfg = yaml.safe_load((args.source/'ros2_ws/src/lunar_obj_tcp_sim/config/simulation.yaml').read_text())
    cfg['simulation'].update(host='127.0.0.1', port=vehicle.port, prefix=prefix,
        map_directory=str(artifact/'map'), mode='nav', auto_start=False,
        start_rviz=False, start_local_rviz=False, initial_scan=False)
    config = artifact/'simulation.yaml'
    config.write_text(yaml.safe_dump(cfg))
    log = (artifact/'launch.log').open('w')
    command = ['ros2', 'launch', 'lunar_obj_tcp_sim', 'obj_tcp.launch.py',
               f'config:={config}', f'platform_config:={args.source}/config/wheel.yaml',
               f'build_dir:={os.environ["NAV1CM_BUILD_DIR"]}']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('p4_tcp_debug_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    feedback = dict(odometry=None, references=[], statuses=[], routes=0, diagnostics=[])
    state_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Odometry, prefix+'/odometry', lambda m: feedback.update(odometry=m), state_qos)
    node.create_subscription(PathExecutionReference, prefix+'/execution_reference', feedback['references'].append, 100)
    node.create_subscription(TrackingStatus, prefix+'/tracking_status', feedback['statuses'].append, 100)
    node.create_subscription(RosPath, prefix+'/global_route', lambda m: feedback.update(routes=feedback['routes']+bool(m.poses)), 100)
    node.create_subscription(String, prefix+'/control/mppi_diagnostics',
        lambda m: feedback['diagnostics'].append(json.loads(m.data)), 100)
    client = ActionClient(node, NavigateToPose, prefix+'/navigate_to_pose')
    handle = None
    result = {'ok': False, 'scope': 'ARM Humble real acados/MPPI + OBJ + loopback TCP kinematics; no external UE physics'}

    def wait(predicate, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError('ROS launch exited; inspect launch.log')
            if vehicle.errors:
                raise RuntimeError(str(vehicle.errors))
        raise TimeoutError('bounded loopback wait expired')

    try:
        wait(lambda: client.server_is_ready() and feedback['odometry'] is not None, args.duration)
        # 等待真实输入地图处理，不以固定睡眠替代就绪检查。
        from lunar_planning_msgs.srv import GetPolicyMap
        maps = node.create_client(GetPolicyMap, prefix+'/get_policy_map')
        wait(maps.service_is_ready, 30.)
        deadline = time.monotonic()+30.
        while time.monotonic() < deadline:
            future = maps.call_async(GetPolicyMap.Request())
            wait(future.done, 10.)
            if future.result().ready:
                break
        else:
            raise RuntimeError('policy map never ready')
        # 地图 owner 就绪不能替代独立跟踪消费者的就绪证据。
        wait(lambda: any(d.get('event') == 'solve_result' for d in feedback['diagnostics']), 30.)
        goal = NavigateToPose.Goal(target_x_m=1., target_y_m=.2,
            has_target_yaw=True, target_yaw_rad=.4)
        sent = client.send_goal_async(goal)
        wait(sent.done, 10.)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('goal rejected')
        done = handle.get_result_async()
        wait(done.done, args.duration)
        outcome = done.result().result
        wait(lambda: abs(vehicle.v) <= .01 and abs(vehicle.w) <= .01, 10.)
        error = math.hypot(vehicle.position[0]-1., vehicle.position[1]-.2)
        yaw_error = abs(math.atan2(math.sin(vehicle.yaw-.4), math.cos(vehicle.yaw-.4)))
        session = bytes(handle.goal_id.uuid)
        accepted = [s for s in feedback['statuses'] if bytes(s.session_id.uuid)==session and s.reference_accepted]
        bound = [r for r in feedback['references'] if bytes(r.session_id.uuid)==session]
        result.update(outcome=int(outcome.outcome), reason=outcome.reason_code,
            position_error_m=error, yaw_error_rad=yaw_error, velocity=[vehicle.v, vehicle.w],
            references=len(bound), accepted_statuses=len(accepted), route_messages=feedback['routes'],
            tcp_commands=len(vehicle.commands), pose=list(vehicle.position)+[vehicle.yaw])
        result['ok'] = bool(outcome.outcome==NavigateToPose.Result.GOAL_REACHED and error <= .02
            and yaw_error <= .05 and bound and accepted and feedback['routes'] and not vehicle.errors)
    except BaseException as error:
        result['error'] = repr(error)
    finally:
        if handle is not None:
            try:
                cancel = handle.cancel_goal_async()
                wait(cancel.done, 5.)
            except Exception:
                pass
        # 保持TCP反馈在线直至控制进程退出，便于执行停稳。
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10.)
                result['forced_shutdown'] = True
        result['diagnostic_events'] = feedback['diagnostics']
        result['launch_returncode'] = process.returncode
        result['vehicle_errors'] = list(vehicle.errors)
        result['stopped_at_exit'] = abs(vehicle.v)<=.01 and abs(vehicle.w)<=.01
        result['ok'] = bool(result.get('ok') and not result.get('forced_shutdown')
            and process.returncode == 0 and not result['vehicle_errors'] and result['stopped_at_exit'])
        vehicle.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        log.close()
        (artifact/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
