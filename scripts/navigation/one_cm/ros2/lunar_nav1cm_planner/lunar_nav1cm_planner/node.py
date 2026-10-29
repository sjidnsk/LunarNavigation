"""P3 GridMap/Odometry/TF/PoseStamped → 两条标准 nav_msgs/Path。"""
from dataclasses import fields
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

from nav1cm.async_coordinator import AsyncNavigationCoordinator
from rclpy.parameter import Parameter
from nav1cm.contracts import WheelLimits
from nav1cm.field_native import NativeDistanceField
from nav1cm.geometry_native import NativeGeometry
from nav1cm.p3_map import P3MapStore
from nav1cm.p3_session import NavigationSession
from nav1cm.ros_conversion import frame_name

from .configuration import load_configuration
from .runtime import PlannerRuntime


def map_input_qos(reliability, durability):
    """地图独立QoS；位姿/TF继续使用原配置。"""
    reliabilities = {"best_effort": ReliabilityPolicy.BEST_EFFORT, "reliable": ReliabilityPolicy.RELIABLE}
    durabilities = {"volatile": DurabilityPolicy.VOLATILE, "transient_local": DurabilityPolicy.TRANSIENT_LOCAL}
    if reliability not in reliabilities or durability not in durabilities:
        raise ValueError("invalid map input QoS")
    return QoSProfile(depth=1, reliability=reliabilities[reliability], durability=durabilities[durability])


def create_resources(limits, capability, settings):
    """内部 factory 可供有界接入验证替换；默认始终使用真实 native/acados。"""
    prefix = Path(get_package_prefix('lunar_nav1cm_planner'))
    terrain = P3MapStore(library_path=prefix / 'lib/libnav1cm_terrain.so', capability=capability)
    try:
        coordinator = AsyncNavigationCoordinator(
            limits=limits, build_dir=settings['build_dir'],
            backend=NativeGeometry(prefix / 'lib/libnav1cm_geometry.so'),
            field_backend=NativeDistanceField(prefix / 'lib/libnav1cm_distance_field.so'),
            allow_unknown=settings['allow_unknown'],
            planning_period_s=settings['planning_period_s'], execution_period_s=settings['execution_period_s'],
            global_period_s=settings['global_period_s'], global_budget_s=settings['global_budget_s'],
            clearance_radius_m=settings['clearance_radius_m'], revalidate_current_map=True,
            planner_options={'n': settings['horizon_steps'], 'dt': settings['horizon_dt_s']})
    except Exception:
        terrain._terrain.close()
        raise
    for key in ('goal_tolerance_m', 'goal_yaw_tolerance_rad', 'stopped_linear_mps', 'stopped_angular_rps'):
        value = float(settings[key])
        if not math.isfinite(value) or value <= 0:
            coordinator.close()
            terrain._terrain.close()
            raise ValueError(f'{key} must be finite and positive')
        setattr(coordinator, key, value)
    return NavigationSession(coordinator), terrain


def path_message(data):
    message = PathMessage()
    message.header.frame_id = data.frame_id
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(data.stamp_ns, 1_000_000_000)
    for point in data.poses:
        pose = PoseStamped()
        pose.header.frame_id = data.frame_id
        pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(point.stamp_ns, 1_000_000_000)
        pose.pose.position.x, pose.pose.position.y = point.x, point.y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(point.yaw / 2), math.cos(point.yaw / 2)
        message.poses.append(pose)
    return message


class PlannerNode(Node):
    def __init__(self, *, resource_factory=None, **kwargs):
        super().__init__('lunar_nav1cm_planner', **kwargs)
        share = Path(get_package_share_directory('lunar_nav1cm_planner'))
        capability_file = self.declare_parameter('wheel_config', str(share / 'config/wheel.yaml')).value
        baseline, capability = load_configuration(capability_file)
        overrides = {field.name: self.declare_parameter(field.name, getattr(baseline, field.name)).value
                     for field in fields(WheelLimits)}
        limits, capability = load_configuration(capability_file, overrides)
        defaults = {
            'map_frame': 'map', 'build_dir': os.environ.get('NAV1CM_BUILD_DIR', ''),
            'allow_unknown': False, 'planning_period_s': .1, 'execution_period_s': .05,
            'global_period_s': 1., 'global_budget_s': 1., 'clearance_radius_m': limits.global_clearance_radius(.9),
            'horizon_steps': 20, 'horizon_dt_s': .2,
            'goal_tolerance_m': .1, 'goal_yaw_tolerance_rad': .05,
            'stopped_linear_mps': .01, 'stopped_angular_rps': .01,
            'map_topic': '/Car/T3/mapping/grid_map',
            'map_qos_reliability': 'best_effort', 'map_qos_durability': 'volatile',
            'odometry_topic': '/Car/T3/localization/odometry',
            'goal_topic': '/Car/T4/navigation/goal',
            'tf_topic': '/tf', 'tf_static_topic': '/tf_static',
            'global_route_topic': '/Car/T4/planning/global_route',
            'local_path_topic': '/Car/T4/planning/local_path',
            'execution_contract_enabled':False,'execution_feedback_timeout_s':2.,
            'execution_reference_topic':'/Car/T4/control/path_execution_reference',
            'tracking_status_topic':'/Car/T4/control/tracking_status',
        }
        settings = {name: self.declare_parameter(name, default).value for name, default in defaults.items()}
        settings['clearance_radius_m']=limits.global_clearance_radius(settings['clearance_radius_m'])
        self.set_parameters([Parameter('minimum_clearance_m',value=limits.minimum_clearance_m),
                             Parameter('clearance_radius_m',value=settings['clearance_radius_m'])])
        if not settings['build_dir']:
            raise ValueError('build_dir 必须显式指定仓库外 acados 构建/缓存目录，或设置 NAV1CM_BUILD_DIR')
        source_qos = map_input_qos(settings['map_qos_reliability'], settings['map_qos_durability'])
        factory = create_resources if resource_factory is None else resource_factory
        session, store = factory(limits, capability, settings)
        self._global_publisher = self.create_publisher(PathMessage, settings['global_route_topic'], 1)
        self._local_publisher = self.create_publisher(PathMessage, settings['local_path_topic'], 1)
        self.runtime = PlannerRuntime(session, store, publish=self._publish,
                                      ros_now_ns=lambda: self.get_clock().now().nanoseconds,
                                      notice=lambda kind, detail: self.get_logger().info(f'{kind}: {detail}'),
                                      map_frame=frame_name(settings['map_frame']) or 'map')
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        dynamic_tf = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        static_tf = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GridMap, settings['map_topic'], self.runtime.offer_map, source_qos)
        self.create_subscription(Odometry, settings['odometry_topic'], self.runtime.offer_odometry, latest)
        self.create_subscription(PoseStamped, settings['goal_topic'], self.runtime.offer_goal, 10)
        self.create_subscription(TFMessage, settings['tf_topic'], self._tf_input, dynamic_tf)
        self.create_subscription(TFMessage, settings['tf_static_topic'], self._tf_input, static_tf)
        self.get_logger().info(f'wheel={capability_file}; limits={limits}; local=0.01m/500x500; build={settings["build_dir"]}')
        self._execution_contract_enabled=settings['execution_contract_enabled']
        self._execution_revision=0
        if self._execution_contract_enabled:
            from lunar_planning_msgs.msg import PathExecutionReference,TrackingStatus
            timeout=float(settings['execution_feedback_timeout_s'])
            self._execution_feedback_timeout_s=timeout
            self.runtime.enable_execution_contract(timeout)
            self._execution_publisher=self.create_publisher(PathExecutionReference,settings['execution_reference_topic'],10)
            self.create_subscription(TrackingStatus,settings['tracking_status_topic'],self._on_execution_status,10)
        self.runtime.start()

    def _tf_input(self, message):
        try:
            self.runtime.offer_tf(message)
        except (ValueError, AttributeError, TypeError) as exc:
            self.get_logger().warning(f'TF_REJECTED: {exc}')

    def _on_execution_status(self,message):
        from nav1cm.execution_ros import status_from_message
        try:
            status=status_from_message(message)
            now=self.get_clock().now().nanoseconds
            if message.header.frame_id!=self.runtime.map_frame or status.stamp_ns>now or now-status.stamp_ns>self._execution_feedback_timeout_s*1e9:return
            self.runtime.offer_tracking_status(status)
        except (ValueError,TypeError,AttributeError,OverflowError):
            self.get_logger().warning('EXECUTION_STATUS_REJECTED')

    def _publish(self, data):
        local=path_message(data.local)
        if getattr(self,'_execution_contract_enabled',False) and self.runtime.execution_context is not None:
            from nav1cm.execution_contract import ExecutionReference,execution_session_id,path_fingerprint
            from nav1cm.execution_ros import reference_message,stamp_ns
            task_id,generation,intent=self.runtime.execution_context
            self._execution_revision+=1
            coordinator=self.runtime.session.coordinator
            reference=ExecutionReference(execution_session_id(task_id,generation),self._execution_revision,
                path_fingerprint(local),intent,stamp_ns(local.header),coordinator.goal_tolerance_m,
                coordinator.goal_yaw_tolerance_rad,coordinator.stopped_linear_mps,coordinator.stopped_angular_rps)
            self.runtime.register_execution_reference(reference)
            self._execution_publisher.publish(reference_message(reference,self.runtime.map_frame))
        self._global_publisher.publish(path_message(data.global_route))
        self._local_publisher.publish(local)

    def stop_runtime(self):
        result = self.runtime.close(timeout_s=2.)
        if result['threads_alive']:
            self.get_logger().error(f'等待处理线程退出，暂不销毁其资源: {result}')
        return result


def main(args=None):
    from nav1cm.ros_lifecycle import sigint_guard
    with sigint_guard() as stopped:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        try:
            node = PlannerNode()
            from nav1cm.ros_lifecycle import spin_until_sigint
            spin_until_sigint(node,stopped)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                # 先停止并唤醒输入线程，退出后再释放 terrain、worker 和 ROS 对象。
                result = node.stop_runtime()
                if result['threads_alive']:
                    raise RuntimeError(f'处理线程尚未退出: {result["threads_alive"]}')
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
