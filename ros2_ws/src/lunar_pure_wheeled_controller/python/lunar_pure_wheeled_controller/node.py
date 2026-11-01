"""ROS adapter for the isolated pure wheeled path tracker."""

from __future__ import annotations

import json
from math import atan2, isfinite, sqrt
from dataclasses import fields
from threading import Lock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from collections import deque
from pathlib import Path as FilePath
import yaml
from time import monotonic

from geometry_msgs.msg import Twist
from lunar_planning_msgs.msg import MotionReference, PathReference, TrackingStatus
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from rcl_interfaces.msg import SetParametersResult

from .frame import MapFromOdom, map_tracking_state, parse_map_from_odom
from .incremental_path import ParsedIncrementalPath, parse_incremental_path
from .reference import ParsedReference, parse_reference
from .tracking import TrackingPolicy, TrackingState, track_trajectory
from .execution import PathExecutor
from .motion_configuration import p3_map_receipt_expired,input_disposition


def map_input_qos(reliability, durability):
    """地图独立QoS；位姿/TF继续使用原配置。"""
    reliabilities = {"best_effort": ReliabilityPolicy.BEST_EFFORT, "reliable": ReliabilityPolicy.RELIABLE}
    durabilities = {"volatile": DurabilityPolicy.VOLATILE, "transient_local": DurabilityPolicy.TRANSIENT_LOCAL}
    if reliability not in reliabilities or durability not in durabilities:
        raise ValueError("invalid map input QoS")
    return QoSProfile(depth=1, reliability=reliabilities[reliability], durability=durabilities[durability])


def _yaw(odometry: Odometry) -> float | None:
    position = odometry.pose.pose.position
    orientation = odometry.pose.pose.orientation
    if not all(
        isfinite(value)
        for value in (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
    ):
        return None
    quaternion_norm = sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if not isfinite(quaternion_norm) or quaternion_norm == 0.0:
        return None
    x = orientation.x / quaternion_norm
    y = orientation.y / quaternion_norm
    z = orientation.z / quaternion_norm
    w = orientation.w / quaternion_norm
    yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw if isfinite(yaw) else None


def incremental_path_qos() -> QoSProfile:
    """Match the incremental navigator's durable local-path contract."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def state_input_qos() -> QoSProfile:
    """Accept the navigator's best-effort odometry and TF input contract."""
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class PureWheeledControllerNode(Node):
    """Publishes bounded wheel commands for the latest valid reference."""

    def __init__(self) -> None:
        super().__init__("lunar_pure_wheeled_controller")
        defaults = {
            "input_mode": "motion_reference",
            "controller_backend": "pure_pursuit",
            "policy_map_service": "/Car/T4/mapping/get_policy_map",
            "mppi_batch_size": 32,
            "mppi_time_steps": 40,
            "mppi_iterations": 1,
            "mppi_temperature": 0.15,
            "mppi_seed": 17,
            "mppi_diagnostics_topic": "/Car/T4/control/mppi_diagnostics",
            "policy_map_timeout_s": 0.5,
            "standard_map_source": "direct_p3",
            "p3_map_topic": "/Car/T3/mapping/grid_map",
            "map_qos_reliability": "best_effort", "map_qos_durability": "volatile",
            "p3_map_timeout_s": 3.0,
            "tf_static_topic": "/tf_static",
            "allow_unknown": False,
            "body_length_m": 0.0,
            "body_width_m": 0.0,
            "minimum_clearance_m": 0.0,
            "mppi_compute_budget_s": 0.04,
            "mppi_compute_envelope_s": 0.05,
            "mppi_nominal_response_time_s": TrackingPolicy().control_response_time_s,
            "mppi_trace_diagnostics": False,
            "execution_contract_enabled":False,
            "execution_reference_topic":"/Car/T4/control/path_execution_reference",
            "mppi_simulation_step_sync": False,
            "mppi_step_ack_topic": "/Car/T4/control/mppi_step_ack",
            "enable_rolling_updates": True,
            "map_frame": "map", "odom_frame": "odom", "base_frame": "base_link",
            "platform_config": "",
            "path_reference_topic": "/Car/T4/planning/path_reference",
            "tracking_status_topic": "/Car/T4/control/tracking_status",
            "reference_topic": "/Car/T4/planning/wheeled_reference",
            "path_topic": "/Car/T4/planning/local_path",
            "odometry_topic": "/Car/T3/localization/odometry",
            "command_topic": "/Car/T5/Car_Cmd_Vel",
            "execution_cancel_topic": "/Car/T4/execution/cancel",
            "tf_topic": "/tf",
            "control_rate_hz": 20.0,
            "odometry_timeout_s": 0.5,
            "tf_timeout_s": 0.5,
            "lookahead_m": 0.5,
            "max_linear_mps": 0.2,
            "max_angular_radps": 0.5,
            "goal_position_tolerance_m": 0.2,
            "goal_yaw_tolerance_rad": 0.2,
            "max_cross_track_error_m": 1.0,
            "spin_kp": 1.5,
            "translation_epsilon_m": 1.0e-3,
        }
        for field in fields(TrackingPolicy):
            defaults.setdefault(field.name, getattr(TrackingPolicy(), field.name))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        input_mode = self.get_parameter("input_mode").value
        if input_mode not in {
            "motion_reference",
            "incremental_path",
            "incremental_reference",
        }:
            raise ValueError(
                "input_mode must be motion_reference, incremental_path or incremental_reference"
            )
        self._input_mode = input_mode
        self._backend = self.get_parameter("controller_backend").value
        self._is_mppi = self._backend in {"mppi", "mppi_path_follower"}
        self._mppi_step_sync = self.get_parameter("mppi_simulation_step_sync").value
        if self._mppi_step_sync and (not self._is_mppi or not self.get_parameter("use_sim_time").value):
            raise ValueError("mppi_simulation_step_sync requires MPPI and use_sim_time")
        self._mppi_publication_sequence = 0
        self._mppi_published_command = [0.,0.]
        if self._backend not in {"pure_pursuit", "mppi", "mppi_path_follower"}:
            raise ValueError("controller_backend must be pure_pursuit, mppi or mppi_path_follower")
        self._standard_mode = self._backend == "mppi_path_follower" and input_mode == "incremental_path"
        self._execution_enabled=self.get_parameter('execution_contract_enabled').value
        if type(self._execution_enabled) is not bool or (self._execution_enabled and not self._standard_mode):
            raise ValueError('execution contract requires standard Path MPPI')
        if self._is_mppi and input_mode != "incremental_reference" and not self._standard_mode:
            raise ValueError("mppi requires incremental_reference")
        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        frame_names = (self._map_frame, self._odom_frame, self._base_frame)
        if any(not isinstance(v, str) or not v.strip() for v in frame_names) or len(set(frame_names)) != 3:
            raise ValueError("map_frame, odom_frame, base_frame must be nonempty and distinct")
        topic_parameters = ["odometry_topic", "command_topic"]
        if self._input_mode == "motion_reference":
            topic_parameters.extend(["reference_topic", "execution_cancel_topic"])
        else:
            topic_parameters.extend(["path_topic", "tf_topic"])
            if self._input_mode == "incremental_reference":
                topic_parameters.extend(
                    ["path_reference_topic", "tracking_status_topic"]
                )
        if self._execution_enabled:
            topic_parameters.extend(['execution_reference_topic','tracking_status_topic'])
        topics = {
            name: self._absolute_topic_name(name, self.get_parameter(name).value)
            for name in topic_parameters
        }
        policy_values = {
            field.name: self.get_parameter(field.name).value
            for field in fields(TrackingPolicy)
        }
        platform_path = self.get_parameter("platform_config").value
        self._p3_source=None
        p3_setup=None
        self._standard_pending_odometry=None
        if self._standard_mode:
            from nav1cm.ros_conversion import LatestTransforms
            self._latest_transforms=LatestTransforms()
            if not platform_path:
                from ament_index_python.packages import get_package_share_directory
                platform_path=str(FilePath(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml')
        capability = {}
        if platform_path:
            capability = yaml.safe_load(
                FilePath(platform_path).read_text(encoding="utf-8")
            )["capability"]
            names = dict(
                max_linear_mps="maximum_forward_speed_mps",
                max_reverse_mps="maximum_reverse_speed_mps",
                max_angular_radps="maximum_spin_rate_radps",
                max_linear_accel_mps2="maximum_acceleration_mps2",
                max_linear_decel_mps2="maximum_braking_deceleration_mps2",
                max_angular_accel_radps2="maximum_yaw_acceleration_radps2",
                max_curvature_per_m="maximum_curvature_per_m",
                max_lateral_accel_mps2="maximum_lateral_acceleration_mps2",
            )
            for key, source in names.items():
                policy_values[key] = min(
                    float(policy_values[key]), float(capability[source])
                )
        if self._standard_mode:
            for name in ("max_linear_mps", "max_reverse_mps", "max_angular_radps"):
                if name not in self._parameter_overrides:
                    policy_values[name] = float(capability[names[name]])
            for name,value in dict(goal_position_tolerance_m=.1,goal_yaw_tolerance_rad=.08726646259971647,stopped_angular_radps=.01).items():
                if name not in self._parameter_overrides:
                    policy_values[name]=min(value,float(capability[names[name]])) if platform_path and name in names else value
        from lunar_pure_wheeled_controller.motion_configuration import resolve_motion_model
        policy_values['motion_model']=resolve_motion_model(capability,policy_values['motion_model'],
            standard_path=self._standard_mode,explicit='motion_model' in self._parameter_overrides)
        self._policy = TrackingPolicy(**policy_values)
        if self._standard_mode:
            from nav1cm.contracts import resolve_minimum_clearance
            from rclpy.parameter import Parameter
            self._minimum_clearance_m=resolve_minimum_clearance(capability.get('minimum_clearance_m',0.0),
                self.get_parameter('minimum_clearance_m').value)
            self.set_parameters([Parameter('minimum_clearance_m',value=self._minimum_clearance_m),
                *[Parameter(name,value=getattr(self._policy,name)) for name in
                  ('max_linear_mps','max_reverse_mps','max_angular_radps')]])
            dimensions=capability['body_extent_m'][:2]
            self._body_dimensions=tuple(float(self.get_parameter(name).value) or float(default)
                for name,default in zip(('body_length_m','body_width_m'),dimensions))
            if any(not isfinite(v) or v<=0 for v in self._body_dimensions):
                raise ValueError('body dimensions must be finite and positive')
            self._standard_map_source=self.get_parameter('standard_map_source').value
            if self._standard_map_source not in ('direct_p3','policy_service'):
                raise ValueError('standard_map_source must be direct_p3 or policy_service')
        if self._is_mppi:
            self._mppi_trace_enabled = self.get_parameter("mppi_trace_diagnostics").value
            from .mppi import MppiExecutor, MppiPolicy
            from .mppi_map import PolicyMapCache
            from lunar_planning_msgs.srv import GetPolicyMap
            from .mppi_path_follower import MppiPathFollower
            executor_type=MppiPathFollower if self._backend=="mppi_path_follower" else MppiExecutor
            if self._standard_mode:
                from .standard_path_follower import StandardPathFollower
                executor_type=StandardPathFollower
            self._executor = executor_type(self._policy,MppiPolicy(
                batch_size=self.get_parameter("mppi_batch_size").value,
                time_steps=self.get_parameter("mppi_time_steps").value,
                iterations=self.get_parameter("mppi_iterations").value,
                temperature=self.get_parameter("mppi_temperature").value,
                seed=self.get_parameter("mppi_seed").value,
                nominal_response_time_s=self.get_parameter("mppi_nominal_response_time_s").value,
                compute_budget_s=float(self.get_parameter('mppi_compute_envelope_s').value),
                state_age_budget_s=None if self._standard_mode else .05))
            self._mppi_diagnostics = self.create_publisher(String,self._absolute_topic_name("mppi_diagnostics_topic",self.get_parameter("mppi_diagnostics_topic").value),10)
            self._map_client = None
            if self._standard_mode and self._standard_map_source=='direct_p3':
                self._map_timeout_s=float(self.get_parameter('p3_map_timeout_s').value)
                if not isfinite(self._map_timeout_s) or self._map_timeout_s<0:
                    raise ValueError('p3_map_timeout_s must be finite and nonnegative')
                p3_setup=(capability,self._body_dimensions,
                    self._absolute_topic_name('p3_map_topic',self.get_parameter('p3_map_topic').value))
            elif self._executor.requires_map:
                self._map_cache = PolicyMapCache()
                self._map_type = GetPolicyMap
                service = self._absolute_topic_name("policy_map_service", self.get_parameter("policy_map_service").value)
                self._map_client = self.create_client(GetPolicyMap, service)
                self._map_future = None
                self._map_received_at = None
                self._map_requested_at = None
                self._map_timeout_s = float(self.get_parameter("policy_map_timeout_s").value)
                if not isfinite(self._map_timeout_s) or self._map_timeout_s <= 0:
                    raise ValueError("policy_map_timeout_s must be finite and positive")
                self._required_map_revision = 0
                self._map_processed_ns = None
            self._mppi_budget_s = float(self.get_parameter("mppi_compute_budget_s").value)
            if not isfinite(self._mppi_budget_s) or self._mppi_budget_s <= 0:
                raise ValueError("mppi_compute_budget_s must be positive and finite (advisory only)")
            if self._map_client is not None:
                self.create_timer(0.1, self._request_policy_map)
        else:
            self._executor = PathExecutor(
                self._policy, rolling_updates=(input_mode == "incremental_reference"
                    and self.get_parameter("enable_rolling_updates").value))
        self._reference_identity = None
        self._retired_sessions = deque(maxlen=64)
        self._invalidated_revision = -1
        self._path_sequence = 0
        self._last_control_ns = None
        self._tracking_status = (
            self.create_publisher(TrackingStatus, topics["tracking_status_topic"], 10)
            if self._input_mode == "incremental_reference" or self._execution_enabled
            else None
        )
        if self._execution_enabled:
            from lunar_planning_msgs.msg import PathExecutionReference
            from .execution_pairing import ExecutionInbox
            self._execution_inbox=ExecutionInbox(max_age_s=None)
            self._execution_offered=None

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if not isfinite(control_rate_hz) or control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be finite and greater than zero")
        self._odometry_timeout_s = float(self.get_parameter("odometry_timeout_s").value)
        self._tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        if not isfinite(self._odometry_timeout_s) or self._odometry_timeout_s <= 0.0:
            raise ValueError("odometry_timeout_s must be finite and greater than zero")
        if not isfinite(self._tf_timeout_s) or self._tf_timeout_s <= 0.0:
            raise ValueError("tf_timeout_s must be finite and greater than zero")
        self._path_received_at=None
        self._active: ParsedReference | ParsedIncrementalPath | None = None
        self._trajectory_cursor = 0
        self._odometry: Odometry | None = None
        self._odometry_received_at: float | None = None
        self._odometry_advanced_at=None
        self._standard_source_high_water=None
        self._mppi_tf_history = deque(maxlen=16)
        self._mppi_odom_history = deque(maxlen=16)
        self._map_from_odom: MapFromOdom | None = None
        self._map_from_odom_received_at: float | None = None
        # 接收组只更新输入快照；默认组独占所有执行器/命令状态。
        self._feedback_lock=Lock()
        self._feedback_group=MutuallyExclusiveCallbackGroup() if self._standard_mode else None
        self._feedback_pending=None
        self._feedback_transforms=None
        self._execution_inputs=deque()
        self._execution_input_capacity=128
        self._execution_ingress_overflow=False
        if self._execution_enabled:
            self.create_subscription(PathExecutionReference,topics['execution_reference_topic'],
                self._receive_execution_reference,10,callback_group=self._feedback_group)
        if self._standard_mode:self._feedback_tf_cache=self._latest_transforms.copy()
        self._commands = self.create_publisher(Twist, topics["command_topic"], 10)
        self.create_subscription(
            Odometry,
            topics["odometry_topic"],
            self._receive_odometry if self._standard_mode else self._on_odometry,
            state_input_qos(), callback_group=self._feedback_group,
        )
        if self._input_mode == "motion_reference":
            self.create_subscription(
                MotionReference,
                topics["reference_topic"],
                self._on_reference,
                10,
            )
            self.create_subscription(
                String,
                topics["execution_cancel_topic"],
                self._on_cancel,
                10,
            )
        else:
            if self._input_mode == "incremental_reference":
                self.create_subscription(
                    PathReference,
                    topics["path_reference_topic"],
                    self._on_path_reference,
                    incremental_path_qos(),
                )
            else:
                self.create_subscription(
                    Path, topics["path_topic"], self._receive_execution_path if self._standard_mode else self._on_path,
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE) if self._standard_mode else incremental_path_qos(),
                    callback_group=self._feedback_group
                )
            self.create_subscription(
                TFMessage,
                topics["tf_topic"],
                self._receive_tf if self._standard_mode else self._on_tf,
                state_input_qos(), callback_group=self._feedback_group,
            )
        if self._standard_mode:
            self.create_subscription(TFMessage,self._absolute_topic_name('tf_static_topic',
                self.get_parameter('tf_static_topic').value),self._receive_tf,
                QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL),
                callback_group=self._feedback_group)
        self.create_timer(1.0 / control_rate_hz, self._control_timer_tick)
        if self._mppi_step_sync:
            ack_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._mppi_step_ack=self.create_publisher(String,
                self._absolute_topic_name("mppi_step_ack_topic",self.get_parameter("mppi_step_ack_topic").value),ack_qos)
            # Late discovery must still receive a zero-command handshake even
            # when the timer was created after this paused simulation stamp.
            self._publish_twist()
            self._publish_mppi_step(self.get_clock().now().nanoseconds)

        # 所有参数/话题、发布器与定时器构造成功后再启动持有 native 资源的 worker。
        if p3_setup is not None:
            source_qos=map_input_qos(self.get_parameter("map_qos_reliability").value,
                                     self.get_parameter("map_qos_durability").value)
            from .p3_map_source import create_p3_source
            from grid_map_msgs.msg import GridMap as P3GridMap
            capability,body,topic=p3_setup
            self._p3_source=create_p3_source(capability,body,minimum_clearance_m=self._minimum_clearance_m,
                allow_unknown=self.get_parameter('allow_unknown').value,map_frame=self._map_frame)
            try:
                self.create_subscription(P3GridMap,topic,self._p3_source.submit_map,source_qos,callback_group=self._feedback_group)
            except Exception:
                self._p3_source.close()
                raise
        self.add_on_set_parameters_callback(self._validate_startup_parameters)

    def _validate_startup_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != "use_sim_time" and self.has_parameter(parameter.name) and parameter.value != self.get_parameter(parameter.name).value:
                return SetParametersResult(successful=False, reason=f"{parameter.name}: startup-only parameter; edit configuration and restart node")
        return SetParametersResult(successful=True)

    @staticmethod
    def _absolute_topic_name(parameter_name: str, value: object) -> str:
        if not isinstance(value, str) or len(value) < 2 or not value.startswith("/"):
            raise ValueError(f"{parameter_name} must be an absolute topic name")
        return value

    def _on_reference(self, reference: MotionReference) -> None:
        parsed = parse_reference(reference)
        if parsed.reason is not None:
            self._clear_active_and_stop()
            return
        self._active = parsed
        self._trajectory_cursor = 0
        if not parsed.trajectory_samples:
            self._path_sequence += 1
            self._executor.set_path(
                parsed.path_xy_yaw, identity=(parsed.plan_id, self._path_sequence)
            )

    def _on_cancel(self, message: String) -> None:
        if (
            self._input_mode == "motion_reference"
            and isinstance(self._active, ParsedReference)
            and message.data == self._active.plan_id
        ):
            self._clear_active_and_stop()

    def _receive_execution_input(self, kind, message):
        # 接收组仅保存有界、有序快照；执行器仍由控制组单写入。
        received=monotonic()
        with self._feedback_lock:
            if self._execution_ingress_overflow:return False
            if len(self._execution_inputs)>=self._execution_input_capacity:
                self._execution_ingress_overflow=True
                return False
            self._execution_inputs.append((kind,message,received))
        return True

    def _receive_execution_path(self, message):
        return self._receive_execution_input('path',message)

    def _receive_execution_reference(self, message):
        return self._receive_execution_input('reference',message)

    def _drain_standard_execution(self):
        with self._feedback_lock:
            if self._execution_ingress_overflow:return False
            pending=list(self._execution_inputs)
            self._execution_inputs.clear()
        for kind,message,received in pending:
            if kind=='path':self._on_path(message,received_at=received)
            else:self._on_execution_reference(message,received_at=received)
        return True

    def _on_path(self, path: Path, *, received_at=None) -> None:
        parsed = parse_incremental_path(path, self._map_frame)
        if self._standard_mode:
            if self._execution_enabled:
                from nav1cm.execution_contract import path_fingerprint
                from nav1cm.execution_ros import stamp_ns
                if parsed.reason is not None:
                    self.request_controlled_stop(parsed.reason);return
                self._execution_inbox.add_path(stamp_ns(path.header),path_fingerprint(path),parsed,
                    monotonic() if received_at is None else received_at)
                self._try_execution_pair()
                return
            if parsed.clear or parsed.reason is not None:
                self.request_controlled_stop(parsed.reason or "EMPTY_PATH")
                return
            self._path_received_at=monotonic() if received_at is None else received_at
            self._active=parsed
            self._path_sequence+=1
            self._executor.set_standard_path(parsed.path_xy_yaw,identity=self._path_sequence)
            return
        if parsed.clear or parsed.reason is not None:
            self._clear_active_and_stop()
            return
        self._active = parsed
        self._path_sequence += 1
        self._executor.set_path(
            parsed.path_xy_yaw, identity=("plain", self._path_sequence)
        )

    def _on_execution_reference(self,message,*,received_at=None):
        from nav1cm.execution_ros import reference_from_message
        try:
            reference=reference_from_message(message)
            now=self.get_clock().now().nanoseconds
            if message.header.frame_id!=self._map_frame or reference.stamp_ns>now:
                return
            if not self._execution_inbox.add_reference(reference,monotonic() if received_at is None else received_at):return
            actual=self._executor.execution_reference
            if (actual is None or actual.session_id!=reference.session_id or reference.intent in (1,2,4)
                    or (reference.intent==3 and actual.intent!=3)):
                # 安全停车意图可先执行；几何接管仍必须等严格配对。
                self.request_controlled_stop('EXECUTION_PAIR_WAIT')
            self._try_execution_pair()
        except (ValueError,TypeError,AttributeError,OverflowError):
            self.request_controlled_stop('INVALID_EXECUTION_REFERENCE')

    def _try_execution_pair(self):
        matched=self._execution_inbox.pop_ready(monotonic())
        if matched is None:return
        reference,parsed,received=matched
        self._execution_offered=reference
        if not self._executor.offer_execution_reference(parsed.path_xy_yaw,reference):
            self.request_controlled_stop('EXECUTION_REFERENCE_REJECTED')
            self._publish_execution_status(reference=reference,state_code=6,accepted=False,reason='EXECUTION_REFERENCE_REJECTED')
            return
        self._path_received_at=received;self._active=parsed;self._path_sequence=reference.revision

    def _publish_execution_status(self,*,reference=None,state_code=None,accepted=None,reason=None):
        if not getattr(self,'_execution_enabled',False):return
        if reference is None:
            waiting=(self._executor.failure is None and
                     self._executor._stop_reason in ('WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE'))
            reference=(self._execution_offered if waiting else None) or self._executor.execution_reference or self._execution_offered
            if waiting and accepted is None:accepted=False
        if reference is None:return
        status=TrackingStatus();status.header.frame_id=self._map_frame
        status.header.stamp=self.get_clock().now().to_msg()
        status.session_id.uuid=list(bytes.fromhex(reference.session_id))
        status.segment_revision=reference.revision;status.reference_digest=reference.path_digest
        status.reference_intent=reference.intent
        status.reference_accepted=(self._executor.reference_accepted and self._executor.execution_reference==reference) if accepted is None else accepted
        status.state=int(self._executor.last_execution_state if state_code is None else state_code)
        status.reason=(self._executor.failure or self._executor._stop_reason or '') if reason is None else reason
        state=getattr(self,'_standard_state',None)
        if state is not None:
            status.linear_speed_mps=state.linear_mps;status.angular_speed_radps=state.angular_radps
        self._tracking_status.publish(status)

    def request_controlled_stop(self, reason="STOP_REQUESTED") -> None:
        if self._standard_mode:
            self._executor.request_stop(reason)
        else:
            self._clear_active_and_stop()

    def _on_path_reference(self, reference: PathReference) -> None:
        identity = (bytes(reference.session_id.uuid), int(reference.segment_revision))
        current = self._reference_identity
        if identity[0] in self._retired_sessions:
            return
        if (
            current is not None
            and identity[0] == current[0]
            and identity[1] < current[1]
        ):
            return
        if reference.state == PathReference.INVALIDATED:
            if (
                current is None
                or identity[0] == current[0]
                and identity[1] >= current[1]
            ):
                self._reference_identity = identity
                self._invalidated_revision = identity[1]
                self._clear_active_and_stop()
            return
        if reference.state != PathReference.ACTIVE:
            self._clear_active_and_stop()
            return
        if (
            current is not None
            and identity[0] == current[0]
            and identity[1] <= self._invalidated_revision
        ):
            return
        parsed = parse_incremental_path(reference.path, self._map_frame)
        if parsed.clear or parsed.reason is not None or reference.segment_revision == 0:
            self._clear_active_and_stop()
            return
        if self._executor.set_path(
            parsed.path_xy_yaw, identity=identity, final=reference.reaches_final_goal
        ):
            if current is not None and current[0] != identity[0]:
                self._retired_sessions.append(current[0])
                self._invalidated_revision = -1
            self._reference_identity = identity
            self._active = parsed
            if self._is_mppi:
                self._required_map_revision = int(reference.traversability_revision)

    def _request_policy_map(self):
        if self._map_future is not None:
            if monotonic() - self._map_requested_at > self._map_timeout_s:
                self._map_client.remove_pending_request(self._map_future)
                self._map_future.cancel()
                self._map_future = None
            else:
                return
        if not self._map_client.service_is_ready():
            return
        request = self._map_type.Request()
        request.since_revision = self._map_cache.revision
        # Native READY and reference revision determine map authority.
        # Do not demand a newer observation than the sensor has produced.
        request.minimum_map_stamp_ns = 0
        self._map_requested_at = monotonic()
        self._map_future = self._map_client.call_async(request)
        self._map_future.add_done_callback(self._on_policy_map)

    def _on_policy_map(self, future):
        if future is not self._map_future:
            return
        self._map_future = None
        try:
            response = future.result()
            if response.frame_id != self._map_frame:
                raise ValueError("policy map frame mismatch")
            processed_ns=int(response.processed_stamp.sec)*1_000_000_000+int(response.processed_stamp.nanosec)
            if not 0<=processed_ns<=self.get_clock().now().nanoseconds:
                raise ValueError("policy map processing stamp is invalid")
            self._map_cache.apply(response)
            self._map_processed_ns = int(response.processed_stamp.sec)*1_000_000_000+int(response.processed_stamp.nanosec)
            self._map_received_at = monotonic()
        except Exception as exc:
            # A delta gap/epoch error must force a full refresh, not authorize
            # movement against a stale cache.
            from .mppi_map import PolicyMapCache
            self._map_cache = PolicyMapCache()
            self._map_received_at = None
            self.get_logger().warning(f"MPPI map unavailable: {exc}")

    def _receive_odometry(self, message):
        # 时间只在实际接收时记录一次，消费快照不重新标时。
        pending=(message,monotonic(),self.get_clock().now().nanoseconds)
        with self._feedback_lock:self._feedback_pending=pending

    def _receive_tf(self, message):
        try:
            transforms=self._feedback_tf_cache.copy()
            transforms.update(message)
        except (ValueError,AttributeError,TypeError) as exc:
            self.get_logger().warning(f'标准 TF 无效: {exc}')
            return
        self._feedback_tf_cache=transforms
        with self._feedback_lock:self._feedback_transforms=transforms

    def _drain_standard_feedback(self):
        with self._feedback_lock:
            pending,self._feedback_pending=self._feedback_pending,None
            transforms,self._feedback_transforms=self._feedback_transforms,None
        if transforms is not None:
            self._latest_transforms=transforms
            if self._p3_source is not None:self._p3_source.set_transforms(transforms)
        if pending is not None:self._standard_pending_odometry=pending
        if transforms is not None or pending is not None:self._accept_standard_odometry()

    def _on_tf(self, message: TFMessage) -> None:
        if self._standard_mode:
            try:
                transforms=self._latest_transforms.copy()
                transforms.update(message)
                self._latest_transforms=transforms
                if self._p3_source is not None:self._p3_source.set_transforms(transforms)
                self._accept_standard_odometry()
            except (ValueError,AttributeError,TypeError) as exc:
                self.get_logger().warning(f'标准 TF 无效: {exc}')
            return
        update = parse_map_from_odom(message, self._map_frame, self._odom_frame)
        if update.found:
            if self._is_mppi and update.transform is not None:
                matches=[t for t in message.transforms if t.header.frame_id==self._map_frame and t.child_frame_id==self._odom_frame]
                stamp=matches[-1].header.stamp
                self._mppi_tf_history.append((int(stamp.sec)*1_000_000_000+int(stamp.nanosec),update.transform))
            self._map_from_odom = update.transform
            self._map_from_odom_received_at = (
                monotonic() if update.transform is not None else None
            )
            if update.transform is None:
                self._mppi_tf_history.clear()
                self._publish_twist()

    def _on_odometry(self, odometry: Odometry) -> None:
        if self._standard_mode:
            self._standard_pending_odometry=(odometry,monotonic(),self.get_clock().now().nanoseconds)
            self._accept_standard_odometry()
            return
        invalid_twist = self._is_mppi and not all(isfinite(value) for value in (
            odometry.twist.twist.linear.x, odometry.twist.twist.linear.y,
            odometry.twist.twist.angular.z))
        if invalid_twist or _yaw(odometry) is None or (self._input_mode != "motion_reference" and (odometry.header.frame_id != self._odom_frame or odometry.child_frame_id != self._base_frame)):
            self._odometry = None
            self._odometry_received_at = None
            self._mppi_odom_history.clear()
            self._publish_twist()
            return
        self._odometry = odometry
        self._odometry_received_at = monotonic()
        if self._is_mppi:
            stamp=odometry.header.stamp
            self._mppi_odom_history.append((int(stamp.sec)*1_000_000_000+int(stamp.nanosec),odometry,self._odometry_received_at))

    def _accept_standard_odometry(self):
        if self._standard_pending_odometry is None:return
        from nav1cm.ros_conversion import state_from_odometry, MissingTransform
        odometry,received,receipt_ns=self._standard_pending_odometry
        try:
            # Odometry twist 位于 child_frame；保留同帧横向反馈，不随地图 TF 旋转。
            lateral_mps=float(odometry.twist.twist.linear.y)
            if not isfinite(lateral_mps):raise ValueError('lateral feedback must be finite')
            value=state_from_odometry(odometry,self._latest_transforms,self._map_frame)
        except MissingTransform:
            return
        except (ValueError,AttributeError,TypeError):
            self._odometry=None;self._odometry_received_at=None
            self._standard_pending_odometry=None
            # 非有限/损坏测量是完整性故障，不能降格成普通断流等待。
            self._executor.failure=self._executor.failure or 'INVALID_FEEDBACK'
            return
        stamp=odometry.header.stamp
        source_ns=int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
        high=getattr(self,'_standard_source_high_water',None)
        if source_ns<=0 or source_ns>receipt_ns or (high is not None and source_ns<high):
            self._executor.failure=self._executor.failure or 'INVALID_FEEDBACK_TIMESTAMP'
            self._odometry=None;self._odometry_received_at=None
            self._standard_pending_odometry=None
            return
        if high is None or source_ns>high:
            self._standard_source_high_water=source_ns
            self._odometry_advanced_at=received
        self._standard_state=TrackingState(value.x,value.y,value.yaw,value.v,value.w,lateral_mps)
        self._odometry=odometry;self._odometry_received_at=received
        self._standard_state_ns=source_ns
        if self._p3_source is not None:self._p3_source.set_center(value.x,value.y)

    def _standard_pose_wait(self,reason='WAITING_FOR_POSE'):
        if getattr(self._executor,'failure',None) is not None:
            return self._standard_unavailable(self._executor.failure)
        reference=getattr(self,'_execution_offered',None) or self._executor.execution_reference
        self._executor.wait_for_pose()
        if self._executor._stop_reason in ('WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE'):
            self._executor._stop_reason=reason
        self._active=None;self._path_received_at=None;self._last_control_ns=None
        self._mppi_diagnostics.publish(String(data=json.dumps(dict(event='input_wait',
            reason=reason,measured_stopped=None,certified_stop=False))))
        self._publish_twist()
        self._publish_execution_status(reference=reference,state_code=0,accepted=False,reason=reason)

    def _standard_unavailable(self,reason,*,retryable=False):
        failure=getattr(self._executor,'failure',None)
        disposition=input_disposition(reason,
            hard_failure=failure is not None or getattr(self._executor,'last_execution_state',0)==6)
        if disposition in ('WAITING_FOR_POSE','WAITING_FOR_MAP'):
            return self._standard_pose_wait(disposition)
        retryable=disposition=='RECOMPUTE'
        if failure is not None:reason=failure
        # 零目标是未知运动时的保底请求，不能当作有界制动认证或实测停车。
        # 可重算失效仅作废本次发布证明；nominal 只供下轮完整重算热启动。
        if not retryable:self._executor.discard_prediction()
        self._mppi_diagnostics.publish(String(data=json.dumps(dict(event='certificate_unavailable',
            reason=reason,retryable=retryable,measured_stopped=None,certified_stop=False))))
        self._publish_twist()
        # 仅证明在发布前作废可重算；输入失效/不安全仍是终端故障。
        # 输入拒绝归属当前收到的参考；真正执行故障仍归属实际执行身份。
        # 否则等待新参考时持续重发旧未接管参考故障，只会被导航正确拒收。
        rejected=(getattr(self,'_execution_offered',None)
                  if failure is None and not retryable else None)
        identity={} if rejected is None else dict(reference=rejected,accepted=False)
        self._publish_execution_status(state_code=1 if retryable else 6,
            reason=('PUBLICATION_DEFERRED:' if retryable else 'EXECUTION_INPUT_UNAVAILABLE:')+str(reason),**identity)

    def _standard_grid(self,state):
        if self._p3_source is not None:
            receipt=self._p3_source.snapshot()
            if receipt is None or p3_map_receipt_expired(receipt.received_at,self._map_timeout_s,monotonic()):return None
            return receipt.grid
        if self._input_is_stale(self._map_received_at,self._map_timeout_s):return None
        return self._map_cache.local_grid(state.x_m,state.y_m,2.5)

    def _tick_standard_path(self):
        # 消费同一快照的推进接收时间；新消息不能给本轮旧计算续期。
        received_at=getattr(self,'_odometry_advanced_at',self._odometry_received_at)
        if self._odometry is None or self._input_is_stale(received_at,self._odometry_timeout_s):
            return self._standard_unavailable('missing_or_stale_feedback')
        state=self._standard_state
        proof_source=getattr(self,'_p3_source',None)
        proof_cache=getattr(self,'_map_cache',None)
        grid=self._standard_grid(state)
        if grid is None:return self._standard_unavailable('missing_or_stale_map')
        now_ns=self.get_clock().now().nanoseconds
        source_ns=self._standard_state_ns
        if now_ns<source_ns:return self._standard_unavailable('future_feedback')
        if self._active is None and getattr(self._executor,'_stop_reason',None) in (
                'WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE'):
            return self._standard_pose_wait('WAITING_FOR_FRESH_REFERENCE')
        dt=.05 if self._last_control_ns is None else (now_ns-self._last_control_ns)/1e9
        self._last_control_ns=now_ns
        if dt<=0:return self._standard_unavailable('nonpositive_dt')
        if self._active is None:
            self._executor.request_stop('NO_PATH')
        self._executor.set_map(grid)
        started=monotonic()
        result=self._executor.update(state,dt,now_s=source_ns/1e9,state_age_s=(now_ns-source_ns)/1e9)
        if self._input_is_stale(received_at,self._odometry_timeout_s):
            return self._standard_unavailable('stale_feedback_after_optimization')
        after_solve_ns=self.get_clock().now().nanoseconds
        if after_solve_ns<now_ns:return self._standard_unavailable('clock_regressed_during_optimization')
        # 每周期最多追加一次证明，截止时刻从证明开始计算，绝不追赶自身耗时。
        covered_ns=now_ns+int(self._executor.config.compute_budget_s*1e9)
        latest=self._standard_grid(state)
        if latest is None:return self._standard_unavailable('stale_map_after_optimization')
        if self._input_is_stale(received_at,self._odometry_timeout_s):
            return self._standard_unavailable('stale_feedback_during_recertification')
        # direct P3 的 epoch 仅在同一 source 生命周期内表示不可变物理世界；
        # revision/origin 是窗口投影。policy cache 的 revision 仍承载内容更新。
        proof_grid=grid
        identity=lambda value:(value.epoch if proof_source is not None else
                               (value.revision,value.epoch,value.origin))
        def same_source():
            return (getattr(self,'_p3_source',None) is proof_source and
                    getattr(self,'_map_cache',None) is proof_cache)
        if not same_source():
            return self._standard_unavailable('map_changed_during_certification',retryable=True)
        certificate_ns=self.get_clock().now().nanoseconds
        world_changed=identity(latest)!=identity(proof_grid)
        if world_changed or certificate_ns>covered_ns:
            if world_changed:proof_grid=latest
            # 仅时间续期仍完整认证原快照；出窗/UNKNOWN 由既有认证拒绝。
            self._executor.set_map(proof_grid)
            if not self._executor.certify_publication(state,result.command.linear_x_mps,
                    result.command.angular_z_radps,(certificate_ns-source_ns)/1e9):
                return self._standard_unavailable('latest_map_or_aged_motion_unsafe')
            covered_ns=certificate_ns+int(self._executor.config.compute_budget_s*1e9)
        final_grid=self._standard_grid(state)
        publication_ns=self.get_clock().now().nanoseconds
        if final_grid is None:
            return self._standard_unavailable('stale_map_after_certification')
        if self._input_is_stale(received_at,self._odometry_timeout_s):
            return self._standard_unavailable('stale_feedback_after_certification')
        if publication_ns<certificate_ns:
            return self._standard_unavailable('clock_regressed_during_certification')
        if not same_source() or identity(final_grid)!=identity(proof_grid):
            return self._standard_unavailable('map_changed_during_certification',retryable=True)
        if publication_ns>covered_ns:
            return self._standard_unavailable('publication_certificate_expired',retryable=True)
        with self._feedback_lock:
            if self._execution_ingress_overflow:
                return self._standard_unavailable('execution_ingress_overflow')
            self._publish_twist(result.command.linear_x_mps,result.command.angular_z_radps)
        self._publish_execution_status()
        self._mppi_diagnostics.publish(String(data=json.dumps(dict(self._executor.diagnostics,
            event='solve_result',phase=result.phase,compute_ms=(monotonic()-started)*1000,
            map_revision=proof_grid.revision,measured_stopped=self._executor._stopped(state)))))

    def _publish_mppi_step(self, stamp_ns):
        self._mppi_step_ack.publish(String(data=json.dumps(dict(event="control_step",
            now_ns=stamp_ns,command=self._mppi_published_command))))

    def _control_timer_tick(self):
        if not self._mppi_step_sync:
            return self._tick()
        stamp_ns=self.get_clock().now().nanoseconds
        before=self._mppi_publication_sequence
        self._tick()
        # An exception or a tick which published nothing cannot acknowledge an
        # old command. Every ordinary early-stop path publishes its zero first.
        if self._mppi_publication_sequence==before:
            raise RuntimeError("simulation control tick produced no command")
        self._publish_mppi_step(stamp_ns)

    def _tick(self) -> None:
        if self._standard_mode:
            self._drain_standard_feedback()
            if not self._drain_standard_execution():
                return self._standard_unavailable('execution_ingress_overflow')
            return self._tick_standard_path()
        if self._is_mppi:
            self._mppi_observe("tick")
        if (
            self._active is None
            or self._odometry is None
            or self._input_is_stale(
                self._odometry_received_at,
                self._odometry_timeout_s,
            )
        ):
            if self._is_mppi:
                self._mppi_observe("stop", reason="missing_or_stale_input")
            self._publish_twist()
            return
        if self._input_mode in {"incremental_path", "incremental_reference"}:
            self._tick_incremental_path()
            return
        self._tick_motion_reference()

    def _tick_incremental_path(self) -> None:
        if (
            not isinstance(self._active, ParsedIncrementalPath)
            or self._odometry is None
            or self._map_from_odom is None
            or self._input_is_stale(
                self._map_from_odom_received_at,
                self._tf_timeout_s,
            )
        ):
            if self._is_mppi:
                self._mppi_observe("stop", reason="missing_or_stale_transform")
            self._publish_twist()
            return
        transform = self._map_from_odom
        odometry = self._odometry
        if self._is_mppi:
            now_ns=self.get_clock().now().nanoseconds
            pairs={ns:t for ns,t in self._mppi_tf_history}
            # Source age is measured in the ROS state clock. Receipt time is a
            # separate communication-liveness guard, not another 50 ms source
            # age check in wall time (simulation may pause or change rate).
            candidates=[(ns,o,pairs[ns]) for ns,o,received in self._mppi_odom_history
                        if ns in pairs and 0<=now_ns-ns<=self._executor.config.state_age_budget_s*1e9
                        and not self._input_is_stale(received,self._odometry_timeout_s)]
            if not candidates:
                self._mppi_observe("stop",reason="no_fresh_coherent_state")
                self._publish_twist()
                return
            state_stamp_ns,odometry,transform=max(candidates,key=lambda pair:pair[0])
        state = map_tracking_state(odometry, transform)
        if state is None:
            self._odometry = None
            self._mppi_odom_history.clear()
            self._publish_twist()
            return
        now_ns = self.get_clock().now().nanoseconds
        dt = (
            0.05
            if self._last_control_ns is None
            else (now_ns - self._last_control_ns) / 1e9
        )
        self._last_control_ns = now_ns
        if dt <= 0:
            if self._is_mppi:
                self._mppi_observe("stop", reason="nonpositive_dt", dt=dt)
            self._publish_twist()
            return
        if self._is_mppi and self._executor.requires_map:
            if (self._input_is_stale(self._map_received_at, self._map_timeout_s)
                    or self._map_cache.revision < self._required_map_revision
                    or self._map_processed_ns is None):
                self._mppi_observe("stop", reason="stale_or_old_map")
                self._publish_twist()
                return
            radius = max(self._policy.max_linear_mps, self._policy.max_reverse_mps) * 4.0 + 1.0
            self._executor.set_map(self._map_cache.local_grid(state.x_m, state.y_m, radius))
        if self._is_mppi:
            if self._mppi_trace_enabled:
                self._mppi_observe("solve_input", state=vars(state), dt=dt,
                                   source_ns=state_stamp_ns, source_age_s=(now_ns-state_stamp_ns)/1e9,
                                   history=[(t,u.tolist()) for t,u in self._executor._command_history[-12:]])
            started = monotonic()
            result = self._executor.update(state, dt, now_s=state_stamp_ns/1e9,state_age_s=(now_ns-state_stamp_ns)/1e9)
            elapsed=monotonic()-started
            self._mppi_diagnostics.publish(String(data=json.dumps(dict(
                self._executor.diagnostics,event="solve_result",now_ns=now_ns,
                command=[result.command.linear_x_mps,result.command.angular_z_radps],compute_ms=elapsed*1000.,
                deadline_missed=elapsed>self._mppi_budget_s,
                map_revision=self._map_cache.revision if self._executor.requires_map else None,phase=result.phase))))
            # Compute budget is telemetry, not execution authority. Actual
            # state validity remains independent of the performance target.
            if (self._input_is_stale(self._odometry_received_at,self._odometry_timeout_s)
                    or self._input_is_stale(self._map_from_odom_received_at,self._tf_timeout_s)):
                self._executor.discard_prediction()
                self._mppi_observe("stop", reason="stale_state_after_optimization", elapsed_s=elapsed)
                self._publish_twist()
                return
            # The initial certificate covers source age plus its modeled
            # compute envelope. Extend it using ROS physical time if needed;
            # a paused simulation does not age merely because solving is slow.
            covered_ns=now_ns+int(self._executor.config.compute_budget_s*1e9)
            while self._executor.requires_map and self.get_clock().now().nanoseconds>covered_ns:
                if (self._input_is_stale(self._odometry_received_at,self._odometry_timeout_s)
                        or self._input_is_stale(self._map_from_odom_received_at,self._tf_timeout_s)):
                    self._executor.discard_prediction()
                    self._mppi_observe("stop",reason="stale_state_during_recertification")
                    self._publish_twist()
                    return
                certificate_ns=self.get_clock().now().nanoseconds
                if not self._executor.certify_publication(
                        state,result.command.linear_x_mps,result.command.angular_z_radps,
                        (certificate_ns-state_stamp_ns)/1e9):
                    self._executor.discard_prediction()
                    self._mppi_observe("stop",reason="aged_motion_not_safe")
                    self._publish_twist()
                    return
                covered_ns=certificate_ns+int(self._executor.config.compute_budget_s*1e9)
                self._mppi_observe("recertified",source_age_s=(certificate_ns-state_stamp_ns)/1e9)
        else:
            result = self._executor.update(state, dt)
        command = result.command
        self._publish_twist(command.linear_x_mps, command.angular_z_radps)
        if self._tracking_status is not None and self._reference_identity is not None:
            message = TrackingStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._map_frame
            message.session_id.uuid = list(self._reference_identity[0])
            message.segment_revision = self._reference_identity[1]
            message.state = getattr(TrackingStatus, result.phase)
            message.direction = result.direction
            message.progress_m, message.cross_track_m = (
                result.progress_m,
                result.cross_track_m,
            )
            message.heading_error_rad = result.heading_error_rad
            message.linear_speed_mps, message.angular_speed_radps = (
                state.linear_mps,
                state.angular_radps,
            )
            message.reason = command.failure_reason or result.phase
            self._tracking_status.publish(message)
        # Hold the terminal executor until a new reference arrives. Repeated
        # terminal feedback is safe because navigation deduplicates revisions.

    @staticmethod
    def _input_is_stale(received_at: float | None, timeout_s: float) -> bool:
        return received_at is None or monotonic() - received_at > timeout_s

    def _tick_motion_reference(self) -> None:
        if not isinstance(self._active, ParsedReference) or self._odometry is None:
            self._publish_twist()
            return
        yaw = _yaw(self._odometry)
        if yaw is None:
            self._odometry = None
            self._publish_twist()
            return
        position = self._odometry.pose.pose.position
        state = TrackingState(
            position.x,
            position.y,
            yaw,
            self._odometry.twist.twist.linear.x,
            self._odometry.twist.twist.angular.z,
        )
        if self._active.trajectory_samples:
            result = track_trajectory(
                self._active.trajectory_samples,
                state,
                self._policy,
                self._trajectory_cursor,
            )
            self._trajectory_cursor = result.next_cursor
            command = result.command
        else:
            now_ns = self.get_clock().now().nanoseconds
            dt = (
                0.05
                if self._last_control_ns is None
                else (now_ns - self._last_control_ns) / 1e9
            )
            self._last_control_ns = now_ns
            if dt <= 0:
                self._publish_twist()
                return
            command = self._executor.update(state, dt).command
        self._publish_twist(command.linear_x_mps, command.angular_z_radps)
        if command.complete or command.failure_reason is not None:
            self._clear_active()

    def _clear_active(self) -> None:
        self._executor.clear()
        self._active = None
        self._trajectory_cursor = 0

    def _clear_active_and_stop(self) -> None:
        self._clear_active()
        self._publish_twist()

    def _mppi_observe(self, event: str, **values) -> None:
        if not self._is_mppi or not self._mppi_trace_enabled:
            return
        odom_stamp = None if self._odometry is None else self._odometry.header.stamp
        self._mppi_diagnostics.publish(String(data=json.dumps(dict(
            event=event,now_ns=self.get_clock().now().nanoseconds,
            latest_odom_ns=None if odom_stamp is None else int(odom_stamp.sec)*1_000_000_000+int(odom_stamp.nanosec),
            latest_tf_ns=None if not self._mppi_tf_history else self._mppi_tf_history[-1][0],
            **values))))

    def _publish_twist(self, linear: float = 0.0, angular: float = 0.0) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self._commands.publish(message)
        if self._mppi_step_sync:
            self._mppi_publication_sequence+=1
            self._mppi_published_command=[message.linear.x,message.angular.z]
        if self._is_mppi:
            self._executor.note_command(message.linear.x,message.angular.z,stamp_s=self.get_clock().now().nanoseconds/1e9)
            self._mppi_observe("publish", command=[message.linear.x,message.angular.z])
        else:
            self._executor.note_command(message.linear.x, message.angular.z)

    def destroy_node(self) -> bool:
        self._publish_twist()
        if self._p3_source is not None:self._p3_source.close()
        return super().destroy_node()


def main() -> None:
    import rclpy

    from rclpy.signals import SignalHandlerOptions

    # Keep the ROS context alive until destroy_node publishes the final stop.
    from nav1cm.ros_lifecycle import sigint_guard
    with sigint_guard() as stopped:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = PureWheeledControllerNode()
        try:
            from nav1cm.ros_lifecycle import spin_until_sigint
            if node._standard_mode:
                from rclpy.executors import MultiThreadedExecutor
                executor=MultiThreadedExecutor(num_threads=2)
                executor.add_node(node)
                try:
                    while rclpy.ok() and not stopped.is_set():executor.spin_once(timeout_sec=.1)
                finally:executor.shutdown()
            else:spin_until_sigint(node,stopped)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
