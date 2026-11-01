"""NavigateToPose + 实测里程计后端；标准 Path 仅用于路程估计。"""
from threading import Lock
import math
import time
from .ros_messages import PolicyMapClient, NavigationResult, InfrastructureError


class Nav1cmNavigationAdapter(PolicyMapClient):
    def __init__(self,node,*,action_name,policy_map_service,odometry_topic,
                 tf_topic='/tf',tf_static_topic='/tf_static',map_frame='map',odom_frame='odom',
                 global_route_topic=None):
        super().__init__(node,policy_map_service=policy_map_service,odometry_topic=odometry_topic,
            tf_topic=tf_topic,tf_static_topic=tf_static_topic,map_frame=map_frame,odom_frame=odom_frame)
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from lunar_planning_msgs.action import NavigateToPose
        from nav_msgs.msg import Path
        self._action_type=NavigateToPose;self.action=ActionClient(node,NavigateToPose,action_name)
        self._execution_lock=Lock();self._goal_serial=0
        self._send_future=self._handle=self._result_future=None
        self._cancel=False;self._cancel_future=None;self.sent_goal=None
        self._feedback=None;self._execution_route=None;self._execution_feedback_wall_s=0.
        self.last_goal_execution=self.last_goal_result=self.last_goal_execution_serial=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        self.subscriptions.append(node.create_subscription(Path,
            global_route_topic or action_name.rsplit('/',1)[0]+'/route',self._route,qos))

    @property
    def ready(self):return self.client.service_is_ready() and self.action.server_is_ready()

    @property
    def inflight(self):return self._send_future is not None

    def begin_goal(self,goal):
        goal=tuple(map(float,goal))
        if len(goal)!=3 or not all(map(math.isfinite,goal)):
            raise ValueError('finite x/y/yaw goal required')
        if self.inflight:raise RuntimeError('navigation action already in flight')
        if not self.action.server_is_ready():raise InfrastructureError('navigation action unavailable')
        self._goal_serial+=1;serial=self._goal_serial;self.sent_goal=goal
        self.last_goal_execution=self.last_goal_result=self.last_goal_execution_serial=None
        self._cancel=False;self._cancel_future=None;self._handle=self._result_future=None
        with self._execution_lock:
            self._feedback=self._execution_route=None;self._execution_feedback_wall_s=0.
        request=self._action_type.Goal(target_x_m=goal[0],target_y_m=goal[1],has_target_yaw=True,target_yaw_rad=goal[2])
        def feedback(message):
            handle=self._handle
            if (serial!=self._goal_serial or not self.inflight or handle is None or
                    bytes(message.goal_id.uuid)!=bytes(handle.goal_id.uuid)):return
            value=message.feedback
            with self._execution_lock:
                self._feedback=dict(session_id=bytes(message.goal_id.uuid).hex(),
                    session_state=int(value.session_state),planning_cycle=int(value.planning_cycle),
                    active_segment_revision=int(value.active_segment_revision),reason_code=str(value.reason_code))
                self._execution_feedback_wall_s=time.monotonic()
        try:self._send_future=self.action.send_goal_async(request,feedback_callback=feedback)
        except Exception as exc:raise InfrastructureError('navigation send transport: '+str(exc)) from exc
        return serial

    def _route(self,msg):
        goal=self.sent_goal
        if not self.inflight or goal is None or msg.header.frame_id!=self.map_frame:return
        points=tuple((float(p.pose.position.x),float(p.pose.position.y)) for p in msg.poses)
        if not points or math.hypot(points[-1][0]-goal[0],points[-1][1]-goal[1])>1.:return
        from .navigation_budget import polyline_length
        try:length=polyline_length(points)
        except ValueError:return
        with self._execution_lock:
            self._execution_route=dict(target=goal[:2],route_length_m=length,points=points)

    def execution_state(self):
        handle=self._handle;session=bytes(handle.goal_id.uuid).hex() if handle is not None else None
        with self._execution_lock:
            feedback=self._feedback if self.inflight and self._feedback and self._feedback['session_id']==session else None
            route=self._execution_route if self.inflight else None
            stamp=self._execution_feedback_wall_s if feedback else 0.
        return dict(session_id=session,accepted=handle is not None and handle.accepted,
            send_done=self._send_future is not None and self._send_future.done(),
            result_pending=self._result_future is not None and not self._result_future.done(),
            feedback_wall_s=stamp,planning_reason=feedback['reason_code'] if feedback else None,
            route_length_m=route['route_length_m'] if route else None,
            global_route=route,reference=None,tracking=None,feedback=feedback,
            measured_pose=self.pose if hasattr(self,'pose') else None,measured_velocity=self.velocity)

    def cancel_goal(self):
        self._cancel=True
        self._advance_goal()

    def _advance_goal(self):
        if not self.inflight:return
        try:
            if self._handle is None and self._send_future.done():
                self._handle=self._send_future.result()
                if self._handle is None:raise InfrastructureError('navigation goal transport lost')
                if self._handle.accepted:self._result_future=self._handle.get_result_async()
            if self._cancel and self._handle is not None and self._handle.accepted and self._cancel_future is None:
                self._cancel_future=self._handle.cancel_goal_async()
        except InfrastructureError:raise
        except Exception as exc:raise InfrastructureError('navigation transport: '+str(exc)) from exc

    def _finish(self,result):
        self.last_goal_execution=self.execution_state();self.last_goal_result=result
        self.last_goal_execution_serial=self._goal_serial
        self._send_future=self._handle=self._result_future=None
        return result

    def poll_goal(self):
        self._advance_goal()
        if self._handle is not None and not self._handle.accepted:
            self._finish(NavigationResult(6,'GOAL_REJECTED',0))
            raise InfrastructureError('navigation action rejected')
        if self._result_future is None or not self._result_future.done():return None
        try:wrapped=self._result_future.result()
        except Exception as exc:raise InfrastructureError('navigation result transport: '+str(exc)) from exc
        if wrapped is None:raise InfrastructureError('navigation result transport lost')
        raw=wrapped.result
        result=NavigationResult(int(raw.outcome),str(raw.reason_code),int(raw.last_segment_revision))
        if result.outcome==6:
            self._finish(result)
            raise InfrastructureError('navigation internal failure: '+result.reason_code)
        return self._finish(result)

    @property
    def terminal_stop_confirmed(self):
        # nav1cm终态已包含导航owner的实际停稳确认，不从客户端旧速度重新推断。
        return (not self.inflight and self.last_goal_result is not None and
                self.last_goal_result.outcome in (0,5))

    def close(self):
        self.action.destroy();self.node.destroy_client(self.client)
        for subscription in self.subscriptions:self.node.destroy_subscription(subscription)