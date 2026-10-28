"""可选 NavigateToPose 传输入口；回调只处理不可变快照和投递请求。"""
from queue import Empty, SimpleQueue
import time
import json

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.signals import SignalHandlerOptions
from lunar_planning_msgs.action import NavigateToPose
from nav1cm.navigation_task import TaskSnapshot
from .node import PlannerNode


class ROSTaskApi:
    def __init__(self, node, runtime, action_name):
        self.runtime = runtime
        self.events = SimpleQueue()
        self.pending = {}
        self.closing = False
        self.group = ReentrantCallbackGroup()
        runtime.event_sink = self.events.put
        self.server = ActionServer(node, NavigateToPose, action_name,
                                   execute_callback=self._execute,
                                   goal_callback=self._goal,
                                   cancel_callback=self._cancel,
                                   handle_accepted_callback=self._accepted,
                                   callback_group=self.group)
        self.timer = node.create_timer(.02, self._drain, callback_group=self.group)

    @staticmethod
    def identity(handle):
        return bytes(handle.goal_id.uuid).hex()

    def _goal(self, request):
        # 传输接受与逻辑有效性分开：无效请求仍返回可解释的 INVALID_GOAL。
        return GoalResponse.REJECT if self.closing else GoalResponse.ACCEPT

    def _accepted(self, handle):
        task_id = self.identity(handle)
        self.pending[task_id] = (handle, Future())
        self.runtime.offer_managed_goal(task_id, handle.request)
        handle.execute()

    def _cancel(self, handle):
        if self.identity(handle) not in self.pending:
            return CancelResponse.REJECT
        self.runtime.request_cancel(self.identity(handle))
        return CancelResponse.ACCEPT

    async def _execute(self, handle):
        task_id = self.identity(handle)
        _, future = self.pending[task_id]
        event = await future
        result = NavigateToPose.Result()
        result.outcome = getattr(NavigateToPose.Result, event.outcome)
        result.reason_code = event.reason_code
        result.last_segment_revision = event.segment_revision
        if handle.is_active:
            if event.outcome == 'GOAL_REACHED':
                handle.succeed()
            elif event.outcome == 'CANCELED' and handle.is_cancel_requested:
                handle.canceled()
            else:
                # 替换不是显式取消握手；以结果中的 CANCELED/GOAL_REPLACED归因。
                handle.abort()
        self.pending.pop(task_id, None)
        return result

    def _drain(self):
        while True:
            try:
                event = self.events.get_nowait()
            except Empty:
                return
            entry = self.pending.get(event.task_id)
            if entry is None:
                continue
            handle, future = entry
            if future.done():
                continue
            if event.outcome is not None:
                future.set_result(event)
            elif handle.is_active:
                feedback = NavigateToPose.Feedback()
                feedback.session_state = getattr(NavigateToPose.Feedback, event.phase, NavigateToPose.Feedback.PLANNING)
                feedback.planning_cycle = event.planning_cycle
                feedback.active_segment_revision = event.segment_revision
                feedback.reason_code = event.reason_code
                handle.publish_feedback(feedback)

    def close(self):
        self.closing = True
        for task_id, (_, future) in tuple(self.pending.items()):
            if not future.done():
                future.set_result(TaskSnapshot(task_id, 0, 'FINISHED', 'INTERNAL_ERROR', 'SHUTDOWN'))

    def destroy(self):
        self.timer.cancel()
        self.server.destroy()


class NavigationTaskNode(PlannerNode):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        name = self.declare_parameter('action_name', '/Car/T4/navigation/navigate_to_pose').value
        self.task_api = ROSTaskApi(self, self.runtime, name)

    def destroy_node(self):
        self.task_api.destroy()
        return super().destroy_node()


def main(args=None):
    from nav1cm.ros_lifecycle import sigint_guard, spin_until_sigint
    with sigint_guard() as stopped:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        try:
            node = NavigationTaskNode()
            spin_until_sigint(node, stopped)
        except KeyboardInterrupt:
            pass
        finally:
            if node is not None:
                node.task_api.close()
                result = node.stop_runtime()
                if result['threads_alive']:
                    raise RuntimeError(f'处理线程尚未退出: {result}')
                node.get_logger().info('TASK_RUNTIME_STOPPED: ' + json.dumps(result, sort_keys=True))
                # execute完成后，rclpy仍需调度GetResult的发送回调；pending为空不能提前销毁。
                # 固定有界排空窗口也允许已发出的结果请求在关闭期间到达。
                deadline = time.monotonic()+1.
                while time.monotonic()<deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                if node.task_api.pending:
                    raise RuntimeError('任务Action回调未在关闭期限内退出')
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
