"""无 ROS 的导航任务合同；仅 Runtime 的 tick 线程可调用写入方法。"""
from dataclasses import dataclass, replace
import math

from .pose_goal import angle_error
from .ros_conversion import PoseGoal, finite


@dataclass(frozen=True)
class TaskSnapshot:
    task_id: str
    generation: int
    phase: str
    outcome: object = None
    reason_code: str = ''
    planning_cycle: int = 0
    segment_revision: int = 0


@dataclass(frozen=True)
class TaskObservation:
    """只读观测提案；输入序列门禁通过后才能提交，不提前改变保持计时。"""
    before: TaskSnapshot
    after: TaskSnapshot
    arrived_since: object


class NavigationTaskCoordinator:
    def __init__(self, session, *, event_sink=None, arrived_hold_s=.3):
        self.session = session
        self.event_sink = event_sink or (lambda event: None)
        self.arrived_hold_s = finite(arrived_hold_s)
        if self.arrived_hold_s < .3:
            raise ValueError('arrival hold must be at least 0.3 seconds')
        self.active = None
        self._arrived_since = None
        self._cycle = 0

    def _emit(self, snapshot):
        self.event_sink(snapshot)
        return snapshot

    def reject(self, task_id, reason_code):
        return self._emit(TaskSnapshot(task_id, 0, 'FINISHED', 'INVALID_GOAL', reason_code))

    def waiting(self, task_id, reason_code):
        return self._emit(TaskSnapshot(task_id, 0, 'PLANNING', reason_code=reason_code))

    def submit(self, task_id, goal, now_s):
        """goal 为 PoseGoal 或 (x,y,yaw|None)；None task_id 表示人工入口。"""
        try:
            if not isinstance(goal, PoseGoal):
                x, y, yaw = goal
                x, y = finite(x), finite(y)
                if yaw is None:
                    if self.session.state is None:
                        return self.waiting(task_id, 'WAITING_FOR_POSE')
                    yaw = self.session.state.yaw
                goal = PoseGoal(x, y, yaw)
            accepted = self.session.offer_goal(goal, force_new=task_id is not None)
        except (ValueError, TypeError) as exc:
            return self.reject(task_id, 'INVALID_GOAL: ' + str(exc))
        if not accepted:
            return self.reject(task_id, self.session.last_goal_rejection)
        if self.active is not None and self.active.outcome is None:
            self._emit(replace(self.active, phase='FINISHED', outcome='CANCELED', reason_code='GOAL_REPLACED'))
        self._arrived_since = None
        self.active = (None if task_id is None else
                       TaskSnapshot(task_id, self.session.accepted_generation, 'PLANNING', reason_code='GOAL_ACCEPTED'))
        return None if self.active is None else self._emit(self.active)

    def cancel(self, task_id, now_s):
        if self.active is None or self.active.task_id != task_id or self.active.outcome is not None:
            return False
        if self.active.phase != 'CANCELING':
            self.session.cancel_goal()
            self._arrived_since = None
            self.active = replace(self.active, phase='CANCELING', reason_code='CANCEL_WAITING_FOR_STOP')
            self._emit(self.active)
        return True

    def observe(self, packet, now_s, *, state_updated=True, defer_commit=False):
        current = self.active
        if current is None or current.outcome is not None:
            return None if defer_commit else current
        before, arrived_since = current, self._arrived_since
        state, goal, limits = self.session.state, self.session.goal, self.session.coordinator
        stopped = state is not None and abs(state.v) <= limits.stopped_linear_mps and abs(state.w) <= limits.stopped_angular_rps
        if current.phase == 'CANCELING':
            if stopped and state_updated:
                current = replace(current, phase='FINISHED', outcome='CANCELED', reason_code='CANCELED_STOPPED')
        else:
            arrived = (packet.generation == current.generation and not packet.waiting and stopped and goal is not None and
                       math.hypot(state.x-goal.x, state.y-goal.y) <= limits.goal_tolerance_m and
                       abs(angle_error(goal.yaw, state.yaw)) <= limits.goal_yaw_tolerance_rad)
            if not arrived:
                arrived_since = None
            elif state_updated:
                if arrived_since is None:
                    arrived_since = now_s
                elif now_s-arrived_since >= self.arrived_hold_s-1e-9:
                    current = replace(current, phase='FINISHED', outcome='GOAL_REACHED', reason_code='GOAL_REACHED')
            if current.outcome is None:
                reason = ('WAITING_FOR_MAP' if 'world' in packet.waiting else
                          'WAITING_FOR_POSE' if 'state' in packet.waiting else
                          'WAITING_FOR_PATH' if packet.route is None else packet.result.status)
                phase = 'EXECUTING' if packet.candidate is not None and packet.route is not None else 'PLANNING'
                current = replace(current, phase=phase, reason_code=reason)
        revision = 0 if packet.route is None else max(0, packet.route.version)
        proposal = TaskObservation(before, replace(current, planning_cycle=self._cycle+1,
                                                    segment_revision=revision), arrived_since)
        return proposal if defer_commit else self.commit_observation(proposal)

    def commit_observation(self, proposal):
        """仅所有者线程提交；Runtime在输入锁内调用并将事件写入内部队列。"""
        if proposal is None or self.active is not proposal.before:
            return self.active
        self.active = proposal.after
        self._arrived_since = proposal.arrived_since
        self._cycle = proposal.after.planning_cycle
        return self._emit(self.active)
