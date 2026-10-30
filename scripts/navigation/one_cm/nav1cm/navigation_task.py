"""无 ROS 的导航任务合同；仅 Runtime 的 tick 线程可调用写入方法。"""
from dataclasses import dataclass, replace
import math
from collections import OrderedDict
from .execution_contract import ExecutionReference, ExecutionStatus, execution_session_id, FOLLOW, BRAKE, HOLD, FINAL_ALIGN

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
        self.configure_execution_feedback()
        self._reset_execution_feedback(0.)

    def configure_execution_feedback(self, required=False, timeout_s=2.):
        timeout_s = finite(timeout_s)
        if type(required) is not bool or timeout_s <= 0:
            raise ValueError('invalid execution feedback configuration')
        self._feedback_required, self._feedback_timeout = required, timeout_s

    def _reset_execution_feedback(self, now_s):
        self._references = OrderedDict()
        self._execution_status = None
        self._execution_received = None
        self._execution_last_accepted = None
        self._execution_started = now_s
        self._execution_revision = -1
        self._execution_stamp = -1
        self._execution_failure = None
        self._execution_accepted = set()
        self._execution_terminal = set()
        self._execution_failed_revisions = set()

    def register_execution_reference(self, ref, now_s):
        if not isinstance(ref, ExecutionReference) or self.active is None or self.active.outcome is not None:
            return False
        if ref.session_id != execution_session_id(self.active.task_id, self.active.generation):
            return False
        if ref.revision in self._references:
            return self._references[ref.revision] == ref
        if self._references and ref.revision <= next(reversed(self._references)):
            return False
        self._references[ref.revision] = ref
        while len(self._references) > 128:
            retired, _ = self._references.popitem(last=False)
            self._execution_accepted.discard(retired)
            self._execution_terminal.discard(retired)
            self._execution_failed_revisions.discard(retired)
        return True

    def offer_execution_status(self, status, now_s):
        if not isinstance(status, ExecutionStatus) or self.active is None or self.active.outcome is not None:
            return False
        ref = self._references.get(status.revision)
        accepted_failure = status.state == 6 and status.revision in self._execution_accepted
        if (ref is None or status.session_id != ref.session_id or status.path_digest != ref.path_digest or
                status.intent != ref.intent or status.stamp_ns < ref.stamp_ns or (status.revision < self._execution_revision and not accepted_failure) or
                (status.revision in self._execution_terminal and status.state not in (5,6)) or
                (status.revision in self._execution_failed_revisions and status.state != 6) or
                (status.stamp_ns <= self._execution_stamp and not accepted_failure) or now_s < self._execution_started or
                (self._execution_received is not None and now_s < self._execution_received)):
            return False
        if status.state == 6:
            self._execution_failed_revisions.add(status.revision)
            if self._execution_failure is None:
                self._execution_failure = status.reason or 'TRACKING_FAILED'
            if status.revision < self._execution_revision or status.stamp_ns <= self._execution_stamp:
                # 已接受旧参考的延迟故障只锁存故障，不回退正常反馈门禁。
                return True
        if status.accepted:
            self._execution_last_accepted = now_s
            self._execution_accepted.add(status.revision)
        if status.state in (5,6):
            self._execution_terminal.add(status.revision)
        self._execution_status, self._execution_received = status, now_s
        self._execution_revision, self._execution_stamp = max(self._execution_revision, status.revision), status.stamp_ns
        return True

    def check_execution_fault(self, now_s):
        if not self._feedback_required or self.active is None or self.active.outcome is not None or self.active.phase in ('CANCELING','ERROR_STOPPING'):
            return None
        status = self._execution_status
        if self._execution_failure is not None:
            return self._execution_failure
        last = self._execution_started if self._execution_last_accepted is None else self._execution_last_accepted
        if now_s - last > self._feedback_timeout:
            return 'EXECUTION_FEEDBACK_TIMEOUT'
        return None

    def _execution_arrived(self, now_s):
        if not self._feedback_required:
            return True
        status = self._execution_status
        return (status is not None and status.accepted and status.state in (0,5) and status.intent in (BRAKE,HOLD,FINAL_ALIGN) and
                self._execution_failure is None and
                0 <= now_s-self._execution_received <= self._feedback_timeout)

    def fail_tracking(self, reason, now_s):
        if self.active is None or self.active.outcome is not None or self.active.phase in ('CANCELING','ERROR_STOPPING'):
            return False
        self.session.cancel_goal()
        self._arrived_since = None
        self.active = replace(self.active, phase='ERROR_STOPPING', reason_code=reason)
        self._emit(self.active)
        return True

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
        self._reset_execution_feedback(now_s)
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
        if current.phase == 'ERROR_STOPPING':
            if stopped and state_updated:
                current = replace(current, phase='FINISHED', outcome='INTERNAL_ERROR')
        elif current.phase == 'CANCELING':
            if stopped and state_updated:
                current = replace(current, phase='FINISHED', outcome='CANCELED', reason_code='CANCELED_STOPPED')
        else:
            arrived = (self._execution_arrived(now_s) and packet.generation == current.generation and not packet.waiting and stopped and goal is not None and
                       math.hypot(state.x-goal.x, state.y-goal.y) <= limits.position_target_tolerance_m and
                       abs(angle_error(goal.yaw, state.yaw)) <= limits.goal_yaw_tolerance_rad)
            if not arrived:
                arrived_since = None
            elif state_updated:
                if arrived_since is None:
                    arrived_since = now_s
                elif now_s-arrived_since >= self.arrived_hold_s-1e-9:
                    current = replace(current, phase='FINISHED', outcome='GOAL_REACHED', reason_code='GOAL_REACHED')
            if current.outcome is None:
                status = None if packet.result is None else packet.result.status
                # NOT_DUE只表示本周期没有重新计算；保留上一真实周期的执行/等待反馈。
                # BUFFER_UNAVAILABLE可能携带受认证的制动候选，不能当可执行导航buffer。
                if status != 'NOT_DUE' or packet.waiting or packet.route is None:
                    reason = ('WAITING_FOR_MAP' if 'world' in packet.waiting else
                              'WAITING_FOR_POSE' if 'state' in packet.waiting else
                              'WAITING_FOR_PATH' if packet.route is None or status == 'BUFFER_UNAVAILABLE' else status)
                    phase = ('EXECUTING' if packet.candidate is not None and packet.route is not None and
                             reason != 'WAITING_FOR_PATH' else 'PLANNING')
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
