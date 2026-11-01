"""P3 导航会话：无 ROS，目标校验与协调器始终由同一个 tick 线程写入。"""
from dataclasses import dataclass, replace

from .contracts import WorldSnapshot
from .pose_goal import angle_error, check_pose_goal
from .ros_conversion import PoseGoal


@dataclass(frozen=True)
class SessionPacket:
    generation: int
    route: object = None
    result: object = None
    waiting: tuple = ()
    candidate: object = None
    raw_route: object = None


class NavigationSession:
    def __init__(self, coordinator):
        self.coordinator = coordinator
        self.world = self.state = self.goal = None
        self.accepted_generation = 0
        self.last_goal_rejection = None
        self.canceling = False
        self.direct_control = False
        self._arrival_evidence = None

    def set_world(self, world):
        self.world = world

    def offer_state(self, state):
        if state is None and self.state is not None and self.goal is not None and not self.canceling:
            # 保留任务及最终位姿，退休失去反馈前的计算和运动缓存。
            self.coordinator.invalidate_goal(generation=self.accepted_generation)
        self.state = state

    def offer_goal(self, goal, *, force_new=False):
        if not isinstance(goal, PoseGoal):
            raise TypeError('goal must be PoseGoal')
        if (not force_new and self.goal is not None and (self.goal.x, self.goal.y) == (goal.x, goal.y) and
                abs(angle_error(self.goal.yaw, goal.yaw)) <= 1e-12):
            # 周期重发同一有效目标不重启计算；后来受阻也仍保留该目标。
            return True
        world = self.world if self.world is not None else WorldSnapshot(0, (), ())
        check = check_pose_goal(world, (goal.x, goal.y), goal.yaw, self.coordinator.limits)
        if not check.accepted:
            self.last_goal_rejection = check.reason_code
            return False
        self.accepted_generation += 1
        self.goal, self.last_goal_rejection = goal, None
        self.canceling = False
        self._arrival_evidence = None
        # 显式代际使两次 tick 之间 A→B→A 也能使最初的 A 全部失效。
        self.coordinator.replace_goal((goal.x, goal.y), goal_yaw=goal.yaw,
                                      generation=self.accepted_generation)
        return True

    def cancel_goal(self):
        self.accepted_generation += 1
        self.goal, self.canceling = None, True
        self.coordinator.invalidate_goal(generation=self.accepted_generation)

    def arrival_current(self, packet):
        # NOT_DUE不产生新证明；仅复用相同输入下上一份真实认证的终态。
        evidence=self._arrival_evidence
        if evidence is None or packet.waiting or packet.generation!=self.accepted_generation:return False
        world,state,goal,generation,result=evidence
        if (world is not self.world or state!=self.state or goal!=self.goal or
                generation!=self.accepted_generation or
                (packet.result is not result and getattr(packet.result,'status',None)!='NOT_DUE')):return False
        return (self.coordinator.goal_reached(self.state) and result.status=='ARRIVED' and
                result.candidate is not None and result.certificate is not None and
                result.certificate.matches(result.candidate,result.grid,self.coordinator.limits,
                                           result.candidate.route_version))

    def tick(self, now_s, *, planning_state=None):
        if self.canceling and self.world is not None and self.state is not None:
            result = self.coordinator.cancel_tick(self.world, self.state, now_s, need_action=False)
            return SessionPacket(self.accepted_generation, result=result, candidate=result.candidate)

        waiting = tuple(name for name in ('world', 'state', 'goal') if getattr(self, name) is None)
        if waiting:
            self._arrival_evidence=None
            return SessionPacket(self.accepted_generation, waiting=waiting)
        if hasattr(self.coordinator, 'velocity_reference'):
            self.coordinator.velocity_reference = self.direct_control
        # 规划可消费延迟补偿后的当前估计；self.state 始终保留 P3 实测。
        state = planning_state if self.direct_control and planning_state is not None else self.state
        result = self.coordinator.tick(self.world, state, (self.goal.x, self.goal.y),
                                       now_s, goal_yaw=self.goal.yaw, need_action=False, observed_state=self.state)
        # 预测终态只能请求保持/制动；真实 P3 未到达不能提交任务完成证据。
        if result.status == 'ARRIVED' and not self.coordinator.goal_reached(self.state):
            result = replace(result, status='GOAL_BRAKING')
        candidate = result.candidate
        velocity_source = (self.direct_control and result.reference_phase_s is not None and
                           result.status == 'BUFFER_EXECUTING')
        if not velocity_source and (candidate is None or result.certificate is None or
                not result.certificate.matches(candidate, result.grid, self.coordinator.limits, candidate.route_version)):
            candidate = None
        if result.status!='NOT_DUE':
            self._arrival_evidence=((self.world,self.state,self.goal,self.accepted_generation,result)
                                    if result.status=='ARRIVED' and candidate is not None else None)
        return SessionPacket(self.accepted_generation, self.coordinator.route, result, candidate=candidate,
                             raw_route=self.coordinator.raw_route)

    def publication_current(self, packet):
        return bool(packet.generation == self.accepted_generation and not packet.waiting and
                    packet.result is not None and packet.result.status != 'NOT_DUE')

    def close(self, timeout_s=2.):
        return self.coordinator.close(timeout_s=timeout_s)
