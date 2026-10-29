"""P3 导航会话：无 ROS，目标校验与协调器始终由同一个 tick 线程写入。"""
from dataclasses import dataclass

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


class NavigationSession:
    def __init__(self, coordinator):
        self.coordinator = coordinator
        self.world = self.state = self.goal = None
        self.accepted_generation = 0
        self.last_goal_rejection = None
        self.canceling = False

    def set_world(self, world):
        self.world = world

    def offer_state(self, state):
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
        # 显式代际使两次 tick 之间 A→B→A 也能使最初的 A 全部失效。
        self.coordinator.replace_goal((goal.x, goal.y), goal_yaw=goal.yaw,
                                      generation=self.accepted_generation)
        return True

    def cancel_goal(self):
        self.accepted_generation += 1
        self.goal, self.canceling = None, True
        self.coordinator.invalidate_goal(generation=self.accepted_generation)

    def tick(self, now_s):
        if self.canceling and self.world is not None and self.state is not None:
            result = self.coordinator.cancel_tick(self.world, self.state, now_s)
            return SessionPacket(self.accepted_generation, result=result, candidate=result.candidate)

        waiting = tuple(name for name in ('world', 'state', 'goal') if getattr(self, name) is None)
        if waiting:
            return SessionPacket(self.accepted_generation, waiting=waiting)
        result = self.coordinator.tick(self.world, self.state, (self.goal.x, self.goal.y),
                                       now_s, goal_yaw=self.goal.yaw)
        candidate = result.candidate
        if (candidate is None or result.certificate is None or
                not result.certificate.matches(candidate, result.grid, self.coordinator.limits, candidate.route_version)):
            candidate = None
        return SessionPacket(self.accepted_generation, self.coordinator.route, result, candidate=candidate)

    def publication_current(self, packet):
        return bool(packet.generation == self.accepted_generation and not packet.waiting and
                    packet.result is not None and packet.result.status != 'NOT_DUE')

    def close(self, timeout_s=2.):
        return self.coordinator.close(timeout_s=timeout_s)
