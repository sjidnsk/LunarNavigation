"""仅由 Path 几何驱动的有界重定时器，及独立解析车体 SAT。

不导入 nav1cm、生产认证或 verification_plant。只供短直线/倒车/转向验证；
不把它当通用车辆控制器，也不继承原 Path 时序的安全认证。
"""
from dataclasses import dataclass
import math


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def clip(value, bound):
    return max(-bound, min(bound, value))


@dataclass(frozen=True)
class State:
    x: float = 0.
    y: float = 0.
    yaw: float = 0.
    v: float = 0.
    w: float = 0.


def integrate(state, a, alpha, dt):
    """常加速度小步中点积分；末速单独更新，绝不以平均速度代替末速。"""
    if not 0 < dt <= .020000001:
        raise ValueError('integration step must be in (0, .02]')
    vm, wm = state.v + a * dt / 2, state.w + alpha * dt / 2
    heading = state.yaw + wm * dt / 2
    return State(state.x + vm * math.cos(heading) * dt,
                 state.y + vm * math.sin(heading) * dt,
                 wrap(state.yaw + wm * dt), state.v + a * dt, state.w + alpha * dt)


class PathFollower:
    def __init__(self, *, freshness_s=.35):
        self.points = []
        self.received_s = -math.inf
        self.freshness_s = freshness_s

    def offer(self, points, now_s):
        self.points, self.received_s = list(points), now_s

    def step(self, state, now_s, dt):
        vd = wd = 0.
        mode = 'brake'
        if self.points and now_s - self.received_s <= self.freshness_s:
            x, y, yaw = self.points[-1]
            dx, dy = x - state.x, y - state.y
            distance = math.hypot(dx, dy)
            # 一个控制步的行程加 5 mm 几何余量，先停车再执行原地转向。
            margin = .005 + .2 * dt
            if distance > margin and abs(state.w) <= 1e-8:
                sign = 1. if dx * math.cos(state.yaw) + dy * math.sin(state.yaw) >= 0 else -1.
                vd = sign * min(.2, math.sqrt(max(0., distance - margin)))
                mode = 'translate'
            elif distance <= margin and abs(state.v) <= 1e-8:
                error = wrap(yaw - state.yaw)
                yaw_margin = .005 + abs(state.w) * dt
                wd = math.copysign(min(1., math.sqrt(max(0., abs(error) - yaw_margin))), error)
                mode = 'spin' if abs(error) > yaw_margin else 'hold'
        a, alpha = clip((vd - state.v) / dt, .5), clip((wd - state.w) / dt, .5)
        return integrate(state, a, alpha, dt), a, alpha, mode


def pose_clear(state, obstacles, *, inflation=0.):
    """旋转矩形与轴对齐障碍矩形的分离轴判定；接触视为碰撞。"""
    c, s = math.cos(state.yaw), math.sin(state.yaw)
    u, v = (c, s), (-s, c)
    half_length, half_width = 1.182 / 2, .818 / 2
    for x0, y0, x1, y1 in obstacles:
        ox, oy = (x0 + x1) / 2 - state.x, (y0 + y1) / 2 - state.y
        hx, hy = (x1 - x0) / 2 + inflation, (y1 - y0) / 2 + inflation
        separated = False
        for ax, ay in ((1., 0.), (0., 1.), u, v):
            body = half_length * abs(ax * u[0] + ay * u[1]) + half_width * abs(ax * v[0] + ay * v[1])
            rect = hx * abs(ax) + hy * abs(ay)
            if abs(ox * ax + oy * ay) > body + rect:
                separated = True
                break
        if not separated:
            return False
    return True


def swept_clear(start, end, obstacles):
    """细分姿态并按相邻采样间车体最大运动上界膨胀，覆盖中段碰角。"""
    angle = wrap(end.yaw - start.yaw)
    bound = math.hypot(end.x - start.x, end.y - start.y) + math.hypot(1.182, .818) / 2 * abs(angle)
    count = max(1, math.ceil(bound / .002))
    inflation = bound / count / 2 + 1e-10
    for i in range(count + 1):
        t = i / count
        state = State(start.x + t * (end.x - start.x), start.y + t * (end.y - start.y), start.yaw + t * angle)
        if not pose_clear(state, obstacles, inflation=inflation):
            return False
    return True
