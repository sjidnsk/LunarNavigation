"""无 ROS/SDK 依赖的四角转向与滚动目标执行器。"""
from copy import deepcopy
import math


def _finite(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


class DriveController:
    """命令和反馈均采用 SI；四元素数组顺序永远为 fl/fr/rl/rr。

    receive_twist(vx, vy, wz, wall_time) -> bool：只支持车体 X 平移与 Z 旋转，
    非零 vy、非有限数和接收流倒退时间使旧指令失效。wall_time 为真实接收墙钟秒；
    异步接收流与控制步流分别单调，已收到的包可以在后续控制步才被消费。
    step(dt, wall_time, steering_positions, body_speed) -> dict：转向为弧度，
    滚动为 rad/s；body_speed 推荐 (线速度模 m/s, yaw rad/s)，以实测双门限
    确认停稳。兼容标量线速度，但该形式只能检查内部 yaw 目标是否归零，
    不能证明实测角速度已停；真实运行适配器必须传二元反馈。

    正常停车/模式及转向切换执行制动坡道，再等实测停稳、转向到位后滚动。
    超时或异常立即将轮速目标归零（紧急停车目标，不代表物理瞬时停稳）。
    safe_stop=True 表示轮速被安全门控为零；reason 提供触发原因。
    """

    def __init__(self, profile):
        self.profile = deepcopy(profile)
        self.cap = self.profile["capability"]
        self.cfg = self.profile["drive"]
        self.radius = self.cap["wheel_diameter_m"] / 2
        x, y = self.cap["wheelbase_m"] / 2, self.cap["track_width_m"] / 2
        self.positions = ((x, y), (x, -y), (-x, y), (-x, -y))
        self.target = (0.0, 0.0)
        self.current = (0.0, 0.0)
        self.steering = [0.0] * 4
        self.command_time = None
        self.last_receive_time = None
        self.last_step_time = None
        self.fault = None
        self.switching = False

    def receive_twist(self, vx, vy, wz, wall_time):
        valid_receive_time = (_finite(wall_time) and wall_time >= 0 and
            (self.last_receive_time is None or wall_time >= self.last_receive_time))
        if valid_receive_time:
            self.last_receive_time = float(wall_time)
        if (not all(_finite(v) for v in (vx, vy, wz, wall_time))
                or vy != 0 or not valid_receive_time):
            self.target = (0.0, 0.0)
            self.command_time = None
            self.fault = "invalid_command"
            return False
        vx = max(-self.cap["maximum_reverse_speed_mps"],
                 min(vx, self.cap["maximum_forward_speed_mps"]))
        yaw_limit = self.cap["maximum_spin_rate_radps"]
        if vx != 0:
            # 内侧轮的转向执行上限也约束弧线；平移上限不限制自转轮切向速。
            tangent = math.tan(self.cfg["steering_limit_rad"])
            steering_curvature = 1 / (self.cap["track_width_m"] / 2 +
                                      self.cap["wheelbase_m"] / (2 * tangent))
            yaw_limit = min(yaw_limit, abs(vx) * steering_curvature,
                            abs(vx) * self.cap["maximum_curvature_per_m"],
                            self.cap["maximum_lateral_acceleration_mps2"] / abs(vx))
        wz = max(-yaw_limit, min(wz, yaw_limit))
        self.target = (float(vx), float(wz))
        self.command_time = float(wall_time)
        self.fault = None
        return True

    def _wheels(self, vx, wz):
        angles, rates = [], []
        for x, y in self.positions:
            wx, wy = vx - wz * y, wz * x
            angle = math.atan2(wy, wx)
            speed = math.hypot(wx, wy)
            # 等价轮轴方向折回 ±90°，反向由滚动符号承担。
            if angle > math.pi / 2:
                angle -= math.pi
                speed = -speed
            elif angle < -math.pi / 2:
                angle += math.pi
                speed = -speed
            angles.append(angle)
            rates.append(speed / self.radius)
        return angles, rates

    def _out(self, reason, safe=False):
        rates = self._wheels(*self.current)[1] if not safe else [0.0] * 4
        return {"steering_rad": list(self.steering), "wheel_radps": rates,
                "safe_stop": safe, "reason": reason}

    def _stop(self, reason):
        self.current = (0.0, 0.0)
        self.switching = True
        return self._out(reason, safe=True)

    def _ramp(self, target, dt):
        vx, wz = self.current
        dv, dw = target[0] - vx, target[1] - wz
        accel = (self.cap["maximum_braking_deceleration_mps2"]
                 if abs(target[0]) < abs(vx) else self.cap["maximum_acceleration_mps2"])
        scale = 1.0
        if dv:
            scale = min(scale, accel * dt / abs(dv))
        if dw:
            scale = min(scale, self.cap["maximum_yaw_acceleration_radps2"] * dt / abs(dw))
        self.current = (vx + scale * dv, wz + scale * dw)

    @staticmethod
    def _mode(vx, wz):
        if vx:
            return "forward" if vx > 0 else "reverse"
        return ("spin_left" if wz > 0 else "spin_right") if wz else "stop"

    def step(self, dt, wall_time, steering_positions, body_speed):
        try:
            feedback = list(steering_positions)
            speeds = ([body_speed, 0.0] if _finite(body_speed) else list(body_speed))
            valid = (len(feedback) == 4 and len(speeds) == 2
                     and all(_finite(v) for v in feedback + speeds))
        except (TypeError, ValueError):
            valid = False
        if (not valid or not _finite(dt) or dt <= 0 or not _finite(wall_time)
                or wall_time < 0 or (self.last_step_time is not None and wall_time < self.last_step_time)):
            self.command_time = None
            self.fault = "invalid_feedback_or_time"
            return self._stop(self.fault)
        self.last_step_time = float(wall_time)
        if self.fault:
            return self._stop(self.fault)
        if self.command_time is None:
            return self._stop("no_command")
        if wall_time < self.command_time:
            self.command_time = None
            self.fault = "invalid_feedback_or_time"
            return self._stop(self.fault)
        if wall_time - self.command_time >= self.cfg["command_timeout_s"]:
            return self._stop("command_timeout")
        target_angles = self._wheels(*self.target)[0]
        moving = any(abs(v) > 1e-12 for v in self.current)
        target_stop = self.target == (0.0, 0.0)
        mode_change = self._mode(*self.current) != self._mode(*self.target)
        angle_change = any(abs(a - b) > self.cfg["steering_tolerance_rad"]
                           for a, b in zip(target_angles, self.steering))
        if moving and (target_stop or mode_change or angle_change):
            self.switching = True
        if self.switching or target_stop:
            self._ramp((0.0, 0.0), dt)
            if any(abs(v) > 1e-12 for v in self.current):
                return self._out("braking")
            stopped = (abs(speeds[0]) <= self.cfg["stopped_linear_speed_mps"]
                       and abs(speeds[1]) <= self.cfg["stopped_yaw_rate_radps"])
            if not stopped:
                return self._out("waiting_for_stop", safe=True)
            self.switching = False
            if target_stop:
                return self._out("stopped", safe=True)
        # 即便刚启动，也不能在实测车体运动时重排转向。
        if not moving and angle_change and (
                abs(speeds[0]) > self.cfg["stopped_linear_speed_mps"]
                or abs(speeds[1]) > self.cfg["stopped_yaw_rate_radps"]):
            self.switching = True
            return self._out("waiting_for_stop", safe=True)
        self.steering = target_angles
        if any(abs(actual - desired) > self.cfg["steering_tolerance_rad"]
               for actual, desired in zip(feedback, self.steering)):
            return self._stop("steering_alignment")
        self._ramp(self.target, dt)
        return self._out("tracking")
