"""显式版本的 JSON 传输契约；所有算法时间使用同机单调时钟。"""
from dataclasses import asdict
import json
import math
import os
from nav1cm.contracts import CellRect, WorldSnapshot, WheelLimits
from nav1cm.verification_plant import TimedPlan

CHANNELS = ('world', 'goal', 'feedback', 'plan', 'ack', 'diagnostic')
ROLES = ('fixture', 'navigator', 'probe')


def topic(channel):
    if channel not in CHANNELS:
        raise ValueError('unknown isolated channel')
    return '/nav1cm_verification/' + channel


def encode(value):
    return json.dumps(value, allow_nan=False, separators=(',', ':'))


def envelope(session, producer, kind, payload, sent_monotonic_s, ros_stamp_ns, *, pid=None):
    message = dict(version=1, session=session, producer=producer, producer_pid=os.getpid() if pid is None else pid,
                   kind=kind, payload=payload, sent_monotonic_s=sent_monotonic_s, ros_stamp_ns=ros_stamp_ns)
    decode(encode(message), session)
    return message


def decode(raw, session):
    def invalid(value):
        raise ValueError('nonfinite JSON: ' + value)
    message = json.loads(raw, parse_constant=invalid)
    if (type(message.get('version')) is not int or message.get('version') != 1 or message.get('session') != session or not session or
            message.get('producer') not in ROLES or type(message.get('producer_pid')) is not int or
            message['producer_pid'] <= 0 or type(message.get('ros_stamp_ns')) is not int or
            message['ros_stamp_ns'] < 0 or not isinstance(message.get('kind'), str) or
            not isinstance(message.get('payload'), dict)):
        raise ValueError('invalid version/session/producer/stamp/payload')
    sent = message.get('sent_monotonic_s')
    if not isinstance(sent, (int, float)) or not math.isfinite(sent) or sent < 0:
        raise ValueError('invalid monotonic timestamp')
    # 检查嵌套数据，禁止标准 json.loads 接受的 NaN/Infinity 漏入信封。
    encode(message)
    return message


def world_to_wire(world):
    return dict(epoch=world.epoch, resolution_m=world.resolution_m,
                blocked=[list(vars(r).values()) for r in world.blocked],
                known_free=[list(vars(r).values()) for r in world.known_free])


def world_from_wire(payload):
    return WorldSnapshot(payload['epoch'], tuple(CellRect(*r) for r in payload['blocked']),
                         tuple(CellRect(*r) for r in payload['known_free']), payload['resolution_m'])


def plan_to_wire(plan):
    return json.loads(encode(asdict(plan)))


def plan_from_wire(payload):
    restored = dict(payload)
    for field in ('times', 'predicted_initial', 'source_vector'):
        restored[field] = tuple(restored[field])
    restored['controls'] = tuple(tuple(c) for c in restored['controls'])
    return TimedPlan(**restored)


def verification_limits():
    return WheelLimits(linear_speed_mps=.5, angular_speed_rps=.5)


def clock_slot(elapsed_s, previous_slot):
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError('clock must be nonnegative finite')
    slot = math.floor((elapsed_s + 1e-9) / .05)
    if slot <= previous_slot:
        return None
    return slot, slot * .05, slot - previous_slot - 1


class PublicationGate:
    """只约束发送；调用方始终先执行 core.tick，再询问此门禁。"""
    def __init__(self):
        self.awaiting = None
        self.ack_accepted = False

    def sent(self, sequence):
        self.awaiting = sequence
        self.ack_accepted = False

    def ack(self, sequence, reason):
        if self.awaiting == sequence:
            if reason == 'ACCEPTED':
                self.ack_accepted = True
            else:
                self.awaiting = None
                self.ack_accepted = False

    def can_publish(self, feedback):
        if self.awaiting is not None and self.ack_accepted and feedback['sequence'] >= self.awaiting:
            self.awaiting = None
            self.ack_accepted = False
        return self.awaiting is None and feedback['pending'] is None


def fatal_protocol_reason(failures):
    """合法迟到只保留证据；来源身份/预测/接续等其他协议错误停止。"""
    return next((row['reason'] for row in failures if row['reason'] not in ('SOURCE_EXPIRED', 'LATE_DELIVERY')), None)


class SessionDeadline:
    """起钟证据到来后独立使用车辆原点，不再消耗冷启动剩余额度。"""
    def __init__(self, started_wall_s, startup_timeout_s, runtime_timeout_s):
        self.started_wall_s = started_wall_s
        self.startup_timeout_s = startup_timeout_s
        self.runtime_timeout_s = runtime_timeout_s
        self.origin_monotonic_s = None

    def observe_origin(self, origin_monotonic_s):
        if not math.isfinite(origin_monotonic_s) or origin_monotonic_s < self.started_wall_s:
            raise ValueError('invalid fixture origin')
        if self.origin_monotonic_s is not None and self.origin_monotonic_s != origin_monotonic_s:
            raise ValueError('fixture origin changed')
        self.origin_monotonic_s = origin_monotonic_s

    def expired(self, now_monotonic_s):
        if self.origin_monotonic_s is None:
            return 'STARTUP_TIMEOUT' if now_monotonic_s - self.started_wall_s >= self.startup_timeout_s else None
        return 'WALL_TIMEOUT' if now_monotonic_s - self.origin_monotonic_s >= self.runtime_timeout_s else None


def validate_environment(env):
    if env.get('ROS_DISTRO') != 'humble':
        raise ValueError('isolated verification requires ROS_DISTRO=humble')
    domain = int(env.get('ROS_DOMAIN_ID', '0'))
    if not 1 <= domain <= 232 or env.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('explicit nonzero isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 required')
