"""不可变执行身份；Path 的全部原始 double 和 ROS header 参与绑定。"""
from dataclasses import dataclass
import hashlib
import math
import re
import struct

FOLLOW, BRAKE, HOLD, FINAL_ALIGN, INVALIDATED = range(5)


def execution_session_id(task_id, generation):
    if type(generation) is not int or generation < 0:
        raise ValueError('invalid generation')
    if isinstance(task_id,str) and re.fullmatch('[0-9a-f]{32}',task_id):
        return task_id
    return hashlib.sha256((str(task_id)+'\0'+str(generation)).encode('utf-8')).hexdigest()[:32]


def path_fingerprint(path):
    digest = hashlib.sha256()
    def header(value):
        frame = value.frame_id.encode('utf-8')
        digest.update(struct.pack('!Q', len(frame)))
        digest.update(frame)
        digest.update(struct.pack('!qI', value.stamp.sec, value.stamp.nanosec))
    header(path.header)
    digest.update(struct.pack('!Q', len(path.poses)))
    for item in path.poses:
        header(item.header)
        p, q = item.pose.position, item.pose.orientation
        digest.update(struct.pack('!7d', p.x,p.y,p.z,q.x,q.y,q.z,q.w))
    return digest.hexdigest()


def _identity(value):
    if not isinstance(value.session_id,str) or re.fullmatch('[0-9a-f]{32}',value.session_id) is None:
        raise ValueError('invalid session identity')
    if type(value.revision) is not int or value.revision <= 0:
        raise ValueError('invalid revision')
    if not isinstance(value.path_digest,str) or re.fullmatch('[0-9a-f]{64}',value.path_digest) is None:
        raise ValueError('invalid path digest')
    if type(value.intent) is not int or value.intent not in range(5):
        raise ValueError('invalid intent')
    if type(value.stamp_ns) is not int or value.stamp_ns <= 0:
        raise ValueError('invalid stamp')


def _number(value, nonnegative=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or (nonnegative and value <= 0):
        raise ValueError('invalid finite number')


@dataclass(frozen=True)
class ExecutionReference:
    session_id: str
    revision: int
    path_digest: str
    intent: int
    stamp_ns: int
    position_tolerance_m: float = .1
    yaw_tolerance_rad: float = .05
    stopped_linear_mps: float = .01
    stopped_angular_rps: float = .01

    def __post_init__(self):
        _identity(self)
        for value in (self.position_tolerance_m,self.yaw_tolerance_rad,self.stopped_linear_mps,self.stopped_angular_rps):
            _number(value,True)


@dataclass(frozen=True)
class ExecutionStatus:
    session_id: str
    revision: int
    path_digest: str
    intent: int
    state: int
    accepted: bool
    stamp_ns: int
    reason: str = ''
    linear_speed_mps: float = 0.
    angular_speed_radps: float = 0.

    def __post_init__(self):
        _identity(self)
        if type(self.state) is not int or self.state not in range(7) or type(self.accepted) is not bool or not isinstance(self.reason,str):
            raise ValueError('invalid execution state')
        _number(self.linear_speed_mps)
        _number(self.angular_speed_radps)
