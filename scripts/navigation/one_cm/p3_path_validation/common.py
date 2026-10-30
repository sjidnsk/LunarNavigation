"""只在本验证目录使用的证据写入、输入场景和消息序列化。"""
import json
import math
import os
from pathlib import Path
import threading
import time


TOPICS = dict(grid='/Car/T3/mapping/grid_map', odom='/Car/T3/localization/odometry',
              goal='/Car/T4/navigation/goal', local='/Car/T4/planning/local_path',
              global_route='/Car/T4/planning/global_route')
CASES = ('contract', 'legacy_tf', 'lifecycle', 'forward_spin', 'reverse', 'empty_smoke')
DURATIONS = dict(contract=29., legacy_tf=10., lifecycle=27., forward_spin=24., reverse=21., empty_smoke=7.)


def json_default(value):
    if hasattr(value, 'tolist'):
        return value.tolist()
    if hasattr(value, '__dict__'):
        return vars(value)
    raise TypeError(type(value).__name__)


class Recorder:
    def __init__(self, folder, role):
        self.folder = Path(folder)
        self.role = role
        self.stream = (self.folder / (role + '.jsonl')).open('x', encoding='utf-8')
        self.lock = threading.Lock()
        self.emit('IDENTITY', pid=os.getpid(), ppid=os.getppid(), pgid=os.getpgrp(),
                  domain=os.environ.get('ROS_DOMAIN_ID'), distro=os.environ.get('ROS_DISTRO'),
                  wall_ns=time.time_ns(), monotonic_s=time.monotonic(),
                  start_ticks=Path('/proc/self/stat').read_text().split()[21])

    def emit(self, kind, **data):
        row = dict(kind=kind, monotonic_s=time.monotonic(), wall_ns=time.time_ns())
        row.update(data)
        with self.lock:
            self.stream.write(json.dumps(row, default=json_default, allow_nan=False) + '\n')
            self.stream.flush()

    def close(self):
        self.stream.close()


def obstacles(case, elapsed):
    if case != 'lifecycle':
        return []
    # 完整墙环，不能因全局允许 UNKNOWN 而从有限地图外绕过。
    rects = [(1.2, -1.4, 1.4, 1.4), (-1.4, -1.4, 1.4, -1.2), (-1.4, 1.2, 1.4, 1.4)]
    if elapsed < 17.:
        rects.append((-1.4, -1.4, -1.2, 1.4))
    else:
        rects.extend([(-1.4, -1.4, -1.2, -1.), (-1.4, 1., -1.2, 1.4)])
    return rects


def transform(case, elapsed):
    if case in ('contract', 'legacy_tf'):
        cutoff = 12. if case=='contract' else 5.
        return (.15, -.08, .1) if elapsed<cutoff else (.18, -.05, .12)
    return (0., 0., 0.)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def path_data(message):
    return dict(frame=message.header.frame_id, stamp_ns=stamp_ns(message.header.stamp),
                poses=[dict(x=p.pose.position.x, y=p.pose.position.y, z=p.pose.position.z,
                            q=[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w],
                            stamp_ns=stamp_ns(p.header.stamp), frame=p.header.frame_id) for p in message.poses])


def yaw_of(quaternion):
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2))
