"""无 ROS 导入的 P3 平面转换与标准 Path 数据。

这里只保存最新变换，不作时间历史、严格同步或时间戳去重。ROS 消息实例
的构造由独立适配包承担；本层可直接接受同字段的普通对象作契约测试。
"""
from collections import deque
from dataclasses import dataclass
import math

from .contracts import VehicleState
from .pose_goal import normalize_yaw


def frame_name(value):
    return str(value or '').strip().lstrip('/')


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('pose/transform values must be finite')
    return value


def quaternion_yaw(rotation):
    """归一化非单位四元数；未填写方向按平面单位方向处理。"""
    x, y, z, w = (finite(getattr(rotation, key)) for key in ('x', 'y', 'z', 'w'))
    norm = math.hypot(x, y, z, w)
    if norm == 0:
        return 0.
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True)
class PoseGoal:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        object.__setattr__(self, 'x', finite(self.x))
        object.__setattr__(self, 'y', finite(self.y))
        object.__setattr__(self, 'yaw', normalize_yaw(self.yaw))


@dataclass(frozen=True)
class PlanarTransform:
    """target_from_source，z 仅转送地图高程原点。"""
    x: float = 0.
    y: float = 0.
    yaw: float = 0.
    z: float = 0.

    def compose(self, other):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return PlanarTransform(self.x + c * other.x - s * other.y,
                               self.y + s * other.x + c * other.y,
                               normalize_yaw(self.yaw + other.yaw), self.z + other.z)

    def inverse(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return PlanarTransform(-c * self.x - s * self.y, s * self.x - c * self.y,
                               -self.yaw, -self.z)


class MissingTransform(ValueError):
    pass


class LatestTransforms:
    """小型最新 TF 树；同一零 stamp 的后续消息仍覆盖旧值。"""
    def __init__(self):
        self._edges = {}

    def copy(self):
        result = LatestTransforms()
        result._edges = self._edges.copy()
        return result

    def update(self, message):
        for msg in message.transforms:
            parent, child = frame_name(msg.header.frame_id), frame_name(msg.child_frame_id)
            if not parent or not child or parent == child:
                continue
            value = msg.transform
            transform = PlanarTransform(finite(value.translation.x), finite(value.translation.y),
                                        quaternion_yaw(value.rotation), finite(value.translation.z))
            # TF 每个 child 只有一个当前父坐标系，不能留下已替换的反向边。
            self._edges[child] = (parent, transform)

    def lookup(self, target, source):
        target, source = frame_name(target), frame_name(source)
        if target == source:
            return PlanarTransform()
        graph = {}
        for child, (parent, transform) in self._edges.items():
            graph.setdefault(child, []).append((parent, transform))
            graph.setdefault(parent, []).append((child, transform.inverse()))
        queue, visited = deque([(source, PlanarTransform())]), {source}
        while queue:
            current, current_from_source = queue.popleft()
            for neighbor, neighbor_from_current in graph.get(current, ()):
                if neighbor in visited:
                    continue
                composed = neighbor_from_current.compose(current_from_source)
                if neighbor == target:
                    return composed
                visited.add(neighbor)
                queue.append((neighbor, composed))
        raise MissingTransform(f'等待最新 TF: {target} <- {source}')


def _map_pose(pose, transform):
    local = PlanarTransform(finite(pose.position.x), finite(pose.position.y), quaternion_yaw(pose.orientation))
    return transform.compose(local)


def goal_from_message(message, transforms, map_frame='map'):
    source = frame_name(message.header.frame_id) or map_frame
    pose = _map_pose(message.pose, transforms.lookup(map_frame, source))
    return PoseGoal(pose.x, pose.y, pose.yaw)


def state_from_odometry(message, transforms, map_frame='map'):
    source = frame_name(message.header.frame_id) or map_frame
    pose = _map_pose(message.pose.pose, transforms.lookup(map_frame, source))
    # Odometry 的 pose 在 header.frame_id，twist 在 child_frame_id。
    # 规划使用该车体纵向有符号速度，不取模长，也不要求 child 名称匹配配置。
    twist = message.twist.twist
    return VehicleState(pose.x, pose.y, pose.yaw, finite(twist.linear.x), finite(twist.angular.z))


def map_from_message(message, transforms, map_frame='map'):
    # ROS GridMap 的 header 位于消息本身；兼容直接带 info.header 的测试对象。
    header = getattr(message, 'header', None) or getattr(message.info, 'header', None)
    source = frame_name(getattr(header, 'frame_id', '')) or map_frame
    return transforms.lookup(map_frame, source)


@dataclass(frozen=True)
class PathPose:
    x: float
    y: float
    yaw: float
    stamp_ns: int


@dataclass(frozen=True)
class PathData:
    frame_id: str
    stamp_ns: int
    poses: tuple = ()


@dataclass(frozen=True)
class PathPair:
    global_route: PathData
    local: PathData
    global_reference: PathData | None = None


def paths(route, trajectory, ros_start_ns, map_frame='map', *, global_reference=None):
    """调用方只传完整认证候选；保留全部位姿，不能由切线替换车体 yaw。"""
    start = int(ros_start_ns)
    global_poses = []
    if route is not None:
        points = route.points
        for index, (x, y) in enumerate(points):
            a, b = (points[index], points[index + 1]) if index + 1 < len(points) else (points[-2], points[-1])
            global_poses.append(PathPose(float(x), float(y), math.atan2(b[1] - a[1], b[0] - a[0]), start))
    local_poses = []
    previous_ns = start
    if trajectory is not None:
        for index, (t, state) in enumerate(zip(trajectory.times, trajectory.states)):
            # 首点固定 t0；取最小严格递增整数 ns，连续量化冲突可累计数 ns。
            # 只修消息表示，不改原 times/状态，也不丢同 xy 的转向位姿。
            stamp_ns = start if index == 0 else max(start + round(float(t) * 1_000_000_000), previous_ns + 1)
            local_poses.append(PathPose(float(state[0]), float(state[1]), float(state[2]), stamp_ns))
            previous_ns = stamp_ns
    # 相同的几何参考交给MPCC截取与RViz；原始全局路线独立保留。
    reference_poses=[]
    if global_reference is not None:
        points=global_reference.points
        for index,(x,y) in enumerate(points):
            a,b=(points[index],points[index+1]) if index+1<len(points) else (points[-2],points[-1])
            reference_poses.append(PathPose(float(x),float(y),math.atan2(b[1]-a[1],b[0]-a[0]),start))
    return PathPair(PathData(map_frame, start, tuple(global_poses)), PathData(map_frame, start, tuple(local_poses)),
                    PathData(map_frame,start,tuple(reference_poses)))
