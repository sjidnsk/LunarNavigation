"""独立厘米地图导航的数据契约；坐标、速度和时间均采用 SI 单位。"""

from dataclasses import dataclass, field
from enum import Enum, IntEnum
import hashlib
from typing import Tuple

import numpy as np


def _readonly_array(value, dtype, ndim=None, columns=None):
    """用不可写的 bytes 持有数据，避免源数组和 setflags 再次打开写权限。"""
    array = np.asarray(value, dtype=dtype)
    if ndim is not None and array.ndim != ndim:
        raise ValueError(f"array must have {ndim} dimensions")
    if columns is not None and array.shape[-1] != columns:
        raise ValueError(f"array must have {columns} columns")
    copy = np.ascontiguousarray(array).tobytes()
    return np.frombuffer(copy, dtype=dtype).reshape(array.shape)


class CellState(IntEnum):
    UNKNOWN = -1
    FREE = 0
    BLOCKED = 1


class CertificationReason(str, Enum):
    VALID = "VALID"
    EPOCH_MISMATCH = "EPOCH_MISMATCH"
    ROUTE_VERSION_MISMATCH = "ROUTE_VERSION_MISMATCH"
    INVALID_SHAPE = "INVALID_SHAPE"
    NONFINITE = "NONFINITE"
    TIME_NONMONOTONIC = "TIME_NONMONOTONIC"
    LIMIT = "LIMIT"
    MODE = "MODE"
    DYNAMICS = "DYNAMICS"
    COLLISION = "COLLISION"
    UNKNOWN = "UNKNOWN"
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    BRAKING_COLLISION = "BRAKING_COLLISION"
    BRAKING_UNKNOWN = "BRAKING_UNKNOWN"
    BRAKING_OUT_OF_BOUNDS = "BRAKING_OUT_OF_BOUNDS"
    BRAKING_LIMIT = "BRAKING_LIMIT"


def circumscribed_radius(body_length_m, body_width_m, minimum_clearance_m=0.0):
    """任意朝向车体外接圆加一次几何净空；不包含离散/执行数值余量。"""
    length,width,clearance=map(float,(body_length_m,body_width_m,minimum_clearance_m))
    if not all(np.isfinite((length,width,clearance))) or min(length,width)<=0 or clearance<0:
        raise ValueError('body dimensions must be positive and clearance nonnegative')
    return float(np.hypot(length,width)/2+clearance)


@dataclass(frozen=True)
class WheelLimits:
    body_length_m: float = 1.182
    body_width_m: float = 0.818
    linear_speed_mps: float = 0.2
    angular_speed_rps: float = 0.1
    linear_accel_mps2: float = 0.5
    angular_accel_rps2: float = 0.5
    curvature_inv_m: float = 1.0
    minimum_clearance_m: float = 0.0
    motion_model: str = "curvature_limited"

    def __post_init__(self):
        if self.motion_model not in ("differential_drive", "curvature_limited"):
            raise ValueError("unknown wheel motion_model")
        numeric = {name: value for name, value in vars(self).items() if name != "motion_model"}
        values = tuple(numeric.values())
        if (not all(np.isfinite(values)) or self.minimum_clearance_m < 0 or
                any(value <= 0 for name,value in numeric.items() if name != "minimum_clearance_m")):
            raise ValueError("wheel limits must be finite and positive")

    def global_clearance_radius(self, requested=None):
        # 零净空纯原型保留显式半车宽参考；生产非零净空采用任意yaw外接圆下限。
        base = self.body_width_m / 2
        required = (circumscribed_radius(self.body_length_m,self.body_width_m,self.minimum_clearance_m)
                    if self.minimum_clearance_m > 0 else base)
        radius = required if requested is None else float(requested)
        if not np.isfinite(radius) or radius < base:
            raise ValueError("clearance radius must be finite and at least half vehicle width")
        return float(max(radius,required))

    def digest(self) -> str:
        """认证时绑定全部车体尺寸及运动约束。"""
        values = np.asarray((
            self.body_length_m, self.body_width_m, self.linear_speed_mps,
            self.angular_speed_rps, self.linear_accel_mps2,
            self.angular_accel_rps2, self.curvature_inv_m, self.minimum_clearance_m,
        ), dtype="<f8")
        return hashlib.sha256(b"WheelLimits-v4-motion-model:" + self.motion_model.encode("ascii") + b":" + values.tobytes()).hexdigest()


@dataclass(frozen=True)
class VehicleState:
    x: float
    y: float
    yaw: float
    v: float
    w: float

    def __post_init__(self):
        if not all(np.isfinite(tuple(vars(self).values()))):
            raise ValueError("vehicle state must be finite")


@dataclass(frozen=True)
class CellRect:
    """原生整数厘米矩形，闭开区间 [x0,x1) × [y0,y1)。"""

    x0: int
    y0: int
    x1: int
    y1: int

    def __post_init__(self):
        if any(type(value) is not int for value in vars(self).values()):
            raise ValueError("cell rectangle coordinates must be integers")
        if self.x0 >= self.x1 or self.y0 >= self.y1:
            raise ValueError("cell rectangle must have positive area")


@dataclass(frozen=True)
class WorldSnapshot:
    epoch: int
    blocked: Tuple[CellRect, ...]
    known_free: Tuple[CellRect, ...]
    resolution_m: float = 0.01

    def __post_init__(self):
        if type(self.epoch) is not int or self.epoch < 0:
            raise ValueError("epoch must be a nonnegative integer")
        if self.resolution_m != 0.01:
            raise ValueError("native sparse rectangles require 0.01 m resolution")
        blocked = tuple(self.blocked)
        known_free = tuple(self.known_free)
        if not all(isinstance(rect, CellRect) for rect in blocked + known_free):
            raise ValueError("world regions must be CellRect objects")
        object.__setattr__(self, "blocked", blocked)
        object.__setattr__(self, "known_free", known_free)


@dataclass(frozen=True)
class GridSnapshot:
    epoch: int
    origin_xy: Tuple[float, float]
    resolution_m: float
    cells: np.ndarray = field(repr=False)
    allow_unknown: bool = False
    _digest: str = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        origin = tuple(float(x) for x in self.origin_xy)
        if len(origin) != 2 or not all(np.isfinite(origin)):
            raise ValueError("origin_xy must contain finite x and y")
        if type(self.epoch) is not int or self.epoch < 0:
            raise ValueError("epoch must be a nonnegative integer")
        if not np.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("resolution must be finite and positive")
        raw = np.asarray(self.cells)
        if (raw.ndim != 2 or 0 in raw.shape or
                not np.issubdtype(raw.dtype, np.integer) or
                not np.isin(raw, [-1, 0, 1]).all()):
            raise ValueError("cells must contain only raw integer UNKNOWN/FREE/BLOCKED values")
        cells = _readonly_array(raw, np.int8, 2)
        object.__setattr__(self, "origin_xy", origin)
        object.__setattr__(self, "cells", cells)
        digest = hashlib.sha256(b"GridSnapshot-v1")
        digest.update(self.epoch.to_bytes(8, "little", signed=False))
        digest.update(np.asarray((*origin, self.resolution_m), dtype="<f8").tobytes())
        digest.update(np.asarray(cells.shape, dtype="<i8").tobytes())
        digest.update(b"\x01" if self.allow_unknown else b"\x00")
        digest.update(cells.tobytes())
        object.__setattr__(self, "_digest", digest.hexdigest())

    def digest(self) -> str:
        return self._digest

    def cell_at_world(self, x: float, y: float) -> CellState:
        ix = int(np.floor((x - self.origin_xy[0]) / self.resolution_m))
        iy = int(np.floor((y - self.origin_xy[1]) / self.resolution_m))
        if ix < 0 or iy < 0 or ix >= self.cells.shape[1] or iy >= self.cells.shape[0]:
            return CellState.UNKNOWN
        return CellState(int(self.cells[iy, ix]))


@dataclass(frozen=True)
class Route:
    version: int
    map_epoch: int
    points: np.ndarray = field(repr=False)
    goal: Tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        points = _readonly_array(self.points, np.float64, 2, 2)
        goal = tuple(float(x) for x in self.goal)
        if points.shape[0] < 2 or not np.isfinite(points).all():
            raise ValueError("route needs at least two finite points")
        if np.any(np.linalg.norm(np.diff(points, axis=0), axis=1) <= 0):
            raise ValueError("route segments must have positive length")
        if len(goal) != 2 or not all(np.isfinite(goal)):
            raise ValueError("goal must be finite xy")
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "goal", goal)


@dataclass(frozen=True)
class ProgressToken:
    route_version: int
    s: float


@dataclass(frozen=True)
class LocalReference:
    route_version: int
    map_epoch: int
    points: np.ndarray = field(repr=False)
    s: np.ndarray = field(repr=False)
    progress_s: float = 0.0
    terminal_yaw: float | None = None

    def __post_init__(self):
        points = _readonly_array(self.points, np.float64, 2, 2)
        s = _readonly_array(self.s, np.float64, 1)
        if len(points) != len(s) or not np.isfinite(points).all() or not np.isfinite(s).all():
            raise ValueError("reference points and arc lengths must be finite and aligned")
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "s", s)
        if self.terminal_yaw is not None:
            if not np.isfinite(self.terminal_yaw):
                raise ValueError("terminal yaw must be finite")
            object.__setattr__(self, "terminal_yaw", float(np.arctan2(
                np.sin(self.terminal_yaw), np.cos(self.terminal_yaw))))


@dataclass(frozen=True)
class Trajectory:
    map_epoch: int
    route_version: int
    times: np.ndarray = field(repr=False)
    states: np.ndarray = field(repr=False)
    controls: np.ndarray = field(repr=False)
    mode: str = "forward"
    source: str = "candidate"

    def __post_init__(self):
        # 非有限值交由独立认证给出明确的 NONFINITE 原因。
        object.__setattr__(self, "times", _readonly_array(self.times, np.float64, 1))
        object.__setattr__(self, "states", _readonly_array(self.states, np.float64, 2, 6))
        object.__setattr__(self, "controls", _readonly_array(self.controls, np.float64, 2, 3))
        if not isinstance(self.source, str) or not self.source:
            raise ValueError("trajectory source must be nonempty")

    def digest(self) -> str:
        """把身份、模式、来源及全部轨迹数值绑定到稳定内容哈希。"""
        digest = hashlib.sha256()
        for item in (str(self.map_epoch), str(self.route_version), self.mode, self.source):
            encoded = item.encode("utf-8")
            digest.update(len(encoded).to_bytes(8, "little"))
            digest.update(encoded)
        for array in (self.times, self.states, self.controls):
            digest.update(np.asarray(array.shape, dtype="<i8").tobytes())
            digest.update(array.astype("<f8", copy=False).tobytes())
        return digest.hexdigest()


@dataclass(frozen=True)
class Certification:
    valid: bool
    reason: CertificationReason
    brake_time_s: float = 0.0
    checked_poses: int = 0
    detail: str = ""
    map_epoch: int = -1
    route_version: int = -1
    trajectory_source: str = ""
    trajectory_digest: str = ""
    grid_digest: str = ""
    limits_digest: str = ""

    def matches(self, trajectory: Trajectory, grid: GridSnapshot,
                limits: WheelLimits, route_version: int) -> bool:
        """唯一可复用门禁：成功状态与轨迹、完整地图、约束均须完全相同。"""
        return bool(self.valid and self.reason == CertificationReason.VALID and
                    self.map_epoch == grid.epoch and self.route_version == route_version and
                    self.trajectory_digest == trajectory.digest() and
                    self.grid_digest == grid.digest() and
                    self.limits_digest == limits.digest())


def resolve_minimum_clearance(configured, requested=None):
    """选定能力的净空是下限；显式请求可收紧，不能静默取消配置安全量。"""
    configured = float(configured)
    requested = configured if requested is None else float(requested)
    if not np.isfinite(configured) or not np.isfinite(requested) or min(configured,requested) < 0:
        raise ValueError("minimum clearance must be finite and nonnegative")
    return max(configured,requested)
