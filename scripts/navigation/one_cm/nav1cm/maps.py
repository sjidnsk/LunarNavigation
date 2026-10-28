"""原生厘米矩形地图、局部窗口及保守净空场。"""

from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import distance_transform_edt

from .contracts import CellState, GridSnapshot, VehicleState, WorldSnapshot, _readonly_array


@dataclass(frozen=True)
class DistanceField:
    epoch: int
    origin_xy: tuple[float, float]
    resolution_m: float
    distance_m: np.ndarray = field(repr=False)
    gradient_x: np.ndarray = field(repr=False)
    gradient_y: np.ndarray = field(repr=False)

    def __post_init__(self):
        for name in ("distance_m", "gradient_x", "gradient_y"):
            object.__setattr__(self, name, _readonly_array(getattr(self, name), np.float64, 2))

    def sample(self, x: float, y: float) -> tuple[float, float, float]:
        """中心下界再减查询点偏移；图外的保守净空为零。"""
        ix = int(np.floor((x - self.origin_xy[0]) / self.resolution_m))
        iy = int(np.floor((y - self.origin_xy[1]) / self.resolution_m))
        if ix < 0 or iy < 0 or iy >= self.distance_m.shape[0] or ix >= self.distance_m.shape[1]:
            return 0.0, 0.0, 0.0
        center_x = self.origin_xy[0] + (ix + 0.5) * self.resolution_m
        center_y = self.origin_xy[1] + (iy + 0.5) * self.resolution_m
        lower = max(0.0, float(self.distance_m[iy, ix]) - float(np.hypot(x - center_x, y - center_y)))
        return lower, float(self.gradient_x[iy, ix]), float(self.gradient_y[iy, ix])


def _rect_slices(rect, origin, resolution, shape, *, contained):
    x0 = (rect.x0 * resolution - origin[0]) / resolution
    x1 = (rect.x1 * resolution - origin[0]) / resolution
    y0 = (rect.y0 * resolution - origin[1]) / resolution
    y1 = (rect.y1 * resolution - origin[1]) / resolution
    # 已知自由须整格落入来源；障碍只要与格面积相交即视为占据。
    if contained:
        ix0, ix1 = int(np.ceil(x0 - 1e-10)), int(np.floor(x1 + 1e-10))
        iy0, iy1 = int(np.ceil(y0 - 1e-10)), int(np.floor(y1 + 1e-10))
    else:
        ix0, ix1 = int(np.floor(x0 + 1e-10)), int(np.ceil(x1 - 1e-10))
        iy0, iy1 = int(np.floor(y0 + 1e-10)), int(np.ceil(y1 - 1e-10))
    height, width = shape
    ix0, ix1 = min(width, max(0, ix0)), min(width, max(0, ix1))
    iy0, iy1 = min(height, max(0, iy0)), min(height, max(0, iy1))
    if ix0 >= ix1 or iy0 >= iy1:
        return slice(0, 0), slice(0, 0)
    return slice(iy0, iy1), slice(ix0, ix1)


def extract_window(world: WorldSnapshot, state: VehicleState, allow_unknown: bool) -> GridSnapshot:
    """提取以车体中心为中心的轴对齐 5 m 窗口，不改变来源 UNKNOWN。"""
    resolution = world.resolution_m
    origin = (state.x - 2.5, state.y - 2.5)
    cells = np.full((500, 500), CellState.UNKNOWN, dtype=np.int8)
    # 先在原生厘米格上合并自由区域，再要求目标格相交的每个源格均自由。
    # 目标格可跨多个相接矩形；逐矩形 contained 会制造一格宽 UNKNOWN 接缝。
    # 只密化局部支持范围（至多 501²），保持精确车体中心和真实未知边界。
    scaled = np.asarray(origin) / resolution
    lower = np.floor(scaled + 1e-10).astype(np.int64)
    upper = np.ceil(scaled - 1e-10).astype(np.int64)
    dx, dy = (upper - lower).tolist()
    support = np.zeros((500 + dy, 500 + dx), dtype=bool)
    for rect in world.known_free:
        x0, x1 = max(0, rect.x0 - int(lower[0])), min(support.shape[1], rect.x1 - int(lower[0]))
        y0, y1 = max(0, rect.y0 - int(lower[1])), min(support.shape[0], rect.y1 - int(lower[1]))
        if x0 < x1 and y0 < y1:
            support[y0:y1, x0:x1] = True
    free = support[:500, :500].copy()
    for sy in range(dy + 1):
        for sx in range(dx + 1):
            free &= support[sy:sy + 500, sx:sx + 500]
    cells[free] = CellState.FREE
    for rect in world.blocked:
        cells[_rect_slices(rect, origin, resolution, cells.shape, contained=False)] = CellState.BLOCKED
    return GridSnapshot(world.epoch, origin, resolution, cells, bool(allow_unknown))


def build_distance_field(grid: GridSnapshot, *, backend=None) -> DistanceField:
    """对占据格面积及窗口边界给出不高估的欧氏净空下界。"""
    if backend is not None:
        return backend.build(grid)
    passable = grid.cells != CellState.BLOCKED
    if not grid.allow_unknown:
        passable &= grid.cells != CellState.UNKNOWN
    if np.any(~passable):
        centers = distance_transform_edt(passable) * grid.resolution_m
        # 每个障碍格均位于其中心半对角线以内，故此减项保证下界。
        lower = np.maximum(0.0, centers - np.sqrt(2.0) * grid.resolution_m / 2.0)
    else:
        lower = np.full(grid.cells.shape, np.inf, dtype=np.float64)
    height, width = grid.cells.shape
    ys = (np.arange(height) + 0.5) * grid.resolution_m
    xs = (np.arange(width) + 0.5) * grid.resolution_m
    edge_y = np.minimum(ys, height * grid.resolution_m - ys)
    edge_x = np.minimum(xs, width * grid.resolution_m - xs)
    lower = np.minimum(lower, edge_y[:, None])
    lower = np.minimum(lower, edge_x[None, :])
    grad_y, grad_x = np.gradient(lower, grid.resolution_m)
    return DistanceField(grid.epoch, grid.origin_xy, grid.resolution_m, lower, grad_x, grad_y)


class DistanceFieldCache:
    """单项几何缓存；完整可通行位图相等才复用不可变距离与梯度。"""

    def __init__(self, backend=None):
        self._backend = backend
        self._key = self._field = None
        self.last_reused = False

    def get(self, grid: GridSnapshot) -> DistanceField:
        passable = grid.cells != CellState.BLOCKED
        if not grid.allow_unknown:
            passable &= grid.cells != CellState.UNKNOWN
        key = (grid.cells.shape, grid.resolution_m, grid.allow_unknown, passable.tobytes())
        self.last_reused = key == self._key
        if not self.last_reused:
            self._field = (build_distance_field(grid) if self._backend is None else
                           build_distance_field(grid, backend=self._backend))
            self._key = key
            return self._field
        # 几何数组已由 DistanceField 冻结；只重绑定当前窗口身份，不再复制数组。
        rebound = object.__new__(DistanceField)
        for name, value in (("epoch", grid.epoch), ("origin_xy", grid.origin_xy),
                            ("resolution_m", grid.resolution_m),
                            ("distance_m", self._field.distance_m),
                            ("gradient_x", self._field.gradient_x),
                            ("gradient_y", self._field.gradient_y)):
            object.__setattr__(rebound, name, value)
        return rebound
