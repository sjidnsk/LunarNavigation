"""P3 GridMap 到不可变厘米矩形快照，不导入任何 ROS 消息或 DRL 包。

先在真实源分辨率累计高程并调用既有 core intrinsic 地形判定，再作几何
转换。厘米矩形只表达计算网格，不能提高观测精度。未知不填成自由；历史
按明确观测更新，源分辨率/旋转/滚动原点改变不重置世界地图。
邻域重分类只能更新仍属于该累计源格的区域；其他源格的新直接观测优先，
不能把旧高程中心的派生分类当作本帧重新观测。
"""

import math
import operator
import threading
from collections.abc import Mapping

import numpy as np

from .contracts import CellRect, CellState, WorldSnapshot
from .p3_geometry import continuous_strip_intervals
from .p3_sources import SourceContextLRU, retire_owners
from .terrain_native import NativeTerrain, TerrainUpdate


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _yaw(rotation):
    if rotation is None:
        return 0.
    x, y, z, w = (_finite(getattr(rotation, name), "quaternion") for name in ("x", "y", "z", "w"))
    norm = math.hypot(x, y, z, w)
    # 未填写的可选方向按平面单位姿态处理；非有限数据仍拒收。
    if norm == 0:
        return 0.
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    if abs(x) > 1e-8 or abs(y) > 1e-8:
        raise ValueError("P3 planar elevation geometry requires upright yaw rotation")
    return math.atan2(2 * w * z, 1 - 2 * z * z)


def _transform(value):
    """接受 (x,y,yaw)、平面对象及真实 Transform/TransformStamped。"""
    if value is None:
        return 0., 0., 0., 0.
    if isinstance(value, Mapping):
        return tuple(_finite(value.get(name, 0.), name) for name in ("x", "y", "yaw", "z"))
    if isinstance(value, (tuple, list, np.ndarray)):
        if len(value) != 3:
            raise ValueError("map_from_source tuple must contain x, y, yaw")
        return (*(_finite(item, "map_from_source") for item in value), 0.)
    if hasattr(value, "yaw"):
        return tuple(_finite(getattr(value, name, 0.), name) for name in ("x", "y", "yaw", "z"))
    transform = getattr(value, "transform", value)
    translation = transform.translation
    return (_finite(translation.x, "translation.x"), _finite(translation.y, "translation.y"),
            _yaw(transform.rotation), _finite(getattr(translation, "z", 0.), "translation.z"))


def _dimension(length, resolution):
    length = _finite(length, "grid length")
    count = length / resolution
    if not math.isfinite(count) or count <= 0:
        raise ValueError("grid length/resolution must define positive dimensions")
    rounded = round(count)
    if rounded < 1 or abs(count - rounded) > 1e-6 * max(1., abs(count)):
        raise ValueError("grid length and resolution do not define integer dimensions")
    return rounded


def _read_message(message, transform):
    """解包 grid_map 的负 x/y 索引和 circular buffer，得到升序 [y,x]。"""
    try:
        info = message.info
        resolution = _finite(info.resolution, "source resolution")
        if resolution <= 0:
            raise ValueError("source resolution must be positive")
        width, height = _dimension(info.length_x, resolution), _dimension(info.length_y, resolution)
        layers = list(message.layers)
        if len(layers) != len(message.data) or layers.count("elevation") != 1:
            raise ValueError("GridMap needs exactly one elevation layer with matching data")
        layer = message.data[layers.index("elevation")]
        layout = getattr(layer, "layout", None)
        dims = getattr(layout, "dim", ())
        row_major = False
        if dims:
            if len(dims) != 2:
                raise ValueError("elevation layout requires zero or two dimensions")
            labels = tuple(getattr(dim, "label", "") for dim in dims)
            column_labels = ("column_index", "row_index")
            row_labels = ("row_index", "column_index")
            column_ok = all(not actual or actual == expected for actual, expected in zip(labels, column_labels))
            row_ok = all(not actual or actual == expected for actual, expected in zip(labels, row_labels))
            if not column_ok and not row_ok:
                raise ValueError("unsupported elevation matrix layout labels")
            # 缺省/空标签采用标准 grid_map 列主序；明确行主序时才覆盖。
            row_major = row_ok and not column_ok
            sizes = (width, height) if row_major else (height, width)
            for dim, expected_size, expected_stride in zip(dims, sizes, (width * height, sizes[1])):
                size = operator.index(getattr(dim, "size", 0))
                stride = operator.index(getattr(dim, "stride", 0))
                if size not in (0, expected_size) or stride not in (0, expected_stride):
                    raise ValueError("elevation layout sizes/strides do not match geometry")
        offset = operator.index(getattr(layout, "data_offset", 0))
        buffer = np.asarray(layer.data, dtype=np.float32)
        if buffer.ndim != 1 or offset < 0 or buffer.size - offset != width * height:
            raise ValueError("elevation data length does not match geometry/layout")
        outer_start = operator.index(getattr(message, "outer_start_index", 0))
        inner_start = operator.index(getattr(message, "inner_start_index", 0))
        if not (0 <= outer_start < width and 0 <= inner_start < height):
            raise ValueError("circular buffer start index is outside geometry")
        rows = (width - 1 - np.arange(width) + outer_start) % width
        columns = (height - 1 - np.arange(height) + inner_start) % height
        indices = rows[None, :] * height + columns[:, None] if row_major else columns[:, None] * width + rows[None, :]
        elevation = np.ascontiguousarray(buffer[offset + indices], dtype=np.float32)
        pose = info.pose
        center_x = _finite(pose.position.x, "map center x")
        center_y = _finite(pose.position.y, "map center y")
        center_z = _finite(getattr(pose.position, "z", 0.), "map center z")
        pose_yaw = _yaw(getattr(pose, "orientation", None))
        c, s = math.cos(pose_yaw), math.sin(pose_yaw)
        half_x, half_y = width * resolution / 2, height * resolution / 2
        corner_x = center_x - c * half_x + s * half_y
        corner_y = center_y - s * half_x - c * half_y
        tx, ty, tf_yaw, tz = _transform(transform)
        c, s = math.cos(tf_yaw), math.sin(tf_yaw)
        origin = (c * corner_x - s * corner_y + tx, s * corner_x + c * corner_y + ty)
        if not all(math.isfinite(value) for value in origin):
            raise ValueError("transformed elevation origin is not finite")
        return elevation, resolution, origin, math.remainder(pose_yaw + tf_yaw, 2 * math.pi), center_z + tz
    except (AttributeError, TypeError, IndexError, OverflowError) as error:
        raise ValueError(f"invalid P3 GridMap geometry/layout: {error}") from error


def _paint(segments, x0, x1, state):
    """覆盖一行的闭开区间并合并相邻同类行程，不创建逐格对象。"""
    if x0 >= x1:
        return segments
    result = []
    for left, right, old_state in segments:
        if left < x0:
            result.append((left, min(right, x0), old_state))
        if right > x1:
            result.append((max(left, x1), right, old_state))
    result.append((x0, x1, state))
    result.sort()
    merged = []
    for left, right, value in result:
        if left >= right:
            continue
        if merged and merged[-1][1] == left and merged[-1][2] == value:
            merged[-1] = (merged[-1][0], right, value)
        else:
            merged.append((left, right, value))
    return tuple(merged)


def _rectangles(rows):
    """相同行程跨连续行合为矩形；大块平地只保留一个矩形。"""
    rectangles = []
    active = {}
    for y in sorted(rows):
        next_active = {}
        for left, right, state in rows[y]:
            key = (left, right, state)
            previous = active.get(key)
            if previous is not None and rectangles[previous][3] == y:
                rectangles[previous][3] = y + 1
                next_active[key] = previous
            else:
                next_active[key] = len(rectangles)
                rectangles.append([left, y, right, y + 1, state])
        active = next_active
    return rectangles


def _cross_section(polygon, y):
    points = []
    tolerance = 1e-9
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if abs(end[1] - start[1]) <= tolerance:
            if abs(y - start[1]) <= tolerance:
                points.extend((start[0], end[0]))
        elif min(start[1], end[1]) - tolerance <= y <= max(start[1], end[1]) + tolerance:
            t = min(1., max(0., (y - start[1]) / (end[1] - start[1])))
            points.append(start[0] + t * (end[0] - start[0]))
    return (min(points), max(points)) if points else None


def _polygon(region, update):
    x0, y0, x1, y1, state = region
    c, s = math.cos(update.yaw), math.sin(update.yaw)
    ox, oy = update.origin_xy_m
    r = update.resolution_m
    polygon = [((ox + c * x * r - s * y * r) * 100,
                (oy + s * x * r + c * y * r) * 100)
               for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
    if not all(math.isfinite(value) for point in polygon for value in point):
        raise ValueError("terrain region cannot be represented in centimeter coordinates")
    return polygon


def _intersect_intervals(left, right):
    result = []
    a = b = 0
    while a < len(left) and b < len(right):
        lo, hi = max(left[a][0], right[b][0]), min(left[a][1], right[b][1])
        if lo < hi:
            result.append((lo, hi))
        if left[a][1] < right[b][1]:
            a += 1
        else:
            b += 1
    return result


def _free_rows(regions, update):
    """自由并集在整条厘米条带内的共同 x 区间，消除来源矩形内部接缝。

    按所有顶点事件分段，并保持同一个自由连通分量的边界对应，证明整个
    条带连续覆盖。两端由不同分量覆盖不能证明中间自由。不密化世界地图。
    """
    bands = {}
    for region in regions:
        if region[4] != CellState.FREE:
            continue
        polygon = _polygon(region, update)
        minimum = min(point[1] for point in polygon)
        maximum = max(point[1] for point in polygon)
        for y in range(math.floor(minimum + 1e-9), math.ceil(maximum - 1e-9)):
            bands.setdefault(y, []).append((polygon, minimum, maximum))
    for y, polygons in bands.items():
        # 与行范围和截面的 1e-9 cm 容差一致：贴近整数边界的浮点顶点
        # 不应再切出无法表示中点的数值薄片；不移动真实顶点或合并内部缺口。
        levels = sorted({float(y), float(y + 1),
                         *(point[1] for polygon, _, _ in polygons for point in polygon
                           if y + 1e-9 < point[1] < y + 1 - 1e-9)})
        covered = None
        for lower_y, upper_y in zip(levels, levels[1:]):
            middle = (lower_y + upper_y) / 2
            active = [polygon for polygon, minimum, maximum in polygons if minimum < middle < maximum]
            if not active:
                covered = []
                break
            sections = [(_cross_section(polygon, lower_y), _cross_section(polygon, upper_y))
                        for polygon in active]
            intervals = continuous_strip_intervals((lo, hi) for lo, hi in sections
                                                   if lo is not None and hi is not None)
            covered = intervals if covered is None else _intersect_intervals(covered, intervals)
            if not covered:
                break
        for left, right in covered or ():
            x0, x1 = math.ceil(left - 1e-9), math.floor(right + 1e-9)
            if x0 < x1:
                yield y, x0, x1, int(CellState.FREE)


def _blocked_rows(region, update):
    polygon = _polygon(region, update)
    minimum_y, maximum_y = min(point[1] for point in polygon), max(point[1] for point in polygon)
    tolerance = 1e-9
    first = math.floor(minimum_y + tolerance)
    last = math.ceil(maximum_y - tolerance)
    for y in range(first, last):
        lo, hi = max(float(y), minimum_y), min(float(y + 1), maximum_y)
        lower, upper = _cross_section(polygon, lo), _cross_section(polygon, hi)
        if lower is None or upper is None:
            continue
        # 水平条带与矩形相交的所有 x 范围均标障碍，含条带内的顶点。
        xs = [*lower, *upper, *(point[0] for point in polygon if lo < point[1] < hi)]
        left, right = math.floor(min(xs) + tolerance), math.ceil(max(xs) - tolerance)
        if left < right:
            yield y, left, right, int(CellState.BLOCKED)


def _observation_rows(elevation, resolution, origin, yaw, update):
    """把本帧有限高程的覆盖放到 native 返回的同一源格坐标中。

    这里只记录观测范围，不作任何地形判定。native 的固定格角用于识别
    同一累计源格，整格平移的滚动窗口不能被误认成另一个来源。
    """
    if (resolution != update.resolution_m or
            abs(math.remainder(yaw - update.yaw, 2 * math.pi)) > 1e-8):
        raise ValueError("native terrain returned incompatible source lattice geometry")
    dx, dy = origin[0] - update.origin_xy_m[0], origin[1] - update.origin_xy_m[1]
    c, s = math.cos(update.yaw), math.sin(update.yaw)
    x, y = (c * dx + s * dy) / resolution, (-s * dx + c * dy) / resolution
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("native terrain returned incompatible source lattice origin")
    x0, y0 = round(x), round(y)
    if abs(x - x0) > 1e-6 or abs(y - y0) > 1e-6:
        raise ValueError("native terrain returned incompatible source lattice origin")
    rows = {}
    for row_index, row in enumerate(elevation):
        finite = np.isfinite(row)
        changes = np.flatnonzero(np.diff(np.r_[False, finite, False].astype(np.int8)))
        if changes.size:
            rows[y0 + row_index] = tuple((x0 + int(a), x0 + int(b), int(CellState.FREE))
                                         for a, b in changes.reshape(-1, 2))
    return rows


def _owned_intervals(owners, y, left, right, source):
    for x0, x1, owner in owners.get(y, ()):
        if x0 >= right:
            break
        lo, hi = max(left, x0), min(right, x1)
        if owner == source and lo < hi:
            yield lo, hi


def _record_observation_owners(previous, source, observations, fresh_observations, update):
    """记录最新直接观测来源；0 表示同一厘米格内有多个来源。

    亚厘米的局部新观测不能把整个厘米格授予旧源格：交叠处先标混合，
    只有最近外源冲突之后有限观测的并集覆盖整格，才授予单一来源。
    首次只覆盖部分未知面积仍可保留同源身份，使跨帧累计能补全未知。
    """
    owners = previous.copy()
    for region in observations:
        for y, left, right, _ in _blocked_rows(region, update):
            row = _paint(owners.get(y, ()), left, right, source)
            for x0, x1, old_source in previous.get(y, ()):
                if x0 >= right:
                    break
                lo, hi = max(left, x0), min(right, x1)
                if old_source != source and lo < hi:
                    row = _paint(row, lo, hi, 0)
            owners[y] = row
    for y, left, right, _ in _free_rows(fresh_observations, update):
        owners[y] = _paint(owners.get(y, ()), left, right, source)
    return owners


def _world_coverage(regions, update):
    """观测或派生区域触及的厘米范围，只保存稀疏行程。"""
    rows = {}
    for region in regions:
        for y, left, right, _ in _blocked_rows(region, update):
            rows[y] = _paint(rows.get(y, ()), left, right, int(CellState.FREE))
    return rows


def _world_coverage_in_source(world_rows, key):
    """把厘米冲突/更新范围保守投回一个源格，不密化世界地图。

    缩放后的临时几何仅用于复用矩形相交计算；坐标乘 100 后恰好是目标
    源格索引。这里不生成高程，也不在投影后重新作地形判定。
    """
    resolution, ox, oy, yaw = key
    c, s = math.cos(yaw), math.sin(yaw)
    scale = .01 / resolution
    relative = TerrainUpdate(.01 * scale, ((-c * ox - s * oy) * scale,
                                           (s * ox - c * oy) * scale), -yaw,
                             np.empty((0, 5), dtype=np.int64))
    return _world_coverage(_rectangles(world_rows), relative)


def _erase_coverage(previous, mask):
    rows = previous.copy()
    for y, intervals in mask.items():
        row = rows.get(y, ())
        for left, right, _ in intervals:
            remaining = []
            for x0, x1, state in row:
                if x0 < left:
                    remaining.append((x0, min(x1, left), state))
                if x1 > right:
                    remaining.append((max(x0, right), x1, state))
            row = tuple(remaining)
        if row:
            rows[y] = row
        else:
            rows.pop(y, None)
    return rows


def _merge_row_maps(previous, incoming):
    rows = previous.copy()
    for y, intervals in incoming.items():
        for left, right, state in intervals:
            rows[y] = _paint(rows.get(y, ()), left, right, state)
    return rows


def _clip_row_maps(values, mask):
    """仅遍历更新范围内的源行；保留源分类，mask 只决定覆盖范围。"""
    rows = {}
    for y in mask if len(mask) <= len(values) else values:
        intervals = mask.get(y, ())
        values_row = values.get(y, ())
        for left, right, _ in intervals:
            for x0, x1, state in values_row:
                if x0 >= right:
                    break
                lo, hi = max(left, x0), min(right, x1)
                if lo < hi:
                    rows[y] = _paint(rows.get(y, ()), lo, hi, state)
    return rows


def _cache_source_classifications(previous, regions):
    rows = previous.copy()
    # 保留 UNKNOWN，避免局部重分类后意外复活源格中的旧 FREE。
    for selected in (CellState.UNKNOWN, CellState.FREE, CellState.BLOCKED):
        for x0, y0, x1, y1, state in regions:
            if state != selected:
                continue
            for y in range(int(y0), int(y1)):
                rows[y] = _paint(rows.get(y, ()), int(x0), int(x1), int(state))
    return rows


class P3MapStore:
    """线程内累计入口：apply(message, map_from_source) -> WorldSnapshot。

    terrain 注入只用于显式隔离测试。生产缺省必须加载真实 NativeTerrain；
    snapshot 从 epoch=0 的全未知地图开始，source_resolution_m 在首帧前为 None。
    来源权限与状态分别保存为稀疏行程，不为每个厘米格长期分配对象。
    只有三态几何改变才产生新 epoch；原始高程和来源权限仍逐帧累计。
    """

    def __init__(self, *, terrain=None, library_path=None, capability=None):
        self._terrain = terrain if terrain is not None else NativeTerrain(library_path, capability=capability)
        self._rows = {}
        self._observation_owners = {}
        self._source_cache = SourceContextLRU()
        self._lattice_ids = self._source_cache.ids
        self._fresh_observations = {}
        self._source_classifications = {}
        self._snapshot = WorldSnapshot(0, (), ())
        self._source_resolution_m = None
        self._lock = threading.RLock()

    @property
    def snapshot(self) -> WorldSnapshot:
        return self._snapshot

    @property
    def source_resolution_m(self):
        return self._source_resolution_m

    def apply(self, message, transform=None) -> WorldSnapshot:
        elevation, resolution, origin, yaw, z = _read_message(message, transform)
        with self._lock:
            update = self._terrain.apply(elevation, resolution_m=resolution, origin_xy_m=origin,
                                         yaw=yaw, z_offset_m=z)
            if not isinstance(update, TerrainUpdate):
                raise ValueError("terrain backend must return validated intrinsic TerrainUpdate")
            observations = _observation_rows(elevation, resolution, origin, yaw, update)
            key = (update.resolution_m, *update.origin_xy_m, update.yaw)
            contexts = self._source_cache.copy()
            observation_regions = _rectangles(observations)
            fresh_sources = self._fresh_observations.copy()
            classifications = self._source_classifications.copy()
            rows = self._rows.copy()
            owners = self._observation_owners
            if observations:
                source, retired = contexts.activate(key, update)
                owners = retire_owners(owners, {identity for _, identity in retired})
                for old_key, _ in retired:
                    fresh_sources.pop(old_key, None)
                    classifications.pop(old_key, None)
                conflicts = _world_coverage(observation_regions, update)
                # 外源直接观测只使相交厘米区域的旧覆盖过期。未观测、远处
                # 更新以及旧 halo 都不能清空已经累计的新鲜部分。
                for other_key, previous in tuple(fresh_sources.items()):
                    if other_key != key:
                        fresh_sources[other_key] = _erase_coverage(
                            previous, _world_coverage_in_source(conflicts, other_key))
                fresh = _merge_row_maps(fresh_sources.get(key, {}), observations)
                fresh_sources[key] = fresh
                cached = _cache_source_classifications(classifications.get(key, {}), update.regions)
                classifications[key] = cached
                affected = _world_coverage(update.regions, update)
                mask = _world_coverage_in_source(affected, key)
                local = _clip_row_maps(cached, mask)
                regions = _rectangles(local)
                # native 只发布本次观测及变化邻域。厘米格可能由多个更细源格
                # 跨帧覆盖，所以用缓存补齐受影响厘米范围的最新 intrinsic
                # 分类，再以冲突后新鲜覆盖作准入，不引入旧覆盖清新障碍。
                eligible = _clip_row_maps(local, fresh)
                fresh_classified = _rectangles(eligible)
                for selected in (CellState.FREE, CellState.BLOCKED, CellState.UNKNOWN):
                    for packet, historical in ((regions, True), (fresh_classified, False)):
                        candidates = (_free_rows(packet, update) if selected == CellState.FREE else
                                      (cell for region in packet if region[4] == selected
                                       for cell in _blocked_rows(region, update)))
                        for y, left, right, state in candidates:
                            intervals = (_owned_intervals(owners, y, left, right, source)
                                         if historical else ((left, right),))
                            for lo, hi in intervals:
                                if selected == CellState.UNKNOWN:
                                    # 新有限 UNKNOWN 及仍有来源权限的 halo 撤销
                                    # 相交 FREE；只移除自由支持，已证 BLOCKED 保留。
                                    # 亚厘米部分未知也不能留下整厘米 FREE 证书。
                                    row = rows.get(y, ())
                                    remaining = []
                                    for x0, x1, old_state in row:
                                        if old_state != CellState.FREE or x1 <= lo or x0 >= hi:
                                            remaining.append((x0, x1, old_state))
                                        else:
                                            if x0 < lo: remaining.append((x0, lo, old_state))
                                            if x1 > hi: remaining.append((hi, x1, old_state))
                                    if remaining: rows[y] = tuple(remaining)
                                    else: rows.pop(y, None)
                                else:
                                    rows[y] = _paint(rows.get(y, ()), lo, hi, state)
                fresh_regions = _rectangles(_clip_row_maps(fresh, mask))
                owners = _record_observation_owners(owners, source,
                                                    observation_regions, fresh_regions, update)
            blocked, free = [], []
            for x0, y0, x1, y1, state in _rectangles(rows):
                (blocked if state == CellState.BLOCKED else free).append(CellRect(x0, y0, x1, y1))
            blocked, free = tuple(blocked), tuple(free)
            snapshot = self._snapshot
            if blocked != snapshot.blocked or free != snapshot.known_free:
                snapshot = WorldSnapshot(snapshot.epoch + 1, blocked, free)
            self._rows = rows
            self._observation_owners = owners
            self._fresh_observations = fresh_sources
            self._source_classifications = classifications
            self._source_cache = contexts
            self._lattice_ids = contexts.ids
            self._snapshot = snapshot
            self._source_resolution_m = resolution
            return snapshot
