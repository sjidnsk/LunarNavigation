"""无 ROS/DRL 依赖的真实 core 地形桥；没有 Python 物理判定回退。

NativeTerrain 在源分辨率累计高程并返回未按车体膨胀的 intrinsic 区域。
分辨率、朝向或亚格相位改变时，由桥在容量 8 的 LRU 中维护独立源格，厘米
历史由 P3MapStore 合并。原生库可显式传入，或通过 NAV1CM_TERRAIN_LIB 指定。
"""

import ctypes
from dataclasses import dataclass, field
import hashlib
import os
from pathlib import Path
import threading
from typing import Mapping
import weakref

import numpy as np

from .contracts import _readonly_array


@dataclass(frozen=True)
class WheelTerrainCapability:
    # 保留旧类型名/三参数构造兼容；结构化平台配置启用尺寸感知窗口。
    maximum_slope_rad: float
    maximum_local_obstacle_relief_m: float
    minimum_underbody_clearance_m: float
    body_long_side_m: float | None = None
    platform_kind: str = "wheel"

    def __post_init__(self):
        values=(self.maximum_slope_rad,self.maximum_local_obstacle_relief_m,self.minimum_underbody_clearance_m)
        if not all(np.isfinite(values)) or any(value < 0 for value in values):
            raise ValueError("terrain limits must be finite and nonnegative")
        if self.platform_kind not in ("wheel","legged"):
            raise ValueError("unknown terrain platform kind")
        if self.body_long_side_m is not None and (not np.isfinite(self.body_long_side_m) or self.body_long_side_m <= 0):
            raise ValueError("platform long side must be finite and positive")

    @classmethod
    def from_mapping(cls, mapping: Mapping):
        values=mapping.get("capability",mapping)
        kind=mapping.get("platform", "legged" if "maximum_step_height_m" in values else "wheel")
        extent=values.get("body_extent_m")
        if extent is not None:
            if len(extent)<2 or not np.isfinite(extent[:2]).all() or max(extent[:2])<=0 or min(extent[:2])<=0:
                raise ValueError("invalid platform planar body extent")
            long_side=float(max(extent[:2]))
        elif "platform" in mapping:
            raise ValueError("platform terrain requires body_extent_m")
        else:
            long_side=values.get("body_long_side_m")
        return cls(float(values["maximum_slope_rad"]),
            float(values["maximum_step_height_m"] if kind=="legged" else values["maximum_local_obstacle_relief_m"]),
            float(values["minimum_body_clearance_m"] if kind=="legged" else values["minimum_underbody_clearance_m"]),
            long_side,kind)


def load_wheel_capability(path=None) -> WheelTerrainCapability:
    """读取既有 wheel.yaml；安装包可传配置路径或设置 NAV1CM_WHEEL_CONFIG。"""
    import yaml
    selected = path or os.environ.get("NAV1CM_WHEEL_CONFIG")
    if selected is None:
        selected = Path(__file__).resolve().parents[4] / "config/wheel.yaml"
    with Path(selected).open(encoding="utf-8") as stream:
        return WheelTerrainCapability.from_mapping(yaml.safe_load(stream))


MEASUREMENT_DTYPE = np.dtype([("x", np.int64), ("y", np.int64), ("height_m", np.float64),
                              ("state", np.int64), ("slope_rad", np.float64),
                              ("relief_m", np.float64)])


@dataclass(frozen=True)
class TerrainUpdate:
    """源格闭开矩形 [x0,y0,x1,y1,state]；state 使用 nav1cm 的 -1/0/1。"""

    resolution_m: float
    origin_xy_m: tuple[float, float]
    yaw: float
    regions: np.ndarray = field(repr=False)
    source_id: int | None = None
    evicted_source_ids: tuple[int, ...] = ()
    cache_capacity: int = 8
    measurements: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=MEASUREMENT_DTYPE), repr=False)

    def __post_init__(self):
        origin = tuple(float(value) for value in self.origin_xy_m)
        if (len(origin) != 2 or not all(np.isfinite(origin)) or
                not np.isfinite(self.resolution_m) or self.resolution_m <= 0 or
                not np.isfinite(self.yaw)):
            raise ValueError("terrain update geometry must be finite with positive resolution")
        regions = np.asarray(self.regions)
        if (regions.ndim != 2 or regions.shape[1] != 5 or
                not np.issubdtype(regions.dtype, np.signedinteger) or
                np.any(regions[:, 0] >= regions[:, 2]) or
                np.any(regions[:, 1] >= regions[:, 3]) or
                not np.isin(regions[:, 4], [-1, 0, 1]).all()):
            raise ValueError("terrain regions must be integer positive rectangles with -1/0/1 state")
        measurements = np.asarray(self.measurements, dtype=MEASUREMENT_DTYPE)
        if measurements.ndim != 1:
            raise ValueError("terrain measurements must be a one-dimensional structured array")
        # bytes 拥有独立不可写内存，后续 native apply 不会修改此快照。
        object.__setattr__(self, "measurements", np.frombuffer(measurements.tobytes(), dtype=MEASUREMENT_DTYPE))
        object.__setattr__(self, "origin_xy_m", origin)
        object.__setattr__(self, "regions", _readonly_array(regions, np.int64, 2, 5))


class _NativeResult(ctypes.Structure):
    _fields_ = [("resolution_m", ctypes.c_double), ("origin_x_m", ctypes.c_double),
                ("origin_y_m", ctypes.c_double), ("yaw", ctypes.c_double),
                ("regions", ctypes.POINTER(ctypes.c_int64)),
                ("region_count", ctypes.c_size_t), ("lattice_count", ctypes.c_size_t),
                ("source_id", ctypes.c_uint64), ("evicted_source_id", ctypes.c_uint64),
                ("cache_capacity", ctypes.c_size_t)]


class NativeTerrain:
    """单实例串行调用的 C ABI 包装；close 或上下文管理器释放累计源地图。"""

    def __init__(self, library_path=None, *, capability=None):
        selected = library_path or os.environ.get("NAV1CM_TERRAIN_LIB")
        if selected is None:
            directory = Path(__file__).resolve().parent
            selected = next((path for name in ("libnav1cm_terrain.so", "nav1cm_terrain.dll", "libnav1cm_terrain.dylib")
                             if (path := directory / name).is_file()), None)
        if selected is None:
            raise RuntimeError("native terrain library unavailable; build the bridge and set NAV1CM_TERRAIN_LIB")
        try:
            self.library_path = Path(selected).resolve(strict=True)
            self.library_sha256 = hashlib.sha256(self.library_path.read_bytes()).hexdigest()
            self._lib = ctypes.CDLL(str(self.library_path))
        except OSError as error:
            raise OSError(f"native terrain library unavailable: {selected}: {error}") from error
        version = self._lib.nav1cm_terrain_abi_version
        version.argtypes, version.restype = [], ctypes.c_int
        if version() != 2:
            raise RuntimeError("unsupported native terrain ABI version")
        self._error = self._lib.nav1cm_terrain_last_error_v2
        self._error.argtypes, self._error.restype = [], ctypes.c_char_p
        create = self._lib.nav1cm_terrain_create_v2
        create.argtypes, create.restype = [ctypes.c_double] * 3, ctypes.c_void_p
        destroy = self._lib.nav1cm_terrain_destroy_v2
        destroy.argtypes, destroy.restype = [ctypes.c_void_p], None
        self._apply = self._lib.nav1cm_terrain_apply_v2
        self._apply.argtypes = [ctypes.c_void_p,
                               np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags="C_CONTIGUOUS"),
                               ctypes.c_size_t, ctypes.c_size_t] + [ctypes.c_double] * 5 + [ctypes.POINTER(_NativeResult)]
        self._apply.restype = ctypes.c_int
        self._measurements = self._lib.nav1cm_terrain_measurements_v3
        self._measurements.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p),
                                       ctypes.POINTER(ctypes.c_size_t)]
        self._measurements.restype = ctypes.c_int
        self._polygon = self._lib.nav1cm_polygon_boolean_v1
        double_ptr = ctypes.POINTER(ctypes.c_double)
        self._polygon.argtypes = [ctypes.c_int, double_ptr, ctypes.c_size_t, double_ptr,
                                  ctypes.c_size_t, ctypes.c_double, ctypes.POINTER(ctypes.c_void_p)]
        self._polygon.restype = ctypes.c_int
        self._polygon_query = self._lib.nav1cm_polygon_query_v1
        self._polygon_query.argtypes = [ctypes.c_void_p, ctypes.POINTER(double_ptr),
                                        ctypes.POINTER(ctypes.POINTER(ctypes.c_size_t)),
                                        ctypes.POINTER(ctypes.c_size_t)]
        self._polygon_query.restype = ctypes.c_int
        self._polygon_destroy = self._lib.nav1cm_polygon_destroy_v1
        self._polygon_destroy.argtypes, self._polygon_destroy.restype = [ctypes.c_void_p], None
        self.capability = (load_wheel_capability() if capability is None else
                           capability if isinstance(capability, WheelTerrainCapability) else
                           WheelTerrainCapability.from_mapping(capability))
        if self.capability.body_long_side_m is None:
            self._handle = create(self.capability.maximum_slope_rad,
                                  self.capability.maximum_local_obstacle_relief_m,
                                  self.capability.minimum_underbody_clearance_m)
        else:
            try:
                platform_create=self._lib.nav1cm_terrain_create_platform_window_v2
            except AttributeError as error:
                raise RuntimeError("native terrain library lacks platform-window support; rebuild required") from error
            platform_create.argtypes=[ctypes.c_int]+[ctypes.c_double]*4
            platform_create.restype=ctypes.c_void_p
            self._handle=platform_create(1 if self.capability.platform_kind=="wheel" else 2,
                self.capability.body_long_side_m,self.capability.maximum_slope_rad,
                self.capability.maximum_local_obstacle_relief_m,self.capability.minimum_underbody_clearance_m)
        if not self._handle:
            raise RuntimeError(f"native terrain create failed: {self._last_error()}")
        self._lock = threading.RLock()
        self._finalizer = weakref.finalize(self, destroy, self._handle)
        self._lattice_count = 0
        self._cache_capacity = 8
        self._active_source_ids = ()

    @property
    def lattice_count(self):
        return self._lattice_count

    @property
    def cache_capacity(self):
        return self._cache_capacity

    @property
    def active_source_ids(self):
        """当前源的 LRU 顺序（最旧在前）；返回不可变身份元组。"""
        return self._active_source_ids

    def _last_error(self):
        return (self._error() or b"unspecified native error").decode("utf-8", errors="replace")

    def apply(self, elevation_m, *, resolution_m, origin_xy_m, yaw=0., z_offset_m=0.) -> TerrainUpdate:
        elevation = np.ascontiguousarray(elevation_m, dtype=np.float32)
        origin = tuple(float(value) for value in origin_xy_m)
        if (elevation.ndim != 2 or min(elevation.shape) <= 0 or
                not np.isfinite(resolution_m) or resolution_m <= 0 or
                len(origin) != 2 or not all(np.isfinite(origin)) or
                not np.isfinite(yaw) or not np.isfinite(z_offset_m)):
            raise ValueError("invalid native terrain source geometry")
        with self._lock:
            if not self._finalizer.alive:
                raise RuntimeError("native terrain instance is closed")
            result = _NativeResult()
            code = self._apply(self._handle, elevation, *elevation.shape,
                               resolution_m, *origin, yaw, z_offset_m, ctypes.byref(result))
            if code != 0:
                raise RuntimeError(f"native terrain apply failed: {self._last_error()}")
            if result.region_count:
                if not result.regions or result.region_count > np.iinfo(np.intp).max // 5:
                    raise RuntimeError("native terrain returned invalid region buffer")
                regions = np.ctypeslib.as_array(result.regions, shape=(result.region_count * 5,)).reshape(-1, 5)
            else:
                regions = np.empty((0, 5), dtype=np.int64)
            measurement_ptr, measurement_count = ctypes.c_void_p(), ctypes.c_size_t()
            if self._measurements(self._handle, ctypes.byref(measurement_ptr), ctypes.byref(measurement_count)):
                raise RuntimeError(f"native terrain measurement query failed: {self._last_error()}")
            count = measurement_count.value
            if count and (not measurement_ptr.value or count > np.iinfo(np.intp).max // MEASUREMENT_DTYPE.itemsize):
                raise RuntimeError("native terrain returned invalid measurement buffer")
            measurements = np.frombuffer(ctypes.string_at(measurement_ptr, count * MEASUREMENT_DTYPE.itemsize),
                                         dtype=MEASUREMENT_DTYPE) if count else np.empty(0, dtype=MEASUREMENT_DTYPE)
            source_id = int(result.source_id) or None
            evicted = (int(result.evicted_source_id),) if result.evicted_source_id else ()
            update = TerrainUpdate(result.resolution_m, (result.origin_x_m, result.origin_y_m),
                                   result.yaw, regions, source_id, evicted, int(result.cache_capacity), measurements)
            if source_id is not None:
                self._active_source_ids = tuple(identity for identity in self._active_source_ids
                                                if identity != source_id and identity not in evicted) + (source_id,)
            self._lattice_count = int(result.lattice_count)
            self._cache_capacity = update.cache_capacity
            return update

    def _polygon_boolean(self, operation, subject, clipper, epsilon):
        arrays = [np.ascontiguousarray(value, dtype=np.float64) for value in (subject, clipper)]
        if any(array.ndim != 2 or array.shape[1] != 2 for array in arrays):
            raise ValueError("polygon coordinates must have shape (n, 2)")
        with self._lock:
            if not self._finalizer.alive:
                raise RuntimeError("native terrain instance is closed")
            result = ctypes.c_void_p()
            try:
                ptr = ctypes.POINTER(ctypes.c_double)
                if self._polygon(operation, arrays[0].ctypes.data_as(ptr), len(arrays[0]),
                                 arrays[1].ctypes.data_as(ptr), len(arrays[1]), epsilon, ctypes.byref(result)):
                    raise ValueError(f"native polygon failed: {self._last_error()}")
                xy, offsets, count = ptr(), ctypes.POINTER(ctypes.c_size_t)(), ctypes.c_size_t()
                if self._polygon_query(result, ctypes.byref(xy), ctypes.byref(offsets), ctypes.byref(count)):
                    raise RuntimeError(f"native polygon query failed: {self._last_error()}")
                return tuple(tuple((float(xy[2*j]), float(xy[2*j+1]))
                                   for j in range(offsets[i], offsets[i+1])) for i in range(count.value))
            finally:
                self._polygon_destroy(result)

    def polygon_intersection(self, subject, clipper):
        return self._polygon_boolean(0, subject, clipper, 0.)

    def polygon_difference(self, subject, clipper, *, area_epsilon=0.0):
        return self._polygon_boolean(1, subject, clipper, area_epsilon)

    def close(self):
        with self._lock:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
