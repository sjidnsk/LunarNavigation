"""显式可选的原生精确距离场；不参与轨迹认证。"""
import ctypes
import hashlib
from pathlib import Path
import numpy as np
from .contracts import CellState
from .maps import DistanceField


class NativeDistanceField:
    def __init__(self, library_path):
        self.library_path = Path(library_path).resolve(strict=True)
        self.library_sha256 = hashlib.sha256(self.library_path.read_bytes()).hexdigest()
        self._lib = ctypes.CDLL(str(self.library_path))
        version = self._lib.nav1cm_field_abi_version
        version.argtypes, version.restype = [], ctypes.c_int
        if version() != 1:
            raise RuntimeError('unsupported distance field ABI')
        self._build = self._lib.nav1cm_distance_field_v1
        self._build.argtypes = [np.ctypeslib.ndpointer(dtype=np.uint8, ndim=2, flags='C_CONTIGUOUS'),
                               ctypes.c_int, ctypes.c_int, ctypes.c_double] + [
            np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags=('C_CONTIGUOUS', 'WRITEABLE'))] * 3
        self._build.restype = ctypes.c_int

    def build(self, grid):
        shape = grid.cells.shape
        if len(shape) != 2 or min(shape) < 2 or max(shape) > np.iinfo(np.int32).max:
            raise ValueError('distance field needs two axes of size 2..INT_MAX')
        if not np.isfinite(grid.resolution_m) or grid.resolution_m <= 0:
            raise ValueError('resolution must be finite and positive')
        passable = grid.cells != CellState.BLOCKED
        if not grid.allow_unknown:
            passable &= grid.cells != CellState.UNKNOWN
        passable = np.ascontiguousarray(passable, dtype=np.uint8)
        arrays = [np.empty(shape, dtype=np.float64) for _ in range(3)]
        code = self._build(passable, *shape, grid.resolution_m, *arrays)
        if code != 0:
            raise RuntimeError(f'native distance field failed: {code}')
        if any(a.shape != shape or a.dtype != np.float64 or not np.isfinite(a).all() for a in arrays):
            raise ValueError('native distance field returned invalid output')
        return DistanceField(grid.epoch, grid.origin_xy, grid.resolution_m, *arrays)
