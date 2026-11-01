"""可选原生偏好积分，每进程只查库一次；旧ABI3库无此符号时保留Python参考。"""
import ctypes
from functools import lru_cache
import os
from pathlib import Path
import numpy as np


@lru_cache(maxsize=1)
def load_cost_function():
    explicit=os.environ.get('NAV1CM_GEOMETRY_LIB')
    paths=[Path(explicit)] if explicit else [p/name for p in list(Path(__file__).absolute().parents)[:6]
                                           for name in ('libnav1cm_geometry.so','lib/libnav1cm_geometry.so')]
    if not explicit:
        # 保留安装前缀的symlink路径，并兼容ROS的ament/动态库搜索前缀。
        paths += [Path(prefix)/'lib/libnav1cm_geometry.so' for prefix in os.environ.get('AMENT_PREFIX_PATH','').split(os.pathsep) if prefix]
        paths += [Path(prefix)/'libnav1cm_geometry.so' for prefix in os.environ.get('LD_LIBRARY_PATH','').split(os.pathsep) if prefix]
    for path in paths:
        if not path.is_file():
            continue
        try:
            library=ctypes.CDLL(str(path))
            library.nav1cm_geometry_abi.restype=ctypes.c_int
            if library.nav1cm_geometry_abi()!=3:
                continue
            function=library.nav1cm_soft_path_cost
            function.argtypes=[np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS'),ctypes.c_int,
                               np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS'),ctypes.c_int]+[ctypes.c_double]*3
            function.restype=ctypes.c_double
            function._library=library
            return function
        except (OSError,AttributeError):
            continue
    return None
