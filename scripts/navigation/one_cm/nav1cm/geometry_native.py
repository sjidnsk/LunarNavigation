"""显式加载任务构建的几何库；不下载、不自动编译、不替代独立认证流程。"""
import ctypes
from pathlib import Path
import numpy as np
from .contracts import CertificationReason


class NativeGeometry:
    def __init__(self, library_path):
        self.library_path=Path(library_path).resolve()
        self._lib=ctypes.CDLL(str(self.library_path))
        self._lib.nav1cm_geometry_abi.restype=ctypes.c_int
        if self._lib.nav1cm_geometry_abi()!=2:
            raise ValueError('unsupported geometry ABI')
        self._check=self._lib.nav1cm_check_poses
        self._check.restype=ctypes.c_int
        self._check.argtypes=[np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS'),ctypes.c_int,ctypes.c_double,
            np.ctypeslib.ndpointer(dtype=np.int8,flags='C_CONTIGUOUS'),np.ctypeslib.ndpointer(dtype=np.int32,flags='C_CONTIGUOUS'),
            ctypes.c_int,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_int,
            ctypes.c_double,ctypes.c_double,ctypes.POINTER(ctypes.c_int)]

    def check_poses(self, poses, margin, context, limits):
        poses=np.ascontiguousarray(poses,dtype=np.float64)
        if poses.ndim!=2 or poses.shape[1]!=3 or not np.isfinite(poses).all() or not np.isfinite(margin) or margin<0:
            raise ValueError('geometry requires finite Nx3 poses and nonnegative margin')
        grid=context.grid
        if context.hazard_prefix.shape!=(grid.cells.shape[0]+1,grid.cells.shape[1]+1):
            raise ValueError('geometry prefix dimensions mismatch')
        checked=ctypes.c_int()
        # 保留零配置margin时原有数值误差矩形外包；正配置是欧氏圆盘净空。
        length,width=limits.body_length_m,limits.body_width_m
        if limits.minimum_clearance_m==0:
            length+=2*margin; width+=2*margin; margin=0.
        result=self._check(poses,len(poses),margin,grid.cells,context.hazard_prefix,*grid.cells.shape,
            *grid.origin_xy,grid.resolution_m,int(grid.allow_unknown),length,width,ctypes.byref(checked))
        reasons={0:None,1:CertificationReason.COLLISION,2:CertificationReason.UNKNOWN,3:CertificationReason.OUT_OF_BOUNDS}
        if result not in reasons: raise RuntimeError('native geometry returned invalid reason')
        return reasons[result],checked.value
