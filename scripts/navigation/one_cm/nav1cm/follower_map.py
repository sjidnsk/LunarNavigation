"""intrinsic 世界地图到完整车体中心支撑；不依赖 ROS 或控制器。

使用实际矩形外接圆，覆盖任意车体 yaw。距离场先在扩展窗口上计算，
再裁成 500²；距离下界额外扣除查询格半对角线，证明闭格全域支撑。
"""
from dataclasses import dataclass
import math
import numpy as np
from .contracts import CellState, GridSnapshot, _readonly_array, circumscribed_radius
from .maps import rasterize_world, build_distance_field


@dataclass(frozen=True)
class CenterSupport:
    cells: np.ndarray
    eligible: np.ndarray
    origin: tuple
    epoch: int
    resolution: float = .01

    def __post_init__(self):
        object.__setattr__(self,'cells',_readonly_array(self.cells,np.int8,2))
        object.__setattr__(self,'eligible',_readonly_array(self.eligible,bool,2))


def center_support(world,x,y,*,body_length_m,body_width_m,minimum_clearance_m=0.0,allow_unknown=False,backend=None):
    values=(x,y,body_length_m,body_width_m,minimum_clearance_m)
    if not all(math.isfinite(v) for v in values) or min(body_length_m,body_width_m)<=0 or minimum_clearance_m<0:
        raise ValueError('车体与窗口几何必须有限且尺寸为正')
    r=.01
    radius=circumscribed_radius(body_length_m,body_width_m,minimum_clearance_m)
    half_diagonal=math.sqrt(2)*r/2
    halo=math.ceil((radius+half_diagonal)/r)+2
    origin=(x-2.5,y-2.5)
    extended=(origin[0]-halo*r,origin[1]-halo*r)
    shape=(500+2*halo,)*2
    cells=rasterize_world(world,extended,shape)
    # 两个距离场分别表达真实障碍和已知支撑，UNKNOWN 不会被改写为 FREE。
    obstacle=build_distance_field(GridSnapshot(world.epoch,extended,r,cells,True),backend=backend)
    known=build_distance_field(GridSnapshot(world.epoch,extended,r,cells,False),backend=backend)
    crop=np.s_[halo:halo+500,halo:halo+500]
    safe_obstacle=obstacle.distance_m[crop]>radius+half_diagonal
    safe_known=known.distance_m[crop]>radius+half_diagonal
    states=np.where(~safe_obstacle,CellState.BLOCKED,np.where(safe_known,CellState.FREE,CellState.UNKNOWN))
    eligible=safe_obstacle if allow_unknown else safe_known
    return CenterSupport(states,eligible,origin,world.epoch)
