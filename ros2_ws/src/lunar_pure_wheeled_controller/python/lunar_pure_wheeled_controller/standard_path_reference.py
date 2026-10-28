"""有限、有序的车体位姿参考；旋转点和拐点构成不可跳过的屏障。"""
from dataclasses import dataclass
import math
from .path_geometry import Polyline, angle


@dataclass(frozen=True)
class OrderedStage:
    path: Polyline
    direction: int
    spin_delta: float = 0.


    def body_yaw_at(self, projection):
        if not self.path.lengths:
            return self.path.final_yaw
        i=projection.index
        a,b=self.path.original[i:i+2]
        fraction=(projection.s-self.path.arcs[i])/self.path.lengths[i]
        return a[2]+fraction*angle(b[2]-a[2])


class StandardPathReference:
    def __init__(self, points, policy):
        values=tuple(tuple(float(v) for v in p) for p in points)
        if not values or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in values):
            raise ValueError('finite ordered x/y/body-yaw poses required')
        self.points=values
        stages=[];run=[values[0]];direction=None;heading=None
        def flush():
            if len(run)>1:stages.append(OrderedStage(Polyline(run),direction or 1))
        for a,b in zip(values,values[1:]):
            if math.dist(a[:2],b[:2])<=1e-8:
                flush();run=[b];heading=None;direction=None
                delta=angle(b[2]-a[2])
                if abs(delta)>1e-9:
                    # 合并同向密集角采样，以小于 pi 的屏障保留圈数和反转。
                    if (stages and stages[-1].direction==0 and
                            stages[-1].spin_delta*delta>0 and
                            abs(stages[-1].spin_delta+delta)<=math.pi/2 and
                            math.dist(stages[-1].path.points[-1],b[:2])<=1e-8):
                        prior=stages.pop()
                        stages.append(OrderedStage(Polyline((b,)),0,prior.spin_delta+delta))
                    else:stages.append(OrderedStage(Polyline((b,)),0,delta))
                continue
            new_heading=math.atan2(b[1]-a[1],b[0]-a[0])
            new_direction=-1 if math.cos(new_heading-a[2])<0 else 1
            if heading is not None and (abs(angle(new_heading-heading))>=policy.corner_angle_rad or new_direction!=direction):
                flush();run=[a]
            direction=new_direction;heading=new_heading;run.append(b)
        flush()
        if not stages:stages.append(OrderedStage(Polyline((values[-1],)),0))
        self.stages=tuple(stages)
