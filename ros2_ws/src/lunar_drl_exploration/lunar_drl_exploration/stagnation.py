"""Bounded measured-progress monitor for training rollout truncation only."""
from collections import deque
import math


class StagnationMonitor:
    """Count repeated executed XY positions in a recent zero-gain window.

    Positions are grouped by Euclidean distance to chronological fixed
    representatives. Unlike grid bins, this has no arbitrary cell boundary;
    unlike connected components, a chain of nearby new poses stays transit.
    This diagnostic does not rank, mask, or replace any navigation action.
    """
    def __init__(self,window_steps,max_positions,position_tolerance_m):
        if (type(window_steps) is not int or type(max_positions) is not int
                or not 0 < max_positions < window_steps):
            raise ValueError('stagnation window must exceed positive max_positions')
        if not math.isfinite(position_tolerance_m) or position_tolerance_m<=0:
            raise ValueError('positive finite stagnation position tolerance required')
        self.window_steps=window_steps
        self.max_positions=max_positions
        self.position_tolerance_m=float(position_tolerance_m)
        self._positions=deque(maxlen=window_steps)
        self.reset()

    def reset(self):
        self._positions.clear()
        self.zero_gain_steps=0
        self.position_count=0
        self.triggered=False

    @property
    def buffered_steps(self):
        return len(self._positions)

    def observe(self,xy,new_area_m2):
        x,y=map(float,xy)
        if not all(math.isfinite(v) for v in (x,y,new_area_m2)):
            raise ValueError('finite executed position and measured area required')
        if new_area_m2>0:
            self.reset()
            return False
        self.zero_gain_steps+=1
        self._positions.append((x,y))
        representatives=[]
        for px,py in self._positions:
            if not any(math.hypot(px-rx,py-ry)<=self.position_tolerance_m
                       for rx,ry in representatives):
                representatives.append((px,py))
        self.position_count=len(representatives)
        self.triggered=(len(self._positions)==self.window_steps
                        and self.position_count<=self.max_positions)
        return self.triggered
