"""Smac栅格化保守性回归。"""
import numpy as np
from scripts.navigation.one_cm.benchmark_smac2d import rasterize

def test_circle_and_diagonal_margin():
    request={'world':{'blocked':[{'x0':200,'y0':200,'x1':201,'y1':201}]},'start':[0,0],'goal':[4,4]}
    grid,origin=rasterize(request, .1, (-1,-1,5,5))
    assert grid[30,30] == 254
    assert grid[30,40] < 254
    assert grid[30,38] == 254
    assert np.any((grid>0)&(grid<254))
