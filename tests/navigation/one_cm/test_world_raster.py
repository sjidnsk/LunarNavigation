"""共享栅格的独立面积覆盖 oracle，含真实未知缝隙和非方窗口。"""
import math
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect,WorldSnapshot
from scripts.navigation.one_cm.nav1cm.maps import rasterize_world


@pytest.mark.parametrize('origin',[(-.023,-.017),(.003,.007),(0.,0.),(-.02,.01)])
@pytest.mark.parametrize('shape',[(9,13),(17,7)])
@pytest.mark.parametrize('gap',[0,1])
def test_rect_union_matches_independent_area_coverage(origin,shape,gap):
    world=WorldSnapshot(3,(CellRect(2,1,3,4),),(
        CellRect(-2,-2,0,8),CellRect(gap,-2,8,2),CellRect(gap,3,8,8)))
    got=rasterize_world(world,origin,shape)
    expected=np.full(shape,-1,np.int8)
    for iy in range(shape[0]):
        for ix in range(shape[1]):
            x0,y0=origin[0]+ix*.01,origin[1]+iy*.01
            x1,y1=x0+.01,y0+.01
            # 将查询矩形沿整数厘米界线分区，在每个正面积子区中心查 FREE 并集。
            xs=sorted({x0,x1}|{k*.01 for k in range(-4,25) if x0<k*.01<x1})
            ys=sorted({y0,y1}|{k*.01 for k in range(-4,25) if y0<k*.01<y1})
            free=True
            for a,b in zip(xs,xs[1:]):
                for c,d in zip(ys,ys[1:]):
                    if b-a<1e-12 or d-c<1e-12:continue
                    x,y=(a+b)/2,(c+d)/2
                    free &= any(r.x0*.01<=x<r.x1*.01 and r.y0*.01<=y<r.y1*.01 for r in world.known_free)
            if free:expected[iy,ix]=0
            if any(min(x1,r.x1*.01)-max(x0,r.x0*.01)>1e-12 and
                   min(y1,r.y1*.01)-max(y0,r.y0*.01)>1e-12 for r in world.blocked):expected[iy,ix]=1
    np.testing.assert_array_equal(got,expected)
    assert world.epoch==3
    with pytest.raises(ValueError):got.flags.writeable=True


@pytest.mark.parametrize('shape',[(500,500),(650,650),(650,713)])
def test_large_shape_adjacent_union_and_true_one_cm_gap(shape):
    x,y=-3.250226100963422,-3.2476729756580833
    split=WorldSnapshot(1,(),(CellRect(-400,-400,0,500),CellRect(0,-400,500,500)))
    whole=WorldSnapshot(1,(),(CellRect(-400,-400,500,500),))
    np.testing.assert_array_equal(rasterize_world(split,(x,y),shape),rasterize_world(whole,(x,y),shape))
    gap=WorldSnapshot(1,(),(CellRect(-400,-400,0,500),CellRect(1,-400,500,500)))
    grid=rasterize_world(gap,(x,y),shape)
    assert np.all(grid[:,325:327]==-1)


@pytest.mark.parametrize('origin,shape',[((float('nan'),0),(3,4)),((0,0),(0,4)),((0,0),(3.5,4))])
def test_invalid_window_rejected(origin,shape):
    with pytest.raises(ValueError):rasterize_world(WorldSnapshot(1,(),()),origin,shape)
