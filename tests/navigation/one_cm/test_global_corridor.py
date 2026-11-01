"""关闭软代价以单独检验选定分解通道的拉直与直连资源回归。"""
from dataclasses import replace
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.global_planner import AdaptiveGlobalPlanner, _segment_clear

@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('vertical', [False, True])
def test_single_obstacle_corridor_shorter_than_center_shortcuts(reverse, vertical):
    rect = CellRect(-100, 400, 100, 600) if vertical else CellRect(400, -100, 600, 100)
    start, goal = ((0, 0), (0, 10)) if vertical else ((0, 0), (10, 0))
    if reverse:
        start, goal = goal, start
    planner = AdaptiveGlobalPlanner(replace(WheelLimits(), minimum_clearance_m=.2),soft_clearance_m=0.)
    result = planner.plan(WorldSnapshot(1, (rect,), ()), start, goal, 1)
    assert result.has_reference
    length = np.linalg.norm(np.diff(result.route.points, axis=0), axis=1).sum()
    assert length < 11.098, length
    radius = planner.clearance_radius_m
    box = np.array([[rect.x0/100-radius, rect.y0/100-radius, rect.x1/100+radius, rect.y1/100+radius]])
    assert all(_segment_clear(a,b,box) for a,b in zip(result.route.points[:-1],result.route.points[1:]))

def test_visible_direct_route_does_not_allocate_decomposition_nodes():
    obstacles = tuple(CellRect(1000+i*2, 1000, 1001+i*2, 1001) for i in range(1000))
    result = AdaptiveGlobalPlanner(WheelLimits(), max_nodes=2,soft_clearance_m=0.).plan(WorldSnapshot(1, obstacles, ()), (0,0), (1,0), 1)
    assert result.has_reference, result.reason_code
    assert result.storage['nodes'] == 0
    assert result.expanded == 0


@pytest.mark.parametrize('start,goal,rects', [
    ((0,0), (10,0), [CellRect(200,-200,300,100), CellRect(500,-100,600,200), CellRect(750,-200,850,100)]),
    ((10,0), (0,0), [CellRect(200,-200,300,100), CellRect(500,-100,600,200)]),
    ((0,0), (10,0), [CellRect(300,-100,500,100), CellRect(400,-50,600,200)]),
    ((0,0), (10,0), [CellRect(300,-300,700,-62), CellRect(300,62,700,300)]),
    # 起终点精确位于分解x边界，但保持物理净空。
    ((3.3909999,-3), (6.6090001,3), [CellRect(400,-100,600,100)]),
])
def test_complex_corridor_is_safe_and_no_longer_than_greedy(monkeypatch, start, goal, rects):
    import scripts.navigation.one_cm.nav1cm.global_planner as api
    planner = AdaptiveGlobalPlanner(replace(WheelLimits(), minimum_clearance_m=.2),soft_clearance_m=0.)
    source = WorldSnapshot(1, tuple(rects), ())
    result = planner.plan(source, start, goal, 1)
    assert result.has_reference
    # 非法直穿候选必须被拒绝，得到原greedy安全回退。
    monkeypatch.setattr(api, '_string_pull', lambda a,b,p,c: [a,b])
    fallback = planner.plan(source, start, goal, 1)
    assert fallback.has_reference
    length = lambda points: np.linalg.norm(np.diff(points,axis=0),axis=1).sum()
    assert length(result.route.points) <= length(fallback.route.points)
    radius = planner.clearance_radius_m
    boxes = np.array([(r.x0/100-radius,r.y0/100-radius,r.x1/100+radius,r.y1/100+radius) for r in rects])
    assert all(_segment_clear(a,b,boxes,corner_radius=radius) for a,b in zip(result.route.points[:-1],result.route.points[1:]))
    assert tuple(result.route.points[0]) == start
    assert tuple(result.route.points[-1]) == goal

@pytest.mark.parametrize('a,b,clear', [
    ((-1,1),(2,1),False), ((-1,-1),(0,0),False),
    ((0,0),(0,0),False), ((-1,1.000001),(2,1.000001),True),
    ((-2,-1),(-1,-1),True),
])
def test_bbox_filter_preserves_closed_collision(a,b,clear):
    assert _segment_clear(a,b,np.array([[0.,0.,1.,1.],[100.,100.,101.,101.]])) is clear


def test_unprovable_or_longer_candidate_preserves_safe_greedy(monkeypatch):
    import scripts.navigation.one_cm.nav1cm.global_planner as api
    planner = AdaptiveGlobalPlanner(WheelLimits(), domain_bounds_m=(-2,-5,12,5),soft_clearance_m=0.)
    source = WorldSnapshot(1,(CellRect(400,-100,600,100),),())
    monkeypatch.setattr(api, '_string_pull', lambda a,b,p,c: [a,b])
    fallback = planner.plan(source,(0,0),(10,0),1)
    for candidate in [[(0,0),(0,-4),(10,-4),(10,0)],
                      [(0,0),(0,-6),(10,-6),(10,0)]]:
        monkeypatch.setattr(api, '_string_pull', lambda a,b,p,c: candidate)
        result = planner.plan(source,(0,0),(10,0),1)
        np.testing.assert_array_equal(result.route.points, fallback.route.points)
