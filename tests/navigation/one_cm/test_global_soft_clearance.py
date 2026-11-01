import math
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.global_planner import AdaptiveGlobalPlanner, _segment_clear


def planner(**kwargs):
    return AdaptiveGlobalPlanner(WheelLimits(), clearance_radius_m=.75, **kwargs)


def scene(*boxes):
    return WorldSnapshot(1, boxes, ())


def success(result):
    assert (result.planning_outcome, result.reason_code, result.has_reference) == (0, 'PLAN_FOUND', True)
    return result.route.points


def independent_cost(points, boxes):
    total = 0.
    for a,b in zip(points[:-1],points[1:]):
        length = math.dist(a,b)
        n = max(1, math.ceil(length/.002))
        samples = a + (np.arange(n)[:,None]+.5)/n*(b-a)
        delta = np.maximum(np.maximum(boxes[None,:,:2]-samples[:,None,:], samples[:,None,:]-boxes[None,:,2:]),0)
        distance = np.sqrt(np.sum(delta**2,axis=2)).min(axis=1)
        total += length*np.mean(1+2*np.maximum(0,1-distance/.4)**2)
    return total


def test_clear_direct_line_near_wall_prefers_clearance_after_shortcuts():
    source = scene(CellRect(0,-100,1000,0))
    start,goal = (-1.,.85),(11.,.85)
    points = success(planner(clearance_weight=2).plan(source,start,goal,1,budget_s=3))
    hard = np.asarray([[-.75,-1.75,10.75,.75]])
    assert all(_segment_clear(a,b,hard) for a,b in zip(points[:-1],points[1:]))
    assert max(points[:,1]) >= 1.1
    assert independent_cost(points,hard) < independent_cost(np.asarray([start,goal]),hard)-1
    assert sum(math.dist(a,b) for a,b in zip(points[:-1],points[1:])) < 13


def test_soft_band_endpoints_legal_and_narrow_only_route_still_found():
    source = scene(CellRect(-1000,-200,1000,0),CellRect(-1000,170,1000,300))
    success(planner(domain_bounds_m=(-2,-1,2,2)).plan(source,(-1,.85),(1,.85),1,budget_s=3))
    assert planner().check_goal(source,(0,.85)).accepted


def test_outside_soft_band_keeps_exact_direct_line():
    points = success(planner().plan(scene(CellRect(0,-100,1000,0)),(-1,1.15),(11,1.15),1))
    np.testing.assert_array_equal(points,[[-1,1.15],[11,1.15]])


def test_zero_weight_and_zero_band_disable_preference_and_preserve_fast_path():
    for kwargs in ({'soft_clearance_m':0.},{'clearance_weight':0.}):
        actual = planner(**kwargs).plan(scene(CellRect(0,-100,1000,0)),(-1,.85),(11,.85),1)
        np.testing.assert_array_equal(success(actual),[[-1,.85],[11,.85]])
        assert actual.storage['nodes']==0


@pytest.mark.parametrize('kwargs',[{'soft_clearance_m':-1},{'soft_clearance_m':float('nan')},{'soft_clearance_m':float('inf')},{'clearance_weight':-1},{'clearance_weight':float('nan')},{'clearance_weight':float('inf')}])
def test_invalid_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        planner(**kwargs)


def test_budget_storage_limits_and_symmetric_repeat_are_stable():
    source=scene(CellRect(0,-100,1000,0))
    actual=planner().plan(source,(-1,.85),(11,.85),1,budget_s=0)
    assert actual.reason_code=='BUDGET_EXHAUSTED' and not actual.has_reference
    assert planner(max_nodes=2).plan(source,(-1,.85),(11,.85),1).reason_code=='STORAGE_LIMIT'
    symmetric=scene(CellRect(0,-50,100,50))
    first=success(planner().plan(symmetric,(-2,0),(3,0),1,budget_s=3))
    second=success(planner().plan(symmetric,(-2,0),(3,0),1,budget_s=3))
    np.testing.assert_array_equal(first,second)


def test_shared_cost_matches_independent_integral_and_has_exact_zero_outside_band():
    from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
    hard=np.asarray([[-.75,-1.75,10.75,.75]])
    points=np.asarray([[-1.,.85],[0.,1.15],[10.,1.15],[11.,.85]])
    assert path_cost(points,hard,clearance_weight=2)==pytest.approx(independent_cost(points,hard),abs=.002)
    outside=np.asarray([[-1.,1.15],[11.,1.15]])
    assert path_cost(outside,hard)==12.
    assert path_cost(points,hard,soft_clearance_m=0)==pytest.approx(sum(math.dist(a,b) for a,b in zip(points[:-1],points[1:])),abs=1e-12)
    assert path_cost(points,hard,clearance_weight=0)==pytest.approx(sum(math.dist(a,b) for a,b in zip(points[:-1],points[1:])),abs=1e-12)


def test_default_soft_route_cost_does_not_regress_against_legacy_obstacle_detour():
    source=scene(CellRect(0,-50,100,50))
    hard=np.asarray([[-.75,-1.25,1.75,1.25]])
    soft=success(planner().plan(source,(-2,0),(3,0),1,budget_s=3))
    legacy=success(planner(soft_clearance_m=0).plan(source,(-2,0),(3,0),1,budget_s=3))
    assert independent_cost(soft,hard) <= independent_cost(legacy,hard)+.002


def test_shared_cost_obeys_budget_callback_during_long_near_wall_integral():
    from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
    calls=[]
    class Expired(Exception):
        pass
    def check():
        calls.append(1)
        if len(calls)==3:
            raise Expired
    with pytest.raises(Expired):
        path_cost(np.asarray([[0.,.1],[10000.,.1]]),np.asarray([[0.,-1.,10000.,0.]]),check=check)
    assert len(calls)==3


def test_round_corner_point_and_segment_geometry_share_radius():
    source=scene(CellRect(0,0,100,100))
    hard=np.asarray([[-.75,-.75,1.75,1.75]])
    assert planner().check_goal(source,(1.6,1.6)).accepted
    assert not planner().check_goal(source,(1.5,1.5)).accepted
    assert _segment_clear((1.6,1.6),(2.,1.6),hard,corner_radius=.75)
    assert not _segment_clear((1.5,1.5),(2.,1.5),hard,corner_radius=.75)
    assert not _segment_clear((1.75,.5),(1.75,.5),hard,corner_radius=.75)
    # 线段中部接近圆角，不能仅检查端点。
    assert not _segment_clear((.9,2.),(2.,.9),hard,corner_radius=.75)


def test_round_corner_direct_and_corner_start_can_exit_conservative_partition():
    source=scene(CellRect(0,0,100,100))
    points=success(planner(soft_clearance_m=0).plan(source,(1.6,1.6),(2.,1.6),1))
    np.testing.assert_array_equal(points,[[1.6,1.6],[2.,1.6]])
    points=success(planner().plan(source,(1.6,1.6),(-2.,0.),1,budget_s=3))
    hard=np.asarray([[-.75,-.75,1.75,1.75]])
    assert all(_segment_clear(a,b,hard,corner_radius=.75) for a,b in zip(points[:-1],points[1:]))


def test_soft_cost_uses_distance_outside_true_round_corner():
    from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
    hard=np.asarray([[-.75,-.75,1.75,1.75]])
    points=np.asarray([[1.6,1.6],[2.1,1.6]])
    n=10000
    samples=points[0]+(np.arange(n)[:,None]+.5)/n*(points[1]-points[0])
    distance=np.linalg.norm(samples-[1.,1.],axis=1)-.75
    expected=.5*np.mean(1+2*np.maximum(0,1-distance/.4)**2)
    assert path_cost(points,hard,clearance_weight=2,corner_radius=.75)==pytest.approx(expected,abs=.002)
    assert path_cost(points,hard,corner_radius=.75)<path_cost(points,hard)


@pytest.mark.parametrize('vertical',[False,True])
@pytest.mark.parametrize('reverse',[False,True])
def test_default_soft_detour_has_bounded_extra_length(vertical,reverse):
    rect=CellRect(-100,400,100,600) if vertical else CellRect(400,-100,600,100)
    start,goal=((0,0),(0,10)) if vertical else ((0,0),(10,0))
    if reverse:
        start,goal=goal,start
    points=success(planner().plan(scene(rect),start,goal,1,budget_s=3))
    assert sum(math.dist(a,b) for a,b in zip(points[:-1],points[1:])) < 11.8


def test_diagonal_round_gap_remains_traversable_when_aabb_partition_is_closed():
    source=scene(CellRect(-1000,-1000,0,0),CellRect(112,112,1000,1000))
    hard=np.asarray([[-10.75,-10.75,.75,.75],[.37,.37,10.75,10.75]])
    start,goal=(-2.,2.),(2.,-2.)
    assert _segment_clear(start,(.56,.56),hard,corner_radius=.75)
    assert _segment_clear((.56,.56),goal,hard,corner_radius=.75)
    points=success(planner(domain_bounds_m=(-3,-3,3,3)).plan(source,start,goal,1,budget_s=3))
    assert all(_segment_clear(a,b,hard,corner_radius=.75) for a,b in zip(points[:-1],points[1:]))


@pytest.mark.parametrize('weight',[.5,1.,2.,4.])
@pytest.mark.parametrize('vertical',[False,True])
def test_endpoints_inside_long_slab_can_follow_a_lower_cost_soft_lane(weight,vertical):
    from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
    raw=np.asarray([[-1.,-2.,11.,0.]])
    witness=np.asarray([[0.,.85],[1.,1.15],[9.,1.15],[10.,.85]])
    if vertical:
        raw=raw[:,[1,0,3,2]]
        witness=witness[:,::-1]
    rect=CellRect(*(int(round(value*100)) for value in raw[0]))
    points=success(planner(clearance_weight=weight).plan(scene(rect),witness[0],witness[-1],1,budget_s=3))
    hard=raw+[-.75,-.75,.75,.75]
    actual=path_cost(points,hard,clearance_weight=weight,corner_radius=.75)
    known=path_cost(witness,hard,clearance_weight=weight,corner_radius=.75)
    assert actual <= known+.002,(actual,known,points)
    assert max(np.linalg.norm(points-witness[0],axis=1)) < 10.1


@pytest.mark.parametrize('weight',[.2,.3,.5,1.])
def test_intermediate_soft_clearance_candidates_can_beat_outer_boundary(weight):
    from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
    source=scene(CellRect(400,-100,600,100))
    hard=np.asarray([[3.25,-1.75,6.75,1.75]])
    witness=np.asarray([[0.,0.],[3.25,2.05],[6.75,2.05],[10.,0.]])
    points=success(planner(clearance_weight=weight).plan(source,(0,0),(10,0),1,budget_s=3))
    actual=path_cost(points,hard,clearance_weight=weight,corner_radius=.75)
    known=path_cost(witness,hard,clearance_weight=weight,corner_radius=.75)
    assert actual <= known+.003,(actual,known,points)


def test_far_obstacle_goal_check_keeps_exact_round_geometry():
    far=tuple(CellRect(100000+i*2,100000,100001+i*2,100001) for i in range(10000))
    actual=planner()
    assert actual.check_goal(scene(*far),(0,0)).accepted
    source=scene(*far,CellRect(0,0,100,100))
    assert actual.check_goal(source,(1.6,1.6)).accepted
    assert not actual.check_goal(source,(1.5,1.5)).accepted


def test_long_constant_clearance_axis_cost_is_exact():
    from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
    points=np.asarray([[0.,.85],[10000.,.85]])
    hard=np.asarray([[-1.75,-1.75,10001.75,.75]])
    expected=10000*(1+2*(1-.1/.4)**2)
    assert path_cost(points,hard,clearance_weight=2,corner_radius=.75)==pytest.approx(expected,abs=1e-8)
