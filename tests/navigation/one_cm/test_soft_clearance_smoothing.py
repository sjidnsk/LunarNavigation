"""全局软净空是路径代价，不得被后续Bezier剪角抹掉。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Route
from scripts.navigation.one_cm.nav1cm.global_planner import path_cost
from scripts.navigation.one_cm.nav1cm.reference_smoothing import shape_route


def test_corner_smoothing_keeps_the_same_clearance_objective():
    raw=Route(3,4,((0.,0.),(2.,0.),(2.,2.)),(2.,2.))
    boxes=np.array([[1.65,.30,1.70,.35]])
    plain=shape_route(raw,boxes,budget_s=2.)
    assert plain.changed
    assert path_cost(plain.route.points,boxes,.4,20.) > path_cost(raw.points,boxes,.4,20.)
    shaped=shape_route(raw,boxes,budget_s=2.,soft_clearance_m=.4,clearance_weight=20.)
    assert path_cost(shaped.route.points,boxes,.4,20.) <= path_cost(raw.points,boxes,.4,20.)+1e-9
    assert not shaped.changed and shaped.reason=='CLEARANCE_COST'


def test_soft_cost_disabled_keeps_existing_corner_smoothing():
    raw=Route(3,4,((0.,0.),(2.,0.),(2.,2.)),(2.,2.))
    shaped=shape_route(raw,np.empty((0,4)),budget_s=2.,soft_clearance_m=0.,clearance_weight=0.)
    assert shaped.changed


@pytest.mark.parametrize('kwargs',[{'soft_clearance_m':-.1},{'clearance_weight':-1.},
                                  {'soft_clearance_m':float('nan')},{'clearance_weight':float('inf')}])
def test_smoothing_rejects_invalid_soft_parameters(kwargs):
    raw=Route(3,4,((0.,0.),(2.,0.),(2.,2.)),(2.,2.))
    with pytest.raises(ValueError):shape_route(raw,np.empty((0,4)),budget_s=2.,**kwargs)


def test_rounded_obstacle_corner_survives_reference_validation():
    from scripts.navigation.one_cm.nav1cm.reference_smoothing import reference_clear,ShapedRoute
    raw=Route(3,4,((1.6,1.6),(2.4,1.6)),(2.4,1.6))
    boxes=np.array([[-.75,-.75,1.75,1.75]])
    shaped=shape_route(raw,boxes,budget_s=2.,corner_radius=.75,
                       soft_clearance_m=.4,clearance_weight=2.)
    assert reference_clear(shaped,boxes,corner_radius=.75)
    unsafe=ShapedRoute(Route(3,4,((1.5,1.5),(2.4,1.5)),(2.4,1.5)),np.empty((0,4)),False,'raw')
    assert not reference_clear(unsafe,boxes,corner_radius=.75)


def test_coordinator_revalidates_the_same_round_collision_boundary():
    from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.contracts import WorldSnapshot,CellRect
    c=NavigationCoordinator(None,runner=object(),reference_smoothing=True,revalidate_current_map=True)
    c._observe_world(WorldSnapshot(4,(CellRect(0,0,100,100),),()))
    raw=Route(3,4,((1.6,1.6),(2.4,1.6)),(2.4,1.6))
    assert c._route_clear(raw)
    c._adopt_route(raw,[],raw_verified=True)
    assert c._route_clear(c.route)
