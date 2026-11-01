"""独立长距离验收应验证同一圆形碰撞边界，而非方框角部。"""
from types import SimpleNamespace as NS
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect,WorldSnapshot,Route
from scripts.navigation.one_cm.benchmark_global import validate

@pytest.mark.parametrize('start,goal,valid',[
    ((1.6,1.6),(2.4,1.6),True),
    ((1.5,1.5),(2.4,1.5),False),
    ((-.8,1.8),(1.8,-.8),False),
    ((1.75001,-1.),(1.75001,2.),True),
])
def test_independent_verifier_uses_circle_distance(start,goal,valid):
    world=WorldSnapshot(2,(CellRect(0,0,100,100),),())
    route=Route(1,2,(start,goal),goal)
    result=NS(route=route,planning_outcome=0,reason_code='PLAN_FOUND',has_reference=True,clearance_radius_m=.75)
    checked=validate(world,start,goal,result,expected_version=1)
    assert checked['valid'] is valid
