"""厘米矩形输入场景；不参与规划器决策。"""
from dataclasses import dataclass, replace
from .contracts import CellRect, VehicleState, WorldSnapshot


@dataclass(frozen=True)
class Scenario:
    name: str
    world: WorldSnapshot
    start: VehicleState
    goal: tuple
    allow_unknown: bool = False
    max_steps: int = 4000
    update_step: int = -1
    updated_world: object = None
    updated_goal: object = None
    expected: str = 'arrival'
    clearance_radius_m: float = .9


def scenarios():
    free = (CellRect(-1000, -1000, 1500, 1500),)
    world = WorldSnapshot(1, (), free)
    start = VehicleState(0, 0, 0, 0, 0)
    unknown = WorldSnapshot(1, (), (CellRect(-120, -120, 120, 120),))
    obstacle = WorldSnapshot(1, (CellRect(200, -60, 240, 60),), free)
    dynamic = WorldSnapshot(2, (CellRect(250, -50, 290, 50),), free)
    # 三个交替挡板迫使小型连续多弯路径，外墙禁止绕过整个通道。
    channel = WorldSnapshot(1, (
        CellRect(-150, -300, 1200, -280), CellRect(-150, 280, 1200, 300),
        CellRect(250, -280, 270, 30), CellRect(550, -30, 570, 280),
        CellRect(850, -280, 870, 30)), free)
    tight = WorldSnapshot(1, (CellRect(-100, -65, 400, -60), CellRect(-100, 60, 400, 65)), free)
    return [
        Scenario('straight', world, start, (1.4, 0), max_steps=600),
        Scenario('turn', world, start, (0, 1.4), max_steps=1000),
        Scenario('bypass-reentry', obstacle, start, (4.5, 0)),
        Scenario('new-obstacle', world, start, (4.5, 0), update_step=100, updated_world=dynamic),
        Scenario('unknown-allow', unknown, start, (2., 0), True, 800),
        Scenario('unknown-deny', unknown, start, (2., 0), max_steps=700, expected='safe-stop'),
        Scenario('goal-outside-window', unknown, start, (3., 0), True, 1000),
        Scenario('goal-later-blocked', world, start, (3., 0), update_step=100,
                 updated_world=WorldSnapshot(2, (CellRect(290, -10, 310, 10),), free), expected='safe-stop'),
        Scenario('route-version', world, start, (2., 0), max_steps=800, update_step=80,
                 updated_world=WorldSnapshot(2, (), free)),
        Scenario('route-foldback', world, start, (2., 0), max_steps=800, update_step=100,
                 updated_goal=(-1., 0)),
        Scenario('multi-bend', channel, start, (10.5, 0), max_steps=4500),
        Scenario('tight-corridor', tight, start, (2., 0), max_steps=800, clearance_radius_m=.409,
                 expected='boundary-observation'),
    ]
