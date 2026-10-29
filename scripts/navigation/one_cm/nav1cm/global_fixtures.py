"""可审查的封闭通道基准布局；构造地图和拓扑说明，不参与规划器决策。"""
from collections import defaultdict, deque
from dataclasses import dataclass
import math

from .contracts import CellRect, WorldSnapshot


@dataclass(frozen=True)
class TopologyScenario:
    name: str
    world: WorldSnapshot
    start: tuple
    goal: tuple
    inner_bounds: tuple
    gates: tuple
    topology: dict
    known_route: tuple
    lower_bound_m: float
    expected_reason: str = "PLAN_FOUND"


def _outer(width_cm, height_cm, thickness=10):
    return [CellRect(-thickness, -thickness, width_cm + thickness, 0),
            CellRect(-thickness, height_cm, width_cm + thickness, height_cm + thickness),
            CellRect(-thickness, 0, 0, height_cm),
            CellRect(width_cm, 0, width_cm + thickness, height_cm)]


def _gate(name, axis, coordinate, low, high, mandatory=True):
    return {"id": name, "axis": axis, "coordinate": coordinate,
            "interval": (low, high), "mandatory": mandatory}


def serpentine(scale_m, turns, epoch):
    size = scale_m * 100
    opening = 120
    thickness = 10
    walls = _outer(size, size)
    xs = [(i + 1) * size // (turns + 1) for i in range(turns)]
    gates = []
    start, goal = (1., 1.), (scale_m - 1., 1.)
    route = [start]
    previous_x = 0
    previous_y = 1.
    for i, x in enumerate(xs):
        if i % 2 == 0:
            walls.append(CellRect(x, 0, x + thickness, size - opening))
            low, high = (size - opening) / 100, size / 100
        else:
            walls.append(CellRect(x, opening, x + thickness, size))
            low, high = 0., opening / 100
        gate_y = (low + high) / 2
        corridor_x = (previous_x + x) / 200
        route.extend([(corridor_x, previous_y), (corridor_x, gate_y)])
        after_x = (x + thickness + (xs[i + 1] if i + 1 < len(xs) else size)) / 200
        route.append((after_x, gate_y))
        previous_x, previous_y = x + thickness, gate_y
        gates.append(_gate(f"alternating_gate_{i:02d}", 0, (x + thickness / 2) / 100, low, high))
    route.extend([(route[-1][0], goal[1]), goal])
    # 任意可行路线必须依序越过这些全高隔墙的唯一门：各段最短距离之和为下界。
    first, last = gates[0], gates[-1]
    def point_gate_distance(p, gate):
        y = min(max(p[1], gate["interval"][0]), gate["interval"][1])
        return math.hypot(p[0] - gate["coordinate"], p[1] - y)
    lower = point_gate_distance(start, first) + point_gate_distance(goal, last)
    for a, b in zip(gates[:-1], gates[1:]):
        gap = max(0., a["interval"][0] - b["interval"][1], b["interval"][0] - a["interval"][1])
        lower += math.hypot(b["coordinate"] - a["coordinate"], gap)
    return TopologyScenario(
        f"serpentine_{scale_m}m", WorldSnapshot(epoch, tuple(walls), ()), start, goal,
        (0., 0., float(scale_m), float(scale_m)), tuple(gates),
        {"family": "enclosed_alternating_baffles", "forced_direction_reversals_lower_bound": turns - 1,
         "alternating_gates": turns, "junctions": 0, "deadends": 0, "independent_cycles": 0,
         "bottlenecks": turns, "gate_width_m": 1.2, "gate_after_inflation_m": 1.2 - .818,
         "wall_thickness_m": .1, "minimum_strip_width_m": min(b - a for a, b in zip([0] + xs, xs + [size])) / 100 - .1,
         "lower_bound_kind": "sum of distances between mandatory raw gate segments"},
        tuple(route), lower)


def _edge(a, b):
    return tuple(sorted((a, b)))


def _path_edges(path):
    return {_edge(a, b) for a, b in zip(path[:-1], path[1:])}


def _graph_path(edges, start, goal):
    adjacency = defaultdict(set)
    for a, b in edges:
        adjacency[a].add(b)
        adjacency[b].add(a)
    queue, parent = deque([start]), {start: None}
    while queue:
        node = queue.popleft()
        if node == goal:
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            return list(reversed(path)), adjacency
        for neighbor in sorted(adjacency[node]):
            if neighbor not in parent:
                parent[neighbor] = node
                queue.append(neighbor)
    return [], adjacency


def _room_scenario(name, cols, rows, edges, start_cell, goal_cell, epoch,
                   closed_edge=None, chosen_gate=None):
    side, width = 400, 90
    walls = _outer(cols * side, rows * side)
    gates_by_edge = {}
    for x in range(cols):
        for y in range(rows):
            for axis, neighbor in [(0, (x + 1, y)), (1, (x, y + 1))]:
                if neighbor[0] >= cols or neighbor[1] >= rows:
                    continue
                edge = _edge((x, y), neighbor)
                fixed = (x + 1) * side if axis == 0 else (y + 1) * side
                low = y * side if axis == 0 else x * side
                high = low + side
                ranges = [(low, high)]
                if edge in edges:
                    door_low, door_high = low + (side - width) // 2, low + (side + width) // 2
                    ranges = [(low, door_low), (door_high, high)]
                    gates_by_edge[edge] = _gate(f"door_{x}_{y}_axis{axis}", axis, fixed / 100,
                                               door_low / 100, door_high / 100, False)
                for a, b in ranges:
                    walls.append(CellRect(fixed - 5, a, fixed + 5, b) if axis == 0 else CellRect(a, fixed - 5, b, fixed + 5))
    traversable = set(edges)
    if closed_edge is not None:
        closed_edge = _edge(*closed_edge)
        traversable.remove(closed_edge)
        gate = gates_by_edge[closed_edge]
        fixed = round(gate["coordinate"] * 100)
        low, high = (round(v * 100) for v in gate["interval"])
        # 唯一更新为原生 1 cm 厚矩形，完整封住已有门，不改其他墙或通道。
        walls.append(CellRect(fixed, low, fixed + 1, high) if gate["axis"] == 0 else CellRect(low, fixed, high, fixed + 1))
    path, adjacency = _graph_path(traversable, start_cell, goal_cell)
    route = tuple((x * 4. + 2., y * 4. + 2.) for x, y in path)
    mandatory = {_edge(a, b) for a, b in zip(path[:-1], path[1:])
                 if not _graph_path(traversable - {_edge(a, b)}, start_cell, goal_cell)[0]}
    unavoidable = set(mandatory)
    if chosen_gate:
        selected_edge, gate_name = chosen_gate
        mandatory.add(_edge(*selected_edge))
        gates_by_edge[_edge(*selected_edge)]["id"] = gate_name
    gates = []
    for edge, gate in gates_by_edge.items():
        gate["mandatory"] = edge in mandatory
        gate["requirement"] = "topological_bridge" if edge in unavoidable else "selected_branch" if edge in mandatory else "optional"
        gate["closed_by_update"] = edge == closed_edge
        gate["rooms"] = edge
        gates.append(gate)
    directions = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])]
    return TopologyScenario(
        name, WorldSnapshot(epoch, tuple(walls), ()), route[0], route[-1], (0., 0., cols * 4., rows * 4.),
        tuple(gates), {"family": "room_and_door_graph", "junctions": sum(len(v) >= 3 for v in adjacency.values()),
                       "deadends": sum(len(v) == 1 and k not in (start_cell, goal_cell) for k, v in adjacency.items()),
                       "independent_cycles": len(traversable) - len(adjacency) + 1,
                       "known_route_turns": sum(a != b for a, b in zip(directions[:-1], directions[1:])),
                       "bottlenecks": len(traversable), "unavoidable_gates": len(unavoidable),
                       "required_route_checks": len(mandatory),
                       "gate_width_m": .9, "gate_after_inflation_m": .9 - .818,
                       "wall_thickness_m": .1, "cell_width_m": 4., "room_graph_edges": sorted(traversable),
                       "map_update_thickness_m": .01 if closed_edge else 0.,
                       "lower_bound_kind": "Euclidean endpoint distance"},
        route, math.dist(route[0], route[-1]))


def branches_deadends(epoch):
    main = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 2), (4, 2), (4, 3), (5, 3)]
    branches = [[(1, 0), (1, 1), (0, 1), (0, 2)], [(2, 1), (3, 1), (4, 1)],
                [(4, 2), (5, 2), (5, 1)]]
    edges = _path_edges(main)
    for branch in branches:
        edges.update(_path_edges(branch))
    # 起点在支路尽头，必须退出支路，不能穿房间之间的完整墙直接切回主路。
    return _room_scenario("branches_deadends", 6, 4, edges, (0, 2), (5, 3), epoch)


def loop_scenario(epoch, blocked=False):
    top = [(0, 3), (0, 4), (1, 4), (2, 4), (3, 4), (4, 4), (5, 4), (5, 3)]
    bottom = [(0, 3), (0, 2), (0, 1), (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2), (5, 3)]
    edges = _path_edges(top) | _path_edges(bottom)
    return _room_scenario(
        "loop_one_cm_blocked" if blocked else "loop_open", 6, 5, edges, (0, 3), (5, 3), epoch,
        closed_edge=((2, 4), (3, 4)) if blocked else None,
        chosen_gate=(((2, 0), (3, 0)), "bottom_alternative") if blocked else (((2, 4), (3, 4)), "top_short_route"))


def multi_loop_bottlenecks(epoch):
    stem = [(0, 2), (1, 2), (2, 2)]
    exit_path = [(6, 2), (7, 2)]
    top = [(2, 2), (2, 3), (2, 4), (3, 4), (4, 4), (5, 4), (6, 4), (6, 3), (6, 2)]
    middle = [(2, 2), (3, 2), (3, 3), (4, 3), (5, 3), (5, 2), (6, 2)]
    bottom = [(2, 2), (2, 1), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (6, 1), (6, 2)]
    edges = set().union(*(_path_edges(path) for path in (stem, exit_path, top, middle, bottom)))
    return _room_scenario("multi_loop_bottlenecks", 8, 5, edges, (0, 2), (7, 2), epoch,
                          chosen_gate=(((4, 3), (5, 3)), "middle_selected_route"))


def centimeter_gate(epoch, opening_cm):
    walls = _outer(1000, 600)
    low = 259
    high = low + opening_cm
    walls += [CellRect(495, 0, 505, low), CellRect(495, high, 505, 600)]
    y = (low + high) / 200
    return TopologyScenario(
        f"enclosed_gate_{opening_cm}cm", WorldSnapshot(epoch, tuple(walls), ()), (2., y), (8., y),
        (0., 0., 10., 6.), (_gate("only_gate", 0, 5., low / 100, high / 100),),
        {"family": "enclosed_single_gate", "forced_turns": 0, "junctions": 0, "deadends": 0,
         "independent_cycles": 0, "bottlenecks": 1, "gate_width_m": opening_cm / 100,
         "gate_after_inflation_m": opening_cm / 100 - .818, "wall_thickness_m": .1,
         "lower_bound_kind": "Euclidean endpoint distance"},
        ((2., y), (8., y)) if opening_cm >= 82 else (), 6.,
        "PLAN_FOUND" if opening_cm >= 82 else "NO_PATH_ENCLOSED")


def topology_scenarios(epoch=1):
    return [serpentine(100, 8, epoch), serpentine(500, 20, epoch), serpentine(1000, 40, epoch),
            branches_deadends(epoch), multi_loop_bottlenecks(epoch), loop_scenario(epoch), loop_scenario(epoch, True),
            centimeter_gate(epoch, 82), centimeter_gate(epoch, 81)]
