"""二维厘米边界自适应分解原型；几何参考不保证矩形车体转向或动力学可行。

按障碍 x 边界作垂直分解，扫描合并 y 区间，再以 portals 连接自由矩形。
图规模依赖障碍排列而非厘米地图面积；最坏组合复杂度仍可达 O(n²)。
半车宽的轴对齐方形膨胀保守覆盖同半径圆，角部额外保守；不宣称最优。
"""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

from .contracts import Route, WheelLimits, WorldSnapshot


@dataclass(frozen=True)
class GoalCheck:
    accepted: bool
    reason_code: str
    map_epoch: int


@dataclass(frozen=True)
class GlobalPlanResult:
    planning_outcome: int
    reason_code: str
    has_reference: bool
    route: Route | None
    domain: tuple
    expanded: int
    storage: dict
    timing: dict
    clearance_radius_m: float


def _point(value):
    point = tuple(float(v) for v in value)
    if len(point) != 2 or not all(math.isfinite(v) for v in point):
        raise ValueError("point must contain finite x,y")
    return point


def _segment_clear(a, b, boxes):
    """精确参数区间裁剪整条线段；边界接触也视为碰撞，不做离散采样。"""
    if not len(boxes):
        return True
    # 包围盒闭区间相交是必要条件；仅排除确定无关障碍。
    boxes = boxes[(boxes[:, 0] <= max(a[0], b[0])) &
                  (boxes[:, 2] >= min(a[0], b[0])) &
                  (boxes[:, 1] <= max(a[1], b[1])) &
                  (boxes[:, 3] >= min(a[1], b[1]))]
    lo = np.zeros(len(boxes))
    hi = np.ones(len(boxes))
    possible = np.ones(len(boxes), dtype=bool)
    for axis in (0, 1):
        delta = b[axis] - a[axis]
        if delta == 0:
            possible &= (a[axis] >= boxes[:, axis]) & (a[axis] <= boxes[:, axis + 2])
        else:
            near = (boxes[:, axis] - a[axis]) / delta
            far = (boxes[:, axis + 2] - a[axis]) / delta
            lo = np.maximum(lo, np.minimum(near, far))
            hi = np.minimum(hi, np.maximum(near, far))
    return not bool(np.any(possible & (lo <= hi)))


def _string_pull(start, goal, portals, check):
    """有序左右portal漏斗；仅生成候选，调用者仍须连续认证。"""
    def area(a, b, c):
        # 与漏斗左右收紧判据一致的顺时针有向面积。
        return (c[0]-a[0])*(b[1]-a[1]) - (b[0]-a[0])*(c[1]-a[1])

    gates = [(start, start), *portals, (goal, goal)]
    apex = left = right = start
    apex_i = left_i = right_i = 0
    points = [start]
    i = 1
    while i < len(gates):
        check()
        new_left, new_right = gates[i]
        if area(apex, right, new_right) <= 0:
            if apex == right or area(apex, left, new_right) > 0:
                right, right_i = new_right, i
            else:
                if points[-1] != left:
                    points.append(left)
                apex, apex_i = left, left_i
                left = right = apex
                left_i = right_i = apex_i
                i = apex_i + 1
                continue
        if area(apex, left, new_left) >= 0:
            if apex == left or area(apex, right, new_left) < 0:
                left, left_i = new_left, i
            else:
                if points[-1] != right:
                    points.append(right)
                apex, apex_i = right, right_i
                left = right = apex
                left_i = right_i = apex_i
                i = apex_i + 1
                continue
        i += 1
    if points[-1] != goal:
        points.append(goal)
    return points


class _Budget(Exception):
    pass


class _Storage(Exception):
    pass


class AdaptiveGlobalPlanner:
    """无缓存完整更新；UNKNOWN 可通行，known_free 不限制搜索范围。

    domain_bounds_m 可用于显式有限域；默认包围起终点与全部膨胀障碍并加
    domain_margin_m。有限域外端点报 DOMAIN_EXHAUSTED。max_nodes 防止
    对抗性矩形排列耗尽内存，触发时明确报 STORAGE_LIMIT。
    """
    def __init__(self, limits: WheelLimits, clearance_radius_m=None,
                 domain_margin_m=2.0, domain_bounds_m=None, max_nodes=200_000):
        radius = limits.global_clearance_radius(clearance_radius_m)
        if not math.isfinite(radius) or radius < limits.body_width_m / 2:
            raise ValueError("clearance radius must be at least half vehicle width")
        if not math.isfinite(domain_margin_m) or domain_margin_m <= 0:
            raise ValueError("domain margin must be positive")
        if type(max_nodes) is not int or max_nodes < 2:
            raise ValueError("max_nodes must be an integer >= 2")
        self.clearance_radius_m = radius
        self.domain_margin_m = float(domain_margin_m)
        self.max_nodes = max_nodes
        self.domain_bounds_m = None
        if domain_bounds_m is not None:
            bounds = tuple(float(v) for v in domain_bounds_m)
            if len(bounds) != 4 or not all(math.isfinite(v) for v in bounds) or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
                raise ValueError("invalid domain bounds")
            self.domain_bounds_m = bounds

    def check_goal(self, world: WorldSnapshot, goal_xy) -> GoalCheck:
        """O(n) 原始占据与配置空间检查；图外和 UNKNOWN 不被拒绝。"""
        x, y = _point(goal_xy)
        r = self.clearance_radius_m + 1e-7
        near = False
        for rect in world.blocked:
            x0, y0, x1, y1 = (rect.x0 / 100.0, rect.y0 / 100.0, rect.x1 / 100.0, rect.y1 / 100.0)
            if x0 <= x < x1 and y0 <= y < y1:
                return GoalCheck(False, "GOAL_IN_OBSTACLE", world.epoch)
            near |= x0 - r <= x <= x1 + r and y0 - r <= y <= y1 + r
        return GoalCheck(not near, "GOAL_CLEARANCE" if near else "GOAL_ACCEPTED", world.epoch)

    def plan(self, world: WorldSnapshot, start_xy, goal_xy, version, budget_s=1.0) -> GlobalPlanResult:
        began = time.perf_counter()
        if not math.isfinite(budget_s) or budget_s < 0:
            raise ValueError("budget_s must be finite and nonnegative")
        start, goal = _point(start_xy), _point(goal_xy)
        deadline = began + budget_s
        domain = ()
        nodes, edges, boundary = [], [], []
        portals = {}
        storage = {"obstacles": len(world.blocked), "nodes": 0, "edges": 0, "slabs": 0,
                   "free_cells": 0, "dense_cells": 0}
        timing = {}
        expanded = 0

        def check():
            if time.perf_counter() >= deadline:
                raise _Budget

        def finish(reason, route=None):
            storage["nodes"] = len(nodes)
            timing["total_s"] = time.perf_counter() - began
            ok = route is not None and reason == "PLAN_FOUND"
            return GlobalPlanResult(0 if ok else 1, reason, ok, route, domain, expanded,
                                    dict(storage), dict(timing), self.clearance_radius_m)

        def node(x, y, outer=False):
            check()
            if len(nodes) >= self.max_nodes:
                raise _Storage
            nodes.append((x, y))
            edges.append([])
            boundary.append(outer)
            return len(nodes) - 1

        def connect(i, j):
            length = math.dist(nodes[i], nodes[j])
            edges[i].append((j, length))
            edges[j].append((i, length))
            storage["edges"] += 1

        try:
            check()
            target = self.check_goal(world, goal)
            check()
            if not target.accepted:
                return finish(target.reason_code)
            origin = self.check_goal(world, start)
            check()
            if not origin.accepted:
                return finish(origin.reason_code.replace("GOAL", "START"))
            if start == goal:
                return finish("ALREADY_AT_GOAL")
            radius = self.clearance_radius_m
            raw = np.asarray([(r.x0 / 100.0, r.y0 / 100.0, r.x1 / 100.0, r.y1 / 100.0)
                              for r in world.blocked], dtype=float).reshape(-1, 4)
            # 最后碰撞认证用物理净空；分解额外留 0.1 微米，避免数值贴边。
            physical = raw + np.array([-radius, -radius, radius, radius])
            inflated = physical + np.array([-1e-7, -1e-7, 1e-7, 1e-7])
            if self.domain_bounds_m is None:
                m = self.domain_margin_m
                domain = (min(start[0], goal[0], *(inflated[:, 0] if len(raw) else (start[0],))) - m,
                          min(start[1], goal[1], *(inflated[:, 1] if len(raw) else (start[1],))) - m,
                          max(start[0], goal[0], *(inflated[:, 2] if len(raw) else (start[0],))) + m,
                          max(start[1], goal[1], *(inflated[:, 3] if len(raw) else (start[1],))) + m)
            else:
                domain = self.domain_bounds_m
            xmin, ymin, xmax, ymax = domain
            if any(not (xmin < p[0] < xmax and ymin < p[1] < ymax) for p in (start, goal)):
                return finish("DOMAIN_EXHAUSTED")
            check()
            if _segment_clear(start, goal, physical):
                check()
                timing["direct_validate_s"] = time.perf_counter() - began
                return finish("PLAN_FOUND", Route(version, world.epoch, np.asarray([start, goal]), goal))
            events = {xmin: [], xmax: []}
            clipped = []
            for box in inflated:
                check()
                x0, y0, x1, y1 = max(xmin, box[0]), max(ymin, box[1]), min(xmax, box[2]), min(ymax, box[3])
                if x0 >= x1 or y0 >= y1:
                    continue
                ident = len(clipped)
                clipped.append((y0, y1))
                events.setdefault(x0, []).append((ident, True))
                events.setdefault(x1, []).append((ident, False))
            xs = sorted(events)
            active = set()
            previous = []
            source_id = node(*start)
            target_id = node(*goal)
            for left, right in zip(xs[:-1], xs[1:]):
                check()
                for ident, enter in events[left]:
                    if enter:
                        active.add(ident)
                    else:
                        active.discard(ident)
                intervals = sorted(clipped[i] for i in active)
                free = []
                cursor = ymin
                for low, high in intervals:
                    if low > cursor:
                        free.append((cursor, low))
                    cursor = max(cursor, high)
                if cursor < ymax:
                    free.append((cursor, ymax))
                current = []
                for low, high in free:
                    center = node((left + right) / 2, (low + high) / 2,
                                  left == xmin or right == xmax or low == ymin or high == ymax)
                    current.append((low, high, center))
                    for endpoint in (source_id, target_id):
                        x, y = nodes[endpoint]
                        if left <= x <= right and low < y < high:
                            connect(endpoint, center)
                # 两个有序自由区间列表的线性相交扫描。
                i = j = 0
                while i < len(previous) and j < len(current):
                    low = max(previous[i][0], current[j][0])
                    high = min(previous[i][1], current[j][1])
                    if high - low > 1e-10:
                        portal = node(left, (low + high) / 2)
                        portals[portal] = ((left, low), (left, high), previous[i][2], current[j][2])
                        connect(previous[i][2], portal)
                        connect(portal, current[j][2])
                    if previous[i][1] < current[j][1]:
                        i += 1
                    else:
                        j += 1
                previous = current
                storage["free_cells"] += len(current)
                storage["slabs"] += 1
            timing["build_s"] = time.perf_counter() - began
            search_began = time.perf_counter()
            dist = {source_id: 0.0}
            parent = {}
            queue = [(math.dist(start, goal), 0.0, source_id)]
            reaches_boundary = False
            while queue:
                check()
                _, cost, current = heapq.heappop(queue)
                if cost != dist[current]:
                    continue
                expanded += 1
                reaches_boundary |= boundary[current]
                if current == target_id:
                    break
                for neighbor, length in edges[current]:
                    new_cost = cost + length
                    if new_cost < dist.get(neighbor, math.inf):
                        dist[neighbor] = new_cost
                        parent[neighbor] = current
                        heapq.heappush(queue, (new_cost + math.dist(nodes[neighbor], goal), new_cost, neighbor))
            else:
                timing["search_s"] = time.perf_counter() - search_began
                return finish("DOMAIN_EXHAUSTED" if reaches_boundary else "NO_PATH_ENCLOSED")
            timing["search_s"] = time.perf_counter() - search_began
            path = [target_id]
            while path[-1] != source_id:
                check()
                path.append(parent[path[-1]])
            path.reverse()
            shortcut_began = time.perf_counter()
            points = [start]
            i = 0
            while i < len(path) - 1:
                check()
                j = len(path) - 1
                while j > i + 1:
                    check()
                    if _segment_clear(nodes[path[i]], nodes[path[j]], physical):
                        break
                    j -= 1
                point = nodes[path[j]]
                if point != points[-1]:
                    points.append(point)
                i = j
            ordered = []
            for k, ident in enumerate(path):
                check()
                if ident in portals:
                    bottom, top, before, after = portals[ident]
                    # 向右时左手为上边界；回折时反向，不能按全局x排序。
                    ordered.append((top, bottom) if path[k-1] == before else (bottom, top))
            candidate = _string_pull(start, goal, ordered, check)
            old_length = sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))
            new_length = sum(math.dist(a, b) for a, b in zip(candidate[:-1], candidate[1:]))
            valid = new_length <= old_length
            for a, b in zip(candidate[:-1], candidate[1:]):
                check()
                valid &= _segment_clear(a, b, physical)
                valid &= all(xmin < p[0] < xmax and ymin < p[1] < ymax for p in (a, b))
            if valid:
                points = candidate
            # 独立于图的最终连续碰撞门禁；任何数值边界问题均不发布参考。
            for a, b in zip(points[:-1], points[1:]):
                check()
                if not _segment_clear(a, b, physical):
                    return finish("GEOMETRY_REJECTED")
            route = Route(version, world.epoch, np.asarray(points), goal)
            timing["shortcut_validate_s"] = time.perf_counter() - shortcut_began
            check()
            return finish("PLAN_FOUND", route)
        except _Budget:
            return finish("BUDGET_EXHAUSTED")
        except _Storage:
            return finish("STORAGE_LIMIT")
