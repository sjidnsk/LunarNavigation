"""二维厘米边界自适应分解原型；几何参考不保证矩形车体转向或动力学可行。

按障碍 x 边界作垂直分解，扫描合并 y 区间，再以 portals 连接自由矩形。
图规模依赖障碍排列而非厘米地图面积；最坏组合复杂度仍可达 O(n²)。
自由空间分解保守使用方形膨胀；连续认证与代价使用真实圆形净空，不宣称最优。
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


def _segment_clear(a, b, boxes, corner_radius=0):
    """连续认证整条线段；接触也视为碰撞，不做离散采样。

    boxes是硬膨胀AABB；corner_radius>0时先内缩恢复原障碍矩形，
    用线段到原矩形的最短距离认证圆形半径，0保留原AABB语义。
    """
    if not len(boxes):
        return True
    # 包围盒闭区间相交是必要条件；仅排除确定无关障碍。
    boxes = boxes[(boxes[:, 0] <= max(a[0], b[0])) &
                  (boxes[:, 2] >= min(a[0], b[0])) &
                  (boxes[:, 1] <= max(a[1], b[1])) &
                  (boxes[:, 3] >= min(a[1], b[1]))]
    if corner_radius:
        raw = boxes + np.array([corner_radius, corner_radius, -corner_radius, -corner_radius])
        if not _segment_clear(a, b, raw):
            return False
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        # 凸矩形与线段的最短距离出现在端点到矩形或角点到线段。
        delta = np.maximum(np.maximum(raw[:, :2]-a, a-raw[:, 2:]), 0.)
        distance2 = np.sum(delta*delta, axis=1)
        delta = np.maximum(np.maximum(raw[:, :2]-b, b-raw[:, 2:]), 0.)
        distance2 = np.minimum(distance2, np.sum(delta*delta, axis=1))
        corners = raw[:, ((0,1), (0,3), (2,1), (2,3))]
        direction = b-a
        norm2 = float(np.dot(direction, direction))
        if norm2:
            t = np.clip(np.sum((corners-a)*direction, axis=2)/norm2, 0., 1.)
            delta = corners-a-t[:,:,None]*direction
            distance2 = np.minimum(distance2, np.sum(delta*delta, axis=2).min(axis=1))
        return not bool(np.any(distance2 <= corner_radius*corner_radius))
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


def _path_cost_python(points, hard_boxes, soft_clearance_m=.4, clearance_weight=1.0, check=None, *, corner_radius=0):
    """长度加硬碰撞边界外有限净空积分；仅用于偏好，绝不认证碰撞。

    corner_radius>0时距原矩形的距离减去圆半径，0保留AABB边界语义。
    仅在软盒与线段相交的参数区间积分，中点步长至多软带宽的1/8；
    点/障碍分批限制临时矩阵大小，每批检查调用者预算。远障无采样开销。
    """
    soft, weight = float(soft_clearance_m), float(clearance_weight)
    if not all(math.isfinite(v) and v >= 0 for v in (soft, weight)):
        raise ValueError("soft clearance and weight must be finite and nonnegative")
    boxes = np.asarray(hard_boxes, dtype=float).reshape(-1, 4)
    total = 0.
    for a, b in zip(points[:-1], points[1:]):
        if check is not None:
            check()
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        length = math.dist(a, b)
        total += length
        if not length or not soft or not weight or not len(boxes):
            continue
        near = boxes[(boxes[:, 0]-soft < max(a[0], b[0])) &
                     (boxes[:, 2]+soft > min(a[0], b[0])) &
                     (boxes[:, 1]-soft < max(a[1], b[1])) &
                     (boxes[:, 3]+soft > min(a[1], b[1]))]
        if not len(near):
            continue
        lo, hi = np.zeros(len(near)), np.ones(len(near))
        possible = np.ones(len(near), dtype=bool)
        for axis in (0, 1):
            delta = b[axis]-a[axis]
            if delta == 0:
                possible &= (a[axis] > near[:,axis]-soft) & (a[axis] < near[:,axis+2]+soft)
            else:
                t0 = (near[:,axis]-soft-a[axis])/delta
                t1 = (near[:,axis+2]+soft-a[axis])/delta
                lo = np.maximum(lo, np.minimum(t0,t1))
                hi = np.minimum(hi, np.maximum(t0,t1))
        intervals = sorted(zip(lo[possible & (lo < hi)], hi[possible & (lo < hi)]))
        merged = []
        for low, high in intervals:
            if merged and low <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], high))
            else:
                merged.append((low, high))
        axis = 0 if a[1] == b[1] else (1 if a[0] == b[0] else None)
        raw_near = near+np.array([corner_radius,corner_radius,-corner_radius,-corner_radius])
        if axis is not None:
            # 轴向长边按原障碍投影端点分段；只有可证明nearest距离恒定才解析积分。
            cuts = sorted(set(np.ravel((raw_near[:,[axis,axis+2]]-a[axis])/(b[axis]-a[axis])).tolist()))
            split = []
            for low,high in merged:
                ends = [low]+[value for value in cuts if low < value < high]+[high]
                split.extend(zip(ends[:-1],ends[1:]))
            merged = split
        for low, high in merged:
            if check is not None:
                check()
            if axis is not None:
                p0,p1 = sorted((a[axis]+low*(b[axis]-a[axis]),a[axis]+high*(b[axis]-a[axis])))
                other = 1-axis
                orth = np.maximum(np.maximum(raw_near[:,other]-a[other],a[other]-raw_near[:,other+2]),0.)
                axial = np.maximum(np.maximum(raw_near[:,axis]-p1,p0-raw_near[:,axis+2]),0.)
                cover = (raw_near[:,axis] <= p0) & (raw_near[:,axis+2] >= p1)
                if np.any(cover):
                    constant2 = float(np.min(orth[cover]**2))
                    if constant2 <= float(np.min(orth*orth+axial*axial)):
                        if check is not None:
                            check()
                        distance = max(0.,math.sqrt(constant2)-corner_radius)
                        total += weight*(high-low)*length*max(0.,1-distance/soft)**2
                        continue
            count = max(1, math.ceil((high-low)*length/(soft/8)))
            for first in range(0, count, 256):
                if check is not None:
                    check()
                t = low + (np.arange(first, min(first+256,count))+.5)*(high-low)/count
                samples = a + t[:,None]*(b-a)
                nearest = np.full(len(samples), np.inf)
                for offset in range(0, len(near), 64):
                    if check is not None:
                        check()
                    batch = near[offset:offset+64] + np.array([corner_radius,corner_radius,-corner_radius,-corner_radius])
                    delta = np.maximum(np.maximum(batch[None,:,:2]-samples[:,None,:],
                                                   samples[:,None,:]-batch[None,:,2:]), 0.)
                    nearest = np.minimum(nearest, np.maximum(0.,np.sqrt(np.sum(delta*delta,axis=2)).min(axis=1)-corner_radius))
                total += weight*(high-low)*length/count*float(np.sum(np.maximum(0.,1-nearest/soft)**2))
    return total


def path_cost(points, hard_boxes, soft_clearance_m=.4, clearance_weight=1.0, check=None, *, corner_radius=0):
    """可选原生偏好积分；无库/旧库/有界工作量超限时回退同公式Python参考。"""
    from .global_cost_native import load_cost_function
    soft,weight=float(soft_clearance_m),float(clearance_weight)
    if not all(math.isfinite(v) and v>=0 for v in (soft,weight)):
        raise ValueError("soft clearance and weight must be finite and nonnegative")
    if check is not None:
        check()
    function=load_cost_function()
    if function is not None:
        values=np.ascontiguousarray(points,dtype=float).reshape(-1,2)
        boxes=np.ascontiguousarray(hard_boxes,dtype=float).reshape(-1,4)
        if check is not None:
            check()
        result=function(values,len(values),boxes,len(boxes),soft,weight,float(corner_radius))
        if check is not None:
            check()
        if math.isfinite(result):
            return result
    return _path_cost_python(points,hard_boxes,soft,weight,check,corner_radius=corner_radius)


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
    soft_clearance_m 为真实圆形硬碰撞边界外的有限偏好带：每米代价为
    1 + clearance_weight * max(0, 1-d/soft_clearance_m)**2。
    带宽或权重为零时保留原长度搜索；软带不参与起终点与碰撞拒绝。
    """
    def __init__(self, limits: WheelLimits, clearance_radius_m=None,
                 domain_margin_m=2.0, domain_bounds_m=None, max_nodes=200_000,
                 soft_clearance_m=.4, clearance_weight=1.0):
        radius = limits.global_clearance_radius(clearance_radius_m)
        if not math.isfinite(radius) or radius < limits.body_width_m / 2:
            raise ValueError("clearance radius must be at least half vehicle width")
        if not math.isfinite(domain_margin_m) or domain_margin_m <= 0:
            raise ValueError("domain margin must be positive")
        if type(max_nodes) is not int or max_nodes < 2:
            raise ValueError("max_nodes must be an integer >= 2")
        if not all(math.isfinite(v) and v >= 0 for v in (soft_clearance_m, clearance_weight)):
            raise ValueError("soft clearance and weight must be finite and nonnegative")
        self.soft_clearance_m = float(soft_clearance_m)
        self.clearance_weight = float(clearance_weight)
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
        r = self.clearance_radius_m
        near = False
        for rect in world.blocked:
            x0, y0, x1, y1 = (rect.x0 / 100.0, rect.y0 / 100.0, rect.x1 / 100.0, rect.y1 / 100.0)
            if x0 <= x < x1 and y0 <= y < y1:
                return GoalCheck(False, "GOAL_IN_OBSTACLE", world.epoch)
            dx, dy = max(x0-x, 0., x-x1), max(y0-y, 0., y-y1)
            near |= dx*dx+dy*dy <= r*r
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
                   "free_cells": 0, "dense_cells": 0, "edge_cost_evaluations": 0}
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

        soft_enabled = self.soft_clearance_m > 0 and self.clearance_weight > 0

        def cost(points, cost_boxes=None):
            boxes = physical if cost_boxes is None else cost_boxes
            if not soft_enabled or not len(boxes):
                return sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))
            return path_cost(points, boxes, self.soft_clearance_m, self.clearance_weight, check, corner_radius=self.clearance_radius_m)

        def connect(i, j, proven_free=False, cost_boxes=None):
            check()
            # 仅同一凸自由矩形内部或其portal边界可复用分解几何证明。
            if not proven_free and not _segment_clear(nodes[i], nodes[j], physical, corner_radius=self.clearance_radius_m):
                return
            # 候选连接先保留几何证明，搜索真正展开时才积分软代价。
            # 两个方向共享一个记录；求值后释放局部障碍数组引用。
            value = [None, cost_boxes] if soft_enabled else [math.dist(nodes[i], nodes[j]), None]
            edges[i].append((j, value))
            edges[j].append((i, value))
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
            direct_clear = _segment_clear(start, goal, physical, corner_radius=radius)
            direct_cost = cost([start, goal]) if direct_clear and soft_enabled else math.dist(start, goal)
            if direct_clear and (not soft_enabled or direct_cost <= math.dist(start, goal) + 1e-10):
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
                if soft_enabled:
                    for cut in (box[0]-self.soft_clearance_m, box[2]+self.soft_clearance_m):
                        if xmin < cut < xmax:
                            events.setdefault(cut, [])
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

                def ordinates(low, high):
                    middle = (low+high)/2
                    if not soft_enabled:
                        return [middle]
                    offset = min(self.soft_clearance_m, (high-low)/2)
                    # 邻近障碍的软外边界贯穿相邻自由slab，避免只能在墙前突然横移。
                    nearby = inflated[(inflated[:,0] <= right+self.soft_clearance_m+1e-9) &
                                      (inflated[:,2] >= left-self.soft_clearance_m-1e-9)]
                    levels = [value for box in nearby for fraction in (.5,.75,1.)
                              for value in (box[1]-fraction*self.soft_clearance_m,
                                            box[3]+fraction*self.soft_clearance_m)
                              if low < value < high]
                    levels = sorted(set(levels), key=lambda y: (min(abs(y-start[1]),abs(y-goal[1])), y))[:6]
                    inner = [value for fraction in (.5,.75)
                             for value in ([low+fraction*self.soft_clearance_m] if low > ymin else []) +
                                          ([high-fraction*self.soft_clearance_m] if high < ymax else [])
                             if low < value < high]
                    return sorted(set([middle, low+offset, high-offset] + levels + inner +
                                      [p[1] for p in (start, goal) if low < p[1] < high]))

                for low, high in free:
                    cell = [node((left+right)/2, y,
                                 left == xmin or right == xmax or low == ymin or high == ymax)
                            for y in ordinates(low, high)]
                    # 凸cell内部所有边的软影响障碍集合可一次裁剪、重复使用。
                    soft = self.soft_clearance_m
                    cell_boxes = physical[(physical[:,0]-soft <= right) & (physical[:,2]+soft >= left) &
                                          (physical[:,1]-soft <= high) & (physical[:,3]+soft >= low)]
                    current.append((low, high, cell, cell_boxes))
                    for k, ident in enumerate(cell):
                        # 自由矩形内按y相邻相连，避免多净空候选产生完全图。
                        if k+1 < len(cell):
                            connect(ident,cell[k+1],proven_free=True,cost_boxes=cell_boxes)
                        for endpoint in (source_id, target_id):
                            x, y = nodes[endpoint]
                            if left <= x <= right and low < y < high:
                                connect(endpoint,ident,proven_free=True,cost_boxes=cell_boxes)
                # 两个有序自由区间列表的线性相交扫描，软候选不改变自由空间。
                i = j = 0
                while i < len(previous) and j < len(current):
                    low = max(previous[i][0], current[j][0])
                    high = min(previous[i][1], current[j][1])
                    if high-low > 1e-10:
                        for y in ordinates(low, high):
                            portal = node(left, y)
                            portals[portal] = ((left,low), (left,high), set(previous[i][2]))
                            for interval in (previous[i],current[j]):
                                cell = interval[2]
                                closest = sorted(cell,key=lambda ident: abs(nodes[ident][1]-y))[:2]
                                center = min(cell,key=lambda ident: abs(nodes[ident][1]-(interval[0]+interval[1])/2))
                                for ident in set(closest+[center]):
                                    connect(ident,portal,proven_free=True,cost_boxes=interval[3])
                    if previous[i][1] < current[j][1]:
                        i += 1
                    else:
                        j += 1
                previous = current
                storage["free_cells"] += len(current)
                storage["slabs"] += 1
            # 长slab内的软带端点不能只朝x中心移动：沿最近障碍的法向到软外边界，
            # 再沿切向提供有限过渡长度。横墙与竖墙共用同一候选规则。
            if soft_enabled and len(raw):
                lane_points = set()
                lane_ids = []
                original_count = len(nodes)
                for endpoint in (start, goal):
                    check()
                    endpoint = np.asarray(endpoint)
                    closest = np.maximum(raw[:,:2],np.minimum(endpoint,raw[:,2:]))
                    delta = endpoint-closest
                    distance = np.linalg.norm(delta,axis=1)
                    nearby = np.flatnonzero((distance > radius) &
                                            (distance < radius+self.soft_clearance_m))
                    nearby = sorted(nearby, key=lambda i: (distance[i],int(i)))[:2]
                    for index in nearby:
                        normal = delta[index]/distance[index]
                        tangent = np.asarray([-normal[1],normal[0]])
                        destination = goal if np.array_equal(endpoint,start) else start
                        sign = 1. if np.dot(np.asarray(destination)-endpoint,tangent) >= 0 else -1.
                        for fraction in (.5,.75,1.):
                            base = closest[index]+normal*(radius+fraction*self.soft_clearance_m+1e-7)
                            for scale in (0.,.5,1.,2.):
                                check()
                                point = tuple(base+tangent*sign*scale*self.soft_clearance_m)
                                if (point not in lane_points and xmin < point[0] < xmax and ymin < point[1] < ymax
                                        and _segment_clear(point,point,physical,corner_radius=radius)):
                                    lane_points.add(point)
                                    lane_ids.append(node(*point))
                for ident in lane_ids:
                    check()
                    distances = np.sum((np.asarray(nodes[:original_count])-nodes[ident])**2,axis=1)
                    closest = np.argpartition(distances,min(31,original_count-1))[:32]
                    # 最多48个端点候选内部连接；原分解图另外取32近邻。
                    # 分开筛选，避免同一端点附近的候选挤掉长车道另一端的连接。
                    others = closest.tolist()+[source_id,target_id]+[other for other in lane_ids if other > ident]
                    for other in sorted(set(others)):
                        connect(ident,other)
            # AABB在对角方向重叠但真实圆不相交时，最近角点中点可恢复窄缝拓扑。
            # 每对障碍仅增加一个候选，不创建全图栅格；全部连接仍连续认证。
            gap_points = set()
            for index, box in enumerate(raw):
                check()
                rest = raw[index+1:]
                delta = np.maximum(np.maximum(rest[:,:2]-box[2:], box[:2]-rest[:,2:]), 0.)
                nearby = ((delta[:,0] > 0) & (delta[:,1] > 0) &
                          (delta[:,0] < 2*radius) & (delta[:,1] < 2*radius) &
                          (np.sum(delta*delta,axis=1) > (2*radius)**2))
                for other in rest[nearby]:
                    check()
                    point = tuple((box[axis+2]+other[axis])/2 if box[axis+2] < other[axis]
                                  else (other[axis+2]+box[axis])/2 for axis in (0,1))
                    if (point not in gap_points and xmin < point[0] < xmax and ymin < point[1] < ymax
                            and _segment_clear(point,point,physical,corner_radius=radius)):
                        gap_points.add(point)
                        ident = node(*point)
                        distances = np.sum((np.asarray(nodes[:ident])-point)**2, axis=1)
                        closest = np.argpartition(distances, min(31,ident-1))[:32]
                        # 保留端点直连，近邻连接上限32，避免候选造成全图完全图。
                        for other_id in sorted(set(closest.tolist()+[source_id,target_id])):
                            connect(ident, other_id)
            # 合法圆角端点可能处于保守AABB分区内部，补充可见连接避免孤立。
            for endpoint in (source_id, target_id):
                if not edges[endpoint]:
                    for other in range(2, len(nodes)):
                        connect(endpoint, other)
            timing["build_s"] = time.perf_counter() - began
            search_began = time.perf_counter()
            dist = {source_id: 0.0}
            parent = {}
            queue = [(math.dist(start, goal), 0.0, source_id)]
            reaches_boundary = False
            while queue:
                check()
                _, g_cost, current = heapq.heappop(queue)
                if g_cost != dist[current]:
                    continue
                expanded += 1
                reaches_boundary |= boundary[current]
                if current == target_id:
                    break
                for neighbor, value in edges[current]:
                    check()
                    if value[0] is None:
                        value[0] = cost([nodes[current], nodes[neighbor]], value[1])
                        value[1] = None
                        storage["edge_cost_evaluations"] += 1
                    length = value[0]
                    new_cost = g_cost + length
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
            # 直连是合法候选，但不再绕过有限净空代价。
            if direct_clear and direct_cost <= dist[target_id]:
                path = [source_id, target_id]
            path_costs = [0.]
            for a, b in zip(path[:-1], path[1:]):
                path_costs.append(path_costs[-1] + cost([nodes[a],nodes[b]]))
            points = [start]
            i = 0
            while i < len(path) - 1:
                check()
                j = len(path) - 1
                while j > i + 1:
                    check()
                    if (_segment_clear(nodes[path[i]], nodes[path[j]], physical, corner_radius=radius) and
                            (not soft_enabled or cost([nodes[path[i]], nodes[path[j]]]) <=
                             path_costs[j]-path_costs[i] + 1e-10)):
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
                    bottom, top, before = portals[ident]
                    # 向右时左手为上边界；回折时反向，不能按全局x排序。
                    ordered.append((top, bottom) if path[k-1] in before else (bottom, top))
            candidate = _string_pull(start, goal, ordered, check)
            old_length = cost(points)
            new_length = cost(candidate)
            valid = new_length <= old_length
            if valid:
                for a, b in zip(candidate[:-1], candidate[1:]):
                    check()
                    if (not all(xmin < p[0] < xmax and ymin < p[1] < ymax for p in (a, b))
                            or not _segment_clear(a, b, physical, corner_radius=radius)):
                        valid = False
                        break
            if valid:
                # 已在同一physical几何上逐段认证，复用证据而不重复扫描。
                points = candidate
            else:
                # 候选不可证时只认证既有greedy结果；不发布不安全参考。
                for a, b in zip(points[:-1], points[1:]):
                    check()
                    if not _segment_clear(a, b, physical, corner_radius=radius):
                        return finish("GEOMETRY_REJECTED")
            route = Route(version, world.epoch, np.asarray(points), goal)
            timing["shortcut_validate_s"] = time.perf_counter() - shortcut_began
            check()
            return finish("PLAN_FOUND", route)
        except _Budget:
            return finish("BUDGET_EXHAUSTED")
        except _Storage:
            return finish("STORAGE_LIMIT")
