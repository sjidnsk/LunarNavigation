"""只组合地图、全局参考、MPCC 和认证；tick 只输出离线动作数据。"""
from dataclasses import dataclass, replace
from time import perf_counter
import numpy as np

from .contracts import CellState, LocalReference, WheelLimits, VehicleState
from .maps import extract_window, DistanceFieldCache
from .reference import extract_reference, project_progress, _arc_lengths, _point_at, TERMINAL_APPROACH_DISTANCE_M
from .certification import certify, prepare_certification
from .seeds import brake_candidate
from .global_planner import AdaptiveGlobalPlanner, GoalCheck, _segment_clear
from .global_worker import GlobalWorker, make_request, restore_result
from .execution import action_prefix
from .pose_goal import angle_error, normalize_yaw, check_pose_goal, terminal_candidate


@dataclass(frozen=True)
class CycleResult:
    status: str
    candidate: object = None
    certificate: object = None
    grid: object = None
    reference: object = None
    events: tuple = ()
    timings_ms: object = None
    deadline_miss: bool = False
    global_result: object = None
    action: object = None
    action_certificate: object = None
    reference_phase_s: object = None


class NavigationCoordinator:
    def __init__(self, planner, *, runner=None, limits=None, backend=None,
                 allow_unknown=False, local_period_s=.05, global_period_s=15.,
                 global_budget_s=1., clearance_radius_m=.75, field_backend=None,
                 revalidate_current_map=False, reference_smoothing=False,
                 global_soft_clearance_m=.4, global_clearance_weight=1.0):
        for name,value in (("global_soft_clearance_m",global_soft_clearance_m),
                           ("global_clearance_weight",global_clearance_weight)):
            if isinstance(value,bool) or not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        self.global_soft_clearance_m = float(global_soft_clearance_m)
        self.global_clearance_weight = float(global_clearance_weight)
        self.planner = planner
        self.limits = WheelLimits() if limits is None else limits
        self.backend = backend
        self.allow_unknown = allow_unknown
        # 全局低频后台刷新；目标和参考失效事件无需等待周期。
        self.local_period_s, self.global_period_s = local_period_s, global_period_s
        if isinstance(global_budget_s,bool) or not np.isfinite(global_budget_s) or global_budget_s < 0:
            raise ValueError('global_budget_s must be finite and nonnegative')
        self.global_budget_s = float(global_budget_s)
        self.clearance_radius_m = self.limits.global_clearance_radius(clearance_radius_m)
        self.goal_tolerance_m = .1
        self.goal_position_target_m = None
        self.goal_yaw_tolerance_rad = .08726646259971647
        self.stopped_linear_mps = self.stopped_angular_rps = .01
        self.route = self.progress = None
        if type(reference_smoothing) is not bool:raise ValueError("reference_smoothing must be bool")
        if reference_smoothing and not revalidate_current_map:
            raise ValueError("reference smoothing requires current map revalidation")
        self.reference_smoothing=reference_smoothing
        self._raw_route=self._shaped_route=None
        self._goal = self._epoch = self._policy = None
        self._goal_yaw = None
        self._position_completed = False
        self.goal_revision = 0
        self._global_inflight = None
        self._next_local = self._next_global = -float('inf')
        self._version = 0
        self._pending_reason = 'INITIAL'
        self._field_cache = DistanceFieldCache(backend=field_backend)
        self._goal_checker = AdaptiveGlobalPlanner(self.limits, self.clearance_radius_m)
        # 原型默认严格 epoch 身份；P3 持续更新地图时显式开启当前图重验。
        self.revalidate_current_map = bool(revalidate_current_map)
        self._accepted_goal_generation = None
        self._observed_world = self._blocked = None
        self._blocked_boxes = np.empty((0, 4))
        self._goal_check_key = self._cached_goal_check = None
        self.runner = GlobalWorker() if runner is None else runner

    @property
    def position_target_tolerance_m(self):
        """位置收敛目标由导航owner统一定义，不能宽于公共验收上限。"""
        target=self.goal_tolerance_m if self.goal_position_target_m is None else self.goal_position_target_m
        if not np.isfinite(target) or target<=0:
            raise ValueError('goal_position_target_m must be finite and positive')
        return min(self.goal_tolerance_m,float(target))

    def close(self):
        if hasattr(self.runner, 'close'): self.runner.close()

    def invalidate_goal(self, *, generation):
        """退休所有在途身份；取消不提交新的规划请求。"""
        self.goal_revision += 1
        self._accepted_goal_generation = generation
        self._goal = self._goal_yaw = self.route = self.progress = None
        self._position_completed = False
        self._global_inflight = None
        self._pending_reason = 'GOAL_CHANGED'

    def cancel_tick(self, world, state, now_s, *, need_action=False):
        """只认证从当前实测状态开始的受限制动，绝不续发旧目标路径。"""
        grid = extract_window(world, state, self.allow_unknown)
        reference = LocalReference(-1, grid.epoch, [[state.x, state.y]], [0.], 0.)
        candidate = brake_candidate(state, grid, reference, self.limits)
        certificate = certify(candidate, grid, self.limits, -1, backend=self.backend)
        if not certificate.matches(candidate, grid, self.limits, -1):
            candidate = None
        action = action_certificate = None
        if candidate is not None and need_action:
            action = action_prefix(candidate, state, getattr(self, 'execution_period_s', self.local_period_s))
            action_certificate = certify(action, grid, self.limits, -1, backend=self.backend)
            if not action_certificate.matches(action, grid, self.limits, -1):
                action = None
        return CycleResult('CANCEL_BRAKING' if candidate is not None else 'NO_CERTIFIED_ACTION',
                           candidate=candidate, certificate=certificate, grid=grid, reference=reference,
                           action=action, action_certificate=action_certificate)

    def replace_goal(self, goal, *, goal_yaw=None, generation):
        """会话接收事件入口；相同最终位姿也不能复活较早代际的结果。"""
        if type(generation) is not int or generation < 1:
            raise ValueError('accepted goal generation must be a positive integer')
        if self._accepted_goal_generation is not None and generation < self._accepted_goal_generation:
            raise ValueError('accepted goal generation must be monotonic')
        self._set_goal(goal, goal_yaw, generation=generation)

    def _set_goal(self, goal, goal_yaw, *, generation=None):
        """每次输入都更新身份，含非执行周期内的 A→B→A；不改写反馈。"""
        goal = tuple(float(x) for x in goal)
        if len(goal) != 2 or not np.isfinite(goal).all():
            raise ValueError('goal must contain finite x,y')
        yaw = None if goal_yaw is None else normalize_yaw(goal_yaw)
        same_yaw = (yaw is None and self._goal_yaw is None or
                    yaw is not None and self._goal_yaw is not None and
                    abs(angle_error(yaw, self._goal_yaw)) <= 1e-12)
        new_generation = generation is not None and generation != self._accepted_goal_generation
        if goal != self._goal or not same_yaw or new_generation:
            self.goal_revision += 1
            self._goal, self._goal_yaw = goal, yaw
            self._position_completed = False
            self.route = self.progress = None
            self._pending_reason = 'GOAL_CHANGED'
        if generation is not None:
            self._accepted_goal_generation = generation

    def _observe_world(self, world):
        """只在不可变快照更换时比较障碍，避免20Hz重复构建全域障碍数组。"""
        if world is self._observed_world:
            return False
        changed = world.blocked != self._blocked
        self._observed_world = world
        if changed:
            self._blocked = world.blocked
            radius = self.clearance_radius_m
            self._blocked_boxes = np.asarray([(r.x0 / 100., r.y0 / 100., r.x1 / 100., r.y1 / 100.)
                                             for r in world.blocked], dtype=float).reshape(-1, 4)
            self._blocked_boxes = self._blocked_boxes + [-radius, -radius, radius, radius]
        return changed

    @property
    def raw_route(self):
        # 缓存只能属于当前有效参考；等待/取消不能暴露已退休几何。
        if self.route is None:return None
        if self._shaped_route is not None and self._shaped_route.route is self.route:
            return self._raw_route
        return self.route

    def _adopt_route(self, route, events, *, raw_verified=False, shaped_reference=None, smoothing_ms=None):
        shaped=None
        began=perf_counter()
        if self.reference_smoothing and shaped_reference is not None:
            from .reference_smoothing import reference_clear, ShapedRoute
            shaped=shaped_reference
            # 子进程完成的曲线只在地图变化时复验；仍使用当前地图完整几何检查。
            if (shaped.changed and route.map_epoch != self._epoch and
                    not reference_clear(shaped,self._blocked_boxes,corner_radius=self.clearance_radius_m)):
                shaped=ShapedRoute(route,np.empty((0,4)),False,'CURRENT_MAP_REJECTED')
        elif self.reference_smoothing:
            from .reference_smoothing import shape_route
            if not raw_verified and not self._route_clear(route):
                raise ValueError("raw route collision")
            shaped=shape_route(route,self._blocked_boxes,raw_verified=True,budget_s=self.global_budget_s,
                               terminal_yaw=self._goal_yaw,
                               soft_clearance_m=self.global_soft_clearance_m,
                               clearance_weight=self.global_clearance_weight,corner_radius=self.clearance_radius_m)
        self._raw_route,self._shaped_route=route,shaped
        self.route=route if shaped is None else shaped.route
        self.progress=None
        if shaped is not None:
            events.append(dict(kind='GLOBAL_REFERENCE_SHAPED',version=route.version,
                map_epoch=route.map_epoch,changed=shaped.changed,reason=shaped.reason,
                raw_points=len(route.points),reference_points=len(self.route.points),
                smoothing_ms=(perf_counter()-began)*1000 if smoothing_ms is None else smoothing_ms,
                adoption_ms=(perf_counter()-began)*1000))

    def _route_clear(self, route, *, start_s=None):
        if start_s is not None:
            # 当前参考执行的是采样折线；只检查从本次有界投影开始的剩余段。
            # 新来源默认仍全验，整形构造的连续曲线凸包证明也不改变。
            arc = _arc_lengths(route.points)
            points = np.vstack((_point_at(route.points, arc, start_s),
                                route.points[np.searchsorted(arc, start_s, side='right'):]))
            return all(_segment_clear(a, b, self._blocked_boxes,corner_radius=self.clearance_radius_m)
                       for a, b in zip(points[:-1], points[1:]))
        if self._shaped_route is not None and self._shaped_route.route is route:
            from .reference_smoothing import reference_clear
            return reference_clear(self._shaped_route,self._blocked_boxes,corner_radius=self.clearance_radius_m)
        return all(_segment_clear(a, b, self._blocked_boxes,corner_radius=self.clearance_radius_m) for a, b in zip(route.points[:-1], route.points[1:]))

    @staticmethod
    def _same_route_geometry(current, incoming):
        """仅同一折线的前向后缀等价；不能吞掉障碍移除后的更短路线。"""
        if current.goal != incoming.goal:
            return False
        vectors = np.diff(current.points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        arc = np.r_[0., np.cumsum(lengths)]
        projected = []
        for point in incoming.points:
            fractions = np.clip(np.sum((point - current.points[:-1]) * vectors, axis=1) / lengths ** 2, 0., 1.)
            distances = np.linalg.norm(current.points[:-1] + fractions[:, None] * vectors - point, axis=1)
            index = int(np.argmin(distances))
            if distances[index] > 1e-6:
                return False
            projected.append(arc[index] + fractions[index] * lengths[index])
        return bool(np.all(np.diff(projected) >= -1e-6) and
                    abs(sum(np.linalg.norm(np.diff(incoming.points, axis=0), axis=1)) -
                        (projected[-1] - projected[0])) <= 1e-6)

    def goal_reached(self, state):
        """统一位姿/停稳判据；任务完成仍由P3新实测观测确认。"""
        return (self._goal is not None and state is not None and
                abs(state.v)<=self.stopped_linear_mps and abs(state.w)<=self.stopped_angular_rps and
                (self._goal_yaw is not None and self._position_completed or
                 np.hypot(state.x-self._goal[0],state.y-self._goal[1])<=self.goal_tolerance_m) and
                (self._goal_yaw is None or abs(angle_error(self._goal_yaw,state.yaw))<=self.goal_yaw_tolerance_rad))

    def _local_reference(self, world, state, grid):
        """末段从当前状态收敛位置；不把沿线投影到头当作实际到达。"""
        reference, self.progress = extract_reference(
            self.route, state, grid, self.progress, terminal_yaw=self._goal_yaw)
        distance = float(np.hypot(state.x-self._goal[0], state.y-self._goal[1]))
        remaining = float(reference.s[-1]-reference.progress_s)
        if (distance > TERMINAL_APPROACH_DISTANCE_M or
                remaining > TERMINAL_APPROACH_DISTANCE_M or
                not np.array_equal(reference.points[-1], self._goal)):
            return reference
        # 任务最终yaw仍保存在导航owner；未到位置时，局部参考只负责位置收敛。
        reference = replace(reference, terminal_yaw=None)
        if distance <= 1e-12:
            return reference
        self._observe_world(world)
        xy = np.asarray((state.x, state.y))
        # 复用现有闭边界线段几何，不越过已知膨胀障碍；实际轨迹仍须完整认证。
        if not _segment_clear(xy, self._goal, self._blocked_boxes):
            return reference
        if not grid.allow_unknown:
            # 仅检查候选直连附近未知格；不把原可用绕行参考换成穿未知的弦。
            radius = self.clearance_radius_m
            origin = np.asarray(grid.origin_xy)
            shape = np.asarray(grid.cells.shape[::-1])
            low = np.maximum(0, np.floor((np.minimum(xy, self._goal)-radius-origin)/grid.resolution_m).astype(int)-1)
            high = np.minimum(shape, np.floor((np.maximum(xy, self._goal)+radius-origin)/grid.resolution_m).astype(int)+2)
            rows, cols = np.nonzero(grid.cells[low[1]:high[1],low[0]:high[0]] == CellState.UNKNOWN)
            lower = origin + np.column_stack((cols+low[0],rows+low[1]))*grid.resolution_m
            boxes = np.column_stack((lower-radius,lower+grid.resolution_m+radius))
            if not _segment_clear(xy, self._goal, boxes):
                return reference
        start_s = reference.progress_s
        return LocalReference(reference.route_version, grid.epoch,
                              [xy, self._goal], [start_s, start_s+distance], start_s)

    def _terminal_plan(self, state, grid, reference):
        yaw_satisfied = (self._goal_yaw is None or
                         abs(angle_error(self._goal_yaw, state.yaw)) <= self.goal_yaw_tolerance_rad)
        if yaw_satisfied:
            # 已有停稳阈值定义可接受的残余速度；不再用预测停车点
            # 二次收紧完成范围，后续新实测越界仍会清除到达保持证据。
            candidate = brake_candidate(state, grid, reference, self.limits)
            arrived=self.goal_reached(state)
            return candidate, 'ARRIVED' if arrived else 'GOAL_BRAKING'
        candidate = terminal_candidate(state, grid, reference, self.limits)
        return candidate, 'GOAL_TURNING' if candidate.source.startswith('terminal-yaw:bounded') else 'GOAL_BRAKING'

    def _update_route(self, world, state, goal, now_s, events, *, goal_yaw=None, observed_state=None):
        """共享全局身份、目标检查与非阻塞路线调度；不执行局部求解。"""
        self._set_goal(goal, goal_yaw)
        goal = self._goal
        blocked_changed = self._observe_world(world) if (self.revalidate_current_map or self.reference_smoothing) else False
        if world.epoch != self._epoch:
            if not self.revalidate_current_map:
                self._pending_reason = 'MAP_CHANGED'
            events.append({'kind': 'MAP_CHANGED', 'epoch': world.epoch})
        if self.revalidate_current_map and blocked_changed and self.route is not None:
            start_s = 0.
            if self.progress is not None and self.progress.route_version == self.route.version:
                try:
                    start_s = project_progress(self.route, state, self.progress).s
                except ValueError:
                    # 非法进度不提供历史段豁免；后续extract_reference仍走原重入处理。
                    pass
            if not self._route_clear(self.route, start_s=start_s):
                events.append({'kind': 'GLOBAL_CURRENT_MAP_REJECTED', 'version': self.route.version,
                               'requested_epoch': self.route.map_epoch, 'current_epoch': world.epoch})
                self.route = self.progress = None
            else:
                events.append({'kind': 'GLOBAL_CURRENT_MAP_REVALIDATED', 'version': self.route.version,
                               'requested_epoch': self.route.map_epoch, 'current_epoch': world.epoch})
        if (self.revalidate_current_map and blocked_changed and self.route is None and
                self._pending_reason not in ('INITIAL', 'GOAL_CHANGED')):
            self._pending_reason = 'MAP_CHANGED'
        if self._policy is not None and self._policy != self.allow_unknown:
            events.append({'kind': 'UNKNOWN_POLICY_CHANGED', 'allow_unknown': self.allow_unknown})
        self._epoch, self._policy = world.epoch, self.allow_unknown
        motion = np.array([state.x, state.y, state.yaw, state.v, state.w])
        global_result = None
        try:
            raw = self.runner.poll()
            if raw is not None:
                submitted, self._global_inflight = self._global_inflight, None
                global_result = restore_result(raw)
                events.append({'kind': 'GLOBAL_RESULT_RECEIVED',
                               'accepted_generation_at_submit': None if submitted is None else submitted.get('accepted_generation'),
                               'goal_revision_at_submit': None if submitted is None else submitted['goal_revision']})
                route = global_result['route']
                if (submitted is None or submitted['goal_revision'] != self.goal_revision or
                        submitted['version'] != global_result['requested_version'] or
                        submitted['epoch'] != global_result['requested_epoch'] or
                        tuple(global_result['requested_goal']) != goal or
                        (not self.revalidate_current_map and global_result['requested_epoch'] != world.epoch)):
                    events.append({'kind': 'STALE_GLOBAL_RESULT', 'version': global_result['requested_version']})
                elif route is not None:
                    current_clear = not self.revalidate_current_map or self._route_clear(route)
                    if not current_clear:
                        events.append({'kind': 'GLOBAL_CURRENT_MAP_REJECTED', 'version': route.version,
                                       'requested_epoch': route.map_epoch, 'current_epoch': world.epoch})
                        route = None
                    elif self.revalidate_current_map and route.map_epoch != world.epoch:
                        events.append({'kind': 'GLOBAL_CURRENT_MAP_REVALIDATED', 'version': route.version,
                                       'requested_epoch': route.map_epoch, 'current_epoch': world.epoch})
                    # 同几何前向后缀保留版本和进度；不同合法路线可接替。
                    keep = (self.route is not None and route is not None and
                            (self.revalidate_current_map and self._same_route_geometry(self.raw_route, route) or
                             (not self.revalidate_current_map and self.route.map_epoch == route.map_epoch and global_result.get('request_reason') == 'PERIODIC')))
                    if route is not None and not keep:
                        self._adopt_route(route,events,raw_verified=True,
                            shaped_reference=global_result.get("shaped_reference"),smoothing_ms=global_result.get("smoothing_ms"))
                        events.append({'kind': 'ROUTE_ACCEPTED', 'version': route.version, 'map_epoch': route.map_epoch, 'points': route.points.tolist()})
                    elif route is not None:
                        events.append({'kind': 'ROUTE_CONTINUITY_RETAINED', 'version': self.route.version, 'computed_version': route.version})
                    if self.revalidate_current_map and route is not None and self._pending_reason == 'MAP_CHANGED':
                        # 此结果已按最新障碍复验；在途地图事件无需再重复求解。
                        self._pending_reason = None
                else:
                    events.append({'kind': 'GLOBAL_FAILED', 'reason': global_result['reason_code']})
        except Exception as exc:
            self._global_inflight = None
            events.append({'kind': 'GLOBAL_ERROR', 'reason': type(exc).__name__, 'detail': str(exc)})
        if (self.revalidate_current_map and self._cached_goal_check is not None and
                not blocked_changed and self._goal_check_key == self.goal_revision):
            goal_check = GoalCheck(self._cached_goal_check.accepted, self._cached_goal_check.reason_code, world.epoch)
        else:
            goal_check = (self._goal_checker.check_goal(world, goal) if self._goal_yaw is None else
                          check_pose_goal(world, goal, self._goal_yaw, self.limits))
            self._goal_check_key, self._cached_goal_check = self.goal_revision, goal_check
        # 规划与认证继续使用state；终点阶段只由真实观测确认。
        observed = state if observed_state is None else observed_state
        distance = np.hypot(goal[0]-observed.x, goal[1]-observed.y)
        # 同一目标的位置阶段只完成一次；进入最终转向后，位置漂移不重开行进。
        # 该历史仍由新目标、目标替换或取消清除，地图/路线刷新不清除。
        if self._goal_yaw is None:
            # 无最终朝向的目标没有朝向阶段，保留既有位置/制动交接。
            arrived_xy = (distance <= self.position_target_tolerance_m or
                          distance <= self.goal_tolerance_m and abs(observed.v) <= self.stopped_linear_mps)
        else:
            if distance <= self.position_target_tolerance_m or self.goal_reached(observed):
                self._position_completed = True
            arrived_xy = self._position_completed
        global_due = now_s + 1e-9 >= self._next_global
        if (goal_check.accepted and not arrived_xy and not self.runner.busy and
                (self._pending_reason or global_due)):
            reason = self._pending_reason or 'PERIODIC'
            self._version += 1
            request = make_request(world, motion[:2], goal, self._version, self.limits, self.clearance_radius_m, self.global_budget_s,
                global_soft_clearance_m=self.global_soft_clearance_m,
                global_clearance_weight=self.global_clearance_weight,
                reference_smoothing=self.reference_smoothing,
                terminal_yaw=self._goal_yaw)
            request['request_reason'] = reason
            # 每次尝试都消费事件并推进既有周期，submit异常也不能每tick重试。
            self._pending_reason = None
            self._next_global = now_s + self.global_period_s
            try:
                self.runner.submit(request)
            except Exception as exc:
                events.append({'kind': 'GLOBAL_SUBMIT_ERROR', 'reason': type(exc).__name__,
                               'detail': str(exc), 'request_reason': reason, 'version': self._version,
                               'epoch': world.epoch, 'goal': goal})
            else:
                # 单在途全局几何 worker 不必理解 yaw；父端绑定完整目标提交身份。
                self._global_inflight = {'version': self._version, 'goal_revision': self.goal_revision,
                                         'epoch': world.epoch, 'allow_unknown': self.allow_unknown,
                                         'accepted_generation': self._accepted_goal_generation}
                self._pending_reason = None
                events.append({'kind': 'GLOBAL_SUBMITTED', 'reason': reason, 'version': self._version})
        return goal_check, arrived_xy, motion, global_result

    def tick(self, world, state, goal, now_s, *, goal_yaw=None, need_action=True, observed_state=None):
        started = perf_counter()
        self._set_goal(goal, goal_yaw)
        if now_s + 1e-9 < self._next_local:
            return CycleResult('NOT_DUE', timings_ms={'total': (perf_counter()-started)*1000})
        self._next_local = now_s + self.local_period_s
        timings, events = {}, []
        stage = perf_counter()
        goal_check, arrived_xy, motion, global_result = self._update_route(
            world, state, goal, now_s, events, goal_yaw=goal_yaw, observed_state=observed_state)
        timings['ipc_schedule_goal'] = (perf_counter()-stage)*1000
        stage = perf_counter()
        grid = extract_window(world, state, self.allow_unknown)
        timings['map'] = (perf_counter()-stage)*1000
        stage = perf_counter()
        version = self.route.version if self.route is not None else -1
        reference = LocalReference(version, grid.epoch, [[state.x, state.y]], [0.], 0.,
                                   self._goal_yaw if arrived_xy else None)
        status = 'WAITING_GLOBAL'
        # 到达位置后直接按当前目标车体朝向转向，旧路线进度和截窗不能覆盖末端参考。
        if self.route is not None and not arrived_xy:
            try:
                reference = self._local_reference(world, state, grid)
            except ValueError as exc:
                events.append({'kind': 'REFERENCE_REJECTED', 'reason': str(exc)})
                self._pending_reason = 'REFERENCE_REENTRY'
                if self.revalidate_current_map:
                    self.route = self.progress = None
        timings['reference'] = (perf_counter()-stage)*1000
        candidate = certificate = None
        if not goal_check.accepted:
            status = goal_check.reason_code
            events.append({'kind': 'GOAL_REJECTED', 'reason': status})
        elif arrived_xy:
            candidate, status = self._terminal_plan(state, grid, reference)
        elif len(reference.points) >= 2:
            stage = perf_counter()
            field = self._field_cache.get(grid)
            field_reused = self._field_cache.last_reused
            timings['field_cache_lookup'] = (perf_counter()-stage)*1000
            timings['field_build'] = 0. if field_reused else timings['field_cache_lookup']
            events.append({'kind': 'SDF_SOURCE', 'grid_digest': grid.digest(), 'reused': field_reused})
            timings['sdf'] = (perf_counter()-stage)*1000
            stage = perf_counter()
            try:
                plan = self.planner.plan(state, grid, reference, distance_field=field, field_grid_digest=grid.digest())
                candidate = plan.candidate
                timings.update({'planner_'+k: v for k, v in plan.timings_ms.items()})
                events.append({'kind': 'LOCAL_RESULT', 'reason': plan.reason, 'attempts': plan.attempts})
                status = plan.reason
            except Exception as exc:
                events.append({'kind': 'LOCAL_ERROR', 'reason': type(exc).__name__, 'detail': str(exc)})
                status = 'LOCAL_ERROR'
            timings['planner_wall'] = (perf_counter()-stage)*1000
        stage = perf_counter()
        context = prepare_certification(grid, backend=self.backend)
        timings['certification_prepare'] = (perf_counter()-stage)*1000
        stage = perf_counter()
        if candidate is not None:
            certificate = certify(candidate, grid, self.limits, version, backend=self.backend, context=context)
            if not certificate.matches(candidate, grid, self.limits, version) or not np.allclose(candidate.states[0,:5], motion, atol=1e-6, rtol=0):
                events.append({'kind': 'CANDIDATE_REJECTED', 'reason': certificate.reason.value if not certificate.valid else 'INITIAL_STATE_MISMATCH'})
                candidate = certificate = None
        timings['certification_identity'] = (perf_counter()-stage)*1000
        action = action_certificate = None
        if candidate is not None and need_action:
            stage = perf_counter()
            try:
                action = action_prefix(candidate, state)
                action_certificate = certify(action, grid, self.limits, version, backend=self.backend, context=context)
                if not action_certificate.matches(action, grid, self.limits, version):
                    events.append({'kind': 'ACTION_REJECTED', 'reason': action_certificate.reason.value})
                    candidate = action = None
            except ValueError as exc:
                events.append({'kind': 'ACTION_REJECTED', 'reason': 'ACTION_PREFIX_INVALID', 'detail': str(exc)})
                candidate = action = None
            timings['action_gate'] = (perf_counter()-stage)*1000
        # 软周期只观测耗时；完整候选均须认证，离线消费动作时前缀另行认证。
        if candidate is None:
            stage = perf_counter()
            candidate = brake_candidate(state, grid, reference, self.limits)
            certificate = certify(candidate, grid, self.limits, version, backend=self.backend, context=context)
            if not certificate.matches(candidate, grid, self.limits, version) or not np.allclose(candidate.states[0,:5], motion, atol=1e-6, rtol=0):
                events.append({'kind': 'BRAKE_REJECTED', 'reason': certificate.reason.value})
                candidate = None
                status = 'NO_CERTIFIED_ACTION'
            action = action_certificate = None
            if candidate is not None and need_action:
                action = action_prefix(candidate, state)
                action_certificate = certify(action, grid, self.limits, version, backend=self.backend, context=context)
                if not action_certificate.matches(action, grid, self.limits, version):
                    events.append({'kind': 'BRAKE_ACTION_REJECTED', 'reason': action_certificate.reason.value})
                    candidate = action = None
                    status = 'NO_CERTIFIED_ACTION'
            timings['brake'] = (perf_counter()-stage)*1000
        timings['total'] = (perf_counter()-started)*1000
        deadline_miss = timings['total'] > self.local_period_s*1000
        if deadline_miss: events.append({'kind': 'LOCAL_DEADLINE_MISS', 'total_ms': timings['total']})
        return CycleResult(status, candidate, certificate, grid, reference, tuple(events), timings, deadline_miss, global_result, action, action_certificate)
