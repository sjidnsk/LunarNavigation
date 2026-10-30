"""只组合地图、全局参考、MPCC 和认证；tick 只输出离线动作数据。"""
from dataclasses import dataclass
from time import perf_counter
import numpy as np

from .contracts import LocalReference, WheelLimits
from .maps import extract_window, DistanceFieldCache
from .reference import extract_reference
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


class NavigationCoordinator:
    def __init__(self, planner, *, runner=None, limits=None, backend=None,
                 allow_unknown=False, local_period_s=.05, global_period_s=1.,
                 global_budget_s=1., clearance_radius_m=.9, stall_timeout_s=4., field_backend=None,
                 revalidate_current_map=False, reference_smoothing=False):
        self.planner = planner
        self.runner = GlobalWorker() if runner is None else runner
        self.limits = WheelLimits() if limits is None else limits
        self.backend = backend
        self.allow_unknown = allow_unknown
        self.local_period_s, self.global_period_s = local_period_s, global_period_s
        self.global_budget_s = global_budget_s
        self.clearance_radius_m = self.limits.global_clearance_radius(clearance_radius_m)
        self.stall_timeout_s = stall_timeout_s
        self.goal_tolerance_m = .1
        self.goal_position_target_m = None
        self.goal_yaw_tolerance_rad = .05
        self.stopped_linear_mps = self.stopped_angular_rps = .01
        self.route = self.progress = None
        if type(reference_smoothing) is not bool:raise ValueError("reference_smoothing must be bool")
        if reference_smoothing and not revalidate_current_map:
            raise ValueError("reference smoothing requires current map revalidation")
        self.reference_smoothing=reference_smoothing
        self._raw_route=self._shaped_route=None
        self._goal = self._epoch = self._policy = None
        self._goal_yaw = None
        self.goal_revision = 0
        self._global_inflight = None
        self._next_local = self._next_global = -float('inf')
        self._version = 0
        self._pending_reason = 'INITIAL'
        self._motion = None
        self._motion_s = 0.
        self._stalled = False
        self._field_cache = DistanceFieldCache(backend=field_backend)
        self._goal_checker = AdaptiveGlobalPlanner(self.limits, self.clearance_radius_m)
        # 原型默认严格 epoch 身份；P3 持续更新地图时显式开启当前图重验。
        self.revalidate_current_map = bool(revalidate_current_map)
        self._accepted_goal_generation = None
        self._observed_world = self._blocked = None
        self._blocked_boxes = np.empty((0, 4))
        self._goal_check_key = self._cached_goal_check = None

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
        self._global_inflight = None
        self._pending_reason = 'GOAL_CHANGED'

    def cancel_tick(self, world, state, now_s):
        """只认证从当前实测状态开始的受限制动，绝不续发旧目标路径。"""
        grid = extract_window(world, state, self.allow_unknown)
        reference = LocalReference(-1, grid.epoch, [[state.x, state.y]], [0.], 0.)
        candidate = brake_candidate(state, grid, reference, self.limits)
        certificate = certify(candidate, grid, self.limits, -1, backend=self.backend)
        if not certificate.matches(candidate, grid, self.limits, -1):
            candidate = None
        return CycleResult('CANCEL_BRAKING' if candidate is not None else 'NO_CERTIFIED_ACTION',
                           candidate=candidate, certificate=certificate, grid=grid, reference=reference)

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
            self.route = self.progress = None
            self._pending_reason = 'GOAL_CHANGED'
            self._motion = None
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

    def _adopt_route(self, route, events):
        shaped=None
        if self.reference_smoothing:
            from .reference_smoothing import shape_route
            shaped=shape_route(route,self._blocked_boxes)
        self._raw_route,self._shaped_route=route,shaped
        self.route=route if shaped is None else shaped.route
        self.progress=None
        if shaped is not None:
            events.append(dict(kind='GLOBAL_REFERENCE_SHAPED',version=route.version,
                map_epoch=route.map_epoch,changed=shaped.changed,reason=shaped.reason,
                raw_points=len(route.points),reference_points=len(self.route.points)))

    def _route_clear(self, route):
        if self._shaped_route is not None and self._shaped_route.route is route:
            from .reference_smoothing import reference_clear
            return reference_clear(self._shaped_route,self._blocked_boxes)
        return all(_segment_clear(a, b, self._blocked_boxes) for a, b in zip(route.points[:-1], route.points[1:]))

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

    def _terminal_plan(self, state, grid, reference):
        yaw_satisfied = (self._goal_yaw is None or
                         abs(angle_error(self._goal_yaw, state.yaw)) <= self.goal_yaw_tolerance_rad)
        stopped = abs(state.v) <= self.stopped_linear_mps and abs(state.w) <= self.stopped_angular_rps
        if yaw_satisfied:
            # 低于停稳阈值仍有真实运动；到达状态不能紧接着被自身制动带出容差。
            candidate = brake_candidate(state, grid, reference, self.limits)
            last = candidate.states[-1]
            final_pose_satisfied = (
                np.linalg.norm(last[:2] - self._goal) <= self.position_target_tolerance_m and
                (self._goal_yaw is None or
                 abs(angle_error(self._goal_yaw, last[2])) <= self.goal_yaw_tolerance_rad))
            return candidate, 'ARRIVED' if stopped and final_pose_satisfied else 'GOAL_BRAKING'
        candidate = terminal_candidate(state, grid, reference, self.limits)
        return candidate, 'GOAL_TURNING' if candidate.source == 'terminal-yaw:bounded' else 'GOAL_BRAKING'

    def _update_route(self, world, state, goal, now_s, events, *, goal_yaw=None):
        """共享全局身份、目标检查与非阻塞路线调度；不执行局部求解。"""
        self._set_goal(goal, goal_yaw)
        goal = self._goal
        blocked_changed = self._observe_world(world) if (self.revalidate_current_map or self.reference_smoothing) else False
        if world.epoch != self._epoch:
            if not self.revalidate_current_map or blocked_changed:
                self._pending_reason = 'MAP_CHANGED'
            events.append({'kind': 'MAP_CHANGED', 'epoch': world.epoch})
        if self.revalidate_current_map and blocked_changed and self.route is not None:
            if not self._route_clear(self.route):
                events.append({'kind': 'GLOBAL_CURRENT_MAP_REJECTED', 'version': self.route.version,
                               'requested_epoch': self.route.map_epoch, 'current_epoch': world.epoch})
                self.route = self.progress = None
            else:
                events.append({'kind': 'GLOBAL_CURRENT_MAP_REVALIDATED', 'version': self.route.version,
                               'requested_epoch': self.route.map_epoch, 'current_epoch': world.epoch})
        if self._policy is not None and self._policy != self.allow_unknown:
            events.append({'kind': 'UNKNOWN_POLICY_CHANGED', 'allow_unknown': self.allow_unknown})
        self._epoch, self._policy = world.epoch, self.allow_unknown
        motion = np.array([state.x, state.y, state.yaw, state.v, state.w])
        if self._motion is None or np.linalg.norm(motion[:2]-self._motion[:2]) > .015 or abs(motion[2]-self._motion[2]) > .015 or np.linalg.norm(motion[3:]-self._motion[3:]) > .015:
            self._motion, self._motion_s, self._stalled = motion, now_s, False
        elif self.route is not None and now_s-self._motion_s >= self.stall_timeout_s and not self._stalled:
            self._pending_reason, self._stalled = 'NO_PROGRESS', True
            events.append({'kind': 'REPLAN_REQUIRED', 'reason': 'NO_PROGRESS'})
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
                        (not self.revalidate_current_map and global_result['requested_epoch'] != world.epoch) or
                        (self.revalidate_current_map and submitted.get('allow_unknown') != self.allow_unknown)):
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
                    # 同图周期重算保留现有路线，避免从移动起点裁短导致进度每秒清零。
                    keep = (self.route is not None and route is not None and
                            ((self.revalidate_current_map and self._same_route_geometry(self.raw_route, route)) or
                             (not self.revalidate_current_map and self.route.map_epoch == route.map_epoch and global_result.get('request_reason') == 'PERIODIC')))
                    if route is not None and not keep:
                        self._adopt_route(route,events)
                        events.append({'kind': 'ROUTE_ACCEPTED', 'version': route.version, 'map_epoch': route.map_epoch, 'points': route.points.tolist()})
                    elif route is not None:
                        events.append({'kind': 'ROUTE_CONTINUITY_RETAINED', 'version': self.route.version, 'computed_version': route.version})
                else:
                    events.append({'kind': 'GLOBAL_FAILED', 'reason': global_result['reason_code']})
        except Exception as exc:
            events.append({'kind': 'GLOBAL_ERROR', 'reason': type(exc).__name__, 'detail': str(exc)})
        if (self.revalidate_current_map and self._cached_goal_check is not None and
                not blocked_changed and self._goal_check_key == self.goal_revision):
            goal_check = GoalCheck(self._cached_goal_check.accepted, self._cached_goal_check.reason_code, world.epoch)
        else:
            goal_check = (self._goal_checker.check_goal(world, goal) if self._goal_yaw is None else
                          check_pose_goal(world, goal, self._goal_yaw, self.limits))
            self._goal_check_key, self._cached_goal_check = self.goal_revision, goal_check
        arrived_xy = np.linalg.norm(np.array(goal)-motion[:2]) <= self.position_target_tolerance_m
        global_due = now_s + 1e-9 >= self._next_global
        coalesced_map_update = (self.revalidate_current_map and self.route is not None and
                                self._pending_reason == 'MAP_CHANGED' and not global_due)
        if (goal_check.accepted and not arrived_xy and not self.runner.busy and
                not coalesced_map_update and (self._pending_reason or global_due)):
            reason = self._pending_reason or 'PERIODIC'
            self._version += 1
            request = make_request(world, motion[:2], goal, self._version, self.limits, self.clearance_radius_m, self.global_budget_s)
            request['request_reason'] = reason
            try:
                self.runner.submit(request)
            except Exception as exc:
                # 全局进程故障不打断本次执行；下次根据最新地图、目标与反馈重建请求。
                self._pending_reason = reason
                events.append({'kind': 'GLOBAL_SUBMIT_ERROR', 'reason': type(exc).__name__,
                               'detail': str(exc), 'request_reason': reason, 'version': self._version,
                               'epoch': world.epoch, 'goal': goal})
            else:
                # 单在途全局几何 worker 不必理解 yaw；父端绑定完整目标提交身份。
                self._global_inflight = {'version': self._version, 'goal_revision': self.goal_revision,
                                         'epoch': world.epoch, 'allow_unknown': self.allow_unknown,
                                         'accepted_generation': self._accepted_goal_generation}
                self._pending_reason = None
                self._next_global = now_s+self.global_period_s
                events.append({'kind': 'GLOBAL_SUBMITTED', 'reason': reason, 'version': self._version})
        return goal_check, arrived_xy, motion, global_result

    def tick(self, world, state, goal, now_s, *, goal_yaw=None):
        started = perf_counter()
        self._set_goal(goal, goal_yaw)
        if now_s + 1e-9 < self._next_local:
            return CycleResult('NOT_DUE', timings_ms={'total': (perf_counter()-started)*1000})
        self._next_local = now_s + self.local_period_s
        timings, events = {}, []
        stage = perf_counter()
        goal_check, arrived_xy, motion, global_result = self._update_route(world, state, goal, now_s, events, goal_yaw=goal_yaw)
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
                reference, self.progress = extract_reference(self.route, state, grid, self.progress, terminal_yaw=self._goal_yaw)
            except ValueError as exc:
                events.append({'kind': 'REFERENCE_REJECTED', 'reason': str(exc)})
                self._pending_reason = 'REFERENCE_REENTRY'
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
        if candidate is not None:
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
        # 软周期只观测耗时；候选和前缀已独立认证，时间不能改变动作选择。
        if candidate is None:
            stage = perf_counter()
            candidate = brake_candidate(state, grid, reference, self.limits)
            certificate = certify(candidate, grid, self.limits, version, backend=self.backend, context=context)
            if not certificate.matches(candidate, grid, self.limits, version) or not np.allclose(candidate.states[0,:5], motion, atol=1e-6, rtol=0):
                events.append({'kind': 'BRAKE_REJECTED', 'reason': certificate.reason.value})
                candidate = None
                status = 'NO_CERTIFIED_ACTION'
            action = action_certificate = None
            if candidate is not None:
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
