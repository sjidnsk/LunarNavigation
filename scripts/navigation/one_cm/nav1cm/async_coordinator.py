"""异步局部规划与连续执行；旧同步协调器保持独立。"""
from dataclasses import replace
from time import perf_counter, process_time
import numpy as np
from .contracts import LocalReference
from .maps import extract_window
from .reference import extract_reference
from .certification import certify, prepare_certification
from .seeds import brake_candidate
from .execution import action_prefix
from .coordinator import CycleResult, NavigationCoordinator
from .local_worker import LocalWorker, close_pool
from .global_worker import GlobalWorker
from .trajectory_buffer import TrajectoryBuffer
from .reference_buffer import ReferenceTrajectoryBuffer


class AsyncNavigationCoordinator(NavigationCoordinator):
    def __init__(self, *, local_worker=None, global_runner=None, build_dir=None,
                 planning_period_s=.1, execution_period_s=.05, feedback_tolerance=None,
                 planner_options=None, **kwargs):
        if not all(np.isfinite(x) and x>0 for x in (planning_period_s,execution_period_s)):
            raise ValueError('periods must be finite and positive')
        super().__init__(None,runner=global_runner,local_period_s=execution_period_s,**kwargs)
        self.local_worker=local_worker if local_worker is not None else LocalWorker(self.limits,build_dir,planner_options=planner_options,field_backend=kwargs.get("field_backend"))
        self.planning_period_s=planning_period_s
        self.execution_period_s=execution_period_s
        self.buffer=(ReferenceTrajectoryBuffer(feedback_tolerance,reaction_s=execution_period_s)
                     if self.revalidate_current_map else TrajectoryBuffer(feedback_tolerance))
        self._next_plan=-float('inf')
        self._sequence=0
        self._inflight=None
        self._buffer_identity=None
        self._last_tick=None

    def invalidate_goal(self, *, generation):
        super().invalidate_goal(generation=generation)
        self.buffer.clear()
        self._inflight = self._buffer_identity = None
        self._last_tick = None
        self._next_plan = -float('inf')

    def replace_goal(self, goal, *, goal_yaw=None, generation):
        revision = self.goal_revision
        super().replace_goal(goal, goal_yaw=goal_yaw, generation=generation)
        if self.goal_revision != revision:
            self.buffer.clear()

    def _buffer_minimum_s(self, state):
        if not self.revalidate_current_map:
            return self.execution_period_s
        # 续用必须剩下一个反应周期及按真实反馈受限制动的时间。
        return self.execution_period_s + max(abs(state.v) / self.limits.linear_accel_mps2,
                                              abs(state.w) / self.limits.angular_accel_rps2)

    def close(self, timeout_s=2.):
        local_result=global_result=None
        try:
            local_result=(self.local_worker.close(timeout_s=timeout_s)
                          if isinstance(self.local_worker,LocalWorker) else self.local_worker.close())
        finally:
            if isinstance(self.runner,GlobalWorker):
                global_result=close_pool(self.runner._pool,timeout_s)
            elif hasattr(self.runner,'close'):
                global_result=self.runner.close()
        return {'local':local_result,'global':global_result}

    def _async_plan(self, state, grid, reference, now_s, events, timings, context):
        accepted_candidate = accepted_certificate = None
        identity=(grid.epoch,reference.route_version,self.goal_revision,self._goal,self._goal_yaw,self.allow_unknown)
        if self._buffer_identity is not None and self._buffer_identity[1:] != identity[1:]:
            self.buffer.clear()
        self._buffer_identity=identity
        try:
            result=self.local_worker.poll()
            if result is not None:
                expected,self._inflight=self._inflight,None
                incoming=result.get('request_identity')
                events.append({'kind':'LOCAL_RESULT','reason':result.get('reason'),'attempts':result.get('attempts',()),
                               'request_identity':incoming,'received_s':now_s,'received_wall_s':perf_counter(),
                               'worker_start_wall_s':result.get('worker_start_wall_s'),
                               'worker_finish_wall_s':result.get('worker_finish_wall_s'),
                               'worker_cpu_ms':result.get('worker_cpu_ms'),'field_reused':result.get('field_reused')})
                request_identity = None if incoming is None else (
                    incoming.get('epoch'), incoming.get('route_version'), incoming.get('goal_revision'),
                    tuple(incoming.get('goal', ())), incoming.get('goal_yaw'), incoming.get('allow_unknown'))
                compatible = (request_identity is not None and
                              (request_identity[1:] == identity[1:] if self.revalidate_current_map else request_identity == identity))
                if incoming != expected or not compatible:
                    events.append({'kind':'STALE_LOCAL_RESULT'})
                elif result.get('candidate') is not None:
                    candidate=result['candidate']
                    if candidate.map_epoch != incoming['epoch'] or candidate.route_version != reference.route_version:
                        raise ValueError('LOCAL_CANDIDATE_IDENTITY')
                    original_digest, original_epoch = candidate.digest(), candidate.map_epoch
                    age=now_s-incoming['feedback_s']
                    minimum_s = self._buffer_minimum_s(state)
                    if self.revalidate_current_map:
                        proposal=self.buffer.prepare_source(candidate,state,minimum_s,self.limits,identity=incoming)
                        candidate=proposal.candidate
                        # 派生当前图轨迹；原请求、原epoch和原candidate哈希都保持可审计。
                        candidate=replace(candidate,map_epoch=grid.epoch)
                        events.append({'kind':'LOCAL_CANDIDATE_REALIGNED','request_identity':dict(incoming),
                                       'computation_age_s':age,'alignment':dict(proposal.alignment),
                                       'derived_candidate_digest':candidate.digest()})
                    else:
                        if candidate.times[-1] - age < minimum_s - 1e-9:
                            raise ValueError('BUFFER_EXHAUSTED')
                        candidate=self.buffer.rebase(candidate,state,age,minimum_s)
                    certificate=certify(candidate,grid,self.limits,reference.route_version,backend=self.backend,context=context)
                    if not certificate.matches(candidate,grid,self.limits,reference.route_version):
                        raise ValueError('LOCAL_CERTIFICATION_'+certificate.reason.value)
                    if self.revalidate_current_map:
                        self.buffer.install_source(proposal,candidate,now_s)
                    else:
                        self.buffer.install(candidate,now_s)
                    accepted_candidate,accepted_certificate=candidate,certificate
                    if self.revalidate_current_map and original_epoch != grid.epoch:
                        events.append({'kind':'LOCAL_CURRENT_MAP_REVALIDATED','request_identity':dict(incoming),
                                       'requested_epoch':original_epoch,'original_candidate_digest':original_digest,
                                       'current_epoch':grid.epoch,'current_grid_digest':grid.digest(),
                                       'derived_candidate_digest':candidate.digest()})
                    events.append({'kind':'LOCAL_ACCEPTED','sequence':incoming['sequence'],'feedback_s':incoming['feedback_s'],
                                   'received_s':now_s,'accepted_s':now_s,'age_s':age,'feedback_error':self.buffer.last_error,
                                   'worker_start_wall_s':result.get('worker_start_wall_s'),'worker_finish_wall_s':result.get('worker_finish_wall_s'),
                                   'worker_ready_wall_s':result.get('worker_ready_wall_s'),'cold_build_ms':result.get('cold_build_ms')})
                    timings.update({'planner_'+k:v for k,v in result.get('timings_ms',{}).items()})
                else:
                    events.append({'kind':'LOCAL_FAILED','reason':result.get('reason')})
        except Exception as exc:
            if not self.local_worker.busy: self._inflight=None
            events.append({'kind':'LOCAL_REJECTED','reason':str(exc)})
        if (not hasattr(self.local_worker,'poll_ready') or self.local_worker.ready) and not self.local_worker.busy and now_s+1e-9 >= self._next_plan:
            self._sequence+=1
            self._inflight=dict(sequence=self._sequence,epoch=grid.epoch,grid_digest=grid.digest(),route_version=reference.route_version,
                                goal=self._goal,goal_yaw=self._goal_yaw,goal_revision=self.goal_revision,
                                allow_unknown=self.allow_unknown,feedback_s=now_s,submitted_s=now_s,
                                submitted_wall_s=perf_counter())
            try:
                self.local_worker.submit(dict(identity=self._inflight,state=state,grid=grid,reference=reference))
                events.append({'kind':'LOCAL_SUBMITTED',**self._inflight})
            except Exception as exc:
                self._inflight=None
                events.append({'kind':'LOCAL_SUBMIT_ERROR','reason':str(exc)})
            self._next_plan=now_s+self.planning_period_s
        if accepted_candidate is not None:
            events.append({'kind':'BUFFER_EXECUTING','remaining_s':self.buffer.remaining_s(now_s),'local_busy':self.local_worker.busy})
            return accepted_candidate,accepted_certificate,'BUFFER_EXECUTING'
        try:
            minimum_s = self._buffer_minimum_s(state)
            candidate=(self.buffer.current(state,now_s,minimum_s,limits=self.limits)
                       if self.revalidate_current_map else self.buffer.current(state,now_s,minimum_s))
            # 旧控制可跨地图更新保留，但必须绑定当前地图重新认证整段。
            candidate=replace(candidate,map_epoch=grid.epoch)
            if self.revalidate_current_map:
                events.append({'kind':'BUFFER_CURRENT_STATE_REALIGNED','elapsed_s':now_s-self.buffer.accepted_s,
                               'alignment':dict(self.buffer.last_alignment),
                               'derived_candidate_digest':candidate.digest()})
            cert=certify(candidate,grid,self.limits,reference.route_version,backend=self.backend,context=context)
            if not cert.matches(candidate,grid,self.limits,reference.route_version):
                raise ValueError('BUFFER_CERTIFICATION_'+cert.reason.value)
            # 以本次反馈固定下一次预计接续，避免累积小误差并确保反馈从未被传送。
            if self.revalidate_current_map:
                self.buffer.update_derived(candidate,now_s)
            else:
                self.buffer.install(candidate,now_s)
            events.append({'kind':'BUFFER_EXECUTING','remaining_s':self.buffer.remaining_s(now_s),'local_busy':self.local_worker.busy})
            return candidate,cert,'BUFFER_EXECUTING'
        except ValueError as exc:
            self.buffer.clear()
            events.append({'kind':'BRAKE_REQUIRED','reason':str(exc)})
            return None,None,'BUFFER_UNAVAILABLE'

    def tick(self, world, state, goal, now_s, *, goal_yaw=None):
        started = perf_counter()
        cpu_started = process_time()
        if not np.isfinite(now_s) or (self._last_tick is not None and now_s < self._last_tick-1e-9):
            raise ValueError('execution clock must be finite and monotonic')
        self._last_tick=now_s
        revision = self.goal_revision
        self._set_goal(goal, goal_yaw)
        if self.goal_revision != revision or self._policy != self.allow_unknown:
            self.buffer.clear()
        if now_s + 1e-9 < self._next_local:
            return CycleResult('NOT_DUE', timings_ms={'total': (perf_counter()-started)*1000})
        # 以首次执行时刻锚定周期相位；迟到只跳过已错过的槽，不补发历史动作。
        due_s = now_s if not np.isfinite(self._next_local) else self._next_local
        skipped = max(0, int(np.floor((now_s-due_s+1e-9)/self.local_period_s)))
        scheduled_s = due_s+skipped*self.local_period_s
        self._next_local = scheduled_s+self.local_period_s
        timings, events = {}, [{'kind':'EXECUTION_SCHEDULE','scheduled_s':scheduled_s,
                               'next_due_s':self._next_local,'skipped_periods':skipped,
                               'lateness_s':max(0.,now_s-scheduled_s)}]
        if hasattr(self.local_worker,'poll_ready'):
            try:
                ready=self.local_worker.poll_ready()
                if ready is not None: events.append({'kind':'LOCAL_WORKER_READY',**ready})
            except Exception as exc:
                events.append({'kind':'LOCAL_WORKER_START_ERROR','reason':str(exc)})
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
        # 末端单点参考绑定当前目标朝向，不再受旧路线进度或出窗位置影响。
        if self.route is not None and not arrived_xy:
            try:
                reference, self.progress = extract_reference(self.route, state, grid, self.progress, terminal_yaw=self._goal_yaw)
            except ValueError as exc:
                events.append({'kind': 'REFERENCE_REJECTED', 'reason': str(exc)})
                self._pending_reason = 'REFERENCE_REENTRY'
        timings['reference'] = (perf_counter()-stage)*1000
        stage = perf_counter()
        context = prepare_certification(grid, backend=self.backend)
        timings['certification_prepare'] = (perf_counter()-stage)*1000
        candidate = certificate = None
        if not goal_check.accepted:
            self.buffer.clear()
            status = goal_check.reason_code
            events.append({'kind': 'GOAL_REJECTED', 'reason': status})
        elif arrived_xy:
            self.buffer.clear()
            candidate, status = self._terminal_plan(state, grid, reference)
            if candidate is not None:
                certificate = certify(candidate, grid, self.limits, version, backend=self.backend, context=context)
        elif len(reference.points) >= 2:
            stage = perf_counter()
            candidate,certificate,status=self._async_plan(state,grid,reference,now_s,events,timings,context)
            timings['local_poll_schedule_buffer']=(perf_counter()-stage)*1000
        else:
            self.buffer.clear()
        stage = perf_counter()
        if candidate is not None:
            if not certificate.matches(candidate, grid, self.limits, version) or not np.allclose(candidate.states[0,:5], motion, atol=1e-6, rtol=0):
                events.append({'kind': 'CANDIDATE_REJECTED', 'reason': certificate.reason.value if not certificate.valid else 'INITIAL_STATE_MISMATCH'})
                candidate = certificate = None
        timings['certification_identity'] = (perf_counter()-stage)*1000
        action = action_certificate = None
        if candidate is not None:
            stage = perf_counter()
            try:
                action = action_prefix(candidate, state, self.execution_period_s)
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
                action = action_prefix(candidate, state, self.execution_period_s)
                action_certificate = certify(action, grid, self.limits, version, backend=self.backend, context=context)
                if not action_certificate.matches(action, grid, self.limits, version):
                    events.append({'kind': 'BRAKE_ACTION_REJECTED', 'reason': action_certificate.reason.value})
                    candidate = action = None
                    status = 'NO_CERTIFIED_ACTION'
            timings['brake'] = (perf_counter()-stage)*1000
        events.append({'kind':'EXECUTION_TICK','now_s':now_s,'feedback_s':now_s,'buffer_remaining_s':self.buffer.remaining_s(now_s),'local_busy':self.local_worker.busy})
        timings['total_cpu'] = (process_time()-cpu_started)*1000
        timings['total'] = (perf_counter()-started)*1000
        deadline_miss = timings['total'] > self.local_period_s*1000
        if deadline_miss: events.append({'kind': 'LOCAL_DEADLINE_MISS', 'total_ms': timings['total']})
        return CycleResult(status, candidate, certificate, grid, reference, tuple(events), timings, deadline_miss, global_result, action, action_certificate)
