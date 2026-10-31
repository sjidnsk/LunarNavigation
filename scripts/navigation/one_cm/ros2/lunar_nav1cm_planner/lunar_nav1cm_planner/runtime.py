"""不依赖 rclpy 的线程调度；回调不运行地图、目标碰撞检查或求解。

地图线程独占 P3MapStore，tick 线程独占 session/coordinator。只在交接最新
输入、不可变地图及发布 guard 时短暂加锁；不会持锁执行 native 地图计算。
"""
from collections import deque
import json
import math
import threading
import time

from nav1cm.navigation_task import NavigationTaskCoordinator, TaskSnapshot
from nav1cm.ros_conversion import (
    LatestTransforms, MissingTransform, goal_from_message, map_from_message,
    paths, state_from_odometry,
)


class PlannerRuntime:
    """factory/消息回调注入只用于内部验证，不构成对外延迟或控制协议。"""
    def __init__(self, session, map_store, *, publish, ros_now_ns, notice=None, map_frame='map', event_sink=None):
        self.session, self.map_store = session, map_store
        self.publish, self.ros_now_ns = publish, ros_now_ns
        self.notice = notice or (lambda *args: None)
        self.map_frame = map_frame
        self.event_sink = event_sink or (lambda event: None)
        # task事件先由唯一tick线程入内部队列；外部sink不得在输入锁内重入。
        self._task_events = deque()
        self.tasks = NavigationTaskCoordinator(session, event_sink=self._task_events.append)
        self._odometry_sequence = self._observed_odometry_sequence = 0
        self._condition = threading.Condition()
        self._transforms = LatestTransforms()
        self._tf_sequence = self._map_sequence = self._goal_sequence = 0
        self._map_message = self._odometry = None
        self._world = session.world
        self._goals = deque()
        self._last_accepted_input = 0
        self._stopping = self._closed = False
        self._threads = []
        self._notices = {}
        self._reported_grid_geometry = None
        self.execution_contract_enabled=False
        self.execution_context=None
        self._odometry_received_s=None
        self._pose_advanced_received_s=None
        self._pose_timeout_s=None
        self._pose_source_high_water=self._pose_clock_high_water=None
        self._pose_integrity_fault=None
        self._pose_waiting=False
        self._tracking_inputs=deque(maxlen=128)

    def configure_pose_input(self,timeout_s):
        if isinstance(timeout_s,bool) or not math.isfinite(timeout_s) or timeout_s<=0:
            raise ValueError('odometry_timeout_s must be finite and positive')
        self._pose_timeout_s=float(timeout_s)
        self._pose_waiting=True

    def _pose_source_current(self,message,received_s,now_s):
        if self._pose_timeout_s is None:return True,None
        try:
            stamp=message.header.stamp
            source_ns=int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
            clock=self.ros_now_ns()
            advanced_s=(received_s if self._pose_source_high_water is None or
                        source_ns>self._pose_source_high_water else self._pose_advanced_received_s)
            valid=(source_ns>0 and received_s is not None and
                   (self._pose_clock_high_water is None or clock>=self._pose_clock_high_water) and
                   (self._pose_source_high_water is None or source_ns>=self._pose_source_high_water) and
                   0<=now_s-received_s<=self._pose_timeout_s and
                   source_ns<=clock and advanced_s is not None and
                   0<=now_s-advanced_s<=self._pose_timeout_s)
            return valid,source_ns
        except (ValueError,TypeError,AttributeError,OverflowError):return False,None

    def _enter_pose_wait(self):
        self._pose_waiting=True
        self.session.offer_state(None)

    def _apply_pose_input(self,message,received_s,now_s,transforms):
        valid,stamp=self._pose_source_current(message,received_s,now_s)
        if self._pose_timeout_s is not None:
            clock=self.ros_now_ns()
            fault=('POSE_CLOCK_ROLLBACK' if self._pose_clock_high_water is not None and clock<self._pose_clock_high_water else
                   'POSE_SOURCE_ROLLBACK' if stamp is not None and self._pose_source_high_water is not None and stamp<self._pose_source_high_water else
                   'POSE_FUTURE_FEEDBACK' if stamp is not None and stamp>clock else None)
            self._pose_clock_high_water=max(clock,self._pose_clock_high_water or clock)
            if fault:
                self._pose_integrity_fault=self._pose_integrity_fault or fault
                valid=False
            if valid and (self._pose_source_high_water is None or stamp>self._pose_source_high_water):
                self._pose_source_high_water=stamp
                self._pose_advanced_received_s=received_s
        if not valid:
            self._enter_pose_wait()
            self._notice('ODOMETRY_WAITING','missing_or_nonadvancing_pose')
            return
        if message is None:return
        try:value=state_from_odometry(message,transforms,self.map_frame)
        except MissingTransform:
            if self._pose_timeout_s is not None:self._enter_pose_wait()
            else:self.session.offer_state(None)
            raise
        except (ValueError,AttributeError,TypeError):
            if self._pose_timeout_s is not None:
                self._pose_integrity_fault=self._pose_integrity_fault or 'INVALID_POSE_FEEDBACK'
                self._enter_pose_wait()
            else:self.session.offer_state(None)
            raise
        if self._pose_timeout_s is not None and self._pose_waiting:
            self._pose_waiting=False
            self._notice('ODOMETRY_RECOVERED','valid_pose_replanning')
        self.session.offer_state(value)

    def enable_execution_contract(self,timeout_s=2.):
        self.execution_contract_enabled=True
        self.tasks.configure_execution_feedback(required=True,timeout_s=timeout_s)

    def offer_tracking_status(self,status):
        with self._condition:
            self._tracking_inputs.append((status,time.monotonic()))

    def register_execution_reference(self,reference):
        # 仅tick线程发布回调调用；登记必须先于实际发送元信息。
        self.tasks.register_execution_reference(reference,time.monotonic())


    def _notice(self, kind, detail):
        detail, now = str(detail), time.monotonic()
        previous = self._notices.get(kind)
        if previous is None or detail != previous[0] or now - previous[1] >= 5.:
            # 每种事件只留一条节流状态；地图epoch/目标变化不会无限堆积键。
            self._notices[kind] = (detail, now)
            self.notice(kind, detail)

    def offer_map(self, message):
        with self._condition:
            self._map_sequence += 1
            self._map_message = message
            self._condition.notify_all()

    def offer_odometry(self, message):
        with self._condition:
            self._odometry = message
            self._odometry_received_s=time.monotonic()
            self._odometry_sequence += 1
            self._condition.notify_all()

    def offer_goal(self, message):
        self._enqueue('manual', None, message)

    def offer_managed_goal(self, task_id, message):
        self._enqueue('managed', task_id, message)

    def request_cancel(self, task_id):
        self._enqueue('cancel', task_id, None)

    def _enqueue(self, kind, task_id, message):
        with self._condition:
            if self._stopping:
                return
            self._goal_sequence += 1
            # 保留有效A→无效B及取消的先后顺序；输入版本同时阻挡正在完成的旧发布。
            self._goals.append((self._goal_sequence, kind, task_id, message))
            self._condition.notify_all()

    def offer_tf(self, message):
        with self._condition:
            self._transforms.update(message)
            self._tf_sequence += 1
            self._condition.notify_all()

    def start(self):
        if self._threads:
            raise RuntimeError('runtime already started')
        self._threads = [threading.Thread(target=self._map_loop, name='nav1cm-map', daemon=True),
                         threading.Thread(target=self._tick_loop, name='nav1cm-tick', daemon=True)]
        for thread in self._threads:
            thread.start()

    def _map_loop(self):
        processed, attempted = 0, None
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopping or
                    (self._map_sequence != processed and
                     (self._map_sequence, self._tf_sequence) != attempted))
                if self._stopping:
                    return
                message, sequence = self._map_message, self._map_sequence
                transforms = self._transforms.copy()
                attempted = (sequence, self._tf_sequence)
            try:
                transform = map_from_message(message, transforms, self.map_frame)
                snapshot = self.map_store.apply(message, transform)
            except MissingTransform as exc:
                self._notice('MAP_WAITING_TF', exc)
                continue
            except Exception as exc:
                processed = sequence
                self._notice('MAP_REJECTED', f'{type(exc).__name__}: {exc}')
                continue
            with self._condition:
                processed = sequence
                self._world = snapshot
                self._condition.notify_all()
            self._notice('MAP_APPLIED', json.dumps({'epoch': snapshot.epoch,
                         'source_resolution_m': getattr(self.map_store, 'source_resolution_m', None)}, sort_keys=True))

    def _deliver_task_events(self):
        # 事件已提交；外部回调引入的新输入严格排在该提交之后。
        while self._task_events:
            snapshot=self._task_events.popleft()
            if snapshot.phase=='ERROR_STOPPING' or snapshot.outcome is not None:
                self._notice('TASK_STATE',json.dumps({key:getattr(snapshot,key) for key in
                    ('task_id','generation','phase','outcome','reason_code')},sort_keys=True))
            self.event_sink(snapshot)

    def _tick_once(self, now_s):
        with self._condition:
            world, odometry = self._world, self._odometry
            pose_received_s=self._odometry_received_s
            input_sequence, odometry_sequence = self._goal_sequence, self._odometry_sequence
            transforms = self._transforms.copy()
            tf_sequence = self._tf_sequence
            goals, self._goals = self._goals, deque()
        if world is not None:
            self.session.set_world(world)
        if odometry is not None or self._pose_timeout_s is not None:
            try:self._apply_pose_input(odometry,pose_received_s,now_s,transforms)
            except (ValueError,AttributeError,TypeError) as exc:
                self._notice('ODOMETRY_WAITING',exc)
        unresolved = []
        for sequence, kind, task_id, message in goals:
            if kind == 'cancel':
                pending = [item for item in unresolved if item[2] == task_id]
                unresolved = [item for item in unresolved if item[2] != task_id]
                if not self.tasks.cancel(task_id, now_s) and pending:
                    self._task_events.append(TaskSnapshot(task_id, 0, 'FINISHED', 'CANCELED', 'CANCELED_BEFORE_ACCEPTANCE'))
                continue
            try:
                goal = ((message.target_x_m, message.target_y_m,
                         message.target_yaw_rad if message.has_target_yaw else None) if kind == 'managed' else
                        goal_from_message(message, transforms, self.map_frame))
            except MissingTransform as exc:
                unresolved.append((sequence, kind, task_id, message))
                self._notice('GOAL_WAITING_TF', exc)
                continue
            except (ValueError, AttributeError, TypeError) as exc:
                if task_id is not None:
                    self.tasks.reject(task_id, 'INVALID_GOAL: ' + str(exc))
                self._notice('GOAL_REJECTED', exc)
                continue
            event = self.tasks.submit(task_id, goal, now_s)
            if event is not None and event.reason_code == 'WAITING_FOR_POSE':
                unresolved.append((sequence, kind, task_id, message))
            elif event is not None and event.outcome == 'INVALID_GOAL':
                self._notice('GOAL_REJECTED', event.reason_code)
            else:
                self._last_accepted_input = sequence
                if task_id is not None:
                    # 唯一owner在逻辑接受后立即捕获原语值；外部notice在输入锁外交付。
                    adapted=self.session.goal
                    accepted=json.dumps({'task_id': task_id, 'accepted_generation': self.session.accepted_generation,
                                         'goal': [adapted.x, adapted.y, adapted.yaw]},sort_keys=True)
                    self._notice('TASK_ACCEPTED',accepted)
                self._notice('GOAL_ACCEPTED', f'generation={self.session.accepted_generation}, pose={goal}')
        remaining = []
        for item in unresolved:
            if item[0] > self._last_accepted_input:
                remaining.append(item)
            elif item[2] is not None:
                self._task_events.append(TaskSnapshot(item[2], 0, 'FINISHED', 'CANCELED', 'GOAL_REPLACED'))
        with self._condition:
            self._goals.extendleft(reversed(remaining))
        self._deliver_task_events()
        if self._pose_integrity_fault is not None:
            self.tasks.fail_tracking(self._pose_integrity_fault,now_s)
        if self.execution_contract_enabled:
            with self._condition:
                tracking_inputs,self._tracking_inputs=self._tracking_inputs,deque(maxlen=128)
            for status,received_s in tracking_inputs:
                self.tasks.offer_execution_status(status,received_s)
            # 定位有效性由自身输入流判定；迟到的控制器等待不得覆盖恢复后的新定位。
            fault=self.tasks.check_execution_fault(now_s)
            if fault:self.tasks.fail_tracking(fault,now_s)
        packet = self.session.tick(now_s)
        proposal = self.tasks.observe(packet, now_s,
                                      state_updated=odometry_sequence != self._observed_odometry_sequence,
                                      defer_commit=True)
        odometry_error = None
        with self._condition:
            # 目标、最新测量和内部事件提交共享线性化点；不执行规划或外部回调。
            if not self._stopping and input_sequence == self._goal_sequence:
                if self._pose_timeout_s is not None:
                    try:self._apply_pose_input(self._odometry,self._odometry_received_s,time.monotonic(),self._transforms)
                    except (ValueError,AttributeError,TypeError) as exc:odometry_error=exc
                    if self.session.state is None:
                        packet=self.session.tick(now_s)
                        proposal=self.tasks.observe(packet,now_s,state_updated=False,defer_commit=True)
                if proposal is not None and (odometry_sequence != self._odometry_sequence or
                                             tf_sequence != self._tf_sequence):
                    # 仅重做轻量位姿转换/纯观测，不因持续等价odom/TF而饿死反馈或终态。
                    # 越界/运动的新测量也须提交非终态提案，以清除旧的到达保持计时。
                    if self._odometry is not None:
                        try:
                            self._apply_pose_input(self._odometry,self._odometry_received_s,
                                                   time.monotonic(),self._transforms)
                        except (ValueError, AttributeError, TypeError) as exc:
                            self.session.offer_state(None)
                            odometry_error = exc
                    odometry_sequence = self._odometry_sequence
                    proposal = self.tasks.observe(packet, now_s,
                        state_updated=odometry_sequence != self._observed_odometry_sequence,
                        defer_commit=True)
                self.tasks.commit_observation(proposal)
        self._observed_odometry_sequence = odometry_sequence
        self._deliver_task_events()
        if odometry_error is not None:
            self._notice('ODOMETRY_WAITING', odometry_error)
        if packet.result is not None:
            result = packet.result
            grid = result.grid
            if grid is not None:
                geometry = (grid.resolution_m, tuple(grid.cells.shape))
                if geometry != self._reported_grid_geometry:
                    self._notice('LOCAL_GRID', json.dumps({'epoch': grid.epoch,
                                 'resolution_m': grid.resolution_m, 'shape': list(grid.cells.shape)}, sort_keys=True))
                    self._reported_grid_geometry = geometry
            if result.global_result is not None:
                raw = result.global_result
                record = {key: raw.get(key) for key in ('planning_outcome', 'reason_code', 'has_reference',
                          'requested_epoch', 'requested_goal', 'requested_version')}
                origin = next((event for event in result.events if event['kind'] == 'GLOBAL_RESULT_RECEIVED'), {})
                record.update(accepted_generation=origin.get('accepted_generation_at_submit'),
                              goal_revision=origin.get('goal_revision_at_submit'),
                              current_accepted_generation=packet.generation)
                self._notice('GLOBAL_RESULT', json.dumps(record, sort_keys=True))
            for event in result.events:
                if event['kind']=='LOCAL_RESULT':
                    self._notice('LOCAL_PLAN_STATUS',json.dumps(dict(reason=event.get('reason'),
                        mode=event.get('candidate_mode'),attempt_reasons=[r.get('reason') for r in event.get('attempts',())]),sort_keys=True))
                if event['kind'] in ('GLOBAL_CURRENT_MAP_REVALIDATED', 'GLOBAL_CURRENT_MAP_REJECTED',
                                      'LOCAL_CURRENT_MAP_REVALIDATED', 'INITIAL_START_COMPLETED', 'GLOBAL_REFERENCE_SHAPED'):
                    self._notice(event['kind'], json.dumps(event, sort_keys=True))
        if packet.waiting:
            self._notice('WAITING_INPUT', ','.join(packet.waiting))
        elif packet.result.status == 'NOT_DUE':
            return packet
        data = paths(packet.raw_route if packet.raw_route is not None else packet.route,
                     packet.candidate,self.ros_now_ns(),self.map_frame,global_reference=packet.route)
        # ROS 时间仅给本次 Path 定 t0；调度/轨迹年龄始终使用单调时钟。
        with self._condition:
            if (self._pose_timeout_s is not None and not self._pose_source_current(
                    self._odometry,self._odometry_received_s,time.monotonic())[0]):
                self._enter_pose_wait()
                packet=self.session.tick(now_s)
                data=paths(None,None,self.ros_now_ns(),self.map_frame)
            if (not self._stopping and input_sequence == self._goal_sequence and
                    packet.generation == self.session.accepted_generation and
                    (packet.waiting or self.session.publication_current(packet))):
                if self.execution_contract_enabled:
                    from nav1cm.execution_contract import FOLLOW,BRAKE,HOLD,FINAL_ALIGN,INVALIDATED
                    active=self.tasks.active
                    status=None if packet.result is None else packet.result.status
                    candidate=packet.candidate
                    intent=(INVALIDATED if candidate is None else HOLD if status=='ARRIVED' else
                            FINAL_ALIGN if status=='GOAL_TURNING' else
                            BRAKE if candidate.source=='brake' or self.session.canceling else FOLLOW)
                    # cancel_goal提升generation，执行身份仍固定在原接受任务上。
                    self.execution_context=(None if active is None else active.task_id,
                        self.session.accepted_generation if active is None else active.generation,intent)
                self.publish(data)
        return packet

    def _tick_loop(self):
        period = self.session.coordinator.execution_period_s
        while True:
            started = time.monotonic()
            with self._condition:
                if self._stopping:
                    return
            try:
                self._tick_once(started)
            except Exception as exc:
                self._notice('TICK_ERROR', f'{type(exc).__name__}: {exc}')
                with self._condition:
                    if not self._stopping:
                        self.publish(paths(None, None, self.ros_now_ns(), self.map_frame))
            # 输入唤醒可提前检查，协调器的单调周期门禁负责不补发历史动作。
            with self._condition:
                if self._stopping:
                    return
                self._condition.wait(timeout=max(.001, period - (time.monotonic() - started)))

    def close(self, timeout_s=2.):
        if self._closed:
            return self._close_result
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        deadline = time.monotonic() + timeout_s
        for thread in self._threads:
            thread.join(max(0., deadline - time.monotonic()))
        alive = [thread.name for thread in self._threads if thread.is_alive()]
        if alive:
            # 尚有线程使用 native/session 时不得销毁它们；调用方可继续有界等候。
            return {'threads_alive': alive}
        result = self.session.close(timeout_s=timeout_s)
        if self.map_store is not None:
            terrain = getattr(self.map_store, '_terrain', None)
            owner = self.map_store if hasattr(self.map_store, 'close') else terrain
            if owner is not None and hasattr(owner, 'close'):
                owner.close()
        self._closed = True
        self._close_result = {'threads_alive': [], 'workers': result}
        return self._close_result
