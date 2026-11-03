"""不依赖 rclpy 的线程调度；回调不运行地图、目标碰撞检查或求解。

地图线程独占 P3MapStore，tick 线程独占 session/coordinator。只在交接最新
输入、不可变地图及发布 guard 时短暂加锁；不会持锁执行 native 地图计算。
直控发布循环持续发送当前交接速度；更新、撤销由现有任务和输入状态决定，不按认证预测时长归零。
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


def brake_diagnostic(packet, cycle_s, buffer):
    """只整理本周期既有事件，不推断制动原因。"""
    import math
    result, candidate = packet.result, packet.candidate
    tick = next ( (e for e in reversed(result.events) if e.get('kind') == 'EXECUTION_TICK'), {} )
    # 事件可能携带轨迹、请求或尝试列表；仅导出已有标量字段以保留原因并限制日志体积。
    def scalar(value):
        return value is None or isinstance(value, (str, bool, int)) or (
            isinstance(value, float) and math.isfinite(value))
    events = [{key: value for key, value in event.items() if scalar(value)}
              for event in result.events
              if event.get('kind') not in ('EXECUTION_TICK', 'EXECUTION_SCHEDULE')]
    def finite(value):
        return value if isinstance(value, (int, float)) and math.isfinite(value) else None
    return dict(cycle_s=cycle_s, accepted_generation=packet.generation, status=result.status,
                candidate_mode=candidate.mode, candidate_source=candidate.source,
                planning_route_version=getattr(result.reference, 'route_version', None),
                candidate_route_version=candidate.route_version,
                buffer_remaining_s=finite(tick.get('buffer_remaining_s')),
                source_phase_s=finite(getattr(buffer, 'source_phase_s', None)),
                local_busy=tick.get('local_busy'), events=events, event_state='present' if events else 'none')


class PlannerRuntime:
    """factory/消息回调注入只用于内部验证，不构成对外延迟或控制协议。"""
    def __init__(self, session, map_store, *, publish, ros_now_ns, notice=None, map_frame='map', event_sink=None, facts_sink=None):
        self.session, self.map_store = session, map_store
        self.publish, self.ros_now_ns = publish, ros_now_ns
        self.notice = notice or (lambda *args: None)
        self.map_frame = map_frame
        self.facts_sink = facts_sink
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
        self.publish_command = None
        self._last_issued_command=(0.,0.)
        self._last_issued_s=None
        # 独立于输入锁，避免计算线程的最终观测提交阻塞20Hz实际发令。
        self._command_lock = threading.RLock()
        self._command_wakeup = threading.Event()
        self._pending_command = self._published_selection = None
        self._command_threaded = False
        self._last_command_selection_s = None
        self._command_publish_error = None
        self._observed_pose_stamp_ns = None
        self._applied_pose_stamp_ns = None

    def configure_direct_control(self, publish_command, *, command_hold_s, reference_interval_s=.2, command_spin_prepare_s=0.):
        if isinstance(command_hold_s,bool) or not math.isfinite(command_hold_s) or command_hold_s<=0:
            raise ValueError("command_hold_s must be explicitly finite and positive")
        if isinstance(reference_interval_s,bool) or not math.isfinite(reference_interval_s) or reference_interval_s<=0:
            raise ValueError("reference_interval_s must be finite and positive")
        # 保留旧启动关键字兼容；P3实测模式不再建命令外推器。
        self.reference_interval_s=float(reference_interval_s)
        self.command_hold_s=float(command_hold_s)
        self.publish_command = publish_command
        self.session.direct_control = True
        self.execution_contract_enabled = False
        self.tasks.configure_execution_feedback(required=False)

    def configure_command_prediction(self, *, spin_prepare_s=0.):
        """只预测最终发布速度的运动；P3实测和完成证据保留原归属。"""
        if self.publish_command is None:
            raise ValueError('command prediction requires direct command control')
        from .command_prediction import CommandPrediction
        self._command_prediction=CommandPrediction(
            spin_prepare_s=spin_prepare_s,use_published_velocity=True)
        self._published_command_inputs=deque(maxlen=512)

    def offer_published_command(self, message, *, received_s=None):
        if getattr(self,'_command_prediction',None) is None:return
        command=(float(message.linear.x),float(message.angular.z))
        stamp=time.monotonic() if received_s is None else float(received_s)
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in command):
            raise ValueError('published velocity must be finite')
        with self._condition:
            self._published_command_inputs.append((stamp,command))

    def _prediction_snapshot(self, now_s):
        measured=self.session.state
        predictor=getattr(self,'_command_prediction',None)
        if predictor is None:return measured,None
        with self._condition:
            commands=tuple(self._published_command_inputs)
            self._published_command_inputs.clear()
        for stamp,command in commands:predictor.issue(stamp,command)
        stamp=self._applied_pose_stamp_ns
        if measured is None or stamp is None:return measured,dict(used_prediction=False)
        predictor.correct(measured,stamp,now_s,self.ros_now_ns())
        covered=predictor.covered(now_s)
        estimated=predictor.predict(now_s,self.session.coordinator.limits)
        fields=lambda value:[value.x,value.y,value.yaw,value.v,value.w]
        return estimated,dict(used_prediction=covered,source_stamp_ns=stamp,
            measured_state=fields(measured),estimated_state=fields(estimated),
            proof_basis='published_velocity_model' if covered else 'p3_measured_pose')

    def _publish_issued(self, command, *, selected=True):
        with self._command_lock:
            self._command_previous=self._last_issued_command
            self.publish_command(*command)
            self._last_issued_command=tuple(command)
            self._last_issued_s=time.monotonic()
            # 重发不等于重新起步；slew按上次新指令实际交接时刻计算。
            if selected:self._last_command_selection_s=self._last_issued_s

    def _invalidate_command(self):
        with self._command_lock:
            self._pending_command=None
        self._command_wakeup.set()

    def _publish_command_once(self):
        # 这里只读交接物和现有输入判活；不得调用session、地图或认证。
        with self._command_lock:
            selected=self._pending_command
            command=(0.,0.) if selected is None else selected[0]
            now=time.monotonic()
            if (self._stopping or selected is not None and
                    selected[1]!=self._goal_sequence or
                    self._pose_integrity_fault is not None or
                    self._pose_timeout_s is not None and not self._pose_source_current(
                        self._odometry,self._odometry_received_s,now)[0]):
                command=(0.,0.)
                self._pending_command=selected=None
            changed=(selected is not self._published_selection or command!=self._last_issued_command)
            self._publish_issued(command,selected=changed)
            self._published_selection=selected

    def _command_loop(self):
        period=self.session.coordinator.execution_period_s
        due=time.monotonic()
        while True:
            self._command_wakeup.wait(max(0.,due-time.monotonic()))
            self._command_wakeup.clear()
            with self._command_lock:
                if self._stopping:return
            try:
                self._publish_command_once()
            except Exception as exc:
                with self._command_lock:
                    self._pending_command=None
                    self._command_publish_error=f'{type(exc).__name__}: {exc}'
            # 不追赶历史周期；慢日志交由计算线程处理。
            due=time.monotonic()+period

    def _command(self, packet, command, source_stamp=None, diagnostics=None, *,
                 input_sequence=None, previous_command=None):
        if self.publish_command is None:
            return
        with self._command_lock:
            previous=self._last_issued_command
            # 若真实输入失效等原因已改变实际指令，按新的已发速度重算slew与证明。
            # 这里只拒绝交接，不清空仍有效的当前命令，也不额外插零。
            if command!=(0.,0.) and previous_command is not None and previous!=previous_command:
                return False
            self._pending_command=(tuple(command),
                self._goal_sequence if input_sequence is None else input_sequence)
            if not self._command_threaded:
                # 供离线单步验证复用同一发布入口；正式start后只有发布循环发令。
                self._publish_command_once()
            actual=self._last_issued_command
        try:
            diagnostics=dict(diagnostics or {})
            diagnostics['actual_published_vw']=list(actual)
            diagnostics['publication_state']='queued' if self._command_threaded else 'published'
            diagnostics.setdefault('final_publish_override_reason',None)
            self._notice('MPCC_COMMAND', json.dumps(dict(v=command[0], w=command[1],
                status=getattr(packet.result, 'status', None),
                source=getattr(packet.candidate, 'source', None),
                mode=getattr(packet.candidate, 'mode', None), command_hold_s=self.command_hold_s,
                reference_dt_s=self.reference_interval_s, previous_issued=previous,
                source_stamp_ns=source_stamp,
                source_age_s=None if source_stamp is None else
                    (self.ros_now_ns()-source_stamp)/1e9, **(diagnostics or {}))))
        except Exception:
            pass  # 诊断失败不能触发再次发布或改变控制流程。
        return True

    def _body_observation_time(self, now_s):
        if self.publish_command is None:return now_s
        try:
            stamp=self._odometry.header.stamp
            return int(stamp.sec)+int(stamp.nanosec)/1e9
        except (AttributeError,ValueError,TypeError):return now_s

    def _new_body_observation(self, sequence):
        try:
            stamp=self._odometry.header.stamp
            source=int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
        except (AttributeError,ValueError,TypeError):
            return self.publish_command is None and sequence != self._observed_odometry_sequence
        return source>0 and (self._observed_pose_stamp_ns is None or source>self._observed_pose_stamp_ns)

    def _observe_body_stamp(self):
        try:
            stamp=self._odometry.header.stamp
            source=int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
            if source>0:self._observed_pose_stamp_ns=max(source,self._observed_pose_stamp_ns or source)
        except (AttributeError,ValueError,TypeError):pass

    def _publication_map_proof(self, packet, world):
        """最多一次当前物理窗口重认证；不追赶持续变化的地图。"""
        if self.publish_command is None or packet.result is None or packet.result.grid is None:
            return packet, True
        # direct 使用 tick 入口同一 world/P3 快照；实际 issued proof 由发布消费端认证。
        return packet, world is not None

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
        self._invalidate_command()
        self._pose_waiting=True
        self.session.offer_state(None)

    def _apply_pose_input(self,message,received_s,now_s,transforms):
        try:
            source=message.header.stamp
            source_ns=int(source.sec)*1_000_000_000+int(source.nanosec)
        except (AttributeError,ValueError,TypeError):source_ns=None
        # 迟到旧包不退休当前有效状态，也不变成时钟完整性故障。
        accepted=self._applied_pose_stamp_ns
        if source_ns is not None and accepted is not None and source_ns<accepted:return
        with self._command_lock:
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
        # 判活仍逐周期执行；同戳不再覆盖已应用的P3状态。
        if source_ns is not None and source_ns==accepted and self.session.state is not None:return
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
        if source_ns is not None:
            self._applied_pose_stamp_ns=source_ns

    def enable_execution_contract(self,timeout_s=2.):
        if self.publish_command is not None:
            return
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
            if self._odometry is not None:
                try:
                    old=self._odometry.header.stamp
                    new=message.header.stamp
                    # 同戳修正与迟到包均不替换已接收观测，也不刷新接收计时。
                    if (int(new.sec)*1_000_000_000+int(new.nanosec) <=
                            int(old.sec)*1_000_000_000+int(old.nanosec)):
                        return
                except (AttributeError,ValueError,TypeError):
                    pass
            with self._command_lock:
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
            with self._command_lock:
                self._goal_sequence += 1
                self._pending_command=None
            self._command_wakeup.set()
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
        if self.publish_command is not None:
            self._command_threaded=True
            self._threads.append(threading.Thread(target=self._command_loop,
                                name='nav1cm-command',daemon=True))
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
            # 已提交输入先交接导航；事实传输失败不得回滚world或重试apply。
            # map线程是store唯一写入者，捕获与本次apply之间不存在另一地图提交。
            if self.facts_sink is not None:
                try:
                    facts = self.map_store.capture_facts()
                    if facts.world is not snapshot:
                        raise RuntimeError('committed map/facts snapshot identity mismatch')
                    stamp = message.header.stamp
                    self.facts_sink(facts,
                        source_resolution_m=self.map_store.source_resolution_m,
                        frame_id=self.map_frame,
                        processed_stamp_ns=int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))
                except Exception as exc:
                    # 包含编码/发布/诊断异常；不改变processed，也不终止地图线程。
                    try:
                        self._notice('MAP_FACTS_REJECTED', f'{type(exc).__name__}: {exc}')
                    except Exception:
                        pass
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
        with self._command_lock:
            error,self._command_publish_error=self._command_publish_error,None
        if error:self._notice('COMMAND_PUBLISH_ERROR',error)
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
        # 本周期认证使用实际采样快照；终态观测仍可读取随后推进的定位。
        p3_new_sample=odometry_sequence != self._observed_odometry_sequence
        proof_state,prediction_diagnostics=self._prediction_snapshot(now_s)
        proof_stamp=self._pose_source_high_water
        if self.publish_command is not None:
            value=self.session.state
            self._notice('COMMAND_STATE_MEASURED',json.dumps(dict(
                measured_state=None if value is None else [value.x,value.y,value.yaw,value.v,value.w],
                source_stamp_ns=proof_stamp,proof_basis='p3_measured_pose')))
        # 同一个不可变估计供本次规划和短令认证；任务观测仍读取session.state。
        if prediction_diagnostics is not None:
            self._notice('COMMAND_STATE_ESTIMATE',json.dumps(prediction_diagnostics))
            packet=self.session.tick(now_s,planning_state=proof_state)
        else:
            packet=self.session.tick(now_s)
        proposal = self.tasks.observe(packet, self._body_observation_time(now_s),
                                      state_updated=self._new_body_observation(odometry_sequence),
                                      defer_commit=True)
        odometry_error = None
        proof_world=world
        packet,physical_map_current=self._publication_map_proof(packet,proof_world)
        with self._condition:
            # 目标、最新测量和内部事件提交共享线性化点；不执行规划或外部回调。
            if not self._stopping and input_sequence == self._goal_sequence:
                if self._pose_timeout_s is not None:
                    try:self._apply_pose_input(self._odometry,self._odometry_received_s,time.monotonic(),self._transforms)
                    except (ValueError,AttributeError,TypeError) as exc:odometry_error=exc
                    if self.session.state is None:
                        packet=self.session.tick(now_s)
                        proposal=self.tasks.observe(packet,self._body_observation_time(now_s),state_updated=False,defer_commit=True)
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
                    proposal = self.tasks.observe(packet, self._body_observation_time(now_s),
                        state_updated=self._new_body_observation(odometry_sequence),
                        defer_commit=True)
                if physical_map_current:
                    self.tasks.commit_observation(proposal)
                    # 尚未重算证明时保留新源帧的完成观察机会；接收判活不受影响。
                    if getattr(packet.result,'status',None)!='NOT_DUE':
                        self._observe_body_stamp()
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
                          'requested_epoch', 'requested_goal', 'requested_version',
                          'timing','queue_ms','handoff_ms','elapsed_ms','worker_compute_ms','smoothing_ms',
                          'total_budget_s','smoothing_budget_s',
                          'worker_start_wall_s','worker_finish_wall_s')}
                origin = next((event for event in result.events if event['kind'] == 'GLOBAL_RESULT_RECEIVED'), {})
                record.update(accepted_generation=origin.get('accepted_generation_at_submit'),
                              goal_revision=origin.get('goal_revision_at_submit'),
                              current_accepted_generation=packet.generation)
                self._notice('GLOBAL_RESULT', json.dumps(record, sort_keys=True))
            for event in result.events:
                if event['kind']=='LOCAL_RESULT':
                    self._notice('LOCAL_PLAN_STATUS',json.dumps(dict(reason=event.get('reason'),
                        mode=event.get('candidate_mode'),attempt_reasons=[r.get('reason') for r in event.get('attempts',())],
                        attempt_seeds=[r.get('seed') for r in event.get('attempts',())],
                        attempt_count=len(event.get('attempts',())),
                        timings_ms=result.timings_ms,
                        **{key:event.get(key) for key in ('request_identity','received_s','received_wall_s',
                           'worker_start_wall_s','worker_finish_wall_s','worker_cpu_ms','field_reused')}),sort_keys=True))
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
        published_brake = False
        # 交接锁内核对实际已发速度基准，最多补算一次；不持锁认证。
        for command_attempt in range(2):
            command=(0.,0.)
            previous_command=None
            command_diagnostics={}
            if self.publish_command is not None:
                from nav1cm.execution import certified_velocity_command
                with self._command_lock:
                    previous_command=self._last_issued_command
                    selected_s=self._last_command_selection_s
                    command_now=time.monotonic()
                elapsed=(self.session.coordinator.execution_period_s if selected_s is None
                         else max(0.,command_now-selected_s))
                if not packet.waiting and not self.session.canceling:
                    command=certified_velocity_command(packet.result,proof_state,self.session.coordinator.limits,
                        reference_interval_s=self.reference_interval_s,last_command=previous_command,
                        elapsed_s=elapsed,command_hold_s=self.command_hold_s,
                        backend=self.session.coordinator.backend,diagnostics=command_diagnostics)
                else:
                    command_diagnostics['command_reject_reason']='NO_INPUT' if packet.waiting else 'CANCELING'
                command_diagnostics.update(p3_new_sample=p3_new_sample,
                    source_identity=dict(getattr(self.session.coordinator.buffer, 'source_identity', {})))
            with self._condition:
                if (self._pose_timeout_s is not None and not self._pose_source_current(
                        self._odometry,self._odometry_received_s,time.monotonic())[0]):
                    self._enter_pose_wait()
                    packet=self.session.tick(now_s)
                    data=paths(None,None,self.ros_now_ns(),self.map_frame)
                if self.publish_command is not None and self._odometry is not None:
                    try:self._apply_pose_input(self._odometry,self._odometry_received_s,time.monotonic(),self._transforms)
                    except (ValueError,AttributeError,TypeError):self.session.offer_state(None)
                if (not self._stopping and input_sequence == self._goal_sequence and
                        packet.generation == self.session.accepted_generation and
                        (packet.waiting or self.session.publication_current(packet)) and
                        (self.publish_command is None or physical_map_current)):
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
                    if self.session.state is None:
                        command_diagnostics['final_publish_override_reason']='POSE_UNAVAILABLE'
                    elif self._pose_integrity_fault is not None:
                        command_diagnostics['final_publish_override_reason']='POSE_INTEGRITY_FAULT'
                    handed_off=self._command(packet,command if self.session.state is not None and
                                  self._pose_integrity_fault is None else (0.,0.), proof_stamp, command_diagnostics,
                                  input_sequence=input_sequence,
                                  previous_command=previous_command)
                    if handed_off is False:continue
                    self.publish(data)
                    published_brake = packet.candidate is not None and packet.candidate.source == 'brake'
                elif self.publish_command is not None:
                    command_diagnostics['final_publish_override_reason']=(
                        'STOPPING' if self._stopping else
                        'GOAL_GENERATION_CHANGED' if input_sequence != self._goal_sequence or
                            packet.generation != self.session.accepted_generation else
                        'MAP_NOT_CURRENT' if not physical_map_current else 'PUBLICATION_NOT_CURRENT')
                    self._command(packet,(0.,0.),proof_stamp,command_diagnostics,
                                  input_sequence=input_sequence)
            break
        if published_brake:
            try:
                record = brake_diagnostic(packet, now_s, self.session.coordinator.buffer)
                self._notice('EXECUTION_BRAKE', json.dumps(record, sort_keys=True, default=str))
            except Exception:
                pass  # 诊断故障不能影响已发布的控制参考。
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
                if self.publish_command is not None:self._invalidate_command()
                self._notice('TICK_ERROR', f'{type(exc).__name__}: {exc}')
                with self._condition:
                    if not self._stopping:
                        if self.publish_command is not None and not self._command_threaded:
                            self._publish_issued((0., 0.))
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
            with self._command_lock:
                self._stopping = True
                self._pending_command=None
                if self.publish_command is not None:self._publish_issued((0., 0.))
            self._command_wakeup.set()
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
