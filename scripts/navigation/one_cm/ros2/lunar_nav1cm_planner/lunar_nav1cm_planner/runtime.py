"""不依赖 rclpy 的线程调度；回调不运行地图、目标碰撞检查或求解。

地图线程独占 P3MapStore，tick 线程独占 session/coordinator。只在交接最新
输入、不可变地图及发布 guard 时短暂加锁；不会持锁执行 native 地图计算。
"""
from collections import deque
import json
import threading
import time

from nav1cm.ros_conversion import (
    LatestTransforms, MissingTransform, goal_from_message, map_from_message,
    paths, state_from_odometry,
)


class PlannerRuntime:
    """factory/消息回调注入只用于内部验证，不构成对外延迟或控制协议。"""
    def __init__(self, session, map_store, *, publish, ros_now_ns, notice=None, map_frame='map'):
        self.session, self.map_store = session, map_store
        self.publish, self.ros_now_ns = publish, ros_now_ns
        self.notice = notice or (lambda *args: None)
        self.map_frame = map_frame
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
            self._condition.notify_all()

    def offer_goal(self, message):
        with self._condition:
            self._goal_sequence += 1
            # 不能用单一 latest raw 槽：同一 tick 前有效 A→无效 B 必须保留 A。
            self._goals.append((self._goal_sequence, message))
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

    def _tick_once(self, now_s):
        with self._condition:
            world, odometry = self._world, self._odometry
            transforms = self._transforms.copy()
            goals, self._goals = self._goals, deque()
        if world is not None:
            self.session.set_world(world)
        if odometry is not None:
            try:
                self.session.offer_state(state_from_odometry(odometry, transforms, self.map_frame))
            except (ValueError, AttributeError, TypeError) as exc:
                self.session.offer_state(None)
                self._notice('ODOMETRY_WAITING', exc)
        unresolved = []
        for sequence, message in goals:
            try:
                goal = goal_from_message(message, transforms, self.map_frame)
            except MissingTransform as exc:
                unresolved.append((sequence, message))
                self._notice('GOAL_WAITING_TF', exc)
                continue
            except (ValueError, AttributeError, TypeError) as exc:
                self._notice('GOAL_REJECTED', exc)
                continue
            if self.session.offer_goal(goal):
                self._last_accepted_input = sequence
                self._notice('GOAL_ACCEPTED', f'generation={self.session.accepted_generation}, pose={goal}')
            else:
                self._notice('GOAL_REJECTED', self.session.last_goal_rejection)
        with self._condition:
            # 较早等待 TF 的目标不能在较新有效目标之后重新覆盖它。
            self._goals.extendleft(reversed([item for item in unresolved if item[0] > self._last_accepted_input]))
        packet = self.session.tick(now_s)
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
                if event['kind'] in ('GLOBAL_CURRENT_MAP_REVALIDATED', 'GLOBAL_CURRENT_MAP_REJECTED',
                                      'LOCAL_CURRENT_MAP_REVALIDATED'):
                    self._notice(event['kind'], json.dumps(event, sort_keys=True))
        if packet.waiting:
            self._notice('WAITING_INPUT', ','.join(packet.waiting))
        elif packet.result.status == 'NOT_DUE':
            return packet
        data = paths(packet.route, packet.candidate, self.ros_now_ns(), self.map_frame)
        # ROS 时间仅给本次 Path 定 t0；调度/轨迹年龄始终使用单调时钟。
        with self._condition:
            if (not self._stopping and packet.generation == self.session.accepted_generation and
                    (packet.waiting or self.session.publication_current(packet))):
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
