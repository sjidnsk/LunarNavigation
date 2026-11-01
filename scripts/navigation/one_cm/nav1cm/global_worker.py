"""独立 spawn 全局搜索；单在途任务，父端只轮询，不等待 Future。"""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import multiprocessing
import os
from time import perf_counter

from .contracts import CellRect, Route, WheelLimits, WorldSnapshot
from .global_planner import AdaptiveGlobalPlanner
from .reference_smoothing import shape_route, ShapedRoute
import numpy as np


def make_request(world, start, goal, version, limits, clearance_radius_m=.75, budget_s=1., *,
                 global_soft_clearance_m=.4, global_clearance_weight=1.0,
                 reference_smoothing=False, reference_smoothing_budget_s=.2, terminal_yaw=None):
    return {'world': asdict(world), 'start': list(start), 'goal': list(goal),
            'version': version, 'limits': asdict(limits), 'clearance_radius_m': clearance_radius_m,
            'global_soft_clearance_m': global_soft_clearance_m,
            'global_clearance_weight': global_clearance_weight,
            'reference_smoothing': reference_smoothing,
            'reference_smoothing_budget_s': reference_smoothing_budget_s, 'terminal_yaw': terminal_yaw,
            'budget_s': budget_s, 'submitted_wall_s': perf_counter()}


def prepare_worker():
    # 在导航等待目标时启动spawn/导入，并预先解析可选原生代价符号；不规划或发布。
    from .global_cost_native import load_cost_function
    load_cost_function()
    return {'worker_pid': os.getpid(), 'ready_wall_s': perf_counter()}


def run_request(request):
    started = perf_counter()
    raw = request['world']
    world = WorldSnapshot(raw['epoch'], tuple(CellRect(**r) for r in raw['blocked']),
                          tuple(CellRect(**r) for r in raw['known_free']))
    planner = AdaptiveGlobalPlanner(WheelLimits(**request['limits']), request['clearance_radius_m'],
        soft_clearance_m=request.get('global_soft_clearance_m', .4),
        clearance_weight=request.get('global_clearance_weight', 1.0))
    result = planner.plan(world, request['start'], request['goal'], request['version'], request['budget_s'])
    payload = asdict(result)
    if result.route is not None:
        payload['route']['points'] = result.route.points.tolist()
    payload['smoothing_ms'] = 0.
    if request.get('reference_smoothing') and result.route is not None:
        began = perf_counter()
        radius = result.clearance_radius_m
        boxes = np.asarray([(r.x0/100-radius, r.y0/100-radius, r.x1/100+radius, r.y1/100+radius)
                            for r in world.blocked], dtype=float).reshape(-1, 4)
        shaped = shape_route(result.route, boxes, raw_verified=True,
            budget_s=request.get('reference_smoothing_budget_s', .2),
            terminal_yaw=request.get('terminal_yaw'), corner_radius=radius,
            soft_clearance_m=request.get('global_soft_clearance_m', .4),
            clearance_weight=request.get('global_clearance_weight', 1.0))
        payload['shaped_reference'] = asdict(shaped)
        payload['shaped_reference']['route']['points'] = shaped.route.points.tolist()
        payload['shaped_reference']['proof_boxes'] = shaped.proof_boxes.tolist()
        payload['smoothing_ms'] = (perf_counter()-began)*1000
    payload['worker_compute_ms'] = (perf_counter()-started)*1000
    payload.update(worker_pid=os.getpid(), requested_version=request['version'],
                   request_reason=request.get('request_reason', 'TEST'),
                   requested_goal=request['goal'], requested_epoch=world.epoch,
                   submitted_wall_s=request['submitted_wall_s'], worker_start_wall_s=started,
                   worker_finish_wall_s=perf_counter())
    try:
        import resource
        payload['worker_rss_peak_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    except (ImportError, AttributeError):
        payload['worker_rss_peak_kib'] = None
    return payload


def restore_result(payload):
    """不用 pickle 保留只读性的偶然行为，重新建立 bytes 支撑的 Route。"""
    result = dict(payload)
    raw = result.get('route')
    if raw is not None:
        route = Route(raw['version'], raw['map_epoch'], raw['points'], raw['goal'])
        if (route.version != result['requested_version'] or route.map_epoch != result['requested_epoch']
                or route.goal != tuple(result['requested_goal'])):
            raise ValueError('worker route/request identity mismatch')
        result['route'] = route
    if result.get('shaped_reference') is not None:
        shaped = result['shaped_reference']
        geometry = shaped['route']
        reference = Route(geometry['version'], geometry['map_epoch'], geometry['points'], geometry['goal'])
        if (raw is None or reference.version != result['route'].version or
                reference.map_epoch != result['route'].map_epoch or reference.goal != result['route'].goal or
                not np.array_equal(reference.points[[0,-1]], result['route'].points[[0,-1]])):
            raise ValueError('worker shaped/raw route identity mismatch')
        result['shaped_reference'] = ShapedRoute(reference, np.asarray(shaped['proof_boxes']).reshape(-1,4),
                                                shaped['changed'], shaped['reason'])
    success = result.get('planning_outcome') == 0 and result.get('reason_code') == 'PLAN_FOUND' and result.get('has_reference') is True
    if success != (raw is not None):
        raise ValueError('worker success/route contract mismatch')
    result['parent_received_wall_s'] = perf_counter()
    result['queue_ms'] = (result['worker_start_wall_s'] - result['submitted_wall_s']) * 1000
    result['handoff_ms'] = (result['parent_received_wall_s'] - result['worker_finish_wall_s']) * 1000
    result['elapsed_ms'] = (result['parent_received_wall_s'] - result['submitted_wall_s']) * 1000
    return result


class GlobalWorker:
    def __init__(self):
        self._pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        self._future = None
        self._preparation = self._pool.submit(prepare_worker)
    @property
    def busy(self):
        return self._future is not None
    def submit(self, request):
        if self.busy:
            raise RuntimeError('only one global request may be in flight')
        self._future = self._pool.submit(run_request, request)
    def poll(self):
        if self._future is None or not self._future.done():
            return None
        future, self._future = self._future, None
        return future.result()
    def close(self):
        self._pool.shutdown(wait=True, cancel_futures=True)
    def __enter__(self): return self
    def __exit__(self, *args): self.close()
