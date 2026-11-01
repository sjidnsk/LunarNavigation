"""独立 spawn 全局搜索；单在途任务，父端只轮询，不等待 Future。"""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import multiprocessing
import os
from time import perf_counter

from .contracts import CellRect, Route, WheelLimits, WorldSnapshot
from .global_planner import AdaptiveGlobalPlanner


def make_request(world, start, goal, version, limits, clearance_radius_m=.75, budget_s=1., *,
                 global_soft_clearance_m=.4, global_clearance_weight=.5):
    return {'world': asdict(world), 'start': list(start), 'goal': list(goal),
            'version': version, 'limits': asdict(limits), 'clearance_radius_m': clearance_radius_m,
            'global_soft_clearance_m': global_soft_clearance_m,
            'global_clearance_weight': global_clearance_weight,
            'budget_s': budget_s, 'submitted_wall_s': perf_counter()}


def run_request(request):
    started = perf_counter()
    raw = request['world']
    world = WorldSnapshot(raw['epoch'], tuple(CellRect(**r) for r in raw['blocked']),
                          tuple(CellRect(**r) for r in raw['known_free']))
    planner = AdaptiveGlobalPlanner(WheelLimits(**request['limits']), request['clearance_radius_m'],
        soft_clearance_m=request.get('global_soft_clearance_m', .4),
        clearance_weight=request.get('global_clearance_weight', .5))
    result = planner.plan(world, request['start'], request['goal'], request['version'], request['budget_s'])
    payload = asdict(result)
    if result.route is not None:
        payload['route']['points'] = result.route.points.tolist()
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
