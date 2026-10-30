"""单在途 spawn worker：真实 MPCC 与距离场缓存仅在子进程初始化一次。"""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import multiprocessing
import os
from time import perf_counter, process_time, monotonic
import math
from .maps import DistanceFieldCache

_planner = _cache = None
_ready = None


def _initialize(limits, build_dir, planner_options, field_library_path):
    global _planner, _cache, _ready
    from .local_mpcc import MpccPlanner
    start=perf_counter()
    _planner=MpccPlanner(limits,build_dir,**planner_options)
    from .field_native import NativeDistanceField
    _cache=DistanceFieldCache(backend=None if field_library_path is None else NativeDistanceField(field_library_path))
    _ready={'worker_ready_wall_s':perf_counter(),'cold_build_ms':(perf_counter()-start)*1000}


def _restore_arrays(payload):
    """pickle 不保留 bytes 后端只读保证；跨进程两端重建契约对象。"""
    restored=dict(payload)
    for key in ('grid','reference','candidate'):
        if restored.get(key) is not None:
            restored[key]=replace(restored[key])
    return restored


def _dispatch(runner, request, request_wrapper=None):
    request=_restore_arrays(request)
    return runner(request) if request_wrapper is None else request_wrapper(runner,request)


def _ready_probe():
    return dict(_ready or {},worker_pid=os.getpid())


def run_request(request):
    start,cpu=perf_counter(),process_time()
    grid=request['grid']
    field=_cache.get(grid)
    options={}
    if request.get('initial_travel_direction'):
        options['initial_travel_direction']=request['initial_travel_direction']
        options['initial_stopped_angular_rps']=request.get('initial_stopped_angular_rps',.01)
    result=_planner.plan(request['state'],grid,request['reference'],distance_field=field,field_grid_digest=grid.digest(),**options)
    return dict(request_identity=request['identity'],candidate=result.candidate,reason=result.reason,
                timings_ms=result.timings_ms,attempts=result.attempts,worker_pid=os.getpid(),
                worker_start_wall_s=start,worker_finish_wall_s=perf_counter(),
                worker_cpu_ms=(process_time()-cpu)*1000,field_reused=_cache.last_reused,**_ready)



def close_pool(pool, timeout_s=2.):
    """有界退出；管理线程独占回收，退出 sentinel 独立证明子进程已结束。"""
    from multiprocessing.connection import wait
    if not math.isfinite(timeout_s) or timeout_s<0:
        raise ValueError('close timeout must be finite and nonnegative')
    # shutdown(wait=False) 会清空这两个引用，必须先保存本池管理线程和进程句柄。
    manager=getattr(pool,'_executor_manager_thread',None)
    processes=list((getattr(pool,'_processes',None) or {}).values())
    handles=[(process.pid,process,process.sentinel) for process in processes]
    pool.shutdown(wait=False,cancel_futures=True)

    def wait_for_owner(duration):
        if manager is not None:
            # POSIX Process.join/is_alive 会 waitpid；不能与管理线程并发 reap。
            manager.join(duration)
        else:
            # 尚无管理线程时只有调用方拥有回收权（通常也尚无进程）。
            deadline=monotonic()+duration
            for process in processes:
                process.join(max(0.,deadline-monotonic()))

    def running(sentinel):
        # 不读取可能尚未发布的 Popen.returncode，也不调用 waitpid。
        return not wait([sentinel],timeout=0)

    wait_for_owner(timeout_s)
    terminated=[]
    for pid,process,sentinel in handles:
        if running(sentinel):
            terminated.append(pid)
            process.terminate()
    wait_for_owner(.5)
    for _,process,sentinel in handles:
        if running(sentinel): process.kill()
    wait_for_owner(.5)
    return {'terminated_pids':terminated,
            'surviving_pids':[pid for pid,_,sentinel in handles if running(sentinel)]}

class LocalWorker:
    def __init__(self, limits=None, build_dir=None, *, runner=None, planner_options=None, field_backend=None, request_wrapper=None):
        if runner is None and (limits is None or build_dir is None):
            raise ValueError('real worker requires limits and explicit build_dir')
        field_library_path = None if field_backend is None else str(getattr(field_backend,'library_path',field_backend))
        kwargs={} if runner is not None else dict(initializer=_initialize,initargs=(limits,str(build_dir),planner_options or {},field_library_path))
        self._pool=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'),**kwargs)
        self._runner=runner or run_request
        self._request_wrapper=request_wrapper
        self._future=None
        self._closed=False
        self._startup=None
        self._ready_status=None
        self.startup_error=None
        if runner is None: self.start()

    def start(self):
        """非阻塞预热；冷构建结果由 poll_ready 读取，期间不接规划请求。"""
        if self._closed: raise RuntimeError('worker closed')
        if self.failed: raise RuntimeError('worker startup failed: '+self.startup_error['detail'])
        if self._startup is None and self._ready_status is None:
            self._startup=self._pool.submit(_ready_probe)

    @property
    def failed(self): return self.startup_error is not None

    @property
    def ready(self): return not self._closed and not self.failed and self._startup is None

    def poll_ready(self):
        if self._startup is None or not self._startup.done(): return None
        future=self._startup
        try:
            self._ready_status=future.result()
        except Exception as exc:
            self.startup_error={'type':type(exc).__name__,'detail':str(exc)}
            raise
        finally:
            self._startup=None
        return self._ready_status

    @property
    def busy(self): return self._future is not None

    def submit(self, request):
        if self._closed: raise RuntimeError('worker closed')
        if self.failed: raise RuntimeError('worker startup failed: '+self.startup_error['detail'])
        if not self.ready: raise RuntimeError('worker warming up; poll_ready first')
        if self.busy: raise RuntimeError('only one local request may be in flight')
        self._future=self._pool.submit(_dispatch,self._runner,request,self._request_wrapper)

    def poll(self):
        if self._future is None or not self._future.done(): return None
        future,self._future=self._future,None
        return _restore_arrays(future.result())

    def close(self, timeout_s=2.):
        """宽限后仅终止本执行器创建的子进程；不无限等待求解或冷构建。"""
        if not math.isfinite(timeout_s) or timeout_s<0:
            raise ValueError('close timeout must be finite and nonnegative')
        if self._closed: return self._close_result
        self._closed=True
        self._close_result=close_pool(self._pool,timeout_s)
        return self._close_result
    def __enter__(self): return self
    def __exit__(self,*args): self.close()
