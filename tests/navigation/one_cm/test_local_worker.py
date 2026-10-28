import time
import pytest
from scripts.navigation.one_cm.nav1cm.local_worker import LocalWorker


def echo(request):
    time.sleep(.1)
    return request


def test_spawn_single_inflight_poll_and_close():
    with LocalWorker(runner=echo) as worker:
        worker.submit({'sequence':1})
        assert worker.busy
        with pytest.raises(RuntimeError): worker.submit({'sequence':2})
        result=None
        deadline=time.monotonic()+15
        while result is None and time.monotonic()<deadline:
            result=worker.poll()
            time.sleep(.01)
        assert result == {'sequence':1}
        assert not worker.busy
    with pytest.raises(RuntimeError): worker.submit({})

def test_spawn_rebuilds_immutable_arrays_and_nonblocking_warmup():
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot, VehicleState, LocalReference
    from scripts.navigation.one_cm.nav1cm.maps import extract_window
    from scripts.navigation.one_cm.nav1cm.seeds import brake_candidate
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    state=VehicleState(0,0,0,0,0)
    grid=extract_window(WorldSnapshot(1,(),(CellRect(-1000,-1000,1000,1000),)),state,False)
    ref=LocalReference(1,1,[[0,0],[1,0]],[0,1])
    trajectory=brake_candidate(state,grid,ref,WheelLimits())
    with LocalWorker(runner=echo) as worker:
        worker.start()
        deadline=time.monotonic()+15
        ready=None
        while ready is None and time.monotonic()<deadline:
            ready=worker.poll_ready(); time.sleep(.01)
        assert worker.ready and ready['worker_pid']>0
        worker.submit({'grid':grid,'reference':ref,'candidate':trajectory})
        result=None
        while result is None and time.monotonic()<deadline:
            result=worker.poll(); time.sleep(.01)
        for array in (result['grid'].cells,result['reference'].points,result['candidate'].states):
            with pytest.raises(ValueError): array.setflags(write=True)

def test_native_backend_crosses_spawn_as_path_only(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import local_worker as module
    captured={}
    class Future:
        def done(self): return False
    class Pool:
        def __init__(self,**kwargs): captured.update(kwargs)
        def submit(self,*args): return Future()
        def shutdown(self,**kwargs): pass
    class NativeHandle:
        library_path='D:/explicit/native-field.dll'
        def __reduce__(self): raise TypeError('native handle must not be pickled')
    monkeypatch.setattr(module,'ProcessPoolExecutor',Pool)
    with module.LocalWorker(limits='limits',build_dir='D:/build',field_backend=NativeHandle()) as worker:
        assert captured['initargs'][-1]=='D:/explicit/native-field.dll'
        assert not worker.ready
        assert worker.poll_ready() is None
import time

def slow(request):
    time.sleep(30)
    return request


def test_close_is_bounded_for_slow_worker():
    worker=LocalWorker(runner=slow)
    worker.submit({'sequence':1})
    started=time.monotonic()
    result=worker.close(timeout_s=.1)
    assert time.monotonic()-started<2.
    assert result['terminated_pids']
    assert result['surviving_pids']==[]
    assert worker.close()==result

def fail_initializer(*args):
    raise RuntimeError('intentional initializer failure')


def test_initializer_failure_is_terminal_and_cannot_submit(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import local_worker as module
    from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
    monkeypatch.setattr(module,'_initialize',fail_initializer)
    with LocalWorker(limits=WheelLimits(),build_dir='D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/unused-init-failure') as worker:
        deadline=time.monotonic()+15
        observed=None
        while observed is None and time.monotonic()<deadline:
            try: worker.poll_ready()
            except Exception as exc: observed=exc
            time.sleep(.01)
        assert observed is not None
        assert worker.failed and not worker.ready
        assert worker.startup_error['type']==type(observed).__name__
        assert worker.poll_ready() is None
        with pytest.raises(RuntimeError,match='startup failed'): worker.submit({})
        with pytest.raises(RuntimeError,match='startup failed'): worker.start()

@pytest.mark.parametrize('manager_finishes',[True,False])
def test_close_uses_exit_sentinel_when_reaper_has_not_published_returncode(monkeypatch,manager_finishes):
    """模拟已退出/reap但returncode仍None；不把缓存状态当成存活证据。"""
    from scripts.navigation.one_cm.nav1cm import local_worker as module
    import multiprocessing.connection
    class Process:
        pid=12345
        sentinel=987
        cached_returncode=None
        signals=[]
        def join(self,timeout): pass  # ECHILD 分支立即返回；缓存仍为空
        def is_alive(self): return self.cached_returncode is None
        def terminate(self): self.signals.append('terminate')
        def kill(self): self.signals.append('kill')
    process=Process()
    class Manager:
        done=False
        def join(self,timeout):
            if manager_finishes:
                process.cached_returncode=0
                self.done=True
        def is_alive(self): return not self.done
    class Pool:
        _processes={process.pid:process}
        _executor_manager_thread=Manager()
        def shutdown(self,**kwargs): self._executor_manager_thread=None
    monkeypatch.setattr(multiprocessing.connection,'wait',lambda handles,timeout: list(handles))
    result=module.close_pool(Pool(),timeout_s=.01)
    assert result=={'terminated_pids':[],'surviving_pids':[]}
    assert process.signals==[]


def test_close_reports_genuine_survivor_after_bounded_signal_attempts(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import local_worker as module
    import multiprocessing.connection
    waits=[]
    class Process:
        pid=12346
        sentinel=988
        signals=[]
        def join(self,timeout): waits.append(timeout)
        def is_alive(self): return True
        def terminate(self): self.signals.append('terminate')
        def kill(self): self.signals.append('kill')
    process=Process()
    class Manager:
        def join(self,timeout): waits.append(timeout)
        def is_alive(self): return True
    class Pool:
        _processes={process.pid:process}
        _executor_manager_thread=Manager()
        def shutdown(self,**kwargs): self._executor_manager_thread=None
    monkeypatch.setattr(multiprocessing.connection,'wait',lambda handles,timeout: [])
    result=module.close_pool(Pool(),timeout_s=.01)
    assert result=={'terminated_pids':[12346],'surviving_pids':[12346]}
    assert process.signals==['terminate','kill']
    assert len(waits)==3 and all(0<=t<=.5 for t in waits)


def test_completed_spawn_worker_closes_without_signals():
    worker=LocalWorker(runner=echo)
    worker.submit({'sequence':1})
    deadline=time.monotonic()+15
    result=None
    while result is None and time.monotonic()<deadline:
        result=worker.poll(); time.sleep(.01)
    assert result=={'sequence':1}
    assert worker.close(timeout_s=.5)=={'terminated_pids':[],'surviving_pids':[]}
