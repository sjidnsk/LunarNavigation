"""隔离进程通道的生命周期、失效和不可变快照契约。"""
import multiprocessing
from time import monotonic, sleep

import numpy as np
import pytest

from lunar_pure_wheeled_controller import p3_map_source as module
from lunar_pure_wheeled_controller.mppi_map import GridMap


def _echo_worker(connection, *_args):
    """只替换 native 计算；真实 spawn、Pipe、父线程均保留。"""
    try:
        connection.send(('ready',))
        while True:
            packet = connection.recv()
            if packet is None:
                return
            pending, transforms, center = packet
            if pending is not None and pending[0] == 'crash':
                return
            if pending is not None and pending[0] == 'hang':
                sleep(30.)
            grid = GridMap(np.ones((2, 2), np.uint8), .01, (0., 0.), epoch='7', revision=3)
            received = pending[1] if pending is not None else 1.
            connection.send(('result', module.MapReceipt(grid, received, monotonic()), True, None, .05))
    finally:
        connection.close()


@pytest.fixture
def process_source(monkeypatch):
    context = multiprocessing.get_context('spawn')
    original = context.Process
    monkeypatch.setattr(context, 'Process', lambda **kw: original(**dict(kw, target=_echo_worker)))
    monkeypatch.setattr(module.multiprocessing, 'get_context', lambda method: context)
    source = module.P3MapSource(None, None, (1.182, .818), native_spec=('terrain', 'field', {}))
    yield source
    source.close()


def test_spawn_receipt_keeps_time_and_refreezes_arrays(process_source):
    source = process_source
    source.set_center(0., 0.)
    source.submit_map('valid', 12.5)
    assert source.wait_idle()
    receipt = source.snapshot()
    assert receipt.received_at == 12.5
    assert receipt.grid.epoch == '7' and receipt.grid.revision == 3
    for array in (receipt.grid.states, receipt.grid.costs, receipt.grid.assumed, receipt.grid.eligible):
        with pytest.raises(ValueError):
            array.flags.writeable = True
    source.set_center(1., 0.)
    assert source.wait_idle()
    assert source.snapshot().grid is receipt.grid
    assert source._process.pid != multiprocessing.current_process().pid


def test_process_exit_invalidates_previous_receipt(process_source):
    source = process_source
    source.submit_map('valid', 1.)
    assert source.wait_idle() and source.snapshot() is not None
    source.submit_map('crash', 2.)
    assert source.wait_idle()
    assert source.snapshot() is None
    assert source.error and 'process' in source.error.lower()


def test_idle_process_exit_is_detected_and_close_reaps(process_source):
    source = process_source
    source.submit_map('valid', 1.)
    assert source.wait_idle()
    source._process.terminate()
    source._process.join(2.)
    deadline = monotonic() + 2.
    while source.snapshot() is not None and monotonic() < deadline:
        sleep(.01)
    assert source.snapshot() is None and source.error
    source.close()
    assert not source._worker.is_alive()
    assert not source._process.is_alive()


def test_close_terminates_busy_process_within_bound(process_source):
    source = process_source
    source.submit_map('hang', 1.)
    assert not source.wait_idle(.2)
    started = monotonic()
    source.close()
    assert monotonic() - started < 3.
    assert not source._worker.is_alive() and not source._process.is_alive()


def test_native_initialization_failure_reaps_child():
    capability = {'maximum_slope_rad': .3, 'maximum_local_obstacle_relief_m': .1,
                  'minimum_underbody_clearance_m': .2}
    existing = {process.pid for process in multiprocessing.active_children()}
    with pytest.raises(RuntimeError, match='P3 process failed'):
        module.P3MapSource(None, None, (1., .5),
                           native_spec=('/nonexistent/terrain.so', '/nonexistent/field.so', capability))
    assert {process.pid for process in multiprocessing.active_children()} == existing


def test_factory_passes_only_native_spec_to_source(monkeypatch):
    import ament_index_python.packages
    monkeypatch.setattr(ament_index_python.packages, 'get_package_prefix', lambda _: '/installed')
    recorded = {}
    def capture(store, backend, body, **kwargs):
        recorded.update(store=store, backend=backend, body=body, **kwargs)
        return recorded
    monkeypatch.setattr(module, 'P3MapSource', capture)
    capability = {'maximum_slope_rad': .3, 'maximum_local_obstacle_relief_m': .1,
                  'minimum_underbody_clearance_m': .2, 'minimum_clearance_m': .02}
    result = module.create_p3_source(capability, (1., .5))
    assert result['store'] is None and result['backend'] is None
    assert tuple(map(str, result['native_spec'][:2])) == (
        '/installed/lib/libnav1cm_terrain.so', '/installed/lib/libnav1cm_distance_field.so')
    assert result['native_spec'][2] == capability
