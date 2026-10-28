"""真实 APP native、P3 worker 活性与在途取消；必须指定新安装库。"""
import json
import os
import threading
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
from nav1cm.terrain_native import NativeTerrain,load_wheel_capability
from nav1cm.p3_map import P3MapStore
from nav1cm.field_native import NativeDistanceField
from nav1cm.maps import extract_window
from nav1cm.contracts import VehicleState,CellState
from lunar_pure_wheeled_controller.p3_map_source import P3MapSource


def message(values,r=.01):
    h,w=values.shape
    return NS(header=NS(frame_id='map'),info=NS(resolution=r,length_x=w*r,length_y=h*r,
        pose=NS(position=NS(x=w*r/2,y=h*r/2,z=0.),orientation=None)),layers=['elevation'],
        data=[NS(data=values[::-1,::-1].ravel(),layout=None)],outer_start_index=0,inner_start_index=0)


def terrain():
    return NativeTerrain(os.environ['NAV1CM_TERRAIN_LIB'],capability=load_wheel_capability(os.environ['NAV1CM_WHEEL_CONFIG']))


def timed_apply(t,values,r,label):
    started=time.monotonic();u=t.apply(values,resolution_m=r,origin_xy_m=(0.,0.))
    elapsed=time.monotonic()-started
    print(json.dumps({'label':label,'seconds':elapsed,'phases_s':t.phase_seconds,'regions':len(u.regions)}),flush=True)
    assert elapsed<2.5, 'native processing must leave headroom inside unchanged 3s liveness'
    return u


@pytest.mark.parametrize('height',[0.,.3])
def test_full_five_meter_native_first_duplicate_small_change_and_hot_switch(height):
    with terrain() as t:
        values=np.full((500,500),height,dtype=np.float32)
        first=timed_apply(t,values,.01,'first-'+str(height))
        second=timed_apply(t,values,.01,'duplicate-'+str(height))
        np.testing.assert_array_equal(first.regions,second.regions)
        changed=values.copy();changed[250,250]+= .4
        sparse=np.full_like(values,np.nan);sparse[250,250]=height+.4
        timed_apply(t,sparse,.01,'sparse-isolated-change-'+str(height))
        timed_apply(t,changed,.01,'duplicate-dense-changed-'+str(height))
        timed_apply(t,np.full((100,100),height,np.float32),.05,'hot-coarse-'+str(height))
        timed_apply(t,values,.01,'hot-fine-'+str(height))


def test_real_intrinsic_unknown_reaches_snapshot_window_and_center_support():
    t=terrain();store=P3MapStore(terrain=t)
    backend=NativeDistanceField(os.environ['NAV1CM_FIELD_LIB'])
    source=P3MapSource(store,backend,(1.182,.818),minimum_clearance_m=.2)
    source.set_center(2.5,2.5)
    try:
        values=np.zeros((100,100),np.float32)
        source.submit_map(message(values,.05));assert source.wait_idle(3.) and source.error is None
        before=source.snapshot();assert before.grid.eligible[250,250]
        values[50,50]=.4
        received=time.monotonic();source.submit_map(message(values,.05),received)
        assert source.wait_idle(3.) and source.error is None
        after=source.snapshot();world=store.snapshot
        print(json.dumps({'unknown_epoch_before':before.grid.epoch,'unknown_epoch_after':after.grid.epoch,
            'receipt_age_s':after.processed_at-after.received_at,'eligible':bool(after.grid.eligible[250,250])}),flush=True)
        assert after.received_at==received and after.grid.epoch!=before.grid.epoch
        assert not after.grid.eligible[250,250]
        w=extract_window(world,VehicleState(2.5,2.5,0.,0.,0.),False)
        assert w.cells[252,252]==CellState.UNKNOWN
        native=t.apply(values,resolution_m=.05,origin_xy_m=(0.,0.))
        assert any(x0<=50<x1 and y0<=50<y1 and state==-1 for x0,y0,x1,y1,state in native.regions)
    finally:source.close()


def test_dense_source_worker_receipts_and_inflight_close():
    t=terrain();store=P3MapStore(terrain=t)
    source=P3MapSource(store,NativeDistanceField(os.environ['NAV1CM_FIELD_LIB']),(1.182,.818),minimum_clearance_m=.2)
    source.set_center(2.5,2.5)
    try:
        for label,r,n,z in [('first',.01,500,0.),('duplicate',.01,500,0.),('small-change',.01,500,0.),('hot',.05,100,.3),('fine',.01,500,.3)]:
            values=np.full((n,n),z,np.float32)
            if label=='small-change':values[250,250]=.4
            received=time.monotonic();source.submit_map(message(values,r),received)
            assert source.wait_idle(3.) and source.error is None
            result=source.snapshot();age=result.processed_at-result.received_at
            print(json.dumps({'worker':label,'receipt_age_s':age,'phases_s':t.phase_seconds}),flush=True)
            assert result.received_at==received and age<2.5
            assert bool(result.grid.eligible[250,250])==(label!='small-change')
    finally:source.close()
    assert not source._worker.is_alive() and not t._finalizer.alive
    entered=threading.Event()
    class InflightTerrain(NativeTerrain):
        def apply(self,*args,**kwargs):
            entered.set();return super().apply(*args,**kwargs)
    t=InflightTerrain(os.environ['NAV1CM_TERRAIN_LIB'],capability=load_wheel_capability(os.environ['NAV1CM_WHEEL_CONFIG']))
    source=P3MapSource(P3MapStore(terrain=t),NativeDistanceField(os.environ['NAV1CM_FIELD_LIB']),(1.182,.818),minimum_clearance_m=.2)
    source.set_center(2.5,2.5);source.submit_map(message(np.zeros((500,500),np.float32)))
    assert entered.wait(2.);time.sleep(.02)
    started=time.monotonic();source.close();elapsed=time.monotonic()-started
    print(json.dumps({'inflight_close_s':elapsed,'error':source.error,'alive':source._worker.is_alive()}),flush=True)
    assert elapsed<2. and not source._worker.is_alive() and not t._finalizer.alive


def test_planner_map_worker_cancels_inflight_dense_apply():
    from lunar_nav1cm_planner.runtime import PlannerRuntime
    from nav1cm.contracts import WorldSnapshot
    entered=threading.Event()
    class InflightTerrain(NativeTerrain):
        def apply(self,*args,**kwargs):
            entered.set();return super().apply(*args,**kwargs)
    t=InflightTerrain(os.environ['NAV1CM_TERRAIN_LIB'],capability=load_wheel_capability(os.environ['NAV1CM_WHEEL_CONFIG']))
    session=NS(world=WorldSnapshot(0,(),()),close=lambda **kwargs:{'closed':True})
    runtime=PlannerRuntime(session,P3MapStore(terrain=t),publish=lambda _:None,ros_now_ns=lambda:0)
    runtime._threads=[threading.Thread(target=runtime._map_loop,name='nav1cm-map-test')]
    runtime._threads[0].start();runtime.offer_map(message(np.zeros((500,500),np.float32)))
    assert entered.wait(2.);time.sleep(.02)
    started=time.monotonic();result=runtime.close();elapsed=time.monotonic()-started
    print(json.dumps({'planner_inflight_close_s':elapsed,'result':result}),flush=True)
    assert result['threads_alive']==[] and elapsed<2. and not t._finalizer.alive
