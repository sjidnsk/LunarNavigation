"""单所有者 P3/native worker；生产地图计算隔离至 spawn 进程。"""
from dataclasses import dataclass, replace
import multiprocessing
from threading import Condition, Thread
from time import monotonic
import numpy as np
from nav1cm.follower_map import center_support
from nav1cm.contracts import resolve_minimum_clearance
from nav1cm.ros_conversion import LatestTransforms, MissingTransform, map_from_message
from .mppi_map import GridMap


@dataclass(frozen=True)
class MapReceipt:
    grid: GridMap
    received_at: float
    processed_at: float


class _MapComputation:
    """线程与子进程共用同一累计地图算法；状态仅由单个 worker 访问。"""
    def __init__(self, store, backend, body, clearance, allow_unknown, frame):
        self.store=store;self.backend=backend;self.body=body
        self.clearance=clearance;self.allow_unknown=allow_unknown;self.frame=frame
        self.world=None;self.valid_received=None;self.key=None;self.revision=0;self.receipt=None

    def process_inputs(self, pending, transforms, center):
        consumed=False;error=None
        try:
            if pending is not None:
                message,received=pending
                transform=map_from_message(message,transforms,self.frame)
                self.world=self.store.apply(message,transform);self.valid_received=received
                consumed=True
            if self.world is not None and center is not None:
                key=(self.world.epoch,center,self.body,self.clearance,self.allow_unknown)
                if key!=self.key:
                    support=center_support(self.world,*center,body_length_m=self.body[0],
                        body_width_m=self.body[1],minimum_clearance_m=self.clearance,
                        allow_unknown=self.allow_unknown,backend=self.backend)
                    # 明确 -1/0/1 -> 0/1/2，不能直接强转 intrinsic。
                    states=np.where(support.cells==-1,0,np.where(support.cells==0,1,2)).astype(np.uint8)
                    self.revision+=1
                    grid=GridMap(states,.01,support.origin,revision=self.revision,
                                 epoch=str(self.world.epoch),eligible=support.eligible)
                    self.key=key
                else:grid=self.receipt.grid
                self.receipt=MapReceipt(grid,self.valid_received,monotonic())
        except MissingTransform as exc:
            error=str(exc)  # 保留待转换消息，下一次 TF 到达再试。
        except Exception as exc:
            error=str(exc);consumed=True
        return self.receipt,consumed,error,getattr(self.store,'source_resolution_m',None)


def _native_process(connection, native_spec, body, clearance, allow_unknown, frame):
    """在新解释器内创建 ABI handle，不继承 ROS/native 线程或句柄。"""
    store=None
    try:
        from nav1cm.field_native import NativeDistanceField
        from nav1cm.p3_map import P3MapStore
        from nav1cm.terrain_native import WheelTerrainCapability
        terrain_path,field_path,capability=native_spec
        store=P3MapStore(library_path=terrain_path,capability=WheelTerrainCapability.from_mapping(capability))
        backend=NativeDistanceField(field_path)
        computation=_MapComputation(store,backend,body,clearance,allow_unknown,frame)
        connection.send(('ready',))
        while True:
            packet=connection.recv()
            if packet is None:return
            connection.send(('result',*computation.process_inputs(*packet)))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as exc:
        try:connection.send(('fatal',f'P3 process failed: {type(exc).__name__}: {exc}'))
        except (EOFError,OSError):pass
    finally:
        if store is not None:store._terrain.close()
        connection.close()


class P3MapSource:
    def __init__(self,store,backend,body_dimensions,*,minimum_clearance_m=0.0,allow_unknown=False,map_frame='map',native_spec=None):
        self._store=store;self._body=tuple(body_dimensions)
        self._minimum_clearance=resolve_minimum_clearance(minimum_clearance_m)
        self._condition=Condition();self._closed=False;self._generation=0;self._completed=0
        self._transforms=LatestTransforms();self._pending=None;self._center=None
        self._receipt=None;self.error=None;self._process=None;self._connection=None;self._failed=False
        self._source_resolution_m=None;self._computation=None
        if native_spec is None:
            self._computation=_MapComputation(store,backend,self._body,self._minimum_clearance,allow_unknown,map_frame)
        else:
            context=multiprocessing.get_context('spawn')
            self._connection,child_connection=context.Pipe()
            self._process=context.Process(target=_native_process,
                args=(child_connection,native_spec,self._body,self._minimum_clearance,allow_unknown,map_frame),
                name='standard-p3-map',daemon=False)
            try:
                self._process.start()
                child_connection.close()
                if not self._connection.poll(5.):raise RuntimeError('P3 process initialization timed out')
                packet=self._connection.recv()
                if packet[0]!='ready':raise RuntimeError(packet[1])
            except BaseException:
                child_connection.close();self._stop_process();raise
        self._worker=Thread(target=self._run,name='standard-p3-map',daemon=False)
        try:self._worker.start()
        except BaseException:
            self._stop_process();raise

    def _wake(self):
        self._generation+=1;self._condition.notify_all()

    def submit_map(self,message,received_at=None):
        with self._condition:
            self._pending=(message,monotonic() if received_at is None else received_at)
            self._wake()

    def set_transforms(self,transforms):
        with self._condition:
            self._transforms=transforms.copy();self._wake()

    def set_center(self,x,y):
        with self._condition:
            if self._center!=(x,y):
                self._center=(x,y);self._wake()

    def snapshot(self):
        with self._condition:
            if self._process is not None and not self._process.is_alive():
                self._process_failure(f'exitcode={self._process.exitcode}')
            return self._receipt

    def wait_idle(self,timeout=5.):
        with self._condition:
            generation=self._generation
            return self._condition.wait_for(lambda:self._completed>=generation or self._failed,timeout)

    def _process_failure(self,exc):
        self._failed=True;self._receipt=None
        self.error=f'P3 process unavailable: {exc}'
        self._completed=self._generation;self._condition.notify_all()

    def _exchange(self,pending,transforms,center):
        self._connection.send((pending,transforms,center))
        while not self._connection.poll(.05):
            if self._closed:return None
            if not self._process.is_alive():raise RuntimeError(f'exitcode={self._process.exitcode}')
        packet=self._connection.recv()
        if packet[0]!='result':raise RuntimeError(packet[1])
        _,receipt,consumed,error,resolution=packet
        if receipt is not None:
            # pickle 不保留 ndarray 的不可写底层；相同版本复用父端地图身份。
            old=self._receipt
            grid=old.grid if old is not None and (old.grid.epoch,old.grid.revision)==(receipt.grid.epoch,receipt.grid.revision) else replace(receipt.grid)
            receipt=replace(receipt,grid=grid)
        return receipt,consumed,error,resolution

    def _run(self):
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda:self._closed or self._generation>self._completed,
                                             timeout=.05 if self._process is not None else None)
                    if self._closed:return
                    if self._process is not None and not self._process.is_alive():
                        self._process_failure(f'exitcode={self._process.exitcode}')
                    if self._failed:
                        self._completed=self._generation;self._condition.notify_all();return
                    if self._generation<=self._completed:continue
                    generation=self._generation
                    pending=self._pending;transforms=self._transforms;center=self._center
                try:
                    result=(self._exchange(pending,transforms,center) if self._process is not None else
                            self._computation.process_inputs(pending,transforms,center))
                    if result is None:return
                    receipt,consumed,error,resolution=result
                    with self._condition:
                        self._receipt=receipt;self.error=error;self._source_resolution_m=resolution
                        if consumed and self._pending is pending:self._pending=None
                        self._completed=generation;self._condition.notify_all()
                except Exception as exc:
                    with self._condition:self._process_failure(exc)
        finally:
            if self._connection is not None:
                try:self._connection.send(None)
                except (OSError,EOFError):pass

    def _stop_process(self):
        if self._process is not None and self._process.pid is not None:
            self._process.join(.5)
            if self._process.is_alive():self._process.terminate();self._process.join(.5)
            if self._process.is_alive():self._process.kill();self._process.join(.5)
        if self._connection is not None:self._connection.close()

    def close(self):
        with self._condition:self._closed=True;self._condition.notify_all()
        if self._process is None:
            self._worker.join()
            terrain=getattr(self._store,'_terrain',None)
            if terrain is not None:terrain.close()
        else:
            self._worker.join(.25)
            self._stop_process()
            self._worker.join(.5)


def create_p3_source(capability,body_dimensions,*,minimum_clearance_m=None,allow_unknown=False,map_frame='map'):
    """生产仅传送已验证纯数据；ABI 和地图累计均归独立进程。"""
    from pathlib import Path
    from ament_index_python.packages import get_package_prefix
    from nav1cm.terrain_native import WheelTerrainCapability
    WheelTerrainCapability.from_mapping(capability)
    prefix=Path(get_package_prefix('lunar_nav1cm_planner'))
    return P3MapSource(None,None,body_dimensions,
        native_spec=(str(prefix/'lib/libnav1cm_terrain.so'),str(prefix/'lib/libnav1cm_distance_field.so'),dict(capability)),
        minimum_clearance_m=resolve_minimum_clearance(capability.get('minimum_clearance_m',0.0),minimum_clearance_m),
        allow_unknown=allow_unknown,map_frame=map_frame)
