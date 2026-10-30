"""单所有者 P3/native worker；回调只投递最新输入，不在 ROS 回调作地图计算。"""
from dataclasses import dataclass
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


class P3MapSource:
    def __init__(self,store,backend,body_dimensions,*,minimum_clearance_m=0.0,allow_unknown=False,map_frame='map'):
        self._store=store;self._backend=backend;self._body=tuple(body_dimensions)
        self._minimum_clearance=resolve_minimum_clearance(minimum_clearance_m)
        self._allow_unknown=allow_unknown;self._frame=map_frame
        self._condition=Condition();self._closed=False;self._generation=0;self._completed=0
        self._transforms=LatestTransforms();self._pending=None;self._center=None
        self._receipt=None;self._valid_received=None;self._world=None;self._key=None;self._revision=0
        self.error=None
        self._worker=Thread(target=self._run,name='standard-p3-map',daemon=False)
        self._worker.start()

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
        with self._condition:return self._receipt

    def wait_idle(self,timeout=5.):
        with self._condition:
            generation=self._generation
            return self._condition.wait_for(lambda:self._completed>=generation,timeout)

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda:self._closed or self._generation>self._completed)
                if self._closed:return
                generation=self._generation
                pending=self._pending;transforms=self._transforms;center=self._center
            try:
                if pending is not None:
                    message,received=pending
                    transform=map_from_message(message,transforms,self._frame)
                    world=self._store.apply(message,transform)
                    self._world=world;self._valid_received=received
                    with self._condition:
                        if self._pending is pending:self._pending=None
                if self._world is not None and center is not None:
                    key=(self._world.epoch,center,self._body,self._minimum_clearance,self._allow_unknown)
                    if key!=self._key:
                        support=center_support(self._world,*center,body_length_m=self._body[0],
                            body_width_m=self._body[1],minimum_clearance_m=self._minimum_clearance,allow_unknown=self._allow_unknown,backend=self._backend)
                        # 明确 -1/0/1 -> 0/1/2，不能直接强转 intrinsic。
                        states=np.where(support.cells==-1,0,np.where(support.cells==0,1,2)).astype(np.uint8)
                        self._revision+=1
                        grid=GridMap(states,.01,support.origin,revision=self._revision,
                                     epoch=str(self._world.epoch),eligible=support.eligible)
                        self._key=key
                    else:grid=self._receipt.grid
                    with self._condition:
                        self._receipt=MapReceipt(grid,self._valid_received,monotonic())
                self.error=None
            except MissingTransform as exc:
                self.error=str(exc)  # 保留待转换消息，下一次 TF 到达再试。
            except Exception as exc:
                self.error=str(exc)
                with self._condition:
                    if self._pending is pending:self._pending=None
            finally:
                with self._condition:
                    self._completed=generation;self._condition.notify_all()

    def close(self):
        with self._condition:self._closed=True;self._condition.notify_all()
        self._worker.join()
        terrain=getattr(self._store,'_terrain',None)
        if terrain is not None:terrain.close()


def create_p3_source(capability,body_dimensions,*,minimum_clearance_m=None,allow_unknown=False,map_frame='map'):
    """生产始终显式选择已安装 native ABI，不加载 planner/session。"""
    from pathlib import Path
    from ament_index_python.packages import get_package_prefix
    from nav1cm.field_native import NativeDistanceField
    from nav1cm.p3_map import P3MapStore
    from nav1cm.terrain_native import WheelTerrainCapability
    prefix=Path(get_package_prefix('lunar_nav1cm_planner'))
    store=P3MapStore(library_path=prefix/'lib/libnav1cm_terrain.so',
                     capability=WheelTerrainCapability.from_mapping(capability))
    try:
        backend=NativeDistanceField(prefix/'lib/libnav1cm_distance_field.so')
        return P3MapSource(store,backend,body_dimensions,
            minimum_clearance_m=resolve_minimum_clearance(capability.get("minimum_clearance_m",0.0),minimum_clearance_m),
            allow_unknown=allow_unknown,map_frame=map_frame)
    except Exception:
        store._terrain.close()
        raise
