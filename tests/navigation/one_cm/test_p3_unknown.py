"""有限 UNKNOWN 撤销自由支持；不把缺帧或旧源 halo 当作新观测。"""
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
from scripts.navigation.one_cm.nav1cm.terrain_native import TerrainUpdate
from scripts.navigation.one_cm.nav1cm.contracts import CellState, VehicleState
from scripts.navigation.one_cm.nav1cm.maps import extract_window


def message(values, resolution=.01, origin=(0., 0.)):
    values=np.asarray(values,dtype=np.float32); h,w=values.shape
    return NS(header=NS(frame_id='map'), info=NS(resolution=resolution,length_x=w*resolution,
        length_y=h*resolution,pose=NS(position=NS(x=origin[0]+w*resolution/2,
        y=origin[1]+h*resolution/2,z=0.),orientation=None)),layers=['elevation'],
        data=[NS(data=values[::-1,::-1].ravel(),layout=None)],outer_start_index=0,inner_start_index=0)


class Terrain:
    def __init__(self): self.state=0
    def apply(self,elevation_m,*,resolution_m,origin_xy_m,yaw,z_offset_m):
        h,w=elevation_m.shape
        return TerrainUpdate(resolution_m,origin_xy_m,yaw,np.array([[0,0,w,h,self.state]],dtype=np.int64))


def state_at(world,x,y):
    x,y=math.floor(x*100),math.floor(y*100)
    for boxes,state in ((world.blocked,1),(world.known_free,0)):
        if any(r.x0<=x<r.x1 and r.y0<=y<r.y1 for r in boxes):return state
    return -1


@pytest.mark.parametrize('resolution,origin',[(.01,(0.,0.)),(.05,(0.,0.)),(.005,(.003,.003))])
def test_finite_unknown_withdraws_free_and_changes_epoch(resolution,origin):
    terrain=Terrain();store=P3MapStore(terrain=terrain);data=np.zeros((20,20))
    before=store.apply(message(data,resolution,origin)); assert before.known_free
    terrain.state=-1
    after=store.apply(message(data+.4,resolution,origin))
    assert not after.known_free
    assert after.epoch==before.epoch+1
    window=extract_window(after,VehicleState(.05,.05,0.,0.,0.),False)
    assert np.all(window.cells==CellState.UNKNOWN)


@pytest.mark.parametrize('initial',[0,1])
def test_no_observation_and_blocked_history_remain(initial):
    terrain=Terrain();terrain.state=initial;store=P3MapStore(terrain=terrain)
    before=store.apply(message(np.zeros((20,20))))
    terrain.state=-1
    assert store.apply(message(np.full((20,20),np.nan))) is before
    after=store.apply(message(np.ones((20,20))))
    if initial==1: assert after is before
    else: assert not after.known_free


@pytest.mark.parametrize('new_resolution,new_origin',[(.02,(0.,0.)),(.01,(.005,.005)),(.005,(.003,.003))])
def test_stale_unknown_halo_cannot_revoke_newer_source(new_resolution,new_origin):
    terrain=Terrain();store=P3MapStore(terrain=terrain)
    store.apply(message(np.zeros((40,40))))
    store.apply(message(np.zeros((10,10)),new_resolution,new_origin))
    terrain.state=-1;neighbor=np.full((40,40),np.nan);neighbor[30,30]=.4
    after=store.apply(message(neighbor))
    assert state_at(after,.035,.035)==0
    assert state_at(after,.305,.305)==-1


def test_partial_direct_unknown_revokes_whole_free_centimeter():
    terrain=Terrain();store=P3MapStore(terrain=terrain)
    store.apply(message(np.zeros((20,20))))
    terrain.state=-1
    after=store.apply(message(np.ones((1,1)),.003,(.012,.012)))
    assert state_at(after,.015,.015)==-1
    assert state_at(after,.025,.015)==0


def test_same_source_unknown_halo_revokes_unobserved_neighbor():
    terrain=Terrain();store=P3MapStore(terrain=terrain)
    store.apply(message(np.zeros((20,20))))
    terrain.state=-1;data=np.full((20,20),np.nan);data[10,10]=.4
    after=store.apply(message(data))
    assert state_at(after,.055,.055)==-1
