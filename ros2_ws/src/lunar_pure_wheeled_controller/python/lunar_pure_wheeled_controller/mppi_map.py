"""Revision-checked fine-map cache and continuous cell-area certificates.

Fine states already include the vehicle footprint. Only the execution margin
is applied here. Unknown is never silently converted to measured free space.
"""
from dataclasses import dataclass
import math
import numpy as np


def _segment_box_distance2(a, b, lo, hi):
    """Exact distance of closed segments to axis-aligned boxes, vectorized."""
    d=b-a
    safe=np.where(abs(d)>1e-15,d,1.)
    u=(lo-a)/safe;v=(hi-a)/safe
    enter=np.max(np.where(abs(d)>1e-15,np.minimum(u,v),-np.inf),axis=-1)
    leave=np.min(np.where(abs(d)>1e-15,np.maximum(u,v),np.inf),axis=-1)
    intersects=(np.maximum(enter,0.)<=np.minimum(leave,1.)) & np.all((abs(d)>1e-15)|((a>=lo)&(a<=hi)),axis=-1)
    dist=np.minimum(np.sum(np.maximum(np.maximum(lo-a,a-hi),0.)**2,axis=-1),
                    np.sum(np.maximum(np.maximum(lo-b,b-hi),0.)**2,axis=-1))
    length=np.sum(d*d,axis=-1)
    for xx,yy in ((0,0),(0,1),(1,0),(1,1)):
        corner=np.stack((hi[...,0] if xx else lo[...,0],hi[...,1] if yy else lo[...,1]),axis=-1)
        t=np.clip(np.sum((corner-a)*d,axis=-1)/np.maximum(length,1e-30),0.,1.)
        dist=np.minimum(dist,np.sum((corner-a-t[...,None]*d)**2,axis=-1))
    return np.where(intersects,0.,dist)


@dataclass(frozen=True)
class GridMap:
    states: np.ndarray
    resolution: float
    origin: tuple
    costs: np.ndarray | None = None
    revision: int = 1
    assumed: np.ndarray | None = None
    epoch: str = ''
    stationary_anchor: tuple | None = None

    def __post_init__(self):
        if self.stationary_anchor is not None:
            anchor=tuple(self.stationary_anchor)
            if len(anchor)!=2 or not np.all(np.isfinite(anchor)):
                raise ValueError('invalid native stationary support anchor')
            object.__setattr__(self,'stationary_anchor',anchor)
        s=np.array(self.states,dtype=np.uint8,copy=True)
        if s.ndim!=2 or not s.size or np.any(s>2) or not math.isfinite(self.resolution) or self.resolution<=0 or not np.all(np.isfinite(self.origin)):
            raise ValueError('invalid fine map')
        c=np.zeros_like(s,dtype=float) if self.costs is None else np.array(self.costs,dtype=float,copy=True)
        assumed=np.zeros_like(s,dtype=bool) if self.assumed is None else np.array(self.assumed,dtype=bool,copy=True)
        if c.shape!=s.shape or assumed.shape!=s.shape or np.any(~np.isfinite(c)) or np.any(c<0) or np.any(assumed & (s!=0)):
            raise ValueError('invalid fine map costs/start connections')
        for name,val in [('states',s),('costs',c),('assumed',assumed)]:
            val.flags.writeable=False;object.__setattr__(self,name,val)

    def _indices(self, points):
        return np.floor((np.asarray(points)-self.origin)/self.resolution).astype(np.int64)

    def sample(self, points):
        idx=self._indices(points);h,w=self.states.shape
        inside=(idx[...,0]>=0)&(idx[...,0]<w)&(idx[...,1]>=0)&(idx[...,1]<h)
        x=np.clip(idx[...,0],0,w-1);y=np.clip(idx[...,1],0,h-1)
        return np.where(inside,self.states[y,x],0),np.where(inside,self.costs[y,x],0.),np.where(inside,self.assumed[y,x],False)

    def valid_points(self,points,margin,allow_assumed=True):
        """Fast proposal filter; final motion still needs the arc certificate."""
        points=np.asarray(points);shape=points.shape[:-1];p=points.reshape(-1,2)
        allow=np.broadcast_to(allow_assumed,shape).reshape(-1)
        reach=int(np.ceil(margin/self.resolution))+1
        oy,ox=np.mgrid[-reach:reach+1,-reach:reach+1]
        offsets=np.stack((ox.ravel(),oy.ravel()),axis=-1)
        cells=self._indices(p)[:,None,:]+offsets
        ix,iy=cells[...,0],cells[...,1];h,w=self.states.shape
        inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
        x=np.clip(ix,0,w-1);y=np.clip(iy,0,h-1)
        free=inside&((self.states[y,x]==1)|self.assumed[y,x])
        lo=cells*self.resolution+self.origin
        delta=np.maximum(np.maximum(lo-p[:,None,:],p[:,None,:]-lo-self.resolution),0.)
        distance=np.sum(delta*delta,axis=-1)
        hit=distance<=1e-20 if margin==0 else distance+1e-14<margin*margin
        # Native-approved start cells remain clearance support after entry.
        # Phase authority applies to the center, independently of its halo.
        _,_,center_assumed=self.sample(p)
        return (~np.any(~free&hit,axis=1)&(allow|~center_assumed)).reshape(shape)

    def valid_segments(self, a, b, margin, allow_assumed=True):
        a,b=np.broadcast_arrays(np.asarray(a,dtype=float),np.asarray(b,dtype=float))
        shape=a.shape[:-1];aa=a.reshape(-1,2);bb=b.reshape(-1,2)
        radius=np.broadcast_to(np.asarray(margin,dtype=float),shape).reshape(-1)
        allowed=np.broadcast_to(np.asarray(allow_assumed,dtype=bool),shape).reshape(-1)
        if np.any(radius<0) or np.any(~np.isfinite(radius)):raise ValueError('invalid margin')
        valid=np.all(np.isfinite(aa)&np.isfinite(bb),axis=1)
        if not np.all(valid):return np.zeros(shape,dtype=bool)
        count=max(1,int(np.ceil(np.max(np.linalg.norm(bb-aa,axis=1),initial=0.)/(self.resolution*.5))))
        if count>4096:raise ValueError('segment certificate exceeds bounded local range')
        reach=int(np.ceil(float(radius.max())/self.resolution))+1
        oy,ox=np.mgrid[-reach:reach+1,-reach:reach+1]
        offsets=np.stack((ox.ravel(),oy.ravel()),axis=-1);h,w=self.states.shape
        first_free=np.full(len(aa),np.inf);last_assumed=np.full(len(aa),-np.inf)
        check_prefix=bool(np.any(self.assumed))
        for j in range(count):
            start=aa+(bb-aa)*(j/count);end=aa+(bb-aa)*((j+1)/count)
            cells=self._indices((start+end)*.5)[:,None,:]+offsets[None]
            ix,iy=cells[...,0],cells[...,1]
            inside=(ix>=0)&(ix<w)&(iy>=0)&(iy<h)
            x=np.clip(ix,0,w-1);y=np.clip(iy,0,h-1)
            free=inside&((self.states[y,x]==1)|self.assumed[y,x])
            row,col=np.nonzero(valid[:,None]&~free)
            if len(row):
                lo=cells[row,col]*self.resolution+self.origin
                dist=_segment_box_distance2(start[row],end[row],lo,lo+self.resolution)
                hit=np.where(radius[row]==0,dist<=1e-20,dist+1e-14<radius[row]**2)
                valid[row[hit]]=False
            if check_prefix:
                row,col=np.nonzero(valid[:,None]&inside)
                lo=cells[row,col]*self.resolution+self.origin;hi=lo+self.resolution
                d=bb[row]-aa[row];nonzero=abs(d)>1e-15;safe=np.where(nonzero,d,1.)
                u=(lo-aa[row])/safe;v=(hi-aa[row])/safe
                enter=np.maximum(0.,np.max(np.where(nonzero,np.minimum(u,v),-np.inf),axis=-1))
                leave=np.minimum(1.,np.min(np.where(nonzero,np.maximum(u,v),np.inf),axis=-1))
                touch=(enter<=leave)&np.all(nonzero|((aa[row]>=lo)&(aa[row]<=hi)),axis=-1)
                free_contact=touch&(self.states[y[row,col],x[row,col]]==1)
                assumed_contact=touch&self.assumed[y[row,col],x[row,col]]
                np.minimum.at(first_free,row[free_contact],enter[free_contact])
                np.maximum.at(last_assumed,row[assumed_contact],leave[assumed_contact])
                valid[row[assumed_contact & ~allowed[row]]]=False
        # Canonical order of grid visitation is irrelevant: compare actual
        # clipped contact intervals, including closed boundary/corner contacts.
        valid &= last_assumed<=first_free+1e-12
        return valid.reshape(shape)

    def valid_trajectories(self, xy, margin):
        state,_,assumed=self.sample(xy)
        entered=np.maximum.accumulate(state==1,axis=1)
        valid=self.valid_segments(xy[:,:-1],xy[:,1:],margin,~entered[:,:-1])
        return np.all(valid,axis=1)&~np.any(entered&assumed,axis=1)

    def valid_motion(self, poses, velocity, dt, margin):
        """Enclose each constant-twist arc by its endpoint tangents.

        The triangle includes the complete arc without spending a uniform
        sagitta budget at a stationary boundary point. Long/ill-conditioned
        arcs are rejected. A cell cannot hide inside these thin triangles.
        """
        a=poses[:,:-1,:2];b=poses[:,1:,:2]
        v,w=velocity[:,:,0],velocity[:,:,1];theta=w*dt
        radius=np.divide(v,w,out=np.zeros_like(v),where=abs(w)>1e-12)
        distance=np.where(abs(w)>1e-12,radius*np.tan(theta*.5),v*dt*.5)
        yaw=poses[:,:-1,2]
        tangent=a+distance[:,:,None]*np.stack((np.cos(yaw),np.sin(yaw)),axis=-1)
        starts=np.stack((a,tangent,a),axis=2);ends=np.stack((tangent,b,b),axis=2)
        state,_,assumed=self.sample(poses[:,:,:2]);entered=np.maximum.accumulate(state==1,axis=1)
        allow=np.broadcast_to((~entered[:,:-1])[:,:,None],starts.shape[:-1])
        valid=self.valid_segments(starts,ends,margin,allow).all(axis=(1,2))
        chord=b-a;length=np.linalg.norm(chord,axis=-1)
        altitude=abs(chord[:,:,0]*(tangent[:,:,1]-a[:,:,1])-chord[:,:,1]*(tangent[:,:,0]-a[:,:,0]))/np.maximum(length,1e-30)
        # With altitude < one cell, a whole square cannot lie inside the
        # triangle unnoticed by its three certified edges.
        valid &= np.all((abs(theta)<.5)&(altitude<self.resolution),axis=1)
        return valid&~np.any(entered&assumed,axis=1)


class PolicyMapCache:
    """Sparse 256x256 wire tiles; apply responses atomically or reject them."""
    def __init__(self):
        self.revision=0;self.epoch=None;self.tiles={};self.connections=set()
        self.resolution=None;self.origin=None;self.frame=None;self.stationary_anchor=None

    def apply(self,r):
        if not r.ready:raise ValueError('map not ready')
        resolution=float(r.resolution_m);origin=(float(r.origin.x),float(r.origin.y))
        if not math.isfinite(resolution) or resolution<=0 or not np.all(np.isfinite(origin)) or not r.frame_id:
            raise ValueError('invalid map geometry')
        if not r.full_snapshot and (self.epoch!=r.epoch or (r.fine_revision!=self.revision and r.base_revision!=self.revision) or r.fine_revision<self.revision or self.resolution!=resolution or self.origin!=origin or self.frame!=r.frame_id):
            raise ValueError('map delta discontinuity')
        tiles={} if r.full_snapshot else dict(self.tiles)
        for t in r.tiles:
            s=np.array(t.states,dtype=np.uint8);c=np.array(t.costs,dtype=float)
            if s.size!=65536 or c.size!=65536 or np.any(s>2) or np.any(~np.isfinite(c)) or np.any(c<0):raise ValueError('malformed map tile')
            tiles[(int(t.tile_x),int(t.tile_y))]=(s.reshape(256,256),c.reshape(256,256))
        if len(r.start_connection_x)!=len(r.start_connection_y):raise ValueError('malformed start connections')
        xs=getattr(r,'assumed_start_x',[]);ys=getattr(r,'assumed_start_y',[])
        if len(xs)!=len(ys):raise ValueError('malformed assumed start prefix')
        stationary_anchor=None
        if getattr(r,'stationary_support_ready',False):
            stationary_anchor=(float(r.anchor_pose.position.x),float(r.anchor_pose.position.y))
            if not np.all(np.isfinite(stationary_anchor)):
                raise ValueError('invalid native stationary support anchor')
        self.tiles=tiles;self.connections=set(zip(xs,ys))
        self.stationary_anchor=stationary_anchor
        self.revision=int(r.fine_revision);self.epoch=str(r.epoch)
        self.resolution=resolution;self.origin=origin;self.frame=r.frame_id

    def local_grid(self,x,y,radius):
        if self.resolution is None:raise ValueError('map unavailable')
        center=np.floor((np.array([x,y])-self.origin)/self.resolution).astype(int)
        reach=int(np.ceil(radius/self.resolution))+2;lo=center-reach;hi=center+reach+1
        width,height=hi-lo;s=np.zeros((height,width),np.uint8);cost=np.zeros_like(s,dtype=float);assumed=np.zeros_like(s,dtype=bool)
        for ty in range(lo[1]//256,(hi[1]-1)//256+1):
            for tx in range(lo[0]//256,(hi[0]-1)//256+1):
                if (tx,ty) not in self.tiles:continue
                a=np.maximum(lo,[tx*256,ty*256]);b=np.minimum(hi,[(tx+1)*256,(ty+1)*256])
                out=np.s_[a[1]-lo[1]:b[1]-lo[1],a[0]-lo[0]:b[0]-lo[0]]
                src=np.s_[a[1]-ty*256:b[1]-ty*256,a[0]-tx*256:b[0]-tx*256]
                ss,cc=self.tiles[(tx,ty)];s[out]=ss[src];cost[out]=cc[src]
        for cx,cy in self.connections:
            if lo[0]<=cx<hi[0] and lo[1]<=cy<hi[1] and s[cy-lo[1],cx-lo[0]]==0:assumed[cy-lo[1],cx-lo[0]]=True
        return GridMap(s,self.resolution,tuple(np.array(self.origin)+lo*self.resolution),cost,self.revision,assumed,self.epoch,self.stationary_anchor)


def map_response_fresh(response, now_ns, timeout_s):
    stamp=int(response.processed_stamp.sec)*1_000_000_000+int(response.processed_stamp.nanosec)
    return 0<=int(now_ns)-stamp<=int(timeout_s*1e9)
