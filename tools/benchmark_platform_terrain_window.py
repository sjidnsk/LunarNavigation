"""平台尺度地形原生分类负载；不发布ROS消息或控制。"""
import argparse
from dataclasses import replace
import json,math,resource,time
from pathlib import Path
import numpy as np
from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain,load_wheel_capability


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--length',type=float,required=True)
    p.add_argument('--resolution',type=float,required=True)
    p.add_argument('--config',required=True)
    p.add_argument('--library',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--legacy',action='store_true')
    p.add_argument('--capture')
    a=p.parse_args();cap=replace(load_wheel_capability(a.config),maximum_slope_rad=math.radians(20))
    if a.legacy:
        if cap.platform_kind!='wheel':raise ValueError('legacy bridge cannot represent legged semantics')
        cap=replace(cap,body_long_side_m=None)
    if a.capture:
        captured=np.load(a.capture);h=np.ascontiguousarray(captured['elevation'],dtype=np.float32)
    else:
        N=round(a.length/a.resolution);q=(np.arange(N)+.5)*a.resolution-a.length/2
        x,y=np.meshgrid(q,q);h=(.025*x+.003*np.sin(2*x+3*y)).astype(np.float32)
        h[((x-2.3)**2+(y-1.2)**2<.85**2)|((x+3.1)**2+(y+2.1)**2<1.05**2)]+=.5
        h[(abs(x)>a.length*.42)|(abs(y)>a.length*.42)]=np.nan
    rows=[]
    with NativeTerrain(a.library,capability=cap) as terrain:
        for trial in range(2):
            if trial:h[h.shape[0]//2,h.shape[1]//2]+=.001
            t=time.perf_counter();update=terrain.apply(h,resolution_m=a.resolution,origin_xy_m=(0.,0.),yaw=0.)
            elapsed=(time.perf_counter()-t)*1000
            states=np.full(h.shape,-1,np.int8)
            for x0,y0,x1,y1,value in update.regions:
                states[max(0,y0):min(h.shape[0],y1),max(0,x0):min(h.shape[1],x1)]=value
            rows.append(dict(trial=trial,apply_ms=elapsed,regions=len(update.regions),counts={str(v):int(np.sum(states==v)) for v in (-1,0,1)}))
    result=dict(platform=cap.platform_kind,long_side_m=cap.body_long_side_m,length_m=a.length,resolution_m=a.resolution,legacy=a.legacy,shape=list(h.shape),finite_cells=int(np.isfinite(h).sum()),rows=rows,rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    Path(a.output).write_text(json.dumps(result,indent=2));print(json.dumps(result))
if __name__=='__main__':main()
