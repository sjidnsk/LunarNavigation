"""离线Smac2D对照：调用官方原生可执行文件，不启动导航或车辆。"""
import argparse
import json
import math
import subprocess
import time
from pathlib import Path
import numpy as np


def rasterize(request, resolution, bounds):
    """障碍距离按矩形连续几何计算；额外整格对角线覆盖相邻中心连线。"""
    x0,y0,x1,y1=bounds
    width,height=math.ceil((x1-x0)/resolution),math.ceil((y1-y0)/resolution)
    if width*height>25_000_000:
        raise ValueError('grid exceeds 25 million cells; choose explicit coarser resolution')
    x=x0+(np.arange(width)+.5)*resolution
    y=y0+(np.arange(height)+.5)*resolution
    distance=np.full((height,width),np.inf)
    for rect in request['world']['blocked']:
        dx=np.maximum(np.maximum(rect['x0']/100-x,x-rect['x1']/100),0)
        dy=np.maximum(np.maximum(rect['y0']/100-y,y-rect['y1']/100),0)
        # 避免对每个障碍遍历全图：仅硬半径+软带附近更新。
        margin=.75+.5+math.sqrt(2)*resolution
        ix=np.flatnonzero(dx<=margin);iy=np.flatnonzero(dy<=margin)
        if len(ix) and len(iy):
            index=np.ix_(iy,ix)
            distance[index]=np.minimum(distance[index],np.hypot(dy[iy,None],dx[None,ix]))
    hard=.75+math.sqrt(2)*resolution
    cost=np.rint(252*np.clip((hard+.5-distance)/.5,0,1)).astype(np.uint8)
    cost[distance<=hard]=254
    return cost,(x0,y0)


def run(requests, binary, resolution, bounds):
    records=[]
    # 每个请求独立进程，诚实记录进程启动/栅格化，不冒称持久Smac复用。
    for label,request in requests:
        began=time.perf_counter();grid,origin=rasterize(request,resolution,bounds)
        raster_ms=(time.perf_counter()-began)*1000
        h,w=grid.shape
        def cell(point):
            result=tuple(math.floor((point[i]-origin[i])/resolution) for i in (0,1))
            if not (0<=result[0]<w and 0<=result[1]<h):raise ValueError('endpoint outside explicit bounds')
            return result
        start,goal=cell(request['start']),cell(request['goal'])
        header=f'{w} {h} {resolution} {request.get("budget_s",30)} {start[0]} {start[1]} {goal[0]} {goal[1]}\n'
        payload=header+' '.join(map(str,grid.ravel()))+'\n'
        began=time.perf_counter()
        proc=subprocess.run([str(binary)],input=payload,text=True,capture_output=True,timeout=request.get('budget_s',30)+60,check=True)
        lines=proc.stdout.splitlines();line=next(i for i,s in enumerate(lines) if s.startswith('RESULT '))
        _,ok,search_ms,iterations,count=lines[line].split()
        points=[tuple(origin[i]+(float(value)+.5)*resolution for i,value in enumerate(s.split())) for s in lines[line+1:line+1+int(count)]]
        records.append(dict(label=label,engine='official_humble_smac_AStarAlgorithm_Node2D',success=ok=='1',search_ms=float(search_ms),iterations=int(iterations),raster_ms=raster_ms,process_io_ms=(time.perf_counter()-began)*1000,points=points,resolution=resolution,hard_radius_m=.75,conservative_extra_m=math.sqrt(2)*resolution,soft_band_m=.5,cost_penalty=1,unknown_allowed=True,unknown_cost_mapping='free; graph has no unknown penalty',stderr=proc.stderr))
    return records


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--requests',required=True);parser.add_argument('--binary',required=True);parser.add_argument('--resolution',type=float,default=.1);parser.add_argument('--bounds',nargs=4,type=float,required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();raw=json.loads(Path(args.requests).read_text())
    if isinstance(raw,dict): raw=[{'label':'first','request':raw},{'label':'repeat','request':raw}]
    records=run([(item['label'],item['request']) for item in raw],args.binary,args.resolution,args.bounds)
    Path(args.output).write_text(json.dumps(records,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
