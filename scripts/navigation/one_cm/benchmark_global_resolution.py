"""长距离复杂几何场景的分辨率对照；不替代P3原生地图/传输测试。"""
import argparse
import json
import math
from pathlib import Path
import platform
import time
from types import SimpleNamespace

from scripts.navigation.one_cm.benchmark_global import validate
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.global_worker import make_request, run_request, restore_result


def scene(length, count):
    # 封闭长廊内交错挡墙，4m门宽；0.75m碰撞圆可通过，不能从外侧绕开。
    boxes=[(-1.,-1.,length+1.,0.),(-1.,12.,length+1.,13.),
           (-1.,0.,0.,12.),(length,0.,length+1.,12.)]
    for i in range(count):
        x=7.13+(length-14.26)*i/max(1,count-1)
        boxes.append((x,0.,x+.37,8.03) if i%2==0 else (x,3.97,x+.37,12.))
    return boxes,(2.,6.),(length-2.,6.)


def quantize(boxes, resolution_cm):
    q=resolution_cm
    # 同一解析障碍保守栅格化；保留合并矩形，不声称模拟稠密栅格处理开销。
    return tuple(CellRect(math.floor(x0*100/q)*q,math.floor(y0*100/q)*q,
                          math.ceil(x1*100/q)*q,math.ceil(y1*100/q)*q)
                 for x0,y0,x1,y1 in boxes)


def run(output, repeats):
    rows=[]
    limits=WheelLimits(motion_model='differential_drive')
    for length,count in ((100,6),(300,16),(750,30),(180,30)):
        boxes,start,goal=scene(length,count)
        for resolution in (20,10,5):
            world=WorldSnapshot(1,quantize(boxes,resolution),())
            for repeat in range(repeats):
                request=make_request(world,start,goal,1,limits,.75,1.,reference_smoothing=True,
                                     global_soft_clearance_m=.4,global_clearance_weight=1.,terminal_yaw=0.)
                began=time.perf_counter()
                raw=run_request(request)
                wall_ms=(time.perf_counter()-began)*1000
                result=restore_result(raw)
                evidence=validate(world,start,goal,SimpleNamespace(**result),expected_version=1)
                shaped=result.get('shaped_reference')
                if shaped is not None:
                    ref=dict(result,route=shaped.route)
                    shaped_evidence=validate(world,start,goal,SimpleNamespace(**ref),expected_version=1)
                else:
                    shaped_evidence=None
                row={'length_m':length,'baffles':count,'resolution_cm':resolution,'repeat':repeat,
                     'blocked_rectangles':len(world.blocked),'equivalent_dense_cells':math.ceil(length*100/resolution)*math.ceil(1200/resolution),
                     'wall_ms':wall_ms,'worker_compute_ms':result['worker_compute_ms'],
                     'timing':result['timing'],'smoothing_ms':result['smoothing_ms'],
                     'smoothing_budget_s':result['smoothing_budget_s'],
                     'reason_code':result['reason_code'],'raw_validation':evidence,
                     'smoothing_reason':shaped.reason if shaped else None,'reference_validation':shaped_evidence}
                rows.append(row)
                Path(output).write_text(json.dumps({'platform':platform.platform(),'budget_s':1.,
                    'scope':'synthetic conservative geometric quantization; sparse rectangles; not P3 native resolution replay',
                    'rows':rows},indent=2),encoding='utf-8')
                print(json.dumps(row),flush=True)
    return rows


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--repeats',type=int,default=3)
    args=parser.parse_args()
    if args.repeats<1:parser.error('repeats must be positive')
    run(args.output,args.repeats)
