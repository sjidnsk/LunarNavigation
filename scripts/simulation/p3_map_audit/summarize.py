"""对同帧生产重放按距离统计，格数与面积同时保留；不发布ROS消息。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

EDGES=np.array([0.,2.,5.,10.,15.,20.,30.,40.,float('inf')])

def grouped(x,y,mask,resolution,pose):
    radius=np.hypot(x-pose[0],y-pose[1])
    count=np.histogram(radius[mask],bins=EDGES)[0]
    return [dict(count=int(n),area_m2=float(n*resolution**2)) for n in count]

def native_grid_counts(rectangles,pose,bounds):
    counts=np.zeros(len(EDGES)-1,dtype=np.int64)
    for rect in rectangles:
        x0=max(rect.x0,math.ceil(bounds[0]/.01-1e-7));x1=min(rect.x1,math.floor(bounds[2]/.01+1e-7))
        y0=max(rect.y0,math.ceil(bounds[1]/.01-1e-7));y1=min(rect.y1,math.floor(bounds[3]/.01+1e-7))
        if x1<=x0 or y1<=y0:continue
        xs=(np.arange(x0,x1)+.5)*.01-pose[0]
        for start in range(y0,y1,64):
            ys=(np.arange(start,min(y1,start+64))+.5)*.01-pose[1]
            counts+=np.histogram(np.hypot(ys[:,None],xs[None,:]),bins=EDGES)[0]
    return [dict(count=int(n),area_m2=float(n*.0001)) for n in counts]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('frame');parser.add_argument('--source',required=True);parser.add_argument('--wheel',required=True);parser.add_argument('--terrain-library',required=True)
    args=parser.parse_args();p=Path(args.frame);source=Path(args.source)
    from rclpy.serialization import deserialize_message
    from grid_map_msgs.msg import GridMap
    import yaml
    sys.path.insert(0,str(source/'scripts/navigation/one_cm'))
    from nav1cm.p3_map import P3MapStore,_read_message
    meta=json.loads((p/'capture.json').read_text());pose=meta['pose']
    message=deserialize_message((p/'map.cdr').read_bytes(),GridMap)
    elevation,r,origin,yaw,z=_read_message(message,None)
    if abs(yaw)>1e-10:raise RuntimeError('audit common square footprint requires axis aligned observed map')
    if meta['odom_minus_map_s']!=0.:raise RuntimeError('this evidence requires exact source timestamp match')
    a=np.genfromtxt(p/'source.csv',delimiter=',',names=True)
    assert len(a)==elevation.size and np.array_equal(a['finite'].astype(bool),np.isfinite(elevation).ravel()), 'native/Python source decoding mismatch'
    expected_x=origin[0]+(np.arange(elevation.shape[1])+.5)*r
    expected_y=origin[1]+(np.arange(elevation.shape[0])+.5)*r
    assert np.allclose(a['x'].reshape(elevation.shape),expected_x[None,:],rtol=0,atol=1e-8)
    assert np.allclose(a['y'].reshape(elevation.shape),expected_y[:,None],rtol=0,atol=1e-8)
    bounds=[origin[0],origin[1],origin[0]+elevation.shape[1]*r,origin[1]+elevation.shape[0]*r]
    result=dict(capture=meta,distance_bins_m=['0–2','2–5','5–10','10–15','15–20','20–30','30–40','≥40'],
        source_footprint_m=bounds,scope='cold replay of one already fused P3 frame; not historical online map; source cell centers define radial bins',layers={},checks={'native_python_decoding_identical':True,'exact_map_odom_stamp':True},wheel_sha256=hashlib.sha256(Path(args.wheel).read_bytes()).hexdigest())
    def add(name,table,mask,res):result['layers'][name]=dict(resolution_m=res,bins=grouped(table['x'],table['y'],mask,res,pose))
    add('source_all',a,np.ones(len(a),bool),r)
    for name,mask in [('source_finite',a['finite']==1),('source_intrinsic_free',a['intrinsic']==1),('source_intrinsic_blocked',a['intrinsic']==2),('source_finite_incomplete_unknown',(a['finite']==1)&(a['complete']==0)&(a['intrinsic']==0))]:add(name,a,mask,r)
    limits=yaml.safe_load(Path(args.wheel).read_text())['capability']
    for name,col,key in [('slope','slope','maximum_slope_rad'),('relief','relief','maximum_local_obstacle_relief_m'),('rise','rise','minimum_underbody_clearance_m')]:add('source_exceeds_'+name,a,(a['finite']==1)&(a[col]>limits[key]),r)
    f=np.genfromtxt(p/'fine.csv',delimiter=',',names=True);fr=float(f['resolution'][0])
    for name,mask in [('exploration_fine_free',f['state']==1),('exploration_fine_blocked',f['state']==2),('exploration_fine_intrinsic_free',f['intrinsic']==1),('intrinsic_free_lost_to_unknown_support',(f['intrinsic']==1)&(f['state']==0)),('intrinsic_free_lost_to_blocked_envelope',(f['intrinsic']==1)&(f['state']==2))]:add(name,f,mask,fr)
    for csv in sorted(p.glob('coarse-*.csv')):
        c=np.genfromtxt(csv,delimiter=',',names=True);cr=float(c['resolution'][0]);inside=(c['x']>=bounds[0])&(c['x']<bounds[2])&(c['y']>=bounds[1])&(c['y']<bounds[3])
        for name,mask in [('free',c['state']==0),('blocked',c['state']==100),('unknown',c['state']==-1)]:add(f'exploration_coarse_{cr:g}m_'+name,c,mask&inside,cr)
    capability=yaml.safe_load(Path(args.wheel).read_text())
    store=P3MapStore(library_path=args.terrain_library,capability=capability);world=store.apply(message)
    for name,rects in [('nav1cm_intrinsic_free',world.known_free),('nav1cm_intrinsic_blocked',world.blocked)]:result['layers'][name]=dict(resolution_m=.01,bins=native_grid_counts(rects,pose,bounds))
    result['checks']['nav_terrain_library_sha256']=hashlib.sha256(Path(args.terrain_library).read_bytes()).hexdigest()
    result['checks']['finite_partition']=int((a['finite']==1).sum())==int(((a['intrinsic']==1)|(a['intrinsic']==2)|((a['finite']==1)&(a['intrinsic']==0))).sum())
    result['checks']['same_resolution_projection_free_count']=sum(b['count'] for b in result['layers']['exploration_coarse_0.2m_free']['bins'])==sum(b['count'] for b in result['layers']['exploration_fine_free']['bins'])
    if not all(v for v in result['checks'].values()):raise RuntimeError('cross-layer accounting inconsistency')
    for layer in result['layers'].values():
        layer['total_count']=sum(b['count'] for b in layer['bins']);layer['total_area_m2']=sum(b['area_m2'] for b in layer['bins'])
    (p/'summary.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({k:dict(count=v['total_count'],area=v['total_area_m2']) for k,v in result['layers'].items()},indent=2))
if __name__=='__main__':main()
