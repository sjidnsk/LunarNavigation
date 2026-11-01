from pathlib import Path
import argparse,os,json,hashlib,math,datetime

parser=argparse.ArgumentParser(description='从月球极区 v02 源高程生成三区标注与选区图，不运行仿真。')
parser.add_argument('--regions',type=Path,default=Path(__file__).resolve().parents[2]/'docs/simulation/isaac-polar-v02-midterm-regions.json',help='区域定义 JSON')
parser.add_argument('--output-dir',type=Path,required=True,help='明确指定的独立产物目录；已有结果不会覆盖')
args=parser.parse_args()
root=args.output_dir.resolve()
config_bytes=args.regions.resolve().read_bytes()
cfg=json.loads(config_bytes)
if cfg.get('schema')!='lunar.isaac-polar-midterm-regions.v1':
    parser.error('不支持的区域定义 schema')
for name in ['region-selection.png','Midterm_Polar_V02_Regions.usda','preparation-evidence.json','README.md']:
    assert not (root/name).exists(), '拒绝覆盖已有产物: '+name
config_copy=root/'isaac-polar-v02-midterm-regions.json'
if config_copy.exists() and config_copy.read_bytes()!=config_bytes:
    parser.error('产物目录已有不同的区域定义，拒绝覆盖')
root.mkdir(parents=True,exist_ok=True)
if not config_copy.exists():
    config_copy.write_bytes(config_bytes)
import numpy as np
regions=cfg['regions'];assert [r['id'] for r in regions]==['A','B','C']
for r in regions:
    x0,x1,y0,y1=r['bounds_xy_m'];p=np.asarray(r['polygon_xy_m'])
    area=.5*float(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1)))
    assert x0<x1 and y0<y1 and min(x0,y0)>=-500 and max(x1,y1)<=500
    assert area==r['area_m2']==(x1-x0)*(y1-y0) and r['size_xy_m']==[x1-x0,y1-y0]
    assert r['start_pose'] is None and r['goal_pose'] is None
for i,a in enumerate(regions):
    for b in regions[i+1:]:
        a0,a1,a2,a3=a['bounds_xy_m'];b0,b1,b2,b3=b['bounds_xy_m']
        assert a1<=b0 or b1<=a0 or a3<=b2 or b3<=a2
sx,sy=regions[2]['spawn_candidate_xy_m'];x0,x1,y0,y1=regions[2]['bounds_xy_m'];assert x0<sx<x1 and y0<sy<y1
scene=Path(cfg['source']['rover_entry']);scene_sha=hashlib.sha256(scene.read_bytes()).hexdigest();assert scene_sha==cfg['source']['rover_entry_sha256']
checkpoints=[]
for key in ['checkpoint','training_copy']:
    p=Path(cfg['exploration_policy'][key]);sha=hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha==cfg['exploration_policy']['sha256'] and p.stat().st_size==cfg['exploration_policy']['bytes']
    checkpoints.append({'path':str(p),'sha256':sha,'bytes':p.stat().st_size})
data=Path(cfg['source']['terrain_data_directory']);h=np.load(data/'heightfield_m.npy',mmap_mode='r');assert h.shape==(2001,2001)
patches=[]
for p in json.loads((data/'detail_patches.json').read_text()):
    p=dict(p);p['height']=np.load(data/(p['name']+'_height.npy'),mmap_mode='r');patches.append(p)
def sample(a,x,y,x0,y0,step):
    fx=float(np.clip((x-x0)/step,0,a.shape[1]-1));fy=float(np.clip((y-y0)/step,0,a.shape[0]-1))
    ix=int(fx);iy=int(fy);jx=min(ix+1,a.shape[1]-1);jy=min(iy+1,a.shape[0]-1);u=fx-ix;v=fy-iy
    return float(a[iy,ix]*(1-u)*(1-v)+a[iy,jx]*u*(1-v)+a[jy,ix]*(1-u)*v+a[jy,jx]*u*v)
def height(x,y):
    for p in patches:
        x0,x1,y0,y1=p['bounds']
        if x0<=x<=x1 and y0<=y<=y1:return sample(p['height'],x,y,x0,y0,p['step'])
    return sample(h,x,y,-500,-500,.5)
merged=np.array(h,copy=True)
for p in patches:
    x0,x1,y0,y1=p['bounds'];z=p['height'][::4,::4];iy=round((y0+500)/.5);ix=round((x0+500)/.5)
    merged[iy:iy+z.shape[0],ix:ix+z.shape[1]]=z
os.environ['MPLCONFIGDIR']=str(root/'matplotlib-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Rectangle
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');matplotlib.rcParams['axes.unicode_minus']=False
fig=plt.figure(figsize=(14,9),constrained_layout=True);gs=fig.add_gridspec(3,2,width_ratios=[1.4,1]);ax=fig.add_subplot(gs[:,0])
im=ax.imshow(merged[::2,::2],origin='lower',extent=[-500,500,-500,500],cmap='terrain',vmin=-36,vmax=41,interpolation='nearest')
ax.set_title('月球极区 v02：中期演示区域',fontproperties=font,fontsize=17);ax.set_xlabel('World X / m');ax.set_ylabel('World Y / m');ax.set_aspect('equal');ax.grid(alpha=.2)
stats=[]
for i,r in enumerate(regions):
    x0,x1,y0,y1=r['bounds_xy_m'];colour=r['colour_rgb']
    ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor=colour,lw=2.5))
    ax.text(x0,y1+13,r['id']+'  '+r['name'],fontproperties=font,color='#141414',fontsize=10,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none','pad':2})
    a=fig.add_subplot(gs[i,1]);z=merged[round((y0+500)/.5):round((y1+500)/.5)+1,round((x0+500)/.5):round((x1+500)/.5)+1]
    a.imshow(z,origin='lower',extent=[x0,x1,y0,y1],cmap='terrain',vmin=-36,vmax=41,interpolation='nearest');a.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,edgecolor=colour,lw=3))
    a.set_title(r['id']+'  '+r['name']+'  '+str(x1-x0)+' × '+str(y1-y0)+' m',fontproperties=font,fontsize=11)
    a.set_xlabel('World X / m',fontsize=8);a.set_ylabel('World Y / m',fontsize=8);a.tick_params(labelsize=8);a.set_aspect('equal')
    patch=next((p for p in patches if p['bounds']==r['bounds_xy_m']),None);arr=np.asarray(patch['height']) if patch else z;step=patch['step'] if patch else .5
    gy,gx=np.gradient(arr,step);sl=np.rad2deg(np.arctan(np.hypot(gx,gy)))
    stats.append({'id':r['id'],'bounds_xy_m':r['bounds_xy_m'],'area_m2':r['area_m2'],'geometry_sample_spacing_m':step,'height_range_m':[float(arr.min()),float(arr.max())],'geometric_slope_percentiles_deg':{str(q):float(np.percentile(sl,q)) for q in [50,90,95,99]},'geometric_fraction_over20deg':float(np.mean(sl>20)),'meaning':'源高程几何统计，未计岩石车体包络，非通行验收'})
    if r['id']=='C':
        for pa in [ax,a]:pa.plot(sx,sy,marker='*',ms=10,color='#d94665',markeredgecolor='white',markeredgewidth=.8)
        a.text(sx+7,sy,'候选出生点',fontproperties=font,fontsize=8,color='#101010',bbox={'facecolor':'white','alpha':.7,'edgecolor':'none'})
bar=fig.colorbar(im,ax=ax,orientation='horizontal',shrink=.85,pad=.05);bar.set_label('源地形高程 / m',fontproperties=font)
fig.suptitle('选区准备图 · 非车辆运行或算法效果证据',fontproperties=font,fontsize=13,color='#555555')
fig.savefig(root/'region-selection.png',dpi=180,bbox_inches='tight');plt.close(fig)
lines=['#usda 1.0','(','    defaultPrim = "World"','    metersPerUnit = 1','    upAxis = "Z"','    subLayers = [@'+str(scene)+'@]','    customLayerData = {','        string task_id = "ISAAC-POLAR-MIDTERM-20261028"','        string preparation_status = "PREPARATION_ONLY"','        string note = "Visual annotations only; no task enforcement, collision, autoplay, or motion commands."','    }',')','','over "World"','{','    def Scope "MidtermPreparation"','    {','        custom string status = "PREPARATION_ONLY"'];cameras={}
for r,s in zip(regions,stats):
    x0,x1,y0,y1=r['bounds_xy_m'];corners=[(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)];pts=[]
    for start,end in zip(corners,corners[1:]):
        count=int(math.ceil(math.dist(start,end)))
        for j in range(count):
            u=j/count;x=start[0]+(end[0]-start[0])*u;y=start[1]+(end[1]-start[1])*u;pts.append((x,y,height(x,y)+cfg['presentation']['boundary_clearance_m']))
    pts.append(pts[0]);colour=', '.join(str(v) for v in r['colour_rgb'])
    lines += ['        def BasisCurves "'+r['usd_name']+'"','        {','            uniform token purpose = "guide"','            uniform token type = "linear"','            uniform token wrap = "nonperiodic"','            int[] curveVertexCounts = ['+str(len(pts))+']','            point3f[] points = ['+', '.join('('+', '.join(format(v,'.6f') for v in p)+')' for p in pts)+']','            float[] widths = ['+str(cfg['presentation']['boundary_width_m'])+'] (interpolation = "constant")','            color3f[] primvars:displayColor = [('+colour+')] (interpolation = "constant")','            custom string label = '+json.dumps(r['name'],ensure_ascii=False),'            custom string role = "visual_annotation_only"','        }']
    xyz=[(x0+x1)/2,(y0+y1)/2,s['height_range_m'][1]+r['top_camera_height_above_region_max_m']];cameras[r['id']]=xyz
    lines += ['        def Camera "'+r['id']+'_Top"','        {','            float focalLength = 24','            float horizontalAperture = 36','            float verticalAperture = 20.25','            float2 clippingRange = (0.1, 10000)','            double3 xformOp:translate = ('+', '.join(format(v,'.6f') for v in xyz)+')','            uniform token[] xformOpOrder = ["xformOp:translate"]','            custom string role = "recording_view_proposal"','        }']
lines += ['        def Sphere "C_SpawnCandidate"','        {','            uniform token purpose = "guide"','            double radius = 1.2','            double3 xformOp:translate = ('+f'{sx}, {sy}, {height(sx,sy)+1.7:.6f}'+')','            uniform token[] xformOpOrder = ["xformOp:translate"]','            color3f[] primvars:displayColor = [(0.85, 0.18, 0.35)]','            custom string role = "spawn_candidate_not_physically_validated"','        }','    }','}']
(root/'Midterm_Polar_V02_Regions.usda').write_text('\n'.join(lines)+'\n',encoding='utf-8')
readme='''# 月球极区中期演示准备包

这是区域与取景准备，不是算法运行结果。A/B 使用已知地图规划；C 用 block-19，从局部观测开始探索。

- region-selection.png：由 v02 高程数据绘制的世界坐标选区图。
- Midterm_Polar_V02_Regions.usda：引用台式完整月球车入口的独立标注场景；新增三区边界、三台顶视相机与候选出生点标记。
- isaac-polar-v02-midterm-regions.json：区域、检查点、身份和地图口径。
- preparation-evidence.json：哈希、几何统计与静态验证边界。
- 生成与预览源码：仓库 scripts/isaac_midterm/，使用明确的区域文件和产物目录。

USD 依赖台式上的完整原资源目录，不能只搬运此文件到另一台机器运行。File → Open 可浏览标注；仅打开 USD 不会挂接配套 Python 驱动或 ROS 发布器。当前 GUI 没有被本轮准备接管。

相机位于 /World/MidtermPreparation/A_Top、B_Top、C_Top。边界无碰撞，不执行任务约束。粉色标记只是出生 XY 候选，需核验落位与姿态。

A/B 起终点与已知地图未生成验证；C 在线 v5 适配、实际建图与新规划器接线待完成。真实运动、Humble 闭环、视频录制、Jazzy、Orin 和实车均 NOT_RUN。Inspection 含人工检查补光。参考真值与选区底图不能进入 C 的 Actor 输入。
'''
(root/'README.md').write_text(readme,encoding='utf-8')
paths=[scene,Path(cfg['source']['terrain_entry']),scene.parent.parent/'assets/density_10_percent.usdc',scene.parent.parent/'assets/terrain_display_lod.usdc',data/'heightfield_m.npy',data/'detail_patches.json']+[data/(p['name']+'_height.npy') for p in patches]
sources=[{'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths]
evidence={'task_id':cfg['task_id'],'date':cfg['date'],'timezone':cfg['timezone'],'preparation_status':'GENERATED_PENDING_USD_PARSE','source_rover_sha256':scene_sha,'source_rover_unchanged':hashlib.sha256(scene.read_bytes()).hexdigest()==scene_sha,'source_files':sources,'checkpoint_identity':checkpoints,'regions':stats,'cameras_xyz_m':cameras,'region_json_checks':'PASS: bounds, area, CCW, disjoint regions, spawn containment','image_role':'source_geometry_selection_map_not_runtime_screenshot','usd_parse':'NOT_RUN','usd_full_composition':'NOT_RUN','gui_render':'NOT_RUN','simulation_started_by_this_preparation':False,'motion_commands_sent':False,'generation_environment':{'python':'/usr/bin/python3 -s','numpy':np.__version__,'matplotlib':matplotlib.__version__},'initial_generation_failure':{'cause':'NumPy 2.2.6 user site incompatible with system Matplotlib NumPy-1.x binary','resolution':'Use existing /usr/bin/python3 -s with system NumPy 1.21.5 and Matplotlib 3.5.1; no package or environment modification','retained_existing_file':'isaac-polar-v02-midterm-regions.json'},'verification_boundaries':cfg['execution_status']}
evidence['executed_at']=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()
assert evidence['source_rover_unchanged'];(root/'preparation-evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'status':'PREPARATION_ARTIFACTS_GENERATED','directory':str(root),'files':[p.name for p in root.iterdir() if p.is_file()],'region_statistics':stats},ensure_ascii=False))
