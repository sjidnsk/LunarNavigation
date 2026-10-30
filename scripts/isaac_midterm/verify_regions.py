from pathlib import Path
import argparse,json,hashlib,math,datetime

parser=argparse.ArgumentParser(description='静态核对三区 USD、源身份和取景配置；不启动 Kit 或物理仿真。')
parser.add_argument('--output-dir',type=Path,required=True,help='prepare_regions.py 生成的独立产物目录')
args=parser.parse_args()
root=args.output_dir.resolve()
from pxr import Sdf,Usd,UsdGeom,UsdPhysics
cfg=json.loads((root/'isaac-polar-v02-midterm-regions.json').read_text())
usd=root/'Midterm_Polar_V02_Regions.usda'
text=usd.read_text()
assert text.count('            uniform token type = "linear"')==3
assert text.count('uniform token purpose = "guide"')==4
layer=Sdf.Layer.FindOrOpen(str(usd));assert layer
assert list(layer.subLayerPaths)==[cfg['source']['rover_entry']]
assert layer.customLayerData['preparation_status']=='PREPARATION_ONLY'
source=Sdf.Layer.FindOrOpen(cfg['source']['rover_entry']);assert source
anon=Sdf.Layer.CreateAnonymous('midterm-annotation-check.usda');anon.TransferContent(layer);anon.subLayerPaths=[]
stage=Usd.Stage.Open(anon);assert stage
prefix=Sdf.Path('/World/MidtermPreparation')
prims=[p for p in stage.TraverseAll() if p.GetPath().HasPrefix(prefix)]
assert len(prims)==8
for p in prims:
    assert not p.HasAPI(UsdPhysics.CollisionAPI) and not p.HasAPI(UsdPhysics.RigidBodyAPI)
    assert not p.GetAppliedSchemas()
    assert p.GetTypeName() not in ['PhysicsScene','PhysicsRevoluteJoint','PhysicsPrismaticJoint','OmniGraph']
checks=[]
for r in cfg['regions']:
    prim=stage.GetPrimAtPath(str(prefix)+'/'+r['usd_name']);curves=UsdGeom.BasisCurves(prim);assert curves
    pts=curves.GetPointsAttr().Get();counts=curves.GetCurveVertexCountsAttr().Get();assert list(counts)==[len(pts)]
    assert pts[0]==pts[-1] and all(math.isfinite(v) for p in pts for v in p)
    x0,x1,y0,y1=r['bounds_xy_m'];assert min(p[0] for p in pts)==x0 and max(p[0] for p in pts)==x1 and min(p[1] for p in pts)==y0 and max(p[1] for p in pts)==y1
    assert UsdGeom.Imageable(prim).GetPurposeAttr().Get()=='guide'
    cam=UsdGeom.Camera(stage.GetPrimAtPath(str(prefix)+'/'+r['id']+'_Top'));assert cam
    xyz=UsdGeom.Xformable(cam).GetOrderedXformOps()[0].Get();assert abs(xyz[0]-(x0+x1)/2)<1e-5 and abs(xyz[1]-(y0+y1)/2)<1e-5
    assert abs(float(cam.GetHorizontalApertureAttr().Get())/float(cam.GetVerticalApertureAttr().Get())-16/9)<1e-6
    checks.append({'id':r['id'],'boundary_points':len(pts),'bounds_xy':'PASS','camera':'PASS','annotation_collision':'NONE','purpose':'guide'})
marker=stage.GetPrimAtPath(str(prefix)+'/C_SpawnCandidate');assert marker and UsdGeom.Imageable(marker).GetPurposeAttr().Get()=='guide'
scene=Path(cfg['source']['rover_entry']);assert hashlib.sha256(scene.read_bytes()).hexdigest()==cfg['source']['rover_entry_sha256']
epath=root/'preparation-evidence.json';ev=json.loads(epath.read_text())
env=scene.parent.parent/'assets/environment.usdc';digest=hashlib.sha256()
with env.open('rb') as f:
    for chunk in iter(lambda:f.read(1024*1024),b''):digest.update(chunk)
ev['source_files']=[item for item in ev['source_files'] if item['path']!=str(env)]
ev['source_files'].append({'path':str(env),'bytes':env.stat().st_size,'sha256':digest.hexdigest()})
ev.update(preparation_status='PREPARED_STATIC_CHECKS_PASS',usd_parse='PASS',source_root_layer_parse='PASS',usd_annotation_checks=checks,usd_added_prim_count=len(prims),usd_full_composition='NOT_RUN',gui_render='NOT_RUN',sensor_visibility='GUIDE_ANNOTATIONS_PREVIEW_ONLY_USE_CLEAN_SOURCE_FOR_SENSOR_CAPTURE')
ev['verified_at']=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()
epath.write_text(json.dumps(ev,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
files=[{'file':p.name,'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(root.iterdir()) if p.is_file() and p.name!='artifact-manifest.json']
(root/'artifact-manifest.json').write_text(json.dumps({'task_id':cfg['task_id'],'files':files},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'status':'PASS','usd_parse':True,'added_prim_count':len(prims),'regions':checks,'source_sha_unchanged':True,'full_composition':'NOT_RUN','gui_render':'NOT_RUN','runtime':'NOT_RUN','files':[p['file'] for p in files]},ensure_ascii=False))
