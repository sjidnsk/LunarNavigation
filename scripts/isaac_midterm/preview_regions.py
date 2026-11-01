import argparse,asyncio,json,traceback,math
from pathlib import Path

parser=argparse.ArgumentParser(description='在新的 Isaac Sim 窗口中预览三区，会话层标注、暂停仿真。')
parser.add_argument('--scene',type=Path,required=True,help='prepare_regions.py 生成的 Midterm_Polar_V02_Regions.usda')
parser.add_argument('--output-dir',type=Path,required=True,help='本次预览独立回执目录')
parser.add_argument('--dry-run',action='store_true',help='只验证参数与场景文件，不导入 Kit、不创建目录')
args,_kit_args=parser.parse_known_args()
SCENE=str(args.scene.resolve())
VIEW=args.output_dir.resolve()
STATUS=VIEW/'stage-status.json'
if not Path(SCENE).is_file():
    parser.error('指定的标注场景不存在')
if VIEW.exists():
    parser.error('预览目录已经存在，请使用新的 output-dir')
if args.dry_run:
    print(json.dumps({'scene':SCENE,'output_dir':str(VIEW),'preview_only':True,'will_start_simulation':False},ensure_ascii=False))
    raise SystemExit(0)
VIEW.mkdir(parents=True,exist_ok=False)
import omni.usd,omni.kit.app
def report(**data):
    STATUS.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
async def preview():
    try:
        report(state='opening',scene=SCENE)
        ctx=omni.usd.get_context();ok,error=await ctx.open_stage_async(SCENE)
        if not ok:raise RuntimeError(str(error))
        from pxr import Usd,UsdGeom,UsdShade,Gf,Sdf
        from omni import timeline as preview_timeline
        preview_timeline.get_timeline_interface().pause()
        stage=ctx.get_stage();cfg=json.loads((Path(SCENE).parent/'isaac-polar-v02-midterm-regions.json').read_text())
        with Usd.EditContext(stage,stage.GetSessionLayer()):
            for r in cfg['regions']:
                curve=UsdGeom.BasisCurves(stage.GetPrimAtPath('/World/MidtermPreparation/'+r['usd_name']))
                points=curve.GetPointsAttr().Get();UsdGeom.Imageable(curve.GetPrim()).CreateVisibilityAttr('invisible')
                verts=[];counts=[];indices=[]
                for p,q in zip(points,points[1:]):
                    dx=float(q[0]-p[0]);dy=float(q[1]-p[1]);length=math.hypot(dx,dy)
                    if length<1e-7:continue
                    nx=-dy/length;ny=dx/length;k=len(verts)
                    verts += [(float(p[0])+nx,float(p[1])+ny,float(p[2])+1.2),(float(p[0])-nx,float(p[1])-ny,float(p[2])+1.2),(float(q[0])-nx,float(q[1])-ny,float(q[2])+1.2),(float(q[0])+nx,float(q[1])+ny,float(q[2])+1.2)]
                    counts.append(4);indices.extend([k,k+1,k+2,k+3])
                mesh=UsdGeom.Mesh.Define(stage,'/World/MidtermPreparation/PreviewRibbons/'+r['id'])
                mesh.CreatePointsAttr(verts);mesh.CreateFaceVertexCountsAttr(counts);mesh.CreateFaceVertexIndicesAttr(indices)
                mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
                rgb=Gf.Vec3f(*r['colour_rgb']);mesh.CreateDisplayColorAttr([rgb])
                mat=UsdShade.Material.Define(stage,'/World/MidtermPreparation/PreviewMaterials/'+r['id'])
                shader=UsdShade.Shader.Define(stage,mat.GetPath().AppendChild('Shader'));shader.CreateIdAttr('UsdPreviewSurface')
                shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(rgb)
                shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(rgb*2)
                shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.6)
                mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
            marker=stage.GetPrimAtPath('/World/MidtermPreparation/C_SpawnCandidate');UsdGeom.Imageable(marker).CreatePurposeAttr('default')
            top=UsdGeom.Camera.Define(stage,'/World/MidtermPreparation/AllRegionsTop')
            top.CreateFocalLengthAttr(35);top.CreateHorizontalApertureAttr(36);top.CreateVerticalApertureAttr(20.25)
            top.CreateClippingRangeAttr(Gf.Vec2f(.1,10000));UsdGeom.Xformable(top).AddTranslateOp().Set(Gf.Vec3d(0,0,1800))
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        viewport=get_active_viewport();viewport.camera_path='/World/MidtermPreparation/AllRegionsTop';viewport.resolution=(1600,900)
        import carb
        carb.settings.get_settings().set('/app/viewport/grid/enabled',False)
        def choose_camera(name):
            viewport.camera_path='/World/MidtermPreparation/'+name
        ui_error=None
        try:
            from omni import ui as preview_ui
            panel=preview_ui.Window('中期展示区域 | Preview',width=310,height=225)
            panel.position_x=15;panel.position_y=90
            with panel.frame:
                with preview_ui.VStack(spacing=7):
                    preview_ui.Label('暂停预览 | No motion')
                    preview_ui.Button('总览 | All regions',clicked_fn=lambda:choose_camera('AllRegionsTop'))
                    preview_ui.Button('A 岩石绕障 | 100 x 100 m',clicked_fn=lambda:choose_camera('A_Top'))
                    preview_ui.Button('B 陨坑坡地 | 75 x 75 m',clicked_fn=lambda:choose_camera('B_Top'))
                    preview_ui.Button('C 自主探索 | 240 x 240 m',clicked_fn=lambda:choose_camera('C_Top'))
            setattr(omni,'_lunar_midterm_preview_panel',panel)
        except Exception as exc:ui_error=str(exc)
        for _ in range(65):await omni.kit.app.get_app().next_update_async()
        report(state='opened',scene=SCENE,loaded_stage=ctx.get_stage_url(),preview_only=True,timeline_playing=preview_timeline.get_timeline_interface().is_playing(),camera=str(viewport.camera_path),region_present=[bool(stage.GetPrimAtPath('/World/MidtermPreparation/PreviewRibbons/'+r['id'])) for r in cfg['regions']],annotation_style='session_only_emissive_ribbons',region_colours={r['id']:r['colour_rgb'] for r in cfg['regions']},camera_panel_error=ui_error,loading_status=str(ctx.get_stage_loading_status()),source_layers_saved=False,viewport_capture_state='requested')
        capture=capture_viewport_to_file(viewport,str(VIEW/'regions-viewport.png'))
        if hasattr(capture,'wait_for_result'):await capture.wait_for_result()
    except Exception:report(state='failed',scene=SCENE,error=traceback.format_exc())
asyncio.ensure_future(preview())
