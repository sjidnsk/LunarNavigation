"""无 GPU 的工程 USD 生成器：规范物理骨架 + 只读 VIPER 视觉叶。"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from tools.isaac_viper.config import load_profile

ROOT = "/World/ProjectVIPER"
CORNERS = ("fl", "fr", "rl", "rr")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _transform(prim, translation=(0, 0, 0)):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translation))
    return xf


def _body(stage, path, origin, mass, inertia):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    _transform(prim, origin)
    body = UsdPhysics.RigidBodyAPI.Apply(prim)
    body.CreateVelocityAttr(Gf.Vec3f(0))
    body.CreateAngularVelocityAttr(Gf.Vec3f(0))
    body.CreateRigidBodyEnabledAttr(True)
    body.CreateKinematicEnabledAttr(False)
    props = UsdPhysics.MassAPI.Apply(prim)
    props.CreateMassAttr(mass)
    props.CreateCenterOfMassAttr(Gf.Vec3f(0))
    props.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    props.CreatePrincipalAxesAttr(Gf.Quatf(1))
    return prim


def _box(stage, path, center, size, *, collision=False, visible=True, color=None):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1)
    _transform(box.GetPrim(), center).AddScaleOp().Set(Gf.Vec3f(*size))
    if collision:
        UsdPhysics.CollisionAPI.Apply(box.GetPrim()).CreateCollisionEnabledAttr(True)
    if not visible:
        box.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    if color:
        box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return box


def _joint(stage, corner, kind, body0, body1, anchor, limit):
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{ROOT}/{corner}_{kind}_joint")
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateAxisAttr("Z" if kind == "steering" else "Y")
    joint.CreateLocalPos0Attr(Gf.Vec3f(*anchor))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    # 仅相邻刚体的关节碰撞过滤；不关闭整车 self collision。
    joint.CreateCollisionEnabledAttr(False)
    if kind == "steering":
        joint.CreateLowerLimitAttr(-math.degrees(limit))
        joint.CreateUpperLimitAttr(math.degrees(limit))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0)
    drive.CreateTargetVelocityAttr(0)
    drive.CreateStiffnessAttr(1500 if kind == "steering" else 0)
    drive.CreateDampingAttr(100 if kind == "steering" else 50)
    drive.CreateMaxForceAttr(80)
    joint.GetPrim().CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(0)
    joint.GetPrim().CreateAttribute("state:angular:physics:velocity", Sdf.ValueTypeNames.Float).Set(0)


def _mesh_points(prim, relative_to=None):
    cache = UsdGeom.XformCache()
    inv = cache.GetLocalToWorldTransform(relative_to).GetInverse() if relative_to else Gf.Matrix4d(1)
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if child.IsA(UsdGeom.Mesh):
            mat = cache.GetLocalToWorldTransform(child) * inv
            for point in UsdGeom.Mesh(child).GetPointsAttr().Get():
                yield mat.Transform(Gf.Vec3d(point))


def _visual(stage, path, instances, source_prim, matrix, substitution, assets_dir):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddReference(str(instances), "/Instances/" + source_prim.GetName())
    prim.SetInstanceable(False)
    UsdGeom.Xformable(prim).MakeMatrixXform().Set(matrix)
    # 解除材质实例后，只在工程层改缺失的灰色法线贴图。
    for child in list(Usd.PrimRange(prim)):
        if child.IsInstance():
            child.SetInstanceable(False)
    for child in Usd.PrimRange(prim):
        for attr in child.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                val = attr.Get()
                if val and "anodised_grey_normal.png" in val.path:
                    attr.Set(Sdf.AssetPath(str(assets_dir / substitution["replacement"])))
    return prim


def _collision_mass(boxes, mass):
    volumes = [math.prod(size) for _, size in boxes]
    masses = [mass * v / sum(volumes) for v in volumes]
    com = [sum(m * c[i] for m, (c, _) in zip(masses, boxes)) / mass for i in range(3)]
    inertia = [0., 0., 0.]
    for m, (center, size) in zip(masses, boxes):
        for i in range(3):
            j, k = (i + 1) % 3, (i + 2) % 3
            inertia[i] += m * ((size[j] ** 2 + size[k] ** 2) / 12 +
                               (center[j] - com[j]) ** 2 + (center[k] - com[k]) ** 2)
    return com, inertia


def _triangle_placement(source, profile, terrain_mesh_path, spawn_xy_m, spawn_yaw_deg):
    """在组合后实际三角面上重新取支撑；不使用高精源或最近顶点近似。"""
    import numpy as np
    from tools.isaac_viper.build_showcase import triangle_support
    mesh = UsdGeom.Mesh(source.GetPrimAtPath(terrain_mesh_path))
    if not mesh:
        raise ValueError('missing placement triangle mesh: '+terrain_mesh_path)
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), int)
    if np.any(counts != 3):
        raise ValueError('placement mesh must have explicit triangular faces')
    points = np.asarray(mesh.GetPointsAttr().Get(), float)
    matrix = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim()), float)
    points = points@matrix[:3,:3]+matrix[3,:3]
    faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), np.int32).reshape(-1,3)
    cx,cy = map(float, spawn_xy_m)
    if not all(math.isfinite(v) for v in (cx,cy,spawn_yaw_deg)):
        raise ValueError('nonfinite spawn pose')
    tri_xy = points[faces, :2]
    keep = (tri_xy[:,:,0].max(axis=1) >= cx-2)&(tri_xy[:,:,0].min(axis=1) <= cx+2)&(tri_xy[:,:,1].max(axis=1) >= cy-2)&(tri_xy[:,:,1].min(axis=1) <= cy+2)
    faces = faces[keep]
    x,y = np.meshgrid(np.linspace(cx-1.25,cx+1.25,7), np.linspace(cy-1.25,cy+1.25,7))
    xy = np.column_stack((x.ravel(), y.ravel()))
    z = triangle_support(points, faces, xy)
    fit = np.linalg.lstsq(np.column_stack((xy[:,0]-cx,xy[:,1]-cy,np.ones(len(xy)))),z,rcond=None)[0]
    normal = Gf.Vec3d(-float(fit[0]),-float(fit[1]),1).GetNormalized()
    tilt = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),normal))
    yaw = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),float(spawn_yaw_deg)))
    rotation = (yaw*tilt).ExtractRotation()
    offsets = [rotation.TransformDir(Gf.Vec3d(x,y,0)) for x,y,_ in profile['simulation']['wheel_centers_m']]
    contact_xy = [[cx+v[0],cy+v[1]] for v in offsets]
    heights = triangle_support(points,faces,contact_xy)
    base_z = max(float(z)-v[2] for z,v in zip(heights,offsets))+.012
    return [cx,cy,base_z], rotation.GetQuat(), {
        'terrain_plane':fit.tolist(), 'fresh_yaw_deg':spawn_yaw_deg,
        'terrain_mesh_path':terrain_mesh_path, 'surface_query':'actual_triangle_barycentric',
        'contact_xy_m':contact_xy, 'contact_height_m':heights.tolist(),
        'sampled_contact_clearances_m':[base_z+v[2]-float(z) for z,v in zip(heights,offsets)],
        'dynamic_settle':'NOT_RUN'}


def _placement(source, profile, terrain_mesh_path=None, spawn_xy_m=None, spawn_yaw_deg=90):
    """只读取源地形和静态 XY 位置；不继承已运行角度、关节或速度。"""
    import numpy as np
    if terrain_mesh_path is not None:
        if spawn_xy_m is None:
            raise ValueError('explicit triangle placement requires spawn_xy_m')
        return _triangle_placement(source, profile, terrain_mesh_path, spawn_xy_m, spawn_yaw_deg)
    old = source.GetPrimAtPath("/World/LunarRover")
    xy = UsdGeom.XformCache().GetLocalToWorldTransform(old).ExtractTranslation()
    cx, cy = map(float, spawn_xy_m) if spawn_xy_m is not None else (float(xy[0]), float(xy[1]))
    terrain = source.GetPrimAtPath("/World/Environment/DetailTerrain_Rover_plain/DetailMesh_Rover_plain")
    points = np.asarray(list(_mesh_points(terrain)), dtype=float)
    near = points[(abs(points[:, 0] - cx) < 1.5) & (abs(points[:, 1] - cy) < 1.5)]
    if len(near) < 100:
        raise ValueError("源地形不足以确定新初始姿态")
    fit = np.linalg.lstsq(np.c_[near[:, 0] - cx, near[:, 1] - cy, np.ones(len(near))], near[:, 2], rcond=None)[0]
    normal = Gf.Vec3d(-float(fit[0]), -float(fit[1]), 1).GetNormalized()
    tilt = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), normal))
    yaw = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(spawn_yaw_deg)))
    rotation = (yaw * tilt).ExtractRotation()
    contacts = []
    for x, y, _ in profile["simulation"]["wheel_centers_m"]:
        offset = rotation.TransformDir(Gf.Vec3d(x, y, 0))
        row = near[np.argmin((near[:, 0] - cx - offset[0]) ** 2 + (near[:, 1] - cy - offset[1]) ** 2)]
        contacts.append((float(row[2]), float(offset[2])))
    base_z = max(z - dz for z, dz in contacts) + .012
    return [cx, cy, base_z], rotation.GetQuat(), {
        "terrain_plane": fit.tolist(), "fresh_yaw_deg": spawn_yaw_deg,
        "sampled_contact_clearances_m": [base_z + dz - z for z, dz in contacts],
        "dynamic_settle": "NOT_RUN"}


def build_stage(profile, assets_dir, source_scene, output_path, *, terrain_mesh_path=None, spawn_xy_m=None, spawn_yaw_deg=90) -> dict:
    """生成新文件；拒绝覆盖。profile 可为 load_profile 返回值或 YAML 路径。"""
    p = load_profile(profile) if not isinstance(profile, dict) else profile
    assets, source_path, output = map(lambda x: Path(x).resolve(), (assets_dir, source_scene, output_path))
    if output.exists():
        raise FileExistsError(f"拒绝覆盖已有场景: {output}")
    substitution_path = assets.parent / "material-substitution.json"
    substitution = json.loads(substitution_path.read_text(encoding="utf-8"))
    if sha256(assets / substitution["replacement"]) != substitution["replacement_sha256"]:
        raise ValueError("法线替代纹理哈希不匹配")
    tracked = [source_path, substitution_path] + sorted(f for f in assets.rglob("*") if f.is_file())
    hashes = {str(f): sha256(f) for f in tracked}
    # 在打开源之前屏蔽旧在线 LiDAR；只读 layer 与匿名 session 不写原文件。
    session = Sdf.Layer.CreateAnonymous()
    Sdf.CreatePrimInLayer(session, "/World/LunarRover/chassis/Sensors").active = False
    pose_source = Usd.Stage.Open(Sdf.Layer.FindOrOpen(str(source_path)), session)
    position, orientation, placement = _placement(pose_source, p, terrain_mesh_path, spawn_xy_m, spawn_yaw_deg)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output))
    stage.OverridePrim("/World/LunarRover").SetActive(False)
    stage.GetRootLayer().subLayerPaths = [str(source_path)]
    disabled_legacy = ["/World/LunarRover", "/World/Cameras/RoverCamera", "/Render"]
    for prim in pose_source.Traverse():
        if "OmniGraph" in prim.GetTypeName() or prim.GetAttribute("node:type").Get():
            disabled_legacy.append(str(prim.GetPath()))
    for path in disabled_legacy:
        stage.OverridePrim(path).SetActive(False)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1)
    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.GetPrim().AddAppliedSchema("PhysxSceneAPI")
    physics.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics.CreateGravityMagnitudeAttr(1.622)
    physics.GetPrim().CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(False)
    physics.GetPrim().CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("MBP")
    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    _transform(root, position).AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orientation)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.AddAppliedSchema("PhysxArticulationAPI")
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(True)
    cap, sim = p["capability"], p["simulation"]
    mass = sim["mass"]
    length = cap["body_extent_m"][0]
    clearance = cap["minimum_underbody_clearance_m"]
    # 轮拱通过上宽下窄的复合碰撞实现；质心/惯性由两个均匀密度盒算出。
    boxes = [([0, 0, .57], [length, .50, .46]),
             ([0, 0, (clearance + .34) / 2], [.55, .25, .34 - clearance])]
    com, inertia = _collision_mass(boxes, mass["chassis_kg"])
    chassis = _body(stage, ROOT + "/chassis", com, mass["chassis_kg"], inertia)
    for name, (center, size) in zip(("Upper", "Keel"), boxes):
        _box(stage, str(chassis.GetPath()) + "/Collision" + name,
             [a - b for a, b in zip(center, com)], size, collision=True, visible=False)
    radius, width = sim["wheel_radius_m"], cap["wheel_width_m"]
    wheel_inertia = [mass["wheel_kg"] * (3 * radius ** 2 + width ** 2) / 12,
                     mass["wheel_kg"] * radius ** 2 / 2,
                     mass["wheel_kg"] * (3 * radius ** 2 + width ** 2) / 12]
    for corner, center in zip(CORNERS, sim["wheel_centers_m"]):
        knuckle = _body(stage, ROOT + f"/{corner}_steer", center, mass["steering_knuckle_kg"],
                        [mass["steering_knuckle_kg"] * .025 ** 2 * .4] * 3)
        sphere = UsdGeom.Sphere.Define(stage, str(knuckle.GetPath()) + "/Collision")
        sphere.CreateRadiusAttr(.025)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(.3, .3, .32)])
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        wheel = _body(stage, ROOT + f"/{corner}_wheel", center, mass["wheel_kg"], wheel_inertia)
        cylinder = UsdGeom.Cylinder.Define(stage, str(wheel.GetPath()) + "/Collision")
        cylinder.CreateAxisAttr("Y")
        cylinder.CreateRadiusAttr(radius)
        cylinder.CreateHeightAttr(width)
        cylinder.CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        _joint(stage, corner, "steering", chassis.GetPath(), knuckle.GetPath(),
               [a - b for a, b in zip(center, com)], p["drive"]["steering_limit_rad"])
        _joint(stage, corner, "rolling", knuckle.GetPath(), wheel.GetPath(), [0, 0, 0], 0)
    source = Usd.Stage.Open(str(assets / "usd/viper.usda"))
    source.SetEditTarget(source.GetSessionLayer())
    source.GetPrimAtPath("/viper").GetVariantSets().GetVariantSet("Physics").SetVariantSelection("none")
    leaves = [q for q in source.Traverse() if q.GetTypeName() == "Xform" and "__" in q.GetName() and "_col" not in q.GetName()]
    nonwheels = [q for q in leaves if not any(f"/{c}_arm/" in str(q.GetPath()) for c in CORNERS)]
    points = [v for q in nonwheels for v in _mesh_points(q)]
    low = [min(v[i] for v in points) for i in range(3)]
    high = [max(v[i] for v in points) for i in range(3)]
    # 收到 +X=.48 后方，保证新功能传感器前向视锥无原 mast 遮挡。
    target_low = [-.48, -.25, .34]
    target_high = [.48, .25, cap["body_extent_m"][2]]
    scales = [(b - a) / (hi - lo) for a, b, lo, hi in zip(target_low, target_high, low, high)]
    body_map = Gf.Matrix4d().SetScale(Gf.Vec3d(*scales)) * Gf.Matrix4d().SetTranslate(
        Gf.Vec3d(*[a - lo * s - c for a, lo, s, c in zip(target_low, low, scales, com)]))
    cache = UsdGeom.XformCache()
    instances = assets / "usd/payloads/instances.usda"
    for leaf in nonwheels:
        _visual(stage, ROOT + "/chassis/Visual/" + leaf.GetName(), instances, leaf,
                cache.GetLocalToWorldTransform(leaf) * body_map, substitution, assets)
    wheel_scales = {}
    for corner in CORNERS:
        wheel_leaves = [q for q in leaves if q.GetName().startswith(corner + "_wheel__")]
        parent = wheel_leaves[0].GetParent()
        vertices = [v for q in wheel_leaves for v in _mesh_points(q, parent)]
        radial = max(math.hypot(v[0], v[2]) for v in vertices)
        ymin, ymax = min(v[1] for v in vertices), max(v[1] for v in vertices)
        scale = [radius / radial, width / (ymax - ymin), radius / radial]
        # 真实径向最大值约束滚动全过程，不采用 X/Z AABB 直径。
        mapping = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, -(ymin + ymax) / 2, 0)) * Gf.Matrix4d().SetScale(Gf.Vec3d(*scale))
        wheel_scales[corner] = {"source_max_radius_m": radial, "scale": scale}
        for leaf in wheel_leaves:
            relative = cache.GetLocalToWorldTransform(leaf) * cache.GetLocalToWorldTransform(parent).GetInverse()
            _visual(stage, ROOT + f"/{corner}_wheel/Visual/" + leaf.GetName(), instances, leaf,
                    relative * mapping, substitution, assets)
    sensors = p["sensors"]
    rig = UsdGeom.Xform.Define(stage, ROOT + "/chassis/FrontSensors")
    _transform(rig.GetPrim(), [-v for v in com])
    _box(stage, str(rig.GetPath()) + "/MountRod", sim["mast"]["center_m"], sim["mast"]["size_m"], color=(.65, .65, .7))
    for name in ("stereo", "lidar"):
        conf = sensors[name]
        _box(stage, str(rig.GetPath()) + "/" + name + "_housing", conf["housing_center_m"], conf["housing_size_m"], color=(.08, .1, .14))
    for side in ("left", "right"):
        origin = sensors["stereo"][side + "_origin_m"]
        housing = sensors["stereo"]
        lens_x = housing["housing_center_m"][0] + housing["housing_size_m"][0] / 2 - .001
        _box(stage, str(rig.GetPath()) + "/" + side + "_lens_window", [lens_x, origin[1], origin[2]],
             [.002, .032, .032], color=(.015, .12, .28))
        camera = UsdGeom.Camera.Define(stage, str(rig.GetPath()) + f"/{side}_camera")
        xf = _transform(camera.GetPrim(), sensors["stereo"][side + "_origin_m"])
        # USD -Z 向 +X，USD +Y 向 +Z；ROS optical +Z 向 +X、+X 向 -Y。
        mat = Gf.Matrix3d(0, -1, 0, 0, 0, 1, -1, 0, 0)
        xf.AddOrientOp().Set(Gf.Quatf(mat.ExtractRotation().GetQuat()))
        camera.CreateFocalLengthAttr(sensors["stereo"]["focal_length_mm"])
        camera.CreateHorizontalApertureAttr(sensors["stereo"]["horizontal_aperture_mm"])
        camera.CreateVerticalApertureAttr(sensors["stereo"]["vertical_aperture_mm"])
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01, 1000))
    for name in ("imu", "lidar"):
        sensor = UsdGeom.Xform.Define(stage, str(rig.GetPath()) + "/" + name)
        _transform(sensor.GetPrim(), sensors[name]["origin_m"])
        sensor.GetPrim().SetCustomDataByKey("runtime_sensor_status", "NOT_RUN; Task4 attaches actual SDK sensor")
    unchanged = all(sha256(f) == digest for f, digest in hashes.items())
    if not unchanged:
        raise RuntimeError("源资产在生成期间改变")
    metadata = {"schema": "project-viper-engineering-v1", "source_hashes": hashes,
                "source_scene_sha256": hashes[str(source_path)], "source_unchanged": unchanged,
                "disabled_legacy_prims": disabled_legacy, "mass_calibration_status": mass["calibration_status"], "wheel_scales": wheel_scales,
                "base_link_origin_in_footprint_m": com, "base_link_to_footprint_translation_m": [-v for v in com],
                "chassis_diagonal_inertia": inertia, "initial_position_m": position,
                "initial_orientation_wxyz": [orientation.GetReal(), *orientation.GetImaginary()],
                "placement": placement, "material_substitution": substitution,
                "decorative_mast": "Original fixed VIPER antenna/mast retained behind +X=.48; functional sensors at profile +X front",
                "runtime": "NOT_RUN; no SimulationApp/GPU/ROS started"}
    stage.GetRootLayer().customLayerData = {"engineering_metadata_json": json.dumps(metadata)}
    stage.GetRootLayer().Save()
    from tools.isaac_viper.validate_stage import validate_stage
    result = validate_stage(str(output), p)
    result.update({"stage": str(output), "metadata": metadata})
    output.with_suffix(".validation.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    if not result["passed"]:
        raise ValueError("生成场景未通过合同: " + "; ".join(result["errors"]))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--assets-dir", required=True)
    parser.add_argument("--source-scene", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument('--terrain-mesh-path')
    parser.add_argument('--spawn-xy-m', nargs=2, type=float)
    parser.add_argument('--spawn-yaw-deg', type=float, default=90)
    args = parser.parse_args()
    print(json.dumps(build_stage(args.profile, args.assets_dir, args.source_scene, args.output,
                                terrain_mesh_path=args.terrain_mesh_path, spawn_xy_m=args.spawn_xy_m,
                                spawn_yaw_deg=args.spawn_yaw_deg), ensure_ascii=False))


if __name__ == "__main__":
    main()
