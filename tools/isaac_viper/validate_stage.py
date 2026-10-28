"""重开实际 USD 验证工程合同；不加载 Isaac/GPU，不冒充运行验收。"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from tools.isaac_viper.config import load_profile

ROOT = "/World/ProjectVIPER"
CORNERS = ("fl", "fr", "rl", "rr")
TOL = 3e-6


def _close(a, b, tol=TOL):
    return len(a) == len(b) and all(math.isfinite(float(x)) and abs(x - y) <= tol for x, y in zip(a, b))


def _points(prim, cache, inverse):
    matrix = cache.GetLocalToWorldTransform(prim) * inverse
    if prim.IsA(UsdGeom.Mesh):
        values = UsdGeom.Mesh(prim).GetPointsAttr().Get() or []
    elif prim.IsA(UsdGeom.Cube):
        r = UsdGeom.Cube(prim).GetSizeAttr().Get() / 2
        values = [Gf.Vec3d(x, y, z) for x in (-r, r) for y in (-r, r) for z in (-r, r)]
    elif prim.IsA(UsdGeom.Sphere):
        # 仿射变换后的球是椭球；各轴支撑半径可解析求出，不依赖可能过期的 extent。
        r = UsdGeom.Sphere(prim).GetRadiusAttr().Get()
        if not math.isfinite(r) or r <= 0:
            return []  # 调用方会将非法球半径记录为合同失败。
        center = matrix.Transform(Gf.Vec3d(0))
        extent = [r * math.sqrt(sum(matrix[i][j] ** 2 for i in range(3))) for j in range(3)]
        return [center + Gf.Vec3d(x, y, z) for x in (-extent[0], extent[0])
                for y in (-extent[1], extent[1]) for z in (-extent[2], extent[2])]
    else:
        return []
    return [matrix.Transform(Gf.Vec3d(v)) for v in values]


def validate_stage(stage_or_path, profile) -> dict:
    """返回 passed/errors 与实际测量；输入 Stage 时允许测试 session 层故障注入。"""
    p = load_profile(profile) if not isinstance(profile, dict) else profile
    stage = stage_or_path if isinstance(stage_or_path, Usd.Stage) else Usd.Stage.Open(str(stage_or_path))
    errors = []

    def check(ok, label):
        if not ok:
            errors.append(label)

    root = stage.GetPrimAtPath(ROOT)
    if not root:
        return {"passed": False, "errors": ["missing /World/ProjectVIPER"]}
    prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
    bodies = [q for q in prims if q.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [q for q in prims if q.IsA(UsdPhysics.Joint)]
    masses = [UsdPhysics.MassAPI(q).GetMassAttr().Get() or 0 for q in bodies]
    mass_sum = sum(masses)
    check(len(bodies) == 9, f"rigid_body_count: {len(bodies)} != 9")
    check(len(joints) == 8, f"joint_count: {len(joints)} != 8")
    check(abs(mass_sum - p["simulation"]["mass"]["total_kg"]) < TOL, f"mass_kg: {mass_sum}")
    check(UsdGeom.GetStageMetersPerUnit(stage) == 1 and UsdGeom.GetStageUpAxis(stage) == "Z", "units/up_axis")
    check(UsdPhysics.GetStageKilogramsPerUnit(stage) == 1, "kilograms_per_unit")
    expected_bodies = {ROOT + "/chassis"} | {ROOT + f"/{c}_{b}" for c in CORNERS for b in ("steer", "wheel")}
    check({str(q.GetPath()) for q in bodies} == expected_bodies, "canonical_body_paths")
    for body in bodies:
        name = str(body.GetPath())
        props, rigid = UsdPhysics.MassAPI(body), UsdPhysics.RigidBodyAPI(body)
        mass_key = "chassis_kg" if body.GetName() == "chassis" else ("wheel_kg" if body.GetName().endswith("_wheel") else "steering_knuckle_kg")
        check(abs((props.GetMassAttr().Get() or 0) - p["simulation"]["mass"][mass_key]) < TOL, name + ": body_mass")
        inertia = props.GetDiagonalInertiaAttr().Get()
        check(bool(inertia) and all(math.isfinite(v) and v > 0 for v in inertia), name + ": positive_inertia")
        check(_close(rigid.GetVelocityAttr().Get() or [], [0, 0, 0]), name + ": zero_linear_velocity")
        check(_close(rigid.GetAngularVelocityAttr().Get() or [], [0, 0, 0]), name + ": zero_angular_velocity")
        check(_close(props.GetCenterOfMassAttr().Get() or [], [0, 0, 0]), name + ": body_origin_is_com")
        check(rigid.GetRigidBodyEnabledAttr().Get() and not rigid.GetKinematicEnabledAttr().Get(), name + ": dynamic_body")
    # 旧场景拓扑明确失败后仍返回基本实际测量，避免以缺字段异常掩盖 447kg。
    if {str(q.GetPath()) for q in bodies} != expected_bodies:
        return {"passed": False, "errors": errors, "rigid_body_count": len(bodies),
                "joint_count": len(joints), "mass_kg": mass_sum}
    cache = UsdGeom.XformCache()
    inverse = cache.GetLocalToWorldTransform(root).GetInverse()
    cap, sim = p["capability"], p["simulation"]
    radius, width = sim["wheel_radius_m"], cap["wheel_width_m"]
    wheel_mass = sim["mass"]["wheel_kg"]
    wheel_inertia = [wheel_mass * (3 * radius ** 2 + width ** 2) / 12,
                     wheel_mass * radius ** 2 / 2,
                     wheel_mass * (3 * radius ** 2 + width ** 2) / 12]
    expected_collisions = {ROOT + "/chassis/CollisionUpper", ROOT + "/chassis/CollisionKeel"} | {
        ROOT + f"/{c}_{b}/Collision" for c in CORNERS for b in ("steer", "wheel")}
    collisions = [q for q in prims if q.HasAPI(UsdPhysics.CollisionAPI)]
    check({str(q.GetPath()) for q in collisions} == expected_collisions, "canonical_collision_paths")
    for collision in collisions:
        check(UsdPhysics.CollisionAPI(collision).GetCollisionEnabledAttr().Get() is True,
              str(collision.GetPath()) + ": collision_enabled")
    knuckle_bounds = []
    wheel_measurements = {}
    for corner, center in zip(CORNERS, sim["wheel_centers_m"]):
        steer = stage.GetPrimAtPath(ROOT + f"/{corner}_steer")
        steer_mat = cache.GetLocalToWorldTransform(steer) * inverse
        check(_close(steer_mat.ExtractTranslation(), center) and
              all(_close(steer_mat[i], Gf.Matrix4d(1)[i]) for i in range(3)), corner + ": knuckle_initial_pose")
        sphere = UsdGeom.Sphere(stage.GetPrimAtPath(str(steer.GetPath()) + "/Collision"))
        check(bool(sphere), corner + ": sphere_collision_missing")
        if sphere:
            sr = sphere.GetRadiusAttr().Get()
            check(math.isfinite(sr) and abs(sr - .025) < TOL, corner + ": knuckle_radius")
            sphere_mat = cache.GetLocalToWorldTransform(sphere.GetPrim()) * cache.GetLocalToWorldTransform(steer).GetInverse()
            check(all(_close(sphere_mat[i], Gf.Matrix4d(1)[i]) for i in range(4)), corner + ": knuckle_collision_transform")
            expected_inertia = [.4 * sim["mass"]["steering_knuckle_kg"] * sr ** 2] * 3
            check(_close(UsdPhysics.MassAPI(steer).GetDiagonalInertiaAttr().Get(), expected_inertia), corner + ": sphere_inertia")
            values = _points(sphere.GetPrim(), cache, inverse)
            if values:
                knuckle_bounds.append(([min(v[i] for v in values) for i in range(3)],
                                       [max(v[i] for v in values) for i in range(3)]))
        wheel = stage.GetPrimAtPath(ROOT + f"/{corner}_wheel")
        mat = cache.GetLocalToWorldTransform(wheel) * inverse
        check(_close(mat.ExtractTranslation(), center), corner + ": wheel_center")
        check(_close(mat.TransformDir(Gf.Vec3d(0, 1, 0)), [0, 1, 0]), corner + ": roll_axis")
        wheel_inverse = cache.GetLocalToWorldTransform(wheel).GetInverse()
        vertices = [v for q in Usd.PrimRange(wheel, Usd.TraverseInstanceProxies())
                    if q.IsA(UsdGeom.Mesh) for v in _points(q, cache, wheel_inverse)]
        if not vertices:
            errors.append(corner + ": missing_actual_visual_mesh")
            continue
        actual_radius = max(math.hypot(v[0], v[2]) for v in vertices)
        ymin, ymax = min(v[1] for v in vertices), max(v[1] for v in vertices)
        check(abs(actual_radius - radius) < TOL, corner + ": per_vertex_rolling_radius")
        check(_close([ymin, ymax], [-width / 2, width / 2]), corner + ": visual_width_centering")
        collision = UsdGeom.Cylinder(stage.GetPrimAtPath(str(wheel.GetPath()) + "/Collision"))
        check(bool(collision), corner + ": cylinder_collision_missing")
        if collision:
            check(collision.GetAxisAttr().Get() == "Y" and
                  _close([collision.GetRadiusAttr().Get(), collision.GetHeightAttr().Get()], [radius, width]), corner + ": collision_geometry")
            check(collision.GetPrim().HasAPI(UsdPhysics.CollisionAPI), corner + ": collision_api")
            check(UsdPhysics.CollisionAPI(collision.GetPrim()).GetCollisionEnabledAttr().Get() is True, corner + ": collision_enabled")
            collision_mat = cache.GetLocalToWorldTransform(collision.GetPrim()) * wheel_inverse
            check(all(_close(collision_mat[i], Gf.Matrix4d(1)[i]) for i in range(4)), corner + ": unscaled_centered_collision")
        check(_close(UsdPhysics.MassAPI(wheel).GetDiagonalInertiaAttr().Get(), wheel_inertia), corner + ": cylinder_inertia")
        wheel_measurements[corner] = {"radius_m": actual_radius, "width_m": ymax - ymin,
                                      "center_m": list(mat.ExtractTranslation()), "vertices": len(vertices)}
        for kind, axis, parent, child in (("steering", "Z", ROOT + "/chassis", ROOT + f"/{corner}_steer"),
                                           ("rolling", "Y", ROOT + f"/{corner}_steer", ROOT + f"/{corner}_wheel")):
            joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(ROOT + f"/{corner}_{kind}_joint"))
            check(bool(joint), corner + kind + ": joint_missing")
            if not joint:
                continue
            check(joint.GetAxisAttr().Get() == axis, corner + kind + ": joint_axis")
            for attr in (joint.GetLocalRot0Attr(), joint.GetLocalRot1Attr()):
                quat = attr.Get()
                check(_close([quat.GetReal(), *quat.GetImaginary()], [1, 0, 0, 0]), corner + kind + ": joint_frame_rotation")
            if kind == "steering":
                limit = math.degrees(p["drive"]["steering_limit_rad"])
                check(_close([joint.GetLowerLimitAttr().Get(), joint.GetUpperLimitAttr().Get()], [-limit, limit]), corner + ": steering_limits")
            check([str(v) for v in joint.GetBody0Rel().GetTargets()] == [parent] and
                  [str(v) for v in joint.GetBody1Rel().GetTargets()] == [child], corner + kind + ": joint_relationship")
            for index, bodypath in ((0, parent), (1, child)):
                local = (joint.GetLocalPos0Attr() if index == 0 else joint.GetLocalPos1Attr()).Get()
                anchor = (cache.GetLocalToWorldTransform(stage.GetPrimAtPath(bodypath)) * inverse).Transform(Gf.Vec3d(local))
                check(_close(anchor, center), corner + kind + ": joint_anchor")
            drive = UsdPhysics.DriveAPI(joint.GetPrim(), "angular")
            check(drive.GetTargetPositionAttr().Get() == 0 and drive.GetTargetVelocityAttr().Get() == 0, corner + kind + ": zero_drive")
            for key in ("position", "velocity"):
                check(joint.GetPrim().GetAttribute("state:angular:physics:" + key).Get() == 0, corner + kind + ": zero_joint_" + key)
    meshes = [q for q in prims if q.IsA(UsdGeom.Mesh)]
    for mesh in meshes:
        bound, _ = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        check(bool(bound), "unbound_visual_material:" + str(mesh.GetPath()))
    visible = [q for q in prims if q.IsA(UsdGeom.Imageable) and UsdGeom.Imageable(q).ComputeVisibility() != "invisible"]
    supported_visible = []
    for prim in visible:
        if not prim.IsA(UsdGeom.Boundable):
            continue
        supported = any(prim.IsA(kind) for kind in (UsdGeom.Mesh, UsdGeom.Cube, UsdGeom.Sphere))
        check(supported, "unsupported_visible_boundable:" + str(prim.GetPath()))
        if not supported:
            continue
        if prim.IsA(UsdGeom.Sphere):
            r = UsdGeom.Sphere(prim).GetRadiusAttr().Get()
            check(math.isfinite(r) and r > 0, "invalid_sphere_radius:" + str(prim.GetPath()))
        supported_visible.append(prim)
    # 所有可见 Boundable 已被处理或显式拒绝；不再静默遗漏新的几何类型。
    visible = supported_visible
    all_points = [v for q in visible for v in _points(q, cache, inverse)]
    low = [min(v[i] for v in all_points) for i in range(3)]
    high = [max(v[i] for v in all_points) for i in range(3)]
    extent = cap["body_extent_m"]
    check(all(a >= b - TOL for a, b in zip(low, [-extent[0] / 2, -extent[1] / 2, 0])) and
          all(a <= b + TOL for a, b in zip(high, [extent[0] / 2, extent[1] / 2, extent[2]])), "nominal_visual_envelope")
    body_points = [v for q in visible if str(q.GetPath()).startswith(ROOT + "/chassis/Visual/") for v in _points(q, cache, inverse)]
    check(bool(body_points) and min(v[2] for v in body_points) >= 2 * radius - TOL and
          max(abs(v[1]) for v in body_points) <= .25 + TOL, "nonwheel_visual_wheel_clearance")
    # 全连续转向用圆盘外包络保守证明，覆盖所有转向角，不依赖离散采样。
    sweep_radius = math.hypot(radius, width / 2)
    sweep_width = cap["track_width_m"] + 2 * sweep_radius
    chassis = stage.GetPrimAtPath(ROOT + "/chassis")
    collision_bounds = []
    for name in ("CollisionUpper", "CollisionKeel"):
        prim = stage.GetPrimAtPath(str(chassis.GetPath()) + "/" + name)
        if not prim or not prim.IsA(UsdGeom.Cube) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            errors.append("compound_collision_missing:" + name)
            continue
        values = _points(prim, cache, inverse)
        lo = [min(v[i] for v in values) for i in range(3)]
        hi = [max(v[i] for v in values) for i in range(3)]
        collision_bounds.append((lo, hi))
        for sphere_lo, sphere_hi in knuckle_bounds:
            check(any(a > d + TOL or b < c - TOL for a, b, c, d in zip(sphere_lo, sphere_hi, lo, hi)),
                  name + ": knuckle_body_interference")
        for x, y, z in sim["wheel_centers_m"]:
            dx, dy = max(lo[0] - x, 0, x - hi[0]), max(lo[1] - y, 0, y - hi[1])
            check(lo[2] > z + radius or hi[2] < z - radius or math.hypot(dx, dy) > sweep_radius,
                  name + ": continuous_steering_body_interference")
    if collision_bounds:
        check(abs(min(lo[2] for lo, _ in collision_bounds) - cap["minimum_underbody_clearance_m"]) < TOL, "underbody_clearance")
        # 从实际 USD 碰撞盒重新积分；防止只填一个任意正惯性。
        volumes = [math.prod(b - a for a, b in zip(lo, hi)) for lo, hi in collision_bounds]
        cmass = sim["mass"]["chassis_kg"]
        weights = [cmass * v / sum(volumes) for v in volumes]
        centers = [[(a + b) / 2 for a, b in zip(lo, hi)] for lo, hi in collision_bounds]
        com = [sum(w * c[i] for w, c in zip(weights, centers)) / cmass for i in range(3)]
        origin = (cache.GetLocalToWorldTransform(chassis) * inverse).ExtractTranslation()
        check(_close(com, origin), "compound_center_of_mass")
        inertia = []
        for i in range(3):
            j, k = (i + 1) % 3, (i + 2) % 3
            inertia.append(sum(w * (((hi[j] - lo[j]) ** 2 + (hi[k] - lo[k]) ** 2) / 12 +
                                    (c[j] - com[j]) ** 2 + (c[k] - com[k]) ** 2)
                               for w, c, (lo, hi) in zip(weights, centers, collision_bounds)))
        check(_close(inertia, UsdPhysics.MassAPI(chassis).GetDiagonalInertiaAttr().Get()), "compound_inertia")
    for name in ("left_camera", "right_camera", "imu", "lidar"):
        prim = stage.GetPrimAtPath(ROOT + "/chassis/FrontSensors/" + name)
        check(bool(prim) and prim.IsActive(), name + ": missing_sensor_mount")
        if not prim:
            continue
        mat = cache.GetLocalToWorldTransform(prim) * inverse
        expected = p["sensors"]["stereo"][name.split("_")[0] + "_origin_m"] if "camera" in name else p["sensors"][name]["origin_m"]
        check(_close(mat.ExtractTranslation(), expected), name + ": profile_extrinsic")
        if "camera" in name:
            camera = UsdGeom.Camera(prim)
            check(bool(camera), name + ": camera_schema")
            if camera:
                for field, attribute in (("focal_length_mm", camera.GetFocalLengthAttr()),
                                         ("horizontal_aperture_mm", camera.GetHorizontalApertureAttr()),
                                         ("vertical_aperture_mm", camera.GetVerticalApertureAttr())):
                    check(_close([attribute.Get()], [p["sensors"]["stereo"][field]]), name + ": " + field)
                check(camera.GetProjectionAttr().Get() == "perspective", name + ": perspective")
                check(_close([camera.GetHorizontalApertureOffsetAttr().Get(),
                              camera.GetVerticalApertureOffsetAttr().Get()], [0, 0]), name + ": aperture_offset")
            check(_close(mat.TransformDir(Gf.Vec3d(0, 0, -1)), [1, 0, 0]) and
                  _close(mat.TransformDir(Gf.Vec3d(0, 1, 0)), [0, 0, 1]), name + ": optical_orientation")
        else:
            check(all(_close(mat.TransformDir(axis), axis) for axis in
                      (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))),
                  name + ": body_aligned_orientation")
        if name != "imu":
            for visual in visible:
                values = _points(visual, cache, inverse)
                if not values:
                    continue
                lo = [min(v[i] for v in values) for i in range(3)]
                hi = [max(v[i] for v in values) for i in range(3)]
                dx = hi[0] - expected[0]
                if dx <= TOL:
                    continue
                dy = max(lo[1] - expected[1], 0, expected[1] - hi[1])
                dz = max(lo[2] - expected[2], 0, expected[2] - hi[2])
                check(dy > dx or dz > dx * .75, name + ": forward_fov_self_occlusion:" + str(visual.GetPath()))
    material_assets = set()
    for prim in prims:
        for attr in prim.GetAttributes():
            if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
                value = attr.Get()
                if value and value.path:
                    material_assets.add(value.path)
                    check(bool(value.resolvedPath) and Path(value.resolvedPath).is_file(), "unresolved_asset:" + value.path)
    metadata_text = stage.GetRootLayer().customLayerData.get("engineering_metadata_json")
    check(bool(metadata_text), "engineering_metadata_missing")
    if metadata_text:
        metadata = json.loads(metadata_text)
        actual_com = (cache.GetLocalToWorldTransform(chassis) * inverse).ExtractTranslation()
        check(_close(metadata.get("base_link_origin_in_footprint_m", []), actual_com), "metadata_base_link_com")
        check(_close(metadata.get("base_link_to_footprint_translation_m", []), [-v for v in actual_com]),
              "metadata_footprint_translation")
        for path, digest in metadata["source_hashes"].items():
            check(Path(path).is_file() and hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, "source_hash_changed:" + path)
    for path in ("/World/LunarRover", "/World/Cameras/RoverCamera", "/Render"):
        legacy = stage.GetPrimAtPath(path)
        check(not legacy or not legacy.IsActive(), "legacy_prim_active:" + path)
    active_graphs = [str(q.GetPath()) for q in stage.Traverse() if "OmniGraph" in q.GetTypeName() or q.GetAttribute("node:type").Get()]
    check(not active_graphs, "legacy_control_graphs:" + str(active_graphs))
    physics = UsdPhysics.Scene(stage.GetPrimAtPath("/World/PhysicsScene"))
    check(bool(physics) and abs(physics.GetGravityMagnitudeAttr().Get() - 1.622) < TOL, "lunar_gravity")
    check(physics.GetPrim().GetAttribute("physxScene:enableGPUDynamics").Get() is False, "cpu_physx")
    check(root.GetAttribute("physxArticulation:enabledSelfCollisions").Get() is True, "self_collisions_must_remain_enabled")
    return {"passed": not errors, "errors": errors, "rigid_body_count": len(bodies), "joint_count": len(joints),
            "mass_kg": mass_sum, "visual_mesh_count": len(meshes), "material_asset_count": len(material_assets),
            "nominal_visual_bounds_m": [low, high], "wheel_measurements": wheel_measurements,
            "steering_sweep_width_m": sweep_width, "steering_clearance_proof": "continuous conservative disk vs actual compound boxes",
            "runtime_boundary": "USD_STATIC_ONLY; Isaac dynamics, GPU, ROS, Humble, Orin and vehicle NOT_RUN"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage")
    parser.add_argument("--profile", required=True)
    parser.add_argument("--report")
    args = parser.parse_args()
    result = validate_stage(args.stage, args.profile)
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    if args.report:
        Path(args.report).write_text(encoded, encoding="utf-8")
    print(encoded)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
