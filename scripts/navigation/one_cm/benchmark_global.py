"""原生厘米稀疏全局完整更新基准；独立 SAT 连续几何验证及逐样本原始数据。"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import subprocess
import time

import numpy as np

from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.global_planner import AdaptiveGlobalPlanner
from scripts.navigation.one_cm.nav1cm.global_fixtures import topology_scenarios


def segment_hits_box(a, b, box):
    """独立分离轴定理：矩形 x/y 轴与线段法向；边界接触算相交。"""
    x0, y0, x1, y1 = box
    if max(a[0], b[0]) < x0 or min(a[0], b[0]) > x1 or max(a[1], b[1]) < y0 or min(a[1], b[1]) > y1:
        return False
    dx, dy = b[0] - a[0], b[1] - a[1]
    cross = [dx * (y - a[1]) - dy * (x - a[0]) for x, y in ((x0, y0), (x0, y1), (x1, y0), (x1, y1))]
    return min(cross) <= 0 <= max(cross)


def make_scale(scale, count, epoch):
    rng = random.Random(20261026 + scale)
    size = scale * 100
    # 第一面墙穿过起终点直线；其余障碍分散覆盖二维区域，厘米边界不量化。
    rectangles = [CellRect(size // 2, size * 4 // 10, size // 2 + 1, size * 6 // 10)]
    while len(rectangles) < count:
        x, y = rng.randint(200, size - 500), rng.randint(200, size - 500)
        w, h = rng.randint(1, 250), rng.randint(1, 250)
        rectangles.append(CellRect(x, y, x + w, y + h))
    return WorldSnapshot(epoch, tuple(rectangles), (CellRect(0, 0, 250, 250),)), (1., 1.), (scale - 1., scale - 1.)


def formal_success(planning_outcome, reason_code, has_reference, reference):
    """验证与统计共用四项成功门禁；reference 可为 Route 或序列化后的 path。"""
    return (planning_outcome == 0 and reason_code == "PLAN_FOUND" and
            has_reference is True and reference is not None)


def validate(world, start, goal, result, *, expected_version):
    reference = result.route
    success = formal_success(result.planning_outcome, result.reason_code, result.has_reference, reference)
    coherent_failure = (result.planning_outcome != 0 and result.reason_code != "PLAN_FOUND" and
                        result.has_reference is False and reference is None)
    status_consistent = success or coherent_failure
    if reference is None:
        return {"valid": coherent_failure, "formal_success": success, "status_consistent": status_consistent,
                "route_identity_valid": None, "length_m": None, "segments": 0,
                "raw_collision": False, "clearance_collision": False}
    points = reference.points
    raw_collision = clearance_collision = False
    r = result.clearance_radius_m
    for a, b in zip(points[:-1], points[1:]):
        for rect in world.blocked:
            box = (rect.x0 / 100, rect.y0 / 100, rect.x1 / 100, rect.y1 / 100)
            raw_collision |= segment_hits_box(a, b, box)
            clearance_collision |= segment_hits_box(a, b, (box[0] - r, box[1] - r, box[2] + r, box[3] + r))
    length = sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))
    identity_ok = (reference.map_epoch == world.epoch and reference.version == expected_version and
                   tuple(reference.goal) == tuple(goal) and tuple(points[0]) == tuple(start) and tuple(points[-1]) == tuple(goal))
    return {"valid": success and identity_ok and not raw_collision and not clearance_collision and length >= math.dist(start, goal) - 1e-8,
            "formal_success": success, "status_consistent": status_consistent, "route_identity_valid": identity_ok,
            "length_m": length, "segments": len(points) - 1,
            "raw_collision": bool(raw_collision), "clearance_collision": bool(clearance_collision)}


def validate_scenario(scenario, result, *, expected_version):
    """验证封闭域与规定门；布局声明不参与规划，验证与规划器几何实现独立。"""
    evidence = validate(scenario.world, scenario.start, scenario.goal, result, expected_version=expected_version)
    gates = [g for g in scenario.gates if g["mandatory"]]
    crossings = {g["id"]: None for g in gates}
    inside = None
    ordered = None
    known_length = sum(math.dist(a, b) for a, b in zip(scenario.known_route[:-1], scenario.known_route[1:]))
    radius = result.clearance_radius_m
    known_valid = None
    if scenario.known_route:
        known_valid = all(not segment_hits_box(a, b, (r.x0/100-radius, r.y0/100-radius, r.x1/100+radius, r.y1/100+radius))
                          for a, b in zip(scenario.known_route[:-1], scenario.known_route[1:]) for r in scenario.world.blocked)
    if result.route is not None:
        points = result.route.points
        xmin, ymin, xmax, ymax = scenario.inner_bounds
        inside = bool(np.all((points[:, 0] >= xmin) & (points[:, 0] <= xmax) &
                             (points[:, 1] >= ymin) & (points[:, 1] <= ymax)))
        progress = 0.
        for a, b in zip(points[:-1], points[1:]):
            length = math.dist(a, b)
            for gate in gates:
                axis = gate["axis"]
                delta = b[axis] - a[axis]
                if delta == 0:
                    continue
                parameter = (gate["coordinate"] - a[axis]) / delta
                transverse = a[1-axis] + parameter * (b[1-axis] - a[1-axis])
                if 0 <= parameter <= 1 and gate["interval"][0] <= transverse <= gate["interval"][1]:
                    if crossings[gate["id"]] is None:
                        crossings[gate["id"]] = progress + parameter * length
            progress += length
        visits = list(crossings.values())
        ordered = all(v is not None for v in visits)
        if scenario.topology["family"] == "enclosed_alternating_baffles" and ordered:
            ordered = all(a < b for a, b in zip(visits[:-1], visits[1:]))
        evidence["valid"] &= inside and ordered and evidence["length_m"] >= scenario.lower_bound_m - 1e-8
    evidence.update({"expected_reason": scenario.expected_reason,
                     "expected_reason_met": result.reason_code == scenario.expected_reason,
                     "inside_enclosure": inside, "mandatory_gates": {k: v is not None for k, v in crossings.items()},
                     "gate_crossing_arc_length_m": crossings, "gate_crossing_order_valid": ordered,
                     "known_route_valid": known_valid, "known_route_length_m": known_length or None,
                     "lower_bound_m": scenario.lower_bound_m,
                     "length_ratio_to_lower_bound": evidence["length_m"] / scenario.lower_bound_m if result.route is not None else None,
                     "length_ratio_to_known_route": evidence["length_m"] / known_length if result.route is not None and known_length else None})
    evidence["valid"] &= known_valid is not False
    # 失败/超时不会伪装成有解或测试通过；由独立 expected_reason_met 分列统计。
    return evidence


def rss():
    status = Path("/proc/self/status")
    if status.exists():
        values = {}
        for line in status.read_text().splitlines():
            if line.startswith(("VmRSS:", "VmHWM:")):
                key, number, _ = line.split()
                values[key[:-1] + "_bytes"] = int(number) * 1024
        return values
    return {"VmRSS_bytes": None, "VmHWM_bytes": None}


def summarize(samples):
    durations = np.asarray([s["wall_s"] for s in samples])
    return {"samples": len(samples), "p50_s": float(np.percentile(durations, 50)),
            "p95_s": float(np.percentile(durations, 95)), "p99_s": float(np.percentile(durations, 99)),
            "max_s": float(max(durations)), "over_budget_rate": sum(s["wall_s"] > s["budget_s"] for s in samples) / len(samples),
            "successes": sum(formal_success(s["planning_outcome"], s["reason_code"], s["has_reference"], s["path"])
                             and s["validation"]["valid"] for s in samples),
            "reasons": dict(Counter(s["reason_code"] for s in samples)),
            "all_geometry_valid": all(s["validation"]["valid"] for s in samples),
            "expected_reason_matches": sum(s["validation"].get("expected_reason_met", False) for s in samples),
            "max_rss_bytes": max((s["rss_after"].get("VmRSS_bytes") or 0) for s in samples),
            "max_hwm_bytes": max((s["rss_after"].get("VmHWM_bytes") or 0) for s in samples),
            "nodes": [min(s["storage"]["nodes"] for s in samples), max(s["storage"]["nodes"] for s in samples)],
            "edges": [min(s["storage"]["edges"] for s in samples), max(s["storage"]["edges"] for s in samples)]}


def run(output, repeats=20, suite="scale"):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[3]
    files = [Path(__file__).resolve(), root / "scripts/navigation/one_cm/nav1cm/global_planner.py",
             root / "scripts/navigation/one_cm/nav1cm/global_fixtures.py",
             root / "scripts/navigation/one_cm/nav1cm/contracts.py",
             root / "tests/navigation/one_cm/test_global_planner.py"]
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    try:
        head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True, stderr=subprocess.DEVNULL).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        head = "NOT_A_GIT_CHECKOUT"
    metadata = {"source_sha256": hashes, "combined_source_sha256": hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
                "git_head": head, "platform": platform.platform(), "machine": platform.machine(),
                "python": platform.python_version(), "numpy": np.__version__, "resolution_m": .01,
                "clearance_radius_m": .409, "cache": "NONE; every sample rebuilds; epoch changes",
                "budget_s": 1., "timed_scope": "plan: goal checks + decomposition + graph + search + shortcut + final validation",
                "independent_validation": "outside timed planner; separating axis theorem against every original and inflated rectangle",
                "geometry_only": True, "rss_at_start": rss()}
    metadata["suite"] = suite
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    planner = AdaptiveGlobalPlanner(WheelLimits())
    all_samples = []

    def sample(name, source, start, goal, selected=planner, budget=1., scenario=None):
        geometry = {"blocked": [asdict(r) for r in source.blocked], "known_free": [asdict(r) for r in source.known_free]}
        map_hash = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
        map_file = output / (name + "-map.json")
        if not map_file.exists():
            map_file.write_text(json.dumps({**geometry, "start": start, "goal": goal, "hash": map_hash}), encoding="utf-8")
        if scenario is not None and not (output / (name + "-fixture.json")).exists():
            (output / (name + "-fixture.json")).write_text(json.dumps(asdict(scenario), indent=2), encoding="utf-8")
        before = rss()
        requested_version = source.epoch
        began = time.perf_counter()
        result = selected.plan(source, start, goal, version=requested_version, budget_s=budget)
        wall = time.perf_counter() - began
        after = rss()
        record = {"scenario": name, "epoch": source.epoch, "map_sha256": map_hash,
                  "requested_version": requested_version, "requested_goal": list(goal),
                  "route_version": None if result.route is None else result.route.version,
                  "route_goal": None if result.route is None else list(result.route.goal),
                  "route_map_epoch": None if result.route is None else result.route.map_epoch,
                  "budget_s": budget, "wall_s": wall, "rss_before": before, "rss_after": after,
                  "planning_outcome": result.planning_outcome, "reason_code": result.reason_code,
                  "has_reference": result.has_reference, "storage": result.storage, "timing": result.timing,
                  "expanded": result.expanded, "domain": result.domain,
                  "clearance_radius_m": result.clearance_radius_m,
                  "direct_raw_blocked": any(segment_hits_box(start, goal, (r.x0/100, r.y0/100, r.x1/100, r.y1/100)) for r in source.blocked),
                  "path": None if result.route is None else result.route.points.tolist(),
                  "validation": validate(source, start, goal, result, expected_version=requested_version)
                  if scenario is None else validate_scenario(scenario, result, expected_version=requested_version)}
        with (output / "samples.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record) + "\n")
        all_samples.append(record)
        return record

    summary = {}
    if suite == "topology":
        for case_number, template in enumerate(topology_scenarios()):
            records = []
            for i in range(repeats):
                scene = replace(template, world=replace(template.world, epoch=100_000 + case_number * 1000 + i))
                records.append(sample(scene.name, scene.world, scene.start, scene.goal, scenario=scene))
            summary[template.name] = {**summarize(records), "topology": template.topology,
                                      "expected_reason": template.expected_reason,
                                      "lower_bound_m": template.lower_bound_m,
                                      "length_m": records[0]["validation"]["length_m"],
                                      "length_ratio_to_lower_bound": records[0]["validation"]["length_ratio_to_lower_bound"],
                                      "length_ratio_to_known_route": records[0]["validation"]["length_ratio_to_known_route"]}
            print(json.dumps({template.name: summary[template.name]}), flush=True)
        (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        # 保存所有失败/超时，退出码只对真正几何证据异常失败，性能结果独立报告。
        if not all(s["validation"]["valid"] for s in all_samples):
            raise RuntimeError("independent topology validation failed; inspect samples.jsonl")
        return summary
    for scale, count in [(100, 100), (500, 500), (1000, 1000)]:
        name = f"scale_{scale}m_{count}rects"
        records = [sample(name, *make_scale(scale, count, 10000 * scale + i)) for i in range(repeats)]
        summary[name] = summarize(records)
        print(json.dumps({name: summary[name]}), flush=True)
    ring = (CellRect(-200, -200, 200, -190), CellRect(-200, 190, 200, 200),
            CellRect(-200, -200, -190, 200), CellRect(190, -200, 200, 200))
    small = [
        ("unknown_outside", (), (0., 0.), (1000., 37.), planner, 1.),
        ("thin_continuous_wall", (CellRect(200, -100, 201, 100),), (0., 0.), (4., 0.), planner, 1.),
        ("gate_83cm", (CellRect(-1, -1000, 1, 0), CellRect(-1, 83, 1, 1000)), (-2., .415), (2., .415), AdaptiveGlobalPlanner(WheelLimits(), domain_bounds_m=(-3,-4,3,4)), 1.),
        ("gate_81cm", (CellRect(-1, -1000, 1, 0), CellRect(-1, 81, 1, 1000)), (-2., .415), (2., .415), AdaptiveGlobalPlanner(WheelLimits(), domain_bounds_m=(-3,-4,3,4)), 1.),
        ("enclosed_ring", ring, (0., 0.), (5., 0.), planner, 1.),
        ("forced_budget", ring, (0., 0.), (5., 0.), planner, 0.),
        ("raw_goal", (CellRect(399, -1, 401, 1),), (0., 0.), (4., 0.), planner, 1.),
    ]
    for i, (name, rectangles, start, goal, selected, budget) in enumerate(small):
        record = sample(name, WorldSnapshot(i, rectangles, ()), start, goal, selected, budget)
        summary[name] = summarize([record])
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if not all(s["validation"]["valid"] for s in all_samples):
        raise RuntimeError("independent validation failed; inspect samples.jsonl")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--suite", choices=("scale", "topology"), default="scale")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("repeats must be positive")
    run(args.output, args.repeats, args.suite)
