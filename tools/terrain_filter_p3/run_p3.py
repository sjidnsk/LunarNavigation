#!/usr/bin/env python3
"""Serial P3 single-frame filter comparison after frozen main/timing completion."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

METHODS = ("baseline", "weighted_plane", "mls", "grouped_ransac", "gaussian_process")
FILTERS = ("none", "median", "bilateral")
VARIANTS = tuple((method, filter_method) for method in METHODS for filter_method in FILTERS)
HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
HISTORICAL = Path("/home/kai/CodexDownloads/lunar_navigation/terrain-fitting-comparison-20261005/p3")
INPUT_SHA256 = "6ca99369ee0173930d4e5375e56d43ad61c89fffb65b0e8cbb045d5e5f74f780"
CAPABILITIES_SHA256 = "b85130422802738980dac769cc09c79c8b7e6373027951d77004abf2d6c3ce30"
TIMING_FIELDS = {"map_ms", "plan_ms", "start_patch_ms", "fit_and_export_loop_ms", "repeat"}
SUFFIXES = ("-ground.bin", "-fine.i8", "-confidence.f32", "-path.csv")
PROFILE = {"platform": "foot", "n": 320, "resolution_m": .05,
           "planar_envelope_xy_m": [[-.34, -.165], [.34, -.165], [.34, .165], [-.34, .165]],
           "terrain_background_half_width_m": .2, "preferred_clearance_m": .4,
           "clearance_weight": 2., "slope_weight": 1., "relief_weight": 1.,
           "confidence_weight": 2., "minimum_confidence": .2, "maximum_residual_m": .03,
           "start_blind_zone_margin_m": 3., "deadline_seconds": 5,
           "start": [7.6133745667, 7.9915921022], "start_yaw_rad": -3.022577,
           "goal": [8.6133745667, 7.9915921022]}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def linked_core(probe):
    listing = subprocess.run(["ldd", str(probe)], text=True, capture_output=True, check=True).stdout
    for line in listing.splitlines():
        if "liblunar_incremental_navigation_core.so" in line and "=>" in line:
            path = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
            if path.is_file(): return {"path": str(path), "sha256": sha(path)}
    raise ValueError("cannot identify actual linked core library")


def file_sources(root):
    return sorted(path for path in root.rglob("*") if path.suffix in (".cpp", ".hpp", ".py")
                  and "__pycache__" not in path.parts)


def source_metadata():
    core = REPO/"ros2_ws/src/lunar_incremental_navigation_core"
    production = [path for root in (core/"src", core/"include") for path in file_sources(root)]
    return {"model_source_sha256": {str(p.relative_to(REPO)): sha(p) for p in production},
            "p3_source_sha256": {p.name: sha(p) for p in file_sources(HERE)}}


def prefix(directory, method, filter_method, repeat):
    return Path(directory)/(method+"__"+filter_method+"-repeat"+str(repeat))


def stable_summary(summary):
    return {key: value for key, value in summary.items() if key not in TIMING_FIELDS}


def historical_projection(current, historical):
    """Compare every historical semantic field while permitting new diagnostics."""
    if isinstance(historical, dict):
        return {key: historical_projection(current[key], value) for key, value in historical.items()
                if key not in TIMING_FIELDS}
    if isinstance(historical, list):
        if len(current) != len(historical): raise ValueError("historical list length changed")
        return [historical_projection(a, b) for a, b in zip(current, historical)]
    return current


def canonical_bytes(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def valid_output(output_prefix, method, filter_method, repeat):
    try:
        result = json.loads(Path(str(output_prefix)+"-summary.json").read_text())
        if (result["method"], result["filter"], result["repeat"], result["n"], result["resolution_m"]) != (method, filter_method, repeat, 320, .05): return False
        for field in ("confidence_weight", "minimum_confidence", "maximum_residual_m", "start_blind_zone_margin_m", "deadline_seconds", "start_yaw_rad"):
            if result[field] != PROFILE[field]: return False
        if result["start"]["position_m"] != [round(x, 11) for x in PROFILE["start"]]:
            # JSON uses 12 significant digits, so compare the fixed historical
            # pose geometrically rather than introducing a display transform.
            if any(abs(a-b)>1e-10 for a,b in zip(result["start"]["position_m"], PROFILE["start"])): return False
        if any(abs(a-b)>1e-10 for a,b in zip(result["goal"]["position_m"], PROFILE["goal"])): return False
        if result["fine_free_count"]+result["fine_blocked_count"]+result["fine_unknown_count"] != 320*320: return False
        if result["raw_observed_count"] != 17438 or result["missing_count"] != 84962: return False
        for suffix, size in (("-ground.bin", 320*320*6*4), ("-fine.i8", 320*320), ("-confidence.f32", 320*320*4)):
            if Path(str(output_prefix)+suffix).stat().st_size != size: return False
        with Path(str(output_prefix)+"-path.csv").open() as stream:
            count = sum(1 for _ in csv.DictReader(stream))
        if count != result["path_points"]: return False
        found = result["local_status"] == 0 and count > 0
        return result["plan_found"] == found and result["has_reference"] == (count > 0) and (not found or result["local_status_name"] == "PLAN_FOUND")
    except (OSError, ValueError, KeyError, TypeError): return False


def historical_hashes(historical):
    manifest = json.loads((historical/"results/manifest.json").read_text())
    if not manifest.get("complete") or manifest.get("expected_runs") != 15 or len(manifest.get("runs", [])) != 15:
        raise ValueError("historical P3 comparison is incomplete")
    if sha(historical/"raw.f32") != INPUT_SHA256 or sha(historical/"capabilities.hpp") != CAPABILITIES_SHA256:
        raise ValueError("historical P3 input/capabilities changed")
    paths = [historical/"results/manifest.json", historical/"fixed_compare.cpp", historical/"capabilities.hpp"]
    paths += [Path(str(historical/"results"/(method+"-repeat"+str(repeat)))+suffix)
              for method in METHODS for repeat in range(3) for suffix in SUFFIXES+("-summary.json",)]
    return {str(p.relative_to(historical)): sha(p) for p in paths}


def frozen_metadata(probe, input_path, historical):
    if input_path.stat().st_size != 320*320*4 or sha(input_path) != INPUT_SHA256:
        raise ValueError("input differs from frozen historical P3 bytes")
    if sha(HERE/"capabilities.hpp") != CAPABILITIES_SHA256: raise ValueError("capability changed")
    return {"input": str(input_path), "input_sha256": sha(input_path),
            "probe": str(probe), "probe_sha256": sha(probe), "linked_core": linked_core(probe),
            "historical": str(historical), "historical_sha256": historical_hashes(historical),
            "profile": PROFILE, "profile_sha256": hashlib.sha256(canonical_bytes(PROFILE)).hexdigest(),
            "capabilities_sha256": CAPABILITIES_SHA256, **source_metadata()}


def validate_completed_experiments(main_path, timing_path, frozen):
    main = json.loads((main_path/"manifest.json").read_text())
    timing = json.loads((timing_path/"manifest.json").read_text())
    for manifest, suite, count in ((main, "full", 20752), (timing, "timing", 405)):
        if not manifest.get("complete") or manifest.get("freeze_mismatch") or manifest.get("failures"):
            raise ValueError("main and serial timing must be complete before P3 starts")
        if manifest["config"]["suite"] != suite or manifest["config"]["phase"] != "all" or manifest["completed_runs"] != count:
            raise ValueError("main/timing scope or run count differs")
        if manifest["config"]["frozen"]["linked_core"] != frozen["linked_core"]:
            raise ValueError("P3 must use the exact actual core library of main/timing")
        for path, digest in frozen["model_source_sha256"].items():
            if manifest["config"]["frozen"]["source_sha256"].get(path) != digest:
                raise ValueError("P3 model source differs from main/timing: "+path)
    if timing["config"]["workers"] != 1: raise ValueError("timing was not serial")
    if not main.get("serial_timeout_recheck", {}).get("complete", False):
        raise ValueError("main serial timeout rechecks must finish before P3")
    return {"main_manifest_sha256": sha(main_path/"manifest.json"),
            "timing_manifest_sha256": sha(timing_path/"manifest.json")}


def verify_results(directory, historical, repeats):
    parity, equality = [], []
    for method, filter_method in VARIANTS:
        summaries = [json.loads(Path(str(prefix(directory, method, filter_method, r))+"-summary.json").read_text()) for r in range(repeats)]
        hashes = {suffix: [sha(str(prefix(directory, method, filter_method, r))+suffix) for r in range(repeats)] for suffix in SUFFIXES}
        summary_hashes = [hashlib.sha256(canonical_bytes(stable_summary(s))).hexdigest() for s in summaries]
        equality.append({"method": method, "filter": filter_method, "repeats": repeats,
                         "artifact_sha256": hashes, "same_bytes": all(len(set(v)) == 1 for v in hashes.values()),
                         "semantic_summary_sha256": summary_hashes, "same_semantic_summary": len(set(summary_hashes)) == 1})
        if filter_method != "none": continue
        for r in range(repeats):
            old_prefix = historical/"results"/(method+"-repeat"+str(r))
            new_prefix = prefix(directory, method, filter_method, r)
            old = json.loads(Path(str(old_prefix)+"-summary.json").read_text())
            artifacts = {suffix: sha(str(new_prefix)+suffix) == sha(str(old_prefix)+suffix) for suffix in SUFFIXES}
            projected = historical_projection(summaries[r], old)
            reference = historical_projection(old, old)
            parity.append({"method": method, "repeat": r, "artifact_byte_parity": artifacts,
                           "historical_semantic_summary_byte_parity": canonical_bytes(projected) == canonical_bytes(reference),
                           "excluded_time_or_id_fields": sorted(TIMING_FIELDS)})
    dump(directory/"none-parity.json", parity); dump(directory/"repeat-artifact-equality.json", equality)
    passed = all(all(p["artifact_byte_parity"].values()) and p["historical_semantic_summary_byte_parity"] for p in parity)
    stable = all(p["same_bytes"] and p["same_semantic_summary"] for p in equality)
    return {"none_parity_passed": passed, "none_runs_compared": len(parity),
            "all_fifteen_variants_repeat_stable": stable, "variants_compared": len(equality)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--historical", type=Path, default=HISTORICAL)
    parser.add_argument("--main-artifact", required=True, type=Path)
    parser.add_argument("--timing-artifact", required=True, type=Path)
    args = parser.parse_args()
    for key in ("probe", "input", "output", "historical", "main_artifact", "timing_artifact"):
        setattr(args, key, getattr(args, key).resolve())
    if args.output.exists(): raise SystemExit("choose a fresh output directory; earlier evidence is preserved")
    frozen = frozen_metadata(args.probe, args.input, args.historical)
    references = validate_completed_experiments(args.main_artifact, args.timing_artifact, frozen)
    args.output.mkdir(parents=True)
    manifest = {"schema": 1, "complete": False, "independent_frames": 1, "timing_repeats": 3,
                "workers": 1, "expected_runs": 45, "methods": METHODS, "filters": FILTERS,
                "frozen": frozen, "references": references, "runs": [], "created_unix": time.time(),
                "historical_frame_stamp": 1789097424.9736662,
                "boundaries": {"ground_truth": "unavailable; no accuracy or safety improvement claim",
                               "frame_pose_sync": "NOT_SYNCHRONIZED; pose about 2.2 seconds later",
                               "start_patch": "baseline support-plane fit and raw sample filtering; final patched FineCellEvaluator uses selected method/filter; assumed cells reported",
                               "path_counters": "exported vertices only; no swept-support claim",
                               "fit_and_export_loop_ms": "independent fitting plus counter/state reads, not pure fit time",
                               "ROS_Action_Humble_Orin_sensor_vehicle": "NOT_RUN"}}
    path = args.output/"manifest.json"; dump(path, manifest)
    try:
        for repeat in range(3):
            order = VARIANTS[repeat:]+VARIANTS[:repeat]
            for method, filter_method in order:
                if sha(args.input) != frozen["input_sha256"] or sha(args.probe) != frozen["probe_sha256"] or linked_core(args.probe) != frozen["linked_core"] or source_metadata() != {key: frozen[key] for key in ("model_source_sha256", "p3_source_sha256")}:
                    raise ValueError("input/probe/core/model/profile/driver changed during P3")
                output_prefix = prefix(args.output, method, filter_method, repeat)
                command = [str(args.probe), str(args.input), str(output_prefix), method, filter_method, str(repeat)]
                started = time.monotonic()
                process = subprocess.run(command, text=True, capture_output=True, timeout=300)
                Path(str(output_prefix)+"-stdout.txt").write_text(process.stdout)
                Path(str(output_prefix)+"-stderr.txt").write_text(process.stderr)
                record = {"method": method, "filter": filter_method, "repeat": repeat, "command": command,
                          "returncode": process.returncode, "wall_seconds": time.monotonic()-started,
                          "summary": str(output_prefix)+"-summary.json", "valid": process.returncode == 0 and valid_output(output_prefix, method, filter_method, repeat)}
                manifest["runs"].append(record); dump(path, manifest)
                if not record["valid"]: raise ValueError("probe/output failed: "+method+"/"+filter_method+"/"+str(repeat))
                result = json.loads(Path(record["summary"]).read_text())
                print(f"[{len(manifest['runs'])}/45] {method}/{filter_method}/{repeat}: status={result['local_status_name']} free={result['fine_free_count']} assumed={result['assumed_cells']}", flush=True)
        final = frozen_metadata(args.probe, args.input, args.historical)
        if final != frozen: raise ValueError("frozen P3 provenance changed before completion")
        if validate_completed_experiments(args.main_artifact, args.timing_artifact, frozen) != references:
            raise ValueError("main/timing reference manifests changed during P3")
        verification = verify_results(args.output, args.historical, 3)
        manifest.update(verification=verification, complete=len(manifest["runs"]) == 45 and all(r["valid"] for r in manifest["runs"]) and verification["none_parity_passed"] and verification["all_fifteen_variants_repeat_stable"],
                        final_frozen=final, finished_unix=time.time())
        dump(path, manifest)
        subprocess.run([sys.executable, str(HERE/"analyze_p3.py"), str(args.output)], check=True)
        if not manifest["complete"]: raise ValueError("P3 parity/stability failed; all original outputs retained")
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        manifest.update(complete=False, error=str(error), finished_unix=time.time()); dump(path, manifest)
        raise SystemExit(str(error))


if __name__ == "__main__": main()
