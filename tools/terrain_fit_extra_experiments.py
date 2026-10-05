#!/usr/bin/env python3
"""Serial sensitivity and repeat timing over inputs from a completed frozen run.

This script intentionally lives outside terrain_confidence_comparison so creating
it does not alter the main run's source fingerprint. It never changes its inputs,
does not tune parameters, and does not invoke any Orin or vehicle interface.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from collections import defaultdict

import numpy as np

HARNESS = Path(__file__).resolve().with_name("terrain_confidence_comparison")
sys.path.insert(0, str(HARNESS))
from protocol import METHODS, SEEDS  # noqa: E402
from runner import file_hash, linked_core_metadata, source_metadata  # noqa: E402
from analyze import fit_metrics, grouped_fit, write_csv  # noqa: E402

SETTINGS = (("minconf010", .1, .03), ("minconf040", .4, .03),
            ("residual020", .2, .02), ("residual050", .2, .05))
FIT_TIMING = (("flat", "dense"), ("flat", "missing50"), ("flat", "noise015"),
              ("convex", "missing50"), ("rocks", "outlier05"), ("ditch15", "occlusion"))
INTEGRATED_TIMING = (("flat", "missing50"), ("convex", "missing50"), ("ditch15", "occlusion"))


def select_cases(main_manifest, phase):
    matrix = main_manifest["cases"]
    sensitivity = [c for c in matrix if c["phase"] == "fit" and c["scene"] in ("flat", "convex", "ditch15")
                   and c["pattern"] in ("noise015", "occlusion") and c["seed"] in SEEDS]
    fit_timing = [c for c in matrix if c["phase"] == "fit" and (c["scene"], c["pattern"]) in FIT_TIMING
                  and c["seed"] == (0 if c["pattern"] == "dense" else SEEDS[0])]
    integrated_timing = [c for c in matrix if c["phase"] == "integrated"
                         and (c["scene"], c["pattern"]) in INTEGRATED_TIMING and c["seed"] == SEEDS[0]]
    if phase in ("sensitivity", "all") and len(sensitivity) != 60:
        raise SystemExit(f"requires full matrix: expected 60 sensitivity cases, found {len(sensitivity)}")
    if phase in ("timing", "all") and (len(fit_timing), len(integrated_timing)) != (12, 6):
        raise SystemExit(f"requires full matrix: expected 12/6 timing inputs, found {len(fit_timing)}/{len(integrated_timing)}")
    tasks = []
    if phase in ("sensitivity", "all"):
        for i, case in enumerate(sensitivity):
            for setting, min_conf, residual in SETTINGS:
                order = list(METHODS[i % len(METHODS):])+list(METHODS[:i % len(METHODS)])
                for method in order:
                    tasks.append(dict(experiment="sensitivity", case=case, method=method, setting=setting,
                                      minimum_confidence=min_conf, maximum_residual_m=residual, repetition=0))
    if phase in ("timing", "all"):
        for i, case in enumerate(fit_timing+integrated_timing):
            for repetition in range(3):
                offset = (i+repetition) % len(METHODS)
                for method in list(METHODS[offset:])+list(METHODS[:offset]):
                    tasks.append(dict(experiment="timing", case=case, method=method, setting="default",
                                      minimum_confidence=.2, maximum_residual_m=.03, repetition=repetition))
    return tasks


def prefix_for(directory, task):
    return directory/"outputs"/task["experiment"]/task["case"]["name"]/(
        task["setting"]+"__repeat"+str(task["repetition"]))/task["method"]


def valid_output(prefix, task):
    case = task["case"]
    try:
        summary = json.loads(Path(str(prefix)+"-summary.json").read_text())
        if summary["method"] != task["method"] or summary["n"] != case["n"] or summary["mode"] != case["phase"]:
            return False
        if abs(summary["minimum_confidence"]-task["minimum_confidence"]) > 1e-12 or abs(summary["maximum_residual_m"]-task["maximum_residual_m"]) > 1e-12:
            return False
        if Path(str(prefix)+"-ground.bin").stat().st_size != case["n"]**2*6*4: return False
        if case["phase"] == "integrated":
            for platform in ("foot", "wheel"):
                if platform not in summary["platforms"]: return False
                if Path(str(prefix)+f"-{platform}.bin").stat().st_size != case["n"]**2*2*4: return False
                if not Path(str(prefix)+f"-{platform}-path.csv").is_file(): return False
        return True
    except (OSError, ValueError, KeyError): return False


def frozen_metadata(probe, main, tasks):
    selected = {t["case"]["name"]: t["case"] for t in tasks}
    return {"probe_sha256": file_hash(probe), "linked_core": linked_core_metadata(probe),
            "source_sha256": source_metadata(), "extra_script_sha256": file_hash(__file__),
            "main_manifest_sha256": file_hash(main/"manifest.json"),
            "input_sha256": {name: file_hash(record["input"]) for name, record in selected.items()},
            "truth_sha256": {name: file_hash(main/"inputs"/(name+"-truth.npz")) for name in selected}}


def read_case(main, case):
    shape = (case["n"], case["n"])
    observed = np.fromfile(case["input"], "<f4").reshape(shape)
    with np.load(main/"inputs"/(case["name"]+"-truth.npz")) as data:
        truth_data = {key: data[key] for key in data.files}
    return observed, truth_data


def summarize(main, directory, tasks, complete):
    sensitivity_rows, confidence_bins, timing_rows = [], [], []
    cache = {}
    for task in tasks:
        prefix = prefix_for(directory, task)
        if not valid_output(prefix, task): continue
        case = task["case"]; summary = json.loads(Path(str(prefix)+"-summary.json").read_text())
        identity = {k: case[k] for k in ("scene", "pattern", "seed", "resolution_m", "n")}
        identity.update(case=case["name"], method=task["method"], experiment=task["experiment"],
                        setting=task["setting"], repetition=task["repetition"], mode=case["phase"])
        if task["experiment"] == "timing":
            timing_rows.append({**identity, "platform": "fit", "fit_ms": summary["fit_ms"]})
            for platform, values in summary["platforms"].items():
                timing_rows.append({**identity, "platform": platform, **values})
            continue
        if case["name"] not in cache:
            observed, truth_data = read_case(main, case)
            defaults = {}
            for method in METHODS:
                path = main/"outputs"/case["name"]/(method+"-ground.bin")
                defaults[method] = np.fromfile(path, "<f4").reshape(observed.shape+(6,))
            cache[case["name"]] = (observed, truth_data, defaults)
        observed, truth_data, defaults = cache[case["name"]]
        ground = np.fromfile(str(prefix)+"-ground.bin", "<f4").reshape(observed.shape+(6,))
        record = {**identity, "phase": "sensitivity", "fit_ms": summary["fit_ms"],
                  "minimum_confidence": task["minimum_confidence"], "maximum_residual_m": task["maximum_residual_m"]}
        metrics, bins = fit_metrics(record, observed, ground, truth_data)
        own_default, _ = fit_metrics({}, observed, defaults[task["method"]], truth_data)
        metrics.update(default_method_accepted_coverage=own_default["accepted_coverage"],
                       default_method_height_rmse_m=own_default["height_rmse_m"],
                       coverage_delta_vs_default=metrics["accepted_coverage"]-own_default["accepted_coverage"])
        baseline = defaults["baseline"][:, :, 0]
        common = truth_data["interior"] & ~np.isfinite(observed) & np.isfinite(baseline) & np.isfinite(ground[:, :, 0])
        errors = ground[:, :, 0][common]-truth_data["height"][common]
        baseline_errors = baseline[common]-truth_data["height"][common]
        metrics.update(default_baseline_common_count=int(common.sum()),
                       default_baseline_common_method_sq_sum=float(np.dot(errors, errors)),
                       default_baseline_common_baseline_sq_sum=float(np.dot(baseline_errors, baseline_errors)),
                       default_baseline_common_method_rmse_m=float(np.sqrt(np.mean(errors**2))) if len(errors) else None,
                       default_baseline_common_baseline_rmse_m=float(np.sqrt(np.mean(baseline_errors**2))) if len(errors) else None)
        sensitivity_rows.append(metrics); confidence_bins.extend(bins)
    timing_groups = defaultdict(list)
    for row in timing_rows:
        key = tuple(row[k] for k in ("mode", "scene", "pattern", "resolution_m", "method", "platform"))
        timing_groups[key].append(row)
    timing_aggregate = []
    for key, rows in timing_groups.items():
        values = dict(zip(("mode", "scene", "pattern", "resolution_m", "method", "platform"), key))
        values.update(repeats=len(rows))
        for metric in ("fit_ms", "map_ms", "plan_ms"):
            data = [r[metric] for r in rows if metric in r]
            if data:
                values.update({metric+"_mean": float(np.mean(data)), metric+"_median": float(np.median(data)),
                               metric+"_p95": float(np.quantile(data, .95)), metric+"_min": float(np.min(data)),
                               metric+"_max": float(np.max(data))})
        if key[-1] != "fit":
            values.update(statuses=[r["status"] for r in rows], reached=[r["reached"] for r in rows],
                          timeout_count=sum(r["status"] == 2 for r in rows))
        timing_aggregate.append(values)
    sensitivity_aggregate = grouped_fit(sensitivity_rows, ("setting", "method", "resolution_m"))
    for aggregate in sensitivity_aggregate:
        group = [r for r in sensitivity_rows if all(r[k] == aggregate[k] for k in ("setting", "method", "resolution_m"))]
        count = sum(r["default_baseline_common_count"] for r in group)
        aggregate.update(default_baseline_common_count=count,
                         default_baseline_common_method_rmse_m=float(np.sqrt(sum(r["default_baseline_common_method_sq_sum"] for r in group)/count)) if count else None,
                         default_baseline_common_baseline_rmse_m=float(np.sqrt(sum(r["default_baseline_common_baseline_sq_sum"] for r in group)/count)) if count else None)
    for name, rows in (("sensitivity-results", sensitivity_rows), ("sensitivity-aggregate", sensitivity_aggregate),
                       ("sensitivity-confidence-bins", confidence_bins), ("timing-results", timing_rows),
                       ("timing-aggregate", timing_aggregate)):
        (directory/(name+".json")).write_text(json.dumps(rows, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        write_csv(directory/(name+".csv"), rows)
    result = {"complete": complete, "sensitivity": sensitivity_aggregate, "timing": timing_aggregate,
              "boundaries": {"sensitivity": "fit estimator only; does not alter platform thresholds or select tuned parameters",
                             "timing": "fresh process per repeat; serial method order rotated; background jobs may affect timing",
                             "truth": "same frozen main inputs and analytic truth; deterministic occlusion seed repeats are not independent",
                             "cold_warm": "no reused process/model cache across repeats; does not establish embedded cold/warm latency",
                             "hardware": "local host only; Humble/Orin/DDS/sensor/vehicle NOT_RUN"}}
    (directory/"summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--main", required=True, type=Path)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--phase", choices=("sensitivity", "timing", "all"), default="all")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--timeout-seconds", type=float, default=180.)
    args = parser.parse_args()
    args.main = args.main.resolve(); args.probe = args.probe.resolve(); args.output = args.output.resolve()
    baseline_manifest = json.loads((args.main/"manifest.json").read_text())
    if not baseline_manifest.get("complete") or baseline_manifest.get("freeze_mismatch"):
        raise SystemExit("main artifact must be completed and frozen")
    tasks = select_cases(baseline_manifest, args.phase)
    frozen = frozen_metadata(args.probe, args.main, tasks)
    main_config = baseline_manifest["config"]
    for key in ("probe_sha256", "linked_core", "source_sha256"):
        if frozen[key] != main_config[key]: raise SystemExit("current executable/library/source differs from main artifact: "+key)
    selected = {t["case"]["name"]: t["case"] for t in tasks}
    for name, record in selected.items():
        if frozen["input_sha256"][name] != record["sha256"]: raise SystemExit("main input changed: "+name)
    config = {"phase": args.phase, "main": str(args.main), "frozen": frozen, "settings": SETTINGS,
              "expected_runs": len(tasks), "methods": list(METHODS), "timing_repetitions": 3,
              "default_confidence_weight": 2.}
    # JSON round trip normalizes tuples so resume comparisons match saved values.
    config = json.loads(json.dumps(config))
    args.output.mkdir(parents=True, exist_ok=True); manifest_path = args.output/"manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if not args.resume: raise SystemExit("artifact already exists; choose new output or --resume")
        if manifest["config"] != config: raise SystemExit("resume fingerprint or configuration changed")
    else:
        manifest = {"schema": 1, "config": config, "created_unix": time.time(), "host": os.uname().nodename,
                    "unique_input_sha256_count": len(set(frozen["input_sha256"].values())),
                    "sensitivity_expected_runs": sum(t["experiment"] == "sensitivity" for t in tasks),
                    "timing_expected_runs": sum(t["experiment"] == "timing" for t in tasks)}
    manifest.update(complete=False, active_unix=time.time(), freeze_mismatch=[])
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    started = time.monotonic()
    for i, task in enumerate(tasks):
        prefix = prefix_for(args.output, task); prefix.parent.mkdir(parents=True, exist_ok=True)
        if args.resume and valid_output(prefix, task): continue
        case = task["case"]
        if file_hash(case["input"]) != case["sha256"]:
            manifest["freeze_mismatch"].append("input changed before invocation: "+case["name"])
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
            raise SystemExit("frozen input changed before invocation")
        command = [str(args.probe), case["input"], str(prefix), str(case["n"]), str(case["resolution_m"]),
                   task["method"], case["phase"], "2", str(task["minimum_confidence"]), str(task["maximum_residual_m"])]
        call_start = time.monotonic()
        process = subprocess.run(command, capture_output=True, text=True, timeout=args.timeout_seconds)
        Path(str(prefix)+"-stdout.txt").write_text(process.stdout); Path(str(prefix)+"-stderr.txt").write_text(process.stderr)
        record = {"case": case["name"], "input_sha256": case["sha256"], "experiment": task["experiment"],
                  "setting": task["setting"], "repetition": task["repetition"], "method": task["method"],
                  "command": command, "returncode": process.returncode, "wall_seconds": time.monotonic()-call_start}
        with (args.output/"run-records.jsonl").open("a") as stream: stream.write(json.dumps(record)+"\n")
        if process.returncode or not valid_output(prefix, task): raise SystemExit("probe failed or incomplete: "+str(prefix))
        if (i+1) % 25 == 0: print(f"[{i+1}/{len(tasks)}] elapsed {time.monotonic()-started:.1f}s", flush=True)
    verified = sum(valid_output(prefix_for(args.output, t), t) for t in tasks)
    fresh = frozen_metadata(args.probe, args.main, tasks)
    mismatches = [key+" changed during run" for key in frozen if fresh[key] != frozen[key]]
    manifest.update(completed_runs=verified, complete=verified == len(tasks) and not mismatches,
                    freeze_mismatch=mismatches, finished_unix=time.time(), wall_seconds=time.monotonic()-started)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    summarize(args.main, args.output, tasks, manifest["complete"])
    print(json.dumps({"complete": manifest["complete"], "completed_runs": verified, "expected_runs": len(tasks)}))
    if mismatches: raise SystemExit("frozen experiment changed; retained artifacts are incomplete")


if __name__ == "__main__": main()
