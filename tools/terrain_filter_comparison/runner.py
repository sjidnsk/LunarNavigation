#!/usr/bin/env python3
"""Frozen correctness matrix; parallel raw timing is not performance evidence."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from protocol import FILTER_PARAMETERS, LEGACY, VARIANTS, digest, load_cases, prefix_for, tasks_for


def linked_core(probe):
    output = subprocess.run(["ldd", str(probe)], capture_output=True, text=True, check=True).stdout
    for line in output.splitlines():
        if "liblunar_incremental_navigation_core.so" in line and "=>" in line:
            path = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
            if path.is_file(): return {"path": str(path), "sha256": digest(path)}
    raise ValueError("cannot identify actual linked core library")


def source_hashes():
    directory = Path(__file__).resolve().parent; repo = directory.parents[1]
    core = repo/"ros2_ws/src/lunar_incremental_navigation_core"
    files = sorted(path for root in (directory, LEGACY, core/"src", core/"include") for path in root.rglob("*")
                   if path.suffix in (".py", ".hpp", ".cpp") and "__pycache__" not in path.parts)
    return {str(path.relative_to(repo)): digest(path) for path in files}


def frozen_metadata(probe, source, matrix):
    return {"probe_sha256": digest(probe), "linked_core": linked_core(probe), "source_sha256": source_hashes(),
            "source_manifest_sha256": digest(source/"manifest.json"),
            "input_sha256": {c["name"]: digest(c["input"]) for c in matrix},
            "truth_sha256": {c["name"]: digest(c["truth"]) for c in matrix}}


def valid_output(prefix, task):
    case = task["case"]
    try:
        summary = json.loads(Path(str(prefix)+"-summary.json").read_text())
        if summary["method"] != task["method"] or summary["filter"] != task["filter"]: return False
        if summary["n"] != case["n"] or summary["mode"] != case["phase"]: return False
        if not math.isclose(summary["resolution_m"], case["resolution_m"], rel_tol=0., abs_tol=1e-12): return False
        if summary["minimum_confidence"] != .2 or summary["maximum_residual_m"] != .03 or summary["confidence_weight"] != 2.: return False
        if Path(str(prefix)+"-ground.bin").stat().st_size != case["n"]**2*6*4: return False
        if case["phase"] == "integrated":
            for platform in ("foot", "wheel"):
                if platform not in summary["platforms"]: return False
                if Path(str(prefix)+f"-{platform}.bin").stat().st_size != case["n"]**2*2*4: return False
                if not Path(str(prefix)+f"-{platform}-path.csv").is_file(): return False
        return True
    except (OSError, ValueError, KeyError, TypeError): return False


def invoke(probe, directory, task, timeout):
    prefix = prefix_for(directory, task); prefix.parent.mkdir(parents=True, exist_ok=True)
    case = task["case"]
    command = [str(probe), case["input"], str(prefix), str(case["n"]), str(case["resolution_m"]),
               task["method"], case["phase"], "2", ".2", ".03", task["filter"]]
    start = time.monotonic()
    record = {"case": case["name"], "input_sha256": case["sha256"], "method": task["method"],
              "filter": task["filter"], "repetition": task["repetition"], "command": command,
              "started_unix": time.time()}
    try:
        if digest(case["input"]) != case["sha256"]: raise ValueError("input changed before invocation")
        process = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        Path(str(prefix)+"-stdout.txt").write_text(process.stdout)
        Path(str(prefix)+"-stderr.txt").write_text(process.stderr)
        record.update(returncode=process.returncode, valid=process.returncode == 0 and valid_output(prefix, task))
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        record.update(returncode=None, valid=False, error=str(error))
    record.update(wall_seconds=time.monotonic()-start, finished_unix=time.time())
    return record


def validate_timing_reference(reference, frozen, matrix):
    """Reject smoke/partial/unfrozen or differently sourced timing controls."""
    config = reference.get("config", {})
    if config.get("suite") != "full" or config.get("phase") != "all":
        raise ValueError("timing reference must be the full all-phase matrix")
    if not reference.get("complete") or reference.get("freeze_mismatch") or reference.get("failures"):
        raise ValueError("timing reference is incomplete or has frozen/execution failures")
    if reference.get("expected_runs") != 20752 or reference.get("completed_runs") != 20752:
        raise ValueError("timing reference does not contain every expected main run")
    reference_frozen = config.get("frozen", {})
    for key in ("probe_sha256", "linked_core", "source_sha256"):
        if reference_frozen.get(key) != frozen[key]: raise ValueError("timing implementation differs from main: "+key)
    for case in matrix:
        for key in ("input_sha256", "truth_sha256"):
            if reference_frozen.get(key, {}).get(case["name"]) != frozen[key][case["name"]]:
                raise ValueError("timing frozen case differs from main: "+key+"/"+case["name"])


def none_parity(source, output, tasks):
    result = {"ground_checked": 0, "state_checked": 0, "mismatches": []}
    for task in tasks:
        if task["filter"] != "none" or task["repetition"]: continue
        prefix = prefix_for(output, task)
        old_prefix = source/"outputs"/task["case"]["name"]/task["method"]
        for suffix, kind in (("-ground.bin", "ground"),):
            new, old = Path(str(prefix)+suffix), Path(str(old_prefix)+suffix)
            if not new.is_file() or not old.is_file():
                result["mismatches"].append({"case": task["case"]["name"], "method": task["method"], "kind": kind, "reason": "missing output"})
                continue
            result[kind+"_checked"] += 1
            if digest(new) != digest(old): result["mismatches"].append({"case": task["case"]["name"], "method": task["method"], "kind": kind})
        if task["case"]["phase"] == "integrated":
            for platform in ("foot", "wheel"):
                new, old = Path(str(prefix)+f"-{platform}.bin"), Path(str(old_prefix)+f"-{platform}.bin")
                result["state_checked"] += 1
                if not new.is_file() or not old.is_file() or digest(new) != digest(old):
                    result["mismatches"].append({"case": task["case"]["name"], "method": task["method"], "kind": platform})
    result["passed"] = not result["mismatches"]
    return result


def timeout_cases(output, matrix, tasks):
    names = set()
    for task in tasks:
        if task["case"]["phase"] != "integrated": continue
        prefix = prefix_for(output, task)
        if not valid_output(prefix, task): continue
        summary = json.loads(Path(str(prefix)+"-summary.json").read_text())
        if any(p["status"] == 2 for p in summary["platforms"].values()): names.add(task["case"]["name"])
    return [case for case in matrix if case["name"] in names]


def serial_timeout_recheck(probe, output, matrix, timeout, resume):
    directory = output/"timeout-rechecks"; directory.mkdir(exist_ok=True)
    tasks = [{"case": case, "method": method, "filter": filter_method, "repetition": 0}
             for case in matrix for method, filter_method in VARIANTS]
    meta = {"workers": 1, "expected_runs": len(tasks), "cases": matrix, "complete": False,
            "scope": "all fifteen combinations for every primary TIMEOUT case; original results are retained"}
    (directory/"manifest.json").write_text(json.dumps(meta, indent=2)+"\n")
    for task in tasks:
        if resume and valid_output(prefix_for(directory, task), task): continue
        record = invoke(probe, directory, task, timeout)
        with (directory/"run-records.jsonl").open("a") as stream: stream.write(json.dumps(record)+"\n")
    meta.update(completed_runs=sum(valid_output(prefix_for(directory, t), t) for t in tasks))
    meta["complete"] = meta["completed_runs"] == len(tasks)
    (directory/"manifest.json").write_text(json.dumps(meta, indent=2)+"\n")
    return meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--suite", choices=("quick", "full", "timing"), default="full")
    parser.add_argument("--phase", choices=("fit", "integrated", "all"), default="all")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--reference", type=Path, help="Completed new full artifact required before serial timing")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-cases", type=int, default=0)
    parser.add_argument("--timeout-seconds", type=float, default=180.)
    args = parser.parse_args()
    if args.workers < 1: parser.error("workers must be positive")
    if args.suite == "timing" and (args.workers != 1 or args.reference is None):
        parser.error("timing requires --workers 1 and --reference completed-new-full")
    args.source = args.source.resolve(); args.probe = args.probe.resolve(); args.output = args.output.resolve()
    matrix, old_manifest = load_cases(args.source, args.suite, args.phase)
    tasks = tasks_for(matrix, args.suite)
    frozen = frozen_metadata(args.probe, args.source, matrix)
    reference_hash = None
    if args.suite == "timing":
        args.reference = args.reference.resolve()
        reference_path = args.reference/"manifest.json"
        reference_hash = digest(reference_path)
        validate_timing_reference(json.loads(reference_path.read_text()), frozen, matrix)
    config = {"suite": args.suite, "phase": args.phase, "workers": args.workers,
              "source": str(args.source), "frozen": frozen, "filter_parameters": FILTER_PARAMETERS,
              "minimum_confidence": .2, "maximum_residual_m": .03, "confidence_weight": 2.,
              "expected_runs": len(tasks), "method_filter_rotation": "case index plus repetition modulo fifteen variants"}
    if reference_hash is not None:
        config.update(reference=str(args.reference), reference_manifest_sha256=reference_hash)
    args.output.mkdir(parents=True, exist_ok=True); manifest_path = args.output/"manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if not args.resume: raise SystemExit("artifact exists; choose new output or --resume")
        if manifest["config"] != config: raise SystemExit("resume fingerprint/configuration changed")
    else:
        manifest = {"schema": 1, "config": config, "cases": matrix, "created_unix": time.time(),
                    "source_unique_inputs": old_manifest.get("unique_input_sha256_count"),
                    "unique_input_sha256_count": len({c["sha256"] for c in matrix}), "host": os.uname().nodename}
    manifest.update(complete=False, active_unix=time.time(), load_average_start=list(os.getloadavg()))
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    selected_names = {c["name"] for c in (matrix[:args.max_cases] if args.max_cases else matrix)}
    pending = [t for t in tasks if t["case"]["name"] in selected_names
               and not (args.resume and valid_output(prefix_for(args.output, t), t))]
    start = time.monotonic(); failed = []
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = [executor.submit(invoke, args.probe, args.output, t, args.timeout_seconds) for t in pending]
        for i, future in enumerate(as_completed(futures)):
            record = future.result()
            with (args.output/"run-records.jsonl").open("a") as stream: stream.write(json.dumps(record)+"\n")
            if not record["valid"]: failed.append(record)
            if (i+1) % 100 == 0: print(f"[{i+1}/{len(pending)}] {time.monotonic()-start:.1f}s, failed={len(failed)}", flush=True)
    verified = sum(valid_output(prefix_for(args.output, t), t) for t in tasks)
    fresh = frozen_metadata(args.probe, args.source, matrix)
    mismatch = [key+" changed during execution" for key in frozen if fresh[key] != frozen[key]]
    if reference_hash is not None and digest(args.reference/"manifest.json") != reference_hash:
        mismatch.append("timing reference manifest changed during execution")
    parity = none_parity(args.source, args.output, tasks) if verified == len(tasks) else {"passed": False, "reason": "incomplete matrix"}
    recheck = {"complete": True, "expected_runs": 0, "completed_runs": 0}
    timed_out = timeout_cases(args.output, matrix, tasks)
    if verified == len(tasks) and not mismatch and parity["passed"] and timed_out and args.suite != "timing":
        recheck = serial_timeout_recheck(args.probe, args.output, timed_out, args.timeout_seconds, args.resume)
        final = frozen_metadata(args.probe, args.source, matrix)
        mismatch.extend(key+" changed during serial timeout recheck" for key in frozen if final[key] != frozen[key])
    manifest.update(completed_runs=verified, expected_runs=len(tasks), failures=failed, freeze_mismatch=mismatch,
                    none_parity=parity, primary_timeout_case_count=len(timed_out), serial_timeout_recheck=recheck,
                    complete=verified == len(tasks) and not failed and not mismatch and parity["passed"] and recheck["complete"],
                    finished_unix=time.time(), runner_wall_seconds=time.monotonic()-start,
                    load_average_end=list(os.getloadavg()),
                    timing_boundary="parallel correctness raw timings are not performance evidence" if args.workers > 1 else "serial fresh processes; local host background load remains uncontrolled")
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    subprocess.run([sys.executable, str(Path(__file__).with_name("analyze.py")), str(args.output)], check=True)
    print(json.dumps({"complete": manifest["complete"], "completed_runs": verified, "expected_runs": len(tasks),
                      "none_parity_passed": parity["passed"], "primary_timeout_cases": len(timed_out)}))
    if mismatch or (verified == len(tasks) and not parity["passed"]):
        raise SystemExit("frozen/parity checks failed; artifacts retained and completion blocked")
    if not manifest["complete"] and not args.max_cases:
        raise SystemExit("execution incomplete or failed; artifacts retained and completion blocked")


if __name__ == "__main__": main()
