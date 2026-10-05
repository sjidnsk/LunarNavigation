#!/usr/bin/env python3
"""Generate frozen cases, run the C++ probe serially, and retain all raw outputs."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

from protocol import METHODS, cases, write_case


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def linked_core_metadata(probe):
    listing = subprocess.run(["ldd", str(probe)], capture_output=True, text=True, check=True).stdout
    for line in listing.splitlines():
        if "liblunar_incremental_navigation_core.so" in line and "=>" in line:
            library = Path(line.split("=>", 1)[1].strip().split()[0]).resolve()
            if library.is_file(): return {"path": str(library), "sha256": file_hash(library)}
    raise SystemExit("cannot identify the actual linked core library from ldd")


def source_metadata():
    tool_directory = Path(__file__).resolve().parent
    core = tool_directory.parents[1]/"ros2_ws"/"src"/"lunar_incremental_navigation_core"
    roots = (tool_directory, core/"src", core/"include")
    files = sorted(p for root in roots for p in root.rglob("*")
                   if p.suffix in (".cpp", ".hpp", ".py") and "__pycache__" not in p.parts)
    return {str(p.relative_to(tool_directory.parents[1])): file_hash(p) for p in files}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--suite", choices=("quick", "full"), default="full")
    parser.add_argument("--phase", choices=("fit", "integrated", "all"), default="all")
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--scene", nargs="+")
    parser.add_argument("--pattern", nargs="+")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-cases", type=int, default=0, help="Smoke run; manifest still states expected full count")
    parser.add_argument("--confidence-weight", type=float, default=2.)
    parser.add_argument("--minimum-confidence", type=float, default=.2)
    parser.add_argument("--maximum-residual", type=float, default=.03)
    parser.add_argument("--timeout-seconds", type=float, default=180.)
    args = parser.parse_args()
    args.output = args.output.resolve(); args.probe = args.probe.resolve()
    if not args.probe.is_file(): raise SystemExit("probe does not exist")
    args.output.mkdir(parents=True, exist_ok=True)
    case_list = [c for c in cases(args.suite, args.phase)
                 if (not args.scene or c.scene in args.scene)
                 and (not args.pattern or c.pattern in args.pattern)]
    config = dict(suite=args.suite, phase=args.phase, methods=args.methods, scenes=args.scene,
                  patterns=args.pattern, confidence_weight=args.confidence_weight,
                  minimum_confidence=args.minimum_confidence, maximum_residual=args.maximum_residual,
                  probe_sha256=file_hash(args.probe),
                  linked_core=linked_core_metadata(args.probe), source_sha256=source_metadata(),
                  protocol_sha256=file_hash(Path(__file__).with_name("protocol.py")),
                  expected_cases=len(case_list))
    manifest_path = args.output/"manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if not args.resume: raise SystemExit("artifact already exists; use --resume or choose a new output")
        if manifest["config"] != config: raise SystemExit("resume config or executable changed; choose a new artifact directory")
    else:
        manifest = {"schema": 1, "config": config, "cases": [write_case(c, args.output/"inputs") for c in case_list],
                    "created_unix": time.time(), "complete": False, "host": os.uname().nodename,
                    "probe": str(args.probe)}
        manifest["unique_input_sha256_count"] = len({c["sha256"] for c in manifest["cases"]})
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    # A resumed run is incomplete until its current artifacts and frozen inputs
    # have been checked; an interruption must not retain a stale success flag.
    manifest.update(complete=False, active_unix=time.time())
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    started = time.monotonic(); completed = 0; expected_runs = 0
    run_records = args.output/"run-records.jsonl"
    for i, (case, record) in enumerate(zip(case_list, manifest["cases"])):
        # Dense integrated map is a fixed baseline reference. Dense fit cases still
        # compare all methods. Rotating the method order reduces ordering bias.
        methods = ["baseline"] if case.phase == "integrated" and case.pattern == "dense" else args.methods
        methods = methods[i % len(methods):]+methods[:i % len(methods)]
        expected_runs += len(methods)
        if args.max_cases and i >= args.max_cases: continue
        for method in methods:
            prefix = args.output/"outputs"/case.name/method
            prefix.parent.mkdir(parents=True, exist_ok=True)
            summary_path = Path(str(prefix)+"-summary.json")
            required = [summary_path, Path(str(prefix)+"-ground.bin")]
            if case.phase == "integrated":
                required.extend(Path(str(prefix)+f"-{p}{suffix}") for p in ("foot", "wheel")
                                for suffix in (".bin", "-path.csv"))
            can_resume = args.resume and all(p.exists() for p in required)
            if can_resume:
                try:
                    data = json.loads(summary_path.read_text())
                    can_resume = data["method"] == method and data["n"] == case.n and data["mode"] == case.phase
                    can_resume = can_resume and Path(str(prefix)+"-ground.bin").stat().st_size == case.n*case.n*6*4
                    if case.phase == "integrated":
                        can_resume = can_resume and all(Path(str(prefix)+f"-{p}.bin").stat().st_size == case.n*case.n*2*4
                                                        and p in data["platforms"] for p in ("foot", "wheel"))
                except (KeyError, ValueError): can_resume = False
            if can_resume:
                completed += 1; continue
            command = [str(args.probe), record["input"], str(prefix), str(case.n), str(case.resolution_m),
                       method, case.phase, str(args.confidence_weight), str(args.minimum_confidence),
                       str(args.maximum_residual)]
            run_start = time.monotonic()
            process = subprocess.run(command, capture_output=True, text=True, timeout=args.timeout_seconds)
            Path(str(prefix)+"-stdout.txt").write_text(process.stdout)
            Path(str(prefix)+"-stderr.txt").write_text(process.stderr)
            result_record = {"case": case.name, "method": method, "command": command,
                             "returncode": process.returncode, "wall_seconds": time.monotonic()-run_start,
                             "finished_unix": time.time()}
            with run_records.open("a") as stream: stream.write(json.dumps(result_record)+"\n")
            if process.returncode: raise SystemExit(f"probe failed {case.name}/{method}: {process.stderr}")
            completed += 1
        print(f"[{i+1}/{len(case_list)}] {case.name} completed, elapsed {time.monotonic()-started:.1f}s", flush=True)
    # Validate expected byte counts as well as JSON, so truncated artifacts cannot
    # become completion evidence merely because a filename exists.
    verified = 0
    for case in case_list:
        methods = ["baseline"] if case.phase == "integrated" and case.pattern == "dense" else args.methods
        for method in methods:
            prefix = args.output/"outputs"/case.name/method
            summary_path = Path(str(prefix)+"-summary.json")
            ground_path = Path(str(prefix)+"-ground.bin")
            ok = summary_path.is_file() and ground_path.is_file() and ground_path.stat().st_size == case.n*case.n*6*4
            if ok:
                try:
                    data = json.loads(summary_path.read_text())
                    ok = data["method"] == method and data["n"] == case.n and data["mode"] == case.phase
                    if case.phase == "integrated":
                        for platform in ("foot", "wheel"):
                            binary = Path(str(prefix)+f"-{platform}.bin")
                            path = Path(str(prefix)+f"-{platform}-path.csv")
                            ok = ok and platform in data["platforms"] and binary.is_file() and binary.stat().st_size == case.n*case.n*2*4 and path.is_file()
                except (KeyError, ValueError): ok = False
            verified += int(ok)
    freeze_mismatch = []
    if file_hash(args.probe) != config["probe_sha256"]: freeze_mismatch.append("probe executable changed during run")
    try:
        if linked_core_metadata(args.probe) != config["linked_core"]: freeze_mismatch.append("linked core library changed during run")
    except (OSError, subprocess.CalledProcessError, SystemExit) as error:
        freeze_mismatch.append("cannot recheck linked core library: "+str(error))
    if source_metadata() != config["source_sha256"]: freeze_mismatch.append("algorithm or harness source changed during run")
    for record in manifest["cases"]:
        if file_hash(record["input"]) != record["sha256"]:
            freeze_mismatch.append("input bytes changed: "+record["name"])
    manifest.update(complete=verified == expected_runs and not freeze_mismatch, expected_runs=expected_runs, completed_runs=verified,
                    freeze_mismatch=freeze_mismatch,
                    finished_unix=time.time(), runner_wall_seconds=time.monotonic()-started)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+"\n")
    subprocess.run([os.sys.executable, str(Path(__file__).with_name("analyze.py")), str(args.output)], check=True)
    print(json.dumps({"complete": manifest["complete"], "completed_runs": verified, "expected_runs": expected_runs}))
    if freeze_mismatch: raise SystemExit("frozen experiment changed during execution; results retained, completion refused")


if __name__ == "__main__": main()
