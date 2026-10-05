"""Frozen-input protocol for five fitters times three optional sample filters."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

LEGACY = Path(__file__).resolve().parents[1]/"terrain_confidence_comparison"
_spec = importlib.util.spec_from_file_location("terrain_confidence_legacy_protocol", LEGACY/"protocol.py")
_legacy = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _legacy
_spec.loader.exec_module(_legacy)
METHODS, legacy_cases = _legacy.METHODS, _legacy.cases

FILTERS = ("none", "median", "bilateral")
VARIANTS = tuple((method, filter_method) for method in METHODS for filter_method in FILTERS)
FILTER_PARAMETERS = {"radius_m": .10, "spatial_sigma_m": .075, "height_sigma_m": .03, "minimum_samples": 3}
TIMING_INPUTS = (("fit", "flat", "missing50", .05), ("fit", "flat", "noise005", .05),
                 ("fit", "flat", "noise015", .05), ("fit", "flat", "noise040", .05),
                 ("fit", "convex", "missing50", .05), ("fit", "convex", "noise015", .05),
                 ("fit", "rocks", "outlier05", .05), ("integrated", "flat", "missing50", .05),
                 ("integrated", "ditch15", "occlusion", .10))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_cases(source, suite="full", phase="all"):
    source = Path(source).resolve()
    manifest = json.loads((source/"manifest.json").read_text())
    if not manifest.get("complete") or manifest.get("freeze_mismatch"):
        raise ValueError("source experiment is not complete and frozen")
    selected = manifest["cases"]
    if suite == "quick":
        names = {case.name for case in legacy_cases("quick", phase)}
        selected = [case for case in selected if case["name"] in names]
    elif suite == "timing":
        selected = [case for case in selected
                    if (case["phase"], case["scene"], case["pattern"], case["resolution_m"]) in TIMING_INPUTS
                    and case["seed"] == 101]
        if len(selected) != 9: raise ValueError("timing requires nine preselected frozen inputs")
    elif suite != "full": raise ValueError(suite)
    if phase != "all": selected = [case for case in selected if case["phase"] == phase]
    output = []
    for case in selected:
        record = dict(case)
        # Exported legacy manifests retain execution-time /tmp paths. Resolve the
        # durable copied input by its case name rather than using that old anchor.
        record["input"] = str(source/"inputs"/(case["name"]+".f32"))
        record["truth"] = str(source/"inputs"/(case["name"]+"-truth.npz"))
        if digest(record["input"]) != record["sha256"]: raise ValueError("input SHA mismatch: "+case["name"])
        record["truth_sha256"] = digest(record["truth"])
        output.append(record)
    return output, manifest


def variants_for(case, suite="full"):
    if suite != "timing" and case["phase"] == "integrated" and case["pattern"] == "dense":
        return (("baseline", "none"),)
    return VARIANTS


def tasks_for(matrix, suite="full"):
    tasks = []
    repetitions = 3 if suite == "timing" else 1
    for i, case in enumerate(matrix):
        for repetition in range(repetitions):
            variants = list(variants_for(case, suite))
            offset = (i+repetition) % len(variants)
            for method, filter_method in variants[offset:]+variants[:offset]:
                tasks.append({"case": case, "method": method, "filter": filter_method, "repetition": repetition})
    return tasks


def prefix_for(directory, task):
    prefix = Path(directory)/"outputs"/task["case"]["name"]
    if task["repetition"]: prefix /= "repeat"+str(task["repetition"])
    return prefix/(task["method"]+"__"+task["filter"])
