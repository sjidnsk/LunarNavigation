#!/usr/bin/env python3
"""Filter effects at matched targets; observed-noise diagnostics stay separate."""
from __future__ import annotations

import argparse
from collections import defaultdict
import importlib.util
import json
from pathlib import Path

import numpy as np

from protocol import FILTERS, LEGACY, METHODS, VARIANTS, prefix_for, tasks_for
from runner import valid_output

_spec = importlib.util.spec_from_file_location("terrain_confidence_legacy_analysis", LEGACY/"analyze.py")
_legacy = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_legacy)


def observed_metrics(observed, ground, truth_data):
    measured = np.isfinite(observed)&truth_data["interior"]
    accepted = measured&np.isfinite(ground[:, :, 0])
    estimate_error = ground[:, :, 0][accepted]-truth_data["height"][accepted]
    raw_error = observed[accepted]-truth_data["height"][accepted]
    count = int(accepted.sum()); total = int(measured.sum())
    estimate_sq, raw_sq = float(np.dot(estimate_error, estimate_error)), float(np.dot(raw_error, raw_error))
    return {"observed_total_count": total, "observed_estimate_count": count,
            "observed_estimate_accepted_rate": count/total if total else None,
            "observed_estimate_sq_sum": estimate_sq, "observed_raw_same_accepted_sq_sum": raw_sq,
            "observed_estimate_rmse_m": float(np.sqrt(estimate_sq/count)) if count else None,
            "raw_observed_rmse_same_accepted_m": float(np.sqrt(raw_sq/count)) if count else None}


def common_metrics(candidate, reference, truth_height, mask):
    accepted = mask&np.isfinite(candidate[:, :, 0])&np.isfinite(reference[:, :, 0])
    a = candidate[:, :, 0][accepted]-truth_height[accepted]
    b = reference[:, :, 0][accepted]-truth_height[accepted]
    count = int(accepted.sum()); a_sq = float(np.dot(a, a)); b_sq = float(np.dot(b, b))
    a_rmse = float(np.sqrt(a_sq/count)) if count else None
    b_rmse = float(np.sqrt(b_sq/count)) if count else None
    return {"count": count, "candidate_sq_sum": a_sq, "reference_sq_sum": b_sq,
            "candidate_rmse_m": a_rmse, "reference_rmse_m": b_rmse,
            "rmse_delta_m": a_rmse-b_rmse if count else None}


def filter_bootstrap(rows, field):
    by_case = defaultdict(dict)
    for row in rows:
        if row["phase"] == "fit" and row["pattern"] not in ("dense", "occlusion", "stripes"):
            by_case[row["case"]][row["method"], row["filter"]] = row
    result = []
    for method in METHODS:
        for filter_method in FILTERS[1:]:
            seed_values = defaultdict(list); pairs = 0
            for case in by_case.values():
                none, candidate = case.get((method, "none")), case.get((method, filter_method))
                if none is None or candidate is None: continue
                a, b = candidate.get(field), none.get(field)
                if a is None or b is None: continue
                seed_values[candidate["seed"]].append(a-b); pairs += 1
            clusters = np.array([np.mean(v) for v in seed_values.values()])
            ci = None
            if len(clusters) > 1:
                draws = np.random.default_rng(1601).choice(clusters, (10000, len(clusters)), replace=True).mean(axis=1)
                ci = [float(x) for x in np.quantile(draws, (.025, .975))]
            result.append({"method": method, "filter": filter_method, "reference_filter": "none", "field": field,
                           "paired_cases": pairs, "seed_clusters": len(clusters),
                           "delta": float(clusters.mean()) if len(clusters) else None, "ci95": ci})
    return result


def common_bootstrap(rows):
    result = []
    for method in METHODS:
        for filter_method in FILTERS[1:]:
            seeds = defaultdict(list)
            for row in rows:
                if row["phase"] == "fit" and row["method"] == method and row["filter"] == filter_method and row["pattern"] not in ("dense", "occlusion", "stripes") and row["rmse_delta_m"] is not None:
                    seeds[row["seed"]].append(row["rmse_delta_m"])
            values = np.array([np.mean(v) for v in seeds.values()]); ci = None
            if len(values) > 1:
                draws = np.random.default_rng(1601).choice(values, (10000, len(values)), replace=True).mean(axis=1)
                ci = [float(x) for x in np.quantile(draws, (.025, .975))]
            result.append({"method": method, "filter": filter_method, "reference_filter": "none",
                           "paired_cases": sum(len(v) for v in seeds.values()), "seed_clusters": len(values),
                           "delta": float(values.mean()) if len(values) else None, "ci95": ci})
    return result


def fit_aggregate(rows, keys):
    output = _legacy.grouped_fit(rows, keys)
    groups = defaultdict(list)
    for row in rows: groups[tuple(row[k] for k in keys)].append(row)
    for aggregate in output:
        group = groups[tuple(aggregate[k] for k in keys)]
        measured = sum(r["observed_total_count"] for r in group); accepted = sum(r["observed_estimate_count"] for r in group)
        aggregate.update(observed_total_count=measured, observed_estimate_count=accepted,
                         observed_estimate_accepted_rate=accepted/measured if measured else None,
                         observed_estimate_rmse_m=float(np.sqrt(sum(r["observed_estimate_sq_sum"] for r in group)/accepted)) if accepted else None,
                         raw_observed_rmse_same_accepted_m=float(np.sqrt(sum(r["observed_raw_same_accepted_sq_sum"] for r in group)/accepted)) if accepted else None)
    return output


def common_aggregate(rows, keys, candidate_key="candidate_sq_sum", reference_key="reference_sq_sum"):
    groups = defaultdict(list)
    for row in rows: groups[tuple(row[k] for k in keys)].append(row)
    output = []
    for key, group in groups.items():
        count = sum(r["count"] for r in group)
        candidate = float(np.sqrt(sum(r[candidate_key] for r in group)/count)) if count else None
        reference = float(np.sqrt(sum(r[reference_key] for r in group)/count)) if count else None
        output.append({**dict(zip(keys, key)), "cases": len(group), "empty_cases": sum(r["count"] == 0 for r in group),
                       "count": count, "candidate_rmse_m": candidate, "reference_rmse_m": reference,
                       "rmse_delta_m": candidate-reference if count else None})
    return output


def platform_aggregate(rows):
    packed = [{**row, "method": row["method"]+"__"+row["filter"]} for row in rows]
    output = _legacy.grouped_platform(packed)
    for row in output: row["method"], row["filter"] = row["method"].split("__", 1)
    return output


def recheck_platform_results(directory, manifest):
    """Keep serial timeout confirmation separate from the primary population."""
    rows = []
    for case in manifest.get("serial_timeout_recheck", {}).get("cases", []):
        shape = (case["n"], case["n"])
        observed = np.fromfile(case["input"], "<f4").reshape(shape)
        with np.load(case["truth"]) as data: truth_data = {k: data[k] for k in data.files}
        reference_case = f"integrated__{case['scene']}__dense__0__r{round(case['resolution_m']*1000)}"
        dense = {}
        for platform in ("foot", "wheel"):
            path = directory/"outputs"/reference_case/("baseline__none-"+platform+".bin")
            if path.exists(): dense[platform] = np.fromfile(path, "<f4").reshape(shape+(2,))[:, :, 0]
        for method, filter_method in VARIANTS:
            task = {"case": case, "method": method, "filter": filter_method, "repetition": 0}
            serial_prefix = prefix_for(directory/"timeout-rechecks", task)
            if not valid_output(serial_prefix, task): continue
            serial = json.loads(Path(str(serial_prefix)+"-summary.json").read_text())
            parallel_prefix = prefix_for(directory, task)
            parallel = json.loads(Path(str(parallel_prefix)+"-summary.json").read_text()) if valid_output(parallel_prefix, task) else {"platforms": {}}
            identity = {k: case[k] for k in ("phase", "scene", "pattern", "seed", "resolution_m", "n")}
            identity.update(case=case["name"], method=method, filter=filter_method, run_role="serial_timeout_recheck",
                            input_sha256=case["sha256"], fit_ms=serial["fit_ms"])
            for row in _legacy.platform_metrics(identity, observed, serial_prefix, serial, truth_data, dense):
                original = parallel["platforms"].get(row["platform"], {})
                row.update(parallel_status=original.get("status"), serial_status=row["status"],
                           parallel_plan_found=original.get("plan_found"), serial_plan_found=row["plan_found"],
                           parallel_reached=original.get("reached"), serial_reached=row["reached"],
                           timeout_disappeared=original.get("status") == 2 and row["status"] != 2,
                           repeated_timeout=original.get("status") == 2 and row["status"] == 2)
                rows.append(row)
    return rows


def plots(directory, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 3, figsize=(14, 4))
    x = np.arange(len(METHODS)); width = .25
    for offset, filter_method in enumerate(FILTERS):
        ordered = [next((r for r in rows if r["phase"] == "fit" and r["method"] == method and r["filter"] == filter_method), None) for method in METHODS]
        for axis, field, scale, label in zip(axes, ("accepted_coverage", "height_rmse_m", "harmful03_rate"),
                                           (100., 100., 100.), ("Missing cells accepted (%)", "Accepted-cell RMSE (cm)", "Error >3cm / all missing (%)")):
            axis.bar(x+(offset-1)*width, [r[field]*scale if r and r[field] is not None else np.nan for r in ordered], width, label=filter_method)
            axis.set_ylabel(label); axis.set_xticks(x, METHODS, rotation=30, ha="right")
    axes[-1].legend(); figure.tight_layout(); figure.savefig(directory/"filter-overview.png", dpi=160); plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("artifact", type=Path)
    args = parser.parse_args(); directory = args.artifact.resolve()
    manifest = json.loads((directory/"manifest.json").read_text()); suite = manifest["config"]["suite"]
    rows, bins, platforms, pair_common, all_common, pool_records, timing_rows = [], [], [], [], [], [], []
    for case in manifest["cases"]:
        n = case["n"]; shape = (n, n)
        observed = np.fromfile(case["input"], "<f4").reshape(shape)
        with np.load(case["truth"]) as data: truth_data = {k: data[k] for k in data.files}
        dense = {}
        if case["phase"] == "integrated":
            reference_case = f"integrated__{case['scene']}__dense__0__r{round(case['resolution_m']*1000)}"
            for platform in ("foot", "wheel"):
                path = directory/"outputs"/reference_case/("baseline__none-"+platform+".bin")
                if path.exists(): dense[platform] = np.fromfile(path, "<f4").reshape(shape+(2,))[:, :, 0]
        grounds = {}
        for task in tasks_for([case], suite):
            prefix = prefix_for(directory, task)
            if not valid_output(prefix, task): continue
            summary = json.loads(Path(str(prefix)+"-summary.json").read_text())
            identity = {k: case[k] for k in ("phase", "scene", "pattern", "seed", "resolution_m", "n")}
            identity.update(case=case["name"], method=task["method"], filter=task["filter"], repetition=task["repetition"],
                            fit_ms=summary["fit_ms"], allocated_bytes=summary["allocated_bytes"])
            if suite == "timing":
                timing_rows.append({**identity, "platform": "fit"})
                timing_rows.extend({**identity, "platform": platform, **metrics} for platform, metrics in summary["platforms"].items())
                continue
            ground = np.fromfile(str(prefix)+"-ground.bin", "<f4").reshape(shape+(6,))
            grounds[task["method"], task["filter"]] = ground
            metrics, score_bins = _legacy.fit_metrics(identity, observed, ground, truth_data)
            metrics.update(observed_metrics(observed, ground, truth_data))
            rows.append(metrics); bins.extend(score_bins)
            if case["phase"] == "integrated": platforms.extend(_legacy.platform_metrics(identity, observed, prefix, summary, truth_data, dense))
        if suite == "timing": continue
        missing = truth_data["interior"]&~np.isfinite(observed)
        for method in METHODS:
            reference = grounds.get((method, "none"))
            if reference is None: continue
            for filter_method in FILTERS[1:]:
                candidate = grounds.get((method, filter_method))
                if candidate is None or case["pattern"] == "dense": continue
                identity = {k: case[k] for k in ("phase", "scene", "pattern", "seed", "resolution_m", "n")}
                pair_common.append({**identity, "case": case["name"], "method": method, "filter": filter_method,
                                    "reference_filter": "none", **common_metrics(candidate, reference, truth_data["height"], missing)})
        if case["pattern"] != "dense":
            pool = missing.copy()
            complete_case = all(variant in grounds for variant in VARIANTS)
            if complete_case:
                for ground in grounds.values(): pool &= np.isfinite(ground[:, :, 0])
            else: pool[:] = False
            pool_records.append({"case": case["name"], "phase": case["phase"], "scene": case["scene"],
                                 "pattern": case["pattern"], "seed": case["seed"], "resolution_m": case["resolution_m"],
                                 "complete_fifteen_variants": complete_case, "missing_count": int(missing.sum()),
                                 "common_count": int(pool.sum()) if complete_case else None})
            if complete_case:
                for method, filter_method in VARIANTS:
                    identity = {k: case[k] for k in ("phase", "scene", "pattern", "seed", "resolution_m", "n")}
                    all_common.append({**identity, "case": case["name"], "method": method, "filter": filter_method,
                                       "reference_filter": "none", **common_metrics(grounds[method, filter_method], grounds[method, "none"], truth_data["height"], pool)})
    target = [r for r in rows if r["pattern"] != "dense"]
    aggregate = fit_aggregate(target, ("phase", "method", "filter"))
    scene = fit_aggregate(target, ("phase", "method", "filter", "scene", "pattern", "resolution_m"))
    timing = []
    if suite == "timing":
        groups = defaultdict(list)
        for row in timing_rows: groups[tuple(row[k] for k in ("phase", "scene", "pattern", "resolution_m", "method", "filter", "platform"))].append(row)
        for key, group in groups.items():
            record = dict(zip(("phase", "scene", "pattern", "resolution_m", "method", "filter", "platform"), key)); record["repeats"] = len(group)
            for field in ("fit_ms", "map_ms", "plan_ms"):
                if field == "fit_ms" and record["platform"] != "fit": continue
                data = [r[field] for r in group if field in r]
                if data:
                    for statistic, function in (("mean", np.mean), ("median", np.median), ("min", np.min), ("max", np.max)):
                        record[field+"_"+statistic] = float(function(data))
                    record[field+"_p95"] = float(np.quantile(data, .95))
            if record["platform"] != "fit": record.update(statuses=[r["status"] for r in group], reached=[r["reached"] for r in group])
            timing.append(record)
    rechecks = recheck_platform_results(directory, manifest)
    for name, data in (("fit-results", rows), ("confidence-bins", bins), ("platform-results", platforms),
                       ("scene-results", scene), ("filter-common-support-results", pair_common),
                       ("all-fifteen-common-results", all_common), ("all-fifteen-pools", pool_records),
                       ("timing-results", timing_rows), ("timing-aggregate", timing),
                       ("recheck-platform-results", rechecks)):
        (directory/(name+".json")).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        _legacy.write_csv(directory/(name+".csv"), data)
        if name == "recheck-platform-results" and not data:
            (directory/(name+".csv")).write_text("case,method,filter,platform,parallel_status,serial_status,parallel_plan_found,serial_plan_found,parallel_reached,serial_reached\n")
    summary = {"complete": manifest.get("complete", False), "runs_analyzed": len(rows) if suite != "timing" else sum(r["platform"] == "fit" for r in timing_rows),
               "aggregate": aggregate, "platform_aggregate_excluding_dense": platform_aggregate(platforms),
               "filter_common_support": common_aggregate(pair_common, ("phase", "method", "filter")),
               "all_fifteen_common": common_aggregate(all_common, ("phase", "method", "filter")),
               "filter_paired_seed_bootstrap": {field: filter_bootstrap(target, field) for field in ("accepted_coverage", "recovered03_rate", "harmful03_rate", "height_rmse_m")},
               "filter_common_support_seed_bootstrap": common_bootstrap(pair_common),
               "all_fifteen_common_seed_bootstrap": common_bootstrap(all_common),
               "timing": timing, "none_parity": manifest.get("none_parity"),
               "serial_timeout_recheck": {"case_count": len(manifest.get("serial_timeout_recheck", {}).get("cases", [])),
                                          "platform_records": len(rechecks),
                                          "parallel_timeout_platform_records": sum(r["parallel_status"] == 2 for r in rechecks),
                                          "serial_timeout_platform_records": sum(r["serial_status"] == 2 for r in rechecks),
                                          "timeout_disappeared_records": sum(r["timeout_disappeared"] for r in rechecks),
                                          "repeated_timeout_records": sum(r["repeated_timeout"] for r in rechecks),
                                          "scope": "separate serial confirmation; not included in primary 150-case aggregates"},
               "boundaries": {"performance": manifest["timing_boundary"],
                              "confidence": "raw evidence score, not safety probability; compare filter coverage and same-target errors together",
                              "observed": "estimate vs raw noisy measurements on each variant's same accepted observed cells; observed acceptance sets differ between variants",
                              "common_pool": "all fifteen pool can omit difficult cells; empty complete pools retain zero count and null errors",
                              "bootstrap": "paired per-case differences, then five seed-cluster means and 10000 resamples; excludes dense/occlusion/stripes",
                              "hazard": "raw physical evidence remains unchanged; analytic and dense-software diagnostics are not vehicle safety oracles",
                              "Humble_Orin_DDS_sensor_vehicle": "NOT_RUN"}}
    (directory/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    if aggregate: plots(directory, aggregate)
    print(f"analyzed {summary['runs_analyzed']} probe outputs, {len(platforms)} platform outputs")


if __name__ == "__main__": main()
