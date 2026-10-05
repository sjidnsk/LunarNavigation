#!/usr/bin/env python3
"""Compute error/coverage, paired seed uncertainty, and independent hazard diagnostics."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

from protocol import METHODS


def mean_or_none(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(values.mean()) if len(values) else None


def rank(values):
    order = np.argsort(values, kind="stable")
    sorted_values = values[order]
    boundaries = np.r_[0, np.flatnonzero(np.diff(sorted_values))+1, len(values)]
    result = np.empty(len(values), dtype=float)
    for start, stop in zip(boundaries[:-1], boundaries[1:]): result[order[start:stop]] = (start+stop-1)/2
    return result


def spearman(a, b):
    if len(a) < 4: return None
    ra, rb = rank(a), rank(b)
    if np.std(ra) < 1e-12 or np.std(rb) < 1e-12: return None
    return float(np.corrcoef(ra, rb)[0, 1])


def paired_bootstrap(rows, field, iterations=10000):
    """Resample seed clusters rather than pretending adjacent cells are independent."""
    by_case = defaultdict(dict)
    for row in rows:
        # Occlusion is deterministic and stripes repeat two phases across the
        # frozen five seeds; neither supplies five independent random clusters.
        if row["pattern"] not in ("dense", "occlusion", "stripes"):
            by_case[row["case"]][row["method"]] = row
    result = {}
    for method in METHODS[1:]:
        seed_differences = defaultdict(list)
        count = 0
        for pair in by_case.values():
            if "baseline" not in pair or method not in pair: continue
            a, b = pair[method].get(field), pair["baseline"].get(field)
            if a is None or b is None: continue
            seed_differences[pair[method]["seed"]].append(float(a)-float(b)); count += 1
        values = np.array([np.mean(v) for v in seed_differences.values()])
        if len(values) < 2:
            result[method] = {"paired_cases": count, "seed_clusters": len(values), "delta": mean_or_none(values),
                              "ci95": None}
            continue
        rng = np.random.default_rng(1601)
        draws = rng.choice(values, (iterations, len(values)), replace=True).mean(axis=1)
        result[method] = {"paired_cases": count, "seed_clusters": len(values), "delta": float(values.mean()),
                          "ci95": [float(x) for x in np.quantile(draws, (.025, .975))]}
    return result


def fit_metrics(record, observed, ground, truth_data):
    interior = truth_data["interior"]
    missing = ~np.isfinite(observed) & interior
    inferred = missing & np.isfinite(ground[:, :, 0])
    smooth = truth_data["smooth"] & inferred
    errors = np.abs(ground[:, :, 0]-truth_data["height"])
    values = errors[inferred]; confidence = ground[:, :, 1][inferred]
    n_missing, n_inferred = int(missing.sum()), int(inferred.sum())
    high = inferred & (ground[:, :, 1] >= .6)
    variance = ground[:, :, 4]
    sigma_ok = inferred & np.isfinite(variance) & (variance > 0)
    gradient_errors = np.linalg.norm(ground[:, :, 2:4]-truth_data["gradient"], axis=-1)[smooth]
    row = dict(record)
    row.update(observed_count=int((np.isfinite(observed)&interior).sum()), missing_count=n_missing,
               inferred_count=n_inferred, accepted_coverage=n_inferred/n_missing if n_missing else None,
               accurate03_count=int((inferred&(errors <= .03)).sum()),
               accurate01_count=int((inferred&(errors <= .01)).sum()),
               bad03_count=int((inferred&(errors > .03)).sum()),
               bad10_count=int((inferred&(errors > .10)).sum()),
               recovered03_rate=int((inferred&(errors <= .03)).sum())/n_missing if n_missing else None,
               harmful03_rate=int((inferred&(errors > .03)).sum())/n_missing if n_missing else None,
               height_abs_sum=float(values.sum()), height_sq_sum=float(np.dot(values, values)),
               height_mae_m=float(values.mean()) if n_inferred else None,
               height_rmse_m=float(np.sqrt(np.mean(values**2))) if n_inferred else None,
               height_p95_m=float(np.quantile(values, .95)) if n_inferred else None,
               height_max_m=float(values.max()) if n_inferred else None,
               gradient_count=int(len(gradient_errors)),
               gradient_sq_sum=float(np.dot(gradient_errors, gradient_errors)),
               gradient_rmse=float(np.sqrt(np.mean(gradient_errors**2))) if len(gradient_errors) else None,
               confidence_mean=float(confidence.mean()) if n_inferred else None,
               confidence_error_spearman=spearman(confidence, values),
               high_confidence_count=int(high.sum()), high_confidence_bad03_count=int((high&(errors > .03)).sum()),
               high_confidence_bad10_count=int((high&(errors > .10)).sum()),
               fallback_count=int((inferred&(ground[:, :, 5] > .5)).sum()),
               variance_positive_count=int(sigma_ok.sum()),
               conditional_95_interval_covered=int((sigma_ok&(errors <= 1.96*np.sqrt(np.maximum(0., variance)))).sum()))
    bins = []
    for lower in np.arange(0, 1., .1):
        mask = inferred & (ground[:, :, 1] >= lower-1e-8) & (ground[:, :, 1] < lower+.1-1e-8)
        if not mask.any(): continue
        e = errors[mask]
        bins.append({**record, "bin_low": round(float(lower), 1), "count": int(mask.sum()),
                     "abs_sum": float(e.sum()), "sq_sum": float(np.dot(e, e)),
                     "bad03_count": int((e > .03).sum()), "bad10_count": int((e > .10).sum()),
                     "max_error_m": float(e.max())})
    return row, bins


def path_diagnostics(path_path, state, dense, hazard, resolution):
    if not path_path.exists(): return {}
    with path_path.open() as stream:
        points = np.array([[float(row["x"]), float(row["y"])] for row in csv.DictReader(stream)])
    samples = []
    if len(points) == 1: samples = points
    elif len(points) > 1:
        for a, b in zip(points[:-1], points[1:]):
            spacing = min(.01, resolution/5)
            samples.extend(a+(b-a)*t for t in np.linspace(0., 1., max(2, int(np.ceil(np.linalg.norm(b-a)/spacing))+1)))
    if not len(samples): return {"path_length_m": 0., "path_samples": 0,
                                 "path_analytic_hazard_samples": 0, "path_dense_blocked_samples": 0}
    indices = np.floor(np.array(samples)/resolution).astype(int)
    inside = (indices[:, 0] >= 0)&(indices[:, 0] < state.shape[1])&(indices[:, 1] >= 0)&(indices[:, 1] < state.shape[0])
    ii = indices[inside]
    return {"path_length_m": float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points)>1 else 0.,
            "path_samples": int(len(indices)), "path_outside_samples": int((~inside).sum()),
            "path_analytic_hazard_samples": int(hazard[ii[:, 1], ii[:, 0]].sum()),
            "path_dense_blocked_samples": int((dense[ii[:, 1], ii[:, 0]] == 100).sum()) if dense is not None else None}


def platform_metrics(record, observed, prefix, summary, truth_data, dense):
    rows = []
    for platform in ("foot", "wheel"):
        binary = Path(str(prefix)+f"-{platform}.bin")
        if not binary.exists(): continue
        state = np.fromfile(binary, "<f4").reshape(observed.shape+(2,))[:, :, 0]
        mask = truth_data["interior"]; hazard = truth_data[f"hazard_{platform}"]
        reference = dense.get(platform)
        free, blocked, unknown = state == 0, state == 100, state == -1
        row = {**record, "platform": platform, **summary["platforms"][platform],
               "free_count": int((free&mask).sum()), "blocked_count": int((blocked&mask).sum()),
               "unknown_count": int((unknown&mask).sum()),
               "analytic_hazard_count": int((hazard&mask).sum()),
               "analytic_hazard_free_count": int((free&hazard&mask).sum()),
               "dense_blocked_count": int(((reference == 100)&mask).sum()) if reference is not None else None,
               "dense_blocked_free_count": int((free&(reference == 100)&mask).sum()) if reference is not None else None,
               "dense_unknown_free_count": int((free&(reference == -1)&mask).sum()) if reference is not None else None}
        row.update(path_diagnostics(Path(str(prefix)+f"-{platform}-path.csv"), state, reference, hazard, record["resolution_m"]))
        rows.append(row)
    return rows


def grouped_fit(rows, keys):
    groups = defaultdict(list)
    for row in rows: groups[tuple(row[k] for k in keys)].append(row)
    results = []
    for key, group in groups.items():
        n = sum(r["inferred_count"] for r in group); missing = sum(r["missing_count"] for r in group)
        grad_n = sum(r["gradient_count"] for r in group); high_n = sum(r["high_confidence_count"] for r in group)
        var_n = sum(r["variance_positive_count"] for r in group)
        results.append({**dict(zip(keys, key)), "cases": len(group), "missing_count": missing, "inferred_count": n,
                        "accepted_coverage": n/missing if missing else None,
                        "height_mae_m": sum(r["height_abs_sum"] for r in group)/n if n else None,
                        "height_rmse_m": np.sqrt(sum(r["height_sq_sum"] for r in group)/n) if n else None,
                        "recovered03_rate": sum(r["accurate03_count"] for r in group)/missing if missing else None,
                        "harmful03_rate": sum(r["bad03_count"] for r in group)/missing if missing else None,
                        "bad03_given_accepted": sum(r["bad03_count"] for r in group)/n if n else None,
                        "height_max_m": max((r["height_max_m"] for r in group if r["height_max_m"] is not None), default=None),
                        "gradient_rmse": np.sqrt(sum(r["gradient_sq_sum"] for r in group)/grad_n) if grad_n else None,
                        "fit_ms_mean": np.mean([r["fit_ms"] for r in group]),
                        "fit_ms_p95": np.quantile([r["fit_ms"] for r in group], .95),
                        "mean_case_spearman": mean_or_none([r["confidence_error_spearman"] if r["confidence_error_spearman"] is not None else np.nan for r in group]),
                        "spearman_valid_cases": sum(r["confidence_error_spearman"] is not None for r in group),
                        "high_confidence_count": high_n,
                        "high_confidence_bad03_rate": sum(r["high_confidence_bad03_count"] for r in group)/high_n if high_n else None,
                        "high_confidence_bad10_rate": sum(r["high_confidence_bad10_count"] for r in group)/high_n if high_n else None,
                        "fallback_rate": sum(r["fallback_count"] for r in group)/n if n else None,
                        "conditional_95_interval_coverage": sum(r["conditional_95_interval_covered"] for r in group)/var_n if var_n else None})
    return results


def grouped_platform(rows):
    groups = defaultdict(list)
    for row in rows:
        if row["pattern"] != "dense": groups[row["method"], row["platform"]].append(row)
    result = []
    for (method, platform), group in groups.items():
        hazards = sum(r["analytic_hazard_count"] for r in group)
        dense_blocked = sum(r["dense_blocked_count"] or 0 for r in group)
        free = sum(r["free_count"] for r in group)
        hazard_free = sum(r["analytic_hazard_free_count"] for r in group)
        dense_false_free = sum(r["dense_blocked_free_count"] or 0 for r in group)
        result.append({"method": method, "platform": platform, "cases": len(group),
                       "plan_found_count": sum(bool(r.get("plan_found", r["status"] == 0 and r["path_points"] > 0)) for r in group),
                       "reached_count": sum(r["reached"] for r in group),
                       "timeout_count": sum(r["status"] == 2 for r in group),
                       "map_ms_mean": float(np.mean([r["map_ms"] for r in group])),
                       "map_ms_p95": float(np.quantile([r["map_ms"] for r in group], .95)),
                       "plan_ms_mean": float(np.mean([r["plan_ms"] for r in group])),
                       "free_count": free,
                       "unknown_count": sum(r["unknown_count"] for r in group),
                       "analytic_hazard_count": hazards,
                       "analytic_hazard_free_count": hazard_free,
                       "analytic_hazard_free_rate": hazard_free/hazards if hazards else None,
                       "analytic_hazard_free_fraction_of_free": hazard_free/free if free else None,
                       "dense_blocked_count": dense_blocked,
                       "dense_blocked_free_count": dense_false_free,
                       "dense_blocked_free_rate": dense_false_free/dense_blocked if dense_blocked else None,
                       "dense_blocked_free_fraction_of_free": dense_false_free/free if free else None,
                       "path_analytic_hazard_case_count": sum(r["path_analytic_hazard_samples"] > 0 for r in group),
                       "path_dense_blocked_case_count": sum((r["path_dense_blocked_samples"] or 0) > 0 for r in group)})
    return result


def common_support_bootstrap(rows):
    # Each method-baseline pair has its own common set; include the method in the
    # pairing identifier so differently accepted baseline subsets never mix.
    paired = []
    for row in rows:
        if row["phase"] != "fit": continue
        for method, value in ((row["method"], row["rmse_delta_m"]), ("baseline", 0.)):
            paired.append({"case": row["case"]+"__"+row["method"], "pattern": row["pattern"],
                           "seed": row["seed"], "method": method, "common_rmse_delta": value})
    return paired_bootstrap(paired, "common_rmse_delta")


def write_csv(path, rows):
    if not rows: return
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)


def plot(directory, aggregate, scene_rows, bins):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError: return
    rows = [r for r in aggregate if r.get("phase") == "fit"]
    if not rows: rows = aggregate
    names = [r["method"] for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    for axis, field, scale, label in zip(ax, ("height_rmse_m", "accepted_coverage", "fit_ms_mean"),
                                       (100, 100, 1), ("Accepted-cell RMSE (cm)", "Missing cells accepted (%)", "Fit time per map (ms)")):
        axis.bar(np.arange(len(rows)), [r[field]*scale if r[field] is not None else np.nan for r in rows])
        axis.set_xticks(np.arange(len(rows)), names, rotation=35, ha="right"); axis.set_ylabel(label)
    fig.tight_layout(); fig.savefig(directory/"comparison-overview.png", dpi=160); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 5))
    for method in METHODS:
        selected = [r for r in rows if r["method"] == method]
        if selected:
            r = selected[0]; ax.scatter(r["accepted_coverage"]*100, r["height_rmse_m"]*100, label=method, s=75)
    ax.set_xlabel("Missing cells accepted (%)"); ax.set_ylabel("Accepted-cell RMSE (cm)"); ax.legend()
    fig.tight_layout(); fig.savefig(directory/"coverage-vs-error.png", dpi=160); plt.close(fig)
    grouped = defaultdict(lambda: [0., 0.])
    for row in bins:
        if row["phase"] != "fit": continue
        values = grouped[row["method"], row["bin_low"]]; values[0] += row["abs_sum"]; values[1] += row["count"]
    fig, ax = plt.subplots(figsize=(8, 5))
    for method in METHODS:
        points = sorted((low+.05, v[0]/v[1]*100) for (m, low), v in grouped.items() if m == method and v[1])
        if points: ax.plot(*zip(*points), marker="o", label=method)
    ax.set_xlabel("Confidence score bin midpoint"); ax.set_ylabel("Pooled mean absolute error (cm)")
    ax.legend(); ax.set_title("Pooled curve; inspect scene tables before judging ranking")
    fig.tight_layout(); fig.savefig(directory/"confidence-vs-error.png", dpi=160); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("artifact", type=Path)
    args = parser.parse_args(); root = args.artifact.resolve()
    manifest = json.loads((root/"manifest.json").read_text()); rows, bins, platform_rows, common, all_common = [], [], [], [], []
    for case in manifest["cases"]:
        n = case["n"]; shape = (n, n)
        observed = np.fromfile(case["input"], "<f4").reshape(shape)
        with np.load(root/"inputs"/(case["name"]+"-truth.npz")) as loaded:
            truth_data = {k: loaded[k] for k in loaded.files}
        dense = {}
        if case["phase"] == "integrated":
            dense_case = f"integrated__{case['scene']}__dense__0__r{round(case['resolution_m']*1000)}"
            for platform in ("foot", "wheel"):
                path = root/"outputs"/dense_case/f"baseline-{platform}.bin"
                if path.exists(): dense[platform] = np.fromfile(path, "<f4").reshape(shape+(2,))[:, :, 0]
        ground_by_method = {}
        for method in manifest["config"]["methods"]:
            prefix = root/"outputs"/case["name"]/method
            summary_path = Path(str(prefix)+"-summary.json")
            if not summary_path.exists(): continue
            summary = json.loads(summary_path.read_text())
            ground = np.fromfile(str(prefix)+"-ground.bin", "<f4").reshape(shape+(6,))
            ground_by_method[method] = ground
            record = {k: case[k] for k in ("phase", "scene", "pattern", "seed", "resolution_m", "n")}
            record.update(case=case["name"], method=method, fit_ms=summary["fit_ms"],
                          allocated_bytes=summary["allocated_bytes"])
            row, case_bins = fit_metrics(record, observed, ground, truth_data)
            rows.append(row); bins.extend(case_bins)
            if case["phase"] == "integrated": platform_rows.extend(platform_metrics(record, observed, prefix, summary, truth_data, dense))
        if "baseline" in ground_by_method:
            baseline = ground_by_method["baseline"][:, :, 0]
            for method, ground in ground_by_method.items():
                if method == "baseline": continue
                mask = truth_data["interior"] & ~np.isfinite(observed) & np.isfinite(baseline) & np.isfinite(ground[:, :, 0])
                if not mask.any(): continue
                a = ground[:, :, 0][mask]-truth_data["height"][mask]; b = baseline[mask]-truth_data["height"][mask]
                common.append({"case": case["name"], "phase": case["phase"], "scene": case["scene"],
                               "pattern": case["pattern"], "seed": case["seed"], "method": method,
                               "count": int(mask.sum()), "method_sq_sum": float(np.dot(a, a)),
                               "baseline_sq_sum": float(np.dot(b, b)),
                               "rmse_delta_m": float(np.sqrt(np.mean(a*a))-np.sqrt(np.mean(b*b)))})
        if all(method in ground_by_method for method in METHODS):
            mask = truth_data["interior"] & ~np.isfinite(observed)
            for ground in ground_by_method.values(): mask &= np.isfinite(ground[:, :, 0])
            for method in METHODS:
                errors = ground_by_method[method][:, :, 0][mask]-truth_data["height"][mask]
                all_common.append({"case": case["name"], "phase": case["phase"], "scene": case["scene"],
                                   "pattern": case["pattern"], "seed": case["seed"], "method": method,
                                   "count": int(mask.sum()), "sq_sum": float(np.dot(errors, errors)),
                                   "abs_sum": float(np.abs(errors).sum()),
                                   "rmse_m": float(np.sqrt(np.mean(errors**2))) if len(errors) else None})
    # Dense cases carry no missing target and should not distort fit-map runtime means.
    target_rows = [r for r in rows if r["pattern"] != "dense"]
    aggregate = grouped_fit(target_rows, ("phase", "method"))
    scene_rows = grouped_fit(target_rows, ("phase", "method", "scene", "pattern", "resolution_m"))
    common_aggregate = []
    for method in METHODS[1:]:
        selected = [r for r in common if r["phase"] == "fit" and r["method"] == method]
        count = sum(r["count"] for r in selected)
        if count: common_aggregate.append({"method": method, "common_support_count": count,
                                         "method_rmse_m": float(np.sqrt(sum(r["method_sq_sum"] for r in selected)/count)),
                                         "baseline_rmse_m": float(np.sqrt(sum(r["baseline_sq_sum"] for r in selected)/count))})
    summary = {"artifact_complete": manifest.get("complete", False), "runs_analyzed": len(rows),
               "unique_input_sha256_count": len({c["sha256"] for c in manifest["cases"]}),
               "aggregate": aggregate, "common_support": common_aggregate,
               "platform_aggregate_excluding_dense": grouped_platform(platform_rows),
               "all_five_common_support": [{"method": method, "count": sum(r["count"] for r in all_common if r["phase"] == "fit" and r["method"] == method),
                                             "rmse_m": float(np.sqrt(sum(r["sq_sum"] for r in all_common if r["phase"] == "fit" and r["method"] == method)/sum(r["count"] for r in all_common if r["phase"] == "fit" and r["method"] == method)))
                                             if sum(r["count"] for r in all_common if r["phase"] == "fit" and r["method"] == method) else None}
                                            for method in METHODS],
               "paired_common_support_seed_bootstrap": common_support_bootstrap(common),
               "all_five_common_support_seed_bootstrap": paired_bootstrap([r for r in all_common if r["phase"] == "fit"], "rmse_m"),
               "paired_seed_bootstrap": {field: paired_bootstrap([r for r in target_rows if r["phase"] == "fit"], field)
                                         for field in ("height_rmse_m", "accepted_coverage", "recovered03_rate", "harmful03_rate", "fit_ms")},
               "boundaries": {"independent_truth": "analytic synthetic height and derivative; not sensor/vehicle proof",
                              "dense_reference": "baseline production software on unperturbed truth; not safety oracle",
                              "hazard": "conservative circle-footprint analytic slope/jump diagnostic; ignores dynamics/contact planning",
                              "confidence": "evidence score; high score defined >=0.6; not safety probability",
                              "bootstrap": "paired differences for stochastic patterns averaged by fixed seed then seed-cluster resampling; excludes dense, deterministic occlusion and repeating stripes",
                              "Humble_Orin_DDS_rosbag_vehicle": "NOT_RUN"}}
    for name, data in (("fit-results", rows), ("confidence-bins", bins), ("platform-results", platform_rows),
                       ("scene-results", scene_rows), ("common-support-results", common)):
        (root/(name+".json")).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        write_csv(root/(name+".csv"), data)
    (root/"all-five-common-support-results.json").write_text(json.dumps(all_common, indent=2, allow_nan=False)+"\n")
    write_csv(root/"all-five-common-support-results.csv", all_common)
    (root/"summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    plot(root, aggregate, scene_rows, bins)
    print(f"Analyzed {len(rows)} fitter outputs and {len(platform_rows)} platform outputs")


if __name__ == "__main__": main()
