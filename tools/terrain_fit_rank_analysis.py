#!/usr/bin/env python3
"""Compare confidence ranking at equal selected percentages on a common pool.

Only post-processing: no fit, planning, parameter selection or vehicle commands.
Ties use fractional weights equal to expected random tie breaking, so grid order
cannot decide which errors enter a top-score subset.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import defaultdict
from pathlib import Path
import unittest

import numpy as np

METHODS = ("baseline", "weighted_plane", "mls", "grouped_ransac", "gaussian_process")
FRACTIONS = (.25, .50, .75, 1.)


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def top_fraction_weights(scores, fraction):
    """Expected inclusion weights for the top fraction under random tied order.

    The target count may be fractional when fraction*n is not integral. Every
    score above the cutoff has weight 1 and every tied cutoff score receives the
    same remaining_count/tie_count weight. The weighted selected count is exactly
    fraction*n, including tiny candidate pools.
    """
    scores = np.asarray(scores, dtype=float)
    if scores.ndim != 1 or not np.isfinite(scores).all(): raise ValueError("scores must be a finite vector")
    if not 0 < fraction <= 1: raise ValueError("fraction must be in (0,1]")
    n = len(scores); weights = np.zeros(n, dtype=float)
    if not n: return weights, {"cutoff": None, "fully_selected_count": 0, "boundary_tie_count": 0, "boundary_weight": None}
    if fraction == 1:
        weights[:] = 1.
        return weights, {"cutoff": float(scores.min()), "fully_selected_count": n,
                         "boundary_tie_count": 0, "boundary_weight": None}
    target = fraction*n
    cutoff = float(np.partition(scores, n-int(np.ceil(target)))[n-int(np.ceil(target))])
    above = scores > cutoff; tied = scores == cutoff
    remaining = target-int(above.sum()); tie_weight = remaining/int(tied.sum())
    weights[above] = 1.; weights[tied] = tie_weight
    return weights, {"cutoff": cutoff, "fully_selected_count": int(above.sum()),
                     "boundary_tie_count": int(tied.sum()), "boundary_weight": float(tie_weight)}


def ranked_metrics(scores, errors, fraction):
    errors = np.asarray(errors, dtype=float)
    if errors.ndim != 1 or not np.isfinite(errors).all() or len(errors) != len(scores):
        raise ValueError("errors must be finite and match scores")
    weights, ties = top_fraction_weights(scores, fraction)
    selected = float(weights.sum())
    sq_sum = float(np.dot(weights, errors**2)); abs_sum = float(np.dot(weights, np.abs(errors)))
    bad03 = float(np.dot(weights, np.abs(errors) > .03)); bad10 = float(np.dot(weights, np.abs(errors) > .10))
    return {"candidate_count": len(scores), "selected_effective_count": selected,
            "height_sq_sum": sq_sum, "height_abs_sum": abs_sum,
            "bad03_effective_count": bad03, "bad10_effective_count": bad10,
            "height_rmse_m": float(np.sqrt(sq_sum/selected)) if selected else None,
            "height_mae_m": abs_sum/selected if selected else None,
            "bad03_rate": bad03/selected if selected else None,
            "bad10_rate": bad10/selected if selected else None, **ties}


def aggregate(rows, keys):
    groups = defaultdict(list)
    for row in rows: groups[tuple(row[key] for key in keys)].append(row)
    results = []
    for group_key, group in groups.items():
        n = sum(row["candidate_count"] for row in group)
        selected = sum(row["selected_effective_count"] for row in group)
        sq = sum(row["height_sq_sum"] for row in group); absolute = sum(row["height_abs_sum"] for row in group)
        bad03 = sum(row["bad03_effective_count"] for row in group); bad10 = sum(row["bad10_effective_count"] for row in group)
        results.append({**dict(zip(keys, group_key)), "cases": len(group),
                        "unique_input_sha256_count": len({row["input_sha256"] for row in group}),
                        "empty_pool_cases": sum(row["candidate_count"] == 0 for row in group),
                        "candidate_count": n, "selected_effective_count": selected,
                        "height_rmse_m": float(np.sqrt(sq/selected)) if selected else None,
                        "height_mae_m": absolute/selected if selected else None,
                        "bad03_rate": bad03/selected if selected else None,
                        "bad10_rate": bad10/selected if selected else None,
                        "bad03_effective_count": bad03, "bad10_effective_count": bad10})
    return results


def write_table(path, rows):
    if not rows: return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)


def plots(directory, overall, scene_pattern):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for pool in sorted({row["pool"] for row in overall}):
        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        for method in METHODS:
            selected = sorted((r for r in overall if r["pool"] == pool and r["method"] == method),
                              key=lambda r: r["selected_fraction"])
            for axis, field, scale, label in zip(axes, ("height_rmse_m", "bad03_rate", "bad10_rate"),
                                               (100., 100., 100.), ("Height RMSE (cm)", "Error >3 cm (%)", "Error >10 cm (%)")):
                axis.plot([r["selected_fraction"]*100 for r in selected],
                          [r[field]*scale if r[field] is not None else np.nan for r in selected],
                          marker="o", label=method)
                axis.set_xlabel("Top confidence fraction per case (%)"); axis.set_ylabel(label)
                axis.set_xticks((25, 50, 75, 100))
        axes[-1].legend(fontsize=8)
        fig.suptitle("Same five-method candidate pool" if pool == "common" else "Own accepted pools differ between methods")
        fig.tight_layout(); fig.savefig(directory/("rank-"+pool+"-overview.png"), dpi=160); plt.close(fig)
    # Hide pooled averaging by also showing representative pattern slices, with
    # resolutions pooled but the analytic scenario/pattern kept explicit.
    slices = (("flat", "noise015"), ("convex", "missing50"), ("rocks", "outlier05"), ("ditch15", "occlusion"))
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for axis, (scene, pattern) in zip(axes.flat, slices):
        for method in METHODS:
            selected = sorted((r for r in scene_pattern if r["pool"] == "common" and r["method"] == method
                               and r["scene"] == scene and r["pattern"] == pattern), key=lambda r: r["selected_fraction"])
            axis.plot([r["selected_fraction"]*100 for r in selected],
                      [r["height_rmse_m"]*100 if r["height_rmse_m"] is not None else np.nan for r in selected],
                      marker="o", label=method)
        axis.set_title(scene+" / "+pattern); axis.set_xlabel("Top score fraction (%)"); axis.set_ylabel("RMSE (cm)")
        axis.set_xticks((25, 50, 75, 100))
    axes.flat[-1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(directory/"rank-common-scene-pattern.png", dpi=160); plt.close(fig)


class RankTests(unittest.TestCase):
    def test_ties_use_equal_fractional_weight_and_ignore_input_order(self):
        scores = np.array([.9, .5, .5, .5, .1]); errors = np.array([0., .01, .10, .30, .5])
        weights, meta = top_fraction_weights(scores, .5)
        np.testing.assert_allclose(weights, (1., .5, .5, .5, 0.))
        self.assertEqual(meta["boundary_tie_count"], 3)
        result = ranked_metrics(scores, errors, .5)
        permuted = ranked_metrics(scores[::-1], errors[::-1], .5)
        self.assertAlmostEqual(result["height_rmse_m"], permuted["height_rmse_m"])
        self.assertEqual(result["selected_effective_count"], 2.5)

    def test_perfect_and_reversed_ranking(self):
        errors = np.array([0., .01, .10, .30])
        perfect = [ranked_metrics(-errors, errors, f)["height_rmse_m"] for f in FRACTIONS]
        reversed_ = [ranked_metrics(errors, errors, f)["height_rmse_m"] for f in FRACTIONS]
        self.assertEqual(perfect[0], 0.)
        self.assertEqual(reversed_[0], .30)
        self.assertTrue(all(a <= b for a, b in zip(perfect[:-1], perfect[1:])))
        self.assertTrue(all(a >= b for a, b in zip(reversed_[:-1], reversed_[1:])))
        self.assertAlmostEqual(perfect[-1], reversed_[-1])

    def test_constant_score_equals_full_pool_error_at_all_fractions(self):
        errors = np.array([0., .04, .20])
        rmse = float(np.sqrt(np.mean(errors**2)))
        for fraction in FRACTIONS:
            result = ranked_metrics(np.ones(3), errors, fraction)
            self.assertAlmostEqual(result["height_rmse_m"], rmse)
            self.assertAlmostEqual(result["bad03_rate"], 2/3)

    def test_empty_pool_preserves_null_error(self):
        result = ranked_metrics(np.array([]), np.array([]), .25)
        self.assertEqual(result["candidate_count"], 0)
        self.assertEqual(result["selected_effective_count"], 0.)
        self.assertIsNone(result["height_rmse_m"])
        self.assertIsNone(result["bad03_rate"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--main", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--include-own-pool", action="store_true", help="Also report own accepted candidates; pools differ")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(RankTests)
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        if not result.wasSuccessful(): raise SystemExit(1)
        if args.main is None: return
    if args.main is None or args.output is None: parser.error("--main and --output are required for post-processing")
    main_dir, output_dir = args.main.resolve(), args.output.resolve()
    manifest_path = main_dir/"manifest.json"; main_hash = file_hash(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    if not manifest.get("complete") or manifest.get("freeze_mismatch"):
        raise SystemExit("requires a complete frozen main experiment")
    if set(METHODS)-set(manifest["config"]["methods"]): raise SystemExit("main experiment must include all five methods")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_manifest = output_dir/"manifest.json"
    if output_manifest.exists(): raise SystemExit("rank artifact already exists; choose a new output directory")
    code_hash = file_hash(__file__)
    meta = {"schema": 1, "complete": False, "main_manifest_sha256": main_hash,
            "postprocess_code_sha256": code_hash, "main_artifact": str(main_dir),
            "fractions": FRACTIONS, "include_own_pool": args.include_own_pool}
    output_manifest.write_text(json.dumps(meta, indent=2)+"\n")
    rows, pool_records = [], []
    matrix = [case for case in manifest["cases"] if case["phase"] == "fit" and case["pattern"] != "dense"]
    for i, case in enumerate(matrix):
        n = case["n"]; shape = (n, n)
        observed = np.fromfile(case["input"], "<f4").reshape(shape)
        if file_hash(case["input"]) != case["sha256"]: raise SystemExit("frozen input changed: "+case["name"])
        with np.load(main_dir/"inputs"/(case["name"]+"-truth.npz")) as data:
            truth_height, interior = data["height"], data["interior"]
        grounds = {method: np.fromfile(main_dir/"outputs"/case["name"]/(method+"-ground.bin"), "<f4").reshape(shape+(6,))
                   for method in METHODS}
        missing = ~np.isfinite(observed)&interior
        common = missing.copy()
        accepted = {}
        for method, ground in grounds.items():
            mask = missing & np.isfinite(ground[:, :, 0])
            if np.any(mask & ~np.isfinite(ground[:, :, 1])): raise SystemExit("accepted estimate has invalid confidence: "+case["name"]+"/"+method)
            accepted[method] = mask; common &= mask
        identity = {key: case[key] for key in ("scene", "pattern", "seed", "resolution_m", "n")}
        identity.update(case=case["name"], input_sha256=case["sha256"])
        pool_records.append({**identity, "missing_count": int(missing.sum()), "common_pool_count": int(common.sum()),
                             "empty_common_pool": not bool(common.any()),
                             "own_pool_counts": {method: int(mask.sum()) for method, mask in accepted.items()}})
        for method, ground in grounds.items():
            pools = [("common", common)]
            if args.include_own_pool: pools.append(("own", accepted[method]))
            for pool_name, mask in pools:
                scores = ground[:, :, 1][mask]
                errors = ground[:, :, 0][mask]-truth_height[mask]
                for fraction in FRACTIONS:
                    rows.append({**identity, "pool": pool_name, "method": method, "selected_fraction": fraction,
                                 "missing_count": int(missing.sum()), **ranked_metrics(scores, errors, fraction)})
        if (i+1) % 100 == 0: print(f"rank postprocess [{i+1}/{len(matrix)}]", flush=True)
    overall = aggregate(rows, ("pool", "method", "selected_fraction"))
    stratified = aggregate(rows, ("pool", "method", "selected_fraction", "scene", "pattern", "resolution_m"))
    scene_pattern = aggregate(rows, ("pool", "method", "selected_fraction", "scene", "pattern"))
    for name, data in (("rank-case-results", rows), ("rank-overall", overall), ("rank-stratified", stratified),
                       ("rank-scene-pattern", scene_pattern), ("candidate-pools", pool_records)):
        (output_dir/(name+".json")).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        write_table(output_dir/(name+".csv"), data)
    plots(output_dir, overall, scene_pattern)
    mismatch = []
    if file_hash(manifest_path) != main_hash: mismatch.append("main manifest changed during post-processing")
    if file_hash(__file__) != code_hash: mismatch.append("postprocess code changed during execution")
    meta.update(complete=not mismatch, freeze_mismatch=mismatch, expected_cases=len(matrix), completed_cases=len(pool_records),
                empty_common_pool_cases=sum(record["empty_common_pool"] for record in pool_records),
                unique_input_sha256_count=len({record["input_sha256"] for record in pool_records}),
                boundaries={"common_pool": "identical missing candidates accepted by all five methods; other methods such as RANSAC may reject difficult cells, so this pool is easier and incomplete",
                            "ranking": "each method selects a different top-score subset from the same candidate pool; equal percentage removes score-scale advantage",
                            "ties": "random tie-order expectation via fractional inclusion; RMSE is square root of expected weighted MSE",
                            "empty": "empty common pools retained; errors/rates are null, not zero",
                            "own_pool": "optional own-accepted curves use different candidate sets and are not a controlled score-ranking comparison",
                            "duplicates": "case counts include deterministic repeated inputs; unique SHA counts separately; no independent-sample CI claimed",
                            "validation": "synthetic local post-processing only; real sensor/Orin/vehicle NOT_RUN"})
    output_manifest.write_text(json.dumps(meta, ensure_ascii=False, indent=2)+"\n")
    (output_dir/"summary.json").write_text(json.dumps({"complete": meta["complete"], "overall": overall,
                                                      "boundaries": meta["boundaries"]}, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"complete": meta["complete"], "cases": len(pool_records), "empty_common_pool_cases": meta["empty_common_pool_cases"]}))
    if mismatch: raise SystemExit("postprocess fingerprint changed; artifacts retained incomplete")


if __name__ == "__main__": main()
