import tempfile
import json
from pathlib import Path
import unittest

import numpy as np

from protocol import FILTER_PARAMETERS, VARIANTS, legacy_cases, prefix_for, tasks_for
from analyze import observed_metrics, common_metrics, common_bootstrap, filter_bootstrap, recheck_platform_results
from runner import none_parity, valid_output, validate_timing_reference


class FilterProtocolTests(unittest.TestCase):
    def matrix(self, suite="full"):
        return [{"name": c.name, "phase": c.phase, "scene": c.scene, "pattern": c.pattern,
                 "seed": c.seed, "resolution_m": c.resolution_m, "n": c.n} for c in legacy_cases(suite, "all")]

    def test_full_count_same_dense_reference_and_unique_outputs(self):
        matrix = self.matrix(); tasks = tasks_for(matrix)
        self.assertEqual(sum(t["case"]["phase"] == "fit" for t in tasks), 18480)
        self.assertEqual(sum(t["case"]["phase"] == "integrated" for t in tasks), 2272)
        self.assertEqual(len(tasks), 20752)
        names = {str(prefix_for(Path("/tmp/filter-test"), task)) for task in tasks}
        self.assertEqual(len(names), len(tasks))
        for task in tasks:
            if task["case"]["phase"] == "integrated" and task["case"]["pattern"] == "dense":
                self.assertEqual((task["method"], task["filter"]), ("baseline", "none"))

    def test_rotation_and_fixed_filters(self):
        tasks = tasks_for(self.matrix("quick"))
        first = tasks[:15]; second = tasks[15:30]
        self.assertEqual({(t["method"], t["filter"]) for t in first}, set(VARIANTS))
        self.assertEqual({(t["method"], t["filter"]) for t in second}, set(VARIANTS))
        self.assertNotEqual((first[0]["method"], first[0]["filter"]), (second[0]["method"], second[0]["filter"]))
        self.assertEqual(FILTER_PARAMETERS, {"radius_m": .1, "spatial_sigma_m": .075, "height_sigma_m": .03, "minimum_samples": 3})

    def test_timing_is_three_repetitions_per_variant(self):
        matrix = self.matrix("quick")[:9]
        tasks = tasks_for(matrix, "timing")
        self.assertEqual(len(tasks), 405)
        self.assertEqual({t["repetition"] for t in tasks}, {0, 1, 2})
        self.assertEqual(len({str(prefix_for(Path("/tmp/filter-test"), t)) for t in tasks}), 405)

    def test_none_parity_detects_byte_change_without_replacing_reference(self):
        case = {"name": "fit__flat__missing50__101__r50", "phase": "fit", "n": 1}
        task = {"case": case, "method": "baseline", "filter": "none", "repetition": 0}
        with tempfile.TemporaryDirectory() as temp:
            source, output = Path(temp)/"source", Path(temp)/"output"
            old = source/"outputs"/case["name"]/"baseline-ground.bin"
            new = Path(str(prefix_for(output, task))+"-ground.bin")
            old.parent.mkdir(parents=True); new.parent.mkdir(parents=True)
            old.write_bytes(b"original"); new.write_bytes(b"original")
            self.assertTrue(none_parity(source, output, [task])["passed"])
            new.write_bytes(b"changed")
            self.assertFalse(none_parity(source, output, [task])["passed"])
            self.assertEqual(old.read_bytes(), b"original")

    def test_output_identity_includes_filter_and_exact_binary_size(self):
        case = {"name": "case", "phase": "fit", "n": 2, "resolution_m": .05}
        task = {"case": case, "method": "baseline", "filter": "median", "repetition": 0}
        with tempfile.TemporaryDirectory() as temp:
            prefix = Path(temp)/"baseline__median"
            summary = {"method": "baseline", "filter": "median", "n": 2, "mode": "fit", "resolution_m": .05,
                       "minimum_confidence": .2, "maximum_residual_m": .03, "confidence_weight": 2.}
            Path(str(prefix)+"-summary.json").write_text(json.dumps(summary))
            Path(str(prefix)+"-ground.bin").write_bytes(bytes(2*2*6*4))
            self.assertTrue(valid_output(prefix, task))
            summary["resolution_m"] = .10
            Path(str(prefix)+"-summary.json").write_text(json.dumps(summary))
            self.assertFalse(valid_output(prefix, task))
            summary["resolution_m"] = .05
            summary["filter"] = "bilateral"
            Path(str(prefix)+"-summary.json").write_text(json.dumps(summary))
            self.assertFalse(valid_output(prefix, task))

    def test_timing_reference_rejects_smoke_failed_and_different_inputs(self):
        frozen = {"probe_sha256": "probe", "linked_core": {"sha256": "lib"}, "source_sha256": {"source": "hash"},
                  "input_sha256": {"case": "input"}, "truth_sha256": {"case": "truth"}}
        reference = {"config": {"suite": "full", "phase": "all", "frozen": frozen}, "complete": True,
                     "expected_runs": 20752, "completed_runs": 20752, "freeze_mismatch": [], "failures": []}
        validate_timing_reference(reference, frozen, [{"name": "case"}])
        for label, value in (("suite", "quick"), ("phase", "fit")):
            changed = json.loads(json.dumps(reference)); changed["config"][label] = value
            with self.assertRaises(ValueError): validate_timing_reference(changed, frozen, [{"name": "case"}])
        for label, value in (("complete", False), ("freeze_mismatch", ["changed"]), ("failures", [{"error": "failed"}]), ("completed_runs", 20751)):
            changed = json.loads(json.dumps(reference)); changed[label] = value
            with self.assertRaises(ValueError): validate_timing_reference(changed, frozen, [{"name": "case"}])
        for label in ("input_sha256", "truth_sha256"):
            changed = json.loads(json.dumps(reference)); changed["config"]["frozen"][label]["case"] = "different"
            with self.assertRaises(ValueError): validate_timing_reference(changed, frozen, [{"name": "case"}])


class FilterAnalysisTests(unittest.TestCase):
    def test_no_timeout_has_empty_independent_recheck_population(self):
        self.assertEqual(recheck_platform_results(Path("/tmp/unused"), {"serial_timeout_recheck": {"expected_runs": 0}}), [])

    def test_serial_recheck_retains_parallel_timeout_and_path_diagnostic(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); heights = root/"input.f32"; truth = root/"truth.npz"
            np.zeros((2, 2), dtype="<f4").tofile(heights)
            np.savez(truth, height=np.zeros((2, 2)), interior=np.ones((2, 2), dtype=bool),
                     hazard_foot=np.ones((2, 2), dtype=bool), hazard_wheel=np.ones((2, 2), dtype=bool))
            case = {"name": "integrated__flat__missing50__101__r50", "phase": "integrated", "scene": "flat",
                    "pattern": "missing50", "seed": 101, "resolution_m": .05, "n": 2,
                    "input": str(heights), "truth": str(truth), "sha256": "fixture"}
            task = {"case": case, "method": "baseline", "filter": "none", "repetition": 0}
            for serial in (False, True):
                prefix = prefix_for(root/"timeout-rechecks" if serial else root, task)
                prefix.parent.mkdir(parents=True, exist_ok=True)
                summary = {"method": "baseline", "filter": "none", "n": 2, "mode": "integrated",
                           "resolution_m": .05, "minimum_confidence": .2, "maximum_residual_m": .03,
                           "confidence_weight": 2., "fit_ms": 1., "platforms": {}}
                Path(str(prefix)+"-ground.bin").write_bytes(bytes(2*2*6*4))
                for platform in ("foot", "wheel"):
                    Path(str(prefix)+f"-{platform}.bin").write_bytes(bytes(2*2*2*4))
                    Path(str(prefix)+f"-{platform}-path.csv").write_text("x,y\n0.025,0.025\n0.075,0.025\n")
                    summary["platforms"][platform] = {"status": 0 if serial else 2, "reached": serial,
                                                     "plan_found": serial, "path_points": 2, "map_ms": 1., "plan_ms": 1.}
                Path(str(prefix)+"-summary.json").write_text(json.dumps(summary))
            records = recheck_platform_results(root, {"serial_timeout_recheck": {"cases": [case]}})
            self.assertEqual(len(records), 2)
            self.assertTrue(all(r["parallel_status"] == 2 and r["serial_status"] == 0 and r["timeout_disappeared"] for r in records))
            self.assertTrue(all(r["path_analytic_hazard_samples"] > 0 for r in records))
            self.assertTrue(all(r["run_role"] == "serial_timeout_recheck" for r in records))

    def test_observed_denoising_compares_same_accepted_observed_cells(self):
        observed = np.array([[.04, .20], [np.nan, -.02]])
        ground = np.full((2, 2, 6), np.nan)
        ground[0, 0, 0] = .01; ground[1, 1, 0] = -.01
        data = {"interior": np.ones((2, 2), dtype=bool), "height": np.zeros((2, 2))}
        row = observed_metrics(observed, ground, data)
        self.assertEqual(row["observed_total_count"], 3)
        self.assertEqual(row["observed_estimate_count"], 2)
        self.assertAlmostEqual(row["observed_estimate_accepted_rate"], 2/3)
        self.assertAlmostEqual(row["observed_estimate_rmse_m"], .01)
        self.assertAlmostEqual(row["raw_observed_rmse_same_accepted_m"], np.sqrt((.04**2+.02**2)/2))

    def test_common_targets_reject_different_accepted_sets(self):
        a = np.full((1, 3, 6), np.nan); b = a.copy()
        a[0, :, 0] = (0., .10, np.nan); b[0, :, 0] = (0., np.nan, .20)
        row = common_metrics(a, b, np.zeros((1, 3)), np.ones((1, 3), dtype=bool))
        self.assertEqual(row["count"], 1); self.assertEqual(row["candidate_rmse_m"], 0.)
        row = common_metrics(a, b, np.zeros((1, 3)), np.zeros((1, 3), dtype=bool))
        self.assertEqual(row["count"], 0); self.assertIsNone(row["candidate_rmse_m"])

    def test_paired_seed_ci_excludes_deterministic_repeats(self):
        rows = []
        for seed in (101, 211, 307, 401, 503):
            for filter_method, value in (("none", .03), ("median", .01)):
                rows.append({"case": str(seed), "phase": "fit", "pattern": "noise015", "seed": seed,
                             "method": "baseline", "filter": filter_method, "height_rmse_m": value})
                rows.append({"case": "occlusion"+str(seed), "phase": "fit", "pattern": "occlusion", "seed": seed,
                             "method": "baseline", "filter": filter_method, "height_rmse_m": 10. if filter_method == "median" else 0.})
        result = next(r for r in filter_bootstrap(rows, "height_rmse_m") if r["method"] == "baseline" and r["filter"] == "median")
        self.assertEqual(result["paired_cases"], 5); self.assertEqual(result["seed_clusters"], 5)
        self.assertAlmostEqual(result["delta"], -.02)
        self.assertTrue(all(abs(x+.02) < 1e-12 for x in result["ci95"]))


if __name__ == "__main__": unittest.main()
