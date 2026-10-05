import csv
from pathlib import Path
import tempfile
import unittest

import numpy as np

from analyze import fit_metrics, path_diagnostics, paired_bootstrap, spearman, grouped_platform, common_support_bootstrap
from protocol import Case, truth


class AnalysisTests(unittest.TestCase):
    def test_accuracy_reports_rejected_missing_cells_in_denominator(self):
        observed = np.full((2, 2), np.nan)
        ground = np.full((2, 2, 6), np.nan)
        ground[0, 0] = (0., .8, 0., 0., 0., 0.)
        ground[0, 1] = (.65, .7, 0., 0., 0., 0.)
        data = {"interior": np.ones((2, 2), dtype=bool), "smooth": np.ones((2, 2), dtype=bool),
                "height": np.zeros((2, 2)), "gradient": np.zeros((2, 2, 2))}
        metrics, bins = fit_metrics({}, observed, ground, data)
        self.assertEqual(metrics["accepted_coverage"], .5)
        self.assertEqual(metrics["recovered03_rate"], .25)
        self.assertEqual(metrics["harmful03_rate"], .25)
        self.assertEqual(metrics["high_confidence_bad10_count"], 1)
        self.assertEqual(sum(b["count"] for b in bins), 2)

    def test_path_checks_segment_interiors_not_only_waypoints(self):
        state = np.zeros((20, 20)); hazard = np.zeros((20, 20), dtype=bool)
        hazard[5, 10] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"path.csv"
            path.write_text("x,y\n0.15,0.55\n1.75,0.55\n")
            result = path_diagnostics(path, state, state, hazard, .1)
        self.assertGreater(result["path_analytic_hazard_samples"], 0)
        self.assertGreater(result["path_samples"], 100)
        self.assertAlmostEqual(result["path_length_m"], 1.6)

    def test_spearman_ties_and_cluster_paired_bootstrap(self):
        self.assertAlmostEqual(spearman(np.array([1., 1., 2., 2.]), np.array([4., 4., 1., 1.])), -1.)
        rows = []
        for seed in (101, 211, 307, 401, 503):
            for method, value in (("baseline", .03), ("mls", .01)):
                rows.append({"case": str(seed), "pattern": "missing50", "seed": seed,
                             "method": method, "height_rmse_m": value})
                rows.append({"case": "occlusion"+str(seed), "pattern": "occlusion", "seed": seed,
                             "method": method, "height_rmse_m": 5. if method == "mls" else .01})
        result = paired_bootstrap(rows, "height_rmse_m")["mls"]
        self.assertEqual(result["seed_clusters"], 5)
        self.assertAlmostEqual(result["delta"], -.02)
        self.assertTrue(all(abs(v+.02) < 1e-12 for v in result["ci95"]))

    def test_ditch_analytic_width_and_curve_gradient(self):
        c = Case("integrated", "ditch15", "dense", 0, .05, 121)
        z, _, _ = truth(c)
        self.assertEqual(int((z[60] < 0).sum()), 3)
        self.assertAlmostEqual((z[60] < 0).sum()*c.resolution_m, .15)
        c = Case("fit", "convex", "dense", 0, .05, 81)
        z, gradient, _ = truth(c)
        finite_difference = (z[40, 51]-z[40, 49])/(2*c.resolution_m)
        self.assertLess(abs(finite_difference-gradient[40, 50, 0]), .001)

    def test_common_support_bootstrap_keeps_pairs_and_excludes_dense(self):
        rows = [{"case": str(seed), "phase": "fit", "pattern": "missing50", "seed": seed,
                 "method": method, "rmse_delta_m": delta}
                for seed in (101, 211, 307, 401, 503) for method, delta in (("mls", -.01), ("gaussian_process", -.02))]
        result = common_support_bootstrap(rows)
        self.assertAlmostEqual(result["mls"]["delta"], -.01)
        self.assertAlmostEqual(result["gaussian_process"]["delta"], -.02)
        self.assertEqual(result["mls"]["paired_cases"], 5)

    def test_platform_aggregate_excludes_baseline_only_dense_references(self):
        shared = dict(platform="foot", status=0, path_points=2, reached=True, map_ms=10., plan_ms=2.,
                      free_count=10, unknown_count=2, analytic_hazard_count=5, analytic_hazard_free_count=1,
                      dense_blocked_count=4, dense_blocked_free_count=1, path_analytic_hazard_samples=0, path_dense_blocked_samples=0)
        rows = [dict(shared, method="baseline", pattern="missing50"),
                dict(shared, method="mls", pattern="missing50"),
                dict(shared, method="baseline", pattern="dense")]
        aggregates = grouped_platform(rows)
        self.assertEqual([row["cases"] for row in aggregates], [1, 1])
        self.assertTrue(all(row["analytic_hazard_free_rate"] == .2 for row in aggregates))
        self.assertTrue(all(row["dense_blocked_free_rate"] == .25 for row in aggregates))
        self.assertTrue(all(row["dense_blocked_free_fraction_of_free"] == .1 for row in aggregates))


if __name__ == "__main__": unittest.main()
