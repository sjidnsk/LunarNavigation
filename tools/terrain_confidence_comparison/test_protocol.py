"""Small independent checks for the experimental protocol, not the fitting code."""
import unittest
import numpy as np
from protocol import Case, cases, truth, perturb, analytic_hazard


class ProtocolTests(unittest.TestCase):
    def test_linear_ground_has_exact_gradient_and_no_legged_hazard(self):
        c = Case("fit", "slope25", "dense", 0, .05, 81)
        z, gradient, smooth = truth(c)
        self.assertAlmostEqual(float((z[20, 21]-z[20, 20])/.05), np.tan(np.deg2rad(25)))
        self.assertTrue(smooth.all())
        self.assertFalse(analytic_hazard(c, z, gradient, smooth, "foot").any())
        self.assertTrue(analytic_hazard(c, z, gradient, smooth, "wheel").all())

    def test_hidden_ditch_retains_independent_truth(self):
        c = Case("integrated", "ditch15", "occlusion", 101, .05, 121)
        z, gradient, smooth = truth(c)
        observed = perturb(c, z)
        self.assertTrue(np.all(~np.isfinite(observed[z < 0])))
        self.assertTrue(np.all(z[z < 0] == -.65))
        self.assertTrue(analytic_hazard(c, z, gradient, smooth, "foot")[60, 60])

    def test_reproducible_seed_and_missing_fraction(self):
        c = Case("fit", "flat", "missing75", 211, .05, 81)
        z, _, _ = truth(c)
        a, b = perturb(c, z), perturb(c, z)
        self.assertEqual(a.tobytes(), b.tobytes())
        self.assertLess(abs(np.isnan(a).mean()-.75), .03)

    def test_full_matrix_has_five_held_out_seeds_and_two_resolutions(self):
        matrix = list(cases("full", "fit"))
        group = [c for c in matrix if c.scene == "convex" and c.pattern == "noise015"]
        self.assertEqual(len({c.seed for c in group}), 5)
        self.assertEqual(len({c.resolution_m for c in group}), 2)
        self.assertEqual(len({c.name for c in matrix}), len(matrix))


if __name__ == "__main__": unittest.main()
