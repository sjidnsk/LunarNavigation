"""Frozen synthetic truth and perturbations for the terrain fitter comparison.

No method sees truth, gradients, hazard masks, or seed labels. All methods receive
the exact same float32 bytes. Seeds are held out from unit tests and are not used
to tune parameters after inspecting aggregate results.
"""
from __future__ import annotations

import hashlib
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

METHODS = ("baseline", "weighted_plane", "mls", "grouped_ransac", "gaussian_process")
SEEDS = (101, 211, 307, 401, 503)
TRUTHS = ("flat", "slope15", "slope25", "slope35", "convex", "concave",
          "step30", "rocks", "crater", "ditch15", "ditch40")
PATTERNS = ("dense", "missing25", "missing50", "missing75", "noise005",
            "noise015", "noise040", "outlier05", "stripes", "range_sparse",
            "occlusion", "mixed")
INTEGRATED = (("flat", "missing50"), ("flat", "noise015"), ("slope15", "missing75"),
              ("slope25", "missing50"), ("slope35", "missing50"),
              ("convex", "missing50"), ("concave", "missing75"),
              ("step30", "noise015"), ("rocks", "outlier05"),
              ("crater", "range_sparse"), ("ditch15", "missing50"),
              ("ditch15", "occlusion"), ("ditch40", "occlusion"),
              ("flat", "stripes"), ("convex", "mixed"))


@dataclass(frozen=True)
class Case:
    phase: str
    scene: str
    pattern: str
    seed: int
    resolution_m: float
    n: int

    @property
    def name(self):
        return f"{self.phase}__{self.scene}__{self.pattern}__{self.seed}__r{round(self.resolution_m*1000)}"


def cases(suite="full", phase="all"):
    seeds = SEEDS if suite == "full" else (SEEDS[0],)
    resolutions = (.05, .10) if suite == "full" else (.05,)
    if phase in ("fit", "all"):
        truths = TRUTHS if suite == "full" else ("flat", "convex", "ditch15")
        patterns = PATTERNS if suite == "full" else ("missing50", "noise015", "occlusion")
        for resolution in resolutions:
            n = 81 if resolution == .05 else 41
            for scene in truths:
                for pattern in patterns:
                    for seed in ((0,) if pattern == "dense" else seeds):
                        yield Case("fit", scene, pattern, seed, resolution, n)
    if phase in ("integrated", "all"):
        specs = INTEGRATED if suite == "full" else (("flat", "missing50"), ("ditch15", "occlusion"))
        # Independent dense software references are always generated with baseline,
        # never treated as physical truth. One per scene/resolution is sufficient.
        for resolution in resolutions:
            n = 121 if resolution == .05 else 61
            for scene in sorted({scene for scene, _ in specs}):
                yield Case("integrated", scene, "dense", 0, resolution, n)
            for scene, pattern in specs:
                for seed in seeds:
                    yield Case("integrated", scene, pattern, seed, resolution, n)


def truth(case):
    r, n = case.resolution_m, case.n
    y, x = (np.indices((n, n)) + .5) * r
    cx, cy = n * r / 2, n * r / 2
    dx, dy = x - cx, y - cy
    z, gx, gy = np.zeros_like(x), np.zeros_like(x), np.zeros_like(x)
    smooth = np.ones_like(x, dtype=bool)
    if case.scene.startswith("slope"):
        gx[:] = np.tan(np.deg2rad(int(case.scene[5:])))
        z = gx * x
    elif case.scene in ("convex", "concave"):
        sign = 1 if case.scene == "convex" else -1
        z = sign * .20 * np.cos(dx * 1.5)
        gx = -sign * .30 * np.sin(dx * 1.5)
    elif case.scene == "step30":
        z = np.where(dx >= 0, .30, 0.)
        smooth = np.abs(dx) > 1.5 * r
    elif case.scene == "rocks":
        d1 = np.hypot(dx, dy); d2 = np.hypot(dx + .8, dy - .7)
        z = np.where((d1 < .35) | (d2 < .22), .65, 0.)
        smooth = (np.abs(d1-.35) > 1.5*r) & (np.abs(d2-.22) > 1.5*r)
    elif case.scene == "crater":
        d2 = dx*dx + dy*dy
        z = -.7 * np.maximum(0., 1. - d2/.8**2)
        gx = np.where(d2 < .8**2, 1.4*dx/.8**2, 0.)
        gy = np.where(d2 < .8**2, 1.4*dy/.8**2, 0.)
        smooth = np.abs(np.sqrt(d2)-.8) > 1.5*r
    elif case.scene.startswith("ditch"):
        half = .075 if case.scene == "ditch15" else .20
        z = np.where(np.abs(dx) < half-1e-9, -.65, 0.)
        smooth = np.abs(np.abs(dx)-half) > 1.5*r
    elif case.scene != "flat":
        raise ValueError(case.scene)
    return z, np.stack((gx, gy), axis=-1), smooth


def perturb(case, z):
    rng = np.random.default_rng(case.seed)
    result = z.copy()
    n, r = case.n, case.resolution_m
    y, x = (np.indices(z.shape)+.5)*r
    cx = cy = n*r/2
    p = case.pattern
    if p.startswith("missing"):
        result[rng.random(z.shape) < int(p[7:])/100] = np.nan
    elif p.startswith("noise"):
        sigma = int(p[5:])/1000
        result += rng.normal(0., sigma, z.shape)
        result[rng.random(z.shape) < .5] = np.nan
    elif p == "outlier05":
        # Five percent of retained observations get large signed height errors.
        outlier = rng.random(z.shape) < .05
        result[outlier] += rng.choice((-.20, .20), int(outlier.sum()))
        result[rng.random(z.shape) < .5] = np.nan
    elif p == "stripes":
        phase = case.seed % 4
        result[(np.indices(z.shape)[0]+phase) % 4 != 0] = np.nan
    elif p == "range_sparse":
        result[rng.random(z.shape) > (.7-.6*x/(n*r))] = np.nan
    elif p == "occlusion":
        result[(np.abs(x-cx) < .175) & (np.abs(y-cy) < 1.)] = np.nan
        if case.scene.startswith("ditch"):
            result[z < 0] = np.nan
    elif p == "mixed":
        result += rng.normal(0., .015, z.shape)
        outlier = rng.random(z.shape) < .02
        result[outlier] += rng.choice((-.15, .15), int(outlier.sum()))
        result[rng.random(z.shape) < .60] = np.nan
        result[(np.abs(x-cx) < .125) & (np.abs(y-cy) < .5)] = np.nan
    elif p != "dense":
        raise ValueError(p)
    return result.astype("<f4")


def interior(case):
    y, x = (np.indices((case.n, case.n))+.5)*case.resolution_m
    margin = .85 if case.phase == "integrated" else .35
    extent = case.n*case.resolution_m
    return (x > margin) & (x < extent-margin) & (y > margin) & (y < extent-margin)


def analytic_hazard(case, z, gradients, smooth, platform):
    """Conservative diagnostic circle footprint over analytic hazards.

    This is independent of fitter scores and production grid states. It is not a
    certification oracle: it ignores contacts, body dynamics and exact step/gap
    traversal. Legged gaps below its configured limit can still be deliberately
    flagged here because this diagnostic models unsupported support, not jumping.
    """
    limit = np.tan(np.deg2rad(30 if platform == "foot" else 20))
    relief_limit = .5 if platform == "foot" else .2
    bad = np.linalg.norm(gradients, axis=-1) > limit+1e-6
    for axis in (0, 1):
        jumps = np.abs(np.diff(z, axis=axis)) > relief_limit
        if axis == 0:
            bad[:-1, :] |= jumps; bad[1:, :] |= jumps
        else:
            bad[:, :-1] |= jumps; bad[:, 1:] |= jumps
    half_diag = np.hypot(.34, .165) if platform == "foot" else np.hypot(.591, .409)
    cells = int(np.ceil(half_diag/case.resolution_m))
    padded = np.pad(bad, cells, constant_values=False)
    result = np.zeros_like(bad)
    for dy in range(-cells, cells+1):
        for dx in range(-cells, cells+1):
            if np.hypot(dx, dy)*case.resolution_m <= half_diag+1e-9:
                result |= padded[cells+dy:cells+dy+case.n, cells+dx:cells+dx+case.n]
    return result


def write_case(case, directory: Path):
    directory.mkdir(parents=True, exist_ok=True)
    z, gradients, smooth = truth(case)
    observed = perturb(case, z)
    input_path = directory/(case.name+".f32")
    input_path.write_bytes(observed.tobytes())
    np.savez_compressed(directory/(case.name+"-truth.npz"), height=z, gradient=gradients,
                        smooth=smooth, interior=interior(case),
                        hazard_foot=analytic_hazard(case, z, gradients, smooth, "foot"),
                        hazard_wheel=analytic_hazard(case, z, gradients, smooth, "wheel"))
    record = asdict(case)
    record.update(name=case.name, input=str(input_path), sha256=hashlib.sha256(observed.tobytes()).hexdigest())
    return record
