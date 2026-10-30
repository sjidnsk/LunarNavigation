"""1 cm 地图处理微基准；结果不代表全局规划或 MPCC 的性能。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import resource
from time import perf_counter_ns

import numpy as np
import scipy
from scipy.ndimage import distance_transform_edt


RESOLUTION_M = 0.01
AXIS_CELLS = 500
UNKNOWN, FREE, BLOCKED = 0, 1, 2


def fixtures():
    states = np.full((AXIS_CELLS, AXIS_CELLS), FREE, dtype=np.uint8)
    yield "open", states.copy(), False
    corridor = states.copy()
    corridor[:100] = BLOCKED
    corridor[400:] = BLOCKED
    yield "corridor", corridor, False
    clutter = states.copy()
    for row, col in ((50, 60), (90, 370), (160, 110), (180, 390),
                     (330, 60), (340, 370), (420, 170), (430, 430)):
        clutter[row:row + 25, col:col + 35] = BLOCKED
    yield "clutter", clutter, False
    unknown = states.copy()
    unknown[:, 390:450] = UNKNOWN
    yield "unknown_forbidden", unknown.copy(), False
    yield "unknown_allowed", unknown.copy(), True


def field_from_mask(allowed):
    # 窗口外按阻断处理；减去半格对角线，得到相对障碍单元面积的保守净空下界。
    padded = np.pad(allowed, 1, constant_values=False)
    distances = distance_transform_edt(padded, sampling=RESOLUTION_M)[1:-1, 1:-1]
    field = np.maximum(0.0, distances - RESOLUTION_M / math.sqrt(2.0)).astype(np.float32)
    return field, np.gradient(field, RESOLUTION_M)


def validate_lower_bound(allowed, field, queries):
    # 使用独立的点到单元矩形距离，核对抽样查询的下界；不计入性能样本。
    obstacles = np.argwhere(~np.pad(allowed, 1, constant_values=False))
    errors = []
    for row, col in queries[:32]:
        delta = np.maximum(np.abs(obstacles - (row + 1, col + 1)) - 0.5, 0.0)
        exact = float(np.sqrt(np.min(np.sum(delta * delta, axis=1)))) * RESOLUTION_M
        errors.append(float(field[row, col]) - exact)
    if max(errors) > 2e-6:
        raise AssertionError("距离场高估了障碍单元净空")
    if np.any(field[~allowed] != 0.0):
        raise AssertionError("阻断单元的净空必须为零")
    return {"queries_checked": len(errors), "max_overestimate_m": max(errors)}


def summary(values):
    array = np.asarray(values)
    return {"p50": float(np.percentile(array, 50)),
            "p95": float(np.percentile(array, 95)),
            "p99": float(np.percentile(array, 99)),
            "max": float(array.max())}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=100)
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    output = args.output.resolve()
    if not output.is_relative_to(root) or output.exists():
        raise ValueError("结果必须写入验证目录的新文件")
    if not 5 <= args.repeats <= 1000:
        raise ValueError("repeats 必须在 5 到 1000 之间")
    queries = np.random.default_rng(20261026).integers(0, AXIS_CELLS, size=(1024, 2))
    results = []
    for name, source, allow_unknown in fixtures():
        samples = {stage: [] for stage in ("copy_mask", "field_gradient", "queries", "total")}
        for repeat in range(args.repeats + 5):
            t0 = perf_counter_ns()
            current = source.copy()
            allowed = current != BLOCKED if allow_unknown else current == FREE
            t1 = perf_counter_ns()
            field, gradients = field_from_mask(allowed)
            t2 = perf_counter_ns()
            values = field[queries[:, 0], queries[:, 1]]
            grad_values = np.stack([g[queries[:, 0], queries[:, 1]] for g in gradients])
            if not (np.isfinite(values).all() and np.isfinite(grad_values).all()):
                raise AssertionError("查询出现非有限值")
            t3 = perf_counter_ns()
            if repeat >= 5:
                for key, elapsed in (("copy_mask", t1-t0), ("field_gradient", t2-t1),
                                     ("queries", t3-t2), ("total", t3-t0)):
                    samples[key].append(elapsed / 1e6)
        validation = validate_lower_bound(allowed, field, queries)
        totals = samples["total"]
        results.append({"case": name, "allow_unknown": allow_unknown,
                        "map_sha256": hashlib.sha256(source.tobytes()).hexdigest(),
                        "stages_ms": {k: summary(v) for k, v in samples.items()},
                        "over_50ms": sum(t > 50.0 for t in totals),
                        "validation": validation, "samples_ms": samples})
    report = {
        "scope": "map_copy_mask_distance_field_gradient_and_1024_cell_queries_only",
        "excluded": ["global_search", "reference_extraction", "candidate_generation", "MPCC",
                     "trajectory_sweep", "terrain_classification", "sensor_input", "ROS_transport"],
        "hostname": platform.node(), "machine": platform.machine(),
        "numpy_version": np.__version__, "scipy_version": scipy.__version__,
        "resolution_m": RESOLUTION_M, "window_m": [5.0, 5.0],
        "shape": [AXIS_CELLS, AXIS_CELLS], "repeats": args.repeats,
        "peak_process_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "results": results,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    for case in results:
        print(json.dumps({"case": case["case"], **case["stages_ms"]["total"],
                          "over_50ms": case["over_50ms"]}))


if __name__ == "__main__":
    main()
