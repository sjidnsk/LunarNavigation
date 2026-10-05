#!/usr/bin/env python3
"""Historical single-frame behavior/coverage comparison, without truth accuracy."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics

import numpy as np

from run_p3 import METHODS, FILTERS, VARIANTS, prefix, dump, verify_results, valid_output


def timings(rows):
    output = {}
    for field in ("map_ms", "start_patch_ms", "plan_ms", "fit_and_export_loop_ms"):
        values = [row[field] for row in rows]
        output[field] = {"mean": statistics.mean(values), "median": statistics.median(values),
                         "min": min(values), "max": max(values), "sample_p95": float(np.quantile(values, .95))}
    return output


def changes(directory, method, filter_method, raw):
    base = prefix(directory, method, "none", 0)
    candidate = prefix(directory, method, filter_method, 0)
    before = np.fromfile(str(base)+"-fine.i8", dtype="i1")
    after = np.fromfile(str(candidate)+"-fine.i8", dtype="i1")
    a = np.fromfile(str(base)+"-ground.bin", "<f4").reshape(-1, 6)
    b = np.fromfile(str(candidate)+"-ground.bin", "<f4").reshape(-1, 6)
    observed = np.isfinite(raw); missing = ~observed
    accepted_a, accepted_b = np.isfinite(a[:, 0]), np.isfinite(b[:, 0])
    common = missing&accepted_a&accepted_b
    delta = np.abs(a[:, 0][common]-b[:, 0][common])
    record = {"method": method, "filter": filter_method, "reference_filter": "none",
              "fine_changed_cells": int((before != after).sum()),
              "missing_candidates_gained": int((missing&~accepted_a&accepted_b).sum()),
              "missing_candidates_lost": int((missing&accepted_a&~accepted_b).sum()),
              "common_missing_candidates": int(common.sum()),
              "common_missing_height_changed_gt_1um": int((delta > 1e-6).sum()),
              "median_absolute_height_change_m": float(np.median(delta)) if len(delta) else None,
              "height_change_is_accuracy_error": False}
    for old_state, old_name in ((-1, "unknown"), (0, "free"), (100, "blocked")):
        for new_state, new_name in ((-1, "unknown"), (0, "free"), (100, "blocked")):
            record[old_name+"_to_"+new_name] = int(((before == old_state)&(after == new_state)).sum())
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path)
    args = parser.parse_args(); directory = args.artifact.resolve()
    manifest = json.loads((directory/"manifest.json").read_text())
    historical = Path(manifest["frozen"]["historical"])
    if len(manifest.get("runs", [])) != 45: raise SystemExit("analysis requires all 45 outputs")
    verification = verify_results(directory, historical, 3)
    equality = json.loads((directory/"repeat-artifact-equality.json").read_text())
    by_variant = {(r["method"], r["filter"]): r for r in equality}
    aggregates = []
    for method, filter_method in VARIANTS:
        rows = []
        for repeat in range(3):
            output_prefix = prefix(directory, method, filter_method, repeat)
            if not valid_output(output_prefix, method, filter_method, repeat): raise SystemExit("invalid output: "+str(output_prefix))
            rows.append(json.loads(Path(str(output_prefix)+"-summary.json").read_text()))
        aggregates.append({"method": method, "filter": filter_method, "repeats": 3, "independent_frames": 1,
                           "same_bytes": by_variant[method, filter_method]["same_bytes"],
                           "same_semantic_summary": by_variant[method, filter_method]["same_semantic_summary"],
                           "example": rows[0], "timing": timings(rows)})
    dump(directory/"comparison-summary.json", aggregates)
    raw = np.fromfile(manifest["frozen"]["input"], "<f4")
    differences = [changes(directory, method, filter_method, raw) for method in METHODS for filter_method in FILTERS[1:]]
    dump(directory/"changes-vs-none.json", differences)
    baseline = next(r["example"] for r in aggregates if r["method"] == "baseline" and r["filter"] == "none")
    expected = {"fine_free_count": 1660, "fine_blocked_count": 44290, "fine_unknown_count": 56450, "path_points": 16}
    ready_variants = [r for r in aggregates if r["example"]["start_patch_status_name"] == "READY"]
    filtered_ready = [r for r in ready_variants if r["filter"] != "none"]
    patch_execution = {"builder_invocations": 45,
                       "ready_variants": len(ready_variants),
                       "variants_with_positive_assumed_cells": sum(r["example"]["assumed_cells"] > 0 for r in aggregates),
                       "filtered_ready_variants": len(filtered_ready),
                       "accepted_filtered_patch_request_validation": "EXECUTED_SINGLE_FRAME_ONLY" if filtered_ready else "NOT_RUN",
                       "internal_branch_coverage": "not instrumented; UNRESOLVED alone does not locate the rejection branch"}
    analysis = {"complete": manifest["complete"], "runs_analyzed": 45, "independent_frames": 1,
                "verification": verification, "historical_baseline_counts": expected,
                "baseline_counts_reproduced": all(baseline[k] == v for k,v in expected.items()),
                "variants": aggregates, "changes_vs_none": differences,
                "patch_execution": patch_execution,
                "historical_frame_stamp": manifest["historical_frame_stamp"], "boundaries": manifest["boundaries"]}
    dump(directory/"analysis.json", analysis)
    text = ["# P3 历史单帧：五种拟合 × 三种滤波", "",
            f"串行完成 {len(manifest['runs'])}/45 次；complete={str(manifest['complete']).lower()}。只有一个独立历史帧，每组合重复三次。",
            f"旧 none 对照逐位一致：{verification['none_parity_passed']}；十五组合重复输出逐位稳定：{verification['all_fifteen_variants_repeat_stable']}。",
            "", "输入为原始320×320、5 cm高程帧，17438格有观测、84962格缺测；本地图坐标原点及起终点保持历史设置。",
            "参考位姿比高程帧约晚2.2秒。本帧没有独立地面真值，以下计数、补值和路径变化不能解释为精度或安全改善。",
            "", "| 拟合/滤波 | 候选补值 | 规划补值 | FREE | BLOCKED | UNKNOWN | 核心结果 | 路径顶点 | 补丁 | 假定格 | 构图均值ms | 补丁均值ms | 规划均值ms |",
            "|---|---:|---:|---:|---:|---:|---|---:|---|---:|---:|---:|---:|"]
    for row in aggregates:
        e = row["example"]; t = row["timing"]
        text.append(f"| {row['method']}/{row['filter']} | {e['candidate_ground_infilled_count']} | {e['planning_ground_infilled_count']} | {e['fine_free_count']} | {e['fine_blocked_count']} | {e['fine_unknown_count']} | {e['local_status_name']} | {e['path_points']} | {e['start_patch_status_name']} | {e['assumed_cells']} | {t['map_ms']['mean']:.2f} | {t['start_patch_ms']['mean']:.2f} | {t['plan_ms']['mean']:.2f} |")
    text += ["", "| 拟合/滤波 | 起点状态/置信度 | 终点状态/置信度 | 路径补值顶点 | 路径假定顶点 | 路径原始UNKNOWN顶点 |",
             "|---|---|---|---:|---:|---:|"]
    for row in aggregates:
        e = row["example"]
        text.append(f"| {row['method']}/{row['filter']} | {e['start']['fine_state']}/{e['start']['fine_confidence']:.4f} | {e['goal']['fine_state']}/{e['goal']['fine_confidence']:.4f} | {e['path_infilled_points']} | {e['path_assumed_points']} | {e['path_fine_unknown_points']} |")
    text += ["", "`changes-vs-none.json` 保留同一后端的格子状态转换、候选增减和共同候选的高度变化；高度变化不是对真值的误差。",
             "`none-parity.json` 比较每次none的地面、状态、评分、路径原文件和旧语义JSON；时间及重复ID单列，新增诊断字段不作为旧文件字段。",
             "`repeat-artifact-equality.json` 比较每组合三份地面、状态、评分、路径字节与去除时间/重复ID后的全部新诊断JSON。",
             "", "起点补丁的支撑平面拟合和原始样本过滤共用baseline；最终patched FineCellEvaluator使用该组合的拟合和滤波，假定输入可能参与滤波。",
             f"本帧READY补丁组合={len(ready_variants)}，其中启用滤波的READY组合={len(filtered_ready)}；接受补丁进入滤波规划请求的验证状态为{patch_execution['accepted_filtered_patch_request_validation']}。",
             "所有45次均调用补丁构建器；内部拒绝分支未插桩，UNRESOLVED本身不能证明执行或未执行了patched evaluator，不能把构建器调用等同于补丁验证通过。",
             "补丁余量保持3 m；补丁构建耗时不在5秒核心规划截止时间内。地图估计和起点补丁的假定格分别记录，不把假定格算成真实观测。",
             "路径统计只覆盖导出顶点，不证明整段扫掠支撑。PLAN_FOUND来自实际核心枚举status=0加非空路径；local_reason_code按核心约定可能为空，未伪造外层Action原因。",
             "三次串行样本的P95只作本帧样本统计；fit_and_export_loop_ms含状态及计数读取，不是纯拟合耗时。",
             "本机核心请求结果不代表同步ROS回放、Humble、Orin、雷达传感器、闭环控制或实车验证，相关层级为NOT_RUN。", ""]
    (directory/"analysis.md").write_text("\n".join(text))
    print(json.dumps({"runs_analyzed": 45, "complete": manifest["complete"], **verification}))


if __name__ == "__main__": main()
