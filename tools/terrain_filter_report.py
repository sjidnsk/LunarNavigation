#!/usr/bin/env python3
"""Generate the reviewed Chinese filter report from finished experiment JSON.

Reads only; never runs a probe, compiles code, imports a model, or creates a PDF.
Partial/quick data are rejected before output. --self-test uses tiny arithmetic
fixtures; --check-inputs performs the final read-only completeness audit.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import shlex
import sys
import unittest

METHODS = ("baseline", "weighted_plane", "mls", "grouped_ransac", "gaussian_process")
FILTERS = ("none", "median", "bilateral")
VARIANTS = tuple((m, f) for m in METHODS for f in FILTERS)
ML = dict(zip(METHODS, ("基线", "加权平面", "二次MLS", "分组RANSAC", "局部GP")))
FL = {"none": "无滤波", "median": "中值", "bilateral": "双边"}
SCENES = ("flat", "convex", "concave", "slope15", "slope25", "slope35", "step30", "rocks", "crater", "ditch15", "ditch40")
SL = dict(zip(SCENES, ("平地", "凸面", "凹面", "15°坡", "25°坡", "35°坡", "30cm台阶", "岩块", "凹坑", "15cm沟", "40cm沟")))
NOISE = ("noise005", "noise015", "noise040", "outlier05", "mixed")
PL = {"dense": "全观测", "missing25": "缺25%", "missing50": "缺50%", "missing75": "缺75%",
      "noise005": "5mm噪声", "noise015": "15mm噪声", "noise040": "40mm噪声",
      "outlier05": "5%离群", "mixed": "混合扰动", "occlusion": "遮挡", "stripes": "条带", "range_sparse": "随距离稀疏"}
TIE_M = 1e-6  # Reporting tolerance fixed before reading final results: 0.001mm.
BT = chr(96)
REPO = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def need(condition, message):
    if not condition:
        raise ValueError(message)


def rmse(sq, count):
    return math.sqrt(sq/count) if count else None


def metric(rows):
    """Pool cell counts/squared errors; do not average case RMSE."""
    total = lambda key: sum(r.get(key, 0) for r in rows)
    missing, inferred = total("missing_count"), total("inferred_count")
    measured, accepted = total("observed_total_count"), total("observed_estimate_count")
    high, variance = total("high_confidence_count"), total("variance_positive_count")
    ratio = lambda n, d: n/d if d else None
    return {"cases": len(rows), "missing_count": missing, "inferred_count": inferred,
            "accepted_coverage": ratio(inferred, missing), "accurate03_count": total("accurate03_count"),
            "bad03_count": total("bad03_count"), "bad10_count": total("bad10_count"),
            "recovered03_rate": ratio(total("accurate03_count"), missing),
            "harmful03_rate": ratio(total("bad03_count"), missing),
            "height_rmse_m": rmse(total("height_sq_sum"), inferred),
            "height_max_m": max((r["height_max_m"] for r in rows if r.get("height_max_m") is not None), default=None),
            "observed_total_count": measured, "observed_estimate_count": accepted,
            "observed_estimate_accepted_rate": ratio(accepted, measured),
            "observed_estimate_rmse_m": rmse(total("observed_estimate_sq_sum"), accepted),
            "raw_observed_rmse_same_accepted_m": rmse(total("observed_raw_same_accepted_sq_sum"), accepted),
            "high_confidence_count": high, "high_confidence_bad10_count": total("high_confidence_bad10_count"),
            "high_confidence_bad10_rate": ratio(total("high_confidence_bad10_count"), high),
            "variance_positive_count": variance,
            "conditional_95_interval_coverage": ratio(total("conditional_95_interval_covered"), variance)}


def common(rows):
    n = sum(r["count"] for r in rows)
    a = rmse(sum(r["candidate_sq_sum"] for r in rows), n)
    b = rmse(sum(r["reference_sq_sum"] for r in rows), n)
    return {"cases": len(rows), "count": n, "empty": sum(r["count"] == 0 for r in rows),
            "candidate_rmse_m": a, "reference_rmse_m": b, "delta": a-b if n else None}


def changes(rows):
    values = [r["rmse_delta_m"] for r in rows if r["rmse_delta_m"] is not None]
    return {"improved": sum(v < -TIE_M for v in values), "worsened": sum(v > TIE_M for v in values),
            "tied": sum(abs(v) <= TIE_M for v in values), "empty": len(rows)-len(values), "valid": len(values)}


def select(rows, **keys):
    return [r for r in rows if all(r.get(k) == v for k, v in keys.items())]


def name(m, f):
    return ML[m]+"+"+FL[f]


def fmt(value, scale=1, digits=2, signed=False):
    if value is None:
        return "null"
    need(math.isfinite(value), "non-finite report metric")
    return format(value*scale, ("+" if signed else "")+"."+str(digits)+"f")


def pct(value):
    return fmt(value, 100)+("%" if value is not None else "")


def mm(value, signed=False):
    return fmt(value, 1000, 3, signed)


def ci(row, scale=1000, digits=3):
    if row is None or row.get("delta") is None:
        return "null"
    value = fmt(row["delta"], scale, digits, True)
    bounds = row.get("ci95")
    return value+" [null]" if bounds is None else value+" ["+fmt(bounds[0], scale, digits, True)+", "+fmt(bounds[1], scale, digits, True)+"]"


def link(label, path):
    target = str(path)
    return "["+label.replace("[", "").replace("]", "")+"]("+("<"+target+">" if " " in target else target)+")"


def table(headers, rows):
    clean = lambda v: str(v).replace("|", "\\|").replace("\n", " ")
    out = ["| "+" | ".join(map(clean, headers))+" |", "| "+" | ".join("---" for _ in headers)+" |"]
    out.extend("| "+" | ".join(map(clean, row))+" |" for row in rows)
    return "\n".join(out)


def validate(manifest, summary, count, suite):
    need(manifest.get("complete") is True and summary.get("complete") is True, suite+" incomplete")
    need(manifest.get("expected_runs") == count and manifest.get("completed_runs") == count, suite+" run count incomplete")
    need(summary.get("runs_analyzed") == count, suite+" analysis incomplete")
    config = manifest["config"]
    need(config.get("suite") == suite and config.get("phase") == "all", "wrong suite/phase")
    need(not manifest.get("freeze_mismatch") and not manifest.get("failures"), suite+" freeze/execution failure")
    need(manifest.get("none_parity", {}).get("passed") is True and not manifest["none_parity"].get("mismatches"), suite+" none parity failure")
    need(manifest.get("serial_timeout_recheck", {}).get("complete") is True, suite+" serial timeout confirmation incomplete")
    need(config.get("filter_parameters") == {"radius_m": .1, "spatial_sigma_m": .075, "height_sigma_m": .03, "minimum_samples": 3}, "unexpected filter parameters")
    need(config.get("minimum_confidence") == .2 and config.get("maximum_residual_m") == .03 and config.get("confidence_weight") == 2., "unexpected acceptance/score parameters")


def load(main_dir, timing_dir, review_path, p3_dir):
    manifest, summary = read(main_dir/"manifest.json"), read(main_dir/"summary.json")
    tm, ts = read(timing_dir/"manifest.json"), read(timing_dir/"summary.json")
    review = read(review_path)
    validate(manifest, summary, 20752, "full")
    validate(tm, ts, 405, "timing")
    need(tm["config"].get("workers") == 1, "timing must be serial")
    need(tm["config"].get("reference_manifest_sha256") == sha(main_dir/"manifest.json"), "timing reference does not bind current full manifest")
    need(review.get("review_complete") is True and bool(review.get("engineering_tests")) and bool(review.get("NOT_RUN")), "review/engineering evidence incomplete")
    for key in ("probe_sha256", "linked_core", "source_sha256", "source_manifest_sha256"):
        need(manifest["config"]["frozen"][key] == tm["config"]["frozen"][key], "main/timing frozen "+key+" mismatch")
    for key in ("input_sha256", "truth_sha256"):
        original = manifest["config"]["frozen"][key]
        need(all(original.get(k) == v for k, v in tm["config"]["frozen"][key].items()), "timing input/truth mismatch")
    data = {key: read(main_dir/(key+".json")) for key in ("fit-results", "platform-results", "filter-common-support-results", "all-fifteen-common-results", "all-fifteen-pools", "recheck-platform-results")}
    data["timing-aggregate"], data["timing-results"] = read(timing_dir/"timing-aggregate.json"), read(timing_dir/"timing-results.json")
    need(len(data["fit-results"]) == 20752, "fit row count incomplete")
    need(sum(r["platform"] == "fit" for r in data["timing-results"]) == 405, "timing repeat count incomplete")
    need(len(data["timing-aggregate"]) == 195 and all(r["repeats"] == 3 for r in data["timing-aggregate"]), "timing aggregation not complete three repeats")
    groups = defaultdict(set)
    for row in data["fit-results"]:
        need(row["accurate03_count"]+row["bad03_count"] == row["inferred_count"], "useful+harm != coverage")
        identity = row["method"], row["filter"]
        need(identity in VARIANTS and identity not in groups[row["case"]], "duplicate/unrecognized main identity")
        groups[row["case"]].add(identity)
    need(len(groups) == 1404, "wrong main input record count")
    for case in manifest["cases"]:
        wanted = {("baseline", "none")} if case["phase"] == "integrated" and case["pattern"] == "dense" else set(VARIANTS)
        need(groups.get(case["name"]) == wanted, "incomplete variants: "+case["name"])
    need(len(data["all-fifteen-pools"]) == 1360 and all(r["complete_fifteen_variants"] for r in data["all-fifteen-pools"]), "all-fifteen pools incomplete")
    need(len(data["platform-results"]) == 4544, "platform records incomplete")
    for row in data["all-fifteen-common-results"]:
        need((row["count"] > 0) == (row["candidate_rmse_m"] is not None), "empty pool reported as zero error")
    target = [r for r in data["fit-results"] if r["pattern"] != "dense"]
    for agg in summary["aggregate"]:
        own = metric(select(target, phase=agg["phase"], method=agg["method"], filter=agg["filter"]))
        for key in ("cases", "missing_count", "inferred_count", "accepted_coverage", "height_rmse_m", "recovered03_rate", "harmful03_rate", "observed_estimate_rmse_m"):
            left, right = own[key], agg[key]
            need(left == right if left is None or right is None else math.isclose(left, right, rel_tol=1e-12, abs_tol=1e-12), "analyzer/report metric mismatch: "+key)
    need(p3_dir is not None, "P3 45-run artifact is required before final report")
    p3 = load_p3(p3_dir)
    need(p3["manifest"]["references"]["main_manifest_sha256"] == sha(main_dir/"manifest.json"), "P3 main reference changed")
    need(p3["manifest"]["references"]["timing_manifest_sha256"] == sha(timing_dir/"manifest.json"), "P3 timing reference changed")
    need(p3["manifest"]["frozen"]["linked_core"] == manifest["config"]["frozen"]["linked_core"], "P3 actual core differs")
    for path, digest in p3["manifest"]["frozen"]["model_source_sha256"].items():
        need(manifest["config"]["frozen"]["source_sha256"].get(path) == digest, "P3 model source differs: "+path)
    environment=read(main_dir.parent/"environment.json")
    return {**data, "manifest":manifest, "summary":summary, "timing_manifest":tm, "review":review, "environment":environment,
            "main_dir":main_dir, "timing_dir":timing_dir, "review_path":review_path, "p3_dir":p3_dir, "p3":p3}


def load_p3(directory):
    manifest, analysis = read(directory/"manifest.json"), read(directory/"analysis.json")
    validate_p3(manifest, analysis)
    return {"manifest":manifest, "analysis":analysis}


def validate_p3(manifest, analysis):
    need(manifest.get("complete") is True and analysis.get("complete") is True, "P3 incomplete")
    need(manifest.get("expected_runs") == 45 and len(manifest.get("runs",[])) == 45 and
         all(r["valid"] for r in manifest["runs"]) and analysis.get("runs_analyzed") == 45, "P3 run count/validity incomplete")
    need(manifest.get("workers") == 1 and manifest.get("timing_repeats") == 3 and
         manifest.get("independent_frames") == 1 and analysis.get("independent_frames") == 1, "P3 scope mismatch")
    need(manifest.get("final_frozen") == manifest.get("frozen") and not manifest.get("error"), "P3 freeze mismatch")
    for verification in (manifest.get("verification",{}), analysis.get("verification",{})):
        need(verification.get("none_parity_passed") is True and verification.get("none_runs_compared") == 15 and
             verification.get("all_fifteen_variants_repeat_stable") is True and verification.get("variants_compared") == 15,
             "P3 none/repeat equality incomplete")
    need(analysis.get("baseline_counts_reproduced") is True, "P3 historical baseline count changed")
    variants = analysis.get("variants",[])
    need(len(variants) == 15 and {(r["method"],r["filter"]) for r in variants} == set(VARIANTS), "P3 variants incomplete")
    need(all(r["repeats"] == 3 and r["same_bytes"] and r["same_semantic_summary"] for r in variants), "P3 repeat equality changed")
    need(len(analysis.get("changes_vs_none",[])) == 10, "P3 changes vs none incomplete")
    patch=analysis.get("patch_execution",{})
    ready=[r for r in variants if r["example"]["start_patch_status_name"]=="READY"]
    filtered=[r for r in ready if r["filter"]!="none"]
    need(patch.get("builder_invocations")==45 and patch.get("ready_variants")==len(ready) and
         patch.get("filtered_ready_variants")==len(filtered) and
         patch.get("variants_with_positive_assumed_cells")==sum(r["example"]["assumed_cells"]>0 for r in variants),
         "P3 patch execution scope mismatch")
    need(patch.get("accepted_filtered_patch_request_validation")==("EXECUTED_SINGLE_FRAME_ONLY" if filtered else "NOT_RUN"),
         "P3 patch validation scope mismatch")


class Report:
    def __init__(self, data, artifact_root):
        self.d, self.root, self.parts = data, Path(artifact_root).resolve(), []
        self.fit = select(data["fit-results"], phase="fit")
        self.target = [r for r in self.fit if r["pattern"] != "dense"]
        self.pair = select(data["filter-common-support-results"], phase="fit")
        self.all_common = select(data["all-fifteen-common-results"], phase="fit")
        self.pool = select(data["all-fifteen-pools"], phase="fit")
        self.platforms = [r for r in data["platform-results"] if r["pattern"] != "dense"]
        self.snapshot = {"overall":[], "pair_common":[], "observed":[], "platforms":[]}

    def add(self, text):
        self.parts.append(text.strip())

    def heading(self, text):
        self.add("## "+text)

    def source(self, label, relative):
        return link(label, self.root/relative)

    def code(self, label, relative, anchor=None):
        path = REPO/relative
        need(path.is_file(), "missing source link: "+relative)
        if anchor:
            found = [i+1 for i, v in enumerate(path.read_text(encoding="utf-8").splitlines()) if anchor in v]
            need(bool(found), "missing source anchor: "+anchor)
            return link(label, str(path)+":"+str(found[0]))
        return link(label, path)

    def render(self):
        for section in (self.opening,self.mechanism,self.protocol,self.overall,self.common_results,self.noise,
                        self.layers,self.controls,self.observed,self.counterexamples,self.platform_results,
                        self.timing,self.p3_results,self.validation,self.reproduction):
            section()
        return "\n\n".join(self.parts)+"\n"

    def opening(self):
        self.add("# 地面拟合加入滤波后的十五组合对比分析报告")
        self.add("日期：2026-10-05。开发分支："+BT+"dev/terrain-confidence"+BT+"。本轮在已有五种拟合方法前加入可选中值和双边滤波；默认仍为baseline+none。")
        self.add("正式主矩阵完成20,752次探针，另完成405次独立串行重复计时和45次历史P3单帧复现，共21,202次正式运行，冒烟167次另计。滤波减轻已有邻点的高度噪声，不能替没有回波的地方创造测量。下面分别检查缺测补值、已有观测误差、平台通行和路径，以及计算开销。")
        self.conclusions()
        values = []
        for resolution in (.05,.10):
            for f in FILTERS[1:]:
                rows = [r for r in self.pair if r["method"] == "weighted_plane" and r["filter"] == f and r["resolution_m"] == resolution and r["pattern"] in ("noise005","noise015","noise040")]
                m = common(rows)
                values.append([str(round(resolution*100))+"cm",FL[f],m["count"],mm(m["reference_rmse_m"]),mm(m["candidate_rmse_m"]),mm(m["delta"],True)])
        self.add("先看用户关心的加权平面噪声恢复：下表合并11种地形和5个种子的5/15/40mm噪声输入，比较滤波与自身none都接受的相同缺测位置。负差表示误差降低；完整覆盖与噪声分层见后文，不能凭此表单独选择默认设置。")
        self.add(table(["网格","加权平面滤波","共同格","none RMSE(mm)","滤后RMSE(mm)","变化(mm)"],values))
        self.add("默认继续none。总体RMSE容易被隐藏沟主导，保守方法也可能只接受容易的位置。是否启用滤波，要同时看准确补值、错误补值、共同位置、路径和时间，不能用一个全局平均数宣布最优。")

    def conclusions(self):
        summary=self.d["summary"]
        lookup={(r["method"],r["filter"]):r for r in summary["filter_common_support_seed_bootstrap"]}
        med,bi=lookup["weighted_plane","median"],lookup["weighted_plane","bilateral"]
        self.add("结论是：本轮不值得默认开启滤波。加权平面在990个随机案例共同位置上，中值的案例平均RMSE反而增加"+mm(med["delta"])+"mm（95%种子簇区间["+mm(med["ci95"][0])+","+mm(med["ci95"][1])+"]），双边增加"+mm(bi["delta"])+"mm（["+mm(bi["ci95"][0])+","+mm(bi["ci95"][1])+"]）；这些幅度较小，但不支持“稳定改善”。原始观测同位置对照也显示，原有稳健拟合已经在压低噪声，追加滤波的收益余地有限。")
        def paired(f,res,pattern,scene=None):
            group=select(self.pair,method="weighted_plane",filter=f,resolution_m=res,pattern=pattern)
            return common(select(group,scene=scene) if scene else group)
        c=paired("median",.05,"noise040");b=paired("bilateral",.05,"noise040")
        coarse=paired("median",.10,"noise040");coarse_b=paired("bilateral",.10,"noise040")
        mild=paired("median",.05,"noise005");slope=paired("median",.05,"missing50","slope25")
        self.add("确有局部收益：5cm网格、40mm噪声合并各地形时，中值共同位置RMSE为"+mm(c["reference_rmse_m"])+"→"+mm(c["candidate_rmse_m"])+"mm，双边为"+mm(b["reference_rmse_m"])+"→"+mm(b["candidate_rmse_m"])+"mm；这两行的共同集合不同，不能横比它们谁更准。10cm同条件改善仅"+mm(-coarse["delta"])+"/"+mm(-coarse_b["delta"])+"mm。5cm轻噪声中值则为"+mm(mild["reference_rmse_m"])+"→"+mm(mild["candidate_rmse_m"])+"mm；5cm、无噪声、缺50%的25°坡从近零误差增到"+mm(slope["candidate_rmse_m"])+"mm，说明中值也会改变本来正确的坡面。")
        wls={f:metric(select(self.target,method="weighted_plane",filter=f)) for f in FILTERS}
        a,c=wls["none"],wls["median"]
        self.add("中值更保守：加权平面总体接受覆盖"+pct(a["accepted_coverage"])+"→"+pct(c["accepted_coverage"])+"，有用补值"+pct(a["recovered03_rate"])+"→"+pct(c["recovered03_rate"])+"，有害补值"+pct(a["harmful03_rate"])+"→"+pct(c["harmful03_rate"])+"。误差下降或错补减少，有一部分来自拒绝更多点；大噪声下拒绝不能直接叫作降噪失败，因为原始危险证据仍必须过关。")
        mls_m,mls_b=lookup["mls","median"],lookup["mls","bilateral"]
        mls_max=metric(select(self.target,method="mls",filter="median"))["height_max_m"]
        self.add("MLS的随机共同位置平均误差下降"+mm(-mls_m["delta"])+"/"+mm(-mls_b["delta"])+"mm（中值/双边），但中值后的拟合阶段最大误差仍达"+fmt(mls_max,1,3)+"m，这是仍保留的严重反例，不能因平均改善就推荐MLS默认。滤波没有解决曲面过冲或隐藏沟。这组平台的解析危险FREE和危险路径案例计数未因滤波降低；加权平面足式/轮式仍分别找到58/59条路径（各150例），GP中值则减少到53/48条，不能用地图更平滑代替实际规划收益。")
        times=self.d["timing-aggregate"]
        rates=[]
        for f in FILTERS[1:]:
            own=select(times,phase="fit",resolution_m=.05,method="weighted_plane",filter=f,platform="fit")
            ratios=[]
            for r in own:
                ref=select(times,phase=r["phase"],resolution_m=r["resolution_m"],scene=r["scene"],pattern=r["pattern"],method="weighted_plane",filter="none",platform="fit")[0]
                ratios.append(r["fit_ms_median"]/ref["fit_ms_median"])
            rates.append(fmt(min(ratios))+"-"+fmt(max(ratios))+"倍")
        self.add("计算代价明确增加：预选7个5cm拟合输入上，加权平面中值/双边的串行中位耗时为自身none的"+rates[0]+"/"+rates[1]+"。保留none默认；滤波可作为特定场景的可选实验配置，需用分层覆盖、共同误差和实际路径判断取舍。")

    def mechanism(self):
        self.heading("1. 这次改了什么，哪些原始证据继续保留")
        self.add(table(["设置","通俗理解","固定参数与行为"],[
            ["none","按原高度拟合","默认，保留原算术路径"],
            ["median","每个点参考身边点的中间高度","欧氏半径10cm，含自身；至少3点；偶数平均中间两值"],
            ["bilateral","参考附近且高度相近的点","相同10cm邻域；空间σ7.5cm，高度σ3cm；原高度作参考"]]))
        self.add("每个候选拟合窗口内滤波一遍，仅更改拟合代表高度；保留坐标、样本数量和原始最高/最低高度，不写回原图，也不把刚滤过或补出的值变成下一次观测。邻点不足3个保持原高度。全部点参考同一份未滤波输入，所以不是递归平滑。同一点在不同候选窗口内可能有不同结果，不是全图固定预处理。")
        self.add("通用原理可参见[OpenCV滤波官方说明](https://docs.opencv.org/4.13.0/d4/d13/tutorial_py_filtering.html)：中值取邻域中间值，双边同时考虑空间和高度差。本项目是自己的高程点邻域C++实现，没有调用OpenCV；本轮参数不是该文推荐参数，也不是雷达标定值。")
        self.add("五后端仍为稳健平面基线、距离加权稳健平面、局部二次MLS、分组RANSAC和局部GP，滤波独立于后端。中值会压低少数异常高度，也可能压平少数真实表面；双边保护明显高差，所以大离群点可能被保留，斜坡的有效邻居也可能得到低权重。")
        self.add("拟合候选轴向最大30cm，最近支持20cm；拟合后原始共识至少4点且占70%，仍检查二维分布与方向包围。原始区间残差上限10cm，中心原始高度差4cm，邻接残差跳变6cm，二阶差8cm，另一个一致表面也继续检查。可信度残差来自原始样本，滤波不能仅靠抹平台阶或尖点提高分数。滤后模型即使更准，原始噪声仍可能导致拒绝；本轮评估的是保留原始证据的完整流程。")
        self.add("滤波分组RANSAC额外对原始样本做一次同参数分组拟合。最终共识为“滤后选组∩原始选组∩原始残差合格”，还必须至少4点、只有一个连通分量且严格包围目标。这避免中值把桥点压平连起两个小组、删除桥点后还假装完整支撑。双拟合和最终连通检查都计入时间；none不增加这些约束。它可能更保守，也可能更慢。")
        prefix = "ros2_ws/src/lunar_incremental_navigation_core/src/map/"
        links = [self.code("一次滤波",prefix+"terrain_sample_filter.cpp","FilterTerrainSamples"),
                 self.code("原始/拟合样本分离",prefix+"terrain_observation_estimator.cpp","const auto& fit_points"),
                 self.code("原始共识交集",prefix+"terrain_observation_estimator.cpp","if(raw_group_support)"),
                 self.code("连通与严格包围",prefix+"terrain_grouped_ransac.cpp","HasConnectedGroupedRansacSupport")]
        self.add("源码入口："+"；".join(links)+"。原理："+self.code("滤波机制说明","docs/terrain-confidence/滤波机制说明.md")+"；"+self.code("五拟合方法说明","docs/terrain-confidence/四种拟合方法说明.md")+"。")

    def protocol(self):
        self.heading("2. 如何保证比较对象相同")
        m, tm, parity = self.d["manifest"], self.d["timing_manifest"], self.d["manifest"]["none_parity"]
        self.add(f"主矩阵{m['completed_runs']:,}/{m['expected_runs']:,}次，1404输入记录、{m['unique_input_sha256_count']}个唯一输入SHA256。拟合1232输入各跑十五组合；集成150扰动输入各跑十五组合，另22个全观测基线软件参考。原始float32字节相同；真值、危险掩码和种子标签不进入算法。重复种子和重复计时不是新增独立场景。")
        self.add(table(["层级","输入数","运行数","用途"],[["拟合",1232,18480,"含全观测；最终通过检查的估计"],["集成扰动",150,2250,"足/轮通行图与核心规划"],["集成全观测参考",22,22,"仅baseline+none软件对照"],["串行计时",9,405,"15组合×3新进程"],["历史P3",1,45,"15组合×3新进程；无真值"]]))
        self.add(table(["设置","范围"],[
            ["地形","平地、凸/凹面、15/25/35°坡、30cm台阶、65cm岩块、70cm凹坑、15/40cm宽65cm深沟"],
            ["模式","全观测；缺25/50/75%；缺50%+5/15/40mm噪声；缺50%+5%±20cm离群；条带；随距离稀疏；遮挡；混合"],
            ["混合","缺60%、15mm噪声、2%±15cm离群及中央遮挡"],
            ["种子","101、211、307、401、503；全观测0"],
            ["门槛","最低证据分数0.2、原始残差3cm、评分代价权重2、规划截止5秒"]]))
        self.add("5/10cm是栅格间距，不是雷达精度。拟合图边长4.05/4.10m，集成图6.05/6.10m。15cm解析沟离散为3/1列、负高度栅格并集宽15/10cm；40cm解析沟为7/3列、并集宽35/30cm。10cm还改变3×3检查物理跨度和RANSAC连通尺度16→21cm。10cm规则网格的10cm滤波半径最多含自身及4轴向邻格，斜对角不在半径内；5cm最多13点。稀疏后更少，分辨率差不能全部归因于滤波。")
        self.add(f"主矩阵及计时freeze_mismatch=[]，输入、真值、源码、脚本、程序及实际核心库执行前后核对SHA256。新none共有{parity['ground_checked']}份地面和{parity['state_checked']}份足/轮状态二进制与旧输出逐位一致：地面6个float32字段，状态/可信度2字段；不比较受环境影响的路径、summary或时间。")
        self.add("主矩阵"+str(m["config"]["workers"])+"个调度worker，并行时间只作诊断。独立计时单worker，绑定完成的主manifest SHA。"+self.source("主manifest","full/manifest.json")+"；"+self.source("串行manifest","timing/manifest.json")+"。")
        self.add("主矩阵的地面误差、已有观测误差和缺测覆盖只统计冻结真值中的interior评估区：拟合阶段格中心距四边均大于35cm，集成阶段均大于85cm。下文“所有缺测格/已有观测格”均指该预定内部区域；平台FREE/BLOCKED/UNKNOWN及危险格分母也只算同区，例如每个平台150案例状态总数为719550格，并非150张整图的全部格子。评估区在实验前固定，不按算法结果改变。路径危险诊断检查完整导出线段并记录越界，不按interior裁剪；P3单帧计数范围另行说明。")
        self.add(table(["指标","含义"],[["接受覆盖","接受缺测格/所有缺测格"],["有用补值","接受且误差≤3cm/所有缺测格"],["有害补值","接受且误差>3cm/所有缺测格；不是事故率"],["RMSE","接受格误差平方均值开根号；逐格加权"],["共同位置","同输入下两组合或全部十五组合都接受的缺测格"],["观测点误差","本组合自身接受的已有观测格上估计/原始对照"],["危险FREE/路径","解析危险及密集软件BLOCKED对照分别计数"]]))
        self.add("五种子区间：逐案例配对差→种子内平均→5种子簇重采样10000次（固定1601），排除全观测、相同遮挡重复和仅两个相位的条带。相邻格不是独立样本；区间描述这组试验的种子敏感性，不代表所有月表的总体范围。")

    def overall(self):
        self.heading("3. 十五组合总体覆盖与误差")
        n = metric(select(self.target,method="baseline",filter="none"))["missing_count"]
        self.add(f"1210个非全观测拟合输入，{n:,}个缺测评价格。覆盖=有用+有害。拒绝的格没有误差，不等于零误差。RMSE汇总平方和再开根号，不平均每案例RMSE。")
        rows=[]
        for m,f in VARIANTS:
            v=metric(select(self.target,method=m,filter=f));self.snapshot["overall"].append({"method":m,"filter":f,**v})
            rows.append([name(m,f),pct(v["accepted_coverage"]),pct(v["recovered03_rate"]),pct(v["harmful03_rate"]),fmt(v["height_rmse_m"],100,3),fmt(v["height_max_m"],100,2)])
        self.add(table(["组合","覆盖","有用≤3cm","有害>3cm","RMSE(cm)","最大(cm)"],rows))
        self.add("![十五组合覆盖、误差和有害补值]("+str(self.root/"full/filter-overview.png")+")")
        self.add("各组合RMSE来自自身接受集合，不等于同一批位置排名；隐藏沟和误差尾部见第9节。"+self.source("逐案例结果","full/fit-results.json")+"；"+self.source("汇总","full/summary.json")+"。")

    def common_results(self):
        self.heading("4. 相同位置上，滤波改善还是变差")
        self.add("先取同一后端滤波与自身none都接受的缺测格。误差两列位置完全相同；仍需结合全缺测覆盖，避免只留下容易位置。")
        rows=[]
        for m in METHODS:
            for f in FILTERS[1:]:
                v=common(select(self.pair,method=m,filter=f));self.snapshot["pair_common"].append({"method":m,"filter":f,**v})
                rows.append([name(m,f),v["count"],v["empty"],mm(v["reference_rmse_m"]),mm(v["candidate_rmse_m"]),mm(v["delta"],True)])
        self.add(table(["组合/自身none","共同格","空案例","none(mm)","滤后(mm)","差(mm)"],rows))
        n=sum(r["common_count"] for r in self.pool);empty=sum(r["common_count"]==0 for r in self.pool)
        self.add(f"十五组合共同池共有{n:,}格，{empty}/1210案例为空。各组合与自身none比较，但都用同一池。共同池可能只剩容易位置；空池误差是null，不能写成零误差。")
        rows=[]
        for m,f in VARIANTS:
            v=common(select(self.all_common,method=m,filter=f))
            rows.append([name(m,f),v["count"],mm(v["reference_rmse_m"]),mm(v["candidate_rmse_m"]),mm(v["delta"],True)])
        self.add(table(["组合","十五共同格","自身none(mm)","本组合(mm)","差(mm)"],rows))
        self.add("随机案例共同位置RMSE差按种子汇总如下；负改善、正变差，区间跨零不支持稳定单向变化。改善/变差计数采用同一随机案例，|变化|≤0.001mm为数值持平，空池另计。案例宏平均差与上面逐格加权差口径不同。")
        bs={(r["method"],r["filter"]):r for r in self.d["summary"]["filter_common_support_seed_bootstrap"]}
        ab={(r["method"],r["filter"]):r for r in self.d["summary"]["all_fifteen_common_seed_bootstrap"]}
        rows=[]
        for m in METHODS:
            for f in FILTERS[1:]:
                group=[r for r in select(self.pair,method=m,filter=f) if r["pattern"] not in ("dense","occlusion","stripes")]
                ch=changes(group);b=bs[m,f];need(ch["valid"]==b["paired_cases"],"change/CI population mismatch")
                rows.append([name(m,f),str(b["paired_cases"])+"/"+str(b["seed_clusters"]),ci(b),ci(ab[m,f]),str(ch["improved"])+"/"+str(ch["worsened"])+"/"+str(ch["tied"]),ch["empty"]])
        self.add(table(["组合","案例/种子","二者共同差[95%](mm)","十五共同差[95%](mm)","改善/变差/持平","空池"],rows))
        lookup={k:{(r["method"],r["filter"]):r for r in values} for k,values in self.d["summary"]["filter_paired_seed_bootstrap"].items()}
        rows=[]
        for m in METHODS:
            for f in FILTERS[1:]:
                key=m,f
                rows.append([name(m,f),ci(lookup["accepted_coverage"][key],100),ci(lookup["recovered03_rate"][key],100),ci(lookup["harmful03_rate"][key],100),ci(lookup["height_rmse_m"][key])])
        self.add("各自接受集合的配对区间也保留。前三列为百分点，最后RMSE集合可能不同，不能替代共同位置。")
        self.add(table(["组合","覆盖差[95%]pp","有用差[95%]pp","有害差[95%]pp","各自RMSE差[95%]mm"],rows))
        self.add("来源："+self.source("二者共同位置","full/filter-common-support-results.json")+"；"+self.source("十五共同位置","full/all-fifteen-common-results.json")+"；"+self.source("共同池","full/all-fifteen-pools.json")+"；"+self.source("种子区间","full/summary.json")+"。")

    def noise(self):
        self.heading("5. 加权平面重点：噪声层级和5/10cm")
        self.add("每行合并11地形、5种子。5/15/40mm噪声叠加50%缺测；离群为5%±20cm并缺50%；混合还含遮挡。各自接受RMSE与右表共同位置对照配套。")
        rows,pairs=[],[]
        for res in (.05,.10):
            for pat in NOISE:
                for f in FILTERS:
                    v=metric(select(self.target,method="weighted_plane",filter=f,pattern=pat,resolution_m=res))
                    label=str(round(res*100))+"cm/"+PL[pat]
                    rows.append([label,FL[f],pct(v["accepted_coverage"]),pct(v["recovered03_rate"]),pct(v["harmful03_rate"]),mm(v["height_rmse_m"])])
                    if f!="none":
                        group=select(self.pair,method="weighted_plane",filter=f,pattern=pat,resolution_m=res);c=common(group);ch=changes(group)
                        pairs.append([label,FL[f],c["count"],mm(c["reference_rmse_m"]),mm(c["candidate_rmse_m"]),str(ch["improved"])+"/"+str(ch["worsened"])+"/"+str(ch["tied"])])
        self.add(table(["网格/扰动","滤波","覆盖","有用","有害","RMSE(mm)"],rows))
        self.add(table(["网格/扰动","滤波","共同格","none(mm)","滤后(mm)","改善/变差/持平"],pairs))
        self.add("![加权平面按网格和噪声分层的共同位置误差]("+str(self.root/"report-charts/wls-common-error-by-noise.png")+")")
        self.add("图中每一对none/滤波只比较同一批位置，分别展示5/10cm和无噪声缺50%、5/15/40mm噪声，合并11地形及5种子。中值与双边面板的共同池不同，不能横比各自none曲线；不同噪声条件的接受集合也会变化，曲线不表示同一批格子随噪声变大的误差轨迹。")
        self.add("大噪声恢复仍受原始3cm残差和跳变检查限制。保守拒绝不等同于滤波没有降噪：它可能把模型高度估得更准，却因为原始证据不足而不允许用来补值。反过来，补值增加也不等于更准确，必须看有用/有害和共同误差。双边高度σ3cm，明显离群点可能几乎只信自己，不能预设它比中值更能去离群。10cm网格邻点更少，很多点不足3样本。"+self.source("分层结果","full/scene-results.json")+"；"+self.source("共同位置","full/filter-common-support-results.json")+"。")

    def layers(self):
        self.heading("6. 平地、曲面、三坡、台阶和沟分别看")
        self.add("每格为滤波与自身none共同位置RMSE差(mm，负改善)，合并该地形11种非全观测模式和5种子。不同后端的共同格数仍可不同，完整计数见JSON。")
        for f in FILTERS[1:]:
            self.add("### "+FL[f]+"滤波")
            rows=[]
            for res in (.05,.10):
                for scene in SCENES:
                    rows.append([str(round(res*100))+"cm/"+SL[scene]]+[mm(common(select(self.pair,method=m,filter=f,scene=scene,resolution_m=res))["delta"],True) for m in METHODS])
            self.add(table(["网格/地形"]+[ML[m] for m in METHODS],rows))
        self.add("曲面需区分模型表达弯曲的能力与滤波改变的曲率。MLS用二次薄片，平面用小板；滤波可能降噪也可能偏离真实形状。坡面中值在不对称稀疏邻域可能偏向一侧，双边也可能降权同坡有效邻点。台阶/沟边仍用原始证据保护，没有观测的沟底不会变成已测地面。")
        rows=[]
        for scene in ("flat","convex","concave","slope15","slope25","slope35","step30","ditch15","ditch40"):
            for f in FILTERS:
                v=metric(select(self.target,method="weighted_plane",filter=f,scene=scene))
                rows.append([SL[scene],FL[f],pct(v["accepted_coverage"]),pct(v["recovered03_rate"]),pct(v["harmful03_rate"]),mm(v["height_rmse_m"])])
        self.add("加权平面逐地形覆盖和有害补值(合并两分辨率)，用来审查误差改善是否来自覆盖损失：")
        self.add(table(["地形","滤波","覆盖","有用","有害","RMSE(mm)"],rows))

    def controls(self):
        self.heading("7. 无噪声对照：本来好的输入是否受影响")
        self.add("全观测没有缺测格，补值覆盖与补值误差为null。这里只看已有观测的接受率和误差。原始输入有float32量化，并非严格实数零误差。")
        rows=[]
        for m,f in VARIANTS:
            v=metric(select(self.fit,method=m,filter=f,pattern="dense"))
            rows.append([name(m,f),pct(v["observed_estimate_accepted_rate"]),mm(v["raw_observed_rmse_same_accepted_m"]),mm(v["observed_estimate_rmse_m"]),v["missing_count"],"null"])
        self.add(table(["组合","观测估计接受","原始RMSE(mm)","估计RMSE(mm)","缺测格","补值RMSE"],rows))
        self.add("再合并缺25/50/75%但不加噪声的输入。滤波并非总改善：不对称地形、曲率或真实高差都可能被改变。")
        rows=[]
        for m,f in VARIANTS:
            group=[r for r in select(self.target,method=m,filter=f) if r["pattern"] in ("missing25","missing50","missing75")]
            v=metric(group)
            c=common([r for r in select(self.pair,method=m,filter=f) if r["pattern"] in ("missing25","missing50","missing75")]) if f!="none" else None
            rows.append([name(m,f),pct(v["accepted_coverage"]),pct(v["harmful03_rate"]),mm(v["height_rmse_m"]),mm(c["delta"],True) if c else "0（自身）"])
        self.add(table(["组合","覆盖","有害","接受RMSE(mm)","共同差(mm)"],rows))
        self.add(self.source("全观测/缺测逐案例","full/fit-results.json")+"。密集软件参考不能替代独立传感器真值或车辆安全真值。")

    def observed(self):
        self.heading("8. 已测到的点：估计是否比原始噪声更准")
        self.add("完全排除缺测补值。每组合仅在自身接受的观测位置上，比较估计与原始输入误差，二者位置相同。不同组合的接受位置仍可能不同，不能以跨组合不同位置的RMSE公平排名。")
        rows=[]
        for m,f in VARIANTS:
            v=metric(select(self.target,method=m,filter=f));self.snapshot["observed"].append({"method":m,"filter":f,**v})
            a,b=v["observed_estimate_rmse_m"],v["raw_observed_rmse_same_accepted_m"]
            rows.append([name(m,f),str(v["observed_estimate_count"])+"/"+str(v["observed_total_count"]),pct(v["observed_estimate_accepted_rate"]),mm(b),mm(a),mm(a-b,True) if a is not None and b is not None else "null"])
        self.add(table(["组合","接受/已有观测格","接受率","原始RMSE(mm)","估计RMSE(mm)","估计-原始(mm)"],rows))
        rows=[]
        for res in (.05,.10):
            for pat in NOISE:
                for f in FILTERS:
                    v=metric(select(self.target,method="weighted_plane",filter=f,pattern=pat,resolution_m=res))
                    rows.append([str(round(res*100))+"cm/"+PL[pat],FL[f],pct(v["observed_estimate_accepted_rate"]),mm(v["raw_observed_rmse_same_accepted_m"]),mm(v["observed_estimate_rmse_m"])])
        self.add("加权平面噪声分层同位置观测误差：")
        self.add(table(["网格/扰动","滤波","观测接受","原始RMSE(mm)","估计RMSE(mm)"],rows))
        self.add("估计误差降低不代表原始传感器数据已被清理，原始图和危险证据保留。"+self.source("独立观测格字段","full/fit-results.json")+"。")

    def counterexamples(self):
        self.heading("9. 隐藏沟和误差尾部：不能被平均值掩盖")
        self.add("遮挡沟移除全部沟底观测，只剩看似平整两岸。即使滤波完全正确也没有新增沟底信息；两岸共面不能证明中间存在支撑。下表按单个确定性输入计数，5种子相同不当作5次独立遮挡。")
        group=select(self.d["fit-results"],phase="integrated",scene="ditch15",pattern="occlusion")
        rows=[]
        for m,f in VARIANTS:
            values=[]
            for res in (.05,.10):
                same=select(group,method=m,filter=f,resolution_m=res)
                need(len(same)==5 and all(len({r[field] for r in same})==1 for field in
                     ("inferred_count","bad10_count","high_confidence_bad10_count")), "deterministic hidden-ditch repetitions changed")
                v=metric(same)
                values.append(str(v["inferred_count"]//5)+"/"+str(v["bad10_count"]//5)+"/"+str(v["high_confidence_bad10_count"]//5))
            rows.append([name(m,f)]+values)
        self.add(table(["组合","5cm接受/>10cm/高分错格","10cm接受/>10cm/高分错格"],rows))
        self.add("高分错格=分数≥0.6且高度误差>10cm，分数不是安全概率。真实通行还取决于平台范围和规划规则，下一节独立统计平台危险。")
        rows=[]
        for m,f in VARIANTS:
            worst=max((r for r in select(self.target,method=m,filter=f) if r["height_max_m"] is not None),key=lambda r:r["height_max_m"],default=None)
            if worst:rows.append([name(m,f),worst["case"],fmt(worst["height_max_m"],100,2),fmt(worst["height_rmse_m"],100,3),pct(worst["accepted_coverage"])])
        self.add("每组合拟合阶段最大误差案例。MLS需审查二次曲面过冲；滤波可能改善尾部，也可能改变局部支持后带来新的尾部。本轮未加入曲率正则化或按真值调参。")
        self.add(table(["组合","最大误差案例","最大(cm)","该例RMSE(cm)","该例覆盖"],rows))
        rows=[]
        for f in FILTERS:
            worst=max((r for r in select(self.d["fit-results"],phase="integrated",method="mls",filter=f) if r["height_max_m"] is not None),key=lambda r:r["height_max_m"],default=None)
            if worst:rows.append([name("mls",f),worst["case"],fmt(worst["height_max_m"],100,2),fmt(worst["height_rmse_m"],100,3)])
        self.add(table(["MLS集成组合","最大误差案例","最大(cm)","该例RMSE(cm)"],rows))
        self.add("GP方差是给定核、噪声、平面均值和已选支持的条件方差，未校准为地面存在性或车辆风险。滤波引入样本相关性，但固定核/噪声没有重新训练，名义95%区间只作诊断。")
        rows=[]
        for f in FILTERS:
            v=metric(select(self.target,method="gaussian_process",filter=f))
            rows.append([FL[f],v["variance_positive_count"],pct(v["conditional_95_interval_coverage"]),v["high_confidence_bad10_count"],pct(v["high_confidence_bad10_rate"])])
        self.add(table(["GP滤波","正方差接受格","名义95%区间真值覆盖","高分>10cm错格","高分>10cm错率"],rows))
        self.add("来源："+self.source("尾部/方差字段","full/fit-results.json")+"；"+self.source("分数分箱","full/confidence-bins.json")+"。汇总最大值没有格级坐标，不能独自判定实现错误；原始输入和ground二进制保留供复查。本轮未重做全套分数保留比例排序，旧排序不能充作新滤波结果。")

    def platform_results(self):
        self.heading("10. 两平台的软件状态与路径")
        self.add("每组合每平台都是同150扰动案例，排除仅baseline+none的22密集参考。UNKNOWN也不能通行。解析危险用真值坡度、高差和外接圆，密集BLOCKED来自软件参考，分开计数。完整路径线段≤1cm采样；危险诊断不是事故率，找到路径也不自动等于到达最终目标。")
        for platform,label in (("foot","足式"),("wheel","轮式")):
            self.add("### "+label)
            states,paths=[],[]
            for m,f in VARIANTS:
                group=select(self.platforms,platform=platform,method=m,filter=f);need(len(group)==150,"wrong platform population")
                totals={k:sum((r.get(k) or 0) for r in group) for k in ("free_count","blocked_count","unknown_count","analytic_hazard_free_count","analytic_hazard_count","dense_blocked_free_count","dense_blocked_count")}
                found=sum(bool(r["plan_found"]) for r in group); reached=sum(bool(r["reached"]) for r in group)
                timeout=sum(r["status"]==2 for r in group);haz=sum(r["path_analytic_hazard_samples"]>0 for r in group);dense=sum((r["path_dense_blocked_samples"] or 0)>0 for r in group)
                self.snapshot["platforms"].append({"platform":platform,"method":m,"filter":f,"cases":150,**totals,"found":found,"reached":reached,"timeout":timeout,"path_hazard_cases":haz,"path_dense_blocked_cases":dense})
                states.append([name(m,f),totals["free_count"],totals["blocked_count"],totals["unknown_count"],str(totals["analytic_hazard_free_count"])+"/"+str(totals["analytic_hazard_count"]),str(totals["dense_blocked_free_count"])+"/"+str(totals["dense_blocked_count"])])
                paths.append([name(m,f),found,reached,timeout,str(haz)+"/"+str(found),str(dense)+"/"+str(found)])
            self.add(table(["组合","FREE","BLOCKED","UNKNOWN","解析危险FREE/危险","参考BLOCKED→FREE/参考"],states))
            self.add(table(["组合","找到路径/150","最终到达/150","TIMEOUT","路径解析危险/找到","路径参考BLOCKED/找到"],paths))
        self.add("### 并行超时的独立串行确认")
        r=self.d["summary"]["serial_timeout_recheck"]
        self.add(f"{r['case_count']}个输入触发平台TIMEOUT，全部十五组合串行复核产生{r['platform_records']}平台记录。复核输入原并行TIMEOUT{r['parallel_timeout_platform_records']}条，串行所有平台TIMEOUT{r['serial_timeout_platform_records']}条；原超时消失{r['timeout_disappeared_records']}条，重复{r['repeated_timeout_records']}条。复核单独保存，不替换主150统计。")
        checks=self.d["recheck-platform-results"]
        if checks:
            rows=[]
            for platform in ("foot","wheel"):
                for m,f in VARIANTS:
                    group=select(checks,platform=platform,method=m,filter=f)
                    rows.append([platform+"/"+name(m,f),len(group),sum(r["parallel_status"]==2 for r in group),sum(r["serial_status"]==2 for r in group),sum(bool(r["serial_plan_found"]) for r in group),sum(r["path_analytic_hazard_samples"]>0 for r in group)])
            self.add(table(["平台/组合","复核例","并行超时","串行超时","串行找到","串行路径危险"],rows))
        else:self.add("没有TIMEOUT需复核；空JSON与CSV表头保留，零计数不是漏测。")
        self.add(self.source("主平台案例","full/platform-results.json")+"；"+self.source("独立串行确认","full/recheck-platform-results.json")+"。未验证ROS Action、同步传感器或车辆执行。")

    def timing(self):
        self.heading("11. 独立405次串行计时")
        rows=self.d["timing-aggregate"]
        self.add("预先9输入×15组合×3新进程，单worker，同程序/库/源码/输入真值。以下为中位/P95(ms)；P95仅3样本，不代表长期尾延迟。本机后台负载未隔离，不能外推Orin。主并行计时不进入本节。地图构建已包含所需估计，不能把地图时间与独立拟合时间简单相加；规划另列。")
        cases=sorted({(r["phase"],r["scene"],r["pattern"],r["resolution_m"]) for r in rows if r["platform"]=="fit"})
        for f in FILTERS:
            self.add("### "+FL[f]+"：九输入地面估计时间")
            values=[]
            for phase,scene,pat,res in cases:
                row=[("集成/" if phase=="integrated" else "拟合/")+SL[scene]+"/"+PL[pat]+"/"+str(round(res*100))+"cm"]
                for m in METHODS:
                    records=select(rows,phase=phase,scene=scene,pattern=pat,resolution_m=res,method=m,filter=f,platform="fit");need(len(records)==1,"missing timing identity")
                    v=records[0];row.append(fmt(v["fit_ms_median"])+"/"+fmt(v["fit_ms_p95"]))
                values.append(row)
            self.add(table(["输入"]+[ML[m]+"中位/P95" for m in METHODS],values))
        for scene,pat,res in (("flat","missing50",.05),("ditch15","occlusion",.10)):
            self.add("### 集成："+SL[scene]+"/"+PL[pat]+"/"+str(round(res*100))+"cm")
            values=[]
            for m,f in VARIANTS:
                row=[name(m,f)]
                for platform in ("foot","wheel"):
                    records=select(rows,phase="integrated",scene=scene,pattern=pat,resolution_m=res,method=m,filter=f,platform=platform);need(len(records)==1,"missing integrated timing identity")
                    v=records[0];row.extend([fmt(v["map_ms_median"])+"/"+fmt(v["map_ms_p95"]),fmt(v["plan_ms_median"])+"/"+fmt(v["plan_ms_p95"]),"/".join(map(str,v["statuses"]))])
                values.append(row)
            self.add(table(["组合","足地图中位/P95","足规划中位/P95","足状态3次","轮地图中位/P95","轮规划中位/P95","轮状态3次"],values))
        self.add("状态0=找到路径，1=无路径，2=TIMEOUT；最终到达字段另存JSON。滤波RANSAC的双拟合和最终mask复核、滤波邻域遍历和后端求解全部计入。allocated_bytes只是探针暴露的缓存统计，不包含全部临时滤波/求解分配，不能当作RSS或总内存。"+self.source("逐次计时","timing/timing-results.json")+"；"+self.source("三次统计","timing/timing-aggregate.json")+"。")
        values=[]
        for m in METHODS:
            for f in FILTERS[1:]:
                ratios=[]
                for phase,scene,pat,res in cases:
                    before=select(rows,phase=phase,scene=scene,pattern=pat,resolution_m=res,method=m,filter="none",platform="fit")[0]["fit_ms_median"]
                    after=select(rows,phase=phase,scene=scene,pattern=pat,resolution_m=res,method=m,filter=f,platform="fit")[0]["fit_ms_median"]
                    if before>0:ratios.append(after/before)
                values.append([name(m,f),fmt(min(ratios))+"×",fmt(max(ratios))+"×"])
        self.add("滤波相对自身none的拟合时间中位数倍率范围，逐个相同输入比较，保留九输入最小/最大而不把不同输入时延混成单一平均：")
        self.add(table(["组合","九输入最小倍率","九输入最大倍率"],values))

    def p3_results(self):
        self.heading("12. 历史P3单帧辅助对照")
        analysis=self.d["p3"]["analysis"];variants=analysis["variants"]
        self.add("串行完成45/45，十五组合各3次，但只有1个独立历史高程帧。输入320×320、5cm，17438格有观测、84962格缺测，stamp="+str(analysis["historical_frame_stamp"])+"。参考位姿约晚2.2秒；没有独立高度真值，本节不计算RMSE、错误FREE或安全改善，也不与合成150案例成功率混合。")
        self.add("原起点(7.6133745667,7.9915921022)、yaw=-3.022577，终点本地图X正方向1m。包络0.68×0.33m、背景半宽20cm、盲区补丁余量3m及能力保持历史设置，路径CSV不增加展示平移。五种none各3次共15份，地面、状态、评分、路径字节和旧语义summary全部复现；十五组合三次新输出及去除时间/重复ID的新诊断也分别稳定。")
        rows=[]
        for v in variants:
            e=v["example"]
            rows.append([name(v["method"],v["filter"]),e["candidate_ground_infilled_count"],e["planning_ground_infilled_count"],e["fine_free_count"],e["fine_blocked_count"],e["fine_unknown_count"]])
        self.add(table(["组合","候选补值格","规划有效补值格","FREE","BLOCKED","UNKNOWN"],rows))
        rows=[]
        for v in variants:
            e=v["example"]
            rows.append([name(v["method"],v["filter"]),e["local_status_name"],e["path_points"],fmt(e["path_length_m"],1,3),e["start_patch_status_name"],e["assumed_cells"]])
        self.add(table(["组合","核心结果","路径顶点","长度(m)","起点补丁","假定格"],rows))
        self.add("候选补值是估计器提出高度，规划有效补值经过整机条件；数量增加不是精度改善。baseline+none复现FREE1660、BLOCKED44290、UNKNOWN56450和16点路径，其他none保留旧对照。PLAN_FOUND是实际核心枚举0加非空路径，原reason_code可按核心约定为空，不伪造外层Action原因。")
        gp={r["filter"]:r["example"] for r in variants if r["method"]=="gaussian_process"}
        self.add("本历史帧GP从none的"+gp["none"]["local_status_name"]+"，变为中值/双边的"+gp["median"]["local_status_name"]+"/"+gp["bilateral"]["local_status_name"]+"，路径"+str(gp["median"]["path_points"])+"/"+str(gp["bilateral"]["path_points"])+"点，终点评分"+fmt(gp["median"]["goal"]["fine_confidence"],1,5)+"/"+fmt(gp["bilateral"]["goal"]["fine_confidence"],1,5)+"。这说明滤波能改变本帧端点准入，分数接近0.2门槛；合成主矩阵GP中值的找到路径数却减少，不能把单帧路径恢复推广成普遍改善或安全证明。")
        rows=[]
        for v in variants:
            e=v["example"]
            rows.append([name(v["method"],v["filter"]),e["start"]["fine_state"]+"/"+fmt(e["start"]["fine_confidence"],1,4),
                         e["goal"]["fine_state"]+"/"+fmt(e["goal"]["fine_confidence"],1,4),
                         e["path_infilled_points"],e["path_assumed_points"],e["path_fine_unknown_points"]])
        self.add(table(["组合","起点状态/分数","终点状态/分数","路径补值顶点","路径假定顶点","路径UNKNOWN顶点"],rows))
        self.add("起点补丁的支撑平面和原始样本过滤共用baseline，最终patched FineCellEvaluator使用当前后端和滤波，假定高度可能参与滤波。该视图只对请求隔离，不改原始图。assumed_cells是候选假定数量，补丁未READY时可能未应用；路径来源计数仅导出顶点，不能证明整段扫掠支撑，不能把假定格算成真观测。")
        patch=analysis["patch_execution"]
        self.add("本帧补丁构建器调用"+str(patch["builder_invocations"])+"次；READY组合"+str(patch["ready_variants"])+"个，其中滤波READY为"+str(patch["filtered_ready_variants"])+"个，正假定格组合"+str(patch["variants_with_positive_assumed_cells"])+"个。接受假定补丁进入滤波请求的验证状态："+patch["accepted_filtered_patch_request_validation"]+"。内部拒绝分支未插桩，UNRESOLVED本身不能证明patched evaluator是否执行；Build调用次数也不等于该流程验证通过。")
        rows=[]
        for v in analysis["changes_vs_none"]:
            rows.append([name(v["method"],v["filter"]),v["fine_changed_cells"],v["missing_candidates_gained"],
                         v["missing_candidates_lost"],v["common_missing_candidates"],mm(v["median_absolute_height_change_m"])])
        self.add(table(["组合/自身none","状态变化格","新增候选","失去候选","共同候选","共同高度绝对变化中位(mm)"],rows))
        self.add("共同高度变化是两估计之间的差，不是相对真值的误差；本帧没有真值。各状态转换、路径原始/有效观测、最小路径分数等保留JSON。")
        rows=[]
        for v in variants:
            t=v["timing"];row=[name(v["method"],v["filter"])]
            for field in ("map_ms","start_patch_ms","plan_ms","fit_and_export_loop_ms"):
                row.append(fmt(t[field]["median"])+"/"+fmt(t[field]["sample_p95"]))
            rows.append(row)
        self.add(table(["组合","构图中位/P95(ms)","补丁中位/P95(ms)","核心规划中位/P95(ms)","拟合/导出循环中位/P95(ms)"],rows))
        self.add("5秒截止只作用于核心规划，不包含构图和起点补丁。拟合/导出循环还含状态和计数读取，不能当纯拟合时间。三次样本P95不代表长期尾延迟；单历史帧不代表同步bag、实时ROS、Humble、Orin或车辆闭环。")
        self.add("来源："+self.source("P3分析","p3/results/analysis.json")+"；"+self.source("全部冻结/请求记录","p3/results/manifest.json")+"；"+
                 self.source("旧none逐位对照","p3/results/none-parity.json")+"；"+self.source("三重复逐位稳定","p3/results/repeat-artifact-equality.json")+"。")

    def validation(self):
        self.heading("13. 工程测试与验证边界")
        review=self.d["review"];rows=[]
        for v in review["engineering_tests"]:
            if "suites" in v: detail=f"{v['suites']}套/{v['cases']}例/{v['failures']}失败"
            elif "passed" in v:detail=f"{v['passed']}通过/{v['failed']}失败"
            elif "runs" in v:detail=str(v["runs"])+"探针；complete="+str(v["complete"]).lower()+"；none一致="+str(v["none_parity"]).lower()
            else:detail=str(v.get("cases",""))+"例，失败="+str(v.get("failures",""))
            evidence=Path(v["evidence"])
            if not evidence.is_absolute():evidence=self.d["main_dir"].parent/evidence
            need(evidence.is_file(),"missing engineering evidence: "+str(evidence))
            try: rel=evidence.resolve().relative_to(self.d["main_dir"].parent.resolve())
            except ValueError: target=link("日志",evidence)
            else:target=self.source("日志",rel)
            rows.append([v["name"],v["status"],detail,target])
        self.add(table(["验证","结果","范围","证据"],rows))
        self.add("core与ROS新构建共470测试通过，新协议11测试通过。静态接口67通过、1项既有publisher断言失败；该publisher源SHA与基准adef7f6d相同，未隐藏失败或为滤波改断言。独立只读实现复核完成，不能替代物理验证。")
        environment=self.d["environment"]
        self.add("本机环境："+environment["uname"]["system"]+"/"+environment["uname"]["machine"]+"；ROS "+environment["ROS_DISTRO"]+"；"+environment["compiler"]+"；"+environment["build"]+"构建。"+self.source("环境记录","environment.json")+"。")
        self.add("回归覆盖降噪、稀疏不足、非递归、不新增样本、原始台阶/尖点/二阶差/区间、raw二维退化、默认none、非法枚举、配置哈希、增量更新等于新建和旧快照隔离，以及两种RANSAC桥接反例。0.7m新障碍要求不能FREE，UNKNOWN也是拒绝通行，未强行改为BLOCKED。")
        self.add("NOT_RUN："+"；".join(review["NOT_RUN"])+"。实现commit："+BT+review["implementation_commit"]+BT+"。未合并、推送或部署。"+self.source("工程/复核清单","review-validation.json")+"。")
        patch=self.d["p3"]["analysis"]["patch_execution"]
        if patch["filtered_ready_variants"]:
            patch_scope="历史单帧有滤波READY补丁分支执行，仅限该帧；其他帧和假定补丁真值仍未验证。"
        else:
            patch_scope="历史单帧没有滤波READY补丁，接受假定补丁进入滤波请求仍为NOT_RUN，不能因45次调用构建器而称已经验证。"
        self.add("主矩阵输入是原始高程快照。请求级补丁是隔离假定视图，接口没有逐点来源字段，假定高度可能参与滤波。"+patch_scope+"背景平面、补丁规则和平台包络共用原实现。")
        self.add("单值float32主输入low=high，WLS/MLS区间质量项恒1、GP额外区间宽度噪声为0，不能据此证明真实质量加权收益。本轮只模拟固定高度噪声与离群，没有传感器强度、入射角、位姿协方差或时序；未验证定位偏差、多帧错位和共同系统偏差，不能扩大为解决真实雷达全部噪声。参数不是实际雷达标定。评分不是安全概率、GP条件方差未校准，滤波不能恢复无回波沟底。")

    def reproduction(self):
        self.heading("14. 产物、复现和使用判断")
        m=self.d["manifest"];frozen=m["config"]["frozen"];source=m["config"]["source"]
        self.add("默认baseline+none。候选启用前，应按噪声/地形/网格层审查准确恢复、错误补值、共同位置和平台/路径，再接受串行费用。MLS仍需审查过冲，分组RANSAC需接受保守覆盖和双拟合，GP方差不能代替地面存在性。这些设置留在专用开发分支，本机对照不是月表部署验收。")
        self.add("报告只读JSON，不重新运行模型。入口："+self.source("主summary","full/summary.json")+"；"+self.source("主manifest","full/manifest.json")+"；"+self.source("串行manifest","timing/manifest.json")+"；"+self.source("P3","p3/results/manifest.json")+"；"+self.source("运行记录","full/run-records.jsonl")+"；"+self.source("环境","environment.json")+"。旧输入："+link("上一轮完整manifest",Path(source)/"manifest.json")+"。")
        self.add(table(["冻结对象","SHA256"],[["探针",frozen["probe_sha256"]],["实际核心库",frozen["linked_core"]["sha256"]],["旧输入manifest",frozen["source_manifest_sha256"]],["新主manifest",sha(self.d["main_dir"]/"manifest.json")],["新计时manifest",sha(self.d["timing_dir"]/"manifest.json")]]))
        self.add("模型与实验源共"+str(len(frozen["source_sha256"]))+"项，逐项哈希及输入/真值哈希见manifest。报告生成器和证据在冻结模型/实验目录外，其输入哈希见"+self.source("报告生成记录","report-generation.json")+"。")
        command="\n".join([
            "terrain_source="+shlex.quote(source),"terrain_new=/tmp/terrain-filter-comparison-20261005",'terrain_probe="$terrain_new/probe"',
            "# 先按README用同一Release核心库编译探针，固定源码/参数。","python3 tools/terrain_filter_comparison/runner.py \\",
            '  --source "$terrain_source" --probe "$terrain_probe" \\','  --output "$terrain_new/full" --suite full --workers 4',
            "python3 tools/terrain_filter_comparison/runner.py \\",'  --source "$terrain_source" --probe "$terrain_probe" \\',
            '  --output "$terrain_new/timing" --suite timing --workers 1 \\','  --reference "$terrain_new/full"',
            "python3 tools/terrain_filter_p3/run_p3.py \\",
            '  --probe "$terrain_new/p3/fixed_compare" --input "$terrain_new/p3/raw.f32" \\',
            '  --output "$terrain_new/p3/results" --main-artifact "$terrain_new/full" \\',
            '  --timing-artifact "$terrain_new/timing"',"python3 tools/terrain_filter_report.py \\",
            '  --main "$terrain_new/full" --timing "$terrain_new/timing" \\','  --p3 "$terrain_new/p3/results" --review "$terrain_new/review-validation.json" \\',
            "  --artifact-root "+shlex.quote(str(self.root))+" \\","  --output docs/validation/2026-10-05-terrain-filter-comparison.md \\",
            '  --body-output "$terrain_new/报告正文.md"'])
        self.add("复现应选新的输出根，不覆盖已保存产物；resume由runner核对相同指纹。P3原frame和probe也需按说明保存并构建。构建/接口："+self.code("本轮README","tools/terrain_filter_comparison/README.md")+"；"+self.code("原探针构建","tools/terrain_confidence_comparison/README.md")+"；"+self.code("P3构建与输入","tools/terrain_filter_p3/README.md")+"。")
        self.add(BT*3+"bash\n"+command+"\n"+BT*3)
        self.add("生成器轻量自测："+BT+"python3 tools/terrain_filter_report.py --self-test"+BT+"。完整性检查：同生成参数改用"+BT+"--check-inputs"+BT+"，不写报告。")


class SelfTests(unittest.TestCase):
    def test_pooled_grid_rmse(self):
        rows=[{"missing_count":1,"inferred_count":1,"height_sq_sum":.01,"accurate03_count":0,"bad03_count":1},
              {"missing_count":99,"inferred_count":99,"height_sq_sum":0.,"accurate03_count":99,"bad03_count":0}]
        v=metric(rows)
        self.assertAlmostEqual(v["height_rmse_m"],.01)
        self.assertEqual(v["accepted_coverage"],v["recovered03_rate"]+v["harmful03_rate"])

    def test_empty_is_null(self):
        self.assertIsNone(metric([])["height_rmse_m"])
        self.assertIsNone(common([{"count":0,"candidate_sq_sum":0.,"reference_sq_sum":0.}])["candidate_rmse_m"])
        self.assertEqual(fmt(None),"null")

    def test_common_pool(self):
        v=common([{"count":1,"candidate_sq_sum":.0004,"reference_sq_sum":.0009},
                  {"count":3,"candidate_sq_sum":0.,"reference_sq_sum":0.}])
        self.assertAlmostEqual(v["candidate_rmse_m"],.01)
        self.assertAlmostEqual(v["reference_rmse_m"],.015)
        self.assertAlmostEqual(v["delta"],-.005)

    def test_change_count(self):
        self.assertEqual(changes([{"rmse_delta_m":v} for v in (-.001,.001,0.,1e-7,None)]),
                         {"improved":1,"worsened":1,"tied":2,"empty":1,"valid":4})

    def test_incomplete_rejected(self):
        with self.assertRaisesRegex(ValueError,"incomplete"):
            validate({"complete":False},{"complete":False},20752,"full")
        with self.assertRaisesRegex(ValueError,"run count"):
            validate({"complete":True,"expected_runs":167,"completed_runs":167},{"complete":True},20752,"full")

    def test_freeze_parity_and_parameters_gate(self):
        def fixture():
            return {"complete":True,"expected_runs":20752,"completed_runs":20752,
                    "config":{"suite":"full","phase":"all","minimum_confidence":.2,
                              "maximum_residual_m":.03,"confidence_weight":2.,
                              "filter_parameters":{"radius_m":.1,"spatial_sigma_m":.075,"height_sigma_m":.03,"minimum_samples":3}},
                    "freeze_mismatch":[],"failures":[],"none_parity":{"passed":True,"mismatches":[]},
                    "serial_timeout_recheck":{"complete":True}}
        summary={"complete":True,"runs_analyzed":20752}
        validate(fixture(),summary,20752,"full")
        bad=fixture();bad["freeze_mismatch"]=["model changed"]
        with self.assertRaisesRegex(ValueError,"freeze"):
            validate(bad,summary,20752,"full")
        bad=fixture();bad["none_parity"]["mismatches"]=["ground bytes differ"]
        with self.assertRaisesRegex(ValueError,"parity"):
            validate(bad,summary,20752,"full")
        bad=fixture();bad["serial_timeout_recheck"]["complete"]=False
        with self.assertRaisesRegex(ValueError,"timeout"):
            validate(bad,summary,20752,"full")
        bad=fixture();bad["config"]["filter_parameters"]["radius_m"]=.2
        with self.assertRaisesRegex(ValueError,"parameters"):
            validate(bad,summary,20752,"full")

    def test_markdown(self):
        self.assertIn("a\\|b",table(["name"],[["a|b"]]))
        self.assertIn("[null]",ci({"delta":0.,"ci95":None}))
        self.assertEqual(link("src","/tmp/a b"),"[src](</tmp/a b>)")

    def test_p3_equality_gate(self):
        proof={"none_parity_passed":True,"none_runs_compared":15,
               "all_fifteen_variants_repeat_stable":True,"variants_compared":15}
        manifest={"complete":True,"expected_runs":45,"runs":[{"valid":True} for _ in range(45)],
                  "workers":1,"timing_repeats":3,"independent_frames":1,"frozen":{"model":"hash"},
                  "final_frozen":{"model":"hash"},"verification":proof}
        analysis={"complete":True,"runs_analyzed":45,"independent_frames":1,"verification":proof,
                  "baseline_counts_reproduced":True,"variants":[{"method":m,"filter":f,"repeats":3,
                  "same_bytes":True,"same_semantic_summary":True,
                  "example":{"start_patch_status_name":"NOT_NEEDED","assumed_cells":0}} for m,f in VARIANTS],"changes_vs_none":[{}]*10,
                  "patch_execution":{"builder_invocations":45,"ready_variants":0,"filtered_ready_variants":0,
                                     "variants_with_positive_assumed_cells":0,"accepted_filtered_patch_request_validation":"NOT_RUN"}}
        validate_p3(manifest,analysis)
        manifest["final_frozen"]={"model":"changed"}
        with self.assertRaisesRegex(ValueError,"freeze"):validate_p3(manifest,analysis)
        manifest["final_frozen"]=manifest["frozen"]
        analysis["variants"][0]["same_bytes"]=False
        with self.assertRaisesRegex(ValueError,"equality"):validate_p3(manifest,analysis)
        analysis["variants"][0]["same_bytes"]=True
        analysis["patch_execution"]["accepted_filtered_patch_request_validation"]="EXECUTED_SINGLE_FRAME_ONLY"
        with self.assertRaisesRegex(ValueError,"patch validation"):validate_p3(manifest,analysis)


def make_plots(data, destination):
    """Called only after all three completion gates; no imports during timing."""
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    patterns=("missing50","noise005","noise015","noise040")
    labels=("No noise\n50% missing","5mm noise","15mm noise","40mm noise")
    x=list(range(len(patterns)));width=.34
    destination.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.titlesize":12,
                         "axes.labelsize":10,"savefig.facecolor":"white"})
    fig,axes=plt.subplots(2,2,figsize=(12,7.4),sharey=True)
    points=[]
    for row,res in enumerate((.05,.10)):
        for col,f in enumerate(FILTERS[1:]):
            axis=axes[row,col];values=[]
            for pat in patterns:
                values.append(common(select(data["filter-common-support-results"],phase="fit",method="weighted_plane",filter=f,resolution_m=res,pattern=pat)))
            before=[float("nan") if v["reference_rmse_m"] is None else v["reference_rmse_m"]*1000 for v in values]
            after=[float("nan") if v["candidate_rmse_m"] is None else v["candidate_rmse_m"]*1000 for v in values]
            axis.bar([v-width/2 for v in x],before,width,label="Own none on shared cells",color="#677f98")
            axis.bar([v+width/2 for v in x],after,width,label="Filtered on same shared cells",color="#d1864a")
            axis.set_title(str(round(res*100))+"cm grid / "+f)
            axis.set_xticks(x,labels);axis.set_ylabel("Accepted shared-cell RMSE (mm)")
            axis.grid(axis="y",alpha=.23);axis.set_axisbelow(True)
            axis.spines["top"].set_visible(False);axis.spines["right"].set_visible(False)
            for i,v in enumerate(values):
                if v["delta"] is not None:
                    axis.text(i,max(before[i],after[i])+1.1,"Delta "+fmt(v["delta"],1000,3,True)+"mm",ha="center",va="bottom",fontsize=8)
                points.append({"resolution_m":res,"filter":f,"pattern":patterns[i],**v})
    ymax=max((max(v["candidate_rmse_m"],v["reference_rmse_m"])*1000 for v in points if v["count"]),default=1)
    for axis in axes.flat:axis.set_ylim(0,ymax+5.5)
    handles,legend=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,legend,loc="upper center",ncol=2,bbox_to_anchor=(.5,.94),frameon=False)
    fig.suptitle("Weighted plane: noise and resolution at matched missing cells",fontsize=15,y=.99)
    fig.text(.5,.018,"11 terrains and 5 seeds pooled. Each panel has its own none/filter intersection; do not compare panel pools.",ha="center",fontsize=9,color="#4b596a")
    fig.tight_layout(rect=(0,.045,1,.895))
    png=destination/"wls-common-error-by-noise.png"
    fig.savefig(png,dpi=200);plt.close(fig)
    numerical=destination/"wls-common-error-by-noise.json"
    numerical.write_text(json.dumps({"rows":points,"units":"m in JSON; mm in figure","scope":"11 terrains x 5 seeds; own method-filter/none intersections"},ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return [png,numerical]


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--main",type=Path,help="Completed full/all matrix (20,752 probes)")
    parser.add_argument("--timing",type=Path,help="Completed serial timing (405 probes, three repeats)")
    parser.add_argument("--p3",type=Path,help="Completed historical P3 results directory (45 probes, none/repeat parity)")
    parser.add_argument("--review",type=Path,help="review-validation.json: review_complete, engineering_tests, NOT_RUN")
    parser.add_argument("--artifact-root",type=Path,help="Absolute exported artifact root used by Markdown links")
    parser.add_argument("--output",type=Path,help="Explicit final repository Markdown path")
    parser.add_argument("--body-output",type=Path,help="Second identical UTF-8 Markdown path for PDF author")
    parser.add_argument("--check-inputs",action="store_true",help="Read-only final completion/identity/arithmetic audit")
    parser.add_argument("--self-test",action="store_true",help="Only tiny synthetic tests; no experiments")
    args=parser.parse_args()
    if args.self_test:
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(SelfTests))
        return 0 if result.wasSuccessful() else 1
    for key in ("main","timing","review","p3"):
        if getattr(args,key) is None:parser.error("--"+key+" is required")
    try:
        data=load(args.main.resolve(),args.timing.resolve(),args.review.resolve(),args.p3.resolve())
        if args.check_inputs:
            print(json.dumps({"complete":True,"main_runs":20752,"timing_runs":405,"p3_runs":45,"formal_total":21202,"report_written":False}))
            return 0
        for key in ("artifact_root","output","body_output"):
            if getattr(args,key) is None:parser.error("--"+key.replace("_","-")+" is required to write")
        report=Report(data,args.artifact_root);content=report.render()
        charts=make_plots(data,args.main.parent/"report-charts")
        sources=[args.main/"manifest.json",args.main/"summary.json",args.main.parent/"environment.json",
                 args.timing/"manifest.json",args.timing/"summary.json",args.review,
                 args.p3/"manifest.json",args.p3/"analysis.json",args.p3/"none-parity.json",args.p3/"repeat-artifact-equality.json"]
        sources.extend(args.main/(k+".json") for k in ("fit-results","platform-results","filter-common-support-results","all-fifteen-common-results","all-fifteen-pools","recheck-platform-results"))
        sources.extend(args.timing/(k+".json") for k in ("timing-results","timing-aggregate"))
        record={"schema":1,"complete":True,"generator":str(Path(__file__).resolve()),"generator_sha256":sha(__file__),
                "source_sha256":{str(p.resolve()):sha(p) for p in sources},
                "outputs":[str(args.output.resolve()),str(args.body_output.resolve())],
                "markdown_sha256":hashlib.sha256(content.encode("utf-8")).hexdigest(),
                "case_tie_tolerance_m":TIE_M,"statistics":report.snapshot,
                "charts":[{"path":str(p.resolve()),"sha256":sha(p)} for p in charts],
                "does_not_rerun_models":True,"does_not_generate_pdf":True}
        for path in (args.output,args.body_output):
            path.parent.mkdir(parents=True,exist_ok=True);path.write_text(content,encoding="utf-8")
        (args.main.parent/"report-generation.json").write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
        print(json.dumps({"complete":True,"outputs":record["outputs"],"lines":len(content.splitlines()),"markdown_sha256":record["markdown_sha256"]},ensure_ascii=False))
        return 0
    except (ValueError,KeyError,OSError) as error:
        print("Report generation blocked: "+str(error),file=sys.stderr)
        return 2


if __name__=="__main__":
    raise SystemExit(main())
