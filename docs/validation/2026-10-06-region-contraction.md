# 区域收缩实验记录

## 设计与来源

方案：`docs/superpowers/specs/2026-10-06-region-contraction.md`。
执行计划：`docs/superpowers/plans/2026-10-06-region-contraction.md`。

基线源码：`61de9e715b40a8d94d2dc04e29f7e56a03ef6baf`。
实验分支：`feat/drl-region-contraction-20261006`。
实验工作树：`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-region-contraction`。
产物：`/home/kai/CodexDownloads/lunar_navigation/drl-region-contraction-20261006`。
Actor SHA256：`425209da25ff20b5d1ac833ec74074d21b45279c56cc0052ae93abaa0272358a`，取自已完成300m测试的冻结版本。

## 执行裁定

- 用户已经要求“给出具体的方案，并做出实验版本，对比测试效果”，据此连续完成方案、实现与测试，不重复请求实施批准。
- 正式训练正在原工作树运行，新增独立实验工作树，不写入原训练源码；原生导航仍通过原工作树拥有的 Jazzy wrapper 加载，实验文件以独立适配器加载，不绕过缓存归属校验。
- A1 保留当前及原动作节点以保持动作一一对应；A2 排除追加当前节点的转向动作，以免绕过原到达容差/转向有效性过滤。该阶段没有修改原导航。
- A2 远程候选以能观测任务内前沿为准，允许站位处于任务区外，延续用户已批准的任务内外收益语义；不会把仅有任务外收益的站位作为追加目标。
- 首轮闭环比较使用月表50m/种子2026100501和洞穴50m/种子2026100502，均256次决策。样本少，用于机制实验；不能给出训练收益或大场景完成率结论。
- 300m诊断探针由真实原生导航重放取得；同快照基准只测压缩、传播和推理，不当作闭环覆盖率结果。

## 任务接口核对

| 任务 | 产生/消费 | 核对 |
|---|---|---|
| 收缩算法 → 评测 | `ContractionResult.observation`, `fine_to_coarse`, `virtual_mask`, `diagnostics` | 原动作逐项映射，真实目标不变 |
| 远程候选 → 评测 | `expand_frontier_actions(obs, max_positions=32)` | 保留原动作前缀，追加已知可达真实目标 |
| 评测 → 汇总 | `case/mode/summary.json`, `steps.jsonl`, 完整观察快照 | 同一Actor/场景/预算，基础设施错误分列 |
| 快照 → 基准 | 原生真实 `DecisionObservation` | 三组在同一输入状态重复交错计时 |

## 验证与结果

实现、独立审阅和冻结对比正在执行。完成后追加实测结果与证据路径。

验证边界：本机Jazzy；Humble、Orin、实车 NOT_RUN；新语义SAC重训练 NOT_RUN。
