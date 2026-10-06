# 已探索区域收缩实验 Implementation Plan

> **For agentic workers:** Use subagent-driven development with independent review. Steps use checkbox syntax.

**Goal:** 实现独立的区域收缩实验并完成冻结模型同场景对比。

**Architecture:** 原始决策图仍负责生成真实站位和收益，实验适配器生成供 Actor 使用的收缩图。远程动作组只替换真实目标列表，由既有 ROS 导航和原奖励计量执行。

**Tech Stack:** Python、NumPy、PyTorch、原生 Jazzy 导航/控制器、pytest。

**Spec:** `docs/superpowers/specs/2026-10-06-region-contraction.md`

## Global Constraints

- 保留正式训练源码和产物；实验独立工作树、产物独立目录。
- 原奖励、导航、预算、传感器保持一致。
- A1 原动作逐项对应；A2 单独标注动作机制变化。
- 虚拟区域节点不得成为真实动作目标。
- 初版不能声称细图构建和收益计算已经被加速。

## Review Focus

- 墙两侧近邻不得合并；测试无原图路径的两个分量。
- 内外收益与任务归属不得相加或混淆；测试不同归属零收益分量。
- 当前零收益节点与动作位置不得丢失；测试原目标字节相同。
- 抽象边不得虚报物理捷径；测试长链内部接口距离。
- 没有任务内前沿时远程组必须回退原动作；测试空正收益候选。

### Task 1: 收缩与远程候选

**Files:** Create `scripts/drl/region_contraction/contraction.py`; test `scripts/drl/region_contraction/test_contraction.py`.

**Interfaces:** `contract_observation(observation) -> ContractionResult`；结果包含 `observation`, `fine_to_coarse`, `virtual_mask`, `diagnostics`。
`expand_frontier_actions(observation, max_positions=32) -> DecisionObservation` 返回原图节点、原动作加已知图可达的任务内前沿真实动作。

- [x] 写断开分量、内外分量、长链、收益节点及目标保真、虚拟目标禁选、确定性、远程可达/上限/回退的失败测试。
- [x] 执行测试确认失败。
- [x] 使用连通分量、代表点和分量内 Dijkstra 实现收缩与边长度；远程动作按当前节点 Dijkstra 排序。
- [x] 执行测试确认通过。

### Task 2: 冻结对照评测

**Files:** Create `scripts/drl/region_contraction/evaluate.py`; test `scripts/drl/region_contraction/test_evaluate.py`.

**Interfaces:** 消费 Task 1 的两个接口；CLI 接受 `--mode baseline|contracted|remote`, `--config`, `--actor`, `--output`, `--family`, `--extent`, `--seed`, `--budget`；运行一回合，原生 ROS 域自动租用。

- [x] 写 A1 动作对应、A2 环境真实目标替换、终止/错误分类与配置身份记录的测试。
- [x] 执行失败测试，实现适配器；不导入实验工作树的 ROS overlay，显式经原源码所属 wrapper 使用原导航，只加载实验适配器。
- [x] 记录逐步 JSONL、汇总、完整观察快照和可视化节点轨迹；按阶段计时。
- [x] 执行测试及小回合原生烟测。

### Task 3: 审阅、同图实验与交付

**Files:** Create `scripts/drl/region_contraction/compare.py`; report `docs/validation/2026-10-06-region-contraction.md`.

- [x] 独立审阅实现和实验方法，修复实质问题。
- [x] 冻结 Actor/配置及所有源码哈希；运行相同场景、种子、预算的 A0/A1/A2 完整回合。
- [x] 汇总覆盖率、循环、计算开销及执行失败；制作节点压缩与探索轨迹可视化。
- [x] 检查 UTF-8、`git diff --check`、相关测试；明确样本数和未验证边界，交付方案、实验源码与结果。

## Completed evidence

实现提交 `787176cd`；A0/A1/A2 各2例完整终态。相关测试80 passed、1 skipped，独立审阅与来源身份核对完成。结果、时序差异及未验证边界见 `docs/validation/2026-10-06-region-contraction.md`。正式训练未替换。
