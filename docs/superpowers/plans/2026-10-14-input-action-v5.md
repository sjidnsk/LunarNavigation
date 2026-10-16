> 历史档案（2026-10-17 归并）：来源 `feat/drl-input-action-v5-20261014@e2b2b5bf9a65f47c9e3d51c1adb405bfe41da4c3`，原专项基线 `31a4eacce515d3314e99ecb82f087dddc3cf9572`。正文保留历史记录；旧 PID、路径和启动命令不代表当前运行环境，请从 [DRL 开发入口](/docs/DRL开发入口.md) 核对当前开发/部署身份。该实验已被后续版本替代的部分不作为当前协议。

# 候选几何输入与六层网络 Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. 用户已确认方案，按当前会话执行，不再次征求同一方案的批准。

**Goal:** 实现可独立选择的 task_graph_v5 / candidate_geometry_v3 输入、网络与训练保存恢复链路。

**Architecture:** 原始已知图先扩展真实远端候选并计算真实路径距离，再做 A2 收缩。真实位置与八方向视角分别编码；六层稀疏图传播；候选几何直达联合评分，按位置/状态复用评分投影。双 Q 和目标 Q 保持独立参数，训练专用真值分支不进入 Actor。

**Tech Stack:** Python、NumPy、PyTorch；已有 Humble native 环境用于必要的采样/训练接口验证。

**Spec:** `/mnt/data/WS/lunar-navigation/analysis-artifacts/input-action-architecture-20261014/design.md`、`architecture.md`、`complexity-review.md`。用户最新决议覆盖其中的容量对照建议：两类图都固定 6 层；四维动态输入直接融合；评分第一层分块投影等价复用。

## Global Constraints

- 宽度 128、8 头、6 层；不改奖励、SAC、温度、场景与传感器分布。
- 训练源、模型、回放、记录保持原位；不启动新正式训练，不做部署适配。
- v4/region_relation_v2 继续可读；v5 不接受旧 v4 回放/权重静默迁移。
- 新分支从 dev/drl-exploration 派生，既有训练改动按哈希复制为独立基线；不暂存/提交这些已有改动，不合并、不推送。
- artifact 位于工作树外；验证从工作树根用 python -m pytest；源码 UTF-8。

## Review Focus

- 旧协议和新协议混入同一回放/批次/发布，必须在修改模型或回放前拒绝。
- 非等长批次内终态没有动作、单动作和孤立当前点必须数值稳定。
- 非连通已知图不把不可达距离伪造为有限数；构造失败时给明确原因。
- 同一凹多边形的最近点并列、循环换起点、反序，几何结果一致；动作顺序和世界目标冻结。
- 含虚拟节点的大图不编码伪位置/视角、不选择虚拟节点、不产生 N×N 全图注意力。

## Task 1: 观测契约与构造

**Files:** 新建 `candidate_observation.py`；修改 `feature_schema.py`、`replay.py`；新建 `test_candidate_observation.py`。

**Interfaces:** `CandidateObservation` 冻结 N×5 几何、N×8×3 视角、8×2 方向、4 动态量、图与动作索引/掩码、完整多边形、Pose/元数据/世界目标。`prepare_candidate_observation(fine, pose, *, contract=True, max_remote_positions=32, metadata=None)` 返回带 v5 observation 的 PolicyGraphResult。无 Torch/真值依赖。

- [x] 先写反例测试：最近点、车体坐标、原图距离 2 m 与收缩 4 m、远端动作保护、掩码、immutable transport；运行确认缺少接口导致失败。
- [x] 实现几何计算与 schema；沿用已有候选扩展/A2，距离在收缩前计算，回放禁止混合协议。
- [x] 运行新测试和既有 policy_graph、replay 完整保留测试。

## Task 2: 打包与网络

**Files:** 新建 `candidate_batch.py`、`candidate_model.py`；修改 `batch.py`、`model.py`、`config.py`；新建 `test_candidate_model.py`。

**Interfaces:** pack_observations 按 schema 派发且拒绝混合；Actor/Critic 按 architecture_version 派发。v5 打包无重复 action_features；truth XY 与支持关系采用冻结 Pose 同一坐标系。评分 head 第一 Linear 按输入列投影后索引复用，GELU 仅在求和之后。

- [x] 先写节点/动作排列、方向收益、虚拟掩码、polygon 反序、头输出/梯度等价、独立参数、空动作测试并确认失败。
- [x] 实现编码器、六层图、对称 polygon 池化、动态量直接融合、独立 Q 及评分投影复用。
- [x] 运行新旧网络测试，确认旧模型接口和参数身份不变。

## Task 3: 训练链路与验收

**Files:** 修改 `ros_env.py`、`sac.py`、`runtime.py`（仅 Actor 发布加载）、`config.py`、必要的训练诊断消费者；新建 `test_candidate_training.py`、设计/验证文档。

**Interfaces:** training config 显式选择 candidate_geometry_v3，resume_semantics 区分 v4/v5；RosExplorationEnv 从原生 fine observation 构造 v5，truth 仍查询完全相同的真实世界目标；Actor 发布支持 v5，部署输入构造不改。

- [x] 先写真实环境构造适配、SAC 更新、CPU 发布、完整 checkpoint 保存恢复、拒绝旧回放/权重测试并确认失败。
- [x] 接入训练接口，保留全量回放保存/恢复断言；不自动兼容旧经验。
- [x] 运行受影响包测试；在 RTX 4080 SUPER 上做有界前向/更新烟测及真实规模压力核查，输出到独立 artifact。
- [x] 更新最终架构图、记录所有验证边界；独立审阅本次增量并处理重要问题。

## 交付

保留新工作树及本次增量补丁（相对已记录的训练源码基线）；不提交复制的已有改动。正式新训练仍需独立随机初始化模型与输出目录，不能从旧 A2 回放恢复。性能烟测和一次更新只能验证工程链路，不能证明已学会入区。
