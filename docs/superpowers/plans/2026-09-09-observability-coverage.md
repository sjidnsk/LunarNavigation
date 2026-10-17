> 历史沿革补记（2026-10-17）：`fix/observability-coverage` @ `a73d561903d145c08769b639076d0e0b3e175481` 曾在独立工作树实施早期 C/O/K 在线估计并记录本机 Jazzy 构建与验证，随后同一来源切换为高程可见性与固定 `CoverageReference` 方案。下文“尚未实现”及原任务状态描述的是 `c6473ae70eb4` 当时阶段；原正文保留用于追溯，不作为当前待执行计划。
>
> 当前开发与运行身份以 [统一 DRL 开发入口](../../DRL开发入口.md) 为准；现行入口为 `lunar_drl_exploration/reference.py` 的 `CoverageReference.build` 与 `coverage.py` 的 `CoverageHistory`。a73 中的 `MODEL_VERSION = 4` 仅为该历史实现版本，不代表当前运行版本。本轮仅补充来源沿革，未重跑历史实验、切换训练或新增 Jazzy、Humble/Orin、实车及策略效果验收。

> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../../DRL开发入口.md) 为准。

# Observability Coverage Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task by task in the current session. Do not dispatch subagents by default. Steps use checkbox syntax. No commit, merge, push or training switch is implied by this document.

**Goal:** Replace inconsistent exclusion/exhaustion estimates with one observable-set calculation, and independently extend the existing pose candidate set to distant known-space representatives without changing the pose action contract.

**Architecture:** Existing map owner exports aligned C/O/K evidence. A pure CoverageEvaluator owns all estimated area and completion calculations, with shared visibility queries. Actor always chooses one target pose (x, y, yaw); the existing navigator and controller execute near and distant targets identically.

**Tech Stack:** Existing C++ ROS 2 core/adapter, Python/NumPy/SciPy/PyTorch, setuptools C++ extension using the Python buffer protocol. No new simulation platform, ROS service or third-party geometry framework.

**Spec:** [2026-09-09-observability-coverage-design.md](../specs/2026-09-09-observability-coverage-design.md)

## Global constraints

- 8 个环境；0.2 m/s；目标 10 倍仿真；传感 10 m/90° 可配置。
- 64 m 局部窗口、0.2 m 细格；任务外允许绕行，面积只统计任务内。
- 实际任务无时间上限；训练/自动验证预算 256/1024，高层动作预算仅产生 truncated。
- 沿用 1800 s 保存和现有有界存储；不复制模型或回放作验证资料。
- 当前运行训练源码只读，运行进程不打断。实施使用独立 worktree 和构建目录。
- 模型输入不得接收未观测遮挡真值或参考分母。缺少可选 K 层允许运行。
- 所有面积与终止判断只从同一 CoverageSnapshot 获取；不保留旧排除/耗尽算法并行运行。
- 覆盖修正与候选范围扩展分开验证；目标始终为现有 Pose(x,y,yaw)，动作特征保持 12 维，不新增无朝向动作类型。
- 不增加覆盖率门槛、连续成功门槛、候选审批和第二个控制器；沿用已有接口/运动约束。
- Jazzy、Humble、Orin/实车证据分别记录；未跑层级写 NOT_RUN。

## 工作树与文档准备

- [ ] 运行 `git branch --show-current`、`git status --short`、`git worktree list`。当前训练来源是已有 `feat/drl-exploration-training` 的未提交实现，不能当作干净 integration HEAD。
- [ ] 从 integration 创建独立 `fix/observability-coverage` worktree；按明确文件清单复制本训练特性所需的现有源码/配置/测试/文档快照，记录来源 HEAD 与内容清单。不得 stash、reset、移动原文件或复制 training-output/build/install/log。此步骤是隔离已有运行源码，不是合并。
- [ ] 阅读 spec 和两份排查记录；在实施树生成 `docs/validation/2026-09-09-observability-coverage.md`，按实际执行填入证据。当前设计文件留在训练特性工作树中供审阅。

## Task 1: 观测包与同源 C/O/K 地图接口

**Files:**
- Modify: `ros2_ws/src/lunar_planning_msgs/msg/PolicyMapTile.msg`
- Modify: `ros2_ws/src/lunar_incremental_navigation_core/include/lunar_incremental_navigation_core/elevation_map.hpp`
- Modify: `ros2_ws/src/lunar_incremental_navigation_core/src/map/persistent_elevation_map.cpp`
- Modify: `ros2_ws/src/lunar_incremental_navigation_ros/{include/lunar_incremental_navigation_ros/map_adapters.hpp,src/map_adapters.cpp,src/elevation_pipeline.cpp,src/policy_map_exporter.cpp}`
- Modify: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/{ros_messages.py,ros_env.py,contracts.py,maps.py,observation_graph.py}`
- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/sensor_simulation.py`
- Test: existing `persistent_elevation_map_test.cpp`, `map_adapters_test.cpp`, `policy_map_exporter_test.cpp`, `tests/drl/test_scene_visibility.py`, `tests/drl/test_maps.py`; create `tests/drl/test_ros_messages.py` for the optional-layer round trip.

**Interfaces:**

```python
@dataclass(frozen=True)
class PolicyTile:
    states: np.ndarray       # uint8: UNKNOWN/FREE/BLOCKED
    observed: np.ndarray     # uint8, valid elevation evidence
    costs: np.ndarray        # float32
    occlusion: np.ndarray    # uint8: UNKNOWN=0, CLEAR=1, OPAQUE=2

@dataclass(frozen=True)
class ObservationPatch:
    origin: tuple[float, float]
    elevation: np.ndarray
    occlusion: np.ndarray   # float32: NaN / 0 clear / 1 opaque

# elevation_message retains optional argument for existing callers.
def elevation_message(origin, elevation, resolution, ns, occlusion=None): ...
```

`ElevationEvidence` / `OwnedElevationEvidence` 增加可选遮挡字段，`ElevationSnapshot::OcclusionAt(GridIndex)` 返回 uint8。有效 K-only 改变也进入现有证据 revision、changed_cells 和导出 tile 增量；不新增独立 revision 握手。

- [ ] 先写遮挡后格不输出证据、首障碍输出 O/K、缺层继续接收、CLEAR 撤回 OPAQUE、相同高程但 K 改变仍有增量的失败用例。
- [ ] 修改模拟观测适配，仅在 `isfinite(elevation)` 的当前有效观测位置填 K，其他位置 NaN。
- [ ] 添加可选层解码与同一世界投影，持久化并导出；不根据 C 推导 K，不改变导航判定规则。
- [ ] 将 PolicyMapStore 的 tile 访问改成命名字段，修改受影响访问处并运行对应测试。

关键不变量：

```python
assert np.isnan(patch.occlusion[~np.isfinite(patch.elevation)]).all()
assert exporter_snapshot_without_layer.occlusion_is_unknown
assert k_only_update.fine_revision > previous.fine_revision
assert np.array_equal(k_only_update.states, previous.states)  # 对固定同高程案例
```

## Task 2: 一个可见性内核

**Files:**
- Create: `ros2_ws/src/lunar_drl_exploration/native/visibility_grid.cpp`
- Modify: package `setup.py`, `package.xml`, `lunar_drl_exploration/visibility.py`
- Test: create `tests/drl/test_visibility_queries.py`; retain scalar reference in tests.

**Interfaces:**

```python
# Native top-level module name avoids source-PYTHONPATH shadowing the installed extension.
import lunar_observability_native

# All coordinates/range use the declared raster origin/resolution.
def visible_mask(occlusion, origin, resolution, pose, range_m, fov_deg): ...
def any_visible_from_region(target_mask, source_mask, occlusion,
                            resolution, range_m): ...
```

- [ ] 用纯 Python DDA 构造对照：首障碍可见、后格不可见、双侧穿角、负坐标、离散中心反向可见性、90° 实际视角及全朝向并集。
- [ ] 实现现有 supercover 语义的 C++ 批量查询，UNKNOWN/CLEAR 均不截断，OPAQUE 首命中后停止。区域查询按预计算距离偏移枚举所有可能来源，找到一个即返回；不以稀疏角度漏采作为排除理由。
- [ ] 模拟观测和后续候选收益调用同一接口；C++ 循环释放 GIL。只在数组入口校验 shape/dtype，避免逐射线创建 Python 对象。
- [ ] 原生扩展与标量参考逐案例相等；保留真实连续位姿的视线测试，不把仿真位置量化到格中心。

## Task 3: 单一 CoverageEvaluator

**Files:**
- Modify: `lunar_drl_exploration/coverage.py`（保留 polygon/ledger）
- Create: `lunar_drl_exploration/coverage_evaluator.py`
- Delete after call-site migration: `lunar_drl_exploration/exhaustion.py`
- Test: create `tests/drl/test_coverage_evaluator.py`; replace obsolete exclusion/exhaustion expectations in existing tests.

**Interfaces:**

```python
@dataclass(frozen=True)
class CoverageSnapshot:
    epoch: int
    revision: int
    task_area: float
    observed_area: float
    explorable_area: float
    excluded_area: float
    remaining_area: float
    raw_ratio: float
    effective_ratio: float
    exhausted: bool
    metric_version: str

class CoverageLedger:
    # Existing fixed-world observed bookkeeping; task polygon/resolution are immutable.
    def update_cells(self, indices, observed, blocked): ...

class CoverageEvaluator:
    def __init__(self, polygon, resolution_m, sensor): ...
    def evaluate(self, view, ledger, pose, start_connectors) -> CoverageSnapshot: ...
    def region_remaining_area(self, bounds) -> float: ...
```

`view` 是 PolicyMapStore 的冻结版本视图，含 meta 与命名 tile。E 位图由评估器保存；不把完整 E 加入每条 replay。

- [ ] 写失败用例：0.2 m 薄环、机器人在墙内/外、单格部分观测、开放洞口、任务外绕行、C 阻塞但 K 可透视、K 缺失、K 撤回、首次观测和重复观测。
- [ ] 实现四邻接 Q、正确的虚拟外部连接、光学 L、距离筛选及 D 中完整可见性查询。只有边框分量经虚拟外部连接，内部空洞不能直接接外部。
- [ ] 从 E/O 统一计算所有面积与 exhausted；不继续接受任意 excluded_area 标量裁剪作“修复”。
- [ ] 根据 C/K/任务/传感/平台分量缓存 E。只变 O 时增量统计；分量或 K 撤回时重算。
- [ ] 复用标签/距离工作数组，不同时存 E/X/R 三份稠密图。

测试中的面积恒等式：

```python
assert report.explorable_area == report.observed_area + report.remaining_area
assert report.task_area == report.explorable_area + report.excluded_area
assert report.observed_area <= report.explorable_area <= report.task_area
assert report.exhausted == (report.remaining_area == 0)
```

## Task 4: 观测构造与统一收益语义

**Files:**
- Create: `lunar_drl_exploration/observation_builder.py`
- Modify: `maps.py`, `observation_graph.py`, `contracts.py`, `model.py`, `runtime.py`, `replay.py`, `config.py`, `config/drl_exploration.yaml`
- Test: `test_observation_graph.py`, `test_model_sac.py`, `test_replay.py`, `test_maps.py`.

**Interfaces:**

```python
class ObservationBuilder:
    def build(self, view, coverage_snapshot, coverage_evaluator,
              pose, previous_action, execution, failed, sensor): ...
# Returns existing MapObservation, with 10 local channels.
```

- [ ] 将 build_observation 移出 PolicyMapStore；调用方传入同一决策快照，构建器不自行计算排除或结束。
- [ ] 本地通道为原 8 个 + occlusion_known + opaque；向量仍 16，执行量 2，节点 158，边 2。
- [ ] `ObservationGraph._directional` 调用 VisibilityQuery，不再用 C==BLOCKED 作射线遮挡。区域剩余使用 E\O，实际收益仍为 T\O 的可见面积。
- [ ] 添加“只改 C 不应改变光学射线、只改 K 应改变射线”的测试；对全局 E 和单个位姿收益的统计粒度分别测试。
- [ ] 模型输入维度、批处理、replay 序列与 manifest 同步。MODEL_VERSION=3、metric_version=observability_grid_v2，graph_schema 保持 [158,12,2]。版本变化来自新增遮挡输入、特征预处理及指标口径；不对运行中训练热更新。

## Task 5: 扩展远处目标候选，保持同一位姿动作

**Files:**
- Modify: `observation_graph.py`, `contracts.py`, `ros_env.py`, `runtime.py`, `evaluation.py`, `model.py`
- Test: `test_graph_execution.py`, `test_observation_graph.py`, `test_runtime.py`, `test_graph_evaluation.py`, `test_ros_closed_loop.py`.

**Interfaces:**

```python
# Existing interfaces remain unchanged.
goal: Pose = snapshot.goal(action_index)  # x, y, yaw
assert snapshot.action_poses.shape[1] == 3
assert snapshot.action_features.shape[1] == 12
# Both near and distant targets use has_target_yaw=True.
```

- [ ] Task 3/4 的覆盖结果可在原候选集合下独立验证；本任务不作为面积正确性的前提。记录局部动作本可逐段走远，此项评价的是长距离转移的表示与学习效率。
- [ ] 增加长已知通道案例：局部目标都零增益，远处已知区域代表点必须始终在动作域；不依赖历史零收益计数。
- [ ] 图中与平台连通的 type=0/type=1 节点均展开现有 12 个朝向；10 m 仅用于局部细节保留，不再裁掉较远目标。排除纯任务 token/type=2 与临时非 FREE 起点/type=3，沿用当前 FREE/图连通含义。
- [ ] 动作距离读取到实际代表点的距离；不得用区域最小距离代替。
- [ ] type=1 代表点也计算自身方向摘要，每个动作仍描述指定朝向下的实际视野；不使用全朝向收益代替某个朝向的收益。
- [ ] 保持 GraphSnapshot.goal 返回 Pose、ACTION_FEATURE_DIM=12、has_target_yaw=true 以及 previous_action 的目标位姿含义。验证远处目标完成终点朝向；不增加 NavigationGoal/action_has_yaw。
- [ ] 网络预处理统一对真实距离/相对位移使用固定 asinh 缩放，保留不可达 sentinel 的语义；用于导航、几何和 baseline 的原始数值保持米和 m²。
- [ ] greedy/random 评估也使用同一位姿集合；预测观察机会不作为已收到奖励。测量远处节点乘以 12 个朝向后的候选数、推理时间和有界回放容量。

## Task 6: 统一环境与推理的决策边界

**Files:**
- Modify: `ros_env.py`, `runtime.py`, `collector.py`, `training.py`, `evaluation.py`, `progress.py`, `checkpoint.py`
- Test: `test_training.py`, `test_training_control.py`, `test_ros_env.py`, `test_runtime.py`, `test_checkpoint.py`, `test_ros_closed_loop.py`.

**Interfaces:**

```python
# After real navigation terminal result, controller settling and final map processing:
report = evaluator.evaluate(view, ledger, pose, view.start_connectors(pose))
success = report.effective_ratio >= coverage_target and not collision
reward = compose_reward(reward_components(delta_observed_area, delta_distance,
                       delta_rotation, invalid, collision, success), weights)
terminated = success or collision
# Collector alone appends the 256/1024 decision truncation.
```

- [ ] 删除导航过程中以旧 excluded_area 取消任务的分支；沿途 O 和距离继续累计。
- [ ] 环境和推理使用同一报告和结束口径；删除旧 estimate_exclusion/check_exhaustion 调用与模块，不新增影子结束条件。
- [ ] 验证全局导航完整沿途面积/路程入一条转移，局部收益为零不截断；保留原折扣/权重及预算自举语义。
- [ ] 记录 metric_version、局部/全局动作次数、覆盖计算耗时与面积字段到现有有界 metrics，不增加文件种类。
- [ ] 版本不匹配只在已有 checkpoint/model 边界处理。新增遮挡输入及改变后的覆盖终止口径不能直接当作旧 resume 接续；不能把“候选数量变多”单独视为清空经验或重建网络的充分理由。本任务不执行训练切换。
- [ ] 显式取消测试按已有导航生命周期运行。若复现历史 cancel 超时，保留证据并在现有关闭路径定位原因，不用覆盖算法吞掉该异常。

## Task 7: 实际验证、文档与交付

**Files:**
- Modify: `README.md`, `docs/操作指令.md`, original graph design/complete design references
- Create/update: `docs/validation/2026-09-09-observability-coverage.md`

- [ ] 从实施工作树根目录运行纯 Python 受影响测试，随后运行完整 `tests/drl`。纯 Python 测试禁用无关 pytest 自动插件，避免 launch 插件的环境依赖混入结果。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/drl -q
```

- [ ] 在独立 Jazzy build/install/log 下构建受影响消息、地图核心、ROS 层和 DRL 包；先 source ROS 再 source 对应 overlay。示例中 `$OBS_BUILD_ROOT` 由操作者设置为仓库外专用目录。

```bash
colcon --log-base "$OBS_BUILD_ROOT/log" build \
  --base-paths ros2_ws/src --build-base "$OBS_BUILD_ROOT/build" \
  --install-base "$OBS_BUILD_ROOT/install" \
  --packages-up-to lunar_drl_exploration
```

- [ ] 按包串行运行 CTest，尤其含 750 m 滚动案例的包。实际 ROS 检查使用与长训 80–87 不同的 DDS domain，最多先增加一个仿真闭环测小洞穴。
- [ ] 小场景用可复现的覆盖序列检查面积闭合及目标结束；用真实导航/控制检查局部观测、长已知通道转移、预算截断和正常收尾。新网络的收敛不是启动工程验证的门槛。
- [ ] 测量 100/300/1000 m 的实际完整覆盖计算耗时与峰值内存，明确区分本设计的 153 ms“仅连通标记”数据。8 环境验证记录吞吐和内存，不预先声称已满足未测性能。
- [ ] 更新旧文档指向新定义，保留旧验证记录为历史；操作文档说明 K 缺失行为、estimated 指标、统一位姿动作下的候选扩展，以及完整新输入/指标版本不能直接当作旧 resume 接续。
- [ ] UTF-8 显式读回与 `git diff --check`；汇报实际改动、检查结果和未运行平台。
- [ ] 交付可审阅实现和验证记录，保持旧训练运行。后续获准切换时以新 schema 新模型/新回放启动，保存周期和输出目录规则保持不变；切换前不清理现有运行产物。
