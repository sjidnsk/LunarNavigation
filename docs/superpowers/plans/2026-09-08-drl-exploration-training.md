> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../../DRL开发入口.md) 为准。

# DRL Exploration Training Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现可运行的连续位姿探索训练、评估与推理系统，复用现有增量地图、导航器、规划器和轮式控制器。

**Architecture:** 新建独立 Python ROS 包负责场景、观测、训练环境、地图编码和 Recurrent SAC。导航进程增加默认关闭的只读策略地图服务，观测证据来自同一 fine snapshot 的原始有效高程。每个仿真实例使用独立 DDS domain 和会话命名空间，reset 销毁并重建本实例的导航/控制进程，学习器与限量回放由父进程共享。

**Tech Stack:** C++20、ROS 2 Jazzy 本机验证 / Humble 目标、Python 3.10+、NumPy、SciPy、PyTorch 2.8.0（本机 CUDA 12.8）、Gymnasium 1.2.0、pytest/ament CTest。

**Spec:** [完整方案](../specs/2026-09-08-drl-exploration-complete-design.md)，来源文档提交 a0f190c；源码基线 df469d1。

## Global Constraints

- 正式训练固定 8 个独立仿真实例，共享一个 GPU 学习器；单实例只用于正确性验证。
- 任务典型宽度 100～300 m，最大约 1 km；活动范围包含任务外绕行，覆盖只计任务内。
- 首版传感器有效距离 10 m、水平视角 90°，配置可调，沿实际运动持续观测并处理遮挡。
- 策略输出连续 (x, y, yaw)；目标交给现有 NavigateToPose、规划器和控制器，不吸附前沿、不瞬移。
- 策略只消费唯一地图所有者导出的派生地图；不向导航器或策略泄露完整场景真值。
- Observed 来自有效高程证据，包含真实观测的障碍格，不从 FREE/BLOCKED 或 footprint 膨胀推断。
- 局部窗口继承 navigation.local_window_size_m=64.0 和 BuildLocalPlanningWindow；0.2 m 时 320×320。
- 覆盖优先、实际路程次之；无任务时间预算；正常训练截断保留 bootstrap。
- batch=8 个序列，burn-in=8、unroll=16；CPU intra-op=8、inter-op=1。
- 回放目标 8 GiB、上限 12 GiB，进程组主存目标 20 GiB 且保留至少 4 GiB 可用主存；显存目标 10 GiB。
- 项目内 training-output/drl-exploration/，总产物及写入峰值不超过 20 GiB。
- 每 1800 s 单调墙钟保存；正常退出/人工中断额外保存；仅 resume.pt、best.pt、限长 metrics.jsonl。
- 检查点限量回放最多 2 GiB；恢复学习器、优化器、随机状态和经验，再开启新 episode。
- 不下载场景资产，不生成 bag、视频、逐步地图、TensorBoard 文件或多版本模型。
- 模拟控制输出只能指向实例隔离话题；不得向 /Car/T5/Car_Cmd_Vel 发布控制。
- 仅在 feat/drl-exploration-training 工作树实现，显式路径提交，不合并、不推送、不删除其他工作树。
- README.md、docs/操作指令.md、config/incremental_navigation_interfaces.yaml 和 validation 文档反映实际实现与证据。
- 本机、Humble、Orin、跨机 DDS、实车和训练性能证据分别记录；未运行项为 NOT_RUN。

## 执行与验收说明

本计划交付训练系统及可重复的验证命令。单元测试和短程学习验证不能证明策略已经收敛或优于前沿基线；
正式多种子训练、同口径覆盖/路程比较由实现的训练与评估入口执行，并独立报告实测结果。
长任务的训练采样上限可配置；不能用采样截断冒充覆盖达标或探索耗尽。
reset 采用隔离进程重建，开销计入吞吐，不重构现有导航生命周期。

## 文件和公共数据接口

新包根目录 P = ros2_ws/src/lunar_drl_exploration，Python 模块目录 M = P/lunar_drl_exploration。
所有公共数组以世界格键或显式原点解释，不随当前地图范围改变坐标。

```python
# M/contracts.py
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

@dataclass(frozen=True)
class MapObservation:
    local: np.ndarray       # float32 [8,H,W], H=W=320 for default model
    tiles: tuple            # immutable float32 [8,32,32] arrays; reuse unchanged arrays
    tile_xy: np.ndarray     # float32 [N,2], positions normalized by fixed task extent
    vector: np.ndarray      # float32 [16]
    epoch: int
    revision: int

@dataclass(frozen=True)
class Transition:
    observation: MapObservation
    action: np.ndarray      # normalized tanh sample [3], never a repaired action
    reward: float
    next_observation: MapObservation
    terminated: bool
    truncated: bool
    episode_id: int
    env_id: int
```

8 个地图通道依次为 FREE、BLOCKED、UNKNOWN、Observed、归一化通行代价、TaskMask、Visited、Valid。
16 个向量通道依次为任务参考系 x/y、sin(yaw)/cos(yaw)、C_task/C_eff/A_X占比、R/L_T、FOV/π、
前次归一化动作 3 维、前次目标失败标记、局部窗口原点相对平台 x/y（除以窗口边长）、剩余面积/A_T。
世界格分辨率来自导航 fine snapshot；默认场景发布 0.2 m 高程。

### Task 1: 场景、观测、覆盖和公共契约

**Files:** Create M/contracts.py、M/config.py、M/scene.py、M/visibility.py、M/coverage.py、M/__init__.py；Create tests/drl/conftest.py、test_scene_visibility.py、test_coverage.py。

**Interfaces:**
- Scene(seed: int, family: str, extent_m: float, resolution_m: float=0.2) 暴露 task_polygon [N,2]、bounds=(xmin,ymin,xmax,ymax)、start: Pose、elevation(x,y)、opaque(x,y)、collides(pose, half_length, half_width)。数组查询支持 NumPy 广播，按几何基元采样而非预建公里级细稠密图。
- observe(scene, pose, range_m=10.0, fov_deg=90.0, resolution_m=0.2) 返回 (origin_xy, elevation[H,W])，origin 是数组左下格边界；不可见格 NaN，首个遮挡格有效，使用保守格遍历防漏射。
- CoverageMonitor(polygon, resolution_m)；update_cells(indices[N,2], observed[N], blocked[N]) -> 新增面积；report(excluded_area_m2=0.0) 返回 task_area、observed_area、excluded_area、explorable_area、remaining_area、raw_ratio、effective_ratio；排除从未观测中扣减且可撤销。
- config.load_config(path, project_root) 返回校验过的字典，局部窗口读取 config/exploration_navigation.yaml，平台字段读取 config/wheel.yaml；创建配置时使用 Task 6 确定的集中默认值。

- [x] 写以下行为测试并确认缺少实现时失败：

```python
def test_obstacle_counts_but_hidden_cells_do_not():
    monitor = CoverageMonitor([(0,0),(3,0),(3,1),(0,1)], 1.0)
    assert monitor.update_cells([[0,0],[1,0],[2,0]], [True,True,False], [False,True,False]) == 2.0
    assert monitor.report()['observed_area'] == 2.0
    assert monitor.update_cells([[0,0],[1,0]], [True,True], [False,True]) == 0.0
```

- [x] 实现带种子的月表几何、单层洞穴拓扑和任务外空间；去掉演示固定十字/回环捷径。
- [x] 实现 fan 遮挡、首次障碍有效、重复不增益、任务外不计数、格中心多边形口径；多边形校验拒绝自交/非有限值。
- [x] 增加种子重建、旋转视野、窄遮挡、封闭区、排除撤销、负世界格键和 1 km 按需查询测试。
- [x] 运行 `python3 -m pytest -q tests/drl/test_scene_visibility.py tests/drl/test_coverage.py`，显式提交本任务文件。

### Task 2: 导航器的只读派生地图服务

**Files:** Create lunar_planning_msgs/msg/PolicyMapTile.msg、srv/GetPolicyMap.srv；Modify 该包 CMakeLists.txt；Create lunar_incremental_navigation_ros/include/.../policy_map_exporter.hpp、src/policy_map_exporter.cpp、test/policy_map_exporter_test.cpp；Modify incremental_navigation_node.cpp、CMakeLists.txt。

**Interfaces:**
- 请求 since_revision、minimum_map_stamp_ns；响应 ready、reason、map_epoch、processed_map_stamp_ns、raw_revision、fine_revision、full_snapshot、frame_id、profile_hash、resolution_m、origin_x_m/y_m、min_x/y、max_x/y、local_min_x/y、local_width/height、PolicyMapTile[] tiles。
- Tile 含 tile_x/y、side、states(uint8[])、observed(uint8[])、costs(float32[])；格序 row-major；状态 0/1/2 与 FineCellState 对应。
- 服务参数 enable_policy_map_export=false、policy_map_service、policy_map_epoch；关闭时不增加大图序列化工作。
- Exporter 只读 CaptureBundle 的 fine/elevation；每个 tile 追踪更新版本，增量响应覆盖 since_revision 后的所有变更；首次或无法建立连续版本关系时发完整快照。

- [x] 写 C++ 测试：膨胀 BLOCKED 且无 elevation 的格 observed=0；有效高度但 UNKNOWN 的格 observed=1；重复输入可确认且不要求 fine revision 增长。
- [x] 使用 BuildLocalPlanningWindow 得到实际窗口几何；返回边界裁剪后的原点/尺寸，不伪造固定居中窗口。
- [x] OnLocalMap 记录成功或重复处理的 GridMap.header.stamp；服务只有该输入完成且 fine.raw_revision 一致时 ready。拒绝输入不作为完成确认。
- [x] 测试完整/增量地图、跨过多个更新的读取、负 tile 键、窗口边缘、错误 revision 和 epoch。
- [x] 构建 messages、core、ROS 包并运行 exporter 与现有 elevation_pipeline 测试。正式控制接口默认行为保持原样，显式提交本任务文件。

### Task 3: 真实导航闭环的仿真环境

**Files:** Create M/maps.py、M/ros_env.py、M/worker.py、M/plant.py、M/ros_messages.py、M/processes.py、test_ros_env.py、test_maps.py；Create P/package.xml、setup.py、setup.cfg、resource/lunar_drl_exploration。

**Interfaces:**
- PolicyMapStore.apply(response) 维护只读派生 tile 副本；build_observation(pose, task, coverage, previous_action, failed, sensor) -> MapObservation。
- RosExplorationEnv(config, env_id) 实现 reset(seed) -> (MapObservation, info)、step(action[3]) -> (MapObservation,reward,terminated,truncated,info)、close()。
- 每实例命名空间 `/lunar_training/env_<id>/episode_<id>`，独立 ROS_DOMAIN_ID；父进程通过 multiprocessing spawn + Pipe 与 worker 收发，worker 内才导入/初始化 rclpy。
- worker 命令 reset/step/close；响应只包含公共契约和简洁 info；ROS 故障以明确异常返回，不当作策略失败经验。

- [x] 测试地图世界键去重、增量合并、未知任务 tile 保留、局部导航裁剪补齐、历史 MapObservation 不随未来地图更新改变。
- [x] 启动本实例 navigator 和现有 wheeled_controller，参数重映射地图、odom、TF、Path、command、Action 和 policy service；所有子进程关闭 ROS 文件日志。
- [x] 发布有效 elevation GridMap、位姿、map→odom TF；使用现有控制器 Twist 推进 SE(2) 运动，碰撞使用场景几何，限制速度/加速度来自平台配置。
- [x] 按方案 4.4 重放经运动能力和碰撞检查的合法起步扫描历史，使用同一扇形模型；初始观测 O0 不奖励，不能预先绘制已观测圆盘。每次 step 记录真实目标执行、沿途新增面积、完整里程和导航结果。
- [x] step 等待处理确认服务，保留拒绝/失败动作的实际收益；同位置 yaw 由控制闭环完成。reset 停止旧子进程，创建新 epoch 与话题空间，拒绝旧响应。
- [x] 单实例运行真实导航移动、转向、不可达、reset 测试；正常功能测试不连接车辆话题。完成后显式提交。

### Task 4: 多尺度循环网络、序列回放和 SAC

**Files:** Create M/model.py、M/replay.py、M/sac.py；Create tests/drl/test_model_sac.py、test_replay.py。

**Interfaces:**
- RecurrentSAC(config, device='cpu')：act(observation, hidden=None, deterministic=False) -> (action[3], hidden)；update(sequences, burn_in=8) -> metrics；state_dict()/load_state_dict(state)。
- SequenceReplay(max_bytes)：append(Transition)、sample(batch_size,burn_in,unroll,rng) -> list[list[Transition]]、snapshot(max_bytes) -> self-contained state、restore(state)；足够数据判定 can_sample。
- Actor、两套 Critic 及 target Critic 使用一致通道/位置编码，各自循环状态；位置分支为连续 tanh 高斯，log probability 包括变换雅可比。

- [x] 写测试验证连续有界动作、不同 tile 数量 mask、未知区域保留和序列不跨 env/episode。
- [x] 实现共享 tile CNN（保留块内位置 token）、局部 CNN、32 个 latent query 的 cross-attention、16 维向量编码、256 维 GRU；token_dim=128。
- [x] 实现循环 burn-in、unroll 和 next observation 的时序 target，终止不 bootstrap、正常截断 bootstrap；Actor/α/Q 优化器、梯度裁剪、target Polyak 更新。

```python
target = reward + discount * (1.0 - terminated) * (target_q - alpha * next_log_prob)
# truncated 不参与上述乘数；学习用的是截断前的 next_observation。
```

- [x] 回放以实际数组/依赖字节预算限量；历史 immutable tile 共享计数且可解析；不足一条完整训练序列时报错而不伪造数据。
- [x] 行为测试证明至少一次真实 update 改变网络与优化器状态、loss/梯度有限、序列可恢复、未来地图不泄漏。
- [x] 运行 CPU 测试和 GPU 前向/反向短测，不下载预训练模型；显式提交。

### Task 5: 中断续训与限量存储

**Files:** Create M/checkpoint.py、M/resources.py；Create tests/drl/test_checkpoint.py、test_resources.py。

**Interfaces:**
- CheckpointStore(output_dir, total_max_bytes, free_reserve_bytes, metrics_max_bytes)：save(payload,best=None)、load()、append_metrics(record)、close()；项目内固定名字，独占写入锁。
- payload 包含 RecurrentSAC.state_dict、replay.snapshot、配置指纹、Python/NumPy/Torch/CUDA RNG、训练/更新计数、课程、场景种子生成器进度、验证列表与选模评分。
- ResourceBudget 检查父/子进程总 RSS、设备显存、回放和保存暂存；不更改环境数量。

- [x] 写原子替换中断、空间不足保留旧检查点、载荷损坏、配置不兼容、缺少优化器、回放依赖不全的测试。
- [x] 同目录受限临时写入，flush/fsync、校验、os.replace、同步目录；完整载荷包含校验信息，不保留多份历史检查点。
- [x] 单调墙钟 1800 s 调度；无更新不重复保存；更好 Actor 先保留 CPU 快照，到保存事件才更新 best.pt。
- [x] SIGINT/SIGTERM 请求停止采集并取消导航，保存最后完整转移；恢复新 episode 和采集器 GRU。测试计时注入而不等待 30 分钟。
- [x] 仅单个 metrics.jsonl 限长压缩；覆盖多次保存、并发拒绝、总临时峰值、原种子恢复和新种子重新初始化。
- [x] 运行 checkpoint/resource 测试，显式提交。

### Task 6: 八环境训练、评估和推理入口

**Files:** Create M/training.py、M/evaluation.py、M/runtime.py、M/cli.py、M/progress.py、M/frontier.py、M/exhaustion.py、M/reference.py；Create config/drl_exploration.yaml、scripts/drl/train.sh、evaluate.sh、infer.sh、requirements.txt；Create tests/drl/test_training.py、test_cli.py。

**Interfaces:**
- CLI `train --config ... [--resume] [--max-transitions N]`；默认运行至用户中断，max-transitions 仅为可复现短程验证。
- CLI `evaluate --config ... --model ... --split validation|test --policy learned|frontier|random`，同一导航与观测闭环；结果写限长 metrics。
- CLI `infer --config ... --model ...` 读取派生地图服务、任务角点消息和位姿，发送连续 NavigateToPose，默认仅实例隔离话题，不自动启动实车控制器。
- 8 个 worker 独立步进；父进程批量推理、采样和优化。每 worker 独立 hidden、episode、随机流，所有 episode 的原始与有效覆盖都记录。

- [x] 配置包含完整方案第 14 节的固定值；训练入口拒绝 env_count≠8，验证专用命令才允许单环境。
- [x] 增加按场景族/尺度隔离的训练、验证、测试 seed 列表；典型范围 100～300 m、课程上限 1000 m；不复制地图资产。
- [x] 实现现有前沿探索器示范采集接口和可选初始化，示范直接进入同一限量回放，不输出离线数据集；未启用示范时明确记录。
- [x] 覆盖权重 100；路程权重从 0 逐步到 0.1；无效目标成本 0.05、碰撞 5；仅实际新增观察进入面积奖励，排除量不产生奖励。
- [x] 独立保守检查探索耗尽；若缺乏可靠证据则记录 STALLED/截断，不能把反复失败判作成功。
- [x] 实现训练轮询、学习更新、候选 best 缓存、保存、关闭与恢复；normal exit 只纳入完整转移，取消所有活动模拟目标。
- [x] 测试 8 worker 隔离、终止/截断、资源过载、种子恢复、命令参数/输出目录/日志边界；显式提交。
- [x] 按用户追加要求加入终端进度条：8 个环境覆盖、采样/更新、回放、吞吐与30分钟保存倒计时；不增加文件，无限训练不显示虚构收敛百分比。

### Task 7: 构建、闭环验证和交付文档

**Files:** Modify README.md、docs/操作指令.md、config/incremental_navigation_interfaces.yaml、完整方案；Create docs/validation/2026-09-08-drl-exploration-training.md；Update 本计划状态。

- [x] 运行全部 tests/drl，相关现有启动/控制器静态检查；构建受影响 ROS 包并运行对应 CTest，含性能场景的包串行执行。
- [x] 执行本机真实导航/控制闭环、8 环境隔离与短程学习更新，记录目标到达、失败、途中观测、地图确认与实际里程。
- [x] 执行保存→进程退出→同目录恢复→新 episode→继续更新验证，核对固定文件数与写入峰值。
- [x] 使用隔离构建目录核对 Humble 构建/测试条件；可运行时执行，不可运行的目标层级明确写 NOT_RUN。
- [x] 文档写出实际可复制的依赖、构建、首次训练、正常中断、续训、评估与推理命令；区分工程测试和策略覆盖性能。
- [x] 按完整方案验收矩阵复核，进行任务与整分支审查，修复关键问题；UTF-8、链接、配置和 git diff --check 通过后显式提交。

## 进度

- [x] 创建独立工作树并核对源码基线；现有启动契约 21 passed、4 skipped。
- [x] 导入已批准完整方案，编写实施计划。
- [x] Task 1：105 项定向测试、300 例洞穴几何复核；审查通过。
- [x] Task 2：实现、定向测试与独立审查通过。
- [x] Task 3：实现、定向测试与独立审查通过。
- [x] Task 4：实现、定向测试与独立审查通过。
- [x] Task 5：实现、定向测试与独立审查通过。
- [x] Task 6：实现、定向测试与独立审查通过。
- [x] Task 7：225 项 DRL 测试及单独 3 项真实 ROS 闭环通过；Jazzy/Humble 各10包构建、core15/ros13/controller4测试通过；八环境续训至280转移/8更新，真实SIGINT保存恢复通过；最终审查APPROVED。

完整结果、资源峰值、短测与长期训练的边界见
[验证记录](../../validation/2026-09-08-drl-exploration-training.md)。所有工程测试模型已清理，
依赖与构建产物保留在项目外专用缓存；没有自动启动长期训练，没有合并或推送功能分支。

### 2026-09-09 后续核对

上述勾选为首次交付的历史验证。用户确认轮式速度上限为0.2 m/s后，配置及两个加载校验器已同步修正。
新速度下发现跟踪执行不一致，实际ROS回归2通过/1失败；当前持续探索尚未就绪。
详见[运动一致性核对](../../validation/2026-09-09-drl-motion-consistency.md)。
旧训练已正常保存退出并保留检查点，未自动混用旧经验继续训练。
