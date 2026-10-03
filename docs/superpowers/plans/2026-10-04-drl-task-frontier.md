# 任务内外前沿收益与 SAC 重新训练 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 统一 Actor 前沿收益、任务内消除奖励和 Critic 候选预测，从零训练 SAC，并与保留的26856进行同图分阶段评测。

**Architecture:** 实测自由侧前沿作为独立模块，旧 pending/witness 证书继续管理观测机会。27维节点、7维动作和新的消除收益贯穿输入、奖励、Critic、replay、导出与恢复。正常阶段先清空更新信用，再发布最终Actor、保存并评测。

**Tech Stack:** Python/NumPy/SciPy、PyTorch 离散 SAC、现有 C++ 原生可见性、隔离 ROS 2 Jazzy、标准库阶段控制和终端面板。

**Spec:** [已认可的设计](../specs/2026-10-03-drl-task-frontier.md)。用户于2026-10-04批准设计、实施计划及子代理执行；任务1基础工程已通过本机Jazzy验证（[证据与冷构建限制](../../validation/2026-10-04-drl-task-frontier.md)）；尚未实现后续新收益或启动完整训练。

## Global Constraints

- 专项分支 `feat/drl-task-frontier`，从 `dev/drl-exploration` 提交 `32155954f14b69d69ecb06d3b73bc7f7e09d084b` 派生；不合并、推送、创建PR或清理历史工作树。
- `task_graph_v4`：节点27维、动作特征7维；`candidate_frontier_truth_v2`；`candidate_graph_sac_v3`。
- 输入任务内外收益各8方向、各除100；奖励与Critic的消除数量除50。禁止把平方米值仅改名。
- FREE中心与未知需求都在任务内才属于任务内前沿；任务外按同域规则独立统计。边界中心属于任务内。
- 一次可见收益至少命中一个同域需求；消除要求冻结的全部同域需求有效观测，且旧前沿确已消失。
- 只奖励任务内消除；区外站位可以观测任务内；保留零收益通行、绕行和回退。
- Actor不得读取隐藏真值。成功仍为无碰撞且reference coverage≥99%；前沿为空不等于成功。
- 全部SAC状态从零，旧26856/checkpoint/replay/overlay保留；旧新输入语义不能普通resume混用。
- `paired_scene_sequence=true`；月表/洞穴配对，50–100m，每回合512决策；训练停滞截断保留128步/8位置，正式评测关闭。
- 独立warmup1024条；10阶段，每阶段8192条/2048更新；阶段后10组月表/洞穴即20回合。
- Actor/Q学习率1e-5，温度学习率0.003，初始双温度5e-5，上限1e-4；两个目标熵因子0.1；gamma=1，polyak=0.005，return_steps=1，position_aux_weight=0，batch64/microbatch8。
- 输出 `/home/kai/CodexDownloads/lunar_navigation/drl-task-frontier-20261003/`；新overlay `/home/kai/.cache/lunar-drl-task-frontier/jazzy/`；模型、构建、日志不入Git。
- 这些SAC超参数用于限制无关变量，其在新收益下从零训练的适用性尚待验证；20480更新是预算，不是收敛保证。

## Review Focus

1. 一个前沿有两个未知需求，只看到一个：可见计数1，预计消除和实际奖励0（Task 2/4）。
2. 任务内FREE仅邻接区外UNKNOWN、区外唯一站位和跨边界接口：不泄漏奖励，也不提前耗尽（Task 2/3）。
3. 负世界坐标、整数栅格origin移动、epoch/resolution/多边形变化：稳定身份或明确拒绝比较（Task 2/4）。
4. 刚到转移上限、并发完成及Ctrl+C处于更新/发布边界：预算不超限，恢复余额准确，最终Actor版本正确（Task 6）。
5. 评测缓存存在但SHA/schema/地图不匹配或缺terminal记录：不能当作完成或选优（Task 7）。

## 文件职责与验证环境

下文 `P=` `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration`，`T=` `ros2_ws/src/lunar_drl_exploration/test`；表中路径为仓库根相对路径。

| 文件 | 职责 |
|---|---|
| `P/frontiers.py`（新） | 稳定前沿、未知支持、可见计数与消除计算 |
| `P/feature_schema.py`（新） | 唯一特征列/尺度/语义常量，避免各模块重复硬编码 |
| `P/task_analysis.py`, `decision.py`, `graph.py` | 证书保持独立；传递实测前沿；输入与转向资格 |
| `P/contracts.py`, `ros_env.py`, `privileged.py` | 前后冻结、全动作奖励、Critic有效虚拟观测 |
| `P/batch.py`, `model.py`, `sac.py`, `config.py` | 新尺寸、尺度、双熵基础与新学习/恢复版本 |
| `P/checkpoint.py`, `replay.py`, `runtime.py`, `evaluation.py`, `collector.py`, `ipc.py` | 完整恢复、传输、发布与导出严格隔离 |
| `P/training.py`, `schedule.py` | 正常有界结束与信用偿还 |
| `P/reward_diagnostics.py`, `terminal.py`, `evaluation.py` | 新收益诊断，保留实际面积/覆盖/循环定义 |
| `scripts/drl/task_frontier/`（新） | 可提交的阶段控制、缓存校验、面板和曲线脚本 |
| `config/drl_task_frontier.yaml`（新） | 本轮完整配置，不改旧训练配置 |

执行任务测试前使用新overlay、源目录和既有依赖venv，不叠加旧DRL安装：

```bash
cd /home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-task-frontier
source /opt/ros/jazzy/setup.bash
source /home/kai/.cache/lunar-drl-task-frontier/jazzy/install/local_setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/lunar_drl_exploration:${PYTHONPATH:-}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
/home/kai/.cache/lunar-drl-training/venv/bin/python -m pytest <本任务列出的测试文件> -q
```

首次新overlay建立在Task 1。在此之前只做源文件/旧消息ABI的文档阶段检查，不声称新原生构建通过。

### Task 1: 固定基础、文档与双熵依赖

**Files:** 修改 `P/config.py`；创建 `T/test_frontier_config.py`；引入提交 `2d56d68e` 已验证的 `P/sac.py`、`training.py`、`checkpoint.py`、`cli.py` 和相应双熵测试；创建本spec/plan及 `docs/validation/2026-10-04-drl-task-frontier.md`。

**Interfaces:** 提供 `LearningConfig(entropy_mode='position_heading', position_aux_weight=0.0)`；双温度仍使用基础提交的接口，正的位置辅助权重在本轮明确拒绝。

- [ ] 记录branch/status/worktree/共同祖先和源26856 SHA；只操作专项树，保持主目录未跟踪配置。
- [ ] `git cherry-pick 2d56d68e` 仅带入独立位置/条件朝向熵；不带入后续 `dcca338b` 位置辅助损失。
- [ ] 增加 `test_frontier_config_rejects_position_aux`：0可序列化/恢复，正数报明确错误；实现固定零字段和验证，不实现辅助loss。
- [ ] 使用 `DRL_CACHE=/home/kai/.cache/lunar-drl-task-frontier/jazzy DRL_BUILD_JOBS=2 scripts/drl/build.sh --symlink-install` 建立专用overlay。
- [ ] 验证导入的 Python module.__file__ 与原生模块路径；跑 `T/test_factorized_entropy.py`、`test_entropy_fork.py`、`test_contracts_maps.py` 和新增配置测试，全部通过。
- [ ] 精确暂存本任务路径并提交；记录这是基础工程通过，不是新收益已完成。

### Task 2: 实测自由侧前沿与消除原语

**Files:** 创建 `P/frontiers.py`、`T/test_frontier_sets.py`、`T/test_frontier_removal.py`；修改 `P/task_analysis.py`、`decision.py`。

**Interfaces:**
- 不可变 `FrontierSet(cell_ids: int64[N], centers: float64[N,2], demand_offsets: int64[N+1], demand_ids: int64[M])`，需求使用CSR；每个前沿至少一个需求，所有数组冻结。cell_ids/demand_ids由同一有符号世界栅格坐标打包函数生成；centers保留真实世界格中心，不能用打包ID反推米坐标。
- 不可变 `FrontierSnapshot(epoch, resolution_m, lattice_origin, task_id, polygon_digest, revision, inside: FrontierSet, outside: FrontierSet, known_ids: int64[K], free_ids: int64[L])`；lattice_origin是固定栅格相位，整数origin移动保持身份。
- `build_frontiers(snapshot: MapSnapshot, workspace: MeasuredWorkspace, task: TaskSpec) -> FrontierSnapshot`。
- `count_removed(before: FrontierSnapshot, after: FrontierSnapshot, *, domain: str) -> int`，domain只接受inside/outside；要求全部旧需求已known、旧front不再存在、中心FREE分类未漂移，身份不一致拒绝比较。
- `TaskAnalyzer.frontiers` 和只读 `DecisionCore.frontiers` 暴露当前冻结值；旧TaskReport.frontier_cells/witnesses保持原含义。

- [ ] 写上述两个测试文件；断言FREE侧、BLOCKED非前沿、8邻格、边界中心、内外未知支持严格归属、每front去重；首次运行应因新模块缺失失败。
- [ ] 实现build_frontiers和稳定世界格身份；实测地图不裁任务外，不由隐藏地图补分类。
- [ ] 写并实现消除测试：部分需求0、全部需求1、新front数量增加不冲销旧消除、只观测区外0、同revision重复0、滚动窗口不删K、负坐标/整数origin移动等价、epoch/resolution/polygon变化拒绝、FREE重分类不获奖。
- [ ] 将frontiers接到现有analyzer/core，不更换pending/witness计算。跑新测试及 `T/test_task_analysis.py`、`test_cumulative_coverage.py`。
- [ ] 精确暂存并提交本任务，验证UTF-8和diff --check。

### Task 3: 分区收益输入与合法转向

**Files:** 创建 `P/feature_schema.py`、`T/test_frontier_features.py`；修改 `P/graph.py`、`decision.py`、`contracts.py`、`batch.py`、`model.py` 和现有图/模型fixtures。

**Interfaces:**
- `NODE_FEATURE_DIM=27`、`ACTION_FEATURE_DIM=7`；`IN_GAIN_SLICE=slice(3,11)`、`OUT_GAIN_SLICE=slice(11,19)`、`HISTORY_SLICE=slice(19,27)`；两个输入尺度100。
- `directional_frontier_counts(frontiers: FrontierSet, workspace: MeasuredWorkspace, positions: ndarray[N,2], sensor: SensorSpec, resolution_m: float) -> int64[N,8]` 在frontiers.py实现；同时判断FREE中心和至少一个需求可见。
- `GraphBuilder.build(..., *, frontiers: FrontierSnapshot | None=None)`；None仅在独立调用时从相同workspace构造，Core传已冻结值。
- 每个动作7列固定为相对XY、cos/sin、任务内收益、任务外收益、方向历史；原地转向使用独立旧pending接口可见矩阵。

- [ ] 写失败测试：区外站位内收益为正/外通道独立；中心可见但需求被BLOCKED遮挡/FOV排除时0；两个需求只看到一个计1；8朝向不串列，真实历史位于19:27。
- [ ] 按上述接口实现计数和27/7维构造；复用原生可见性，不引入新的近似射线。
- [ ] 分离旧pending可见矩阵和新收益矩阵；写 `test_external_frontiers_do_not_create_task_turn`、`test_cross_boundary_pending_turn_survives_zero_inside_frontiers`。
- [ ] 更新pack_observations和MeasuredEncoder尺寸/动作列；固定一组人工输入验证批处理、Actor/Q输出与动作一一对齐。
- [ ] 跑新测试和 `T/test_current_opportunities.py`、`test_graph_decision.py`、`test_valid_actions.py`、`test_metric_graph.py`、`test_model.py`；全通过后精确提交。

### Task 4: 全动作奖励和Critic候选预计消除

**Files:** 修改 `P/contracts.py`、`ros_env.py`、`privileged.py`、`batch.py`、`model.py`；创建 `T/test_frontier_reward.py`、`T/test_frontier_privileged.py`。

**Interfaces:**
- `RewardParts` 保留new_area_m2/distance_m/turn_rad，新增整数 `removed_frontiers_in`、`removed_frontiers_out`；两者必须非负，外部只作诊断。
- `make_transition(..., removed_frontiers_in: int, removed_frontiers_out: int, ...)` 不提供默认为旧面积奖励的回退；按spec完整公式计算。
- `PrivilegedBuilder.build(observation, observed, *, frontiers: FrontierSnapshot)`。
- `PrivilegedActionContext.gains` 替换为原始计数 `removed_frontiers`；PrivilegedBatch中的 `expected_removed_frontiers` 除50，只归一化一次。
- `predict_removed(before: FrontierSnapshot, terrain: TerrainGrid, pose: Pose, sensor: SensorSpec) -> int`：虚拟reveal仅接受finite heights且intrinsic!=UNKNOWN，调用Task 2同一消除原语。

- [ ] 写失败奖励测试：消除3、移动10m、转pi、未完成得到 `3/50-.02-.005-.001`；只观测外部得到无正探索项；完成仅加一次5；无碰撞且99%判据保留。
- [ ] step()在执行前冻结core.frontiers，在最终地图ACK/core.observe后结算；真实距离/转角仍从整段累计差取得，停滞继续按真实面积。
- [ ] 写候选测试：两需求只命中一个，预计0；无有限高度或未分类命中0；有效一次真值观测后的真实消除数等于预测；endpoint预测不要求等于沿途奖励。
- [ ] 实现predict_removed并替换旧面积gains；端点truth只流入Critic，Actor输入与候选集合不随隐藏真值改变。
- [ ] 跑新测试及 `T/test_ros_env.py`、`test_privileged_actions.py`、`test_graph_critic_schema.py`、`test_coverage_completion.py`；精确提交。

### Task 5: 学习、replay、发布和恢复语义一致

**Files:** 修改 `P/config.py`、`sac.py`、`checkpoint.py`、`replay.py`、`collector.py`、`ipc.py`、`runtime.py`、`evaluation.py`、`feature_schema.py`；创建 `T/test_frontier_identity.py` 并更新相应现有fixtures。

**Interfaces:** feature_schema提供新三个schema及 `FRONTIER_SEMANTICS`，固定前沿/归属/FOV/尺度/奖励公式标识；resume_semantics包含全部标识。新actor_state、导出、runtime和collector发布使用v4；外层bounded_training_v1可保留但旧内部语义必须拒绝。

- [ ] 写失败测试：旧19维Actor、旧面积privileged、旧replay/checkpoint、仅尺寸相同但奖励语义不符都拒绝；校验失败不得替换现有有效文件。
- [ ] 更新所有schema检查和传输冻结；新字段经dumps_transport/loads_transport、replay保存加载保持数值/单位。
- [ ] 从零创建learner、target、optimizer、温度、replay、RNG和0计数，证明target初始等于新Q且优化器空；无源checkpoint读入。
- [ ] 验证一个真实SAC update后save/restore逐项一致：模型/target、Adam、双温度、RNG、replay/schedule；导出Actor在相同新输入上与learner Actor一致。
- [ ] 跑新测试及 `T/test_replay_checkpoint.py`、`test_runtime.py`、`test_training_identity.py`、`test_collector.py`、`test_sac.py`、`test_evaluation.py`；精确提交。

### Task 6: 正常阶段补完更新，Ctrl+C恢复信用

**Files:** 修改 `P/training.py`、`schedule.py`；创建 `T/test_stage_credit_drain.py`，更新 `T/test_training_cli.py`、`test_bounded_training_signals.py`。

**Interfaces:** 维持 `train(..., max_transitions=...)` 的本次新增转移预算；控制状态分正常限额drain与用户/异常stop。正常drain不再GRANT，收齐/ACK既有完成，继续合法learner更新到credit=0，最终PUBLISH收到collector确认后，保存stopped状态。Ctrl+C保存credit并退出，resume在新派发前偿还旧credit；更新中异常不能保存污染optimizer。

- [ ] 写失败并发测试：到转移上限后无新派发，inflight=0、credit=0；warmup1024后第一阶段总9216trans/2048updates。
- [ ] 实现正常drain路径和最终PUBLISH确认；不能收到STOPPED后直接跳过尚未赚完的更新，不能无限等待已停止collector确认。
- [ ] 在credit=1/32、更新前后、PUBLISH前后注入Ctrl+C；断言恢复既不丢转移/信用，也不重复SAC update、transition ACK或阶段评测。
- [ ] 使用缩小warmup/budget的真实子进程fixture证明精确算术，固定 `transitions=warmup+s*block`、`updates=s*block/4`；覆盖最大并发reservations，credit上限32不突破。
- [ ] 跑新测试及 `T/test_schedule.py`、`test_training_cli.py`、`test_bounded_training_signals.py`、`test_training_fork.py`；精确提交。

### Task 7: 多阶段控制、同图缓存和三类曲线

**Files:** 创建 `config/drl_task_frontier.yaml`、`scripts/drl/task_frontier/{run.sh,controller.py,dashboard.py,curves.py,plot_curves.py}`、`T/test_task_frontier_controller.py`；修改 `P/reward_diagnostics.py`、`terminal.py`、`evaluation.py`。

**Interfaces:**
- `stage_target(stage: int) -> tuple[int,int]` 返回 `(1024+8192*stage,2048*stage)`；stage0为独立warmup。
- `validate_evaluation(record: dict, *, actor_sha: str, semantic_id: str, cases: list[dict]) -> None` 必须核对完整terminal记录、地形/reference ID、配置摘要和0error。
- 控制manifest包含phase、stage、targets、actuals、credit、checkpoint/ActorSHA、semantic_id、eval完成组数及退出状态；单根文件锁。
- 固定20cases：index=0…9，seed=2026100100+index，每seed月表+洞穴，extent=50.+index*50./9.，决策512。从旧baseline-summary原始JSON冻结实际数值与顺序到cases.json，并与旧continue.py表达式核对；不四舍五入重建地形。
- baseline使用旧 `phase12_python` 和保留actor-26856.pt，新模型使用本轮overlay；同种子/extent/sensor/navigation/referenceID配对，旧schema不进入新loader。

- [ ] 写并实现controller预算测试：不再减旧108452/26856；预热不足可恢复；trans已满但updates不足必须先drain；缓存错SHA/语义/scene或19/20条不能跳过。
- [ ] 从旧已接受的终端/曲线脚本复制功能到可提交脚本（旧文件保留），移除硬编码旧基数、外部工作树导入和旧奖励单位，启动入口显式选择新overlay。
- [ ] 配置写全Global Constraints；baseline单独调用旧wrapper，不把新RewardParts/schema强加给旧model。
- [ ] 记录三类曲线：数值学习（loss/Q/target/双温度/熵/目标差/上限占比）、固定状态动作排序和内外收益、固定20图覆盖/成功/面积零收益/循环/区外连续步/退出返回；另加奖励拆分。使用固定实测snapshot重新构造新特征，避免旧19维向量补零。
- [ ] 测试真实new_area=0定义在新奖励下保持；诊断三部分reward之和加完成项等于transition.reward，outside消除无探索项。
- [ ] 验证小窗口终端布局、训练/评测/暂停/恢复/错误状态，三类曲线输出UTF-8 CSV+PNG；跑新测试及 `T/test_reward_diagnostics.py`、`test_terminal.py`、`test_stagnation.py`、`test_evaluation.py`；精确提交。

### Task 8: 原生验证、阶段训练和文档交付

**Files:** 修改 `docs/DRL开发入口.md`、`docs/操作指令.md`、包README、本spec/plan及 `docs/validation/2026-10-04-drl-task-frontier.md`。产物全部写输出根。

**Interfaces:** run.sh负责start/resume/monitor；manifest与exit.json是运行状态证据，finished比较报告仅在20个terminal完整后生成。

- [ ] 重建新专用overlay，运行包内完整Python测试；相关同名ROS测试包分别运行。检查diff --check及UTF-8，记录确切通过/skip/NOT_RUN。
- [ ] 执行 `LUNAR_DRL_RUN_ROS=1 /home/kai/.cache/lunar-drl-training/venv/bin/python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_ros_native_execution.py -q`；用真实月表/洞穴小预算完成一次新特征→奖励→SAC更新→Ctrl+C→resume→精确正常drain烟测，烟测独立输出，不混入正式计数。
- [ ] 全专项新鲜代码审阅，修复阻止正确性的发现并重跑受影响测试。旧26856/source SHA与旧overlay保持。
- [ ] 固定cases/config/code/schema摘要；验证或补齐旧26856完整20图基线。基线完成再启动独立warmup1024，之后阶段1，正常结束核对9216/2048/credit0/inflight0和最终Actor版本。
- [ ] 阶段1评测20/20后进入阶段2…10。每阶段完整训练和完整评测才能选优；沿用旧screen评分元组：完成数、负的长期面积零收益循环失败数、覆盖总和；相同分数保持先前checkpoint，不新增路程决胜规则。实际路程继续作为独立效率指标报告。26856保留独立基线，不因新奖励总分不同直接比较。
- [ ] 每次状态报告核对进程树、manifest/exit、checkpoint/输出新鲜度和GPU，分别说明训练推进与能力是否改善；故障保留日志与最后有效checkpoint，未完评测不标finished或best。
- [ ] 更新文档实现/公式/命令/参数/证据/效果与剩余边界，验证完整10阶段最终比较或明确中断事实；Humble/Orin/实车仍NOT_RUN。只精确提交源码与文档，不合并/推送。

## 自审记录与交接

- spec每节映射Task 2–8；双熵依赖仅Task 1，位置辅助loss不引入。
- 前沿类型、raw count/归一化单位、ActionContext/Batch字段名在任务间一致。
- Review Focus五项分别由Task 2/3/4/6/7的明确断言覆盖。
- 接口冻结先于并行实现；奖励与Critic共享消除函数，不能各自发明近似定义。
- 推荐当前会话采用子代理逐项实现并逐项独立审阅。核心任务按2→3→4→5的接口依赖顺序推进；Task 6可在接口稳定后独立处理。实施前等待用户审阅本计划并选择执行方式。


文档阶段证据：专项工作树创建后源码未变；源包contracts/maps 14项测试在隔离Jazzy消息overlay下通过。此前未加载ROS消息的7项失败和pytest自动加载launch插件缺少lark的错误均属于环境入口，已使用匹配消息overlay并禁用自动插件后重跑通过；这不是新原生构建或新算法验证。
