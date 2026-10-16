> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../../DRL开发入口.md) 为准。

# 增量观测图强化学习探索实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 将现有连续矩形目标 SAC 训练与推理替换为图位置/朝向联合动作，并复用真实导航闭环。

**Architecture:** 单一派生地图生成不可变图观测，categorical Actor 选择物理位姿，现有导航和控制执行。CPU 采集与 GPU 学习解耦，有限回放和原子检查点负责接续。

**Tech Stack:** Python、NumPy、SciPy、PyTorch、ROS 2 Jazzy/Humble、现有 C++ 增量导航。

**Spec:** [完整设计](../specs/2026-09-09-graph-viewpoint-exploration-design.md)。

## Global Constraints

- 在已有 `feat/drl-exploration-training` 隔离工作树继续，保留进入时的导航重构和其他改动；不提交、合并、推送或清理他人文件。
- 图几何不认证平台路线；不添加批量预规划、目标投影、失败黑名单、覆盖停滞终止门槛。
- 固定 8 环境、0.2 m/s 上限、无横移、0.05 s 积分、10 倍目标倍率、10 m/90° 默认视野。
- 局部输入继承导航 64 m/0.2 m；任务外允许绕行，覆盖只统计任务内实际观测。
- 节点初始间距 2 m、动作范围 10 m、12 个朝向；全局区域 32 m，方向特征 36 个扇区。
- Actor/Critic 不读取真值；图、候选及 mask 必须随历史状态保存，不能用活图解释旧动作。
- 每 1800 秒墙钟保存；产物目录 `training-output/drl-exploration`，一个 resume、一个 best、限长指标；回放 8/12 GiB，磁盘回放子集 2 GiB。
- 验证按源码、Jazzy、Humble、Orin/实车分别记录，未运行部分标为 NOT_RUN；短测不能证明收敛。
- Python 测试在工作树根目录使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/kai/.cache/lunar-drl-training/venv/bin/python -m pytest`，禁用无关 ROS 自动测试插件；ROS 闭环另行 source 正确 overlay。

## Task 1: 不可变图契约、增量构图与地图适配

**Files:** `contracts.py`、`maps.py`、新增 `observation_graph.py`，均位于 `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/`；`tests/drl/test_observation_graph.py`、`test_maps.py`。

**Interfaces:** `MapObservation` 增加 `graph: GraphSnapshot | None`；旧纯地图测试可暂时不带图，但新策略必须有图。
`GraphSnapshot` 至少保存 `node_ids[N]`、`node_positions[N,2]`、`node_features[N,F]`、`edge_index[2,E]`、`edge_features[E,2]`、
`action_nodes[K]`、`action_poses[K,3]`、`action_features[K,A]`、`action_mask[K]`、epoch/revision。
所有数组不可变，特征维度通过模块常量声明；`goal(index) -> Pose` 仅解释本快照索引。
`ObservationGraph(config).build(store, pose, task, sensor) -> GraphSnapshot`；地图提供 dirty 块/只读细格访问。

- [x] 先增加未知目标、窄通道、任务外连接、旋转、快照不变性测试并观察失败：

```python
graph = builder.build(store, pose, task, sensor)
assert len(graph.action_poses) > 0
for x, y, yaw in graph.action_poses:
    assert store.state_at_world(x, y) == 1
before = graph.action_poses.copy()
store.apply(next_map)
builder.build(store, pose, task, sensor)
np.testing.assert_array_equal(graph.action_poses, before)
```

- [x] 按世界 2 m 块内 FREE 分量、细格跨界连接、dirty 更新实现几何图；分裂/合并使旧 ID 失效而不复用。
- [x] 图距离选动作，加入真实 FREE 当前位姿的非恒等转向；保留中转，按 12 朝向展开；无需导航预查询。
- [x] 实现局部细图/远处区域图/任务未知摘要及 36 方向特征，保持 sparse 邻接；不为每次观测复制完整全局栅格。
- [x] 增量读图与现有局部窗口统一；起点盲区连接消费所属地图层的只读结果，不自行推断 FREE。
- [x] 运行 `... -m pytest tests/drl/test_observation_graph.py tests/drl/test_maps.py -q`，记录失败到通过证据和特征 schema。

## Task 2: 共享起点几何的只读连接

**Files:** `ros2_ws/src/lunar_planning_msgs/srv/GetPolicyMap.srv`、`lunar_incremental_navigation_ros` 的 `policy_map_exporter.*`、节点服务回调和测试；必要的 core 几何接口及测试。

**Interfaces:** 地图响应附带当前起点、共享起点视图导出的真实 FREE 出口格及连接长度；图只消费出口，不把补丁格当作终点。
正常 FREE 起点不需要额外处理；接口仍为现有 GetPolicyMap，不新增服务或导航预规划。

- [x] 先测试 UNKNOWN 起点有界补丁只能导出真实 FREE 连接、不能将补丁计入 observed；无出口明确返回起点状态。
- [x] 复用 `RequestLocalStartPatchBuilder` 与当前平台/细地图 profile，导出临时起点连接；普通格邻接与禁止切角共用规则。
- [x] Python 地图适配保存该只读结果；图构造在起点真实 FREE 时保持直接连接。
- [x] 构建消息、受影响核心/ROS 包，运行受影响测试；不同发行版采用独立 build/install/log。

## Task 3: 图网络与离散 SAC

**Files:** `model.py`、`sac.py`、`tests/drl/test_model_sac.py`。

**Interfaces:** 保留 `RecurrentSAC.act/act_batch/update/state_dict/load_state_dict/actor_state_dict` 的入口名，动作输出改为整数索引。
模型输入 GraphSnapshot 的 schema 常量；Actor 输出 masked categorical，双 Critic 输出所有候选 Q 值。
`act_batch` 返回 `(indices[B], hidden[1,B,H])`；网络记录 categorical 模型版本，拒绝旧高斯权重。

- [x] 用两种候选数量的 batch 测概率和 padding，单候选概率为 1、空集合明示异常：

```python
indices, hidden = learner.act_batch(observations)
assert all(0 <= int(a) < len(o.graph.action_poses) for a, o in zip(indices, observations))
```

- [x] 局部 CNN + 3 层 sparse 图消息传递 + 32 全局 token + GRU；Actor、双 Critic 编码器独立，无全图 N² attention。
- [x] 实现离散 SAC 精确动作求和，后继状态采用自身 mask，terminal 不 bootstrap，截断使用真实后继；变长短序列带有效位。
- [x] 温度目标为 `0.3 log K`，初始 0.01，上限 0.02；预热后新转移 25000～100000 线性退火至 0，单动作跳过温度损失。
- [x] 使用合成已知 Q 值验证 Bellman 数值与 mask，再运行实际反向更新、独立梯度、burn-in、模型恢复测试。

## Task 4: 动作事务、奖励、图回放与配置迁移

**Files:** `contracts.py`、`ros_env.py`、`runtime.py`、`plant.py`、`reward.py`、`replay.py`、`config.py`、`config/drl_exploration.yaml`、对应 tests/drl 测试。

**Interfaces:** `Transition.action` 为整数，另保存物理 goal、导航结果、Actor 版本、里程/转角及奖励分项。
`RosExplorationEnv.step(index)` 使用最近一次返回观测的固定图映射目标；训练和推理共用该映射。
奖励分项固定顺序 `(new_area_m2, distance_m, rotation_rad, failed, collision, first_success)`。

- [x] 先测试冻结目标映射，不因地图更新重解释索引；旧连续动作明确不兼容。
- [x] 删除 DRL 的矩形动作缩放、高斯语义和默认 1000 步重置；普通导航失败仍是一条可学习转移。
- [x] 实际积分累计绝对转角；奖励采用 `area/78.54 - .01*distance/10 - .001*rotation/(2*pi) - .05*failure - 5*collision + 5*success`，碰撞优先且不叠加普通失败/成功。
- [x] 回放比较/恢复涵盖完整图与候选，历史对象重封装不可变且保留共享存储；短序列边界不拼接。
- [x] 新配置声明图动作、奖励、模型版本，保留 8 环境/10 倍/1800 秒/存储预算；移除失效旧参数和无关的严格指纹限制。
- [x] 运行动作、奖励、回放、地图和配置测试，保留覆盖/可探索面积独立性测试。

## Task 5: 独立采集/学习、保存与评估

**Files:** `training.py`、`worker.py`、`processes.py`、`checkpoint.py`、`evaluation.py`、`progress.py`、`resources.py`、对应 tests/drl 测试；可按职责增加 `collector.py`。

**Interfaces:** CPU 采集拥有 8 Worker 和 Actor 副本；GPU 学习拥有 SAC/回放，队列传转移；最新参数通道只保留一个版本。
转移队列上限 64，更新额度上限 32，预热 256 条不补历史更新债务；每 32 次更新发布参数。

- [x] 用可控慢学习器及独立 worker 事件测试：一个环境完成后可以再次派发，不等待 SGD 或所有环境；队列满时不丢结果。
- [x] 采集/学习分进程，CPU Actor 独立、动作边界切参数，以最近 8 步重建记忆；线程初始学习 8、采集 2、interop 1。
- [x] 基于新增转移推进课程和熵日程；没有新增转移时耗尽有限额度后等待，不无限学习同一经验。
- [x] 保存取优化步骤后一致快照，后台原子写入，环境继续执行；正常退出取消导航、完整转移入库后保存。
- [x] 续训恢复模型/优化器/有限回放，8 个新 episode；只验证语义兼容，不因线程、日志路径、保存间隔差异拒绝续训。
- [x] 完整验证固定 Actor，在任务结束或训练采样截断边界复用一个环境，不暂停其他采集；截断评估不选 best。
- [x] 进度展示实际面积、里程、转角、候选数、采样与更新速率、队列/额度、Actor 版本、实际倍率及保存倒计时。
- [x] 运行调度、checkpoint、progress、CLI、评估测试；验证 30 分钟由模拟墙钟驱动及原子写入失败保留旧文件。

## Task 6: 闭环验证、文档和新训练

**Files:** `tests/drl/test_ros_closed_loop.py`，README、`docs/操作指令.md`、完整设计状态及新增验证记录。

- [x] 全部 Python DRL 测试通过，受影响 ROS 包构建/测试通过；记录原有测试与重构后测试数量和范围。
- [x] 在独立 DDS 域跑真实导航、规划和控制闭环：移动、原地转向、地图增长、任务外绕行、导航失败不中断任务、恢复。
- [x] 验证 8 个真实环境可独立采集并发生学习更新，覆盖或真实运动有增量；不能用 UNKNOWN 立即失败构造吞吐短测。
- [x] 用 100～300 m 以及 1 km 程序场景测构图/推理输入规模和内存，保存标量证据，不落逐帧地图。
- [x] 更新配置说明、启动/中断/恢复命令和实施差异；标清 Humble 与 Orin/实车未运行范围。
- [x] 用户已授权按新方案执行并允许清理旧训练产物；在验证新入口后仅通过现有产物管理器切换旧连续经验，启动新图策略训练。
- [x] 最终审查整个本次增量，修正关键问题；不提交、合并或推送。

完成记录：源码/测试/新图训练及接续已执行，证据见[验证记录](../../validation/2026-09-09-graph-viewpoint-exploration.md)。1km规模结果为稀疏已知地图，完整固定集策略收敛和Orin/实车仍为NOT_RUN。

- [x] 持续训练发现固定起点位于原生BLOCKED坡面：保留种子和失败证据，在场景拥有层构造局部起始支撑面；
  月表/洞穴各三个起点原生地图与实际转向回归通过，保留已有428条图经验接续训练。

- [x] 运行暴露长期无收益场景占用采样环境后，分离采样与任务结束：训练256动作、自动验证1024动作预算；
  原奖励/导航结果和真实后继保持，记录截断摘要，真实八环境换图及已有检查点接续验证通过。
