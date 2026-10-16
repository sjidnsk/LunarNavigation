> 历史档案（2026-10-17 归并）：来源 `feat/drl-position-temperature-20261013@d7de50d41e782052a70f73f8b012930082697d0d`，原专项基线 `8b0b6319598d0ef7735ae804ba9da1ce3f61a4af`。正文保留历史记录；旧 PID、路径和启动命令不代表当前运行环境，请从 [DRL 开发入口](/docs/DRL开发入口.md) 核对当前开发/部署身份。该实验已被后续版本替代的部分不作为当前协议。

# 课程二位置熵系数上限调整与 Actor–Critic 诊断

日期：2026-10-13。专项分支：`feat/drl-position-temperature-20261013`。

本次按用户确认，将位置熵系数的上限从 `0.0001` 提高到 `0.001`，航向上限保持 `0.0001`，从当前保存状态继续课程二 block02。`0.001` 是本轮试验值，不是已经证明最佳的参数。提高上限的目的，是让现有自适应调节器在位置熵不足时有继续增大系数的空间；是否提高任务能力，仍由固定场景评测判断。

**当前交付状态：源码、专项测试、独立构建和正式迁移已完成，训练已恢复，位置 α 已越过旧上限；完整包回归的最终汇总见下文。** 这些证据确认配置生效及运行恢复，不表示策略效果已提高。

## 1. 本次变化及保留内容

| 项目 | 处理 |
|---|---|
| 位置熵上限 | 新配置 `maximum_position_alpha=0.001` |
| 航向熵上限 | 原 `maximum_alpha=0.0001`，在当前 `position_heading` 模式下继续约束航向 |
| 实际位置与航向 α | 保留源检查点的 `log_alpha`、`log_alpha_heading`，不直接设为新上限 |
| 温度优化器 | 保留 Adam 一阶、二阶矩和步数，不重置 |
| 网络与学习状态 | 保留 Actor、双 Critic、双 target、新区域查询分支及全部网络优化器 |
| 回放与调度 | 保留 replay、RNG、课程、采集器、计数和剩余更新信用 |
| 已通过的计算优化 | 保留 GPU 稀疏边点积及消息聚合融合，仍为 FP32 原公式 |
| 网络结构、奖励、采样 | 不改；辅助奖励上限仍为 10、尺度 50 m；三类场景/回合均匀采样不变 |
| 训练和评测安排 | 继续 block02 原边界；每段原定固定 60 场景评测不变 |

这里的 α 是 SAC 熵正则系数，并非直接作用于动作 logits 的 softmax 温度。当前 Actor 仍直接计算 `softmax(logits)`。提高 α 上限不会立刻摊平动作分布，也不会把实际 α 立即跳到 `0.001`；实际 α 从保存值继续自适应更新。

熵项同时进入 Actor 目标及 Critic 的 soft bootstrap 目标，因此本次属于学习目标系数允许范围的变化，应记录清晰边界。不能将其描述为只有推理采样噪声变化。

## 2. 实现与迁移语义

实现文件：

- `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/config.py`：增加可选 `maximum_position_alpha` 及有效位置上限属性；旧配置缺失该字段时规范化为 `None`，继续使用原上限。`joint` 模式保持原有上限语义。
- `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/sac.py`：位置 `log_alpha` 按有效位置上限截断；航向 `log_alpha_heading` 仍按原 `maximum_alpha` 截断。普通 checkpoint 恢复继续严格校验学习语义，不能靠修改配置绕过。
- `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/position_temperature.py`：提供显式 `raise_position_temperature_cap` 迁移，只接受分解熵模式下批准的 `1e-4 → 1e-3` 位置上限变化，并要求未完成预留为零。

迁移函数只返回独立拥有的检查点及证明，不自行写文件、不控制进程。调用方须先保存暂停、确认控制器及训练子进程退出、备份源文件，再协调检查点、训练配置、运行身份与入口的更新。迁移记录保留 `learned_state_sha` 和 `preserved_state_sha`，分别约束除学习配置元数据外的完整 learner 状态，以及 replay、schedule、RNG、curriculum、collector_state、counters。证明明确记录 `temperature_reset=false`、`optimizer_reset=false`、`replay_reset=false`。

配置与迁移元数据变化会改变检查点文件 SHA；不能要求此次文件 SHA 与迁移前相同。应保存原文件及 SHA，并核验被要求保留的学习与经验状态内容校验值相同。实际温度恢复值和更新计数以正式暂停检查点为准，不能拿较早诊断检查点的计数冒充迁移边界。

## 3. 调整依据

在调整前核查至 update 66047 的最近100条不同更新记录中，位置熵中位数约 `0.126`，目标中位数约 `0.313`，100/100条位置熵低于目标，而位置 α 一直处于 `0.0001` 上限；每批平均最高位置概率的中位数约95.2%。源码温度损失为 `log_alpha × (entropy − target_entropy)`；熵不足时会推动 log α 增大，随后被上限截住。这支持优先试验放宽位置上限，不支持宣称该因素已被证明是全部失败的根因。

航向 α 的 float32 指数值为零是源模型已继承的状态。本次不重置或提高航向上限。log α 的温度损失梯度不依赖 `exp(log_alpha)`，因此指数下溢不等于温度参数永远失去梯度；也不能据此直接认定区域方向选择失败由航向项造成。

## 4. 调整前 CPU 静态诊断

诊断输入为计算提速迁移前冻结文件：

```text
/home/kai/CodexDownloads/lunar_navigation/drl-region-query-bound10-20261013/managed/migrations/fused-edge-message-20261013/resume-before.pt
updates = 65964
SHA256 = 2e3e653b567c60dc84403cd1fd201acf17ea3151a70b0e90d36f188cafa9be90
```

脚本与完整输出已保存至实验目录（原 `/tmp/position-temperature-diagnostic-20261013` 是执行暂存目录）：

```text
/home/kai/CodexDownloads/lunar_navigation/drl-region-query-bound10-20261013/managed/migrations/position-temperature-20261013/diagnostic/
  diagnostic.py
  diagnostic.json
  run.log
  README.md
```

该诊断只使用 CPU，Torch 和数值库最多 2 线程，未创建或推进优化器、未初始化 CUDA、未修改训练状态。原检查点 SHA 前后相同。12 个前向探针合计约 11.6 秒；此耗时不包含完整进程启动与输入反序列化，不能解释为训练性能指标。

### 筛选与排名口径

冻结回放含 795 条经验，其中区外月表 564 条。407 条满足“当前位置在区域外、动作前参考任务区域已观测格数为零、记录动作新增任务面积为零”。限制测量图不超过 2000 节点、真值图不超过 12000 节点后，剩余 406 条、14 个回合。每回合只取最新合格状态，按当前位置距任务区域不超过 50 m、超过 50 m 各选最多 6 个，最终得到 12 个独立回合的状态。

超过 50 m 的当前距离是机器人运动后的状态，不意味着把课程起点距离上限改大。回放行为版本范围为 64704–65952；本次统一用 update 65964 的 Actor/Q1/Q2 对这些历史观测重新前向评分。只有 1 条选中记录带 `truncated`，其余记录未结束，故不能称为“12 个最终失败场景”。

Actor 位置概率为同一位置全部合法航向联合概率之和。主要 Critic 排名先逐动作取 `min(Q1,Q2)`，再按当前 Actor 的条件航向概率聚合为位置 Q；另存每个位置最佳航向的 min Q，避免将位置与航向偏好混为一谈。候选终点到任务多边形的欧氏距离变化只用于描述接近方向，不能作为最优动作标签；绕行可能暂时远离任务区域。Q1/Q2 的差值和排序分歧是诊断量，不是置信区间。

### 结果

| 指标 | 结果 |
|---|---:|
| Actor 与条件 min Q 最优位置一致 | 9/12 |
| Actor 与最佳航向 min Q 最优位置一致 | 9/12 |
| Q1、Q2 条件最优位置一致 | 7/12 |
| Actor 最大位置概率中位数 | 99.68% |
| 存在几何接近候选的状态 | 12/12 |
| 条件 min Q 最优位置几何靠近／远离 | 6／6 |

三个 Actor 与条件 min Q 排名不一致的状态：

| 回合前缀、步数 | Actor 所选位置的 Q 排名 | 最优位置 Q 高出多少 | Actor 给 Q 最优位置的概率 |
|---|---:|---:|---:|
| `ebae2d26acba`，26 | 8/8 | 3.563 | `5.45e-14` |
| `2f2f7ea96a30`，127 | 2/8 | 0.090 | `1.88e-13` |
| `8283dc645ce4`，159 | 2/4 | 0.515 | 5.74% |

第一例中两个 Critic 都将另一位置排第一，但 Actor 几乎完全选择第八名。Actor 所选终点几何上远离约 2.30 m，而 Q 第一名终点靠近约 2.27 m。这是 Actor 高度集中且未跟随当前 Critic 排序的明确局部证据。

另一方面，9/12 状态 Actor 已跟随条件 Q 第一名，且 6 个状态的 Critic 第一名暂时远离任务区域。增加熵只能增加尝试机会，不能保证修正 Critic 的长期价值判断；这些远离动作是否评错仍需闭环证据。本样本经过困难条件筛选，不代表总体成功率，不能从局部排名直接推断新上限能提高成功率。

完整 JSON 保留浮点原值；少数位置概率和比 1 大约 `1e-7`，属于 float32 求和误差，展示按约 100% 理解。独立复算已检查回合不重复、概率和、排名和 Q 差一致。

## 5. 验证状态与后续验收

本轮已报告的专项执行结果：

- 位置上限测试先出现 14 项预期失败，实现后 14 项通过；相关回归 82 项通过、2 项跳过。
- 显式迁移测试先出现 6 项预期失败，实现后 6 项通过。
- 完整DRL包共1282项：首次CPU环境1244通过、11失败、27跳过，耗时668.89秒。11项均由未载入ROS环境导致：`test_contracts_maps.py` 7项缺 `lunar_planning_msgs`，`test_cumulative_coverage.py` 1项和 `test_runtime.py` 2项缺 `lunar_pure_wheeled_controller`，`test_task_frontier_controller.py` 1项缺 `ament_index_python`。在独立Jazzy环境关闭无关pytest插件自动加载后，对这4个文件完整重跑，分别14/11/4/31项全通过。跨正确环境汇总1255项通过、27项CUDA/显式启用类测试跳过；不能描述成一次统一环境运行零失败。
- 更早一次通过ROS启动脚本运行pytest时，插件自动加载因缺 `lark` 在测试收集前退出；保留在 `regression-environment-error.log`，它与上述11项测试失败记录不同。本轮未安装依赖或修改既有测试掩盖环境问题。
- 独立Jazzy缓存6包构建通过：`/home/kai/.cache/lunar-drl-position-temperature/jazzy-20261013`。65个安装Python模块与源码逐字节相同；相对计算提速版本仅 `config.py`、`sac.py`、`position_temperature.py` 运行模块变化。
- 正式暂停边界：累计转移266537，更新66242，未完成预留0。迁移及恢复已执行，内容校验见下一节。

已有 GPU 消息聚合融合的独立性能及数值证据见 [融合优化记录](2026-10-13-fused-edge-message-performance.md)。本次不以重复提高温度的学习实验代替计算优化验收，也不将既有融合提速归因于 α 调整。

恢复后先确认实际 α 从原值继续变化、航向上限及状态保留、网络与优化器没有重置、课程与更新计数连续，再在原段边界完成同一组 60 场景评测。既定训练累计边界仍为 263709 / 271901 / 280093 / 288285 条，更新边界仍为 65536 / 67584 / 69632 / 71680；当前目标是继续 block02。

验收同时查看初始不可见场景的首见、完整覆盖成功、决策和行走开销，以及区内月表和洞穴回归。熵提高、α 上升或数值有限都不单独代表学习改进。block02 已包含调参前训练，后续将在同段继续，因此是前后学习设置混合的一段，不能宣称为从同一初始状态、相同预算得到的纯单因素对照；正式归因仍需冻结起点和预算的独立对照。

Humble、Orin、真实机器人验证：`NOT_RUN`。本机源码、CPU 诊断和训练恢复证据不能外推为实机验收。

## 6. 运行迁移证据

证据目录：`/home/kai/CodexDownloads/lunar_navigation/drl-region-query-bound10-20261013/managed/migrations/position-temperature-20261013`。

测试汇总为 `verification.json`，首次全包结果 `regression-cpu.log`，补跑结果为 `ros-contracts.log`、`ros-coverage.log`、`ros-runtime.log`、`ros-frontier.log`；构建日志 `build.log`。所有首次失败结果均保留。

- 原检查点SHA：`f96f6faf7ea855ce69f620837c5136c776b866908a8702f0a43343620b639b8e`。
- 迁移后、恢复训练前检查点SHA：`36225c54fb5450f43cae9d63be940ef495b18c9d8ba0b664db205d0f5bb76161`。
- 两侧学习状态内容SHA一致：`3d30a18cd90967ba5ae86041b68d7b86c15bacc590a11b02b8bb3cce5d5c98f4`。覆盖网络、target、全部Adam状态、log α及更新数，排除有意修改的学习配置元数据。
- 两侧回放/RNG/调度/课程/采集器/计数内容SHA一致：`5b3eaa2b1af7da902a5fcc177e41edbd88fdfcb0ae2d5813bd497e8abf7f0ba4`。
- 新运行runtime：`a30d7b616568b2bd0089f373d8802580f65d06187f1f94fb52ae5ee480113e24`。

保存副本 `resume-before.pt`、候选副本 `resume-after.pt`，同时保存配置、身份、启动入口前后文件、迁移证明 `migration.json` 和实际部署脚本 `apply-migration.py`。迁移全程持控制器锁，先准备并验证候选文件，再以同目录替换提交；异常时逆序恢复原文件。它是带回滚的多文件提交，不是硬断电原子事务；硬中断后须依据备份核对恢复。

原基线60项缓存逐项核对case、Actor/config SHA与runtime来源后，仅更新采用记录的目标runtime；原结果文件和来源保留，block01结果不重写。原prefill检查点及其SHA保留。恢复命令仍使用原实验 `control.sh`，从同一课程block02继续，剩余5364条转移，目标271901。

恢复进程快照：控制器309253、训练309299、面板309595，三者均核对使用新缓存；旧面板已关闭。终端保留原最新布局及P/R/Q操作，位置/航向温度在已有指标区显示。`postflight.json` 保存了恢复约171秒、40次更新的观察：累计266693条转移、66282次更新，位置α从第一步的`0.000100394`升到`0.000116234`，航向指数值仍为0，学习指标均有限；排除前三次后的更新耗时中位3.6177秒，没有Traceback、OOM、融合回退或NaN标记。这证明位置自适应越过旧上限，不证明任务质量改善。PID和计数均为当时快照。

保留的是暂停保存检查点所包含的回放；既有512MiB快照限制没有改变，不能称为暂停前全部内存经验或物理仿真瞬时状态的无损恢复。
