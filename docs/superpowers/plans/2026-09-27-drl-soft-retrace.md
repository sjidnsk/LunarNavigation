# 有界 soft Retrace 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 实现可回归的一步/最多八步soft Retrace，并完成一次预算固定的真实对照。
**Architecture:** 在现有Transition追加采集元数据，Replay按真实连续回合取片段；SAC只改变Critic目标。
完整恢复入口显式记录跨度变化，冻结Actor格式及推理逻辑不变。
**Tech Stack:** Python、NumPy、PyTorch、ROS2隔离Jazzy；现有IPC、Replay、artifact预算。
**Spec:** [设计](../specs/2026-09-27-drl-soft-retrace-design.md)

## Global Constraints

- 共同原25320完整检查点；gamma1、tau.005、原奖励/C0网络/lr1e-5、batch64/micro8、ratio.25。
- 每臂最多2048新增，60m/512决策/8环境，按相同槽位回合种子序列；原source只读。
- 同一根8GiB总/128MiB诊断；保留历史，新两臂snapshot各512MiB，不自动重训或续训。
- 没有运行时禁回头、黑名单、强制换目标、循环惩罚。评测关闭训练截断。
- 一步为默认；全部兼容/状态迁移必须显式，不能伪造旧经验mu。

## Review Focus

- 旧snapshot缺新字段：Task1/3测试默认一步且保留原权重、Adam、回放、RNG。
- 同scene不同回合和淘汰间隙：Task1测试不跨接、不改变anchor分布。
- Actor更新或DONE重发：Task2测试记录真实同次mu，token重发字段不变。
- gamma1、terminal空候选及truncated：Task3用手算值和真实网络回归验证；不套收缩保证。
- 八步实际常退回一步：Task3/5保留实际长度/权重标量，效果结论附使用比例。

## Task 1：连续片段与兼容数据契约

Files: `contracts.py`、`replay.py`、新`test/test_replay_sequences.py`（包内路径，下同）。
Interfaces: Transition.step_index/behavior_log_prob默认None；
ReplayBuffer.sample_sequences(batch_size,rng,max_steps)->list[tuple[Transition,...]]。

- [x] 先写拒绝半缺元数据、非整数/负step、非有限/正log_mu测试并观察失败。
- [x] 测交错回合、step间隙、不同scene、相邻状态不一致、terminal/truncated、actor_version变化及n=1同RNG锚点。
- [x] 实现共享引用序列及两臂共同维护、计费、淘汰的索引；无后继返回短片段，旧数据单条。
- [x] 旧transport/快照恢复、字节上限和索引释放测试通过；保留原sample API。

## Task 2：采样概率随真实动作传输

Files: `collector.py`、`worker.py`、`test/test_collector.py`或新增专用测试。
Consumes: Task1元数据字段。Produces: 同一STEP/token的behavior_log_prob和真实step_index。

- [x] 写测试证明概率来自被抽中的同次multinomial权重，归一化且不改变随机轨迹。
- [x] collector发送mu；worker校验执行动作/版本/回合/步号后附加到完成Transition，ACK重发复用payload。
- [x] 覆盖重发、动作中断、恢复及原无Torch worker约束；不改runtime.ActorPolicy。

## Task 3：soft Retrace目标与恢复边界

Files: `sac.py`、`config.py`、`checkpoint.py`、`cli.py`、`training.py`及相关测试。
Interfaces: LearningConfig.return_steps=1（整数1..8）；SACLearner.update允许单条或tuple片段batch，
只对首条训练Actor/Critic；单条目标继续用td_targets；多步公式见spec。

- [x] 写手算三/八步、截断ratio、零系数、熵只计一次、同策略期望、terminal/truncated测试并观察失败。
- [x] 实现detach的分块目标，任一前向不超过microbatch；n=1逐参数/optimizer更新回归，有限数值测试。
- [x] 加actual trace长度、multi比例、有效系数质量及legacy比例的有界标量。
- [x] 旧学习配置缺return_steps视为1；普通resume拒绝改跨度；paired-multistep只允许设计中的差异。
- [x] 初态身份仅排除return_steps并测试检测polyak/其他状态差异；训练按return_steps调用对应采样。
- [x] fork精确保留状态、CLI拒绝不合法模式、legacy/polyak原功能测试通过。

## Task 4：工程集成与单根控制器

Files: 现有scripts/drl入口按需最小修改；有界实验辅助脚本与产物放既有外部evaluation目录。

- [x] 独立检查公式/序列/恢复实现，修复问题后运行完整DRL包测试与隔离Jazzy构建。
- [x] 源码与安装一致；一次有限真实运行确认元数据和多步实际生效，预算计入实验，不暗中增加训练。
- [x] 控制器冻结代码/配置/源/脚本SHA，存储峰值预检，顺序1步与8步、各7200秒上限。
- [x] 复用原有退出归档、实际更新量/credit和完整初态比对；每臂只读一次完整终态并导出小Actor。
- [x] 保存多步实际利用率和训练摘要，不能用训练完成事件冒充冻结成功率。

## Task 5：真实对照与阶段关闭

Files: `docs/validation/2026-09-27-drl-multistep-experiment.md`、开发入口、操作指令。

- [x] 工程门槛通过后执行每臂2048新增，不因慢/结果不好增加预算或重跑。
- [ ] 冻结各5张开发图，核对同地图元数据、Actor未变、进程清理、实际99%完成和循环。
- [ ] 仅门槛通过才进入8回合新种子和最多4回合复核；关键仍失败则停止扩展。
- [ ] 汇总真实结果/局限/NOT_RUN、存储与代码版本，按明确路径提交；经干净状态和祖先验证同步开发线。

阶段记录：两臂训练及摘要已完成；2026-09-27 19:16启动冻结开发评测，结果待验收。
20:07套件BrokenPipe中断；一步5例完整，八步前三例完整，第四例只有预算结束进度。
保留原证据、归档失联包装器状态，仅补八步moon2702/2703两例，不重训或扩大评测。
已存cave2702退化意味着原门槛失败，补齐结果不触发新种子/复核，整阶段关闭仍待补评退出。
已验证工程与训练记录可先同步开发线；最终策略门槛及阶段关闭仍待冻结结果。
默认一步不变，不把代码合入解释为候选胜出。按用户要求，运行期间不持续轮询。

## 验证命令

通过现有installed_python隔离Jazzy环境，用Python在sys.path首位插入
`ros2_ws/src/lunar_drl_exploration`后调用pytest.main，确保测试新源码且保留native安装依赖。
先运行各Task测试，再运行整个`ros2_ws/src/lunar_drl_exploration/test`。
构建：`DRL_CACHE=/home/kai/.cache/lunar-drl-stagnation/jazzy scripts/drl/build.sh --packages-select lunar_drl_exploration`。
UTF-8、本地文档链接和git diff --check必须通过；不把Jazzy结果当Humble/Orin/实车通过。
