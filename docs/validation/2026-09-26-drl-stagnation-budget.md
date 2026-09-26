# DRL 长期停滞回合截断

本文保留9月26日停滞修复的原始验证记录。后续统一存储预算、资源字段fork白名单、
小场景恢复训练和开发线整合状态以[9月27日记录](2026-09-27-drl-bounded-recovery.md)为准。

## 范围与判断

用户确认保持学习式解决，允许训练提前截断长期停滞回合。本次从
`dev/drl-exploration@452ca016` 派生 `fix/drl-stagnation-budget`，工作树
`.worktrees/drl-stagnation-budget`。没有合并、推送或更改原训练产物。

本次解决采集预算被无新增的重复动作占用的问题。它不改变 Actor、Critic、奖励、动作合法性、
导航或控制，不证明网络已学会解环。此前 `test/drl-loop-learning@d5de79c0` 的 Actor 对照
首臂异常退出、未形成效果结论；本修复不把该专项合入，也不将其历史工程测试当作效果证据。

## 实现契约

- 新配置 `config/drl_exploration_stagnation.yaml` 保持原训练目标和有效 batch64/microbatch8。
  仅训练 worker 显式以 `training=True` 创建环境时，启用 128 步停滞窗口。
- 连续无新增测量覆盖，且最近 128 个完成动作的实际终点归入不超过 8 个位置组时截断。
  按时间顺序取固定代表点、欧氏距离不超过 0.25 m 归组；使用有界滑窗，不依赖变化的图节点 ID，
  也不采用栅格边界敏感的位置取整。任何正测量新增会清空窗口。
- 持续经过不同站位的长程回退不会因“没有新增”单独触发。局限是：少数站位内长时间停留，
  然后才恢复收益的轨迹仍可能被截断；128 是保守工程候选，不是最优阈值或可恢复性证明。
- 最后一条真实转移以 `truncated=True` 保存，保持 `terminated=False`，SAC 继续从该末状态
  bootstrap；不改奖励，不发成功奖励。99% 达标或当前机会耗尽等既有终止条件优先。
  worker 等待 replay admission ACK 后才重置；截断后的同一环境禁止继续 step。
- `truncation_reason=STAGNATION` 与导航 `reason_code=GOAL_REACHED` 分开记录；终端显示“停滞截断”。
  同时达到原决策预算时，原因记 `DECISION_BUDGET`。
- 冻结评估默认 `training=False`，即使传入启用停滞的 YAML，也仍按原任务/决策预算评价。
  部署入口没有运行时禁回头、黑名单、强制换目标或随机解困。

外部采集截断与任务终止需分开，自举仅由真正终止关闭，参考
[Gymnasium 的截断说明](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/)。
本项目已有 `test_terminal_skips_empty_actions_truncated_bootstraps_and_target_detaches` 对
gamma=1 与 .995 验证这一契约。截断没有消除 gamma=1 的环内自举，不能代替长期价值学习。

## 继承已有训练成果

新增 `train --fork-from <原 resume.pt>`，与 `--resume` 互斥：

- 只读校验原检查点自身的 config/semantics，再允许变更三个停滞采集参数和新输出路径。
  安装目录迁移允许 capability_path 改变，但平台能力数值必须一致。
- 权重、Adam、温度、replay、累计采集/更新数、课程和 RNG 均保留。旧经验在未改奖励/任务的
  off-policy 目标下仍有效；这不保证原快照的经验分布足够丰富。
- 目的目录必须不存在或为空；源目录、软链接别名及非空目录均拒绝。开始新 worker 前保存
  初始可恢复检查点与来源信息。后续普通 `--resume` 继续保留 fork 来源。
- 停滞参数进入新检查点身份，旧 `--resume` 不静默改变采集规则。关闭停滞时保留旧身份兼容。
  改变奖励、模型、课程、学习参数等不在这个 fork 的允许范围。

## 历史轨迹反事实

本次重新读取原始文件计算，属于离线截断反事实，**不是新策略的导航效果或训练加速实测**。
原始脚本和 JSON：`/home/kai/CodexDownloads/lunar_navigation/drl-stagnation-20260926/`，
文件为 `replay_counterfactual.py`、`replay-counterfactual.json`、`window-sensitivity.json`。

| 输入 | 64 步窗口 | 128 步窗口 | 256 步窗口 |
| --- | --- | --- | --- |
| 旧长训滚动窗口：1380 转移、8 回合，全部零新增 | 8 回合触发，省略后续 857 动作 | 7 回合触发，省略后续 316 动作 | 窗口短于256，无法判断 |
| 9/23 短筛查窗口：1897 转移、9 回合 | 2 回合触发，丢失后来325.44 m²新增 | 未触发上述两回合 | 未触发 |

64 步两个反例分别在触发后 90/10 步恢复新增，期间 Actor 版本也发生变化。
不能认定它们永远无收益，也不能从日志认定它们是拓扑必要回退。128 步避免了这两个已知反例，
因此选作新配置；不能用当前有限样本保证其他回合不会被误截断。

旧 20 例中的 8 个成功回合，最长零新增均不足64步；此前最新 Actor 的洞穴75步成功轨迹
最长零新增24步，因此这些已知成功轨迹不会触发128步门槛。`closed-loop.json` 只保存目标点，
没有每步实际位姿；该文件的空间分组只可标为 goal proxy，不能当真实物理检测证据。

## 验证记录

| 验证 | 实测结果 |
| --- | --- |
| 核心停滞/环境测试 | 37 passed；新增用例先观察13个RED，再修复；包含实际位姿而非目标点、持续新站位回退、成功/耗尽优先、reset和默认评估关闭 |
| 配置/fork及既有训练恢复回归 | 77 passed；含真实多进程采集与完整128/8/6网络更新、fork后再次resume |
| 完整源码包测试 | 376 passed / 2 skipped，85.74 s；跳过项为opt-in原生测试 |
| 最终 colcon 包级完整测试 | 377 passed / 2 skipped，87.90 s；比源码首轮多1个无关cwd参数回归；test-result：379 tests / 0 errors / 0 failures / 2 skipped |
| 独立 Jazzy 构建 | 6 packages；首次全新cache 67 s，最终源码重建1.79 s |
| 新安装包原生导航测试 | 2 passed，16.05 s；真实规划、控制、动作到位与末转移 |
| 源码/安装身份 | 8个变更模块逐文件SHA256一致，native扩展来自新cache |
| 原检查点只读fork验证 | admitted98,212 / updates24,296 / replay1,428；只变更停滞窗口0→128，未创建训练目的目录 |
| 独立代码审查 | 已完成，未发现需修复的具体问题 |
| 反事实脚本交叉验证 | 独立脚本与产品monitor的51个回合/参数组合首触发步一致 |

首次完整源码运行在安装路径兼容修复落地前载入了旧函数，结果为375 passed / 1 failed / 2 skipped，
失败用例 `test_legacy_fork_accepts_relocated_capability_file_with_identical_values`，日志保留在
`pytest-initial.log`。修复只允许能力数值相同的安装路径变化，完整重跑见上表；没有放宽平台能力校验。

首次 colcon 包级测试另发现既有 `test_fixed_small_curriculum_keeps_large_counts_small` 用相对cwd
读取 YAML，结果375 passed / 1 failed / 2 skipped（`colcon-test-initial.log`）。仅修测试以
`__file__` 定位仓库配置，并增加临时cwd参数回归，相关7项通过；生产CLI的配置路径语义不变。

本机 Jazzy 源码/原生依赖探针（domain228，CPU Actor推理，无学习更新）：

- 旧Actor23788、moon/2026092205、60 m，关闭保护的128步基线：覆盖88.27%，预算截断，182.25 s。
- 同一Actor/地图、开启128步保护、256步预算：第183步停滞截断，末128步无新增、6组实际终点；
  覆盖49.75%，无碰撞，240.15 s；`STAGNATION`、非成功，所属原生进程全部退出。
- 当前诊断副本Actor24296、cave/2026092206、60 m、开启128步保护：第75步达到99.094%，
  `terminated=True / truncated=False`，成功奖励保留，78.28 s，无碰撞，所属原生进程全部退出。
  这复验了一个已知成功案例，不代表全部洞穴或大场景不会退步。
- 这两次原生运行因异步测量形成不同轨迹，而且预算不同，**不能直接相减为算法加速或成功率变化**。
  保护触发前没有改选点或推进动作；第二次只证明真实导航中的停滞截断接线生效。

源码探针使用已核对无native源码差异的loop-learning Jazzy依赖与本工作树Python；新cache安装包另行验证，
不把旧cache名称当作新包已安装的证据。所有原始日志/JSON保存在上述外部目录，命令脚本也保留。
原 `resume.pt` 完成后重新计算 SHA256，仍为
`71e2485ea4e53d59f2ca7a7f624c42715c0aba0967994cb459a3b7bd2fa95307`；原开发树和loop-learning树均干净。

完整大场景训练、训练后99%完成率对照、长期学习式解环效果、Humble、Orin、目标DDS和实车：`NOT_RUN`。
本机 Jazzy 仿真与本机 GPU 探针不能作为上述环境的验收。
