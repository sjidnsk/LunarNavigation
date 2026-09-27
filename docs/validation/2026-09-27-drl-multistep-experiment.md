# 小场景一步与八步回报对照

## 目的与当前边界

按用户确认的顺序实现多步收益传播，再从共同25320检查点做固定预算对照。
此前Polyak对照未解决关键moon2703图的循环，本轮不沿用0.02作为新基线。
默认仍为一步、tau=.005；八步是待检验候选。工程实现与真实策略效果分别验收。

设计见[soft Retrace设计](../superpowers/specs/2026-09-27-drl-soft-retrace-design.md)，
任务见[实施计划](../superpowers/plans/2026-09-27-drl-soft-retrace.md)。

## 改动

Transition增加同次采样的联合动作log_mu及回合内零起步序号；旧数据两个字段均缺省。
Replay仍均匀抽64个anchor，只沿真实连续、同回合且边界内容一致的记录扩展，最多八步。
序列复用原观察，不保存额外图副本，索引两臂共同维护并计入回放字节预算。
Critic采用带截断重要性比例的soft Retrace；Actor、alpha只训练原anchor。
一步继续原TD代码路径；真终止不bootstrap，预算/训练停滞截断保留最后状态bootstrap。

`learning.return_steps`默认1，合法整数1..8。普通resume不允许改变跨度；只有显式
`--fork-experiment paired-multistep`可迁移跨度并保留全部网络、目标、四个Adam、RNG、回放和计数。
原始检查点先按原语义验证；fork记录跨度迁移，源文件只读。初态对照仅排除return_steps，仍比较tau。

仅新增五项累计标量：anchor数、实际片段长度和、多步anchor数、旧经验数、有效系数质量和。
它们用于报告实际使用程度，不能把配置八步当作全部样本均用八步。

## 固定实验条件

| 项目 | 一步基线 | 八步候选 |
| --- | --- | --- |
| 起点 | 原102308条/25320更新、credit1、4954条回放 | 相同完整起点 |
| 回报跨度 | 1 | 最多8，lambda固定1 |
| 奖励/折扣/网络 | 原奖励、gamma1、C0、tau.005 | 相同 |
| 预算 | 最多2048条新增、7200秒 | 相同 |
| 场景 | 60m月面/洞穴、512决策、8环境、同槽位回合种子 | 相同 |
| 更新 | batch64/micro8、lr1e-5、ratio.25 | 相同 |
| 新落盘回放上限 | 512MiB | 512MiB |

共用原`drl-small-recovery-20260926`产物根：总8GiB、诊断128MiB；原完整源、历史Polyak结果均保留。
新快照限额只影响新保存，启动仍恢复原4954条。两臂顺序运行，失败不自动续训/重跑。

冻结joint argmax且关闭训练截断，先各5图：cave/moon×2026092701/2702及moon2703。
关键图仍失败、已有成功退化或无可信改善则停止扩展。通过才进入新种子8回合及最多4回合复核。
不新增运行时禁回头、黑名单、强制目标、循环接管或循环惩罚。

## 验证与结果

工程验证于2026-09-27完成：受影响DRL包全量 **567 passed、2 skipped（109.12秒）**；
隔离Jazzy构建lunar_drl_exploration成功。checkpoint/cli/training/config/contracts/replay/sac/
collector/worker/model/batch/ros_env共12个模块的源码与安装SHA一致。
两项跳过为未启用的可选测试；本地测试不代表目标平台验收。

测试覆盖手算一步/三步/八步回报、熵只计一次、零系数和比率截断、同策略期望、
终止空候选与截断bootstrap、一步完整更新一致、microbatch等价及Actor仅更新anchor；
真实worker/collector传输、执行中发布Actor、重发去重、采集至完整更新的累计trace统计、
旧检查点恢复、严格单变量fork、快照实际造成的内部缺口及回放预算。
审查发现paired-multistep继承了过宽的采集fork白名单，已收紧并以9项拒绝测试验证。

控制器与冻结评测纯契约检查通过，覆盖边界、credit、配置漂移拒绝、存储峰值、
实际trace使用和有限子进程超时回收。旧十状态面板若评分，标签明确为一步参考，
它缺历史行为概率，不能作为八步训练目标证据。

真实训练及冻结结果待执行后补入。工程通过不代表策略已解决循环。
Humble、Orin、实车及大图：NOT_RUN。
