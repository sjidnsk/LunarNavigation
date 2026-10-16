# 任务前沿预测计算优化

## 范围与等价性

基于 `a1382530c376a15f3c1184fdda76b651906b3681`，只修改 `privileged.py` 候选端点的任务内前沿消除预测。每次 `PrivilegedBuilder.build` 按冻结 inside 前沿准备 demand 的已知成员关系和 center 的 FREE 成员关系；使用有序 snapshot ID 的二分查询，不逐候选扫描整个已知地图。每个候选保留原生方向射线、有限高度且非 UNKNOWN 的有效观测，并只投影到冻结 inside demand/center。

所有旧需求已知且中心在观测更新后仍为 FREE，等价于旧虚拟 snapshot 中该前沿消失且满足 `count_removed` 的要求。已观测 BLOCKED center 不获收益；区外站位可以观测区内前沿，区外前沿不计收益。空 inside 仍调用原生观测以保留 sensor/pose 校验。准备对象只存在于单次 build，不能跨状态缓存。

Actor 输入、图与动作集合、原生观测、实际路线累计奖励、归一化、训练配置、SAC/优化器/RNG、checkpoint/replay schema 均不改变。

## 验证与局限

新增回归测试先在原代码上得到 `1 failed, 4 passed`，失败原因是候选计算合并十万个已知 ID。旧 snapshot 算法独立保留为测试 oracle；覆盖随机有效/无效观测、已知需求、非 FREE center、共享需求、跨 revision 刷新、 signed/fractional lattice、空 inside 错误校验。现有测试覆盖实际观测消除、区外站位及 Actor 不变。

证据位于 `/home/kai/CodexDownloads/lunar_navigation/drl-task-frontier-20261003/performance-optimization/`，基准脚本对旧模块和新模块交替测量 7 次并取中位数，包含完整 `PrivilegedBuilder.build`。所有动作描述数组（支持连接、距离、位置、方向及原始收益计数）完全一致。

| 状态 | 已知格数 | inside/outside 前沿 | 候选数 | 原计算 | 优化后 |
|---|---:|---:|---:|---:|---:|
| 保存月表观测 04710152 | 5521 | 0/318 | 56 | 190.4 ms | 6.48 ms |
| 保存月表观测 d0a509b4 | 5521 | 0/318 | 32 | 111.3 ms | 4.60 ms |
| 合成大地图压力状态 | 100456 | 390/0 | 56 | 6195 ms | 4.27 ms |

前两行还与保存的训练输入收益数组完全一致；第三行用远处已知 ID 扩大已知集合，属于合成复杂度压力测试，不能当作真实机器人性能。测量不包含运动、ROS、图重建或 Actor 推理，也不能据此宣称整步动作加速相同比例或策略质量提高。

Python 测试采用现有隔离 Jazzy native overlay，测试/基准明确加载优化工作树 Python 源。原生源和二进制未改；不代表 Humble/Orin/实车验证。当前活动训练源在验证期间保持不变，部署需停止已归属控制器、保留 checkpoint 和旧评测 provenance 后再恢复。

最终验证：完整 DRL suite 为 826 passed、2 skipped，4 个真实启动锁/身份测试因隔离工作树借用缓存而报 `foreign cache identity`；该保护符合预期，未修改缓存 marker 或绕过 wrapper。在缓存归属的原训练工作树复核这些未变的启动测试。最终小调整保留空候选不执行预测校验的旧行为，相关收益/候选测试 11 passed。原生 ROS 集成 2 passed（16.38s）。独立审阅两次均无阻断项，第二次覆盖空候选调整。

部署时，旧版阶段 case 记录归档并从相同 stage1 Actor 重新评测。26856 的 legacy wrapper、模块、二进制、配置、Actor 和 eval/controller/wrapper 文件保持一致；逐 case 校验后只重新绑定当前源码提交和 provenance digest，保留实际执行提交及原文件路径/hash，评测数值不改。SAC checkpoint、Actor、训练配置 hash 要保持原值。上述迁移和启动证据写入外部 `rollout-migration.json` 与 `rollout-verification.json`，不能把旧基线描述为优化源码的新执行结果。
