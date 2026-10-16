# 课程二启动、恢复与终端验证

本记录证明 A1 已实际启动、保存停止和恢复可用、面板已更新。**完整课程未完成，大量程阶段尚未解锁，学习效果尚未通过冻结评测。**

后续环境分配修正、8→12→16 环境完整状态迁移、分数额度恢复修复和实际吞吐证据见 [调度修正与扩容续训](2026-10-10-course-two-scheduler-repair.md)。下文的 8 环境/PID 数值为初次启动时的历史记录。

## 来源与配置

- 原完整状态：`/home/kai/CodexDownloads/lunar_navigation/drl-a2-ordered-continuation-20261007/managed/run/resume.pt`。
- SHA256：`cf58f56b1aaf5411645123ffc76b027eaefa4146391cac6ef2e8ae1fc755f6bc`；原文件未修改。
- 原累计 254,976 转移、63,488 更新。Actor 对应已完成课程 block31。
- 新目录：`/home/kai/CodexDownloads/lunar_navigation/drl-course-two-20261010/managed`。
- 8 环境、CUDA、原 SAC/奖励/A2 图与导航参数；A1 10 m / 90°、g=1–5 m；先限 2,048 新转移，目标累计 257,024。
- 自动保存 300 秒，完整 snapshot 上限 512 MiB，训练产物预算 12 GiB，进程 PSS 上限 20 GiB、系统保留 4 GiB。外部冻结备份和逐场景评测目录另计，未声称被训练产物预算覆盖。
- `identity.json` 验证 fork 前后的 learner、replay、schedule、RNG、collector 状态 SHA256 一致；课程二独立计数，原课程历史保留。

## 验证证据

1. 原包完整 Python 套件：1,041 passed / 2 skipped，326.66 s，日志 `/tmp/course2-full-suite.log`。该运行早于最后审阅修复；后续针对阶段恢复、固定传感器、fork、零决策计数、量程门槛、信号回归分别通过 28 / 46 / 15 项专项检查，日志分别为 `/tmp/course2-repaired-tests.log`、`/tmp/course2-final-focused.log`、`/tmp/course2-launch-fix-green.log`。
2. 10 m × 60/90/180/360° × 月面/洞穴，共 8 个真实 ROS 短闭环通过。每例完成初始化和冻结 Actor 一次决策，检查初始零区域覆盖、同一 SensorSpec 贯通、参考身份固定与无碰撞。输出：实验父目录 `qualification-lattice/native-preflight.json`（取前 4 个 10 m 成功记录）、`qualification-cave-10m/native-preflight.json`。这不是完整回合成功评测。
3. 实际保存：255,144 转移、63,529 更新、待更新额度 1、未完成 reservation=0。相对原状态增加 168 转移、41 更新，剩余 1 次更新额度被保留。停止原因 SIGINT；`run.json` 报 `owned_children_closed=true`，逐 PID 检查所属训练子进程退出。
4. 保存的完整模型与优化器经 `TrainingState.restore` 恢复，哈希与保存的 learner 状态一致。回放包含 `c2_A1=134`，以及旧课程四标签；8 个未完成场景的规格已保存。检查点副本和审计在 `recovery-check/paused-resume.pt`、`paused-audit.json`、`paused-run.json`。
5. 真实恢复后剩余采集预算为 **1,880**，目标仍为 257,024。运行记录确认原 8 个未完成场景重新初始化，seed、课程二 scenario_id 和 SensorSpec 均匹配；计数持续增加。审计时达到 255,763 转移、63,684 更新，证据 `recovery-check/resumed-audit.json`。物理回合从起点重建，不是模拟器逐帧恢复。
6. 面板更新位于独立 `scripts/drl/course_two_dashboard.py`，不改变运行中的训练代码身份。80×24、124×30、150×48 概览均按终端字符宽度检查；真实 PTY 验证 Tab 详情、返回和 Q 退出，返回码 0。只替换监看进程，训练控制器 PID 45914 保持不变。证据 `recovery-check/panel-*.txt`、`panel-pty.ansi`。
7. 22:55 增加当前训练块预计倒计时及预计结束日期/时间。28 项定向脚本断言覆盖近期速度替代进程均速、刷新间倒计时、暂停/恢复、过期指标、零速度、新块、进程退出、采集完成与评测边界，以及三个尺寸的概览/详情字符宽度。真实 PTY 检查倒计时显示、Tab 切换和 Q 退出通过，返回码 0。校验使用与运行中训练一致的 `DRL_CACHE=/home/kai/.cache/lunar-drl-task-frontier/jazzy-task8-20261004`，确认 runtime 与 identity/readiness 一致；默认 `jazzy` 缓存含不同原生库，不能用于此身份对照。监看 PID 47757 替换为 55007；控制器 45914、训练 51893 均保持，转移从修改前 257,703 增长到刷新后 257,851。证据在 `recovery-check/countdown-20261010/`，含修改前备份、`verification.json`、`panel-*.txt`、`panel-pty.ansi`、`live-reload.json`。此改动只估计当前采集预算耗时，不含保存/评测，也未修改环境任务分配。

## 发现与处理

- 独立审阅复现了跨阶段旧场景占满槽位、信用耗尽而新任务预热不足的死锁。晋级时重新分配场景，同阶段恢复保持原规格；新增回归通过。
- B 的新量程档位改为先验收 g≤5 m 容易层，评测和训练均计入 B 总预算。普通 CLI 恢复默认读取已保存的量程课程状态。
- 初始化即完成单列 `zero_decisions`、`initial_completed`，不算策略成功。
- 初次 6 环境启动因旧 reward diagnostics 绑定 8 个槽位而失败，未采集；完整保留失败目录，再从原状态按 8 环境迁移。fork 现禁止该隐含配置变化。
- 连续起点生成 9.116 cm 短移，原生导航反复 `PLAN_FOUND/PATH_DEVIATED`，没有平移。新生成器整体平移场景，使初始位姿遵循旧课程的 cell-center 约定；精确 d/g/参考路径保持一致。原日志保留在 `qualification-goal-audit`。底层通用短距离导航故障仍是独立问题。
- 40 m / 0.2 m 场景触发 `exact ray prefix template exceeds resource budget`。没有通过增大缓存、改变传感器模型或降低分辨率绕过。readiness 只解锁 A1/A2；B/C/D 到达时会保存停止，待原生后端修正及完整 24 场景预检后再处理。100 m/360° 的全链验证、单调性/旋转不变性和大图学习资源验证尚未完成。

这些证据不证明策略已学会使用目标方向，也不替代冻结开发验证、正式四组对照、留出泛化及 Orin/实机实验。
