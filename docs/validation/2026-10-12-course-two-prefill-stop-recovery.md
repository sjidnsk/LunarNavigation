# 课程二补采停止与保存恢复修复

日期：2026-10-12。工作树 `lunar-runtime/.worktrees/drl-task-frontier`。

## 原因

00:44 的补采不是预算耗尽：新收录 473 条经验后，三类场景均提供至少 22 条可按回合均匀抽样的不同经验（每回合最多 4 条），满足 `preparation_ready`。4096 条是补采上限，满足供给后允许提前结束。更新次数保持 63488。

停止协议先发布最终 Actor、完成 admission ACK 和 BARRIER，再保存检查点，最后等待 STOPPED。原采集器先给 10 秒协作退出，后对剩余环境逐个 terminate/join；worker 明确忽略 SIGTERM，单个挂起初始化还需 3 秒等待再 kill。主进程所有确认均限 12 秒，这与 10+3 秒路径冲突，12 路还会累积串行等待。故已保存的检查点可用，但最后的停止确认被误判为失败。

## 修复与验证

协作退出保持共享 10 秒预算；到期后同时 kill 尚未退出且无待 ACK 完成记录的 worker，再非阻塞回收，STOPPED 要求全部 worker 真正退出。完成的转移仍先 ACK。STOP 确认预算包含 10 秒环境退出和 12 秒控制余量，发布与 BARRIER 仍为 12 秒；导航/初始化的 180 秒预算保持原值。

新回归使用 12 个不可取消的初始化：原版在限时内收不到 STOPPED，修复后全部退出且无残留 PID。采集器和训练 CLI 共 51 项通过；固定评测缓存、控制器和终端统计共 35 项通过。对本轮生命周期修改运行上述相关回归，没有重复未改动的原生射线性能和整包场景测试。

## 保存状态恢复

修复前源码、identity/state、原始错误日志和 `resume.pt` 已备份到 `/home/kai/CodexDownloads/lunar_navigation/drl-course-two-distance-sensor-10-30m-20261011/stop-recovery-20261012`。恢复点为 255449 转移/63488 更新，checkpoint SHA 为 `5fdeddb70021b6cbb1f9e39109872ee42305b3bea692cdb773a3b4101489a1c3`。当前 learner（含优化器/温度）的 checksum 与冻结 block31 完整状态完全一致；473 条新经验及原 RNG/collector 状态均从保存文件恢复。

60 场基线的 actor、config、case 和各结果文件 SHA 已核对。评测算法与场景未改，结果原文件与原 runtime 标记保留，用逐文件 SHA/from-runtime/to-runtime 的明确审核记录复用。任何字节、模型、配置或场景不符都会拒绝。模型训练后的评测采用新 runtime。

控制器已从同一管理目录恢复，baseline cached=60，直接进入 C3 首段训练，目标转移 257497（255449+2048），目标更新 64000（63488+512）。终端已重载，继续支持 P 保存停止、R 恢复、Q 退出显示。启动证据见修复目录的 migration-proof.json 和恢复后的运行日志；学习效果仍需首段完成后的固定集评测。

## 实际暂停恢复复验

真实训练恢复后已产生新采集和新优化器更新。再主动保存暂停，8.699 秒退出；核对的 39 个 learner/collector/worker/ROS 进程均已退出。保存状态为 255541 转移/63510 更新、unfinished_reservations=0，保留 1 次未消耗更新额度；最终 Actor 与 learner 更新版本一致。该 checkpoint、collector RNG 和训练 RNG 校验值记录在 paused-checkpoint-proof.json，并保存完整副本 resume-after-verified-pause.pt。

随后再次恢复，继续从该检查点训练，60 个基线缓存再次复用，新采集和更新推进。最新 live 证据为 recovery-complete.json；控制器和 learner PID 是记录时快照。首段固定目标仍为 257497 转移/64000 更新。此次手动暂停用于验收退出协议，当前已重新运行。
