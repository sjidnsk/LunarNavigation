# 区内洞穴阶段 50/50 配额续训

用户授权：仅将当前区内洞穴阶段的新增转移和 replay 采样配额同时改为区内月表 50%、区内洞穴 50%，奖励、预算和网络保持一致，继续训练。

## 实验边界

- 原实验 `/home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-30-100-bonus20-20261004/managed` 已安全暂停，作为 20/80 对照保留。
- 新实验 `/home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-ratio50-20261006/managed` 使用显式 `ordered-ratio` 检查点迁移。恢复点为 101,303 条转移、25,069 次更新，更新信用保留 3/4。
- 配置新增 `ordered_rehearsal_by_phase=[0.2,0.5,0.2,0.2]`，仅第 2 阶段变化。普通恢复严格拒绝比例变化；后续阶段恢复原 20% 旧任务配额。
- 完整保留 Actor、Critic、目标网络、优化器、温度、replay、随机状态、调度和采集状态。保留历史成功数、回合数、覆盖率；仅从切换点清零当前阶段转移/采样配额计数，避免用新比例追补历史分配。
- 完成奖励 +20、训练 30–100m、固定评测 50–100m、面积决策预算、网络、学习率、熵设置、导航、固定场景及每块 8192 条转移 / 2048 次更新均不变。
- 课程保留全局 12 块已完成记录及当前第 13 块边界 107,520 条转移 / 26,624 次更新。新配比独立开始最多 10 块验证窗口（`allocation_epoch` 记录原阶段已完成 9 块），连续评测通过要求仍为 2 次。阶段监看显式区分配比窗口和全局历史。

## 验证

- 完整检查点迁移逐项相等检查通过；原检查点 SHA256 `ceb335f31f07a645611db272e2681a2c747914e81df1d9d250ef0942daa8eb85` 保持不变。证据 `fork-verification.json`。
- 独立代理审阅通过，独立运行 55 项测试；最终相关测试 132 项通过。证据 `independent-review.md`、`tests.log`。
- Jazzy 构建 6 包通过。真实 CUDA/native 短训练新增 128 条转移、32 次更新，月表/洞穴分别 64/64 条转移、1024/1024 次 replay 采样。CPU/CUDA 随机状态恢复一致，并完成最终检查点保存。证据 `native-smoke-verification.json`、`native-smoke-telemetry.jsonl`、`build.log`。
- 短训练计入新实验，不回滚学习状态；保存点为 101,431 条转移、25,101 次更新。正式训练继续完成当前全局训练块，再按原固定场景评测。
- 新实验持续记录 `managed/curves/{learning,fixed,evaluation,reward}.csv/png`。比例执行验证不能证明策略改善，后续需比较同图评测成功率、覆盖率和循环情况。

上述证据均位于新实验父目录。正式启动门禁绑定最终干净提交、配置和课程计划哈希；原实验不覆盖、不自动重启。
