# 四阶段课程训练与监看

用户确认的顺序是：任务区内起点月表、任务区内起点洞穴、任务区外起点月表、任务区外起点洞穴。四阶段共享一个 SAC，阶段间完整恢复；此新实验从零初始化，旧混合训练（24,152转移/5,781更新）和26856基线保留在原输出目录。导航、前沿收益输入、奖励、位置辅助损失均不改变。起点约束不会限制机器人后续跨越任务边界。

## 配置与门槛

源码配置 `config/drl_ordered_curriculum.yaml`。输出 `/home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-20261004/managed`。独立的 `course-plan.json` 固定评测种子、任务顺序、面积预算和晋级门槛，并由 review gate SHA 与 course-state.plan_digest 双重绑定。

保持随机50–100m地图；决策按原面积插值：50m×50m为256次，100m×100m为512次，中间按面积插值并32次取整。不计算可探索面积。首阶段纯月表区内；后续新增转移目标当前任务80%、已学任务合计20%，旧任务之间等分。learner按完成转移和在途动作控制GRANT；每个回合标签冻结。64条batch最大余数配额为51/13，三类旧任务分摊13条，终端展示实际比例；整数取整不会被报告为精确80%。仅尚未首次收集当前新任务时允许按可用旧类bootstrap；已学旧类或曾出现的当前类丢失，明确报错。快照裁剪保留各课程类代表样本，预算容不下则报错。

预热1024转移。每块新增8192转移、2048更新后冻结Actor评测。当前任务5张固定图；后续另评测5张旧任务图，按已学任务轮换分配，每轮最多10张。当前任务至少4/5无碰撞达到99%覆盖、两点循环步数合计占比<=20%、旧任务池整体成功率>=80%，连续两轮通过后晋级。**旧池整体80%不代表每个旧类均80%**，逐场景/标签结果保留。默认月表阶段上限20块、洞穴10块，达到上限仍未通过时 `needs_attention`，保存停止，不强行晋级。参数作为工程默认，不代表验证最优值。

## 运行

在已提交且通过 review-gate 的专项工作树中：

```bash
export DRL_CACHE=/home/kai/.cache/lunar-drl-task-frontier/jazzy-task8-20261004
bash scripts/drl/task_frontier/run.sh python scripts/drl/task_frontier/course_controller.py start --root /home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-20261004/managed
bash scripts/drl/task_frontier/run.sh python scripts/drl/task_frontier/course_controller.py monitor --root /home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-20261004/managed
```

监看按起点任务显示每场景覆盖率、回合决策上限、成功/结束回合、累计转移、本阶段转移与采样占比、平均有效结束回合步数。中断/在途回合不计结束，已完成转移保留。阶段切换未收到新统计时显示等待，不贴上旧比例。

平均最终覆盖率按有效结束回合等权计算，包含成功、失败和预算截断；中断和在途场景不计入。监看显示当前任务累计平均、最近20个已记录结束回合均值及实际样本数，各任务表行也标出均值样本数。任务切换后分别保留历史，不将不同任务混为一个提升指标。旧检查点仅从仍保留的terminal replay记录回填，缺失历史明确显示，不补零或推算。当前首次迁移6个结束回合只能核对2个，缺4个；后续新结束回合完整累计并随检查点恢复。趋势应结合成功率和固定评测判断。

`q`只关闭监看；`Ctrl+C`通过owned controller请求保存退出。恢复使用同一命令将 `start` 改为 `resume`；不得直接改 config/course-plan 继续恢复。阶段日志原子保存，评测缓存核验Actor、起点模式、任务标签、场景/参考/运行语义；保存前中断重做同块，保存后恢复下一块。

## 曲线

`curves/learning.csv/png`：学习损失、温度和策略诊断；`fixed.csv/png`：同实测状态中的Q/策略；`evaluation.csv/png`：完整冻结评测，按四类任务分别画线；`reward.csv/png`：奖励分项。`course.csv`保存四任务分配和结束统计，以及覆盖率样本数、总和、累计均值、最近20均值与样本数、历史缺失数；无样本均值为空。学习/奖励随训练更新，固定/评测每块完成后新增点，不能从预热期间空学习图推断停止。

本地浏览器服务：

```bash
python3 scripts/drl/task_frontier/course_browser.py --root /home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-20261004/managed --url-file /home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-20261004/browser-url.txt
```

打印localhost URL，在Codex内置浏览器打开；15秒检查一次图片更新时间。

## 验证边界

课程配额、严格起点、分层replay、裁剪与随机状态恢复、门槛及缓存均有测试。全包904 passed / 2 skipped；之后补充审阅修复的受影响模块回归125 passed。真实本机Jazzy GPU smoke使用正式128/8/6模型、batch64和8个环境，仅预热缩为8并将smoke场景限定50m/16决策；两阶段32+32转移、6+8更新，阶段2新增洞穴26/月表6，恢复完整状态。单case16决策评测用于接线验证，覆盖14.74%，未达到99%，不能当作训练效果。固定20组起点分类组合和随机20个洞穴区外起点合法，不证明所有随机地图均可用。其他部署平台未运行。

详细证据在实验根的 `native-smoke/报告.md`、`native-smoke/report.json`、`independent-review.md`、`review-learner.md` 和 `start-feasibility.json`。正式启动、保存恢复及进程状态证据在 `launch-evidence.json`。

2026-10-04覆盖率监看增量：新增8项测试，受影响模块回归93 passed。独立审阅两项Minor（旧任务样本标注、短窗口总和一致性）均已修复。恢复验证使用正式配置短程16条转移，详细证据见实验根 `coverage-smoke-telemetry.jsonl` 和 `coverage-monitor-review.md`。

## 30–100米训练分支

用户于2026-10-04要求加入30–50米简单场景，提高收集成功样本的机会。新输出为 `/home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-30-100-20261004/managed`，原50–100米实验保存暂停。训练边长均匀采样30–100米；30–50米沿用256决策，50–100米保持原面积插值到512决策。固定评测仍使用原50–100米5张图，课程晋级门槛不变。完成奖励仍为5，普通奖励不变。

`ordered-extent` 显式fork仅允许有序课程将训练范围从50–100米扩大到30–100米，保留原模型、目标网络、优化器、replay、更新信用、RNG、课程计数和监看覆盖率历史，并记录来源与配置差异；普通resume仍拒绝场景范围变化。新目录课程进度接续未完成训练块，曲线沿用已记录历史，配置切换边界在fork证据中记录。新旧训练覆盖率分布不同，均值抬升不能单独证明学到更多能力；固定50–100米评测用于判断效果。

本次范围调整验证：新fork18项测试先RED后GREEN；原fork/CLI回归80 passed；主代理受影响模块148 passed。真实正式配置短程16条转移、5次CUDA更新（含恢复信用1），从8588/1890恢复到8604/1895，包含小于50米的真实场景；保存与RNG恢复均验证。来源状态逐项相等及原文件SHA不变见新实验根 `extent-fork-evidence.json`，native运行见 `native-smoke-report.json` 与 `native-smoke-telemetry.jsonl`，独立审阅无待修复项见 `independent-review.md`。
