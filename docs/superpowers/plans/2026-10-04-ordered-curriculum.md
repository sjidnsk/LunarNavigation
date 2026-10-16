# Ordered Curriculum Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans or superpowers:subagent-driven-development.

**Goal:** 落实用户确认的四阶段课程，并恢复可监看、可恢复的训练。
**Architecture:** 配置启用课程；起点过滤与replay按静态任务标签分层；learner持有完成转移配额；独立controller持有评测门槛和阶段日志。沿用旧controller基础所有权、曲线与安全停止。
**Tech Stack:** Python、NumPy、PyTorch SAC、隔离 ROS2 Jazzy。
**Spec:** docs/drl/2026-10-04-ordered-curriculum-design.md

## Global Constraints
- 不改导航、奖励、模型输入、位置辅助损失或地图面积预算。
- 非课程配置保持旧行为；旧实验结果不得覆盖。
- 一个 learner；阶段继承完整学习状态；从零新实验。

## Review Focus
- 起点模式找不到合法点时必须报错。
- replay快照裁剪丢掉旧类时不得静默宣称80/20。
- 旧长回合和在途动作不得导致按回合的比例偏差。
- 控制器在导出、评测、升级各个边界恢复必须幂等。
- 全局累计统计不得从滚动metrics重新拼接。

### Task 1: 起点与配置
Files: config.py、scene.py、ros_env.py、test_ordered_start.py。
Produces: TrainingConfig.ordered_curriculum: bool=False; ordered_rehearsal_fraction: float=.2; ordered_budget_rule: dict|None=None。Scene.initial_pose(terrain, start_mode='any')。RosExplorationEnv.reset(..., start_mode='any', curriculum_task=None)，descriptor保存start_mode和curriculum_task。resume_semantics课程启用时纳入规则及版本；禁用保持原语义。
- [ ] 写测试并观察失败：区内/外合法点、确定性、非法mode、无合法点、配置/恢复语义。
- [ ] 实现，运行测试至通过。

### Task 2: replay分层
Files: replay.py、test_ordered_replay.py。
Consumes: scene.generator_descriptor['curriculum_task']。
Produces: sample_stratified(batch_size,rng,weights)->list[Transition]；sample_sequences(...,weights=None)；task_counts property；样本标签 helper transition_task(transition, scenes)->str。
- [ ] RED测试：80/20批量、旧类等分、缺失类明确失败、快照恢复、淘汰后索引有效、多步不跨回合。
- [ ] 实现高效任务索引，计入字节预算；快照保留每个活跃任务样本，若容不下明确报错。
- [ ] GREEN并回归原replay测试。

### Task 3: learner课程与持久统计
Files: training.py、新ordered_curriculum.py、test_ordered_curriculum.py。
Produces: TASKS tuple上述四标签；task_weights(phase,rehearsal=.2)->dict；课程运行phase通过DRL_COURSE_PHASE整数0..3注入，仅可同阶段或上一阶段+1恢复。Curriculum.next选择按累计已完成转移比例最欠配任务；每槽active任务，每次完成Admission计数；GRANT也按欠配任务控制而非仅reset。run_training调用stratified及记录统计。
Statistics: status_record['course']={phase, task, weights, by_task:{task:{transitions,samples,ended,successes,episode_steps}}}；持久course-summary.json和counters，受中断回合不记ended。
- [ ] RED测试：首阶段单任务、任务顺序、完成转移而非回合分配、8192配额、64样本近80/20且累计比例、恢复统计、非法跳阶段。
- [ ] GREEN并回归bounded训练和identity测试。

### Task 4: 阶段控制、监看与交付
Files: 新scripts/drl/task_frontier/course_controller.py、course_dashboard.py；eval_case.py、curves.py；课程计划和文档。
- [ ] RED测试：连续两轮门槛、循环阈值、旧任务成功率、阶段上限停止、固定场景身份及恢复。
- [ ] Implement独立controller，重用所有权/安全停止，最多10评测场景；冻结Actor。补start_mode缓存/配对身份，兼容any旧评测；曲线兼容课程结果。
- [ ] GREEN全包测试、真实Jazzy GPU小规模训练/恢复/单case评测、独立审阅。
- [ ] 提交明确路径；gate固定当前commit/config，启动课程第一阶段；打开终端、Codex浏览器曲线，报告真实启动证据。
