# Course Two Launch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 从已完成课程 block31 启动区域接近与传感器泛化课程，提供终端监看、受控保存退出和完整状态恢复。

**Architecture:** 复用当前 A2 专项及原 SAC/ROS 闭环。新增场景不可变描述、课程二采样状态和受控 fork；独立控制器管理有界训练、检查点、评测门槛和终端操作，原训练路径保持默认兼容。

**Tech Stack:** Python 3.12、PyTorch、ROS 2 Jazzy、现有原生地图/规划/传感器后端。

**Spec:** `/home/kai/WS/lunar-navigation/docs/research/2026-10-10-第二课程设计-区域接近与传感器泛化.md`。

## Global Constraints

- 量程 10–100 m；FOV 60/90/120/180/360 度；一个场景只有一份固定 SensorSpec，重试和恢复不重抽。
- 第一阶段 A1 为 10 m/90 度、量程外间隙 1–5 m；A2 扩展到 1–20 m；保持 20% 原课程复习。
- 保留 Actor、双 Q、目标网络、优化器、温度、兼容 replay、累计计数；禁止从 Actor 导出文件冒充完整续训。
- 原输出及检查点只读；新输出独立；不合并、不推送、不修改其他工作树。
- 原奖励、A2 图/远处观察点、导航、SAC 参数冻结；所有能力声明以实际检查为准。
- A1 先有界 2,048 转移小试，随后按文档区段预算和完整评测推进；未满足门槛保存停止，不自动加预算。
- 用户已明确授权启动、终端监看和退出恢复，本轮直接实施并验证，无需再次询问是否开训。

## Review Focus

- 同场景 reset/恢复意外改变传感器或任务身份：场景哈希、原生观测与参考身份一致性测试。
- 老 replay 或跨区段标签使训练采样死锁或新样本未入 batch：配额与受控 collector 回归。
- 停止发生在更新/采集边界导致计数、信用或优化状态丢失：真实保存/恢复对比及信号测试。
- 并发启动/陈旧 PID 误停其他任务：文件锁、进程启动身份核验测试。
- 初始化已覆盖、零收益和未完成轨迹被算成功：独立计数及完整门槛测试。

### Task 1: Fixed sensor scenes and bounded approach geometry

**Files:** `lunar_drl_exploration/course_two_scene.py`, `ros_env.py`, `test/test_course_two_scene.py`（均在 `ros2_ws/src/lunar_drl_exploration/` 下）。

**Interfaces:** `make_case(seed, stage, ordinal, ...) -> dict` 生成 JSON 场景规格；`build_case(case, platform) -> (terrain, pose, task, sensor, descriptor)` 生成确定性环境；`reset(..., course_two_case=None)` 接受冻结规格；`reset_scene(..., sensor=None)` 贯通同一规格。

- [ ] 写测试：同 case 重建得到相同身份/参数；八方向覆盖、距离上限、360 度；原生可见性与观测上下文匹配。
- [ ] 运行测试并确认新入口缺失导致失败。
- [ ] 实现场景描述和生成，增加传感器贯通及观测/接近预算元数据，保留默认旧 reset。
- [ ] 运行新测试及原 sensor/reference、环境测试，记录通过与未运行边界。

### Task 2: Stateful sampler and full-state fork

**Files:** `course_two.py`, `config.py`, `training.py`, `checkpoint.py`, `cli.py`, `test/test_course_two_training.py`。

**Interfaces:** `CourseTwo(stage, state=None)` 实现当前 OrderedCourse 的调度/计数接口；`TrainingConfig.course_two` 默认 False；`--fork-experiment course-two` 只迁移显式允许的采样和输出字段。

- [ ] 写失败测试：fork 保留学习状态/replay、拒绝顺带改奖励或模型；新阶段采样并恢复 RNG/配额；batch 有新任务占比。
- [ ] 实现课程二独立协议，保存旧课程历史、逐场景规格和新课程统计；不跳过恢复校验。
- [ ] 运行 checkpoint、课程、采样和信号回归；保留课程一兼容性。

### Task 3: Managed launch, terminal and evaluation boundary

**Files:** `course_two_control.py`, `scripts/drl/course_two.sh`, `test/test_course_two_control.py`, `docs/drl/2026-10-10-course-two-operations.md`。

**Interfaces:** shell `start/status/monitor/stop/resume`；控制器持有 root 锁、子进程和状态文件；stop 向所属 learner 发送 SIGINT 并等待保存；resume 从新目录完整检查点继续。

- [ ] 写失败测试：重复启动、PID 身份校验、暂停和剩余预算；门槛不能由部分评测或初始完成冒充通过。
- [ ] 实现启动/监看/退出/恢复和固定评测清单；状态明确区分采集、更新、评测、暂停、失败、待处理及完成。
- [ ] 运行控制器测试并将操作命令与检查点语义写入文档。

### Task 4: Native verification and actual launch

**Files:** 新实验目录 `/home/kai/CodexDownloads/lunar_navigation/drl-course-two-20261010/`；验证记录 `docs/validation/2026-10-10-course-two-launch.md`。

- [ ] 运行受影响完整 Python 套件；原生传感器组合预检（10/40/100 m × 60/90/180/360 度）、短闭环和资源检查。
- [ ] 对真实 block31 做迁移前后状态哈希比较；真实短训练后保存停止、恢复，再验证计数递增和场景传感器一致。
- [ ] 按 executing-plans 要求进行一次独立整体审阅，修复重要发现并复测。
- [ ] 启动 A1 有界小试并打开终端监看；核验 PID、最新指标、检查点、运行配置和剩余预算。
- [ ] 交付终端、停止/恢复命令及状态；不把进程健康宣称为策略改善。

## Execution ledger

- 2026-10-10：复用既有干净的 `feat/drl-task-frontier` 专项工作树，HEAD `6d29042`；这是同一 A2 专项的课程续训，避免为了分支形式复制尚未合回 dev 的 120 个基线文件。没有合并或移动分支。
- 基线测试首次受无关 ROS pytest 插件自动加载影响（缺 lark），改用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 运行本包测试，不修改训练依赖。
- 以本文件记录实现与验证，不删除用户需要审阅的记录；本轮不为实施技能额外要求再次授权。
