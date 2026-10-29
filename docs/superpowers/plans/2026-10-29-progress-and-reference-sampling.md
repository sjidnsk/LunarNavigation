# 进展锚点与参考一致MPPI顺序修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 修正已实测的物理进展丢失，再改善标准Path MPPI对合理运动的候选覆盖。

**Architecture:** 复用现有_last_pose及原先未参与MPPI进展计算的_best_progress标量；将标量明确命名为_translation_credit_m，记录方向重绑前未达阈值的有向位移，不夹断负值。参考修订自身不清时钟，不制造位移。第二切片只在跟踪器内由现有Path车体yaw与弧长生成一致运动初始化/采样，继续使用现有评分、限幅和完整认证；具体形式在第一切片验证后定。

**Tech Stack:** Python/NumPy、现有MPPI和标准Path控制器、ROS2 Humble隔离组件。

**Spec:** docs/validation/2026-10-29-terminal-sampling-progress-diagnosis.md；用户本轮明确授权上述顺序。

## Global Constraints

- nav1cm + 局部MPCC + 标准Path MPPI；不训练、不改模型/config/原安装。
- 不新增领域实体或消息；全局、局部规划和执行认证职责不变。
- 3cm/0.01rad进展门槛、15s缺省超时、到达容差与有序屏障保持；阶段尾部语义另行讨论。
- 原任务/分支hk/drl-integration-20261029/工作树复用，基线83426f8；owner主线公共跟踪，推理验收owner dev/drl-exploration；单写入者。

## Review Focus

- 静止时刷新参考或换方向不可清时钟；负向位移与往返应抵消，不能取绝对值。
- 真实阶段变化只重定向后续运动度量，保留已发生但未结算的位移。
- 新任务/clear不得继承旧credit；终端角进展保持原语义。
- 采样必须遵守参考方向、运动模型、速度/加速度与原认证；不对加权结果旁路。
- 不使用本机纯测推断Humble/Orin就绪，固定Actor多目标结果独立记录。

## Task 1：进展生命周期

- [x] 增加累计小位移跨多次重绑、往返、静止重绑、clear/新session及真实r16顺序重放反例；先观察RED。
- [x] 归并有向位移结算：重绑前保留credit，实际阈值达成后同时清credit和时钟；更新所有相同状态复位点。
- [x] 运行完整纯控制器与相关反馈测试；在r16原始位姿与参考接管序列重放检查进展不再丢失；独立提交切片。

## Task 2：参考一致初始化/采样

- [x] 用r15/r16原评分帧确定几何派生候选，先做前进、倒车、原地转、曲率模式和障碍认证回归RED。
- [x] 由Path位置/车体yaw随弧长的变化生成全时域seed，替代原固定seed候选及空warm start；保留原Gaussian扰动分布和importance修正；不堆固定速度补丁，不改优化权重或代价。
- [x] 精确评分帧回放与原地图认证；纯控制器、受影响Humble包构建/测试，固定Actor有界多目标复核。严格闭环仍按原门槛判定。
- [x] 更新验证报告/任务登记并本地显式提交；不合入长期线、不push/PR。

Ruling: 首版加入的预测域航向反馈衰减导致既有短预测窗边走边转回归失败，恢复原恒定1.5反馈；这是保持已有行为约束，不调整权重或引入新参数。仅参考几何前馈和速度/yaw耦合是本轮新增。

Ruling: 保留原扰动分布，避免另改proposal分布却不匹配importance修正。参考序列占用原seed候选槽，空warm start以该序列初始化；既有warm start仍保留在原候选槽0，所有候选共用原加权与认证。
