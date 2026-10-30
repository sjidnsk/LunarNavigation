# 2厘米位置收敛与10厘米验收分离 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 保留10cm验收上限，按用户确认的2cm目标继续位置收敛，再进入纯旋转。

**Architecture:** 导航owner定义goal_position_target_m，统一position_target_tolerance_m有效值（不得宽于公共goal_tolerance_m）。同步/异步末段切换、制动终点、任务完成和既有ExecutionReference位置容差均使用同一有效值。继续原MPCC与标准Path MPPI，不增消息或身份实体。

**Tech Stack:** Python/NumPy、ROS2 Humble/acados、既有PathExecutionReference。

**Spec:** execution-tests/arrival-boundary-analysis.json：16次首个FINAL_ALIGN的中心误差与最终误差完全一致，后续平移0；用户2026-10-30确认2cm。

## Global Constraints

- 基线9ef6f3e，复用hk/drl-integration-20261029及原D盘工作树；公共导航owner integration/pure-planner-orin，推理验收owner dev/drl-exploration；单写入者。
- 不改全局路线目标、MPCC代价、跟踪代价、3cm/0.01rad进展、超时、停稳/朝向容差或训练。
- 纯协调器缺省target=None保留历史未指定语义；ROS正常入口显式缺省target=.02。若公共验收更严则取二者较小值。
- 复用执行参考原位置容差字段传递2cm，不增消息；跟踪器不得重新决定全局任务目标。

## Review Focus

- 距目标8cm时不能冻结为纯旋转或提前Action成功；两协调器行为一致。
- 2cm内尚有非零平移，仍先认证制动，制动终点不得超出有效目标。
- 更严公共容差不可被内部目标放宽，非法目标应在资源创建时拒绝并清理。
- 无执行侧带的历史入口也不能绕过位置收敛目标提前完成。
- 2cm仅合成场景目标，不能外推实车/坡面/传感器噪声。

- [ ] 同步/异步8cm继续规划、1.5cm末段旋转、制动边界、更严公共容差及任务完成先RED。
- [ ] 实施owner有效位置目标，ROS参数默认2cm并通过已有参考字段传递；包级ROS参数/发布回归先RED后GREEN。
- [ ] 原纯导航/跟踪回归及Humble受影响包构建测试；固定Actor有界多目标真实组件验收，追加2cm质量检查，保留原10cm严格验收。
- [ ] 独立只读审查、结果与基线并列可视化、更新任务登记并本地提交；不移动长期线。

## r19后用户确认的方向修复

用户明确选择按最终姿态选择相容的末段前进/倒车方向。局部MPCC在末段范围内按最终车体朝向在路线方向的投影选择速度符号；模式选择与旋转对准共用同一决策。正交数值死区沿用原偏好；范围外与无终端姿态参考沿用原规则。移动中需换向仍先制动，不改变跟踪器、全局规划或认证职责。

- [x] 实际r19停滞姿态、相容方向、移动换向四项反例先RED，最小实现后通过。
- [x] 补齐正交、跨π与末段距离边界；独立只读审查无阻断。
- [x] 新隔离Humble包571通过，原生定向57通过；实际停滞姿态的受控acados探针生成对准候选（尚待认证，不冒充完整闭环）。
- [ ] r20固定Actor多目标2cm闭环、取消复位及新帧验收；保留r19失败。
