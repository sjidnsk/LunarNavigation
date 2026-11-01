# NAV1CM全局参考整形 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变全局搜索/MPCC/控制器职责的前提下提供安全、可回退的平滑全局参考，并在216验证。
**Architecture:** 纯几何reference_smoothing模块；coordinator接收/复核；ROS双路线发布与RViz接线。所有执行路径仍通过原局部认证和执行合同。
**Tech Stack:** Python/NumPy、ROS2 Humble ARM、既有acados MPCC、RViz。
**Spec:** docs/superpowers/specs/2026-10-30-nav1cm-reference-smoothing.md

## Global Constraints
- 预算25ms、采样间距0.05m、裁切0.75m且不超过邻段1/4，最多4096点；整形失败回退已验证原Route。
- 不修改MPCC/控制器、安全阈值、任务/参考身份合同，不自动合并/push；216控制器保持启用，P3/Env_X只读。

## Review Focus
- 曲线从障碍角切入、数值触边、采样点之间穿障：连续凸包证明，触边拒绝。
- 新地图仅碰撞曲线而原折线仍安全：证据复核失效退休参考，不能沿旧局部缓存继续。
- 等价周期重算和几何切换：raw比较稳定保留，真实新版本重新投影进度。
- 极短边/重复点/反向折返/预算/顶点耗尽：有界回退、保持端点、无NaN/零长边。
- ROS实际MPCC输入与RViz平滑显示一致；取消/等待/目标替换不会残留旧图形。

### Task 1: 几何整形（独立agent工作树）
- [x] tests/navigation/one_cm/test_reference_smoothing.py先失败，覆盖安全/回退/连续性/版本/预算。
- [x] 新建scripts/navigation/one_cm/nav1cm/reference_smoothing.py，不改搜索/动力学；接口shape_route与reference_clear。
- [x] 运行纯几何回归、独立审查、显式提交。

### Task 2: 参考链路与ROS（主工作树）
- [x] coordinator/async/session/ros_conversion回归先失败；接入raw_route和shape证据，当前图重验/版本退休。
- [x] ROS node/runtime发布raw与actual global reference；参数显式启用、真实ROS回归。
- [x] Windows适用测试、216候选完整相关包构建/测试、独立整体审查。

### Task 3: 部署/RViz与收尾
- [ ] 记录216原文件、PID、参数及当前目标；仅精确替换受影响文件并重启planner，控制器PID不变。
- [ ] RViz接入青色全局参考/黄色局部，核对正式PLAN_FOUND、话题非空、整形诊断与实际局部参考。
- [ ] 更新任务登记、验证报告、提交；源码/ARM/ROS/运行视觉和完整导航闭环证据分别记录。

范围更新：用户授权TCP统一最终生产部署/重启/车辆复测；Task3改为精确候选交付。已交付产品2338d7b、manifest和overlay；生产/RViz/车辆步骤不由本会话执行，待TCP验收。不要把未勾选的现场步骤标成通过。参考验证报告。
