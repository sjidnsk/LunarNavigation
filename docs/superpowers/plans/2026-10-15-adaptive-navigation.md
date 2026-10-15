# Adaptive Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现独立于探索的自适应全局导航，在复杂和大范围单层场景中与现用算法进行同条件对比。
**Architecture:** 独立地图构建器输出只读四叉树快照，导航搜索读取快照，真实FREE区域邻接，全局A*，细路径验证，增量tile缓存；旧全局规划原样保留。
**Tech Stack:** C++20, ROS2 Humble, ament/gtest, Release CMake, Python实验汇总。
**Spec:** docs/superpowers/specs/2026-10-15-adaptive-navigation-design.md

## Global Constraints
无任务区域/前沿/收益输入；单层地形；模型/回放/训练参数和安装库不动；500ms主全局预算；ROS原局部/车辆参数不动；不合并、不推送、不部署车辆。用户明确要求在当前会话先实现后实验，按授权直接执行，不再增加审批。

## Review Focus
- 自适应叶边界和对角连接不能漏掉唯一通道或穿障碍角。
- 稀疏大范围不能分配整个包围矩形，超时覆盖冷构图和展开。
- UNKNOWN不能被粗化成FREE；地图不足不宣称物理无路。
- 变化tile/原点/profile使安全缓存正确失效，新捷径重新查询。
- 全局路线可细图连通但车辆仍不可执行时，局部失败必须保留原因和原目标。

### Task 1: Core planner and regression tests
**Files:** include/lunar_incremental_navigation_core/adaptive_navigation_map.hpp; src/map/adaptive_navigation_map.cpp; include/lunar_incremental_navigation_core/adaptive_route_planner.hpp; src/global/adaptive_route_planner.cpp; test/adaptive_route_planner_test.cpp; CMakeLists.txt.
**Interfaces:** AdaptiveNavigationMapBuilder::Update(fine,deadline,stop) -> immutable snapshot; AdaptiveRoutePlanner::Plan(const AdaptiveNavigationSnapshot&,Point2,Point2,SearchDeadline,const StopToken&) -> AdaptiveRouteResult; AdaptiveRouteResult contains GlobalRouteResult route and build/cache/graph metrics.
- [x] 写真实回归：U形矩形外绕行、不同FREE分量、禁止切角、unknown不走、闭合断路、连续端点、负坐标、terrain风险绕行、超时取消、tile替换/新捷径、开阔压缩。
- [x] 编译运行，确认新功能缺失导致RED；实现后逐项GREEN。
- [x] core全套串行CTest通过，保留旧global测试。

### Task 2: Navigation integration
**Files:** planning_session_coordinator.hpp/.cpp; ROS incremental_navigation_node.cpp; coordinator/node tests.
**Interfaces:** 新增可选plan_adaptive_global回调；legacy原入口保留；global_planner_mode选择。
- [x] 回归证明fine规划读取同版本fine、失败/超时不退化成乐观全局路线、目标不变，旧端口仍兼容。
- [x] 接入mode及诊断，保持所有局部参数和预算。
- [x] 构建core与ROS并运行受影响测试。

### Task 3: Reproducible comparison runner
**Files:** benchmark/adaptive_navigation_benchmark.cpp; scripts/navigation/compare_navigation.py; benchmark tests.
**Interfaces:** runner导出逐次JSONL，Python汇总报告/图，种子和地图哈希可复现。
- [x] 检查对比双方同快照、同预算、独立路径验证、失败不混入成功质量均值。
- [x] 实现场景和cold/warm/update对比，执行完整矩阵，记录RSS/路径/构图成本及超时。
- [x] 分析退化，必要修正需新增RED→GREEN测试；重跑受影响矩阵。

### Task 4: Native navigation and final review
**Files:** 隔离运行脚本和验证文档，输出仓库外analysis-artifacts。
- [x] 固定目标真实ROS对比，包含原洞穴失败状态；验证路径通过而不是仅有全局路线。
- [x] 最终独立代码审查，修复重要发现并回归。
- [x] git diff --check、正式结果汇总、明确Humble与Orin/实车证据边界。交付源码、原始结果、报告，不自动续训。

## 审查与修正记录

独立代码审查已完成，重要问题均以回归修复：地图等待总期限不续期；无新里程计也能在地图就绪/期限到达后推进；极端有限坐标不能越界转换；UNKNOWN与BLOCKED不能合并成同一种叶；同叶实际代价回到原细图积分。对比脚本同时修复障碍移除时未恢复原代价及变更地图哈希。

完整矩阵进一步发现2km密集岩石图的区域A*会在直达通道上过量展开。新增RED用例后实现连续细格直达验证：只有整条线FREE且地形附加代价为零时才直接返回，达到欧氏代价下界；有风险/障碍/未知仍进入区域搜索。最终库重新冻结，完整矩阵与原生闭环均重跑；原轮次保留为before-direct供追溯。

## 最终结果与保留项

150组/6300条最终冻结记录：总预算口径新算法878条安全路线、168次NO_ROUTE、4次超时；搜索预算口径880条安全路线、168次NO_ROUTE、2次超时，无返回路线复核失败。超时集中于两张2km密集岩石图（首次构图，以及新增障碍后的搜索）；密集图缓存约891MiB，尚非所有场景达标。

原生Humble闭环12次：新算法6/6到达，原算法4/6到达，均无碰撞。cave004使用原地形/原卡住位姿/原目标，输入改为完整已知地图，不宣称复现原部分观测探索。所有新算法成功同时有PLAN_FOUND/ACTIVE、真实PathReference及位置朝向容差证据。

长墙全局首条路线152.46m，实际行驶215.78m。原滚动局部目标评分存在跨墙投影导致折返的效率问题，保留源码与数值分析，未在本轮只替换全局算法的实验中改动局部算法。

Ruling: 保留独立分支与全部未提交修改，不执行分支集成菜单、合并、推送或删除 — AGENTS明确未经用户要求不得执行集成与清理，本次授权为实现与对比 — 后续集成仍需用户指定。

报告与原始数据：/mnt/data/WS/lunar-navigation/analysis-artifacts/adaptive-navigation-20261015/report.md。

最终补充：重规划等待无新里程计的挂起也已RED→GREEN修复；最终ROS包13/13通过，六组闭环再次全部执行，所有运行使用当前私有ROS库，Core库SHA与冻结矩阵一致。final-verification.json记录最终核对结果。
