# 前沿探索适配实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 已有常规前沿探索接入当前 P4 连续 MPCC 导航，统一等待、终态与人工接管语义。
**Architecture:** 探索只选目标并消费当前 NavigateToPose；导航负责运动与完成。复用现有20cm探索地图、既有UUID/代际和任务命令。
**Tech Stack:** Python、C++、pytest、ROS2 Humble、CTest。
**Spec:** docs/superpowers/specs/2026-11-01-frontier-navigation-adaptation.md（用户2026-11-01明确“按方案逐批实施、每批复审”）。

## Global Constraints

- EXPLORATION-CONTINUITY-20261101；owner integration/pure-planner-orin；复用hk/global-route-terminal-20261101、D:/CodexDownloads/lunar_navigation/global-route-terminal-20261101，基线47c3c2e。实现按批单写入，复审只读。
- P3/Env_X不改，只写远端/home/yanfa/P4；当前运行不切换，不自动发车，不训练，不实施共享基础地图。
- 平台.2m/s、.1rad/s，持令1.2s、全局15s、地图20cm、默认策略none；不加新进展累计、固定等待门槛或ROS实体。
- 每批RED/GREEN、适用测试、独立复审后显式路径提交；最终全部审查；不merge/push/删除工作树或产物。

## Review Focus

- 默认启动不得自发选点；显式frontier也须等待区域START。
- 等待地图/定位/路线不得取消活动目标或污染地点失败记忆。
- 人工RViz替换旧探索目标时，旧回调不得覆盖人工目标或立即争抢。
- 观察朝向及成功必须来自当前Action，不能从局部路径长度或位置再推断。
- 独立域验证不能连接车辆命令话题；不能把包测试写成实车闭环通过。

### 批1：显式探索入口

Files: scripts/simulation/p4_tcp_runtime.py；tests/test_p4_tcp_runtime.py；必要运行说明。
Interfaces: launch_settings(..., *, prefix=PREFIX, goal_topic='/goal_pose', exploration_strategy='none')；命令行--exploration-strategy none|frontier。参数只组合现有节点，不生成START/目标，不改变默认。
- [ ] 新测试：frontier透传、默认none、非法策略拒绝、main显式CLI传给launch_settings；保留后端mpcc_cmd_vel与所有能力参数。
- [ ] 运行测试确认RED；最小实现并跑tests/test_p4_tcp_runtime.py和tests/launch/test_nav1cm_exploration_launch.py。
- [ ] 独立复审并提交。

### 批2：等待与终态

Files: navigation_client.hpp/.cpp；incremental_exploration_node.hpp/.cpp；launch/nav1cm_exploration.launch.py；对应test_navigation_client.cpp、test_incremental_exploration_node.cpp、tests/launch/test_nav1cm_exploration_launch.py；清理已失效参数的必要文档。
Interfaces: 既有NavigationFeedbackState增加定位等待语义；现有Action结果与任务消息布局不改。目标保留至导航真实终态或人工取消/替换。
- [ ] RED：定位等待映射正确、长时间等待无取消且失败计数不增、恢复沿当前目标执行；GOAL_REPLACED暂停区域任务直到RESUME，旧回调不争抢。
- [ ] 删除navigation_map_wait_timeout_s及其专用字段/定时器/累计/取消后地点失败分支；不只改默认、不改名保留。逐一核对调用来源同步清理。
- [ ] 补齐定位等待状态。保留真实NO_PATH/INVALID_GOAL候选记忆，TIMEOUT若为通用执行故障不能无证据标地点不可达；人决定恢复或结束，区域任务保留。
- [ ] 更新回归，以当前Action成功驱动一次下一目标，不添加完成判断。
- [ ] 隔离Humble构建/测试受影响C++包，独立复审并提交。

### 批3：链路集成与交付

Files: 对应探索Action集成测试、必要隔离runner；docs/validation/2026-11-01-frontier-navigation-adaptation.md、任务登记。
- [ ] 在已实现组件上验证选点→Action→当前导航接口→成功后下一点，等待恢复、暂停/恢复/取消、旧代际、人工替换、观察朝向。
- [ ] Windows相关入口/算法回归；Humble隔离域真实C++ Action客户端与导航组件集成，速度输出限定隔离话题；必要构建只放P4新目录。机器不可达时保留可运行脚本并明确NOT_RUN，不宣称完成设备验证。
- [ ] 确认已部署P4/P3/Env_X进程未改变；源码及安装哈希、原始失败证据记录。
- [ ] 全分支独立复审；任务状态已提交待合入/待部署，显式提交文档；不自动切换运行或启动探索。
