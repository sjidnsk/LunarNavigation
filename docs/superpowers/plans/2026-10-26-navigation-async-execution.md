# 异步规划与延迟闭环 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 依次完成异步局部规划/执行、峰值定位、延迟运动闭环及隔离Humble节点验证。

**Architecture:** 独立局部worker只产候选；执行端独占时间化轨迹缓冲，重新接续与认证；ROS适配不进入算法核心。现有同步入口保留作基线。

**Tech Stack:** Python multiprocessing、NumPy/SciPy/native distance field、acados、ROS2 Humble rclpy/ament_python。

**Spec:** docs/superpowers/specs/2026-10-26-navigation-async-execution-design.md

## Global Constraints

- 继续NAV-1CM-20261026与hk/nav-1cm-feasibility-20261026；无提交、合并、推送、删除，无P3/起步盲区/实车。
- 5m×5m、500²、1cm严格车体中心；原始UNKNOWN和可配置通行策略保留。
- 正式运动档.5m/s、.5rad/s、a/alpha=.5，车体1.182×.818m、行进曲率≤1；动作必须经过当前图与反馈认证，含连续扫掠与制动尾段。
- 默认执行20Hz、局部请求10Hz、全局1Hz；所有worker单在途，不堆积旧请求；超周期只统计。
- 旧同步代码兼容；各子任务实现串行，独立审查；产物位于任务D盘或P4_veri隔离目录。
- 地图≤100m，实际沿用25m短路线；不以冻结状态或时钟证明延迟安全，不把较低层结果冒充ROS/实车。

## Review Focus

- 迟到轨迹接入时控制年龄、反馈偏差、重积分和新证书是否完整。
- 地图/目标/路线变化，旧结果是否可能覆盖新状态；旧缓冲剩余段是否继续安全。
- worker繁忙是否阻塞执行，进程异常/关闭是否有界；轨迹到期是否错误外推。
- 峰值计时是否区分重置/装填/solve与CPU/墙钟，是否存在重叠相加或未经证明归因。
- 延迟测试是否真正推进车辆；ROS消息与定时器是否独立、域和话题是否隔离。

### Task 1: 异步局部worker与执行缓冲

**Files:** 新增 scripts/navigation/one_cm/nav1cm/{local_worker,trajectory_buffer,async_coordinator}.py；可按职责需要增加一个共享纯Python协议模块。修改execution.py仅补公共切片/重积分能力且保持原接口。新增tests/navigation/one_cm/test_async_execution.py、test_local_worker.py。

**Interfaces:** AsyncNavigationCoordinator(..., local_worker=..., global_runner=..., planning_period_s=.1,execution_period_s=.05).tick(world,state,goal,now_s)，返回动作、当前grid、动作认证、事件及计时；字段尽量兼容CycleResult。LocalWorker.submit/poll/busy/close；worker进程初始化一次真实MPCC。TrajectoryBuffer管理时间年龄、当前反馈重积分、候选接入和剩余时域；方法由实现者在报告固定供Task3/4使用。

- [x] RED：局部worker忙时执行旧轨迹、100/150/250ms迟到接续、地图/目标/版本陈旧拒收、动作耗尽、状态偏差与不安全接续拒绝、不能超时单独制动。
- [x] 实现独立worker与协议、轨迹缓冲及编排；所有实际动作重新认证，受限停车保持物理推进；不重写碰撞/动力学数学。
- [x] 纯Python测试用确定runner，补可用时真实进程单在途/关闭测试；现有接口回归。
- [x] 报告实际API、容差、启动/关闭、来源及计时契约；不得启动正式长批次。

### Task 2: 拆分求解器计时与峰值短重放

**Files:** local_mpcc.py；新增scripts/navigation/one_cm/benchmark_solver_timing.py；tests/navigation/one_cm/test_local_mpcc.py或独立test_solver_timing.py。Task1必要的worker透传只做小幅兼容。

**Interfaces:** LocalPlanResult保留candidate/reason/timings_ms/attempts；新增细分wall/cpu及真实get_stats诊断，明确旧solve字段包含关系。

- [x] 先检查固定acados源码支持字段，RED/GREEN保证reset、input、solve_call、read/validate计时不重叠，缺诊断不影响候选。
- [x] 保存136ms峰值原始输入并做有界重放，记录正常/峰值是否复现、wall/cpu差别和迭代；不删慢样本。
- [x] 只对测量确认的热点做必要修正；未定位不得盲改reset/求解算法或约束。输出短报告供下一阶段采用。

### Task 3: 有界延迟闭环与证据

**Files:** 新增scripts/navigation/one_cm/benchmark_async_navigation.py及必要纯fixture/runner；新增tests/navigation/one_cm/test_async_benchmark.py；不把场景分支写进算法。

**Interfaces:** 消费Task1固定API；支持真实localworker、100/150/250ms注入、逻辑执行时间和wall节拍模式，输出完整JSONL/summary/SVG与源码/库身份。

- [x] 先RED/GREEN覆盖延迟期间仍推进、旧结果不能篡改反馈、目标拒绝优先、时钟/年龄和缓冲状态记录。
- [x] 在原多弯与增障做短有界回归；每动作独立认证与公开动力学重放，报告到达/安全制动/失败、接续偏差、控制节拍、latency和buffer余量。
- [x] 主agent统一在Orin跑正式样本，单机不并行性能负载；不得隐匿失败或暂停模拟时钟等待worker。

### Task 4: 隔离Humble节点与探针

**Files:** 新增实验内ros2包（建议scripts/navigation/one_cm/ros2/lunar_nav1cm_verification）及独立启动/探针脚本、tests/navigation/one_cm/test_ros_adapter_contract.py；README补运行指令。不得修改生产节点/launch/消息。

**Interfaces:** 仅适配Task1公开API；隔离ROS_DOMAIN_ID及/nav1cm_verification/*；地图/状态/目标输入和动作/诊断消息契约由实现者先写明再实现。合成反馈模型与导航回调应能独立运行，避免worker延迟冻结反馈。

- [x] 静态测试依赖、topic隔离、模型参数、时间戳与转换契约；算法包不依赖ROS。
- [x] ament_python包、launch和探针只做隔离验证；独立build/install/log，不依赖生产overlay。
- [x] 主agent在Humble构建与包测试，再跑真实节点/DDS、定时器和延迟输入探针；收集证据后退出进程。
- [x] 全部完成后统一审查、更新原任务登记与结果；未运行层级列NOT_RUN，资源保留未提交。
