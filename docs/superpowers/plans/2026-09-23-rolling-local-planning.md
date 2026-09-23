# 移动中滚动局部规划 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking. 本任务推荐由当前执行者按依赖顺序实施，执行方式由用户选择。

**Goal:** 在同一轮式导航任务内边行驶边更新局部路径，正常更新不产生额外停车。

**Architecture:** core 协调器串行拥有任务与活动路径，单个后台工作线程只计算候选结果；控制器接收同任务新引用时重新定位路径进度并保持跟踪。复用现有可行性、地图失效、速度限制和版本匹配，保持 ROS 消息不变。

**Tech Stack:** C++20、ROS 2、ament/colcon、GTest、Python/pytest、现有 Pure Pursuit 执行器。

**Spec:** `docs/superpowers/specs/2026-09-23-rolling-local-planning-design.md`，用户已确认。

## Global Constraints

- 不新增规划算法、控制器包、速度发布者、连接段规划器或多阶段路径交接协议；不修改探索策略、底盘协议及生产部署。
- 同一时刻最多一个计算任务。忙碌时只合并一个待更新意图，不排队保存旧位姿；下一次调度重新捕获最新输入。
- 地图修订号变化本身不是拒绝条件。
- 路径更新不清空任务级无进展计时。
- 周期设为 0 表示关闭滚动更新，恢复段结束调度；拒绝负值和非有限值。
- 主要调参只有更新周期和提前距离。控制器启用开关由统一 launch 根据周期派生，不增加第三个调优参数。
- 本任务限轮式 `incremental_reference`；其他输入模式与足式维持原语义。
- 仅在 `feat/rolling-local-planning` 独立工作树实施；不合并、推送或连接真实底盘。所有 artifact 写入仓库外。
- 本机 Jazzy、Humble、Orin、DDS、实车证据分开记录；未执行项为 NOT_RUN。

## Review Focus

1. 高频地图修订且路径未被影响：结果不能持续被版本门槛饿死；任务 2 验证。
2. 失效或取消与后台完成同时发生：旧结果不能复活引用；任务 2、4 验证。
3. 自交路径以及计算完成时起点已落后：不能跳到后续分支或回头追起点；任务 3 验证。
4. 同一路径反复更新但车辆不动：无进展计时必须触发；任务 3 验证。
5. 最终对齐与后台完成同时发生：普通结果不能重启跟踪；任务 2、4 验证。

## 文件分工与验证命令

新建 core `include/lunar_incremental_navigation_core/rolling_update_policy.hpp` 和 `src/session/rolling_update_policy.cpp`：只负责触发节奏。
新建 core `include/lunar_incremental_navigation_core/planning_cycle.hpp` 和 `src/session/planning_cycle.cpp`：不可变请求/候选和单次计算。
修改 core `planning_session_coordinator.hpp`、`src/session/planning_session_coordinator.cpp`：请求准备、结果应用、活动引用所有权。
修改 ROS `src/incremental_navigation_node.cpp`：单工作线程及事件收发，不搬入控制策略。
修改控制器 `execution.py`、`path_geometry.py`、`node.py`：同任务更新与进度连续性。
更新各包 CMakeLists.txt 或已有安装配置以包含新增文件，不引入第三方依赖。

以下命令从工作树根目录在全新 shell 执行。不得叠加其他 ROS 发行版 overlay。

```bash
source /opt/ros/jazzy/setup.bash
export ROLLING_ARTIFACT_ROOT="$HOME/CodexDownloads/lunar_navigation/rolling-local-planning-20260923"
colcon --log-base "$ROLLING_ARTIFACT_ROOT/log" build \
  --base-paths ros2_ws/src \
  --packages-up-to lunar_incremental_navigation_ros lunar_pure_wheeled_controller lunar_incremental_controller_demo \
  --build-base "$ROLLING_ARTIFACT_ROOT/build" \
  --install-base "$ROLLING_ARTIFACT_ROOT/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DLUNAR_BUILD_DEMO=ON
source "$ROLLING_ARTIFACT_ROOT/install/setup.bash"
export PYTHONPATH="$PWD/ros2_ws/src/lunar_pure_wheeled_controller/python:$PYTHONPATH"
```

core 测试入口：

```bash
ctest --test-dir "$ROLLING_ARTIFACT_ROOT/build/lunar_incremental_navigation_core" \
  -R 'rolling_update_policy|planning_session_coordinator' --output-on-failure -j1
```

ROS 测试入口：

```bash
ctest --test-dir "$ROLLING_ARTIFACT_ROOT/build/lunar_incremental_navigation_ros" \
  -R incremental_navigation_node --output-on-failure -j1
```

每个代码任务按红灯→实现→绿灯执行。新增测试的红灯必须来自所需行为缺失，不把环境导入错误当作有效红灯。
先运行现有相关测试保存基线，新增源码后重新运行上面的构建命令再运行 CTest。

## Task 1: 独立的周期与提前触发策略

**Files:** 新建上述 rolling_update_policy 两个文件和 `ros2_ws/src/lunar_incremental_navigation_core/test/rolling_update_policy_test.cpp`；修改 core CMakeLists.txt 注册源文件与 GTest。

**Interfaces:** 新建 `RollingUpdatePolicy(double period_s, double distance_m)`；
`bool ShouldStart(double now_s, uint64_t revision, double remaining_m)`；
`void Started(double now_s, uint64_t revision)`；`void Finished()`；`void SetFrozen(bool)`；`void Reset()`。
调用者只在存在活动引用且任务执行中调用；时间由 core 的单调时钟换算，不能用 ROS 仿真时间驱动超时。

- [x] 写入核心失败测试，包含固定周期、提前一次、忙碌抑制、失败后按周期重试、冻结、关闭、参数错误：

```cpp
TEST(RollingUpdatePolicy, EarlyAttemptDoesNotBusyLoop) {
  RollingUpdatePolicy policy(1.0, 2.0);
  policy.Started(0.0, 1);
  policy.Finished();
  EXPECT_TRUE(policy.ShouldStart(0.2, 2, 1.0));
  policy.Started(0.2, 2);
  EXPECT_FALSE(policy.ShouldStart(0.25, 2, 1.0));
  policy.Finished();
  EXPECT_FALSE(policy.ShouldStart(0.3, 2, 1.0));
  EXPECT_TRUE(policy.ShouldStart(1.3, 2, 1.0));
}
```

- [x] 构建并运行 core 测试入口，确认新增行为测试失败。
- [x] 实现最少字段：上次开始时间、busy、frozen、提前触发过的 revision；周期 0 直接禁用。提前更新只对尚未触发过的当前 revision 生效，Started 记录尝试而非成功。

```cpp
if (period_s_ == 0.0 || busy_ || frozen_) return false;
return now_s - last_started_s_ >= period_s_ ||
       (remaining_m <= distance_m_ && revision != early_revision_);
```

- [x] 运行新增测试和现有协调器测试；确认输入 NaN、负周期、负距离被拒绝，重置后不继承旧任务调度状态。
- [x] 显式暂存上述四个文件，提交 `feat: add bounded rolling update scheduling`。

## Task 2: 拆分候选计算与协调器状态更新

**Files:** 新建 planning_cycle 两个文件；修改协调器头/实现、core CMakeLists.txt 和 `test/planning_session_coordinator_test.cpp`。

**Interfaces:** 将 StateInput、PlanningSessionPorts 放入 planning_cycle.hpp，协调器头包含该文件保持现有包含兼容。
使用既有 FinalGoal、SnapshotBundle、PlatformCapability、TraversabilityProfile、SearchDeadline、StopToken、CycleOutput 类型。
新增类型和函数边界如下，CycleCandidate 不包含可直接发布的活动引用或任务终态：

```cpp
struct CycleRequest {
  uint64_t request_id;
  SessionId session_id;
  StateInput state;
  FinalGoal goal;
  SnapshotBundle snapshots;
  PlatformCapability capability;
  TraversabilityProfile profile;
  double local_window_size_m;
  std::chrono::nanoseconds global_subdeadline;
  SearchDeadline deadline;
  bool rolling;
};
struct CycleCandidate {
  uint64_t request_id;
  SessionId session_id;
  LocalPlanResult local;
  GuidanceStatus guidance_status;
  std::optional<GlobalRoute> global_route;
  std::shared_ptr<const RequestLocalPlanningView> view;
  std::string reason_code;
};
CycleCandidate ComputeCycle(const CycleRequest&, PlanningSessionPorts&,
                            const NowFn&, const StopToken&);
```

新增 `PreparedCycle { optional<CycleRequest> request; CycleOutput immediate; }`。
协调器新增 `PrepareCycle(StateInput, SnapshotBundle, CycleTrigger, SearchDeadline, bool rolling) -> PreparedCycle`、
`ApplyCycle(CycleCandidate, StateInput, SnapshotBundle) -> CycleOutput`、`SetFinalAlignment(bool)`。
保留 PlanCycle 作为同步兼容包装，走同一套 Prepare/Compute/Apply；不能维护两份局部搜索代码。
无计算请求时，现有状态处理仍通过 immediate 输出反馈；不要依靠隐藏消息队列传递即时输出。

- [x] 在现有 StubPorts、MakeFine、State 测试夹具中新增参数化测试：滚动更新的 NoPath/Timeout 不结束仍有效任务；首次规划相同错误保留原终止分类。
- [x] 为取消、失效、最终对齐增加结果晚到测试；关键断言顺序如下：

```cpp
auto prepared = coordinator.PrepareCycle(state, snapshots, trigger, deadline, true);
ASSERT_TRUE(prepared.request);
auto candidate = ComputeCycle(*prepared.request, ports, now, stop);
coordinator.Cancel(CancelReason::kCanceled);
auto applied = coordinator.ApplyCycle(std::move(candidate), state, snapshots);
EXPECT_FALSE(applied.path_reference.has_value());
EXPECT_FALSE(applied.terminal.has_value());
```

其中 state/snapshots/trigger/deadline/ports/now/stop 使用现有测试夹具构造；新增行为断言以完成 Start 和首次 PLAN_FOUND 为前置条件。
- [x] 构建运行 core 测试确认红灯，随后把全局查询、目标选择、起点补丁、局部搜索移入 ComputeCycle；工作线程独占 ports 及缓存，不能捕获协调器可变 Impl。
- [x] ApplyCycle 先按 session/request_id 排除过期结果，再检查当前地图；普通更新失败保留 active_path。只有采用成功才增加 segment_revision。

```cpp
if (!RequestStillCurrent(candidate)) return IgnoredOutput();
if (candidate.local.status != LocalPlanResult::Status::kPlanFound &&
    request_was_rolling && active_path_is_executable) {
  return RetainedPathOutput(candidate.reason_code);
}
```

上面三个内部谓词/输出辅助函数在协调器内实现：RequestStillCurrent 比较 pending 请求身份；IgnoredOutput 不带 path/terminal；RetainedPathOutput 保留引用并输出本周期失败反馈。
- [x] 复用并提取现有路径遍历检查，保留平台/起点补丁语义。增加“无关地图修订可采用”和“新障碍拒绝候选”测试；不能把 revision 相同作为唯一可行性条件。
- [x] 验证失效/取消/最终对齐清除 pending 身份，普通位姿更新不清除；重复结果只应用一次；同步兼容包装通过现有 coordinator 测试。
- [x] 运行 core 全包 CTest，串行执行包含长距离场景的测试。显式暂存本任务文件，提交 `refactor: separate planning candidates from session state`。

## Task 3: 控制器同任务路径更新及末端语义

**Files:** 修改 `ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/{execution,path_geometry,node}.py`；测试 `test/test_execution.py`、`test/test_node.py`。

**Interfaces:** `PathExecutor(policy, *, rolling_updates=False)`；set_path 原签名不变；update(state, dt) 原签名不变。
节点参数 `enable_rolling_updates` 默认 false，仅在 incremental_reference 模式使用。统一 launch 从导航周期是否大于 0 派生此布尔值。

- [x] 写入热更新行为测试和关闭模式兼容测试：

```python
def test_same_session_update_keeps_tracking_while_moving():
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0, 0, 0), (5, 0, 0)), identity=("a", 1), final=False)
    executor.update(TrackingState(0, 0, 0), 0.05)
    executor.update(TrackingState(0.5, 0, 0, linear_mps=0.1), 0.05)
    executor.set_path(((0.4, 0, 0), (6, 0, 0)), identity=("a", 2), final=False)
    result = executor.update(TrackingState(0.6, 0, 0, linear_mps=0.1), 0.05)
    assert result.phase == "TRACKING"
    assert result.command.linear_x_mps > 0
```

- [x] 运行 `python3 -m pytest -q ros2_ws/src/lunar_pure_wheeled_controller/test/test_execution.py` 确认新增行为红灯。
- [x] set_path 仅在滚动启用、同 session、新 revision、原状态 TRACKING 时允许保持跟踪；重新设置路径局部索引，保留任务级无进展和实际速度连续性。不同任务、故障、失效仍走启动恢复。

```python
continue_tracking = (
    self.rolling_updates and self.phase == "TRACKING"
    and self.identity is not None and identity is not None
    and self.identity[0] == identity[0] and identity[1] > self.identity[1]
)
```

- [x] 用实时位姿重投影，按路径点序和行进方向约束自交时的候选；不让投影跳过尚未行驶的回环，不添加直连段。测试直线旧起点落后、折返、自交、过大横向偏差、方向反转。
- [x] 将跨引用无进展证据改为实际位姿运动累计，使用既有进展阈值；小幅往复噪声不能按累计路程无限续命。新增固定位置持续更新、阈值内噪声、正常移动三组测试。
- [x] 非最终末端仅在 rolling_updates=True 时不做最终 yaw；保留 false 模式现有 `test_non_final_reference_still_executes_its_terminal_heading`。
- [x] 回归大角度对齐、换向、最终位置丢失、最终停稳、旧/重复 revision、同 revision 内容变化；运行整个控制器测试目录。
- [x] 显式暂存本任务文件，提交 `feat: track rolling references without automatic braking`。

## Task 4: ROS 单工作线程与事件优先处理

**Files:** 修改导航 `src/incremental_navigation_node.cpp`、`test/incremental_navigation_node_test.cpp`；调度参数加入现有参数读取与构造，不新增 ROS 消息。

**Interfaces:** 消费 Task 1 的 policy 和 Task 2 的 PreparedCycle/ComputeCycle/ApplyCycle；工作线程接收一个请求、返回一个结果，使用 condition_variable 和 stop token，协调器状态只在 planning callback group 访问。

- [x] 增加带条件变量阻塞的 plan_wheel 测试替身；在搜索未放行时分别提交取消、地图阻塞和密集更新；断言最大并发计算为 1、取消/失效无需等待搜索完成、后继请求使用最新位姿。
- [x] 运行 ROS 节点测试确认红灯；替换同步 RunCycle 的计算部分，复用原 PublishPath、终态提交与诊断输出。

```text
PlanningTick:
  先处理取消/抢占、最新地图失效和当前引用的执行反馈
  再处理工作线程返回结果，经 ApplyCycle 决定发布
  最后若 policy 允许且 worker 空闲，用最新输入 PrepareCycle
worker:
  等待单个请求 → ComputeCycle → 将结果交回 → 等待
```

- [x] 新任务、失效和最终对齐淘汰 pending 请求；正常位置/地图变化不反复取消搜索。地图派生线程与规划缓存所有权不能交叉；锁内只交换值，不执行搜索或等待 join。
- [x] 移动时按当前路径长度减 TrackingStatus.progress_m 计算剩余弧长，只有匹配当前 revision 的反馈有效；无匹配反馈时使用 core 中的路径投影结果，不使用到终点直线距离替代弧长。
- [x] 增加最终对齐晚到结果、旧 COMPLETED、旧 FAILED、失效后结果、worker 析构中断等节点测试；使用条件变量同步，不用长期 sleep 制造时序。
- [x] 诊断记录触发原因、耗时、候选采用/保留/淘汰和引用版本；计算数据随结果返回，避免后台改写 node 的 cycle_metrics。
- [x] 运行 ROS 节点测试与 core/控制器回归，显式暂存本任务文件，提交 `feat: compute rolling paths outside navigation event handling`。

## Task 5: 配置、隔离闭环与操作文档

**Files:** 修改 `config/incremental_navigation_interfaces.yaml`（按现有结构记录新参数）；
`ros2_ws/src/lunar_incremental_controller_demo/launch/controller_rviz.launch.py`；
`ros2_ws/src/lunar_incremental_controller_demo/lunar_incremental_controller_demo/{run_case,evidence}.py`；
`ros2_ws/src/lunar_incremental_controller_demo/test/test_evidence.py`；
新建 `tests/launch/test_rolling_local_planning_contract.py`；更新 README.md、docs/操作指令.md；新建验证报告。

- [x] 编写 launch 参数合同测试，固定周期 0 时控制器滚动关闭、正值时启用，非法值被拒绝；导航只在轮式且启用跟踪反馈时接受正周期。
- [x] 在统一 launch 加入两个参数，并派生控制器开关：

```python
period = float(LaunchConfiguration('rolling_replan_period_s').perform(context))
navigator['rolling_replan_period_s'] = period
navigator['rolling_replan_distance_m'] = float(
    LaunchConfiguration('rolling_replan_distance_m').perform(context))
controller['enable_rolling_updates'] = period > 0
```

- [x] 扩展证据记录器保存带本地单调时间的路径版本、执行阶段、位姿/速度和规划诊断；不要只保留去重后的终态。raw 原始数据位于 artifact 目录。
- [x] evidence.py 增加纯函数 `summarize_rolling_events(events)`，输出引用更新次数、移动中更新次数、更新关联停车次数；final 和方向/急转制动单独分类。用以下最小样例测试正常更新不算停车：

```python
def test_moving_update_has_no_stop():
    events = [
        dict(t=0., kind='tracking', revision=1, speed=.1, phase='TRACKING'),
        dict(t=.1, kind='reference', revision=2, final=False),
        dict(t=.2, kind='tracking', revision=2, speed=.1, phase='TRACKING'),
    ]
    result = summarize_rolling_events(events)
    assert result['moving_updates'] == 1
    assert result['update_stops'] == 0
```

- [x] 记录器将消息归一化成上述 event 字典；以匹配 revision 的首次跟踪反馈判断更新是否发生在运动中。额外保存原始时间线，让停顿原因可人工复核，不仅凭统计标签声称无停车。
- [x] 运行接口与 demo 测试：

```bash
python3 -m pytest -q tests/test_incremental_navigation_message_contract.py \
  tests/test_incremental_navigation_interface_contract.py \
  tests/launch/test_rolling_local_planning_contract.py \
  ros2_ws/src/lunar_incremental_controller_demo/test
```

- [x] 更新操作文档：统一启用两端、关闭恢复旧模式、仅隔离域运行、参数如何由实测耗时确定。新增报告先记录待执行项，不填虚构通过结果。
- [x] 显式暂存本任务文件，提交 `feat: expose rolling planning in isolated controller demo`。

## Task 6: 前后对照与最终验证

**Files:** 只补充 `docs/validation/2026-09-23-rolling-local-planning.md` 与受结果影响的操作说明；发现缺陷则先增加对应失败回归，再修复所属模块。

- [x] 重新构建并串行运行 core、ROS 全包 CTest 和控制器 Python 测试；不并发运行长距离 benchmark。
- [x] 在未使用的独立 ROS 域启动 baseline（周期 0）和滚动模式，两者分开启动、每次重置场景。下面 72 仅为命令示例，先确认与现有运行实例不冲突；0.5 s/1.0 m 是实验起点，不是生产默认。

```bash
export ROS_DOMAIN_ID=72
ros2 launch lunar_incremental_controller_demo controller_rviz.launch.py \
  case:=multiple_exits start_rviz:=false \
  rolling_replan_period_s:=0.5 rolling_replan_distance_m:=1.0
```

在相同 overlay 和 ROS 域的另一终端运行：

```bash
ros2 run lunar_incremental_controller_demo run_case --case multiple_exits \
  --timeout 240 --output "$ROLLING_ARTIFACT_ROOT/multiple-exits-rolling.json"
```

- [x] 对照周期 0 的 multiple_exits；补充 forward、reverse、final_yaw、detour。若 multiple_exits 没有平顺连续换段窗口，增加确定性长直廊场景到现有 vehicle_sim.py 的 Scene 中，使局部观测半径小于总路线长度；不修改车辆积分或让仿真器读路径移动。
- [x] 核对同任务移动中 revision 更新、平顺续接窗口额外停车为 0、原始命令限幅、碰撞、最终位置/朝向与 GOAL_REACHED；核对控制器仍为唯一速度发布者。
- [x] 汇总总耗时、非必要停车次数、规划 p50/p95、跟踪误差、失败记录和对应原始数据路径。无解、慢计算、取消竞态的确定性证据来自 Task 2/4 的测试，不假称 demo 已覆盖。
- [x] 如存在 Humble 环境，使用独立 build/install/log 重建测试；没有对应环境则明确 NOT_RUN。不得自动启动生产控制器或部署 Orin。
- [x] 执行 UTF-8 读取、`git diff --check`、工作树差异检查；完成代码审查，审查重点为单所有者、失效先于结果、关闭模式回归和无进展监测。
- [x] 将结果与限制写入报告，显式暂存文档并提交 `docs: record rolling planning validation evidence`。最终交付包含提交、测试结果、演示证据、未运行环境；不自动合并或推送。

## 自审覆盖与当前状态

职责/并发：Task 1、2、4；周期/提前与关闭：Task 1、5；失败保留/地图修订/过期结果：Task 2、4；
热更新/自交/无进展/局部末端：Task 3；最终确认：Task 2、3、4；闭环和文档：Task 5、6。
已按当前会话顺序实施 Tasks 1–6。复选框表示任务步骤已执行；不表示每种环境无条件通过。
最终结果、既有失败、闭环曲率制动边界和原始证据以 [验证记录](../../validation/2026-09-23-rolling-local-planning.md) 为准。

实施调整：复用 `Polyline` 而未改 path_geometry API；新候选使用完整地图遍历；在已有 limits.py 中增加受限曲率过渡；
接口配置按原 schema 用注释记录；增加 `Published()` 防止末端短引用刷新风暴；增加滚动候选诊断。
忙碌合并、最新位姿、析构停止、匹配执行弧长均有节点回归。独立审查的四项发现均已修复。
静态合同有一个基线失败，Humble 要求 32 MiB 栈，原生 Orin/目标 DDS/实车为 NOT_RUN。
平顺换路窗口已观察到零额外停车；完整圆形观测场景复跑有几何超限制动，未将其隐去或放宽控制限制。
