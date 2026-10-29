# 前沿探索与DRL对接NAV1CM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 共用任务/地图接口，依次完成前沿与DRL对接，不切换训练任务。

**Architecture:** optional Action任务入口组合现有planner；map-only节点复用既有地图内核；策略只选点。公共代码在现有NAV专题继续，DRL在其owner独立专题使用公共Humble安装underlay。

**Tech Stack:** Python/NumPy、C++20、ROS2 Humble、现有NavigateToPose/GetPolicyMap、acados MPCC、MPPI。

**Spec:** docs/superpowers/specs/2026-10-29-exploration-nav1cm-design.md

## Global Constraints

- 输入 P3 GridMap elevation、Odometry、TF，接受任意有效正源分辨率及运行中变化、静态/零stamp TF；不新增严格时间戳、协方差或可选层门槛。
- 规划局部窗口5m×5m、500×500、0.01m；有效导航净空0.2m，实际车体与运动约束不放宽；地图重采样不增加物理精度。
- 全局允许UNKNOWN；局部allow_unknown默认false；真实UNKNOWN及已观测标记独立，暂不加入起步盲区补全。
- 普通计算超过50ms不单独触发制动；现有碰撞、运动及制动认证继续执行。
- 最新有效目标替换，无效新目标说明原因且保留原任务；已接受目标后来受阻仍保留，由策略决定何时取消/重选。
- 任务成功必须由当前目标的实际xy/yaw与停稳判定，沿用选定能力容差并保持0.3s；Path存在、局部段结束和Action传输接受均不充当实际到达。
- 明确取消停止旧任务后续路径、使在途旧结果失效并受限制动，实测停稳才结束取消；有效目标替换本身不强制车辆先停。
- 训练网络、观测字段定义、动作含义、奖励公式和现有默认后端保持；不启动/恢复/切换新或既有训练任务，不写冻结运行包、权重、账本或历史结果。
- 只在已获准的kai@10.63.250.129 Ubuntu22.04/Humble x86_64隔离环境验证；ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1，实际控制输出私有主题，禁止连接真实T5。
- 场景最大100m，实际运动使用其中短距离；不跑公里长程。依赖、build/install/log、证据均仓库外D盘或远端任务artifact独立目录。

## Review Focus

1. 目标A→B→A、无效B、旧cancel/异步结果：不能覆盖或结束当前有效任务；Task1测试。
2. 动态运动、局部段结束、空Path、缺位姿：不能误报成功或取消停稳；Task1/3测试。
3. 分辨率/旋转/增量、UNKNOWN与observed混淆：不可伪造自由或丢累计地图；Task2真实native对照。
4. 全局未知捷径与已知可达候选：不能以全局有路冒充可执行或探索完成；Task3短闭环。
5. DRL默认路径、观测schema和冻结训练：可选后端不得改变旧默认或启动训练；Task4测试/身份审计。

---

### Task 1: 共用导航任务层与可选Action入口

**Files:**
- Create: scripts/navigation/one_cm/nav1cm/navigation_task.py
- Modify: scripts/navigation/one_cm/nav1cm/p3_session.py, coordinator.py, async_coordinator.py (only cancellation/invalidation hooks)
- Modify: scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/runtime.py, node.py
- Create: scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/task_node.py and scripts/navigation_task_node
- Modify: same package CMakeLists.txt/package.xml/README.md
- Test: tests/navigation/one_cm/test_navigation_task.py; ROS package test/test_navigation_task_runtime.py and test/test_navigation_task_action.py
- Update: docs/任务登记.md for task identity/result, concise docs/validation/2026-10-29-exploration-nav1cm.md.

**Interfaces:**
- Consumes existing NavigationSession.offer_goal/offer_state/tick; existing NavigateToPose Action unchanged; current WheelLimits/tolerances.
- Produces NavigationTaskCoordinator (pure/noROS), immutable TaskSnapshot(task_id,generation,phase,outcome,reason_code); submit/cancel/observe public methods; Runtime.offer_managed_goal(task_id,message), request_cancel(task_id), event_sink; task_node executable navigation_task_node.
- ROS Action default /Car/T4/navigation/navigate_to_pose configurable; unchanged Path/P3 topics. Event DTO and exact method signatures finalized by implementer in task report before downstream use; no hidden mutable session reads from ROS callbacks.

- [ ] Step1 RED: tests test_invalid_replacement_keeps_active, test_a_b_a_and_late_cancel, test_segment_end_not_arrival, test_moving_at_goal_not_success, test_cancel_requires_measured_stop, test_manual_goal_replaces_managed, test_missing_yaw_waits_measured_heading, test_future_worker_result_invalid_after_cancel. Assert real state transitions and existing limits, not only API shape.
- [ ] Step2 run python -m pytest focused tasks before product changes, retain expected RED.
- [ ] Step3 implement minimal pure state machine and serialized Runtime queue/hooks, with same target identity and invalid-goal cause; existing ordinary planner behavior stays default. Action logical rejection has reason without killing old task; different request same pose remains separately attributable.
- [ ] Step4 add optional ROS task node, real Action client acceptance/cancel/feedback/result smoke in Humble; MultiThreaded/reentrant or nonblocking executor handling, lifecycle must exit without lingering tasks/workers. No PathReference/ACK or mandatory TF stamps.
- [ ] Step5 run focused + affected complete one_cm/runtime/package tests once after final changes; build affected package in independent Humble directory and record installed Python/native identity. Tolerances from chosen config, actual arrived hold0.3s; explicit cancellation stops publication/inflight and waits actual stop.
- [ ] Step6 self-review; git diff --check; explicit-path commit, report with exact signatures, all commands/outputs/identity, RED/GREEN, remaining NOT_RUN. Root task-scoped spec+quality review required before Task2.

### Task 2: 独立探索地图接口

**Files:**
- Create: ros2_ws/src/lunar_incremental_navigation_ros/include/lunar_incremental_navigation_ros/exploration_map_node.hpp, src/exploration_map_node.cpp, src/exploration_map_main.cpp
- Modify: same package CMakeLists.txt/package.xml/README as needed
- Modify: same package include/src policy_map_exporter.hpp/.cpp for additive observed-only export; existing Export unchanged
- Reuse: elevation_pipeline, exploration_map_projector, map_adapters/input_store/state_adapter; extend only concrete missing map-only behavior with legacy regression.
- Test: same package test/exploration_map_node_test.cpp, policy_map_exporter_test.cpp; source adaptation tests for variable resolution/TF as needed.
- Update: current task registry and validation doc.

**Interfaces:**
- Consumes P3 GridMap, Odometry and latest/static TF; same selected capability file minimum_clearance0.2; ordinary input data cannot require terrain optional layers.
- Produces executable lunar_exploration_map_node (no navigator/action/planning instances), /Car/T4/mapping/exploration_map nav_msgs/OccupancyGrid and configurable /Car/T4/mapping/get_policy_map GetPolicyMap.
- Produces additive PolicyMapExporter::ExportObserved(input,since_revision,minimum_map_stamp_ns=0), preserving current Export behavior; no UNKNOWN assumed-start or false stationary-support.
- Policy snapshot observed/intrinsic/navigation/cost/elevation and epoch/full/delta fields match current DRL PolicyMapStore contract; source changes do not relabel geometry. Policy/exploration res independently configured, first use current profiles not1cm whole world.

- [ ] Step1 RED: native known/unknown/obstacle and partial-update cases; same occupied scene observed-only Export never inserts assumed_start and legacy Export unchanged; invalid delta base causes full refresh; rolling crop retains earlier observed cells.
- [ ] Step2 run RED, then implement standalone map node by composition of ElevationPipeline/Projector/Exporter; no IncrementalNavigationNode or nav planner startup. Expensive compute on map worker, immutable snapshots for service/publish.
- [ ] Step3 test real elevation-only inputs at .01/.05/.2 resolution including hot change, rotation/translation, thin obstacle and NaN gaps, zero/static TF; compare full vs incremental and source geometry, observed bits and conservative traversal. Extend shared adapters only if needed, preserving old users and fields.
- [ ] Step4 independent Humble build/core affected tests + full affected ROS package checks serially (no CPU-parallel performance tests). Actual node publish/query discovery/cleanup with real native implementation; basic snapshot accepts ordinary direct observed start, unknown start reports unavailable without blind patch.
- [ ] Step5 self-review/explicit commit/report; root spec+quality gate before frontier. Report exact params, map source/geometry and real commands, no historical PASS substitution.

### Task 3: 前沿探索与新导航闭环

**Files:**
- Modify: launch/exploration_navigation.launch.py and corresponding package launch installation/config only as needed for explicit nav1cm mode
- Create: launch/nav1cm_exploration.launch.py (composition, strategy none/frontier; drl reserved until Task4 implementation)
- Modify: ros2_ws/src/lunar_pure_exploration_ros/src/navigation_client.cpp and incremental_exploration_node.cpp only new waiting/rejected result mappings needed; existing core candidate/scoring/coverage unchanged
- Modify: scripts/orin/start_exploration.sh explicit optional backend selection preserving old default, no actual hardware invocation
- Create: tests/launch/test_nav1cm_exploration_launch.py; package focused result/wait handling tests; scripts/navigation/one_cm/frontier_validation/ bounded real joint runner and grader tests
- Update docs/任务登记.md, docs/validation/2026-10-29-exploration-nav1cm.md and operator section.

**Interfaces:**
- Consumes Task1 Action task node and Task2 standalone map node, existing frontier task/status topics and policy.
- Produces explicit nav1cm exploration composition; exploration_strategy=none|frontier with controller default false, explicit control true for private tests. Only one goal source and one Action server; no old navigation node or PathReference/service dependencies for controller.

- [ ] Step1 RED tests launch graph selection and frontier WAITING_FOR_PATH cancellation/reevaluation; no map/no path/partial local completion cannot mean exploration complete; only proper terminal consumed for current active goal.
- [ ] Step2 implement thin launch/client config mappings; known FREE candidate reachability still based on exploration map, not optimistic global nav route. Use existing wait policy for strategy cancellation/reselect and preserve coverage/failed-candidate semantics.
- [ ] Step3 build/test affected packages once; actual real frontier node → real taskAction → installed planner → standardPath → installedMPPI → independent plant, synthetic100m-domain short motion (not100m travel), nominal response .15, clearance .2.
- [ ] Step4 cover at least reach/observe-next-goal, blocked/unknown waiting and explicit cancel/reselect, external cancel-stop. Real recorded sensor/map changes drive opportunity/coverage; don't substitute fake planner/follower or fabricate exhausted status. Existing known path vs unknown-shortcut behavior explicitly observed; stop scope expansion and report algorithm limitation if no executable progress.
- [ ] Step5 strict actual formal planner0/PLAN_FOUND/has_reference, taskUUID/goal identity, actual xy/yaw/stop, raw observed growth, independent body geometry/motion limits, private graph (no T5 publisher), exception and cleanup gates; retain failures. ROOT checks installation/raw identity and independent review.
- [ ] Step6 explicit commit/operator/validation records, no default app or training switch/owner merge. Review-clean gate before DRL implementation.

### Task 4: DRL可选新导航后端与固定权重短评测

**Worktree:** root prepares independent hk/drl-nav1cm-adapter-20261029 at actual remote dev baseline a094b90206db9c9bef2b6cf075db55cf210ccc22, managed D-drive path recorded before dispatch. Shared code provided by committed Task3 Humble underlay; do not merge main wholesale or modify existing dev/owner refs.

**Files:**
- Create: ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/navigation_backend.py, nav1cm_navigation.py
- Modify: same package config.py, ros_env.py, ros_messages.py (factor/reuse map-only client without breaking old adapter), package.xml and CLI parameter plumbing as needed
- Test: same package test/test_nav1cm_backend.py, test/test_navigation_backend_selection.py and focused observation/transition/cleanup cases
- Create: scripts/drl/verify_nav1cm_backend.py (explicit synthetic integration/inference only, no training command)
- Update docs/任务登记.md, DRL optional-backend instructions and validation file.

**Interfaces:**
- Consumes same NavigateToPose/GetPolicyMap contracts and common Task3 installation; new adapter begin_goal(goal), cancel_goal(), poll_goal(), execution_state(), poll_snapshot(min_stamp,refresh=False), ready/velocity/pose to preserve RosExplorationEnv logical client methods.
- Produces navigation_backend=nav1cm config, default remains incremental_v2; old legacy constructor behavior/source manifests unchanged. nav1cm state binds current ActionUUID/feedback and measured odometry; global Path length optional estimate only. No PathReference/TrackingStatus/old planning diagnostics dependency in new mode.

- [ ] Step1 RED default/explicit backend selection, invalid goal ordinary result, canceled current attempt/stopped state, observation schema/candidate order identity and unchanged reward/network config; no training process construction under verification CLI.
- [ ] Step2 implement adapter factory and optional owned processes using common installed task/map/MPPI packages, fresh namespace and isolated domain; do not start old nav node in new mode or require old path identity ACK.
- [ ] Step3 new backend runtime profile minimum_clearance.2/body/limits consistent; don't mutate frozen profile files. Exclude old UNKNOWN start-patch behavior; require actual initial observed support in integration fixtures. Validate actor observation schema and feature data, do not invent a map or feed truth.
- [ ] Step4 focused/full affected DRL tests and isolated Humble package build over common underlay. Legacy default smoke and new explicit launch/cleanup tested; frozen/owner/source manifest byte checks before/after.
- [ ] Step5 read-only compatible existing weight + SHA/schema verification; fixed synthetic short episode(s) on new backend, no optimizer/training/replay mutation; same scenario/limits for any comparison, separate output/run identity. Record actual submitted goal/current action result/observation updates/physical motion and cleanup. Existing formal training/test cohort untouched. If weight unavailable or mismatched, report that exact remaining validation without substituting random-policy PASS.
- [ ] Step6 ordinary navigation failure remains distinguishable from transport/data/integrity failure; preserve current budget/accounting semantics, no blanket failure downgrade. Explicit commit/report with baseline/underlay/new backend/default assertions; root checks raw evidence and task review before final whole-branch gate.

## Controller closeout

- Root owns shared source/ROS slot between handovers; only one implementation writer/runtime at a time. No subagents spawned by workers.
- Each task report exact BASE..HEAD, RED/GREEN, commands/results, loaded native/Python identities and actual scope; reviewers get unique full diff packages and do not rerun unchanged suites.
- After four review-clean gates, one broad final review across MAIN and DRL exact ranges; coordinated fix wave if needed. No duplicate unchanged full builds/performance runs.
- Commit source/docs only, preserve dependencies/logs/failures in external artifacts. Final status: 已提交待合入 (owners unchanged), explicit resources pending authorized recycle and Orin/realP3/vehicle NOT_RUN.


### Task4 依赖修复 C1：P3 自由区域的数值接缝

Task4 固定既有 Actor 的真实短窗暴露共用 P3MapStore._free_rows 缺陷：0.2m 源格、(-3,-3)原点产生 -79.99999999999999cm 顶点，数值微片段被判断为空，连续已观测自由区出现假的1cm UNKNOWN行，起点净空.885858m小于硬圆.921722m。ROOT已用最小纯函数与真实709观测初帧离线重放确认；原失败保留，既有Task1–3资格仍绑定原SHA。

- 在 MAIN 专题执行最小数值几何修复，保持真实UNKNOWN/亚厘米缺口、旋转、来源权限与native/solver/车体/净空约束；不换Actor/目标/场景或放宽约束。
- 先行精确反例与真实缺口/旋转回归，执行受影响P3地图/几何/缓存/native和planner包检查；不重复无关训练或已通过的运动窗。
- 独立build/install/log前缀，记录source/实际加载native身份，独立规格与质量审查通过后交给原Task4实现者，用相同45s fixture/Actor继续真实运动验证与legacy smoke。
- 具体证据、最小reproducer、范围与成本裁定位于本计划workspace/task-4-c1-brief.md及外部任务ART/task4-common-seam-ruling.md；不改owner分支、不推送或操作正式训练。
