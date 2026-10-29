# 标准 Path 新导航集成实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Root owns Git baseline reconciliation and final landing.

**Goal:** 将已验证新规划器和扫掠版MPPI跟踪器集成为标准Path轮式导航链，短闭环通过后切换轮式代码默认入口。
**Architecture:** 纯规划、独立地图来源、标准Path跟踪与验证器职责分离；生产模块不依赖验证plant或DRL。旧接口保留，新接口无需业务协议扩张。
**Tech Stack:** ROS2 Humble、Python、C++ native、acados/CasADi、nav_msgs/Path、P3 GridMap。
**Spec:** ../specs/2026-10-28-navigation-standard-path-integration-design.md

## Global Constraints

- 业务输出仍只有两条nav_msgs/Path；controller适配标准Path，不要求planner补PathReference、Action、ACK或地图服务。
- 局部0.01m、500×500、5m×5m；任意正源分辨率及热变化，UNKNOWN保留，allow_unknown默认false且显式可选。
- 轮式车体1.182×0.818m，|v|≤0.2、|w|≤1.0、a/alpha≤0.5、行进曲率≤1.0；能力文件和显式覆盖作为统一来源。
- 目标xy/yaw容差0.1m/0.05rad、停稳v/w≤0.01；原地转向需完整车体支持。
- 不因超过50ms单独制动；实际指令历史、有限参考、地图/运动认证保留。
- 最新TF含静态/零stamp和链式转换；不增加严格同步或可选元数据准入门槛。
- 不触碰现役DRL冻结运行/训练或足式默认，不启动真实T5控制；测试域92且ROS_LOCALHOST_ONLY=1，命令重映射到验证话题。
- 无批量删除、reset/stash/覆盖他人文件；不推送origin、不创建PR、不删除分支/worktree。

## Review Focus

- 高频裁剪/重发路径不能使BRAKING/进度/停滞时钟反复重置，也不能跳到自交路径的远端。
- body yaw和重复xy转向不能被折线简化吞掉；局部端点不能冒充最终导航到达。
- 空/坏/停发Path需保留真实速度及延迟历史受限制动，不能用瞬时置零作为模型停车证据。
- intrinsic地图必须包含完整车体支持后用于follower认证，未知/边界不能误标测量FREE。
- 标准Path的VOLATILE QoS及静态零stamp TF必须在真实DDS中证明接通，非仅mock。

## Root前置与恢复记录

- [x] 保存NAV源码f33b2eb7及FOLLOWER源码83d6d10a，冻结原字节与旧验证身份。
- [x] 合并台式主线b2068706与本地主线439178f2，基线汇合ed2d28a0；跟踪器集成8009830b，任务登记冲突逐条保留。
- [ ] 为本轮源码统一LF及末尾空行，记录与原快照仅换行的等价性；读UTF8、AST、git diff --check。
- [ ] 台式Humble隔离构建与两模块完整相关回归；失败定位后修复，不用历史PASS替代。

### Task 1: 标准Path有序参考与跟踪执行

**Files:** Modify controller/node.py, mppi_path_follower.py; create focused standard-path reference/execution module(s) in ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller; extend test/test_mppi_node.py and add test/test_standard_path_follower.py; register tests in CMakeLists.txt.
**Interfaces:** Expose controller-side set_standard_path(points, state/identity context) and request_controlled_stop; preserve legacy set_path/PathReference contracts. Standard mode consumes ParsedIncrementalPath.path_xy_yaw and receipt time; publishes ordinary Twist only.

- [ ] RED: tests for mppi_path_follower+incremental_path startup/QoS, identical geometry newstamp, rolling path updates maintaining real progress/history, reverse body yaw, repeatedxy spin, intermediate corner spin and yaw wrap, nonfinal local exhaustion waiting, empty reference controlled stop.
- [ ] Implement a focused ordered-pose reference and standard-mode controller path; retain legacy Polyline/backend semantics. Only actual pose progress advances a rolling reference; preserve command history and physical stall timer. Local endpoints always bounded stops/waits, never external navigation completion. Explicit discontinuity or direction change uses braking.
- [ ] GREEN: focused pure tests then complete controller suite in actualHumble; tests must show acceleration-bounded velocity decay after stop intent and actual movement through at least30 rolling updates.
- [ ] Commit explicit task files and report RED/GREEN commands, outputs, changed interfaces and any limits; root dispatches independent spec+quality review.

### Task 2: P3地图来源、最新TF与ROS连接

**Files:** Create neutral nav1cm/follower_map.py and controller/p3_map_source.py (or focused equivalent); modify controller/node.py, mppi_map.py, package.xml/CMakeLists.txt; add test/test_p3_map_source.py plus pure map adapter tests under tests/navigation/one_cm; document configuration.
**Interfaces:** Use P3MapStore/NativeTerrain and LatestTransforms without session/coordinator/node imports. Map source produces immutable current center-support GridMap plus receipt liveness, independent of old GetPolicyMap. Standard mode selects p3 map source; old policy_service remains available.

- [ ] RED: center clear but footprint hits obstacle; swept corner/arc and full-stop tail; unknown false/true with source semantics preserved; map boundary; 0.05→0.02→0.01→0.005 source changes, rolling origin/rotation; sameepoch refresh updates liveness; static zeroTF and chainedframes.
- [ ] Implement conservative full-body support mapping and single-owner map worker, latest map/state/TF conversion and reliable/volatile Path input. Keep numerical/structural checks only where needed. Empty/state/map unavailable response must distinguish bounded stop evidence from unknown motion; no fake zero-speed feedback.
- [ ] GREEN: focused map/TF tests, preserved original swept-map regressions and controller suite; actual native terrain and planner packaging build/test, no DRL dependencies.
- [ ] Commit and report; independent spec+quality review before joint validation.

### Task 3: 真实planner—follower短距离联合验证

**Files:** Add scripts/navigation/one_cm/path_follower_validation/ focused fixture, plant, supervisor/evaluator modules; add meaningful supervisor/metric tests; add docs/validation/2026-10-28-nav1cm-path-follower-integration.md.
**Interfaces:** Fixture publishes actual P3 messages, targets, measured odom/TF from independently integrated Twist; real installed planner and real installed follower communicate standardPath. Independent rectangle SAT/limits verifier consumes rawstates and analytic obstacles only; no production certify or controls shortcut. Domain92/localhost only, commands to /nav1cm_integration/cmd_vel.

- [ ] RED: grader rejects missing real follower motion, wrong ROS types, premature process exit, collision/constraint breach and missing formal0/PLAN_FOUND/true source proof.
- [ ] Implement bounded forward+terminalspin, reverse, rolling long-enough local reference, corner/obstacle or mapupdate, empty/stop/resume and source-resolution/zeroTF checks. Include250ms compute delay and demonstrate current safe path use where valid. Maps≤100m, actual paths short; no1km run.
- [ ] GREEN: realHumble Release build and package tests; run short jointcases once, preserve all failures and source/log identities; diagnose actual causes, rerun only affected cases after changes. Supervisor failure independently vetoes result.
- [ ] Report actual timing, arrival/hold, speed/acceleration, collisions, path updates, exits and worker cleanup. Distinguish simple movement vs obstacle movement coverage. Commit harness/report and independent spec+quality review.

### Task 4: 轮式默认入口与主线落地

**Files:** Add/modify wheel-specific launch/config, scripts/orin/start_navigation.sh and scripts/orin/build.sh as required; wheel application start_hardware_navigation.sh adapted in app-owner isolated topic when applicable; adapt publish_navigation_goal.sh to PoseStamped for newmode; update README, docs/操作指令.md, task registry and validation record.
**Interfaces:** Wheel profile selects NAV1CM + standardPath mppi_path_follower + directP3 map source after Task3 PASS. Preserve explicit incremental_v2 fallback, legged default and legacy exploration mode; no Action-based exploration silently attached to PoseStamped navigation.

- [ ] Test selectors, package discovery/overlay and explicit build cache; wheel newmode and explicit oldmode; legged and frozenDRL untouched. Do not enable production command output during tests.
- [ ] Switch wheel code defaults only after joint PASS; installed launch must actually select real newcomponents, not merely changedYAML ignored by hardcoded script args. Use dry-run or bounded remapped real launch proof.
- [ ] Run relevant launch/config/static checks plus actual affected package build/tests; whole-branch independent review and any necessary fixes.
- [ ] Land reviewed commits to clean owner main via authorized Git synchronization, preserve dirtyWindows main and note its unchanged HEAD. Application-specific work stays with its owner; no originpush/PR/resource deletion. Record commits/qualification/remaining NOT_RUN boundaries and verify original main dirty-file hashes.
