# 前沿与DRL接入新导航：交付汇总

任务NAV-1CM-AUTO-20261029。当前状态：共用任务层、地图接口、前沿探索、DRL可选后端及C1已完成；最终跨分支审查的两项Important经单次协调修复、同一审查员限定复审全部关闭，0新增C/I/M。源码已提交待归属线合入；主工作区已有改动保持独立，默认应用入口和正式训练任务未切换。此结论限源码集成资格。

## 职责与接口

- 策略层（frontier或DRL）负责选点，复用现有NavigateToPose；共用任务层负责目标替换、等待、取消及实际xy/yaw/停稳判定。
- 独立探索地图接口复用原高程处理与可通行性核心，提供OccupancyGrid/GetPolicyMap；不借旧导航节点取图。P3源分辨率可变化，导航局部计算仍为5m×5m、500×500、1cm，净空统一0.2m；真实未知保留，未加起步盲区补全。
- 新规划器输出两条标准nav_msgs/Path；MPPI从P3地图做局部跟踪。DRL显式navigation_backend=nav1cm，原默认incremental_v2、默认配置/恢复身份保持；infer显式flag优先于配置，再用默认。
- RosEnv与部署InferenceRuntime复用同一adapter factory和canonical观测准备；v5 Actor使用既有候选/schema/network/reward。薄ROS入口为lunar_drl_inference_node。

## 分支与提交

公共owner integration/pure-planner-orin，专题hk/nav-1cm-feasibility-20261026，工作树D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree；本次任务起点185ece29，Task1产品基线211c7c41，Task3修复产品68f52d2，C1产品734838aa。DRL owner dev/drl-exploration，专题hk/drl-nav1cm-adapter-20261029，工作树D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation；起点a094b902，产品9149f3c9、接口修复4ebcbf24、任务级审查HEAD874c1527。两专题代码分别提交待合入，不把整条公共main合进DRL，不移动owner或上游引用。

## 实际验证与限度

| 范围 | 实际结果 | 限度 |
|---|---|---|
| 共用任务层 | Humble Action与纯层回归，当前代际/取消/替换/实际停稳；竞态修复已独立复审 | 初始Action探针合成odom；实际运动另见前沿 |
| 地图接口 | native/core/ROS与任意源分辨率、观测语义及增量回归，真实250000cell大帧资格 | 大帧预检0目标，不作运动成功 |
| 前沿闭环 | 最终75/60/30/60四窗：9次实际到达、11次取消停稳；20精确UUID，90正式规划；一次早发静态TF全链及正向障碍地图更新 | 未知捷径/已知绕行优先仍未触发 |
| C1共用地图 | .2m源格浮点事件误造1cm未知缝已修，原709观测中2660误UNKNOWN恢复；净空.886→2.488m；Linux168与planner6/6 | 保留真实UNKNOWN，不改传感器/车体/净空；非新增策略资格 |
| DRL新后端 | 固定既有v5 Actor、45s窗实际1.629m、709→957观测、44次0/PLAN_FOUND/has_reference，当前UUID取消停稳，4子进程exit0 | 距目标约.1758m、yaw未对齐，未到达；不作策略效率或覆盖声明 |
| 原默认兼容 | 同权重8s smoke实际.987m，旧reference/tracking同UUID，取消停稳，3子进程exit0 | 不与新后端不同时长/不同地图构造作性能比较 |
| DRL入口修复 | 自定义map_frame与错帧保留当前任务、infer后端配置优先级12项Humble安装定向通过 | 独立console main/public task topic真实生命周期仍NOT_RUN |

Jazzy、Humble容器、Orin、实车、真实P3流、跨机生产DDS、rosbag、长期策略效果/完整RosEnv episode、正式训练部署未验收。本轮没有启动或切换正式训练；原权重、配置及正式源码manifest哈希一致，ledger未打开，不声称其字节前后审计。

## 执行记录与证据边界

新后端原readiness依赖ABI失败、C1前45s零运动、legacy首窗frame不匹配及所有测试/诊断失败都保留。Task4曾误选111/227组，涉及临时ReplayBuffer、两次临时learner.update、一次ControlledEnv假经验run_training，另有既有client-only域205；后续guard尝试不清洗历史。没有把这些结果写成“全组从未构造训练对象”。此事已向用户说明，临时47MB证据与真实成本保留；不改正式训练资源。

window03未及时捕获每个子进程现场/proc/maps，资格依赖预检实际装载、安装/源码/启动prefix/环境与真实raw；早期个别pytest完整argv未另存。不会后验伪造或为补历史重复成功物理窗。181个unique processed_stamp包含初始0，实际非零为180。

完整源/安装/raw审计在D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029；Task4旧运动archive d6c9494e、87源码与93回执、当前UUID/物理限值/正式规划/退出及冻结身份均已ROOT独立核对。新接口修复另有fix-round1安装与12项XML，不回填为旧运动运行源码。SDD工作区保留任务报告/独立复审与最终审查记录。

## 资源处置

两专题工作树、独立source/build/install/log/cache、原失败与归档保留待授权回收；未合并/推送/创建PR/设置upstream/删除。C1 shell CRLF留下的4目录和1文件准确路径见artifact下task4-common-map-fix/receipts/crlf-empty-paths.json。按用户禁止批量删除规则保留本计划工作区，不执行skill中的递归删除。

小型默认测试产物C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-64/test_inference_configuration_r0仅用于失败导入单测，不含训练权重、不用于恢复；审查结束后可按明确单路径逐个清理，本任务未删除。正式源码和主要产物均在D盘或授权测试台式任务目录。

## 实现裁定与误判代价

以下按计划账本出现顺序保留全部Ruling行及其代价；重复的落实记录不代替源码与运行验收。

1. Ruling: formalize the already user-approved design and implementation details without asking for another plan approval — direct human confirmation authorizes the ordered implementation, higher priority than redundant skill handoff — cost if wrong: localized spec/plan rework, no owner merge or training side effects.
2. Ruling: allow a minimal additive FineTraversabilityBuilder::DeriveProjected(raw,capability,profile,unique_ptr<const PlatformElevationEvaluator>,previous=nullptr) and observed-only export view needed by the standalone map adapter — existing FineCellEvaluator already accepts an intrinsic model, but ordinary builder fixes the model to canonical raw height and cannot retain conservative fine-source intrinsic/coverage changes at unchanged raw revision; use native source classification rather than fake coarse 3x3 statistics, preserve original Derive/Export semantics and all existing public type layouts — cost if wrong: projected-vs-native state inconsistency, revision/delta loss or ABI/link mismatch; require targeted source-projection/unknown/coverage tests, full affected native/ROS tests serially, and actual loaded-core/installed identity qualification before downstream use.
3. Ruling: fix the existing planning_snapshot_contract_test.cpp large test buffers by heap allocation, preserving all values/assertions and public construction behavior — root read actual gdb showing failure in unchanged anonymous MakeFineTile before DeriveProjected, and same test binary forced to old core still exits139 under default8192KB stack;32768KB success is diagnostic only — cost if wrong: fixture behavior could drift or an actual stack bug remain hidden; require focused default8MB-stack test then complete affected core/ROS tests with unchanged assertions and report all original failures.
4. Ruling: Task3 may add minimal optional common map QoS selection to PlannerNode and explicitly configure both new-mode subscribers together, retaining ordinary planner best_effort/volatile defaults — Task2 directly proved dense .01 frame transport drop before callback, current Task1 source hardcodes that subscription and cannot select reliable — cost if wrong: source compatibility regression or missed/duplicate maps; focused defaults and actual large-frame receipt qualification required. No DDS/system tuning, timing/covariance gate or physical change. Full scope/qualified interfaces added task-3-brief before dispatch; no Task3 implementation yet.
5. Ruling: include existing observed-only InferenceRuntime.attach in Task4 backend-factory plumbing and provide one thin ROS inference executable lunar_drl_inference_node; Task3 may prepare its optional drl hook without running it before Task4 — root found runtime.py:141 independently hardcodes old RosNavigationAdapter, so RosExplorationEnv-only adaptation would leave actual DRL inference on old navigation despite the user's request — cost if wrong: extra public entry wiring or legacy inference regression; keep existing infer/default behavior and networks/rewards intact, test same factory/defaults and actual fixed-weight attached runtime, no training construction. Owner DRL files runtime.py/new thin node/setup entry are added minimal scope, not a new policy/state machine. Shared launch remains MAIN-owned, optional DRL package required only when explicitly selected.
6. Ruling: keep the original blocked_wait_reselect v1 as failed diagnostic evidence and create a separately declared v2 with the unknown slit translated ONLY from x=[.9,1.3] to x=[1.6,2.0], keeping y=[-1.3,1.3], width/height, initial measured outer rectangle, obstacle event, body, .2 clearance, .15 response, motion limits, strategy and fixed60s unchanged — actual installed native replay of the saved original source proves center [250,250] UNKNOWN/eligibleFalse, with required radius .9187224777339302 crossing the x=.9 unknown boundary, contradicting this task's explicit known-start-support/no-blind-patch assumption; zero motion in that original fixture cannot validate reselect behavior — cost if wrong: translating the unknown feature may change candidate ordering or weaken the detour challenge; require native initial-support eligibility and observed-only policy readiness at the actual start before motion, preserve obstacle/detour geometry and all physical thresholds, log selected goals/routes honestly, and retain original failures/fixtures/sourceSHA separately.
7. Ruling: add ONE separately predeclared blocked_obstacle_update case of fixed60s, reusing qualified blocked-v2 geometry/support, source/50Hz cadence, .2 clearance/.15 response/body/limits and ordinary8s cleanup; inject the same0.6m-square/2m-high actual synthetic obstacle immediately after the first logically accepted CURRENT frontier goal satisfying measured body-to-goal distance>=1.6m, instead of the old v2 t>=5s condition — all v2selectedgoals stayedleftofunknownslit andOBSTACLE_ADDED neverfired, so that result proves taskwaiting/reselection but cannot supply the planned actualobstacle/map-update evidence — cost if wrong: an extra60s simulation/log cost and possibly exposing a realmap-update defect; preserve the four qualified runs, require explicitevent→source/mapupdate→matchinggoalWAIT→CANCELED_STOPPED→differentXYgoal and actualsafe motion, never weaken physics or manipulatefrontendgoalselection.
8. Ruling: add ONE separately predeclared blocked_obstacle_update case of fixed60s, reusing qualified blocked-v2 geometry/support, source/50Hz cadence, .2 clearance/.15 response/body/limits and ordinary8s cleanup; inject the same0.6m-square/2m-high actual synthetic obstacle immediately after the first logically accepted CURRENT frontier goal satisfying measured body-to-goal distance>=1.6m, instead of the old v2 t>=5s condition — all v2selectedgoals stayedleftofunknownslit andOBSTACLE_ADDED neverfired, so that result proves taskwaiting/reselection but cannot supply the planned actualobstacle/map-update evidence — cost if wrong: an extra60s simulation/log cost and possibly exposing a realmap-update defect; preserve the four qualified runs, require explicitevent→source/mapupdate→matchinggoalWAIT→CANCELED_STOPPED→differentXYgoal and actualsafe motion, never weaken physics or manipulatefrontendgoalselection.
9. Ruling: permit one minimal structured internal Runtime notice TASK_ACCEPTED containing task_id (actual managed Action UUID hex), accepted_generation and fully adapted map goal [x,y,yaw], captured by the existing single Runtime owner immediately after logical acceptance and delivered through the existing notice mechanism outside input locks — current_goal has no UUID and cross-topic callback order cannot establish identity; the owner already possesses both request ID and immutable accepted goal, so an internal notice closes the test-observation gap without rebuilding the public interface — cost if wrong: diagnostic identity drift, lock reentry regression or incorrect binding; test rejected/replaced/new-generation and notice reentry/default behavior, bind by exactID, reject conflicting/missing bindings, and record actual installed notice/source identity.
10. Ruling: Task4 reuses canonical pure v5/v4 observed policy preparation across RosEnv and deployed InferenceRuntime and refreshes OWN development-source-manifest/test exact file expectations, while preserving frozen manifest raw00bf7066/79files/portableprofile/historical observed_active_runtime/default serialization and all live source/weights/ledger — runtime rawv4 vs loaderacceptedv5 is a genuine connection gap and strict83filedevSHA would otherwise become false — cost if wrong: candidate/schema/default drift or frozen identity corruption; exact regressions and before/after raw hashes required. No policy/network/reward/training redesign or heldout access. New validations docs/validation/2026-10-29-drl-nav1cm-backend.md. Full rationale/interfaces/qualified paths appended task4brief before any code.
11. Ruling: final-review D1/M1-M2: defer existing pytest pythonpath/shared duplicate mixin warnings; no mixin chosen and no wrong artifact evidence; do not change shared environment — cost if wrong: a hidden environment influence could remain; actual selected source/native SHA and raw warnings remain auditable.
12. Ruling: final-review D2/legacy-timeout: keep initial OutsideToInside5s failure and later scoped/full16pass separately; no cause assertion or unchanged full rerun — cost if wrong: a rare old legacy defect may remain; owner sees original failure and no production readiness claim.
13. Ruling: final-review D3/Windows-POSIX: defer known Windows2POSIX path fixture issues; Linux native qualification stated separately — cost if wrong: Windows test portability remains limited; not misrepresented as WindowsROS success.
14. Ruling: final-review D4/unknown-detour: unknown shortcut versus known detour priority remains NOT_TRIGGERED; do not manufacture a passing case in this fix wave — cost if wrong: priority interaction unqualified; future scenario task may discover a defect.
15. Ruling: final-review D5/long-map: long-term rolling-map resource bounds and100m source1cm driving deferred; receive-only250000frame and shortwindows cannot prove them — cost if wrong: memory growth or sustained timing could remain; limit delivery to current short synthetic qualification.
16. Ruling: final-review D6/receipt-gaps: preserve missing window03 live child mappings and oldargv as evidence limits; no retrospective fill or successful motion repeats — cost if wrong: runtime attribution/reproducibility weaker than full live capture; rawprefix/source/install/precheck are explicit.
17. Ruling: final-review D7/old-T5: retain earliest domain92/local-only defaultT5 zeroTwist attempt, deliveryUNKNOWN; do not infer historical no-delivery or vehicle movement — cost if wrong: potential delivery uncertainty remains, private later graphs cannot erase incident.
18. Ruling: final-review D8/test-incident: preserve111/227 temporaryreplay/learner/dummytraining/domain205 incident and47MB/guardcost, accept current product fixes as separate code qualification; no formal job switched — cost if wrong: historical boundary violation cannot be undone; if hidden side effect exists it requires separate audit, no blanket no-training-object claim.
19. Ruling: final-review D9/frozen-ledger: use independent frozenactor/config/manifestSHA audits; ledger not read and no byte-beforeafter assertion — cost if wrong: ledger integrity is outside this task proof; do not expose/read it solely to claim clean audit.
20. Ruling: final-review D10/policy-deploy: defer policy efficacy/new-old performance/Orin-realvehicle; windows differ and new45sNOARRIVAL — cost if wrong: policy/physical deployment may still fail; only source integration can be delivered.
21. Ruling: retain pre-read targeted Runtime/helper/caller/default tests instead of broad DRL/fullsuite repetition during finalfix/finish — broad collection previously constructed forbidden temporary training objects, while selected29Humblecases cover amendedcontracts and defaultidentity — cost if wrong: unrelated regression could remain; disclose source-integration/NOT_RUN boundary and require suitable owner/deployment qualification.
22. Ruling: finish with both topic branches committed and resources preserved pending owner integration/recycle authorization, without owner merge/push/PR/upstream or skill-recursive deletion — explicit project/user authorization boundaries override generic finishing menus and workspace cleanup — cost if wrong: owner delivery and disk reclamation remain manual follow-up; record exact topics/commits/artifacts rather than silently claiming integrated/fully recycled.

### NAV-1CM-AUTO-20261029 最终协调修复波（限定复审通过）

最终跨分支审查提出 I1/I2 两项 Important，本轮唯一实现者 auto_nav_final_fix 在原专题处理。MAIN FIX_BASE eae70febe3b58ba03cc6a72de9df4387b9ae5bea；测试 c8ba920、产品 a1e4c46920ab7e4b7485d4b6270ae87c47d78c1b。提交前若收到新 odom/TF，唯一 tick owner 在相同输入锁内只重做最新位姿转换及纯任务观察，再提交状态；越界/运动重置到达保持，等价高频更新仍可反馈/结束。求解与外部 event/notice 不在该临界区，goal/cancel 原有顺序门禁保持。

Windows 确定性 RED4失败/11通过，GREEN15通过；Humble独立planner构建2.31s，纯 Runtime/task Runtime CTest2/2（21项）0.79s。DRL I2对应产品072f75d：owned配置位置/yaw容差传到map/planner/follower三方；独立构建4.84s，纯helper/caller/default身份8项通过。没有新增ROS节点、运动窗、Actor推理或训练构造。

新安装位于远端 exploration-integration-20261029/final-fix-wave/{main,drl}/install；MAIN38/DRL87安装Python与各Gitblob一致，开发87清单吻合，5个native装载实际core仍b63079f3。旧安装目标文件等于FIX_BASE，正式79清单/portable配置和Actor/config SHA保持；ledger未打开。完整raw/XML/身份在本地同任务ART/final-fix-wave/receipts.tar.gz与identity.json（归档SHA8435241a1599f3185e003a1db6ab9dc2c41196b3deecdd75396163bd7c445ed3）。

状态：同一final reviewer限定复审已通过，I1/I2均关闭；已提交待owner合入。原NOARRIVAL、console/完整RosEnv episode NOT_RUN、误选训练构造历史、M1/M2和旧失败保持；本轮新源不回填旧物理窗。专题及全部安装/证据保留待授权回收，无merge/push/PR/upstream/delete。

## 最终验收与后续边界

同一原审查员完整限定复审MAIN eae70feb..6e21fdf6和DRL b16c8ca8..6678837f，I1最新odom/TF终态与I2 owned三方到达容差均ADDRESSED，0新增Critical/Important/Minor；[原始总审查与最终复审](2026-10-29-exploration-nav1cm-review.md)保留两阶段完整结论。没有把曾经提出问题的报告改写为从未有问题。

ROOT独立核对300远端源码文件/Git对象、125实际安装Python、87开发清单、5 native与实际core b630、21+8定向Humble XML无失败、旧安装3个目标文件及旧运动归档、冻结权重/配置/正式manifest；旧owned启动分支AST和配置源码与BASE相同。本轮最终修复仅纯测试与独立构建，没有新运动、Actor推理或训练对象。早期Task4误选训练单测事件与原失败成本保持，不能升级为全程没有训练构造。

公共产品最终a1e4c46920ab7e4b7485d4b6270ae87c47d78c1b，DRL产品最终072f75d6fdd482495567212af20503ed81dd3f3a；后续收尾提交仅文档。本机Windows无ROS，Humble证据来自获准x86_64台式。真实P3/Orin/实车、console main/public task topic真实生命周期、完整新RosEnv episode和策略效果仍按表中NOT_RUN；DRL45s未到达保持，不把旧运动窗口记作新修复提交运行。

归属线integration/pure-planner-orin与dev/drl-exploration未由本任务移动。两个专题工作树及D盘/测试台式安装与原始证据保留待授权合入或回收；没有合并、推送、PR、upstream或删除操作。后续集成须按已记录underlay/source身份验证，不直接切换正式训练。
