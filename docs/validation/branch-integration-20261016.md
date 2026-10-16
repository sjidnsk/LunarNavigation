# 2026-10-16 分支整合验证摘要

记录更新于 2026-10-17（Asia/Shanghai）。本页整理已经执行的有限资格和真实落地，不表示全仓通过、功能齐备或可部署。六线入口、97 分支逐项归属和恢复约定见 [分支整合台账](../分支整合台账.md)。

外部证据 S = `/home/kai/CodexDownloads/lunar_navigation/branch-integration-20261016`，D = `D:/CodexDownloads/lunar_navigation/branch_integration_20261016`。以下 S 路径均为仓库外原始报告/命令/日志，未打包进此文档提交；历史报告的候选、待审状态属于其采样阶段，实际落地以 landing JSON 为准。本文不重写历史报告，也不重跑套件。

## 源码、执行与实际落地身份

main 生产锚点 `d452677ae0c681dc865bac35a4092beab3034f9b`，公共两测试守卫后锚点 `95d0b88e8ce33ff83c8bf2523e56eb2a4588bfb8`，也是当前 dev/drl-exploration。terrain 生产锚点 `355ed41b47e6833e44f249fd1e0f76ecd680bf56`，两测试同步后实际 `158a2e168a1d184811e51d45d328a78baf65370a`。三 app 实际 HEAD 为 TCP `f7abebc2f00c421e28c6adca5a1ae6bace54a816`、wheel `422866169329dfbf58b782a62a937f6bdeb28f32`、legged `4377d2762122e5eb9ed147b3a6883d17b4bbe4dd`。均现场核对；本文自身提交和最终 main 文档 HEAD 使用外部最终清单，不能把 95d0 称为文档落地后的最终 HEAD。

|阶段|实际资格及限定|外部证据|
|---|---|---|
|Task1，490333080a25c78b2298dc26a94c7e2b827ef207|86 行治理、导航两离线工具与 README；当前 C++ benchmark/消息契约 5 passed；扩展接口 2 FAIL 在 BASE 独立复现，后由 Task2 严格契约处理|S/task-1-report.md、task-1-landing.json；task-1-final-1.log|
|Task2，7acdfb5cda8d47e2d1c68d198d2a3fcbc0984b1f|terrain 兼容 490333；fresh core 29组418、ROS 13组107、契约68、controller90通过。固定4输入×5方法×两版40有限探针对照；非无机制效果/性能资格|S/task-2-report.md、task-2-landing.json；task-2-humble/logs、fixed-comparison/comparison.json|
|Task5，fdcf27b0c17093e7e79b62efd995a67c83dcacc2|从应用提炼 rolling controller，并经两轮 RED/GREEN 修复持续 revision 的初始对齐/正负反向阈值预算；最终 controller198+contract68=266通过；fresh controller CTest6组198。main/terrain实际落地各266通过，terrain中间同步头fa9158091c80eb339d7ac980c85f6bf814074b79|S/task-5-report.md、task-5-landing.json；task-5-humble/logs/fix-round2-*|
|Task3 初次完整执行，dd01de8b20c892880ce386c350942e0fa5ded591|fresh core20组311、ROS14组119、controller6组198；DRL普通解释器1513 passed/16 skipped/1 failed；定向50、demo21/114、工具185。失败和 -O 限定见下文|S/task-3-report.md；task-3-humble/build.sh、colcon-test.sh、drl-all.sh及日志|
|Task3 fix1 4de9a4f143e478cbdc57fce18cd19c4704aac467，fix2 d452677ae0c681dc865bac35a4092beab3034f9b|fix1 启动器/生命周期等50通过，旧入口1通过，真实Humble单例经入口1通过；fix2 README真实配置/禁止误导infer契约3通过。完整构建/1513项未重复。d452 经独立复审落main/dev，落地19项及79开发清单核验通过|S/task-3-humble/fix1、fix2；task-3-landing.json|
|Task6 初次完整执行，8ec5abf0960bd5b9cd7ee5f757d937081aee1226|terrain 合并d452；fresh13包；core30组434、ROS14组125、controller198、contracts68；CPU native221 passed/2 ROS opt-in skipped；固定40有限探针通过|S/task-6-report.md；task-6-humble/final-validation.sh、logs、fixed-comparison-final|
|Task6 修复，355ed41b47e6833e44f249fd1e0f76ecd680bf56|Build/BuildStationary/exporter显式传profile背景/方法；实际core/ROS重建安装；请求21、pipeline24、exporter2、中心5、普通Python producer parity2通过。未重跑434/125/221及40；这些仍归8ec原范围|S/task-6-humble/fix1/green.sh、test-counts.json、native-binding.json；task-6-landing.json|
|Task4A 三应用|各原26/25/31快照、逐段兼容d452、各自接口/配置/入口/控制器；闭包与失败详下表。三app两测试各10次，共60通过|S/task-4-report.md、task-4-test-sync-evidence.json、app-landings.json、task-4-landing-verification.json|
|Task7，95d0/158a|仅两公共测试文件；main overlay各10次共20通过，未改生产；terrain仅转同两测试并确认其余树等于355e，未在terrain重跑完整套件。独立review PASS（Critical/Important/Minor均0）后root实际ff main/dev及terrain|S/task-7-report.md、task-7-landing.json；task-7-humble证据目录以报告内路径为准|

上述验证均为 Humble-only CPU、串行性能场景及外置 build/install/log；未用 Jazzy 输出替代 Humble。Task2/5 的只读既有消息依赖使用报告指定 runtime install，完整依赖与命令以各报告为准。Task3 launcher 要求 PYTHONOPTIMIZE=1；pytest 重写测试断言仍执行，但非测试模块 assert 不执行，保留优化解释器警告。开发源码清单含79文件；旧 deployed manifest SHA256 `00bf70663cc394768d8ba5fc262a2ce56cdb76c5a84905f911aca8c62586c5cb` 未改。开发源码成熟与训练权重/策略收敛、部署输入适配是不同资格。

## 应用快照、安装资源与实际二进制

|应用|原 BASE|原字节 snapshot（单父 BASE）|最终验证摘要|
|---|---|---|---|
|TCP|4273314e4bb2893c7cfd9ecb61cc71955f799cae|253e19f825a5e4e571f420c5ce14daf7984a8557，26条|core389、navROS129、controller205、contracts68、native221+2skip；普通producer2；应用129pass/7skip/1环境FAIL|
|wheel|800b4f11b7a5d0b38e20e25d13c2c7cb79041c36|76ad0a0894ca97a6bfbedbafa16b33f0188a0ee2，25条|navROS128、controller225、exploration243pass/8FAIL（251项）；contracts68、应用29、native221+2skip、普通producer2；planner配置通过|
|legged|2d6f6b9913bf6e0d4cb8ec418fae2d0d3e79b738|6aa006e68280cb756671bed88a8ebf90bee55fcb，31条|core407、navROS128、controller225；exploration初轮250/251，1时序缺口修复后定点通过；16合成场景通过；contracts68、应用51、native221+2skip、普通producer2；未重跑整251项|

精确原提交 author/committer、时间、mode/SHA/路径和根独立核验见 S/task-4-snapshots.json、root-app-snapshot-verification.json。root正式原位重建同一快照对象再ff三个已审HEAD，tree精确匹配且clean；保全未改原82条快照字节。应用后续兼容修改属于快照之后的受审提交，不与原字节保全混为一谈。

TCP fresh14包；wheel fresh navROS/controller/explorationROS 后追加 plannerROS；legged fresh core/navROS/DRL/plannerROS/controller/explorationROS。TCP/wheel 64文件 core/native编译输入闭包相同 `a416d34d18289940c10b0c32d34b717faf50146df01ad65268c792e6f80e54dd`，legged为 `a402f92772e0eb268ab89b39c9ed65d0678639c3cd9674983945f8233ef37868`，后者独立fresh相关包。共同未变库仅在头/源码/ABI/动态依赖闭包确证后复用。79 Python相等不代表native或ROS资格。

每应用 S/task-4-{tcp,wheel,legged}-humble 中 native-binding.json 记录真实import、ldd、RUNPATH、/proc/self/maps及库SHA。安装闭包还包括 CMake/setup 读取的仓库根config/launch/rviz；各自 installed-resource-proof.json 与 S/task-4-external-install-inputs.txt 记录实测 ament前缀与get_package_share_directory资源SHA，分别覆盖TCP44、wheel56、legged58资源。包内Git树相同单独不能证明安装身份。wheel的第4个loader失败来自复用TCP安装中的legged资源，补本应用planner安装后消失；不能据三个包Git树相同继续用错share。

Task6 355e新core SHA256为 `6c1603e396946f54229b7b0e2e937b61cf9d64aa3b3962f81628f55728ecc8f4`，ROS库为 `049281c0dc9e40fc0ebb295494cf2e328f74611ebf4587565ecbe7ecbce1216e`；未变terrain扩展为 `fe5c680ced6f46c8f31541158e010187264af44cabc0d3a322361972338ca652`。其公开headers/messages/native源码未变，旧扩展实际动态加载新core；不是把仍加载旧core的so当新资格。40探针未进入本轮修复调用点，复用限于旧低层view规划对照，旧binding哈希不冒称当前40次运行。

应用保留配置、TF、硬件和到达契约及baseline-confidence；保留center_known=false→UNKNOWN、phase/derivative/local_window/stationary、精确stamp和delta/full语义。完整UNKNOWN输入bounds对应native GridView仅作本分支provider/consumer适配，没有把terrain四可选拟合器带入main/app。custom intrinsic优先与默认observations回退构造/Evaluate消费点、每冲突决定在 S/task-4-report.md 和 task-4-source-review-locations.json：TCP/wheel platform_elevation_physics.cpp:434的use_observations默认false，:439选择共享observations或move自定义分类器，:469实际Evaluate，:609三参构造显式true、:617四参构造保留false；legged对应位置由该JSON逐行记录；terrain只读对照为 S/root-terrain-custom-intrinsic-check.json。profile背景显式传到BuildStationary/exporter，未引入可选method API。

## 已定位修复与仍然失败的资格

TCP arrival旧测试 `RejectsInvalidArrivalToleranceOverrides` 原期望runtime_error，公共新契约 `RejectsNonpositiveConfiguredGoalTolerances` 为invalid_argument；仅旧断言改为准确类型。应用incremental_navigation_main.cpp:19捕获std::exception，入口非法参数实际退出1；不是留下窄runtime_error漏捕。原RED和命令见Task4报告及invalid-arrival-entry.log。

wheel最初4个loader失败曾被假设为TCP配置混入；反证表明原wheel.yaml与BASE clearance均0.4。独立BASE test/src/header/YAML、独立ament share及fresh loader实际14项11pass/3FAIL：

- `PlatformConfigLoader.LoadsPlannerInstalledWheelGeometry`；
- `PlatformConfigLoader.AcceptsAnyFinitePolygonWithAtLeastThreeVertices`；
- `PlatformConfigLoader.RejectsMalformedFootprintAndClearance`。

三个旧期望0.2及ReplaceOnce源值最小改为实际0.4后GREEN，非法值/严格断言仍保留；未改生产几何/阈值。第四项legged原YAML替换失败才是安装资源混配，处理见上节。原始假设、反证、BASE RED和最终GREEN均保留在S/task-4-wheel-humble。

两个公共测试同源时序缺口，现已在三app和main/dev/terrain同步：

- `ExplorationNodeTest.FailureMemoryLimitStillCancelsThirdStuckPlanBeforeStableError`：replan_count先于新goal到达，HasHandle只表示旧map非空，导致旧SUCCEEDED handle再次Finish。改为新goal计数及对应handle事件条件；未改生产终止状态机。
- `IncrementalExplorationNodeTest.SuppressedCandidatesAreGrayAndActiveCandidateStaysRed`：diagnostics到达不代表status optional存在。实际RED读取空optional；守卫等待正确第三goal的xy/yaw与PLANNING/NAVIGATION_GOAL_SUBMITTED状态，一次捕获并ASSERT has_value，再保留原计数和红/灰断言。未增加sleep或超时。首次误等EXECUTING的10次RED也保留。

原函数在main/dev d452、terrain355及三app snapshot同源，S/task-4-public-test-sync-origin.json记录原片段（incremental test:1254、exploration test:3160）和SHA。Task7只改上述两文件/两hunk，在公共d452生产安装绑定下20次通过；三app60次通过。有限重复不证明所有调度绝无竞态；不宣称BASE实际跑出同gibberish。生产/native大套件未因纯测试守卫重复。

Task3普通模式唯一FAIL为 `test_grid_wire_preserves_center_lattice_and_nan_holes`：Humble生成消息Float32MultiArray.data调试断言拒绝NumPy float32数组。普通整包1513pass/16skip/1FAIL保留；按仓库既有PYTHONOPTIMIZE=1约定单例1pass，fix1由真实launcher再1pass，没有改消息实现/断言。16skip为14 CUDA unavailable、2真实ROS opt-in；逐项名称在S/task-3-report.md“14项CUDA、2项ROS opt-in NOT_RUN”。

Task3扩大roots为321pass/4FAIL/1skip/2deselected，其中isolation已13项定向GREEN；剩余三个在fdcf完整BASE源码/安装实际复现，留给公共测试/运维入口owner：

- `tests/exploration/test_task7_implementation_contract.py::test_rank_and_failure_ownership_contract` 固定旧失败枚举，当前含NavigationNoPath/Timeout；
- `tests/test_operator_scripts.py::test_start_all_passes_grid_v1_mode_to_the_real_planner_invocation` 后台fake ros2未及时捕获planner argv；
- `tests/launch/test_incremental_exploration_navigation_rviz_contract.py::test_no_path_feedback_replaces_the_candidate` 候选与BASE均12次PLAN_FOUND/CANCELED，no_path_observed=false、candidate_replaced=false；clean_shutdown=true、control_topic_present=false。独立BASE三个pkg prefix均指向main-full-install，非candidate overlay。见S/task-3-humble/main-baseline-tests.log及报告main-probe-temp定位。

TCP `lunar_obj_tcp_sim/test/test_scripts.py::test_build_places_colcon_log_base_before_build_verb` 硬编码Jazzy，本机仅Humble；BASE测试与脚本同SHA并独立复现。Humble fake-colcon argv smoke通过只证明参数顺序，不替代Jazzy资格。应用7skip为6真实TCP E2E和1本地demo opt-in；native2skip为真实ROS opt-in。后续owner为TCP脚本/平台资格维护者，未安装Jazzy。

### wheel八项场景FAIL

以下保持原fixtures、断言和阈值。实际独立BASE源码/fixture、原wheel YAML独立share、BASE相同pure库执行也8 FAIL；297源码SHA及binary ldd/SHA在S/task-4-wheel-humble/base-scenarios-binding.json，准确命令/日志在Task4报告。不是仅凭源相同或一次当前失败宣称既存。后续owner：公共探索场景/验证维护者与app/orin-real-vehicle；本轮不作未经确认的算法修复，wheel场景资格仍未通过。

|精确失败名称|BASE与候选观察到的固定key/几何或取消差异|
|---|---|
|SyntheticScenarioRunnerTest.ConcaveReachableFrontierPublishesReferenceAndRemainsExecuting|fixture 3081,2250,0/cell6,4；实际0.4几何生成2881,2250,0/cell5,4，fake响应表拒绝，未进入EXECUTING|
|SyntheticScenarioRunnerTest.PointReachableNarrowPassageSurvivesRealWfdAndCandidatePrefilter|fixture首项5581,1750,225；实际6381,1750,225/cell12,3，无引用/执行|
|SyntheticScenarioRunnerTest.MapContentGrowthReevaluatesFailedCellAndExecutesOneCompleteTrajectory|fixture首项4081,2250,450；实际3881,2250,225/cell7,4无响应，Records()==1前提不满足|
|SyntheticScenarioRunnerTest.PartlyUnknownTaskCompletesBelowOneOnlyAfterEveryCandidateNoPath|同窄通道固定key拒绝，无法穷尽NO_PATH完成|
|SyntheticScenarioRunnerTest.OutsideToInsideUsesProductionApproachThenWfdBeforeExhaustiveCompletion|期望取消scenario-plan:task7-outside-to-inside/candidate/64；实际取消/0与/57，引用/0,/29,/57,/72，最终COMPLETED_NO_REACHABLE_FRONTIER；后段key16581对应实际16381，候选/请求序列变化。不是完全未取消|
|StampVersionCovariance/MetadataInvariantScenarioTest.MetadataOnlyUpdatePreservesExactActiveActionBehavior/0|concave固定key不符，初始执行引用未建立，尚未验证stamp不变性|
|StampVersionCovariance/MetadataInvariantScenarioTest.MetadataOnlyUpdatePreservesExactActiveActionBehavior/1|同上，尚未验证version不变性|
|StampVersionCovariance/MetadataInvariantScenarioTest.MetadataOnlyUpdatePreservesExactActiveActionBehavior/2|同上，尚未验证covariance不变性|

## 未达门槛、NOT_RUN及现存minor

返航约17k行最新累计来源 `bd39d264f15078c5e545f087604ee9287103aa86` 的两份2026-09-08诊断仍有默认64m下9 FAIL/3 waiting，单调进度筛选排除必要绕行；128m下12/12反事实不构成默认修复。历史/关键帧/轻量返航图未进入main；仅已适配local_goal_region公共部分不等于返航模块采用。需要异步取消、地图版本、安全前缀适配后固定480请求复验，本轮NOT_RUN。

terrain默认baseline，四可选拟合器仅dev。历史P3冻结输入不可用、完整同输入全量/增量性能与无机制主线效果对照未完成；40有限探针不补足这些门槛。candidate_geometry_v3真实输入/Orin推理适配仍NOT_READY/NOT_RUN；README移除可执行infer误导，不把旧v2入口拼接为可运行candidate链路。权重未收敛并不否定已审源码合入，也不授予策略/部署资格。

Orin、实车、GPU/CUDA、正式训练/模型/真实评测、外部TCP/真实通信生产者、真实DDS/bag回放、RViz、动力学/E2E、P3/全量增量性能/无机制效果、返航矩阵以及Jazzy均未补跑。localhost ROS fixture、普通producer CPU对照、现有源身份复用不能外推这些资格。

保留的minor：

- 离线比较工具复用输出目录存在覆盖旧产物风险；本轮使用独立任务目录，未把工具改成并发/复用目录安全。
- 旧pressure handoff子进程退出竞态初轮出现，current/0647定向及最终工具185项通过，但未修复或声称消除；它不是正式region_relation/candidate训练入口依赖。
- 已分类构建告警：Task3/6宿主colcon mixin覆盖22条、Time显式构造1条、禁用byte-compile6条；优化解释器警告保留。未改宿主配置/升级依赖消警告。
- 全仓同名demo test_evidence的Humble launch_testing收集冲突保留，采用包边界21/114覆盖；不宣称全仓收集通过。

Task3根全库测试曾意外触发Docker auto-pull；仅停止本任务进程，未启动测试容器。DockerRootDir=/home/kai/docker；事后无目标完整镜像/本测试容器，但无before inventory，不能确认或排除partial layer残留；未删除共享缓存/未prune。首轮2failed/164passed/5skipped后KeyboardInterrupt(exit2)日志保留；Docker验证NOT_RUN，后续明确排除。证据S/task-3-humble/docker-interruption-proof.json。

## 十项范围裁决及代价

原始连续记录为S/progress.md；以下完整保留十项Ruling，不把公共测试守卫收口虚构为新的算法裁决。

|#|决定与依据|代价/后续责任|
|---:|---|---|
|1|原86分支不因“非祖先”全部合并；旧架构/已替代实验保留历史，先完成开发与验证再晋级|若适用性判断有误，从完整备份重审对应模块|
|2|formal ea20为里程碑；唯一活跃DRL开发线是dev/drl-exploration|以后发布策略需用户明确要求调整，不再建立第二训练线|
|3|main接纳各模块成熟源码；权重未收敛不自动阻止DRL源码晋级|必须完成公共兼容审查/构建测试，不能仅交治理文档；策略资格仍独立|
|4|遗漏checkpoint state释放修复经RED/最小GREEN进入dev，冻结79和deployed清单不改|开发/已部署双身份、新开发清单及后续维护，旧证明不可冒充新验收|
|5|Task2后、Task3前增Task5，提炼三app成熟rolling controller；main原SetReference仍BRAKING|多一轮控制器回归/独立review，未切车辆默认运行或部署|
|6|未完成返航17k快照不因“完整架构”强行晋级，保留来源并标待完成|main返航仍待修复验收；9失败/3等待和64m问题不能掩盖|
|7|可信度仅dev，不晋级main/app；P3、全量/增量性能、无机制对照门槛不足|main暂缺完整候选拟合机制，后续按冻结输入补齐资格|
|8|Task3后增Task6，同步成熟公共架构到terrain；共同祖先fdcf、16处双边修改/12冲突标记不能ff或整文件取边|额外fresh构建与独立审查；保留专题/完整备份供重审，候选仍不晋级|
|9|完整输入bounds native适配仅留实际需要的dev/app；main保持已资格裁剪语义|维护分支adapter；统一公共契约须另做双边资格，不带出4拟合器|
|10|wheel八场景原BASE实际复现后保留断言/阈值，先完成已资格公共同步，不为绿色改key/取消/参数|场景资格仍缺，后续owner明确，独立审查继续判断边界；禁止all-green/设备可部署声明|

S/root-review-adjudications.md 记录两次Declined的完整裁决：Task4七点分别保留wheel8、TCP/Jazzy、设备/通信/P3等NOT_RUN、candidate推理NOT_READY、要求公共两测试实际收口、pressure/告警、要求原位落地/最终台账/恢复包；Task7五点分别保留wheel/Jazzy、以身份复用而非重跑冒充、terrain仅两测试边界、硬件/训练/真实通信不切换、20次有限同步不保证穷尽调度。公共测试要求已由Task7实际落地；最终文档/整体审查/新bundle在本文提交时仍待root完成。未将Declined简单等同“问题都已修复”。

## 保全观察与提交边界

原451历史+82app=533提交记录、2原始raw留存、102原refs/72原工作树/冻结79核验见S/pre-register-preservation-verification-reviewed-identity.json。原运行config.json SHA256保持 `8d159d7e55fcb42aa23aef21900595199d8371f809543b3b83ef2ef1618deefb`（原采样见S/task-3-humble/runtime-config-after.json），不改训练cwd/部署包；源码与运行保护不能混作算法验证。

运行 `/mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/experiment-20261014/identity.json` 的旧采样SHA256为 `fde5e33368d6a7d0303108db0aa399179a5710583b3e6f708150cb20b628b0f2`，当前为 `88874af292129930e92ea02718d195e992b5c5f12f02143a37c0195ace8b98a8`。Task3初轮after证据mtime2026-10-16 16:54:01.903073Z，fix1首采17:12:23.330072Z，当前文件mtime17:07:08.703813Z。旧正文未留，写入方未确认；不归因于外部或某agent，也不称全程identity字节不变。fix1/fix2、Task6、Task7前后和当前均为888…，root另存当前只读132KB副本S/runtime-identity-observed-20261017.json及S/runtime-identity-transition-audit.json，没有回滚runtime。原硬编码旧锚点导致的S/pre-register-preservation-verification.json FAIL保留；最新严格检查使用显式审核后的观察身份，未放宽冻结79/原config/source/ref检查。

本Phase B只修改台账和本页；不改生产/历史报告/长期refs/应用实现，不重跑套件，不push/delete/reset/rebase/amend。最终文档提交、main落地及完整union bundle的实际hash/fsck/verify均由独立review后root写外部最终清单，具体约定见台账。旧完整bundle不能冒充覆盖本轮新提交。
