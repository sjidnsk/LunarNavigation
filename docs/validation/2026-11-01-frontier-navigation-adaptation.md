# 常规前沿探索适配验证

任务 EXPLORATION-CONTINUITY-20261101，owner integration/pure-planner-orin。工作树 D:/CodexDownloads/lunar_navigation/global-route-terminal-20261101，专题 hk/global-route-terminal-20261101，实施基线47c3c2e。用户已批准设计及逐批实施/复审。当前状态：三批源码实现及独立复审完成，相关组件与隔离合成植物闭环通过；完整C++包仍有5项未改动范围失败，不能声明整包验收完成。已提交待合入、待部署，当前在线P4不切换。

## 分工与边界

探索负责区域、覆盖、前沿和观察朝向，复用NavigateToPose选目标；当前导航负责路线、局部MPCC、连续cmd_vel与实测到达。地图仍沿用独立20cm探索地图；共享基础地图暂停。本轮不改前沿打分算法，不训练，不涉及DRL模型。只写远端P4新隔离目录，不改P3/Env_X，不切换当前P4或启动真实车辆探索。

## 批1：显式入口

提交b8193c2。launch_settings关键字exploration_strategy默认none、可选frontier，CLI --exploration-strategy透传。默认和启用策略均不发布区域START或自动目标，已有mpcc_cmd_vel和平台参数不变。新增2项测试RED：2失败55通过；GREEN：106通过。独立规格/代码审查通过。AST透传检查是静态证据，不能称为ROS实际运行。

## 批2：修复前实际证据

在/home/yanfa/P4/artifacts/frontier-adaptation-20261101创建独立source/build/install/log，源码161项SHA核对。Humble core/ROS包首次构建成功，耗时6分14秒；core含GCC既有ABI说明，隔离overlay提示已记录，未改编译安全检查。

原基线两个CTest组：NavigationClient通过；增量探索40项中38通过2失败，均涉及一次性假服务反馈早于客户端关联目标。夹具在已有4秒WaitFor预算内周期发布反馈，保持产品时限不变。首次新增RED还捕获一次测试调用漏reason参数的编译错误；修正后重新构建。

四项新增行为回归在旧生产实现上全部失败（batch2-red-r2.log/xml，编译退出0、测试退出1）：

- LongInputWaitRetainsOwnedGoal：长等待真实触发取消，并变成NAVIGATION_MAP_WAIT_TIMEOUT_CANCELING。
- PoseWaitRetainsOwnedGoalAndReportsReason：WAITING_FOR_POSE被当普通规划，未上报定位等待。
- ManualReplacementPausesUntilResume：人工替换没有进入PAUSED。
- GenericTimeoutPausesWithoutRejectingCandidate：通用TIMEOUT没有暂停并保留候选。

以上不是仿真车辆到达证据，而是域170下真实C++节点与Action传输的缺陷复现。GREEN及独立审查待追加。

## 实施裁定

- 复用当前隔离专题与已有UUID/任务命令，不再建立控制层或进展实体。若未来需要并行开发再拆工作树；现阶段每批单写入者。
- 等待规则整条删除，不只改默认0，也不改名保留。历史报告不批量改写；活跃入口与测试同步更新。
- 通用TIMEOUT只说明执行或服务超时，不能证明地点地形不可达，进入既有暂停并保留区域任务，由人恢复；真实NO_PATH/INVALID_GOAL保留候选处理。代价是超时后需要显式恢复，避免无依据扩大地点失败记忆。
- 旧等待取消重选验证合同不得作为新行为的成功标准。保留旧原始证据，当前验证器需明确适配直接MPCC和等待保留语义。

## 资格边界

当前尚未完成本轮完整真实导航植物闭环。真实P3/车辆、Orin、DRL推理与训练均NOT_RUN；运行安装未切换。源码/静态/组件/合成植物证据分别记录，不能相互代替。原失败日志和隔离构建保留，未merge/push/删除工作树。

## 批2最终结果

产品提交9f58b71，测试清理360efaa、7b71c74。专用等待秒数、字段、计时器、诊断及两个launch参数全部删除；定位等待使用既有状态表达，地图/定位/路线等待保留同一活动目标。TIMEOUT及未预期取消进入现有PAUSED，保留区域，直到RESUME；真实NO_PATH/INVALID_GOAL仍处理候选。未新增状态实体/控制层/进展累计器，未修改导航速度、碰撞或完成阈值。

第二批独立规格及代码审查通过。保留测试中残余TIMEOUT换点断言已修正；客户端单次反馈早于目标关联会丢失的问题在夹具中改为周期反馈，并确保断言失败也先取消executor再join，未增加产品超时。最终实际Humble相关2组37项全部通过（NavigationClient2、IncrementalExploration35，8.17秒），完整包重跑时这两组也通过。

## 批3：验收器与真实隔离闭环

提交90dfd39及复审修复693ceb8、c10f17b。复用现有独立植物/净空/身份工具；显式选mpcc_cmd_vel及外部平台配置，按真实指令发布者和进程确认后端。直接模式使用实际MAP_APPLIED、ready policy与已知探索栅格证明输入，不要求不存在的MPPI诊断。原始COMMAND及积分STATE分别检查有限值与平台限幅。固定独立植物只允许匹配的轮式差速车体、足迹、0.2米净空，避免测试模型不符。

到达按批准5度和既有位置/停稳/保持条件；历史无参数记录仍按0.05rad。当前验收不接受任意放大角度阈值。同一导航进程的FINISHED/GOAL_REACHED日志必须先于新一代TASK_ACCEPTED，且与独立实测到达UUID匹配；不同观察位姿包括同地点新朝向，整圈等价朝向或相同位姿新UUID不算新目标。原“等待后取消重选”的两个旧运行case明确退役，历史原始结果及旧grade保留审计，不强行套在新行为上。

首次植物r1因验证器提取参数时漏strategy局部变量而失败，原始日志保留。ROOT只对由本轮创建且校验cmdline/PGID的域92私有launch组189253发SIGINT；确认无残留，没有操作生产进程。独立复审另发现后端归属、时序证明、原始指令与平台几何等资格缺口，补负例后修复，13RED/44通过变为163通过。不是通过放宽产品条件修复。

修正后的r2固定75秒，域92、ROS_LOCALHOST_ONLY=1、私有/nav1cm_task3/cmd：

- 实际Humble launch、C++前沿节点、当前导航Action、真实局部MPCC、连续Twist及独立运动植物均参与；没有MPPI节点或真实车辆接口。
- 2个目标自然GOAL_REACHED，3个不同观察位姿被接受。两次位置误差均0.018655243m；朝向误差约-2.80度、4.16度；实际v/w均0，保持0.4131秒、0.4500秒。
- 同一目标可暂时WAITING_FOR_PATH而继续保持活动，未因等待取消/计地点失败；合成源新增351格观测。
- 正式规划均以planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true核对；指令限幅、独立净空、身份、输入、运动、传感频率及清理门禁全部通过。
- 末段显式任务取消后实测停稳，launch正常退出，无验证进程残留。

实际运动采集源码693ceb8。最终c10f17b仅修正同位置新朝向的后继识别；使用同一r2原始日志重算派生结果仍通过，识别2次后继交接，未重跑/延长运动。原summary及原始日志不改；新增regrade-c10f17b.json保存旧summary SHA与新判据来源，不能把重放当第二轮运动。

该植物复用冻结合成场景，源地图0.1m、探索栅格0.2m、odom约50Hz；平台速度为P4的0.2m/s和0.1rad/s，持令1.2s、全局15s。验证器allow_unknown=false、初始方向auto、无Env_X轮组准备过程；在线P4为true/forward，故本结果只证明当前后端与限幅下的隔离探索闭环，不能等同完整P4/P3/TCP生产联调或真实定位延迟验收。

## 最终回归及仍未通过项

| 层级 | 本轮实际结果 |
|---|---|
| Windows完整one_cm与相关TCP/launch | 1155 passed、91 skipped（693ceb8） |
| 最后观察位姿更正定向回归 | 57 passed（c10f17b） |
| Humble当前探索客户端/状态机 | 37 passed |
| 75秒真实组件+独立植物 | 全部门禁通过，2自然到达、3目标 |
| Humble探索core完整包 | 227通过、1失败，共228 |
| Humble探索ROS完整包 | 240通过、4失败，共244 |

集合重叠，不能相加。Windows首次完整回归6失败是未设置已有geometry.dll/field.dll环境变量，补齐后上述1155通过；未改算法或把失败转skip。DLL在D:/CodexDownloads/lunar_navigation/nav-tcp-native-20261101。

Humble最初6项legacy异常是漏上传source/config/wheel.yaml夹具，补齐后重跑，原失败保留。最终剩余5项如下，涉及本轮未修改的core/legacy代码，122项源/夹具SHA与首次基线一致；尚需独立核对，不能仅凭源码未变就宣称这些失败无害：

- core：CandidateGeneratorTest.ClosedTangencyMatrixRejectsEveryNonfreeState。
- legacy：SyntheticScenarioRunnerTest.FreshBatchTimeoutCannotCompleteAndRetriesTheCandidate。
- legacy：SyntheticScenarioRunnerTest.OutsideToInsideUsesProductionApproachThenWfdBeforeExhaustiveCompletion。
- legacy：ExplorationNodeTest.LatestCacheNeverChangesInFlightCycleAuthority。
- legacy：ExplorationNodeTest.IdenticalStationaryOdometryDoesNotStarveApproachBuildCommit。

未改旧金标准、未放宽时间/安全规则、未通过重跑选择性隐藏失败。这5项保留为独立基线审计事项；本轮当前增量探索链路及相关37项通过不替代整包验收。

## 交付与运行边界

- 分支hk/global-route-terminal-20261101，最终产品/验证提交c10f17b；没有merge/push/PR或删除工作树。
- 远端必要源码和构建仅在/home/yanfa/P4/artifacts/frontier-adaptation-20261101；本地汇总D:/CodexDownloads/lunar_navigation/frontier-adaptation-20261101/qualification-final.json。
- 当前生产operator176765、车辆156544、P3 fusion156548、采集156545均存活且PID不变，在线10个产品源码SHA仍为7d7b5a7。当前P4未启用新的frontier开关，未发送真实车辆探索区域START或旧目标。
- 新入口部署后可显式使用--exploration-strategy frontier启用已有节点，默认none；区域任务仍用PureExplorationTask START/PAUSE/RESUME/CANCEL，启用节点不自动启动区域探索。真实运行切换需后续明确安排。
- DRL为下一阶段，未改训练、模型、策略schema或奖励；共享基础地图仍暂停。
- 原失败、构建与运行产物保留待审查/授权回收。首轮小型pytest夹具默认写入C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-158，测试结束后可删除；未批量删除，之后临时目录均显式在D盘。
