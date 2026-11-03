# RViz 清空区域与默认显示修复

## 范围与归属

任务 TCP-RVIZ-CLEAR-REGION-20261103，owner `app/orin-unreal-simulation`。专题 `hk/rviz-clear-region-20261103`，隔离工作树 `D:/CodexDownloads/lunar_navigation/rviz-clear-region-20261103`，基线 `99453a6c`。本子任务仅源码、模板、回归与文档；不部署、不重启，不操作运行中 RViz，不发送现场 ROS 目标或控制，不修改 P3/Env_X。

## 根因与修复职责

1. 面板原“清空区域”只清 `BoundarySelection` 并对 `boundary_draft` 发布 DELETE，没有通知探索后端。现复用已有 CANCEL，保留重复命令抑制；空闲时也允许清历史显示。
2. 后端原 CANCEL 重置边界、任务栅格与候选，但 `PublishLocked` 在边界/栅格不存在时不发布消息，保留 TransientLocal 旧样本。清空后的三个探索专用 MarkerArray 话题 `task_boundary`、`task_map_markers`、`frontiers` 应持续保留明确 DELETEALL，避免后续空包覆盖删除历史。草稿仍由面板自己的 DELETE 清理。
3. `Current exploration goal` 的 Pose 协议不支持删除；默认关闭重复 Pose 显示，由已有 `Exploration frontiers` 红色选中候选 Marker 表达目标。保留 `current_goal_topic` 接口。`Exploration task map` 同时按请求默认关闭。

保留既有 Action 目标归属取消；没有 cancel-all。手动导航目标、导航目标 Marker、导航路径与基础地图职责保持。暂停/恢复仍保留区域，关闭面板不自动发送 CANCEL。任务代际、IDLE 检查与取消预期沿用原实现，旧反馈和旧成功不能重算已清区域。

## 红绿与验证记录

- 本机配置旧值红测：任务地图 1 failed / 11 passed；重复 Pose 1 failed / 12 passed。
- 修改配置后：`D:/conda_envs/lunar-explorer/python.exe -m pytest tests/test_p4_topics.py tests/test_p4_tcp_runtime.py tests/test_start_p4_script.py -q`，93 passed / 5 skipped，3.44 s。
- 主会话统一协调的隔离 Humble panel 红测：旧实现 `test_panel_dds.cpp:184` 等待 clear 发布第六条任务消息失败；日志 `/home/yanfa/P4/build/map-unification-cd27fa78/rviz-panel-red-early.log`。
- 主会话统一协调的隔离 Humble 后端红测：旧实现 `CancelReplacesAllTaskVisualizationHistoryForLateSubscribers` 在 cpp:1416 失败，三个话题未保留 DELETEALL；日志 `/home/yanfa/P4/build/map-unification-cd27fa78/rviz-clear-backend-red.log`。手动 Action 回归第一次在目标刚被接受时读到 ACCEPTED，测试改为等待 EXECUTING 后再清空；没有为测试改动产品。
- 配置提交 `7cbeaa21`；功能提交 `e5ff1206`，仅 panel CANCEL 接线与 PublishLocked 明确删除消息。最新本机相关组仍 93 passed / 5 skipped，3.36 s。
- Humble GREEN：真实 DDS panel 1/1 通过；CancelReplacesAllTaskVisualizationHistoryForLateSubscribers 和 CancelAfterManualReplacementLeavesManualActionExecuting 2/2 通过。手动目标按同 UUID 的服务端真实执行状态核验；测试修正 1a08dc32，不修改产品。完整包测另补。

新增真实 ROS 回归覆盖活动/空闲清空、连续点击、草稿 DELETE、探索目标取消、手动独立 Action 不被取消，以及清空后迟到反馈/成功和新地图之后的晚加入订阅者历史删除。

## 验证边界与处置

本子任务源码已提交，处置为已提交待验证/待 owner 合入；隔离 GREEN 与包测交由主会话统一执行，结果未到齐前不声明 ROS 修复通过。本机 Python 与源码检查不等同 Humble/DDS/实际 RViz 通过。Jazzy、正式现场交互、Orin/车辆运动闭环 NOT_RUN；隔离 Humble 构建与 DDS 结果单独补记。未移动长期线、未 push、未清理工作树；专题保留待统一整合及后续回收授权。


## 主会话现场起步诊断（只读）

正式运行仍为 `build/budget2-coverage99-20261102/install`，不是前期隔离地图/认证优化安装。运行目录 `run/p4-20261103-131406-49720`。首目标 13:14:54 接受，全局 PLAN_FOUND 且有参考，局部原地旋转成功求解；首目标期间44条输出，间隔中位1146.59 ms、P95 1823.15 ms、最大1858.98 ms。同一时期指令计算中位137.45 ms，局部规划总耗时中位201.73 ms；不能把完整发布间隔全部归因于认证单项。

Env_X 当前接口 `cmd_timeout=1.0 s`、`spin_prep_time=3.0 s`。其原始日志在首目标开始90秒内记录37次断令PARK、38次AUTO；37个闭合AUTO窗口1.154578至2.796秒，全部短于3秒准备。准备期间接口仅转轮角，轮速强制为零；断令PARK清除准备状态。这证明车轮转而车身不转的直接链路。非零命令方向最初保持同号，不是初始左右角速度来回反号。

P4 的 execution_period=0.05 s 仅为唯一串行tick循环的目标周期；规划、地图复核、认证完成后才发布，无独立持续发布。转发器仅接收后转发，接管心跳不补发速度。空闲时当前两级话题间隔约0.05秒，不能据此推断活动任务期间正常。尚未实施发令调度修复，不能声称本轮RViz修复解决运动问题。没有修改Env_X/P3、没有放宽断令限制、没有发送现场目标或速度。

正式默认模板及当前运行配置文件已仅将 `Exploration task map` 与重复 `Current exploration goal` 设为关闭，原文件保存在 `build/rollback-rviz-clear-20261103`。当前RViz窗口有未保存布局，未强制重载。清空功能C++修复当前仅隔离构建验证，未替换正式运行进程；现场RViz交互与车辆闭环仍NOT_RUN。


补充调度结论：最小修复归属PlannerRuntime，计算与实际发布解耦；计算继续独占session和认证，唯一轻量发布者仅消费原证明剩余持令段，不得用重发刷新证明起点或有效期。TCP转发器不扩展规划合同，Env_X准备和断令逻辑不改。该方案本轮仅定位和审查，尚未实施。

完整包测首次结果：RViz 3/3；探索9/12目标通过。FrontierAddsAndDeletesUseMapFrameAndCurrentSimulationStamp旧断言把DELETEALL当成ADD，正在修正测试分类。另两个未改动的legacy库目标失败：IdenticalStationaryOdometryDoesNotStarveApproachBuildCommit单独复跑仍失败；FreshBatchTimeoutCannotCompleteAndRetriesTheCandidate单独复跑通过。二者链接未修改的legacy库，不属于新增清理实现路径；仍保留失败记录，不声明全包通过。


## 最终验证与收尾

Humble私域175最终incremental整组39/39通过，日志`rviz-incremental-final.log`；RViz包3/3通过，日志`rviz-clear-full-panel.log`；Python入口与配置93pass5skip。最终时间戳分类修正b64fff55保留ADD=100、逐ID DELETE=101及DELETEALL初始化/取消时刻断言，未弱化产品检查。旧legacy库两个超时证据继续保留，未声明完整探索包全绿。

当前处置：子任务源码已归并到主会话专题并保存提交，待owner应用线合入及正式C++部署/现场交互；尚未移动长期分支、push或回收工作树。正式模板默认关闭已落地，当前窗口未强制重载。运动问题完成因果定位，持续发令调度修复尚未实施。
