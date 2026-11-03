# TCP发布指令预测与1.6米局部前瞻

任务TCP-PUBLISHED-VELOCITY-PREDICTION-20261102；owner app/orin-unreal-simulation；ROOT，专题hk/tcp-terrain-window-deploy-20261102，来源61fc6444。用户先要求高频TCP补偿，经只读桌面P3_216_交接文档后明确改为“P3位姿加发布速度外推”，最终前瞻由2.4m改1.6m，最后明确免测直接替换正式运行。

## 实现与职责

复用现有CommandPrediction工具，新增显式use_published_velocity分支，不另建控制器。预测输入只订阅车辆最终/car/cmd_vel（来自统一topic contract），不是P4尚可能被转发层修改的提案，也不使用UE真值位姿或四轮反馈。源时刻P3位姿映射到单调时间，以实际发布历史为目标速度、按现有加速度能力积分；稳态解析圆弧，加速段小步积分并缓存。保留原生spin准备模型，P3同戳修正重新锚定，旧戳不覆盖，命令历史不覆盖则返回P3实测，不因预测不可用制造停车。新模式不使用旧30秒源龄门槛。

P3实测仍留在session.state、observed_state、任务到达/停稳与定位判活证据。每个tick只有一份规划估计快照，同时传session.tick(planning_state=...)及certified_velocity_command，覆盖局部窗口、参考重锚、完整与短令认证；发布时不增加“新状态数值不同就零速”门禁。诊断保留COMMAND_STATE_MEASURED并增加COMMAND_STATE_ESTIMATE，标注是否实际使用预测。

公共入口默认关闭command_prediction_enabled，只有TCP正式启动显式启用；prediction_command_topic由vehicle.command_topic同源传入。Node无新增进程，仅一个轻量速度订阅，回调只入队，tick独占预测器。未修改P3/Env_X源码或配置。

## 前瞻

TCP覆写horizon_steps=80、horizon_dt_s=.2，即16秒，.1m/s下理论1.6m。公共默认仍20/.2；控制.05、规划滚动.1、持令1.2、全局15秒不变。既有5m局部窗口可覆盖1.6+.75=2.35m，未引入窗口扩展。每周期继续滚动，不开环执行完整1.6m。

此前隔离真实acados N80三场景共9次通过完整认证，前进末端1.5877m（目标3m、非目标截断）。规划约35–46ms，完整认证48–49ms；solve_call约6–9ms。证据D:/CodexDownloads/lunar_navigation/tcp-command-prediction-20261102/horizon/results-80.json。N120/24秒与7m窗口仅研究证据，不部署。

## 已执行与免测边界

核心新14项及旧17项31通过；runtime共用快照、P3同戳修正及最终发布输入来源补5项，合计36通过。接线RED已复现预测参数/80步未传递；实现后159项接线/相关测试中158通过，1个旧静态断言仍预期.2而正式配置已.1；按用户新配置修正该断言后，用户要求停止测试，未重跑。Humble新订阅/最终组合/车辆闭环本轮NOT_RUN，不能用此前模式回退组件测试替代。

用户免测后只进行语法、SHA、导入及正式启动参数必需核对。静止TCP采样只能证明20Hz数据存在，moving-1仍静止，moving-2已结束且没有P3/cmd配对，均不作运动预测精度验收。指令模型会受执行迟滞/打滑/速度比例影响；P3交接所述时间基准偏移仍需以运行证据评估，未声称消除所有物理延迟。UE真值未进预测输入。

## 处置

本提交保存源码/接线/测试，正式部署回执另补；不自动移动长期线、不push，不删除工作树或证据。最终状态以部署receipt与startup_parameters为准。

## 正式部署收尾（按用户免测）

预测源码93ee4257已正式替换14个明确副本，TCP设备原有global_soft_clearance_m预算允许项保留并同步回专题源码。未覆盖其他设备变体，未改P3/Env_X。正式导入、AST、SHA、startup参数核对通过；operator87088/navigation87161/RViz87142，运行/P4/run/p4-20261102-210301-87088。

实际启动command_prediction_enabled=true、prediction_command_topic=/car/cmd_vel、horizon_steps=80、horizon_dt_s=.2、前后.1m/s、角.1rad/s；20度/relief.3/clearance.21/soft.5/UNKNOWN保留。P3输入持续，COMMAND_STATE_ESTIMATE出现used_prediction=true/proof_basis=published_velocity_model；当时为零速，不代表运动预测或闭环验收。未代发目标，未跑用户已叫停的测试。

回滚和部署回执/home/yanfa/P4/build/rollback-command-prediction-20261102/receipt.json。处置已提交/已部署TCP候选；不push/长期线合入，不删除分支/工作树，模型误差和后续用户观察结果单列。
