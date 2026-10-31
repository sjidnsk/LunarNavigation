# MPCC滚动交接对比

沿P4-CONTINUOUS-GOAL-ACCEPTANCE-20261101，用户要求以顺滑优先，比较速度跳变、插零和重新起步。owner integration/pure-planner-orin / app/orin-unreal-simulation；复用hk/tcp-continuous-handoff-20261101。当前生产551ab08不变，分析脚本d04b2bc/1157a53仅写P4隔离源码路径，不发布控制、不修改P3或Env_X。

## 当前运行日志快照

运行tcp-interactive-continuous-goal-551ab08-20261101，navigation.log前31115574字节，SHA256 4050d6bd8c48116c7ad4d389481d4566b3ce8f6429e1b4f934a8e2c00ccf043a。11641条MPCC_COMMAND，时间戳1793473566.1591275至1793474615.192231。统计固定前缀，不将增长中的文件不同快照混用。

|同目标来源切换|次数|实际Δv P95/max m/s|实际Δw P95/max rad/s|原始目标Δv P95/max|原始目标Δw P95/max|严格零速命令|
|---|---:|---|---|---|---|---:|
|同模式同路线|5102|.008530/.087882|.017819/.099990|.006713/.149965|.045845/.199980|1|
|同模式换路线|62|.016875/.019366|.020753/.048633|.036400/.146986|.091648/.159099|0|
|换模式同路线|5|.019239/.019826|.015116/.016428|.110535/.111133|.009677/.009872|0|

全5169次交接实际Δv P95 .009192、Δw P95 .017819。参考最大角速目标从约+.1变为-.1，最终发布受slew限制。普通同模式同路线仍有跳变，因此不能都归因全局路线改变。

1469条严格零速：1464 BRAKE_SOURCE、3 GOAL_TURNING、1 POSE_UNAVAILABLE、1 ISSUED_CERTIFICATE_MISMATCH。1464条EXECUTION_BRAKE状态为1222 ARRIVED、222 GOAL_BRAKING、14 WAITING_GLOBAL、6 BUFFER_UNAVAILABLE；记录数不是独立停车事件数。到达后的保持不可冒充卡顿。

同目标切换唯一认证插零：源4895至4896、forward，原始issued目标v约6.77e-12、w约-.09999，认证MODE导致零速，约.08096秒后恢复。该问题独立于新候选求解时间，尚未修改。13个非零到零片段中8个前态缺任务身份，不能确认归属；已知身份且同目标恢复3例也只是运动恢复代理，不能直接认定重新进入起步状态。INITIAL_START_COMPLETED仅2次，分别goal_revision2/3，没有记录同目标每段重复完成起步。

远端结果：P4/artifacts/handover-field-log-comparison-20261101.json、handover-field-groups-20261101.json、handover-field-context-20261101.json。旧9f43f65日志目标、工况及观察时长不同，保留统计但不作暖启动因果对照。

## 隔离对比与边界

新增scripts/analysis/compare_mpcc_handover.py，仅加载不同MpccPlanner，共用协调器、参考缓冲、认证、动力学。每arm自身cmd推进下一状态；相同初态、路线/地图刷新调度，不是完全相同状态的离线回放。MODEL_SIMULATION不等于P3/车辆闭环。冻结limits来自P4已有轮式profile（.2m/s/.1rad/s），持令认证1.2s；原生几何/距离场库复用现有安装。直线、弯道、原地转向、epoch/route刷新，N20/N30，12s/场景。

本机7项工具测试通过。独立复审发现首版未配置P4的initial_travel_direction=forward，INITIAL_START_COMPLETED恒0，不能验证产品起步流程；首轮只保留通用模型对比，须补齐同一forward设置后重跑。报告严格零、.005近零恢复代理及真实起步事件分开计数，不添加产品门禁或放宽认证。

修正33b9023已将forward传入两组协调器并冻结在元数据中；9项工具测试通过，ROOT含原交接/缓冲回归34通过2跳过，独立复审8通过/1导出测试未运行，无新增阻断。旧generic分析进程按精确PID+命令行核验后停止；保留旧日志，不混入新版结果。远端新脚本compare_mpcc_handover_forward.py SHA256 418ed4af2bc7a10936571e9532ed9192038138e7ac04a153b98faa3abbc316ce，结果路径handover-forward-model-N20/N30-20261101.json。生产导航进程未切换。

补充日志相关性统计存handover-field-p3-correlation-20261101.json：按当前命令行p3_new_sample分组。该标记不等价于候选规划输入是否采用新P3样本，不能据此认定P3校正不是原因；仅作后续定位线索。

## 完成结果

33b9023修正后的N20/N30两组进程均返回0，29ec8bf补充plan-period参数后N20/.5秒曲线两组返回0。最后10项工具测试通过。共18个12秒模型场景，0.1秒组每场景采纳120条来源，0.5秒组24条；所有场景在开始运动后严格插零0、近零恢复0。直线/曲线/刷新每组初始起步完成一次；原地转向12秒仅转至1.18rad，尚未达到pi/2，起步完成0不能写成旋转完成。每场景第一周期尚无候选的BRAKE_SOURCE1单列，不算移动中插零。

以下为曲线同一pose/同一前瞻时刻的新旧目标绝对差，均为模型单位m/s与rad/s；非最终命令加速度或jerk。

|前视/更新周期|接续|Δv P95|Δw P95|Δv max|Δw max|
|---|---|---:|---:|---:|---:|
|4s/0.1s|baseline|0.001454|0.003322|0.007369|0.007399|
|4s/0.1s|warm|0.000773|0.002828|0.005391|0.008717|
|6s/0.1s|baseline|0.001522|0.003038|0.007935|0.008928|
|6s/0.1s|warm|0.000766|0.002589|0.005797|0.002919|
|4s/0.5s|baseline|0.002890|0.011010|0.004350|0.012177|
|4s/0.5s|warm|0.001663|0.006784|0.002502|0.008010|

旧模块SHA256 bedc7e65ca1024f9aa8b6ec039cab53e49eb6920e3de6bdf8d91100cccc9cb4f；新暖接续模块9f6e8a584fea4c980f7dbe60bcb7f81bb83e82fde40884df713941339ec8b645。行进旧组nominal120，新组nominal1/warm119；刷新场景新组nominal3/warm117。原地旋转旧组本来已有warm119，不可把它称作完全无暖启动基线。

产物SHA256：N20前进组3d0e9925047e5f95bf35e55488134f91e5133036eea3317d2e06be64b2752927；N30组b7c8c8ca81e7aaf1f44373caaf3d538897d84dca60b969e341614373282d1aab；N20/.5秒组d4523a332bc974a92691ca3fac459241e1999b8b5e4adbec25cc253b27e49a82。远端汇总P4/artifacts/handover-comparison-summary-20261101.json，本地外部摘要D:/CodexDownloads/lunar_navigation/handover-comparison-summary-20261101.json不提交。

结论：更新快增加接替次数，但没有必然导致插零或重起步。0.5秒减少次数、单次跳变P95却增大，不能认定更顺或车辆更差，尚未量化连续加速度/jerk/执行器响应。N20暖接续P95改善但角速max增大；本轮理想曲线中N30+warm交接指标较好，不外推到所有场景，更长时域同时影响现有朝向融合距离，不能把全部收益归于warm。没有复现现场原始角速目标接近+.1至-.1跳变；P3延迟/命令估计/底盘准备时序未入模型，车辆效果未验收。生产参数不改。

## 周期与认证窗口澄清

实际节点启动快照：planning_period_s=.1决定局部求解提交节奏，execution_period_s=.05决定控制发布调度；horizon_steps20、horizon_dt_s=.2组成4秒预测参考；reference_interval_s=.2用于在几何相位前方读取速度目标。command_hold_s=1.2独立决定“本次issued速度保持不变”的预测检查时长。真实认证还包含从冻结反馈/估计状态到issued的响应，以及末尾制动；认证不是仅.05秒，也不等于整条4秒参考。加长hold会扩大碰撞检查范围/样本，不会自动延长实际发布间隔或消除MODE拒绝。没有新指令时是否停车由现有执行接口职责处理，不从认证时长推导新的超时门禁。
