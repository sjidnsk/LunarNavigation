# EXECUTION_FEEDBACK_TIMEOUT根因修复与216验证（2026-10-30）

结论：已提交公共根因修复与TCP候选适配，并在216部署、构建及测试；整体低速导航闭环仍未通过。最终200ms配置下，P3输入未满足运动预检，未发送目标。不能将本报告写成已起步、已跟踪或已到达。

## 任务与来源

- 任务EXECUTION-FEEDBACK-REPAIR-20261030。公共owner integration/pure-planner-orin，专题hk/execution-feedback-repair-20261030，基线eabbfc4；TCP候选owner app/orin-unreal-simulation，专题hk/tcp-execution-feedback-repair-20261030，基线2495108。
- 公共实现aa314c0、2111872、955e06a、53165f9、e456a0d；TCP对应cb44638、5dd1f86、6ba5e28、b36e835、14d9a5b，配置b6ae9f7。
- 本轮未移动长期线、不push、不删除工作树。两候选已提交待合入，保留待验证/回收。目标机仅写/home/yanfa/P4；P3、Env_X源码只读，使用现有建图和车辆TCP接口。

## 取证与修复

1. 初始无运动取证目录`/home/yanfa/P4/run/execution-feedback-diagnostic-20261030-r2`。追加认证20次，均值约95.7ms、P95约136ms、最大150ms，超过当时50ms包络；控制tick最大347.5ms，参考回调源龄最大约0.969s。该轮首个执行故障为INVALID_STATE，不能冒称复现了同一种TIMEOUT。
2. 参考目录只保留128条发布记录，129次发布可淘汰仍在执行的精确accepted参考，使有效心跳失配。修复只额外pin当前真实accepted身份，下一次真实接管后释放旧pin；保留未接受不能续期、反重放、终态、摘要/会话/时间戳检查。它是独立复现的协议缺陷，不是后续所有TIMEOUT的归因。
3. Path/参考原在控制互斥组内接收。改用独立接收组和有界有序快照，保留真实接收时间，仍由控制组单写入执行器。队列溢出锁存，最终发布与锁存检查同锁，计算期间溢出也禁止非零发布。源龄过期在update前拦截，避免INVALID_STATE被永久锁存。
4. 完整轨迹及有向边只对逐位相同记录复用几何证明，inverse恢复原场景顺序；正负零、nextafter、边界、权限、全部延迟积分、运动学约束及完整制动尾保留。216无运动A/B中age4s整轨迹长键排序从约40–51ms降至约3.7ms；不能据此声称所有控制周期满足20Hz。
5. 用户后续明确允许放宽认证窗口，覆盖初始“不改认证超时”的限制。新增mppi_compute_envelope_s，公共默认.05，TCP配置.2；同一个值传入MppiPolicy.compute_budget_s，既定义发布窗口也扩大_brake_safe的等待/制动计算包络。不是只推迟过期判断。求解性能提示参数mppi_compute_budget_s不变；定位4.5s、地图6s、140步预测、路径/执行反馈超时不变；原a69b1d6历史覆盖保留。

## 216构建与测试

- Ubuntu/Humble ARM目标机增量构建控制器及lunar_nav1cm_planner通过。
- 控制器26组包检查580项，规划器9组39项：最终xunit共619项，0失败、0错误、0跳过。控制器第一遍25组通过、1组旧测试将角速度写死1.0失败；修正为按当前能力配置核对后只重跑该组通过。保留原失败日志。
- Windows静态launch合同：公共19项、TCP冲突并集29项通过。216另跑协议、真实ROS并发接收、时刻保持、overflow、参数及几何等价回归。独立审查闭环前通过；overflow竞态曾作为阻断发现并补回归修复。
- 目标产物`/home/yanfa/P4/artifacts/execution-feedback-repair-20261030/`含deployment-baseline.json、deployment-receipt.json、final-file-hashes.json、构建/包测日志、final-summary.json、input-health.json、p3-fusion-status-snapshot.json。原诊断r1观察器初始化错误和健康探针GridMapInfo字段错误只属工具失败，不当有效数据；后续修正探针证据独立保留。

## 实际测试与限制

| 运行 | 结果 |
| --- | --- |
| repaired-closedloop-20261030，50ms | 0.05m/s、0.1rad/s、车前0.5m目标接受；226条非空局部路径，0非零命令，TCP位移0，最终EXECUTION_FEEDBACK_TIMEOUT。目标期间定位接收约14.5s断档，没有当前任务身份的accepted心跳。原result.reference_accepted=true含目标前idle身份，不当作当前任务接受证据；后续探针按goal UUID绑定。 |
| repaired-closedloop-20261030-r2，200ms及全部优化 | 参数回读确认.2/4.5/6/140。发目标前45秒未能持续2秒满足预检（预检余量：定位3.5s、地图5.5s）；未发送目标、0非零命令、TCP位移0。追加认证8次26.0–112.5ms，P95约109.6ms；无publication_certificate_expired。这是等待阶段数据，不代表非零跟踪负载验收。 |

两次均遥测确认停稳、park、launch正常退出；最后核对测试命名空间进程已退出，192.168.10.23:6665(sensor_capture_node)与:6668(lunar_car_node)均ESTAB。没有停止其他会话的RViz、编译或其他平台进程。

独立45秒输入健康采样：定位37条、最大间隔8.784s、到达源龄0.861–3.921s；地图13条、最大间隔12.0s、到达源龄4.246–5.827s。P3状态曾出现localization_valid=false、no_active_source、vision_timeout及视觉协方差超限，说明持续输入就绪不能由TCP已连接或曾收到一条消息代替；当前P3源端具体退化链尚未修复。

配对本身不等待odom：先drain Path/参考，严格配对即可offer pending；缺失/过期odom阻止update，因此不能认证接管新会话。旧idle状态不属于当前task，规划器拒绝并超时符合合同，不能伪造新accepted或续期。尚有诊断边界：未接管时输入故障默认绑定旧active身份；规划器最新pose没有独立源龄门禁，GOAL_ACCEPTED不证明输入新鲜。本轮未修改该合同语义以掩盖上游断档。

## 阻断及恢复入口

自动审批拒绝拟议r3：把连续2秒稳定预检改成首次满足现有预算即发目标，理由是降低未获明确授权的输入稳定性门槛。该命令未执行、r3未创建、未绕过拒绝重试。完整修改和两轮证据已持久化；下一次运动应先解决P3输入断档并恢复原预检，或获得用户对具体预检变更的明确确认。

整体起步/跟踪/到达仍未验收；200ms配置下正式目标NOT_RUN。实车、外部UE其他场景、DRL训练/推理、Jazzy均NOT_RUN，不由本次Humble测试推断。
