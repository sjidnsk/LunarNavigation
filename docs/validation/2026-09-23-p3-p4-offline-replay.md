# 2026-09-23 足式 P3 → P4 原生 Orin 离线联调

## 结论

Humble/ARM64 上 P3 原始数据回放 → 正式高程/位姿/TF → P4 地图派生链路通过。
导航与探索未达到可执行路径验收：手工目标返回 NO_PATH / START_BLOCKED；探索确实生成候选并调用导航，但也遇到 START_BLOCKED。
没有 PLAN_FOUND + ACTIVE PathReference 证据，没有 GOAL_REACHED，更没有真实车辆闭环。

## 条件与操作

- 目标 yanfa@192.168.100.237，P4 足式源码 856d9332，ROS 2 Humble，域 69，CycloneDDS。
- 输入 /home/yanfa/ros2mid360.bag，约 341.53 s；3415 帧 PointCloud2、67018 条 Imu。
- 停止上一轮由本会话启动的无传感器等待进程；使用 P3 既有 replay_dog.sh、hardware237_bag.yaml 与专用 input_stamp_offset_sec=70440746.73060846。
- P3/P4 导航和探索均 use_sim_time=true；静态 TF 适配保留 P3 时间戳与原始变换。
- 控制器全程关闭，速度话题发布者最大数为 0；不伪造执行反馈，不更改 P3 源码或地图/障碍阈值。
- P4 通过现有脚本启动：start_navigation.sh legged true、start_exploration.sh legged true、start_static_tf_input.sh。

## 实测结果

| 层级 | 结果 |
| --- | --- |
| P3 正式定位 | 回放期间有 qualified 输出；末段 fusion_status 记录 filtered=2900、lio=2900、localization_valid=true。此文件保留的是末次状态，不等于回放结束后仍有新定位。 |
| P3 局部地图 | odom，64×64 m，0.2 m 分辨率，102400 格；elevation 等五层。 |
| P4 地图输入 | received_map_count=339、applied_map_count=339、rejected_map_count=0、incompatible_qos_count=0、middleware_lost_count=0、derivation_lag=0。 |
| P4 探索地图 | 已持续发布 map 坐标系、1 m 分辨率的 OccupancyGrid，存在 FREE/BLOCKED/UNKNOWN；不是仅发现话题。 |
| 足式配置 | 两节点读回 legged，use_sim_time=true；导航 tracking feedback=true，探索 FOV=360。 |
| 手工导航 | 目标从当前粗图附近 FREE 格选择，(22.3,-9.5)，Action 接受，结果 outcome=1 / START_BLOCKED；没有活动参考路径。粗图 FREE 不等于细图可执行认证。 |
| 探索 | 24×24 m 测试区域；观测到 3 个前沿、6 个候选、NAVIGATION_GOAL_SUBMITTED / NAVIGATION_PLANNING，随后 START_BLOCKED / WAITING_FOR_MAP_CHANGE；最终 CANCEL 后状态 CANCELED。 |
| P3 建图处理 | mapped_scans=1810，backpressure_dropped_scans=312。此为上游处理背压丢弃，与 P4 地图消息拒收统计不同，不能混称“无丢帧”。 |

观测器在回放已经开始后接入，其消息计数仅覆盖后段；339 来自导航节点自身全程累计计数。
观察器结束前，player 已 STOPPED，P3 pipeline/monitor 仍 RUNNING。P4 导航/探索/TF 适配保留，测试任务已取消。

## START_BLOCKED 的定位边界

代码在 RequestLocalStartPatchBuilder 中，原始细图起点为 BLOCKED，或者对 UNKNOWN 起点构造补图后复评为 BLOCKED，都会返回 START_BLOCKED；本次消息未区分两个分支。
随后采集的位置约 (12.297,4.784) 的 7×7 局部细图全为 UNKNOWN，原始高程存在少量 NaN。
该快照的位置/时间晚于首次拒绝，不能拿它证明第一次拒绝的根因，也不能据此认定是 TF 或尺寸错误。
下一步应冻结失败请求的同版本高程、细图、起点和补图中间结果，区分原始障碍膨胀与补图后二次拒绝；不能单凭本次记录放宽支持/障碍阈值。
P3 背压丢帧与此问题的因果关系尚未确认。

## 证据

目标：/home/yanfa/P4/deployment-evidence/p3p4-replay-20260923/
本机：/home/kai/CodexDownloads/lunar_navigation/p3p4-replay-20260923/

- probe.json：实际话题计数、地图诊断、参数、目标和探索事件。
- start-cell-audit.json：后续原始高程与细图局部快照（非失败时刻冻结快照）。
- planning-evidence.json：观测窗口内为空，不能声明规划成功。
- p3/fusion_status.json、map_statistics.json、pipeline.log、player.log：P3 输出与背压统计。
- navigation.log、exploration.log、tf.log：P4 运行日志。
- probe-stop-reason.txt：回放结束后主动停止临时观察器；probe.log 尾部 KeyboardInterrupt 来自该操作。

本记录没有验证定位精度、跟踪控制、真实足式步态或跨障能力。bag 位姿为固定录制轨迹，不能作为跟踪任意新目标的运动反馈。
