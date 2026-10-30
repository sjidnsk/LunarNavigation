# P3同帧地图分层与20cm探索对照

任务P3-MAP-LAYER-AUDIT-20261030；owner integration/pure-planner-orin；专题hk/p3-delayed-input-20261030，起点488ba87。仅诊断源码/产物写216新P4，P3只读，不开启控制或与tcp2共用输出接口。

## 对齐与重放范围

真实GridMap、最近真实Odometry及固定单位map->odom静态TF，以原始CDR保存。frame-r3地图/定位源戳均1793340490865824402ns，差0秒；收图源年龄4.8899秒，地图64x64m、0.2m，共102400格。没有插值、重标、伪造位姿/地面或反馈。frame-r2源戳1793340456859824402ns同样精确配对；两个样本各层总数一致。

每次从空消费者状态重放同一张已经融合的P3地图，复用216现有生产AdaptExplorationElevation、PlatformElevationEvaluator、ExplorationMap及ProjectConservative。nav1cm另走真实P3MapStore/NativeTerrain。不是从运行中的不同时间话题拼接，也不等于复现消费者全部历史累计状态。P3变换固定单位；本工具拒绝把不匹配坐标/时间当同帧证据。

## 按距车距离统计

| 距离m | 源格总数 | 有效高程格 | 地形自由格（20cm） | 车体支撑自由细格（20cm） | 自由探索格（1m） |
|---|---:|---:|---:|---:|---:|
| 0–2 | 316 | 48（15.19%） | 17 | 0 | 0 |
| 2–5 | 1660 | 1579（95.12%） | 1518 | 945 | 18 |
| 5–10 | 5884 | 3695（62.80%） | 1078 | 177 | 2 |
| 10–15 | 9832 | 17（0.17%） | 0 | 0 | 0 |
| 15及以上 | 84708 | 0 | 0 | 0 | 0 |
| 合计 | 102400 | 5339 | 2613 | 1122 | 20 |

距离按格中心相对精确同戳车辆位置计算，左闭右开。不同分辨率的格数不能直接相减；对应面积链为213.56m²有效高程→104.52m² intrinsic自由→44.88m²整车支撑自由→20m²（1m探索图）。nav1cm厘米图有1045200个intrinsic自由格，仍只代表104.52m²，未增加观测精度；车体认证在其后独立执行，不与探索细图混为一层。

## 主要损失原因

1. 上游范围被主动裁剪。实际运行profile的lidar_mapping_crop启用且active=true、omnidirectional=true、full_density_range_m=10、range_feather_m=0。terrain_mapper先计算weights再只保留weights>0；stereo_geometry在feather=0时执行水平距离<=10，确认是硬裁剪。雷达输入70m、LIO检测40m不等于高程建图范围。10–15m的17格是当前地图中观测到的既有边缘证据，不据此宣称当前裁剪外仍接收新地面。
2. 稀疏邻域。5339个有限高程格中2726个因neighborhood_complete=false成为UNKNOWN；本帧坡度/起伏/正台阶超限计数全为0，未发现BLOCKED。不能据此断言未知区域物理安全。
3. 车体支撑域。地图硬膨胀圆半径0.918722m，含车体外接圆及0.2m净空；2613个intrinsic自由格中1491个因支撑域接触UNKNOWN而失去自由认证，剩1122个。没有改安全包络。
4. 1m保守聚合。只在整格完全自由覆盖时发布FREE，44.88m²降为20m²，额外损失55.44%。
5. 近车缺口独立存在。最近有限高程格中心距车1.5299m；0–2m只有48个有限格、无支撑自由格。自身过滤盒x±0.591m/y±0.409m/z[-0.05,1.3]的确启用，但不足以单独解释整个近车空洞；需原始匹配点云分阶段计数区分视野/地面回波与过滤，不补造地面。

P3运行统计另取快照（不是上述同帧点数证据）：裁剪前64499点、后50262点；最新融合5277格，累计initialized=5339，memory_pressure=false、map_writable=true；单帧预算6000格、近车8m优先。没有证据证明6000预算是当前主要瓶颈。现用概率高程分支；max_elevation_points=12000不是该分支主要高程融合格上限，不应单凭此参数解释覆盖不足。源/安装stereo_geometry SHA一致，配置/源码/库摘要保存在P4诊断目录。

## 用户要求20cm对照

不变更车体、前沿最短长度1m、候选五个朝向、搜索与碰撞工作预算；任务边界与先前P3观察一致，为真实车位周围±16m。使用生产FrontierDetector.DetectAll与CandidateGenerator，另记录从实际机器人格出发的已知FREE连通性（不是nav1cm的UNKNOWN允许路径规划）。

| 探索尺度 | 自由格 | 自由面积m² | 前沿簇 | 候选观察点 | 起点FREE | 已知FREE连通格 |
|---|---:|---:|---:|---:|---|---:|
| 1m | 20 | 20.00 | 2 | 0 | 否 | 0 |
| 40cm | 241 | 38.56 | 1 | 1 | 否 | 0 |
| 20cm | 1122 | 44.88 | 2 | 4 | 否 | 0 |

20cm恢复4个安全候选观察位姿；不等于从UNKNOWN起点到候选的路径、Action交接、控制或自然到达通过。该表是离线真实核心对照；用户随后要求将当前探索默认改20cm，实施与部署证据在后续切片记录，不混写为已部署。

## DRL与层级职责

标准frontier通过exploration_map消费粗图。DRL的Nav1cmNavigationAdapter继承PolicyMapClient，通过GetPolicyMap消费策略分块；默认policy_resolution_m=0.2，响应包含fine导航状态、intrinsic状态和观测掩码，另有静止支撑状态。DRL不订阅1m exploration_map；地图节点可能同时生成1m副产物不代表策略消费它。内部图构造/特征归一化不等于地图分辨率；本轮不改模型输入合同或启动DRL。

## 验证与恢复

统计几何3项测试通过；原生与Python源矩阵解码逐格一致、源时间完全匹配、同分辨率投影自由数一致、有限格分类守恒。216诊断工具独立构建通过，两个真实帧复现总数。系统Python已有SciPy/NumPy版本范围告警原样保留，本次原生分类/投影未调用SciPy求解器。

必要工具scripts/simulation/p3_map_audit/{capture.py,replay.cpp,summarize.py,CMakeLists.txt}。P4/run/p3-map-layer-audit-20261030下保留frame-r2/r3、原始CDR、CSV、summary、构建日志、配置和库摘要；正式报告不上传P3/P4。D盘execution-tests/p3-map-layer-audit-20261030保留摘要与same-frame-map-layers.png。r1时间不配对失败证据保留；没有为了配对放宽阈值。

诊断未发布目标、速度或假地图，未启线上控制/规划节点，不修改P3或tcp2正在使用的源码/安装态。源码本地保存待合入；未push/merge/删除。闭环及实车NOT_RUN。
