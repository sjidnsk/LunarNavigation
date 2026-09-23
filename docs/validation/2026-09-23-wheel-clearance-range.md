# 轮式额外净空范围统一与分辨率离线对照

日期：2026-09-23。

## 配置决策

用户批准：TCP 与实车轮式统一 minimum_clearance_m=0.4，clearance_weight=2；硬碰撞模型不变。P4 精细地图分辨率仍来自 P3 输入，未强制重采样，也未修改 P3。0.1 m 仅为本地验证及后续 P3 配置候选。足式配置未修改。

实车更新 config/wheel.yaml、config/wheel_orin.yaml。TCP 更新 config/wheel.yaml 和 tools/tcp_deployment/templates/debug/p3_joint/wheel-p3.yaml；在通用 exploration_navigation.yaml 及 P3 联调 navigation.yaml、exploration-only.yaml 中显式固定权重 2。OBJ 启动入口继续使用轮式节点默认权重 2。

这里 0.4 m 是硬碰撞外接圆之外的软代价作用范围，不是额外硬膨胀，也不保证实际车辆保持该距离。

## 离线方法与结果

使用已有 Release 轮式核心（实车分支 6acb99b8），直接调用 FineTraversabilityBuilder 与 WheelLocalPlanner。测试改变 profile.preferred_clearance_m，权重固定 2；非 ROS、非会话或控制器测试。轮式能力与几何保持原值。TCP 核心本次未改；没有分别启动两条分支的运行节点。

地图 22.2 × 22.2 m：三个随机迷宫、一个折返迷宫、一个封闭终点。0.2/0.1/0.05 m 三档分辨率，0.2/0.3/0.4/0.5 m 四档净空范围，每组一次排除的预热及五次记录，共 300 次。240 次可达结果均成功，60 次封闭终点均无路，无超时。

0.1 m 可达场景汇总：

| 净空范围 | 规划中位耗时 ms | 所有路径采样最小余量 m | 路径长度中位数 m |
|---|---:|---:|---:|
| 0.2 | 11.99 | 0.260 | 66.54 |
| 0.3 | 13.96 | 0.431 | 68.95 |
| 0.4 | 14.63 | 0.514 | 70.51 |
| 0.5 | 15.03 | 0.595 | 72.10 |

余量指标：每条输出折线以不超过 2 cm 的步长采样，计算到原始高障碍格面积的距离，再减车体外接圆半径约 0.719 m。该指标不是地图派生代价，也不是车辆实际轨迹净空；仅查采样点邻近 1.5 m 范围障碍，无路径的哨兵值 100 不参与统计。连续线段最小值相对采样可能更小，不能作为严格碰撞证明。

补充直通道宽度 1.4/1.8/2.2/2.6/3.0 m，三分辨率、四范围，每组合一次记录，共 60 次：44 成功、16 无路；同一地图下四档净空范围的可达性完全一致。1.4 m 在所有分辨率均无路；1.8 m 在 0.2 m 无路，在 0.1/0.05 m 可达；更宽通道均可达。这包含栅格化及地形评估窗口的综合效应，不能推广为任意通道的精确宽度门槛。

结果支持先采用 0.4 m 软代价范围，并以 0.1 m 做本地轮式验证。耗时为单机单批探索性测量；首批无余量探针及短时静态测试曾与部分采样运行重叠，不用于严格性能增幅结论。

## 配置检查与证据边界

实车现有硬件/盲区/启动配置测试 6/6；TCP 启动/部署包/探索导航 launch 测试 18/18。四个能力入口 YAML 读回均为 0.4。未改变核心源码，无新的 ROS 二进制构建或安装；正式部署需从各分支重新生成配置包并核对运行时参数。

产物目录：/home/kai/CodexDownloads/lunar_navigation/wheel-clearance-20260923/，包含 probe.cpp、capabilities.hpp、corridor.cpp、results-metric.csv、corridors.csv 及通道地图。迷宫输入复用 /home/kai/CodexDownloads/lunar_navigation/maze-platform-benchmark-20260923/map-*.bin。probe-metric 和 corridor 的参数分别为地图目录和记录重复次数。

NOT_RUN：Humble/Orin、P3 实际地图重建、TCP 动力学闭环、实车验证。不能宣称已修复现场 START_BLOCKED。未推送或部署。
