# 独立探索地图入口

`lunar_exploration_map_node` 只处理地图、定位与 TF，发布 `OccupancyGrid` 并提供已有 `GetPolicyMap` 服务；不构造旧导航节点、规划 session 或 Action server。它是显式可选入口，原导航节点及原 `Export`/`Project` 行为不变。

```bash
ros2 run lunar_incremental_navigation_ros lunar_exploration_map_node --ros-args \
  -p platform_config:=/absolute/path/to/wheel.yaml
```

部署时地图组件与 NAV1CM 任务节点应选择同一份轮式能力文件。默认安装的 `config/wheel.yaml` 与任务节点一致，导航净空为 0.2 m；地图硬包络覆盖车体外接圆加该净空。离地高度继续使用能力文件的独立物理阈值。

## 参数与接口

| 参数 | 默认值 |
| --- | --- |
| `platform_config` | 本包安装目录下 `config/wheel.yaml` |
| `policy_resolution_m` | `0.2` |
| `coarse_resolution_m` | `1.0` |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_link` |
| `map_topic` | `/Car/T3/mapping/grid_map` |
| `odometry_topic` | `/Car/T3/localization/odometry` |
| `tf_topic` / `tf_static_topic` | `/tf` / `/tf_static` |
| `exploration_map_topic` | `/Car/T4/mapping/exploration_map` |
| `policy_map_service` | `/Car/T4/mapping/get_policy_map` |
| `map_qos_reliability` | `reliable`；可选 `best_effort` |
| `map_qos_durability` | `volatile`；可选 `transient_local` |
| `local_window_size_m` | `64.0`，仅策略查询窗口 |
| `goal_position_tolerance_m` / `goal_yaw_tolerance_rad` | `0.1` / `0.05` |

策略分辨率和探索分辨率独立于导航的 5 m × 5 m、0.01 m 局部窗口。分辨率参数必须是有效正有限数。源 GridMap 接受不同正分辨率与运行中热换；普通 elevation-only 输入不要求 terrain 可选层，也不要求 TF/里程计非零时间戳或协方差。GridMap pose 与 map→odom TF 正确组合，位于 map frame 的源图可直接接入。静态 TF 使用可靠 transient-local 订阅；里程计与动态 TF 为 best-effort。真实 P3 的 QoS 仍应在部署环境验证，1 MB 源图建议使用默认可靠接收。

## 地图语义

- 源格先用原生 `PlatformElevationEvaluator` 分类，再按实际世界 XY 足迹裁剪到固定策略格；任何源 BLOCKED 保留为 BLOCKED，完整覆盖且全部 FREE 才形成 FREE，否则 UNKNOWN。未知覆盖与已观测分类独立，不能将平均高程重新分类成虚假的自由空间。
- 有限观测替换相交的历史足迹，包括有限但本征 UNKNOWN 的证据；NaN 不清除历史。滚动窗口外的历史观测保留。源格 terrain 统计只在源格语义下使用，不写入粗格的 `LocalTerrainMeasurements`。策略高程是用于显示/传输的面积加权值，导航分类与成本采用源格保守分类，不声称增加了物理精度。
- 非水平变换保留真实投影足迹和高度；无法证明可通行的倾斜/退化部分保持 UNKNOWN。这是 2D/2.5D 接口，没有晋级为通用 3D 重建器。
- 导出包含 `states`（整车导航）、`intrinsic_states`、`observed`、`costs`、`elevation_m`。`observed` 沿现有合同取 0/1/2，表示未知/已观测自由/已观测阻挡，并非独立布尔掩码。
- `ExportObserved` 永不填 `assumed_start_*`；只有当前真实 anchor 的已知自由支撑满足硬包络时，才给出直接起点连接和 `stationary_support_ready`。未知起点、缺少有效里程计/TF时返回 `ready=false, INPUT_UNAVAILABLE`。旧出口仍保留其原起步补丁行为。
- 新 `ProjectConservative` 基于导航中心可行性 `fine.State`：任意相交 BLOCKED 则占据，全部面积 FREE 才置自由，其余 UNKNOWN。它不是纯原始 observed mask。旧 `Project` 的 any-FREE 语义不变。

## 线程、快照与成本

订阅回调仅存输入并唤醒地图工作线程。源分类、面积历史、原生派生、tile 导出与 OccupancyGrid 投影在工作线程执行；服务只读取已发布的不可变结果。每次节点启动生成新 epoch。raw 高程版本和 fine 分类版本独立；源 receipt 的 `processed_stamp` 在重复有效输入时也更新。

`ExplorationMap::Capture(since_revision)` 提供 128 项版本日志；无效/过旧基线返回全量，同版本为空增量。ROS 服务可用完整当前 tile 集形成较旧正版本的替换增量，同版本返回空增量，0 或未来版本返回全量。客户端仍须按 epoch 检查连续性。由于地图只增长且无 tile 删除，完整 tile 替换增量可与全量一致重放。

归一化后等价的 TF（含 q/-q）不触发源图重派生。只有里程计或等价 TF 到来时，重新检查真实 anchor/支撑，复用当前 tile 与投影；真正变化的 TF 仍重新投影源观测。DEBUG 日志提供接收 cell 数、适配/派生/导出耗时、policy cell/Fragment 数，不设置 50 ms 接收或计算门槛。

历史采用每个策略格的源多边形片段，内存与实际保留源观测相关；5 m、1 cm 源图可以保留 25 万个片段。当前只完成有界场景的真实 native/ROS 验证；未证明长期、大范围、高频非对齐滚动下的资源上界。没有删除历史或降采样障碍来掩盖成本。

## C++ 接口

- `ExplorationMap(capability, profile, policy_resolution_m=.2, coarse_resolution_m=1.)`；`Apply(const OwnedElevationEvidence&, int64_t stamp_ns)`；`Capture(uint64_t since_revision=0)`；`Project()`；`StorageStats()`。组件由一个工作线程持有，捕获结果由不可变指针保持。
- 新 `ObservedPolicyMapExportInput { PolicyMapExportInput map; shared_ptr<const PlatformElevationEvaluator> intrinsic; }` 包装原输入，原公开结构布局不变。`ExportObserved` 同时接受原输入和新包装，默认 `minimum_map_stamp_ns=0`。
- 新 core `FineTraversabilityBuilder::DeriveProjected(raw, capability, profile, unique_ptr<const PlatformElevationEvaluator>, previous=nullptr)` 共享原全量派生、哈希和校验，并使用 `:projected-v1` 模型标识；可在 raw 不变时生成新 fine revision。原 `Derive` 保持不变。
- 新 `AdaptExplorationElevation(InputSnapshot)` 支持 GridMap pose 与 TF 组合；原 `AdaptLocalElevation` 保持旧行为。`ExplorationMapProjector::ProjectConservative(fine, coarse_geometry)` 是独立保守出口。

Humble 构建需要同时选择本次 core 和 ROS 包的独立安装前缀。具体当前源码、库哈希、原始失败、默认栈回归和真实 ROS 证据见 `docs/validation/2026-10-29-exploration-nav1cm.md`。Jazzy、Orin、真实 P3/生产 DDS、实车、前沿及 DRL 集成本 Task 均未验收。
