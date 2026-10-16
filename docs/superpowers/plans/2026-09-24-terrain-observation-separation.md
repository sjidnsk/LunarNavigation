# P4 地面估计与平台评价职责分离实施方案

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax. 已获用户确认并实施；本轮未授权自动提交或部署。结果见 `docs/validation/2026-09-24-terrain-observation-separation.md`。

**Goal:** 保留 P3 原始观测和 5 cm 网格，提供确定、可复用的平台无关地面估计，再独立执行平台与整机通行约束。

**Architecture:** 在现有 core 内提取一个地面估计器，按输入视图和参数生命周期缓存。平台评价读取地面趋势及原始高差证据；整机评价负责包络、净空和代价。ROS 调度、规划搜索、控制接口保持原职责。

**Tech Stack:** C++20、现有 ROS 2 core/ROS 包、GoogleTest/CTest；不增加依赖。

**Spec:** `docs/validation/2026-09-23-terrain-observation-architecture-audit.md`。本方案具体化审查建议，所有行为验收仍需实施后验证。

## 全局边界

- 只操作 P4。P3 话题、消息、高程、定位和 TF 不修改。
- P3 输入为 5 cm 时保留 5 cm；不从较粗输入宣称获得真实 5 cm 观测。
- 轮式和足式使用相同地面估计算法与观测参数；保留各自地形背景尺度、尺寸、运动能力、底盘净空和碰撞约束。
- 继续使用 UNKNOWN / FREE / BLOCKED，加一个连续可信度；可信度是工程证据分数，不是通行概率。
- 推断不写回原始高程，不递归作为原始观测，不单独充当终点、足式真实支持或起点盲区出口证据。
- 不新增 ROS 节点、话题、持久地图系统；先使用本次派生生命周期内的缓存。
- 三个部署工作区现有未提交修改先记录 SHA、状态和差异。精确同步本任务部分，不整体复制平台差异文件；不合主线、不自动提交/推送/部署。

## 职责与参数归属

| 层 | 输入 | 输出 | 参数归属 |
|---|---|---|---|
| 输入/原始地图 | P3 高程、TF 与时间信息 | 不可变高程区间快照 | 坐标、输入有效性、历史观测合并 |
| 地面估计 | 目标格、原始视图、观测参数 | 地面趋势、拟合残差、连续可信度 | 20 cm 搜索尺度、样本分布、一致性与最低可信度 |
| 平台评价 | 地面估计、原始高差/支持证据、平台能力 | 内在 FREE/BLOCKED/UNKNOWN 与可信度 | 坡度、台阶、底盘净空、支持条件 |
| 整机评价 | 内在评价、真实包络 | 整机状态与路径代价 | 尺寸、碰撞、净空偏好、代价权重 |
| 规划/执行 | 整机状态和代价 | 路径与跟踪指令 | 搜索、简化与控制；不补地图 |

原始观测仍是障碍证据。估计地面用于理解坡面和补缺测，不能替换实测高点/低点。FREE 的整机可信度保守汇总地面证据；明确的 BLOCKED 可以由原始障碍证据支持，不能因为平面拟合失败就把确定障碍改成 UNKNOWN。

## 文件与接口

所有代码路径相对 `ros2_ws/src/lunar_incremental_navigation_core/`：

- 新增 `src/map/terrain_observation_estimator.hpp/.cpp`：平台无关地面趋势、补足、质量评价、输入视图内缓存。
- 修改 `src/map/platform_elevation_physics.cpp`：移出补足和可信度拟合逻辑，保留平台及整机评价；移除估计阶段对最大坡度的依赖。
- 修改 `src/map/fine_traversability_builder.cpp`：修正派生依赖半径、缓存参数身份和增量更新范围。
- 修改 `include/lunar_incremental_navigation_core/fine_traversability_builder.hpp`：仅在现有评价器需要访问共享估计结果时增加内部连接；保留现有外部调用入口。
- 修改 `CMakeLists.txt`，新增 `test/terrain_observation_estimator_test.cpp`；补充 `terrain_window_test.cpp`、`fine_traversability_builder_test.cpp`、`request_local_start_patch_test.cpp`。
- `local_planning.hpp`、ROS pipeline 和搜索器原则上不改算法；验证现有证据门控及代价消费仍成立。

内部最小接口草案（类型位于 `map` 内部命名空间，复用现有 GridIndex/Point2/ElevationRangeView）：

```cpp
struct TerrainObservationParameters {
  double support_axis_distance_m{0.20};
  double maximum_plane_residual_m{0.03};
  double minimum_confidence{0.20};
};
struct GroundEstimate {
  double surface_height_m{};       // 目标格处拟合地面，不覆盖真实高程
  Point2 gradient{};               // dz/dx, dz/dy，单位 m/m
  double residual_rms_m{};         // 原始观测相对拟合面的误差
  double residual_min_m{}, residual_max_m{};
  double confidence{};
};
class TerrainObservationEstimator {
 public:
  TerrainObservationEstimator(const ElevationRangeView&,
                              TerrainObservationParameters);
  const std::optional<GroundEstimate>& Estimate(GridIndex);
  int dependency_radius_cells() const noexcept;
};
```

估计器的输入视图与参数在生命周期内固定，不接收 PlatformCapability、车体包络或代价权重。缓存项按 GridIndex 寻址，引用稳定性由分块缓存保证；同一视图同一格的失败结果也缓存。正常派生绑定原始快照；起点假设视图使用独立请求级评价器，禁止共享原始视图的估计缓存，也禁止将其结果写回基础地图。

首轮沿用当前 0.20 m、0.03 m、0.20 的观测参数，先验证结构分离；它们是初始工程值，不借本次重构扩大所有容差。输入/参数非法时构造失败，不生成可通行结果。参数若后续可配置，必须纳入 profile 身份与增量失效判断。

## 重点审查的五类输入

1. 完全相同的连续坡面，轮式爬不上但足式可通过：地面估计一致，通行结论不同（任务 1/2）。
2. 孔洞落在窗口/瓦片边界：估计不随父窗口、查询顺序或分块顺序变化（任务 1）。
3. 粗糙连续地面与真实台阶/坑边混合：平滑不能消除真实障碍（任务 3）。
4. 跨估计邻域的新增观测、地图扩边和零代价权重：增量结果等于全量（任务 4）。
5. 起点假设地形及长期高程区间：假设不污染原始证据；区间端点仍参加误差/障碍检查（任务 1/4）。

## 任务 1：统一每格地面估计与缓存

- [x] 在新测试文件构造 41×41、5 cm 平地/斜坡，分别保留全部观测及每隔四格一个观测。先记录当前同格不同父窗口与平台坡度耦合探针结果。
- [x] 编写以下行为测试，确认旧实现不具备统一估计接口，再实现估计器。

```cpp
// 使用上述稀疏斜坡原始快照 raw；两实例绑定同一个不可变视图。
TerrainObservationEstimator first(*raw, {}), second(*raw, {});
const auto expected = first.Estimate({18,18});
(void)second.Estimate({19,19});
const auto actual = second.Estimate({18,18});
ASSERT_TRUE(expected); ASSERT_TRUE(actual);
EXPECT_DOUBLE_EQ(expected->surface_height_m, actual->surface_height_m);
EXPECT_DOUBLE_EQ(expected->confidence, actual->confidence);
EXPECT_FALSE(raw->ElevationRangeAt({18,18}));
```

- [x] 目标格使用自身确定的物理邻域，与调用它的父格无关。对原始点拟合局部平面，计算去坡残差及连续分数；估计点不参与下一次原始样本计数。
- [x] 保留样本数、空间覆盖、非退化、残差和距离限制；估计是否可靠不读取平台最大坡度。
- [x] 测试重复查询只计算一次，原始视图与假设视图缓存隔离；无观测、共线、单侧、非有限值不产生有效估计。
- [x] 输出确定性与计算次数证据。本步不声明真实帧 UNKNOWN 已解决。

## 任务 2：平台评价与地面估计解耦

- [x] 在 `terrain_window_test.cpp` 添加 0.45 rad 密集/稀疏平面回归：相同地面估计，轮式超过 20° 返回 BLOCKED，足式低于 30° 返回 FREE；分辨率与包络保持不变。
- [x] 将补足调用切换到估计器；拟合坡度和残差由观测阶段提供，平台评价只比较自身限值。保留原始高差证据的读取，避免只看平滑高度。
- [x] 明确分类顺序，写入代码与测试：

```text
有明确且充分的原始障碍/越限证据 → BLOCKED
无可靠地面且无明确障碍证据     → UNKNOWN
可靠地面、满足平台物理约束     → FREE + 连续可信度
```

- [x] 复核原有支持台阶、单点突起、超限坡面、车底净空测试。预测地面不得把实测越限点改成 FREE。

## 任务 3：将连续粗糙地面与断层支持判据分开

- [x] 先补反例：稳定坡面叠加小幅噪声、缺测孔洞、明确双层台阶、窄障碍、下陷；验证状态与分数，不只比较 FREE 数量。
- [x] 对满足观测一致性的连续地面，用拟合趋势表示地面变化，残差主要降低可信度；真实高差仍先与平台硬约束比较。
- [x] 仅当证据指向不同高程表面、且需要证明表面支持时使用原有支持检查。取消“任何残差超过 12.5 mm 都必须找到相邻高低格子对”的统一入口。
- [x] 局部单平面无法解释的区域保留未知或由原始障碍证据判阻塞；不增加复杂多曲面重建，不跨断层插值。
- [x] 验证以下约束：连续噪声地面可以降低分数而非一律 UNKNOWN；原有可通过台阶保持可通过，超限台阶保持不可通过；孤立突起没有支持时不能靠平滑变 FREE。

这一阶段的结果必须独立于任务 1/2 对照。若连续性与高差证据不能可靠区分，保留 UNKNOWN 并记录，不通过修改平台限值使测试转绿。

## 任务 4：整机汇总与增量依赖

- [x] `FineCellEvaluator` 继续使用原包络，障碍优先，未解未知保持未知；FREE 可信度取覆盖范围内最低值，代价仍为 `原代价 + confidence_weight × (1 - c)`。
- [x] 根据实际调用链计算依赖范围。保守初始上界：

```cpp
influence_m = hard_radius_m + preferred_clearance_m +
              std::sqrt(2.0) * (feature_radius_cells +
                                estimator_radius_cells) * resolution_m;
```

- [x] 把真实原始障碍读取范围也纳入上界；地图扩边仍使用原边界更新机制。逐格核对全量/增量状态、代价和可信度，不能只检查计数。
- [x] 使用瓦片边界外的新增观测、零 confidence_weight、独立地图同 revision、假设视图切换构造失效回归。
- [x] 起点假设保持请求级来源标记；终点/足式支持/盲区出口仍读取真实原始证据。检查路径简化仍保留可信度代价偏好。

## 任务 5：三分支同步与验收

- [x] 足式分支完成估计器与评价分离；将观测/缓存/整机公共部分按差异同步实车轮式、TCP 轮式，保留各分支已有平台实现差异。
- [x] 新增 CMake 源文件与 gtest 目标后执行独立构建。沿用已核实的本机 core 构建目录：

```bash
cmake --build /home/kai/CodexDownloads/lunar_navigation/search-optimization-deployment-sync-20260923/orin-legged-vehicle -j 4
ctest --test-dir /home/kai/CodexDownloads/lunar_navigation/search-optimization-deployment-sync-20260923/orin-legged-vehicle --output-on-failure
```

- [x] 对实车轮式、TCP 轮式各自同名构建目录执行上述构建/CTest，包测试串行运行；不要混合不同分支库。ROS 包重新构建并装入各自隔离前缀，补齐实际使用的 Jazzy 消息库路径，再执行 ROS CTest。
- [x] ASan/UBSan 运行新估计器、地形、整机增量和请求起点测试；保存失败与最终通过日志。
- [x] 固定当前 P3 float32 输入、配置和时间帧，对照补足率、各层 UNKNOWN 原因、整机 FREE、实际规划结果；不是仅绘制点云。生成同坐标 P3/P4/可信度图。
- [x] 九类既有地图与大图按相同尺寸/分辨率执行至少 10 次预热后采样，记录 p50/p95、估计计算次数和内存。应证明重复估计消除且未因依赖范围扩大而引入未解释的整体性能回退；不能预先承诺某个加速比。
- [x] 更新 `docs/validation/2026-09-24-terrain-observation-separation.md`，列出与当前版本同条件对照、所有未通过项和 NOT_RUN。原有历史验证记录不覆盖。
- [x] 形成可分别审查的差异：估计器与必要接线、连续地面判据调整、增量/性能验证。整理提交清单供后续提交授权；本方案不自动提交或合并主线。

## 完成标准

同一原始证据产生唯一地面估计；平台不同只影响通行判断；噪声残差与真实高差分别有回归保护；全量/增量一致；规划消费连续代价；三分支职责一致。真实帧改善幅度以结果为准，不能预设机器人附近必然连通。本机结果与 Humble/Orin/真实车辆验证明确分开。
