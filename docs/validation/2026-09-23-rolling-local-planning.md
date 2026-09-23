# 移动中滚动局部规划验证记录

## 范围与结论

分支 `feat/rolling-local-planning`，基线 `6868236f`；主要实现截至 `4d21601b`。
设计见 [设计文档](../superpowers/specs/2026-09-23-rolling-local-planning-design.md)，任务见 [实施计划](../superpowers/plans/2026-09-23-rolling-local-planning.md)。

实现了同一轮式任务在运动中更新局部路径，不再以到达旧局部目标作为下一次计算的前提。
正常更新失败保留仍可执行的引用；取消、地图失效和最终对齐能淘汰后台迟到结果。
控制器在同任务正常跟踪中采用新引用，按实时位姿重新定位进度；小幅曲率变化按现有限制过渡。
这不保证所有换路都不停：新路径需要超限曲率、急转、换向或路径已耗尽时，仍执行原有制动。

只对轮式 `incremental_reference` 启用。生产默认关闭，周期 `0` 保持旧调度和控制语义；未修改生产部署、探索策略、底盘协议或 ROS 消息。

## 职责与实现裁定

1. `RollingUpdatePolicy` 只管理周期、提前距离、忙碌和冻结。新增 `Published()` 消耗已进入提前距离的新引用，防止末端短路径不断触发立即刷新。
2. `ComputeCycle` 只计算候选；协调器串行拥有 session、revision 和活动路径。请求携带 tolerance/stop token，结果携带失败/等待分类，复用原语义。
3. ROS 单线程 worker 独占计算端口/缓存，最多一个请求和一个结果；忙碌期间合并更新意图，下一次捕获最新位姿。关闭模式使用同步包装，但计算函数相同。
4. 更新采用复用地图遍历。新候选完整遍历，避免自交时采用与控制器投影不一致的“最近后缀”；代价是刚驶过的候选前缀变为阻塞时可能保守拒绝一次。地图修订号不同本身不拒绝。
5. 控制器复用 `Polyline` 的投影范围，不增加连接段规划。曲率过渡只在滚动模式启用，保持线/角加速度、曲率和横向加速度上限；短时跟踪偏差继续受原有偏离检查约束。高弯曲或过时前缀仍可能需要恢复。
6. 无进展使用实际位姿变化，更新路径不能重置卡住证据。故障锁存时更新引用身份以便导航接收正确版本的 FAILED；等效最终引用不重启 FINAL_ALIGN。
7. 普通刷新在非 TRACKING 阶段暂停，失效/取消/恢复继续处理。提前距离优先使用匹配 session/revision 的执行弧长，缺失时回退 core 投影。
8. 接口清单只添加参数说明注释，保持既有严格 schema。诊断增加候选 request/session、触发类别、采用/保留/淘汰、版本和耗时；滚动 worker 不发布搜索内部调试标记，避免跨线程操作调试发布器。
9. 按仓库规则保留忽略的执行 ledger，不做递归清理；工作树保留，不合并或推送。

## 构建与自动化证据

环境：本机 Ubuntu 24.04、x86_64、ROS Jazzy；C++ Release，`BUILD_TESTING=ON`、`LUNAR_BUILD_DEMO=ON`。
Humble 使用本机 `phase2-humble-gridmap:local` 容器，源目录只读、网络禁用、独立 build/install/log。

| 检查 | 结果 | 原始证据 |
| --- | --- | --- |
| Jazzy 5 个相关包构建 | 通过 | `build-verified.log` |
| Jazzy core CTest | 17/17 测试组通过 | `core-verified.log` |
| Jazzy ROS CTest | 13/13 测试组通过，含 7 个滚动节点用例 | `ros-verified.log` |
| Jazzy 控制器、demo、launch Python | 194 项通过 | `python-verified.log` |
| Humble 5 个相关包构建、core、ROS | 通过；17/17、13/13 | `humble-verified.log`，栈条件见下文 |
| Humble 最终 Python 回归 | 194 项通过 | `humble-python-final.log` |
| 消息/接口/demo/launch 静态合同 | 39 通过、1 个既有失败 | `static-contracts-verified.log` |
| demo 速度 topic 发布者 | 只有 `controller_demo_executor`，订阅者为 vehicle plant | `command-publishers.txt` |

静态合同失败 `test_stack_config_launch_and_cpp_keep_the_exploration_map_contract_closed` 要求恰有一个 OccupancyGrid 发布者；基线 `6868236f` 的 `incremental_map_publisher.cpp:35,45` 已有两个。该发布逻辑未改，不将此次验证报告为全仓库测试全部通过。

Humble 默认 8192 KiB 栈下，未修改的 `PlanningSnapshotContractTest.PartialBoundaryTilesKeepCellsOutsideOldBoundsUnobserved` 出现 SEGFAULT；同一二进制在 32768 KiB 栈通过。最终容器验证显式使用 `--ulimit stack=33554432:33554432`，原始失败及对照保存在 `humble-snapshot-probe.log`。没有证明默认栈配置可通过。

覆盖包括：无解/超时保留旧路径，取消/失效/最终对齐的迟到结果，无关地图修订、新障碍和 unknown，自交错误投影，旧反馈，静止持续换路，无进展计时，失败身份，换向和最终停稳。
慢计算通过条件变量阻塞 fake solver；验证取消、地图失效无需等待计算完成，最大并发为 1、后继输入为最新位姿、析构可中断 worker。它们是确定性测试证据，不声称现场已出现并验证同样竞态。

一次独立只读审查发现 3 个 P1、1 个 P2：候选自交漏检、弧长重置掩盖静止、故障反馈身份过旧、最终对齐被在途引用重启。四项均接受并以失败回归验证后修复（`b98b5600`），无遗留审查项。随后闭环暴露的问题按所属模块修复并回归。

## 闭环方法与统计口径

隔离域 `179`、`ROS_LOCALHOST_ONLY=1`，每例重启 launch；没有连接车辆。
真实控制器 `/lunar_demo/controller/cmd_vel` 驱动 SE(2) 积分，plant 不读取路径来移动。
地图 32 m、分辨率 0.2 m，速度上限 0.2 m/s；滚动参数 0.5 s / 1.0 m，baseline 周期 0。
`rolling_straight` 为 6 m 无障碍目标、3 m 圆形合成观测；`multiple_exits` 为 12 m 目标、10 m 圆形观测。圆形观测不是 LiDAR 物理模型。

`*-verified.json` 是最终同版本比较，脚本 `run_cases_verified.py`；全量 events 保存单调时间、引用、路径点、执行阶段、速度、位姿和诊断。
`summary-verified.json` 由 `summarize_cases.py` 生成：规划 p50/p95 取真实搜索的周期总耗时（`local_elapsed_ms > 0`），排序后线性插值；横向误差 p95 只取 TRACKING 样本。
每场景每模式一次运行，耗时包含启动等待，样本量不足以推断稳定性能提升。

`update_stops` 是移动更新后 BRAKING/ALIGNING 的候选计数，排除最终引用末端 0.35 m 内的停稳；它不等于“无必要停车”。转弯、换向、几何超限需要结合原始事件复核。非最终 COMPLETED 计数单独反映原模式到局部目标停稳的次数。

| 场景/模式 | 总时间 s | 移动更新/引用数 | 非最终停稳 | 更新关联制动 | 周期 p50/p95 ms | 横向误差 p95 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| rolling_straight-baseline | 36.28 | 0/4 | 3 | 0 | 0.953/1.277 | 0.0079 |
| rolling_straight-rolling | 36.97 | 57/60 | 0 | 1 | 0.750/1.513 | 0.0076 |
| multiple_exits-baseline | 100.47 | 0/2 | 1 | 0 | 6.171/8.984 | 0.0079 |
| multiple_exits-rolling | 92.47 | 156/160 | 0 | 2 | 4.495/12.470 | 0.0039 |
| forward-rolling | 18.37 | 28/29 | 0 | 0 | 5.756/6.294 | 0.0000 |
| reverse-rolling | 18.47 | 28/29 | 0 | 0 | 5.622/6.752 | 0.0000 |
| final_yaw-rolling | 22.44 | 28/29 | 0 | 0 | 5.747/6.551 | 0.0000 |
| detour-rolling | 60.86 | 91/96 | 0 | 3 | 6.017/7.092 | 0.0067 |

8 例均达到目标且停稳，碰撞和原始线速度命令超限均为 0。最终位置误差 0.1545–0.1587 m；final_yaw 朝向误差 0.1445 rad（满足原有接受阈值）。


每份通过报告同时满足：PLAN_FOUND 与活动引用匹配、当前最终版本 COMPLETED、GOAL_REACHED、实际停稳、最终位置/要求朝向、零碰撞和原始线速度命令不超限；reverse 还要求实际倒车，局部观测场景要求出现非最终引用。

### 保留的失败和中间结果

- 初始 forward 滚动运行产生 324 个引用：提前阈值对新短引用反复触发，`Published()` 修复后约 29 个；早期日志未覆盖。
- `rolling_straight-rolling-final.json`：4 次更新关联制动、38.07 s。相同曲率必须一步满足角速度变化导致小幅方向变化刹停。正向/倒车失败测试复现后，增加滚动曲率过渡；`*-smooth.json` 记录 58 次移动更新、0 次候选刹停、33.59 s，对照 36.36 s。
- `rolling_straight-rolling-verified.json`：最终同版本复跑仍有一次移动更新关联制动。revision 36 的局部目标从观测边缘改变方向；制动前后相邻 plant 样本重建的 Pure Pursuit 曲率约为 -1.05 至 -1.26 m⁻¹，超过现有限制 1.0 m⁻¹。没有放宽限制、筛选掉该试次或改场景以消除统计。该试次不存在非最终段 COMPLETED，仍然实现运动中跨局部目标更新。
- 平顺窗口可零额外停车（`*-smooth` 全程、前进/倒车和最终朝向的跟踪阶段）；不能宣称局部目标在圆形观测边缘跳动时也永不刹停。降低这种目标抖动属于局部目标稳定性后续优化，本次未修改目标选择算法。
- detour/multiple_exits 的候选制动位于转弯窗口，原有曲率/拐点制动保留；不宣称复杂路线零停车。
- 会话中断留下的 v2 final_yaw 为部分证据；最终完整结果使用 verified。只停止本任务自己的进程组，未清理其他 ROS 实例。

## 复现与证据索引

所有证据位于 `/home/kai/CodexDownloads/lunar_navigation/rolling-local-planning-20260923/`，不提交大体积运行数据。
关键文件：`*-verified.json`、对应 `*-launch.log`/`*-recorder.log`、`cases-verified.log`、`summary-verified.json`、两个运行/汇总脚本，以及上表构建测试日志。
`*-final`、`*-smooth`、`*-v2` 保留修复过程；不要混用不同阶段数据宣称相同版本性能。

构建命令见实施计划。完成构建后，在本工作树的新 shell 中执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROLLING_ARTIFACT_ROOT="$HOME/CodexDownloads/lunar_navigation/rolling-local-planning-20260923"
source "$ROLLING_ARTIFACT_ROOT/install/setup.bash"
ctest --test-dir "$ROLLING_ARTIFACT_ROOT/build/lunar_incremental_navigation_core" --output-on-failure -j1
ctest --test-dir "$ROLLING_ARTIFACT_ROOT/build/lunar_incremental_navigation_ros" --output-on-failure -j1
PYTHONPATH="$PWD/ros2_ws/src/lunar_pure_wheeled_controller/python:$PYTHONPATH" python3 -m pytest -q \
  ros2_ws/src/lunar_pure_wheeled_controller/test \
  ros2_ws/src/lunar_incremental_controller_demo/test \
  tests/launch/test_rolling_local_planning_contract.py
```

单例运行和记录命令见 [操作说明](../操作指令.md#移动中滚动局部规划隔离轮式闭环)。批量复现时先修改 artifact 脚本输出后缀，避免覆盖本次原始报告。

## 环境边界

| 环境 | 状态 |
| --- | --- |
| 本机 Jazzy 单测/ROS 组件/控制闭环 | 已运行，见上述结果 |
| Humble 容器构建/测试 | 已运行，要求 32 MiB 栈；未运行 Humble 闭环 |
| 原生 Orin、目标机 DDS/QoS、TCP/Unreal、实车 | NOT_RUN |
| 生产配置启用、部署、合并、推送 | 未执行 |

本次结论是隔离环境的实现与回归证据，不是 Orin 或实车部署通过证明。

## 后续：默认开启与 RViz2（2026-09-23）

按用户要求，统一 demo 默认周期改为 0.5 s；轮式导航已启用 tracking feedback 时也默认 0.5 s，其他平台/未启用反馈时保持 0。控制器默认开启滚动适配，但只作用于 incremental_reference。统一 launch 显式传入周期 0 可同时关闭两端。上文“生产默认关闭”记录的是此前版本状态，本次修改未连接或部署真实车辆。

默认配置下重新构建 5 包；ROS 13/13 测试组通过，原 Python 回归 194 项通过，含新增默认值合同的 launch 测试 3 项通过（新增 1 项）。日志为 `default-build.log`、`default-ros.log`、`default-python.log`。

以未传周期参数的 `case:=multiple_exits start_rviz:=true` 启动域 179 的 RViz2 闭环。画面显示绿色局部路径、蓝色实际轨迹和当前 revision/速度；截图 `rviz-default-moving.png` 中可见 TRACKING、revision 17、实际速度 0.200 m/s、碰撞/超限为 0。RViz2 窗口保持打开供交互；进程组记录在 `rviz-default-processes.json`，启动日志 `rviz-default-launch.log`，完整结果 `rviz-default.json`。

本次默认值变更验证为本机 Jazzy；未重跑 Humble/Orin。RViz 有一次 Map shader sampler 报错，实际检查画面仍显示地形、路径、车体和状态，未把渲染日志声称为无错误。
