# Orin 足式部署与地形评估适配记录

日期：2026-09-22。分支 `app/orin-legged-vehicle`，源基线
`app/orin-real-vehicle@3fceeb74d84205408c04064c146dbf069afbd26a`。

## 问题、决策与实现

基线实车与 `app/orin-unreal-simulation@88065f7b` 的地图评估核心相同。轮式已有
5×5 背景面／3×3 残差模型；足式仍按相邻高差计算坡度，能力内台阶被提前阻断。
本次将地形测量提取为共用模板，足式以自身坡度／跨阶阈值和局部支持要求解释测量。
轮式公式和决策保持不变；足式使用背景导数中位数，避免拟合细化将台阶当坡面。

足式显著残差的两侧要求相邻支持，即使高度低于跨阶阈值，孤立返回也不能认证落脚。
5 mm 数值尺度对应 12.5 mm 支持带宽上限；不放大为跨阶高度的固定比例。
未知中心或局部支撑不足保持 UNKNOWN。增量重算半径由一格改为两格对角范围。
足式部署 YAML、安装清单、导航／探索默认入口和操作文档同步更新。

不改变足式有向边检查、轮式规划器、控制器或外部接口。足式能力沿用 Quad48 配置，
未校准为实机能力。完整三维机身／落脚／步态执行认证不在本实现中；详见
[部署说明](../ORIN_LEGGED_DEPLOYMENT.md)。

## 可复现验证

环境：本机 ROS 2 Jazzy，GCC 13，Release，独立工作树。所有构建和日志位于仓库外：
`/home/kai/CodexDownloads/lunar_navigation/orin-legged-20260922/`。

- 修改前核心 CTest：17/17，通过，日志 `baseline-ctest.log`。
- TDD：新增基础测试在旧实现上 4 个失败，日志 `legged-red.log`；能力内台阶、坡面残差、
  孤立返回均能重现问题。
- 新 5×5 评估加旧一格重算半径：增量与全量比较失败，日志 `halo-red.log`；扩大半径后通过。
- 支撑分组带宽采用跨阶阈值比例会误放行孤立 5 cm 返回，日志 `support-red.log`；
  加数值带宽上限后通过。
- 原有代价／几何增长测试使用孤立高点作为可通行或确定障碍，与新支持语义不符。
  改成有相邻支持的 2×2 地形，保留饱和代价、BLOCKED 优先及增量／全量一致性断言。
- 基础测试覆盖 0.1／0.2 m 分辨率、正负台阶、斜坡上台阶、连续对角坡面、孤立返回、
  缺失支撑。贯通测试使用 Quad48 平面包络，经真实地图派生和局部规划，能力内台阶找到
  路径，超限台阶和未知沟隙无路径。轮式地形测试保持原断言。
- Jazzy 构建：8 个依赖／运行包成功，`ros-build.log`；既有
  `pure_plan_motion_server.cpp:1179` 有 ROS Time 显式构造警告。
- 三个受影响包（增量核心、增量 ROS、探索 ROS）串行测试，`ros-test.log`；
  `colcon test-result` 汇总为 639 tests、0 errors、0 failures、0 skipped（最终隔离域重跑结果见 `colcon-final-result.log`）。
- 部署配置、接口、隔离清单、launch 与脚本相关 Python 最终测试：54 passed，`deployment-final.log`。
- 实际启动发现探索 loader 拒绝导航的 `start_blind_zone_margin_m` 字段。新增测试先失败
  （`exploration-config-red.log`），修复为允许该字段且严格校验有限非负／重复键。
  未知键继续拒绝，字段不改变探索几何。修复后导航和探索实际启动，均读取
  `legged_orin.yaml`，导航反馈为 true，证据 `config-smoke.log`／`config-smoke-launch.log`；
  使用 localhost、独立域 183，不提供地图、不发送目标、不启动车辆控制。
- 探索包重跑曾在 `UnchangedApproachStallDoesNotResubmitOrComplete` 等待消息超时；
  隔离到 localhost 域 184 后该测试组 2.47 s 通过，再整包串行复核。保留首次失败，
  不将一次隔离复跑推断为所有 DDS 环境稳定。
- 仓库 Python 全量首次运行：8 failed、322 passed、5 skipped，`pytest-tests.log`。
  其中本次默认值／文件清单断言已修正；未 source 新 overlay 的两个 legacy 安装运行测试
  在 source 后通过。`start_all` 捕获子进程命令有一次竞争失败，单测及后续全量通过。
- source 本次安装 overlay 后，排除已阻塞的旧 Humble 下载测试重跑：
  **333 passed、2 failed、1 skipped、1 deselected**，`pytest-final.log`。
  未把仓库全量标为通过。剩余项：
  - `test_rank_and_failure_ownership_contract`：文本断言仍要求旧枚举集合，在原实车
    分支同样失败，见 `baseline-python-failures.log`；本次不修改探索失败原因语义。
  - `test_no_path_feedback_replaces_the_candidate`：轮式 demo 未观测到预期候选替换，
    输出一直为 CANCELED；本轮未解决、未证明由足式修改造成，保留回归风险。
- `test_installed_launch_has_complete_live_graph_in_no_mount_humble_container` 在临时
  容器的 apt-get update 阶段持续约四分钟，尚未开始构建；本次主动终止该容器内的
  下载进程，测试记录为失败，后续重跑明确 deselect。不能作为 Humble 通过证据。

运行命令（在本分支根目录；先 source Jazzy，产物不得混入 Humble）：

```bash
source /opt/ros/jazzy/setup.bash
artifact=/home/kai/CodexDownloads/lunar_navigation/orin-legged-20260922
CMAKE_BUILD_PARALLEL_LEVEL=4 PYTHONNOUSERSITE=1 colcon --log-base "$artifact/ros-log" build \
  --base-paths ros2_ws/src --build-base "$artifact/ros-build" \
  --install-base "$artifact/ros-install" \
  --packages-up-to lunar_incremental_navigation_ros lunar_pure_exploration_ros \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source "$artifact/ros-install/setup.bash"
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 colcon --log-base "$artifact/test-log" test --base-paths ros2_ws/src \
  --build-base "$artifact/ros-build" --install-base "$artifact/ros-install" \
  --packages-select lunar_incremental_navigation_core lunar_incremental_navigation_ros lunar_pure_exploration_ros \
  --executor sequential --ctest-args -j 1 --output-on-failure
colcon test-result --test-result-base "$artifact/ros-build" --verbose
```

## 发布与证据边界

只创建本地足式分支。原实车、TCP 部署、integration 分支未移动；未推送、合并或部署设备。
通用算法改动需要后续按 integration 的独立验证与合入流程同步，本次不直接修改该工作树。

Humble 足式构建／运行、原生 Orin、真实 DDS／P3 地图、足式执行器和实机闭环均为
`NOT_RUN`。本机合成地图测试不替代真实地形与运动能力验证，未建立 Orin 性能或 p50／p95 数据。

## 同日追加：360° 传感器视场与平台比较

按用户要求，将足式分支 `config/orin_hardware.yaml` 和
`config/exploration_navigation.yaml` 的 `sensor_fov_deg` 从 90 改为 360；距离仍为 10 m。
不修改 legacy/demo 的独立配置、候选 yaw 偏移、终点朝向策略或 P3 驱动。算法已有完整圆周
分支，仍执行遮挡检查；360° 不代表实际地图已覆盖四周。足式与轮式的实现比较写入
`docs/ORIN_LEGGED_DEPLOYMENT.md`。

本次配置变更验证（同一 Jazzy 隔离产物目录）：

- `fov360-python.log`：配置、launch、接口相关 28 项检查通过。
- `fov360-build.log`：两个受影响 ROS 包重新构建、安装成功；已核对安装 YAML 为 360°／10 m。
- `fov360-tests.log`：两个受影响 ROS 包在 localhost 域 184 串行测试通过。
- `test_information_gain`、`test_visibility_traversal` 两组 CTest 通过，覆盖圆周视场与遮挡。
- `fov360-smoke.log`：localhost 域 183 实际启动导航及探索，读回视场 360.0、距离 10.0，
  两节点使用同一足式配置；未发送目标或车辆控制。

本次仅运行与配置变更相关检查，不重复上一阶段的全量仓库测试；上文未解决的全量回归项和
Humble／Orin／实机 NOT_RUN 边界继续保留。需要重启目标机探索节点才能读取新配置，
本次没有远程部署或重启设备。

## 同日追加：净空权重、3 m 起点余量与跟踪器分析

按用户最新要求，两个足式导航入口显式设置 clearance_weight=2.0；legged_orin 的
start_blind_zone_margin_m 从 0 改为 3.0。上述早期 0 m 描述保留为历史决策记录，
当前生效值以本节和部署文档为准。足式净空距离仍为 0.3 m，360°／10 m 配置保留。
更新现有配置加载测试的预期值为 3.0。跟踪器源码分析见 ORIN_LEGGED_TRACKER_ADAPTATION.md；
未修改控制器实现，也未启动车辆命令发布者。

同一隔离产物目录的证据：

- clearance-build.log：两个 ROS 包构建、安装成功。
- clearance-python.log：配置／launch／接口 28 项检查通过。
- clearance-tests.log：导航包测试通过，包含 Orin 足式配置加载及 3.0 余量断言。
  探索包 SuppressedCandidatesAreGrayAndActiveCandidateStaysRed 首次出现状态计数断言失败；
  测试等待候选抑制计数后立即读取另一个状态消息，存在异步到达时序的可能，但未确证根因。
- clearance-retest.log：同一环境单独重跑 test_incremental_exploration_node 整组通过；
  不修改该测试，也不将复跑通过记为问题已修复，首次失败保留在 test-log 对应运行日志中。
- clearance-smoke.log：实际启动导航／探索读回 clearance_weight=2.0、足式部署配置路径、
  跟踪反馈开启、视场 360°、距离 10 m；未发目标或控制命令。

Humble、原生 Orin、真实底盘与足式闭环仍 NOT_RUN；本次没有远程部署。之前全量测试的
未解决项继续保留，本次不宣称全量回归全绿。
