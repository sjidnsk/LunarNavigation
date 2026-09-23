# Orin 足式部署

本分支 `app/orin-legged-vehicle` 从实车基线 `app/orin-real-vehicle` 的
`3fceeb74d84205408c04064c146dbf069afbd26a` 分出。使用 `incremental_v2` 足式导航与探索，
保留 P3 地图／定位、私有 TF、QoS、执行反馈，不引入 TCP。

## 配置与启动

`config/orin_hardware.yaml` 选择 `legged` 和相对路径 `legged_orin.yaml`；路径相对于
配置文件所在目录解析。导航与探索使用同一足式配置。通用增量入口
`config/exploration_navigation.yaml` 也默认足式；`scripts/orin/start_navigation.sh` 和
`start_exploration.sh` 默认读取实机配置。显式切换平台时必须同时提供匹配的配置文件。

`legged_orin.yaml` 继承 Quad48 配置：机身 0.68 × 0.33 m，最大坡度 30°、台阶 0.5 m、
间隙 0.3 m，前后速度上限 1.5 m/s、横向 0.8 m/s。这些是既有配置上限，不是此次测得的
实机性能或建议执行速度。未知区域仍不可通行。起点盲区额外余量按最新要求设为 3 m；它是机身包络半径之外的余量，
不是无条件放行半径。原有起点补图的支撑与出口认证仍保留。

在目标 Orin 的本分支源码根目录构建（本次目标环境 NOT_RUN）：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --merge-install \
  --packages-up-to lunar_incremental_navigation_ros lunar_pure_exploration_ros lunar_pure_wheeled_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cd ..
```

先启动 P3 定位、高程图和真实静态 map→odom，然后两个终端分别运行：

```bash
bash scripts/orin/start_static_tf_input.sh
bash scripts/orin/start_hardware_navigation.sh
```

与足式 P3 `start_dog.sh` 对齐，默认域 69，使用本包 `config/cyclonedds_p3.xml`；所有正式入口自动加载同一环境。网络及运行目录覆盖方式见
[静态 TF 说明](STATIC_TF_INPUT.md)。这两个入口只启动适配与导航，不启动控制器或发送目标。
足式基础跟踪入口复用现有控制器，消费版本化参考并回传匹配反馈，启动方法见下节。
用户确认真实足式驱动尚未接入；当前仅验证标准 Twist 与本地理想平面模型。

需要探索时，在已设置相同域、RMW 和 CYCLONEDDS_URI 的另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=69
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds_p3.xml"
source scripts/orin/p3_environment.sh
ros2 launch lunar_pure_exploration_ros exploration_navigation.launch.py \
  config_file:="$PWD/config/orin_hardware.yaml" \
  stack_mode:=incremental_v2 \
  start_navigation:=false start_exploration:=true start_rviz:=false
```

核对运行配置与真实样本：

```bash
ros2 param get /incremental_navigation platform_type
ros2 param get /incremental_navigation platform_config
ros2 param get /incremental_navigation enable_tracking_feedback
ros2 topic info -v /Car/T3/mapping/grid_map
ros2 topic echo --once /Car/T3/mapping/grid_map
ros2 topic echo --once /Car/T3/localization/odometry
```

预期 `legged`、`legged_orin.yaml`、执行反馈 `true`。话题发现不能代替地图、规划及到达闭环。
本分支实机与通用 incremental_v2 配置的传感器视场为 **360°**，距离仍为 **10 m**。
该参数进入探索信息增益的全向可见性计算，遮挡仍有效；它不修改 P3 传感器驱动、实际
高程图覆盖或 legacy/demo 的独立传感器模型。候选朝向偏移仍为五个，终点朝向要求仍开启，
因为视场全向不等于机身几何或导航终点朝向无关；本次没有修改候选策略。

轮式配置、legacy 入口和 `scripts/start_all.sh` 保留历史用途；后者会启动轮式控制器，
不是本分支的足式部署入口。

## 地图评估与边界

共用 5×5 背景观测窗口：至少 15 个有效格、中心两侧观测、双轴相邻差分与非退化拟合。
足式另外保留中心 3×3 完整观测要求，不将未知格拟合成地面。足式使用导数中位数得到的
背景坡度，避免最小二乘细化朝另一台阶面倾斜；轮式保持原来的稳健加权拟合坡度。
以中心 3×3 原始高程上下界相对背景平面的残差判断台阶起伏。显著起伏需要两侧相邻支持，
孤立异常返回 UNKNOWN。12.5 mm 是从 5 mm 数值噪声尺度派生的支持分组带宽上限，不是跨障阈值。

5×5 是观测窗口，不是落脚范围：0.1 m 栅格对应 0.5 m 宽，0.2 m 对应 1.0 m。
机身平面包络及原有有向边支撑、高差、间隙、动作跨度检查继续独立执行。
增量更新的影响范围包含两格对角半径，使窗口边缘变化能更新派生图。

不使用轮式底盘离地间隙裁剪足式跨阶高度。当前 2.5D 图与边检查不等于三维机身碰撞、
具体落脚点、步态稳定性、悬空障碍或真实跨沟认证；配置含 `minimum_body_clearance_m`
也不证明已实现这些能力。未知沟隙仍保守拒绝，本次没有扩大跨未知区域能力。

验证见 [足式地图与部署回归](validation/2026-09-22-orin-legged-terrain.md)。

## 足式与轮式的实现差异

以下比较当前 incremental_v2 实现及两条 Orin 部署配置，不泛化到 legacy 规划器。

| 层级 | 轮式 | 足式 |
| --- | --- | --- |
| 背景坡度 | 5×5 背景初始化后采用稳健加权最小二乘细化坡度 | 同一观测窗口，使用背景导数中位数坡度，避免台阶倾斜拟合 |
| 观测与支撑 | 5×5 至少 60% 覆盖并满足双轴条件，允许部分局部缺测 | 还要求中心 3×3 完整观测；显著残差即检查两侧相邻支持，孤立返回 UNKNOWN |
| 起伏判断 | 残差起伏和相对中心向上凸起分别受障碍起伏／底盘离地间隙约束 | 残差按跨阶高度判定，不能拿轮式底盘间隙替代跨阶上限 |
| 局部搜索与边检查 | 平面 8 邻域，检查派生 FREE、起步阶段及对角防穿角 | 同样平面搜索，另有动作跨度、线段覆盖格、原始高差／间隙的有向边认证及缓存 |
| 终点朝向 | 检查是否支持旋转；不支持时要求入射方向满足目标朝向 | 使用有限的旋转基元组合认证目标朝向；不是 32 个 yaw 状态的空间搜索 |
| 净空软代价 | 默认 clearance_weight=2.0，偏好远离障碍 | 部署显式 clearance_weight=2.0，与轮式一致；机身硬包络检查仍保留 |
| 起点盲区配置 | wheel_orin 额外余量 3 m | legged_orin 额外余量 3 m；均保留起点补图认证 |
| 执行器 | 仓库已有轮式路径跟踪控制器 | 新增低速 vx／omega 跟踪入口；横移未实现，足式底层驱动未接入 |

共同部分包括高程持久化、粗全局引导、全局规划、滚动会话、前沿探索、信息增益、路径简化
及正式 ROS 接口。二者输出都是平面路径参考，不是足端轨迹。框架内默认到点容差存在平台
差异，但两条实车 YAML 都显式覆盖为 0.1 m／0.05 rad，因此这不是当前实车入口的有效差异。
机身包络生成方式不同（轮式配置多边形／足式由机身长宽生成矩形），之后都使用外接圆进行
精细图硬膨胀。足式尚无完整三维机身碰撞与落脚／步态认证。

代码依据：`src/map/platform_elevation_physics.cpp`、`src/wheel/wheel_local_astar.cpp`、
`src/legged/legged_local_planner.cpp`（均在 `lunar_incremental_navigation_core`），以及
`lunar_incremental_navigation_ros/src/incremental_navigation_node.cpp` 的 `PlatformProfileFor`。

净空权重一致后，平台自身的净空距离仍分别使用足式 0.3 m、轮式 0.2 m，
因此不代表两个平台的净空代价完全相同。正式增量入口与节点缺省值同步为权重 2.0；通用配置也选择 legged_orin.yaml。
跟踪器复用边界见 [足式跟踪器适配分析](ORIN_LEGGED_TRACKER_ADAPTATION.md)。

## 足式基础跟踪入口

单独启动控制器前，设置与导航相同的 ROS_DOMAIN_ID、RMW_IMPLEMENTATION 和 DDS 配置。
导航脚本不会自动启动它；同一命令话题只运行一个控制器。目标机驱动尚未接入，以下是
后续联调入口，不是已完成实机部署的声明：

```bash
bash scripts/orin/start_legged_controller.sh
```

配置为 `config/orin_legged_controller.yaml`，能力文件为 `config/legged_orin.yaml`。
前进上限 0.1 m/s，角速度上限 0.15 rad/s，与轮式一致默认允许倒车，横向输出始终为零。
线加速度 0.1 m/s²、计划制动减速度 0.15 m/s²、角加速度 0.15 rad/s²，均为初始控制配置，
不是足式能力表中的实测制动能力。紧急停止及失效路径仍立即输出零速，底盘实际停止响应
须在驱动接入后验证。里程计接收超时 0.5 s、TF 接收超时 5 s，与轮式部署配置一致；输入需持续更新。

节点名与包名继续沿用 `lunar_pure_wheeled_controller`；`platform_type=legged` 使用足式
能力字段并要求 `incremental_reference` 模式。默认轮式入口保留。低速限制来自专用配置文件，
手动运行通用节点时必须加载它，不能只传足式能力文件。

本地完整导航验证入口（仅在隔离域运行，先 source 对应 ROS 与构建 overlay）：

```bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python3 tests/launch/legged_tracking_probe.py \
  --artifact-dir "$HOME/CodexDownloads/lunar_navigation/legged-tracking-probe"
```

该脚本用合成平面高程图及理想速度积分模型连接真实导航节点与跟踪器，隔离话题均位于
`/legged_tracking_probe/`。不代表腿部动力学、真实定位质量或真实底盘通过验收。

终点朝向预算：导航 0.05 rad、足式规划认证 0.025 rad、跟踪器 0.02 rad。修改任一项时
需保持规划误差与跟踪误差之和小于导航阈值；离散旋转原语不能达到严格窗口时仍允许 NO_PATH。
完整闭环证据见 [足式低速跟踪验证](validation/2026-09-22-orin-legged-tracking.md)。


## 2026-09-23 参数统一

正式增量导航的硬件、通用、直接节点入口统一为：tracking feedback 开启、净空权重 2、
起点盲区额外余量 3 m、位置容差 0.1 m、导航朝向容差 0.05 rad、私有 TF 输入。
平台选择对应的 `_orin.yaml`，显式指定不匹配的平台／能力文件仍拒绝启动。
足式传感器视场保持 360°，轮式保持 90°；物理能力数值仍按各自平台配置。

轮式和足式跟踪器统一：前视 0.5 m、偏离阈值 1 m、TF 超时 5 s、允许倒车、跟踪位置
容差 0.1 m／朝向容差 0.02 rad，两种 launch 均自动加载各自部署 YAML。
前视恢复到 0.5 m 解决已复现的 5 cm／10 cm 横向偏差失败；曲率上限仍为 1/m，因此
1 m 偏离阈值不是能纠正全部 1 m 偏差的保证。低速上限仍是足式自己的 0.1 m/s 和 0.15 rad/s。

足式区间支持修正为同类区间端点比较，完整平地 ±13/20 mm 区间可通过。中央 3×3 完整
观测继续防止将未知落脚区拟合为地面；中位数坡度继续避免台阶被拟合为陡坡。
这两项是观测证据与地形分离策略，不是认为足式物理越障能力弱于轮式。

本轮证据见 [参数统一验证记录](validation/2026-09-23-deployment-policy-unification.md)。

跨平台启动时，显式配套指定 `platform_type` 与 `platform_config`，例如
`platform_type:=wheel platform_config:=$PWD/config/wheel_orin.yaml`。单独改变类型但仍显式指定
另一平台能力文件时会报错，不会静默替换。传感器 FOV 仍服从所选 `config_file`；跨平台
使用时应选择该平台配置文件。旧 `scripts/start_all.sh` 继续显式使用 legacy motion reference 与 `/tf`。

P3 源码接口核对、数据流与完整联调命令见 [足式 P3 联调说明](P3_LEGGED_INTEGRATION.md)。此前终端若已导出域 59，需显式改为 69；启动脚本保留人工覆盖，不会偷偷改写。
