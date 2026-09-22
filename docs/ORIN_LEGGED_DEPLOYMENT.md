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
实机性能或建议执行速度。未知区域仍不可通行。起点盲区额外余量为 0 m，不继承轮式的
3 m 扩张；原有起点补图仍有自身支撑与出口认证，0 m 余量不等于停用所有起点补图。

在目标 Orin 的本分支源码根目录构建（本次目标环境 NOT_RUN）：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --merge-install \
  --packages-up-to lunar_incremental_navigation_ros lunar_pure_exploration_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cd ..
```

先启动 P3 定位、高程图和真实静态 map→odom，然后两个终端分别运行：

```bash
bash scripts/orin/start_static_tf_input.sh
bash scripts/orin/start_hardware_navigation.sh
```

沿用实车域 59 与 CycloneDDS 配置；网络及运行目录覆盖方式见
[静态 TF 说明](STATIC_TF_INPUT.md)。这两个入口只启动适配与导航，不启动控制器或发送目标。
足式外部执行器仍须消费既有参考并回传匹配反馈，本次没有新增执行器或关闭反馈等待。

需要探索时，在已设置相同域、RMW 和 CYCLONEDDS_URI 的另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=59
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
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
