# 2026-09-23 足式与轮式部署策略统一

## 基线与范围

足式基线 `dc0b144a`（`app/orin-legged-vehicle`），轮式基线 `3fceeb74`（`app/orin-real-vehicle`）。
本轮落实七项审查反馈：修正足式高程区间支持判据；统一非物理能力配置和入口；修复轮式探索加载器的盲区字段兼容。
足式尺寸、地形能力、低速上限不变；足式 FOV 360°、轮式 FOV 90°。尚未接入真实足式驱动，不增加横移或步态接口。

## 修改与理由

| 项目 | 本轮结果 |
| --- | --- |
| 平坦高程区间支持 | 足式支持判定用去背景后的 low/high 端点匹配对应极值，不再用中值匹配区间极值。±13 mm、±20 mm 的连续平坦区间由 UNKNOWN 改为 FREE；孤立极值仍 UNKNOWN，超坡度/台阶能力仍拒绝。轮式地图判据不变。 |
| 前视与偏离 | 两平台均 lookahead=0.5 m、max_cross_track_error=1 m、max_curvature=1/m。足式旧 0.3 m 前视导致 5 cm 偏差曲率超限。恢复 0.5 m 后，5/10 cm 偏差通过理想跟踪闭环。1 m 只是拒绝阈值，不代表可纠正 1 m。 |
| 朝向预算 | 两平台正式局部规划器显式使用导航容差的一半（默认 0.025 rad）；控制器 0.02 rad；导航 0.05 rad。规划加跟踪 0.045 rad，留有余量。保留底层结构体默认值用于非部署调用，部署不再依赖它。 |
| 导航配置 | 硬件 YAML、通用 YAML、正式节点默认：feedback=true、位置 0.1 m、yaw 0.05 rad、clearance=2、blind-zone fallback=3 m、TF=/P4/input/map_to_odom。blind margin 是包络外余量，仍需起点补图认证。 |
| 控制配置 | wheel/legged launch 自动加载各自部署 YAML，显式参数覆盖最后生效；前视 0.5 m、偏离 1 m、TF timeout 5 s、allow_reverse=true、位置 0.1 m、yaw 0.02 rad。保留各自速度/加减速能力。 |
| 地形仍有差异 | 中央 3×3 完整观测是支撑证据要求，不能由运动能力更强推出缺测可通行。相邻导数中位数使分层台阶不被最小二乘拟合成陡坡；已有台阶/坡度测试约束此行为。保留此差异，本轮只修不合理的区间匹配。 |
| 旧探索加载器 | 可选 start_blind_zone_margin_m 接受 0/3 m（两平台），仍拒绝负值、NaN、重复键和其他非法字段。轮式实车已有 wheel_orin.yaml 可正常加载。 |

纯追踪直线路径、静止且朝向沿路径时，曲率近似 `2e/(L²+e²)`。
`L=0.3,e=0.05` 时约 1.081/m，超过 1/m；`L=0.5` 时约 0.396/m。
恢复前视解决已复现的小偏差问题；此条件下可行横偏上界约 0.134 m，尚不支持“1 m 可纠偏”的声明。
若以后需要更大偏差恢复，应另行验证停转重对齐或自适应前视，不能仅增大偏离拒绝阈值。

## 入口与兼容

正式节点自动解析已安装的对应 `*_orin.yaml`；探索包也安装两平台部署能力文件。
命令行 `platform_config` 的优先级为：非空 CLI > 显式 YAML common.platform_config > 自动选择。
跨平台启动须匹配 `platform_type:=wheel platform_config:=$PWD/config/wheel_orin.yaml`；显式不匹配仍报错。
FOV 服从显式配置文件，不随 selector 偷偷改写。只切换 selector 不能代替选择完整平台配置。
旧 start_all.sh 明确保留 motion_reference、/tf 及原跟踪参数；RViz 位姿反馈 demo 明确关闭不具备的 tracking feedback。
轮式分支为共享控制启动代码安装了足式可选配置，但默认仍为轮式；足式地形实现的修正只在足式部署分支。

## 本地验证

环境：本机 ROS 2 Jazzy、localhost 隔离 DDS 域。原生 Humble / Jetson Orin / 真实车辆均 NOT_RUN。
证据目录：`/home/kai/CodexDownloads/lunar_navigation/orin-legged-unify-20260923/`。

- RED/GREEN：terrain_red/green.txt、controller-red/green.txt、nav-defaults-red/green.log、wheel-yaw-red.log、margin-red.log、wheel-loader-red/green.log、launch-platform-override-red.log、legacy-controller-red.txt。
- 足式四包重新构建并执行 colcon test 通过；控制器独立源码测试 178 passed；根目录部署/launch/接口/脚本测试 70 passed；外部接口检查通过。
- 足式硬件与通用入口实际启动、导航/探索参数读回，两节点各自全量参数无差异（hardware/generic-*.yaml）。两控制器安装后启动参数符合统一值，命令输出重定向到隔离测试 topic（controller-readback.log）。
- 完整 DDS 理想平面闭环：两次 GOAL_REACHED，位置误差均 ≤0.1 m、yaw 误差均 ≤0.05 rad；一次运动中 CANCELED 后零输出。最大速度 0.1 m/s、0.15 rad/s、vy=0（tracking-probe/result.json）。
- 轮式分支独立构建 9 包通过；四个相关包 colcon test 通过，test-result 无错误/失败；控制器源码测试 178 passed，根目录部署/launch/接口/脚本测试 68 passed，外部接口检查通过（wheel-build.txt、wheel-test.txt、wheel-test-results.txt、wheel-root-python-tests.txt）。旧 planner 包有既有 Time 初始化编译警告，无构建失败。
- 轮式硬件与通用入口实际读回参数也完全一致，保持 wheel 身份与 FOV 90°（wheel-hardware/generic-*.yaml）；双树独立复核发现的配置覆盖与 legacy 输入回归已修复，复核测试各 31 passed。

本地理想模型不代表真实制动、打滑、步态或复杂地形执行能力。未验证真实地图中高程区间误判的出现频率。

## 复核入口（本地 Jazzy）

在足式源码根目录运行，先加载匹配安装产物；以下不是目标机部署命令：

```bash
source /opt/ros/jazzy/setup.bash
source /home/kai/CodexDownloads/lunar_navigation/orin-legged-20260922/ros-install/setup.bash
ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 python3 tests/launch/legged_tracking_probe.py \
  --artifact-dir /home/kai/CodexDownloads/lunar_navigation/orin-legged-unify-20260923/recheck-probe
python3 -m pytest -q tests/test_orin_hardware_profile.py \
  tests/launch/test_exploration_navigation_launch.py \
  tests/test_incremental_navigation_interface_contract.py \
  tests/test_external_interface_contract.py tests/test_orin_humble_operator_scripts.py \
  tests/test_operator_scripts.py
python3 tools/check_pure_planner_external_interfaces.py .
```

轮式独立构建位于证据目录的 `wheel-build` / `wheel-install`，足式使用既有 `orin-legged-20260922/ros-build` / `ros-install`。
两套产物分离；测试日志与 readback 脚本保存在证据目录。不要将此 Jazzy overlay 用于 Humble/Orin。
