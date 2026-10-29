# VIPER 轮式工程适配实施计划

> **For agentic workers:** 使用 superpowers:subagent-driven-development，按任务顺序实现、独立审查和整合。不要并行写入同一工作树。

**Goal:** 保留 VIPER 外观，按项目轮式基线完成可运行控制与车头传感器适配，并在台式打开。
**Architecture:** 新规范物理骨架承载只读 VIPER 视觉；纯 Python 控制与感知计算独立于 SDK；Kit 内桥接真实物理/RTX 数据，主机 Humble 独立验证。
**Tech Stack:** Python、OpenUSD、Isaac Sim 6.0.1、ROS 2 Humble、pytest。
**Spec:** `docs/superpowers/specs/2026-10-28-isaac-viper-engineering-design.md`。

## Global Constraints

- 只在本专题工作树写源码；原集成目录、模型源、旧展示场景和训练保持不变。
- 所有模型、构建、运行日志、回执与预览在仓库外 D 盘或台式 `/mnt/data` 本任务目录。
- ROS_DOMAIN_ID=117、ROS_LOCALHOST_ONLY=1；不在真实车辆域发命令。
- 权威几何与运动约束读取 wheel_orin.yaml；质量50kg/双目14cm为用户确认的待标定仿真值。
- 传感器与杆在车头 local +X，真实传感器数据不得用代理数据冒充。
- 不合并、不推送、不创建 PR、不删除分支、工作树或历史产物；提交只含本任务显式路径。

## Review Focus

- wheelbase/track/径向与轴向缩放错误，特别是滚动时胎块超径。
- 零转向 footprint 与真实扫掠混淆、轮拱穿插、错误质量/惯性或遗留运行初态。
- Twist 单位/轮速符号、转向先行、停止切换、超时/NaN 不停车。
- Kit/Humble Python ABI 混用、ROS域串线、四元数顺序和光学坐标系错误。
- 相机空帧、点云超范围/自身遮挡、IMU无效读数、伪LiDAR或仅topic存在。

### Task 1: 配置与轮组执行策略

**Files:** Create `tools/isaac_viper/__init__.py`, `tools/isaac_viper/config.py`, `tools/isaac_viper/drive.py`, `config/isaac_viper.yaml`, `tests/isaac_viper/test_drive.py`, `tests/isaac_viper/test_config.py`。
**Consumes:** 原 wheel_orin.yaml 和用户冻结值。
**Produces:** `load_profile(path) -> dict`，包含 `capability`/`simulation`/`sensors`/`ros`；`DriveController(profile)` 的 `receive_twist(vx,vy,wz,wall_time)`、`step(dt,wall_time,steering_positions,body_speed)`，返回有 `steering_rad`/`wheel_radps`/`safe_stop`/`reason` 的四角命令，角顺序 fl/fr/rl/rr。

- [x] 先写行为测试并看到缺实现的失败：直行0.2m/s轮速约1.2539185rad/s，倒退符号相反；自转轮角约±0.884rad且滚动符号左右相反；弧线独立手算夹具。
- [x] 验证速度/曲率/加减速度、0.5秒超时、非有限值/非平面拒绝、停止切换和转向到位门控；验证源配置缺字段/不合理值拒绝及仿真假设标记。
- [x] 最小实现，运行 `python -m pytest tests/isaac_viper/test_drive.py tests/isaac_viper/test_config.py -q`；记录 RED/GREEN。
- [x] 仅提交本任务显式文件并写 task-1-report.md，供独立任务审查。

### Task 2: 规范物理与 VIPER 视觉

**Files:** Create `tools/isaac_viper/build_stage.py`, `tools/isaac_viper/validate_stage.py`, `tests/isaac_viper/test_stage_contract.py`。
**Consumes:** Task1 profile、只读原地形场景、source-assets/usd/payloads/instances.usda、材质替换记录。
**Produces:** `build_stage(profile, assets_dir, source_scene, output_path) -> dict`；工程 USD 路径 `/World/ProjectVIPER`，子刚体 `chassis`、`{corner}_steer`、`{corner}_wheel`；关节 `{corner}_steering_joint` 与 `{corner}_rolling_joint`；`chassis/FrontSensors/{left_camera,right_camera,imu,lidar}`，前置杆与外壳。

- [x] 先建立实际 USD 几何合同验证，让旧447kg/原尺寸/17关节展示场景明确失败；不以源代码文本断言代替生成结果检查。
- [x] 生成9刚体8关节规范初态、显式碰撞与惯性、复合轮拱；挂载实例视觉叶，不导入源物理。前后轮分别按真实径向最大值缩放，轮宽独立适配。
- [x] 所有非轮视觉及传感器外壳满足零转向名义包络与限高；前置雷达、双目、杆；轮转向扫掠单独验证和报告。
- [x] 在台式无GPU PXR环境生成/重开验证原文件哈希、18→9/17→8转换、轮尺寸/位置/碰撞、质量惯性、零速初态、关节关系及材质解析；记录命令和JSON结果。
- [x] 仅提交显式文件与测试，写 task-2-report.md，独立任务审查。

### Task 3: 物理、ROS 与传感器运行桥

**Files:** Create `tools/isaac_viper/sensors.py`, `tools/isaac_viper/ros_bridge.py`, `tools/isaac_viper/run_scene.py`, `tests/isaac_viper/test_sensors.py`。
**Consumes:** Task1执行策略与Task2结构；Kit自带Humble rclpy、CPU PhysX tensors、真实RTX Camera/LiDAR和IMU API。
**Produces:** 独立ROS域上的 canonical Twist/odom/TF/clock，以及配置定义的双目图像/CameraInfo/深度/有效点云/IMU/LiDAR；运行状态 JSON 与遥测。

- [x] 先写投影/范围/光学坐标/四元数/点云布局的独立数值夹具，看到缺实现失败，再实现纯计算部分。
- [x] 建立CPU物理 numpy articulation view，按DOF名字映射单位与目标，使用实际根姿态/速度反馈，不从USD静态属性伪造运行里程计。
- [x] 启用Kit捆绑Humble ABI及ROS隔离断言，订阅Twist和发布实际时钟/odom/TF；新节点不改其他ROS进程。
- [x] 前置相机/IMU/自建RTX LiDAR；获取有效真实数据，发布各帧和有效范围约束，原始与有效点云分开。
- [x] 台式运行实际非零Twist、方向/转弯/停车/超时和主机Humble收包探针；检查有效样本、范围、帧、时间一致性和自身遮挡。修复必须保存失败证据。
- [x] 仅提交显式文件并写 task-3-report.md，独立任务审查。

### Task 4: 项目控制器验收、展示与收尾

**Files:** `tools/isaac_viper/verify_ros.py`、`tests/isaac_viper/test_verify_ros.py`、`docs/Isaac_VIPER工程适配.md`；更新同任务登记、设计与外部验证记录。

- [x] 复用未改源码2d9a8ce的独立Humble两包构建/211tests，避免重复；本地纯Python与Humble/Kit证据分别记录。
- [x] 实际项目controller→已运行Kit执行器完成两次有独立身份的短路径尝试；两次均FAIL，不能将输入/位移响应写成终端到达。
- [x] 提供可复用独立验证器，记录cmd、真实root/关节、odom/TF/clock、新采样停车尾窗与失败原因；非零命令和真实位移有证据。
- [x] 相关纯Python165passed/8SDKskip；全仓pytest真实收集71errors/5skip，保留环境限制；UTF8回读与diff-check。
- [x] 主线程独立Task4及限定复审已Approved；全分支审查完成，结论With fixes（I1 startup合同门、M1入口/状态）。
- [x] 最终统一fix wave实现493f206：本机172passed/11SDKskip；实际CPU/PXR旧8项+新增3组19场景全过，base validator RED13fail，冻结v5 SHA不变。
- [ ] 最终波限定复审尚待；新runtime Kit/ROS集成NOT_RUN，不预记物理停车/到达PASS。
- [x] 最终EngineeringView hero-window已由主线程下载实际view确认全车/前sensor可见；真实像素ROI全车/近景及回执另存，GUI仅Pause153.47，未Stop或修改actor位姿。
- [x] 任务处置为已提交待合入/功能限制保留；工作树、GUI、模型、构建overlay和全部产物保留待回收，不merge/push/PR/delete。

#### Task4结果与变更裁定（2026-10-29）

run1默认no_progress_timeout_s=15：240.044wall秒，前向+.608843m，终点误差.191417m/.384357rad，FAIL。最后>.01m观测progress到持续零cmd相隔14.992473秒；原PathExecutor离线回放得到TRACKING_STALLED且cmd分量最大差.0001613，仍为离线重建而非内部状态实收。

run2经主线程明确裁定只改bench no_progress_timeout_s=300（原RTF约.05461，15sim约274.7wall），其余安全门/目标/240wall预算不变。新目标来自真实当前pose，不作同起点A/B。240.049wall秒，前向+.659146m，终点误差.141251m/.408577rad，FAIL；最小位置差.108742m，未进内部.1m控制带，无纯转向cmd。离线回放得到先BRAKING后CURVATURE_INFEASIBLE（k=-2.50728>能力1的绝对值，heading .062969≤.08），cmd最大分量差.0025875。无第三次GPU重试，不改算法/goal/curvature凑通过。

两run实际verifier为0b99cb3/c86d35，Path与固定TF曾用wall header，Kit原TF为sim header；direct controller仅按接收age可工作，不声称tf2时基一致。运行后74f7da5最小修复header转写真实latest /clock、停顿保留相同stamp；新增6项RED/GREEN，ROS/Kit新header行为NOT_RUN。Kit运行源码仍750bc3bf/a850，与最终提交身份分别记录。

报告：外部`engineering/coordination/task-4-report.md`；两run原始events/result/diagnosis/offline-replay及launcher/stdout均在台式E/task4-controller-run{1,2}及相应同前缀文件。Task3严格停车矩阵stop4/stop5/timeout未全过原样保留。Jazzy、Orin、实车、真实标定、SLAM、完整导航/探索、PLAN_FOUND仍NOT_RUN。

Task4独审round1：修复I1缓存重发隐藏真实新采样gap（含倒退与首末边界）和I2非有限TF误刷新/缺raw，聚焦及全相关纯套件165passed/8skip；Task4限定复审已Approved，r1文件已同步并完成cp310编译，ROS集成NOT_RUN。未新增外部运行，不改原两run身份/FAIL/成本。

Task1–3勾选表示对应实现、实际检查、留证与任务审查已经执行；不表示Task3严格停车全部通过。最终startup修复后本机172passed/11SDKskip，实际PXR源码493f206共11组全过，见E/final-contract-cpu-receipt.json；最终限定复审尚待。
