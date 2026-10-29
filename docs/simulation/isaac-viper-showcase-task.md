# ISAAC-VIPER-MIDTERM-20261028 / showcase-20261029

- 续接原工程任务；用户明确允许合入并放入e572ed3/1a60611三区展示，允许场景简化，传感分辨率/频率与旧平台一致。
- owner integration/pure-planner-orin；专题hk/isaac-viper-showcase-20261029；工作树D:/Codexworktrees/isaac-viper-showcase/Lunar-Navigation；初始基线1a60611ad62b8c53c9a589f50e1bde2bf45b7e14。
- 原工程accf75b由e15c667按19文件差量晋级，未合入34个继承TCP提交；原工程登记与完整历史仍在原专题及D盘coordination/viper-original-task-registration.md，台式共享入口/mnt/data/WS/lunar-navigation/docs/任务登记.md。
- 主线中央登记有其他任务未提交内容，本阶段用本文件记录本子阶段并更新既有台式同任务入口，不覆盖/暂存它。
- 执行者Codex/root；调查只读并行，源码subagent顺序单写入；本地Windows，台式SDK/Humble独立验证。
- 状态：进行中；原2次闭环与部分严格停车FAIL保留；新的三区算法效果/Actor仍NOT_RUN。
- 外部产物D:/CodexDownloads/isaac-viper-showcase-20261029；台式对应analysis-artifacts/isaac-viper-showcase-20261029。
- 最终处置待实际验证和授权合入；不推送/PR/删除，分支/worktree/所有源和新旧产物保留。

## Task 2 本地实现交接

- 执行者 Codex/implement_light_showcase，Windows 单一源码写入者；基线 `9db7ed332986d8f5a8233e341f15e5c237d13ba5`，沿用本任务专题/工作树与 owner。
- builder、统一 1 m 几何、80 tri 岩石代理、实际三角落位、只含相机的三区层、UI/文件视角通道已实现；原 profile 与 Task 1 分段计时保留。
- 本地局部测试和语法验证已执行，实际 PXR/GPU/ROS、误差数值、RTF、控制停车、GUI 由 root 接续；旧失败/旧源保留。
- 处置：代码 `8ddaffe` 已提交待合入，交 root 实际验证；不推送、PR、删除或回收工作树。准确提交及命令见 D 盘 coordination/task-2-report.md。

## Task 3 恢复展示细节（本地实现交接）

- 执行者 Codex/refine_showcase_detail，Windows 唯一源码写入者；基线 `67055c3ae3292b9043b645c293fe3f5c6e7451ce`，继续使用同一专题、工作树与 owner。依据用户实际视觉反馈和设计附录实施，不改中央 dirty 登记。
- 默认 `light` 质量身份改为 `uniform0p5m-rock320-v2`：统一 0.5 m、2001×2001 顶点 / 8,000,000 三角，每岩石 162 顶点 / 320 三角（上限 512）。代理仍保留世界 AABB，同一实际 Mesh 用于显示与碰撞；旧 `rock_proxy(low, high)` 仍默认 80 三角。
- 新地形由实际高程 XY 梯度生成单位 `vertex` 平滑法线；只给地形增加灰色 `UsdPreviewSurface`，roughness=.95、metallic=0，无下载纹理。ENV 继承的 GroundContact physics purpose 绑定、车辆 Tire 材质与 VIPER 31 个精细外观 Mesh 不改。
- root 实际查看旧 960×540 展示纹理在 4K 最大化窗口被放大，因此新默认 `viewport_size=[1280,720]`；runner 已直接读取回执中的此设置，无需修改运行逻辑。双目仍为 640×480@15Hz，LiDAR 10Hz、IMU 100Hz；14cm、有效 .05..10m/90deg 与原扫描、50kg、.01s 步长、控制/安全门、真实 clock、Task 1 typed array / phase_ms 均保持。
- 质量门按受支持 version/spacing/rock 拓扑推导实际数量，检查 metadata 一致性、实际地形点位/三角索引、岩石 ico 拓扑与 enabled collision。继续接受冻结的 `uniform1m-rock80-v1` 与 `original-source-v1`；旧版无需新法线，新版必须有 finite/unit/正确数量的梯度法线、vertex 插值、哑光视觉材质与 GroundContact physics 继承。
- RED：默认 4 m 平面为 25 点/32 面，未达新版 81 点/128 面；320tri API、梯度法线与版本映射尚缺。GREEN：授权相关 suite `191 passed / 17 skipped`；真实 PXR 质量/拓扑/法线/材质/碰撞故障变异已编写，本地跳过，不算 SDK 通过。
- 处置：本地实现已提交待 root 实际验证/合入；准确提交与命令见 D 盘 `coordination/task-3-report.md`。旧 source/asset/receipt 与旧运行证据保留，未同步或操作台式，不推送/PR/清理。
- 本版实际 SDK/PXR、新版 RTF、GPU/ROS 动态/前进制动、GUI 视觉均 `NOT_RUN`；0.5m/320tri + 平滑哑光 + 1280×720 必须作为整体重新实测。旧 1m 动态通过不能复用为本版证据；A/B known-map、起终点、Actor、建图/新规划器、完整导航、Orin/实车仍 `NOT_RUN`。


## 用户最终场景选择

用户要求用回原先准备的A/B/C三区场景。最终运行选择original-source-v1：直接引用e572ed3对应d24b639e原场景的地形、岩石、材质和碰撞，不采用1m或0.5m替代网格。保留规范VIPER、前置传感器与已验证消息快速路径，传感规格640x48015Hz/10Hz/100Hz不变。

台式入口 /mnt/data/WS/lunar-navigation/analysis-artifacts/isaac-viper-showcase-20261029/original-v1/viper-showcase.usda，SHA256 e0573e650232211b6ef0c0281fe644028855a95f823eda7f9db9d51a9e5eb70e；运行源balanced-source/6370c29。启动回执original-selected-gui-launch.json。替代档及其测试仅作历史对照，不能将其RTF/动态结果转记原档。原档已通过当前PXR组合正例，实际GUI需最后核验；原两次导航FAIL与A/B地图、Actor、完整规划闭环NOT_RUN边界保留。

当前处置：已提交待最终审查合入；中央登记的其他会话dirty内容不覆盖，不推送、不删除工作树或历史资产。

## Task 4 原三区真实新导航展示（本地实现交接）

- 续接同一任务、owner、专题与工作树；执行者 Codex/implement_navigation_showcase，基线 `cddf95e8fd576863cb34082985320e668f92dfc8`；Windows 唯一源码写入者，root 统一 Humble 构建、台式部署和有界运行。
- 按用户最终原场景选择，新增 CPU `navigation_scene`：严格冻结原 `original-v1` 根层 SHA，从原 A 地形和实际启用碰撞 Mesh/实例世界变换生成 0.2m 绝对高程图，逐三角/格裁剪取最高值、未知 NaN；保留岩石和格内 relief。在 A 内搜索 16×16m 近场和 4m 候选起终点，用 wheel_orin 包络外接圆加 .5m 余量和保守坡度/起伏门；拒绝合成回退。新 USD 强层仅更改 VIPER 新初始位姿和 root metadata，原层不改。
- 新增 host Humble cp310 `navigation_showcase`：RELIABLE/TRANSIENT_LOCAL 规范双轴反序 GridMap；仅真实同 stamp odom/direct map→odom 生成 Path；保存原始 diagnostics/reference/action/odom/TF。显式 `--send-goal` 才发一次真正 NavigateToPose；等待时钟、地图、真实 TF/odom 与起点健康。它不发布 T5 或 planner Path。超时有界取消；PLAN_FOUND + 同 session/revision 的非空 ACTIVE reference 才派生正式规划摘要，GOAL_REACHED 与停车另判。
- `run_scene --navigation-overlay-file` 使用 Kit SDK 实际 SceneView/Curve API、每 app 帧读取严格 bounded JSON，Pause 同样可看；全局蓝/局部黄/真实轨迹青。它不创建 USD 路径 Mesh，RTX 不可见。A 近景相机来自实际起终点/新初始高程；A/B/C 相机和原物理、传感、typed 传输、安全门保持。
- 本机真实 RED 为缺模块的 6 项断言失败，GREEN 相关 suite 197 passed / 17 SDK skipped（另 5 subtests passed）；CLI help 已实际执行。SDK 示例仅只读核对，不算运行通过。准确命令与最终提交见外部 coordination/task-4-report.md。
- 本地代码已提交待 root 同步/验收/审查；PXR 真实地图生成、Kit UI、Humble 真实导航/控制/到达/停车、C Actor、Orin/实车均 `NOT_RUN`。源/新旧产物和失败保留，不推送、PR、删除或清理；中央 dirty 登记不改。

### Task4 review 修复 round1

- P1：规划显示按本次A碰撞地图的绝对高程采样，原z/XY/ROS消息保留；未知/越界断线，无0高程回退。actual仍取真实odom三维高度。JSON记录显示地图路径/哈希/原根层哈希，Kit复验。
- P2：local改为已接受Action UUID、ACTIVE reference/revision与PLAN_FOUND匹配的原reference.path；global只接volatile新样本，以本goal请求后的DDS source_timestamp与紧邻同周期reference关联，原Path header stamp不改、不认领原生UUID。旧session和INVALIDATED不画live。
- 首份验证规划保留为清楚标注HISTORY，失效/终态清live且标状态，最终全局完整对照不消失；新goal清旧plan/history/trace并保持stamp高水位。非法会话JSON清旧曲线后拒绝。
- 真实RED8failed；修复后相关206passed/17SDKskip/5subtests，含实际poll曲线坐标8.12..10.12与真实轨迹11.12、原raw规划z=0不改的定向测试。EOF空行已清，git diff --check cddf95e通过。实际Kit/Humble仍NOT_RUN，提交与命令见外部task4report，交root同步/复审/实跑。

## Task 5 原 A 百米绕障和仿真专用 2m/s 档

- 延续同任务、owner与专题；执行者 Codex/implement_long_navigation_demo，基线 `7fb7f583af9eebde5b52d183e158e9e0f0cc44cb`；唯一源码写入者。root负责台式同步、Kit/Humble导航和控制；本执行者只读SSH CPU/PXR，不启动ROS/GPU/控制，不写台式。
- 新 `isaac_viper_navigation_demo.yaml` 引用 `wheel_isaac_navigation_demo.yaml`，显式 `simulation_only`；只将forward上限改2m/s、自转上限改2rad/s。reverse .2、加速度/制动/横向加速度/曲率/安全watchdog、原车几何质量和传感规格均保持；生产 `wheel_orin.yaml` 未改。真实导航器/控制器/Kit必须用同一demo能力档。
- 原A100×100m、.2m栅格由真实collision批量精确裁剪生成，未知NaN、最高高程/格内relief不平滑；地图/层/能力配置hash入回执。先按原Task4额外保守12deg/.08m搜索，真实拒绝；root授权从原cap权威20deg/.2m读取后，保持车包络外接圆+.5m，真实找到静态连通绕障fixture。无扩界，无C/Actor真值。
- 只读真实CPU结果：1344437三角/55 Mesh，提取.85s、整区地图累计4.64s、fixture累计5.20s。起点 `[282.3,-307.1]`、终点 `[333.5,-216.5]`，净距104.0663m，离线无切角A*可行长度111.8077m，直线阻挡格 `[308.3,-261.1]`。这是选点预检；该路线从不发布或绘制，新navigator规划长度仍 `NOT_RUN`。
- 总览相机按XY/真实高程八角点和显式24mm/36×20.25mm、16:9视锥拟合；`navigation_follow` 按真实物理pose更新独立近景相机。只改相机，不传送车辆；viewport线加宽、会话/地图高度/历史绑定保持。
- host默认150sim/300wall，单次显式 `--send-goal` 和拒覆盖、健康门、超时取消保持。`result.json.long_route_evidence` 单列真实navigator global长度、原session/revision、真实阻挡格到规划折线净距，长度>=100且车包络绕过阻挡才满足；PLAN_FOUND、GOAL_REACHED、严格停车分别判定。
- 本机真实RED 5failed；证据门RED 1failed/5passed；额外真实global直穿障碍RED 1failed/5passed。最终相关suite 213passed/17SDKskip/5subtests，CLI与diff检查通过。17skip不算SDK通过；真实原A只读PXR地图预检已执行，派生stage/PhysX/实际Kit视图/Humble闭环/严格停车/Orin/实车仍NOT_RUN，交root实测。
- 处置：已提交待root验证、独立审查和合入；准确提交与输出见外部 `coordination/task-5-report.md`。旧失败、原资产和所有输出保留，无push/PR/清理，不碰中央dirty登记。

### Task5 runtime fix1：C++严格能力根schema

- root实际新导航启动FATAL：`PLANNER_ERROR ...wheel_isaac_navigation_demo.yaml: unknown field: simulation_only`，尚无goal/plan；controller零，Kit实际trace线由root观察。此前Python loader宽松通过不能证明C++兼容。
- 仅移除demo capability YAML的非法根字段 `simulation_only`，保留profile marker、文件名、注释与demo capability_version；所有capability/base_frame等语义逐项不变，生产wheel_orin不改。无需派生第二份能力档。
- 新C++ gtest直接调用实际LoadPlatformConfig(demo)，核2/2和原reverse/accel/brake/yawaccel/lateral/坡度/clearance/足迹/primitive；再注入simulation_only验证仍拒绝未知字段。该C++测试需root在已有Humble包重编执行，本机Windows NOT_RUN。
- 本地schema回归真实RED 1failed/6passed，明确根多出simulation_only；删除该字段后相关完整Python suite与原严格ROS double setter回归一起复测。准确结果、提交和root实际运行边界见外部task-5-report.md附录。未修改C++生产parser、地图/源USD、速度值/安全值或发送行为。

### Task5 显示fix2：保留全程实走痕迹

- 基线8e2c0e9；用户反馈轨迹仅短段、全局不显，root已明确此前nav FATAL无goal/plan不能假画全局。本修复只改善真实数据留存和覆盖可读性，原PLAN_FOUND/session门不放宽。
- ActualTrace容量满不再pop起点；首点/最新点保护，按最小3D点到邻段偏差渐进抽稀到75%容量，保留跨全时段形状并记录audit。局部删除偏差明确不是全局近似误差界；原actual_odom/odometry事件仍完整保存，不按静止位置去重，不改变stamp高水位/健康门/新goal行为。
- result.actual_trace_audit和overlay实际trace.simplification记录accepted/retained/omitted、compactions、首尾stamp与方法。抽稀后UI明确ACTUAL full-span simplified display；GLOBAL PLAN蓝线8px（历史同宽），ACTUAL traveled青线3px，LOCAL REF黄线4px。真实全局未生成时仍不画规划。
- 真实RED2failed/8passed，暴露旧起点被pop到stamp194与全局宽度不足；核心GREEN17passed。新增limit8/201点跨全程L形回归验证首尾与两个拐角、容量/时间顺序/抽稀审计/重复stamp/新goal高水位。最终完整相关结果与提交见外部task5report；Kit截图实际效果待root部署验证，不SCP/运行。

### Task5 fixture fix3：原生端点第二阶段与默认双线展示

- 基线4649708。root R3实际goal发送/接受，但NO_PATH/ENDPOINT_BLOCKED；旧approx起点282.3,-307.1本格intrinsic/observed FREE，硬膨胀state BLOCKED，邻格原生3×3坡度20.2226deg>20deg。旧静态近似不能声明native端点合格；导航阈值/算法不改。
- generator新增显式native二阶段CLI：`--native-policy-json .../native-aligned-manifest.json --native-policy-profile-hash c10bb8e11bf9e420 --start-xy-m 280.7 -307.1 --goal-xy-m 330.5 -218.5`，四项须同时提供。核对实际manifest/NPZ键、源map和CDR SHA、geometry/axis/state wire、源scene receipt/源层hash/profile SHA/capability深相等、原生与本次PXR高程/relief逐值完全相等（含NaN）；原生hash显式与运行回执比对，不假称重算C++hash。
- native observed/intrinsic/inflated均FREE才可作端点/无切角离线通路；两端仍要求原物理候选包络mask通过；直线须命中三层native BLOCKED的有限真实高程格，并继续做原mesh精确归因。offline路线仍不发布/不绘制，captured native FREE不是新run落稳/PLAN_FOUND证明。非native第一阶段回执现在明确仅approx候选、native NOT_RUN。
- 真正只读SSH CPU核验新起终点三层1/FREE，净距101.6366m、native无切角可行110.3994m；阻挡324.5,-229.1三层2/BLOCKED；原起点真实被新函数拒绝。约5.58s含原PXR重算；新Pose/USD/真实新导航仍由root生成/运行，不SCP不控制。
- 按用户显示口径，overlay默认只GLOBAL PLAN+ACTUAL traveled，local live/history仅在host显式`--show-local-path`时显示。过滤只作用snapshot副本；local/PathReference原订阅、raw、证据、controller接线和会话门不变；图例动态对应实际显示种类。
- native功能真实RED7failed，默认隐藏local RED1failed/10passed。身份/硬膨胀/高程篡改/断联拒绝及显示副本不变回归已补；最终suite、提交和真实边界见外部task5report。处置已提交待root同步、实际SDK/Humble新run和独立复审。

### Task5 runtime fix4：旧Humble双参MessageInfo订阅兼容

- 基线ec5bf6c。root新native run r1 host在global/reference双参回调报TypeError；只读实机Humble executors.py398..407确认take_message原tuple被裁成msg_info[0]、execute只callback(msg)，node.py1322的订阅callback签名同为单参。
- navigation_showcase新增限定旧Humble API的MessageInfoExecutor(SingleThreadedExecutor)工厂，仅显式登记global_route与PathReference两个订阅保留原take_message `(msg,info)`，await真实双参callback；普通单参回调仍delegate原executor，不修改rclpy安装或全局猴子补丁。
- dds_source_timestamp支持实际dict/object，保留原正int source_timestamp；缺失/非法为0，原DDS新鲜度/session门继续拒绝，绝不取墙钟或接收时间补值。主循环及pending/cancel/result四处spin均显式同一个executor，shutdown正常关闭它。
- 真实RED3failed；单测覆盖原msg/info identity、未登记单参、无消息、async双参、dict/object原timestamp与非法0拒绝。适配器单测替代的是本机缺失rclpy依赖，不冒称真实DDS通过；最终suite/提交见外部task5report。未启动/控制/修改远端进程或输出，root将在domain218独立DDS smoke再做r2。
