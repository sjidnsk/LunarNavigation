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
