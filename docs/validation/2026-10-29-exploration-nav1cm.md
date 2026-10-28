# NAV-1CM-AUTO-20261029 验证记录

## Task1 共用任务层与可选 Action

公共 owner 为 integration/pure-planner-orin，专题 hk/nav-1cm-feasibility-20261026；本阶段基线211c7c41cdb14ad4eae0bb86650ee137cc40c42e。实现源码提交9a752d55，验收补强7a9e278f，SIGINT回归0f87c86b，修复后的已安装源码663c39bb9f9d2d0f3186ae802370835c4175306e。已提交待主线程独立审查、待合入，未移动owner。

- 纯任务状态机按UUID/代际处理替换、无效目标、取消与实测到达；Runtime为唯一session/coordinator写入线程。人工PoseStamped与Action经过同一序列队列，无效新请求保留旧目标。
- 到达要求当前目标xy/yaw及配置停稳容差、0.3秒保持和后续里程计测量。取消退休旧发布/在途身份，使用现有能力约束及当前地图认证制动，实测停稳后结束。有效替换不要求先停稳。
- 默认planner_node保持原接口；可选navigation_task_node复用NavigateToPose。有效替换结果为业务CANCELED/GOAL_REPLACED、ROS ABORTED；显式取消完成才是ROS CANCELED。
- 真实SIGINT验收发现pending任务列表清空早于rclpy GetResult回调发送，已新增RED回归并增加1秒有界排空窗口；不再丢失SHUTDOWN结果。

### 当前验证

Windows已有D:/conda_envs/lunar-explorer/python.exe：初始任务8 failed、Runtime3 failed、缺Action入口1 failed；实现后扩展纯任务/Runtime13 passed。默认Anaconda缺scipy的首次收集错误保留。

获准Ubuntu22.04/Humble x86_64、ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1：

- 独立colcon构建lunar_nav1cm_planner成功。
- 完整one_cm套件649 passed、1 skipped，真实acados及三个native库启用。唯一skip是依赖缺失测试发现acados已经安装；运行输出中的两条QP求解失败由原套件覆盖，不是pytest失败。
- 修复前完整ROS包套件14 passed；修复后包CTest串行5/5通过，内含新增SIGINT回归，总计15个pytest用例。核心源码从7a9e278f到663c39bb未变，不重复运行已通过的核心套件。
- 实际Action客户端覆盖有效接受、反馈、无效目标保留旧任务、同位姿替换、迟到旧UUID取消、人工替换、无yaw、朝向等待、运动不成功、实测停稳后取消、到达保持及关闭。
- 默认真实native/MPCC节点smoke生成局部Path，正式全局日志同时满足planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true。该smoke的里程计为脚本发布，只证明任务传输/状态机及真实planner接入，不能作为MPPI/独立plant运动闭环证据。
- 真实入口在活动Action期间SIGINT：客户端收到ROS ABORTED/outcome INTERNAL_ERROR/reason SHUTDOWN，exit_code=0。正常smoke所有worker自然退出；冷启动SIGINT按既有超时机制终止一个local worker，surviving_pids均为空。
- 38个安装Python文件与源码逐文件SHA256一致；实际导入均来自本任务install。native三个SHA及core动态链接路径见安装回执。

首次Humble合集因ROS安装的scripts实体包遮蔽仓库namespace而收集失败；关闭自动插件仍未解决。最终纯one_cm仅从PYTHONPATH剔除/opt/ros/humble/local/lib/python3.10/dist-packages，保留native/acados依赖；ROS包套件恢复原路径。所有测试从仓库根目录以python3 -m pytest执行。旧pytest不识别仓库pythonpath配置的warning被保留，未改测试/源码掩盖环境问题。

本地证据根：D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029/task1。远端证据根：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027/exploration-integration-20261029/task1。完整命令见final-r3-run.sh、sigint-fix-run.sh；回执见remote-receipts/full-pure-r3.txt、ctest-final.txt、native-smoke.json、sigint-result.json、installed-identity.json，失败日志保留。

NOT_RUN：Jazzy、Orin、实车、生产DDS/真实P3、真实MPPI+独立plant闭环、前沿/DRL接线及任何正式训练。任务范围内没有启动控制器、连接真实T5、改变训练或合并/推送/PR/upstream/删除。专题与本任务构建资源保留待审查/待授权处置。


### Task1 审查修复 round1：I1 取消/到达终态原子顺序

审查指出旧Runtime在输入锁内检查序列、解锁后修改任务终态；取消可以插入两者之间，使旧观测仍成功并永久忽略取消。0a78e337基线确定性RED为2 failed/4 passed，分别复现observe入口重入与真实输入线程在观测交接点入队；原始输出保存在task1/fix-round1/red-runtime.txt。

修复提交9f8da10e0825ef647b2cb63900e41cee4dd4c6cd：先生成不改变active/保持计时/轮次的不可变TaskObservation，再在同一个Condition临界区验输入序列、提交任务状态并写内部事件队列。临界区不执行外部回调；event_sink锁外交付，因此默认RLock重入不能绕过终态提交，sink也能等待另一输入线程。早于提交的取消使旧提案作废并在下tick进入真实制动/停稳取消；晚于终态提交的取消按旧取消处理，顺序明确。Runtime仍为session/task唯一写入者。

本轮当前结果：Windows纯任务+新旧Runtime 22 passed；Humble纯任务10 passed、Runtime12 passed，独立包构建通过，CTest 5/5（18用例）通过；默认真实native Action smoke及活动Action SIGINT通过。正常smoke自然退出无残留；冷启动SIGINT受控terminate一个local worker（PID757097），两类surviving_pids均为空，进程exit0。38个安装Python文件SHA匹配本轮源码，3个native SHA与前版相同。原649项数值/地图suite未重复，之前结果保留为历史，不声称本轮重新执行。pytest配置警告M1保持deferred，本轮未修改全仓配置或依赖。

本轮产物均位于原task1证据根的fix-round1子目录（独立build/install/log/acados-cache）。完整命令run.sh，日志remote-receipts/pure-task.txt、runtime.txt、ctest.txt、native-smoke.json、sigint-result.json、installed-identity.json。当前仅完成I1修复及实现者验证，仍待原审查者对I1和新增breakage进行scoped复审；Task2尚未解锁。NOT_RUN和脚本Odometry测量边界不变。


## Task2 独立探索地图：实现者验收，待独立审查

基线 ea39645b535e0ca058f5d906afa515cd34326198；产品提交 01b895b276d87b659bfc02ee7391c671998567d1。沿用公共专题，不移动 owner。独立 `lunar_exploration_map_node` 复用原生源格物理分类、持久高程、派生、投影与导出；原导航节点不启动。参数及最终 C++ 接口见 [地图包说明](../../ros2_ws/src/lunar_incremental_navigation_ros/README.md)。默认策略 0.2 m、探索 1 m，与导航 5 m/0.01 m 窗口独立；轮式能力与 Task1 相同，硬包络实际覆盖车体外接圆加 0.2 m。

经 root 明确批准增加 `DeriveProjected`、`ExportObserved` 包装输入与 `ProjectConservative`，原公开结构布局、`Derive`、`Export` 和旧 any-FREE `Project` 不变。固定策略格按真实源足迹保留历史，任意源 BLOCKED 不被平均抹平；不完全覆盖/有限 UNKNOWN 不误报 FREE，NaN 保留历史；源统计不冒充粗格精确 3×3 统计。非水平变换不能证明的区域保留 UNKNOWN。新探索投影采用导航中心可行性 `fine.State`，并非纯 observed mask。

### RED、诊断与 GREEN

- 7fd170bb 的原生编译 RED：缺新组件头与 `ExportObserved`。首轮 ROS 14/15 的旋转测试错误地将世界策略格索引用于局部 raw view；修正测试坐标后通过，原失败保留。
- 默认 8 MB core 19/20：既有 `PlanningSnapshotContractTest.PartialBoundaryTilesKeepCellsOutsideOldBoundsUnobserved` 在未修改的 `MakeFineTile` 栈帧崩溃。同一测试强制旧库仍 exit139；32 MB 仅作诊断通过。root 审阅 gdb/旧库/32 MB 三份证据后批准仅把既有 fixture 数组移到堆，全部数值及断言不变。默认栈单例 GREEN，随后完整 core 20/20（323 用例）通过；未通过放宽环境替代修复。
- 最终 ROS 包串行 CTest 15/15（128 用例）通过，其中新探索地图 8 例，包含源 .01/.05/.2/.07、热换、薄障碍、局部更新、持久观测、任意 yaw、可选统计、退化/倾斜、成本和重复片段数量。core 源码/二进制在后续节点缓存修复中未变，保留已执行 core 全套证据，不冒称重复执行。
- 首次 1 MB/.01 源图在 best-effort 下未进入 callback；保留失败的旧 stamp 回执和接收日志，8 秒定向传输证据排除 worker 已接收后计算阻塞。gdb attach 被 ptrace 拒绝，未改内核策略。相同程序改为 reliable 后 8 阶段通过；最终默认 reliable/volatile，可配置。真实 P3 QoS 尚未验收。
- 稳定 20 Hz 流 RED：仅 odom 与等价 TF 仍额外 Apply 20 次，1.06058 s 内 CPU 0.59 s。缓存修复后 1.03948 s 内 CPU 0.01 s、额外 Apply 0；当前 anchor 精确更新，等价归一化 q/-q 不重派生。真实 TF 平移 1 m 则 fine revision 5→6，真实 anchor 同步平移、旧源 receipt stamp 7 保持真实；未知起点 ready=false、无 assumed start。

### 实际 Humble 地图、资源与身份

Ubuntu 22.04/Humble x86_64、domain92/localhost1，实际 native 与已安装独立节点，全部默认 8192 KB 栈。源图均 5 m × 5 m：.2 平图→.01 平图→.01 重复→.01 薄障碍→.01 局部 NaN→.05→.2→.05 旋转平移，共 8 阶段通过。起始 stamp0、普通 elevation-only 和静态 TF 被接受；完整/增量、未来版本全量、当前版本空增量、实际 OccupancyGrid、服务图、未知 anchor、无 Action/T5、SIGINT exit0/PID 消失均有原始 CDR/JSON/log。

1 cm/250000 cell 首帧端到端 242.503 ms，worker 适配 0.940 ms、Apply 225.136 ms、缓存导出 0.054 ms，最后服务查询 0.474 ms；重复 215.671 ms、薄障碍 228.417 ms、局部 NaN 22.185 ms、热换 .05/.2 分别 23.475/5.013 ms。未设置 50 ms 硬拒绝。薄障碍保留 4 个 intrinsic BLOCKED；后续有限平图合法清除它。

首个 .01 帧 history 625 policy cells/250000 fragments，RSS 98092 KiB、HWM 99728 KiB；重复 history 不增，RSS 118672/HWM130624；薄障碍与局部帧仍 250000 fragments，最高 HWM140740 KiB。热换 .05/.2 后 fragments 降至10000/625，分配器 RSS 未立即回落；旋转平移后1314 cells/17709 fragments，真实 TF 变化后1475/22507。实测为有限序列，未声称长期非对齐滚动内存上界或大世界 1 cm 能力。

新 core 独立安装 `task2/install`，SHA256 `b63079f301e2f1b1d6a0fc126d86eda152d98b6d30baa0b530b7da28260a9ce4`；ROS 库 `64115eb0165b395e0c54d59313c5c596a18a95a3ad69e171fd112c0da389519e`。旧 core underlay SHA `0092ca1863b642f9bc0cbc7bb443b22446c26d1b29f153a3e217ce376e9bb4ec` 未变。源码/安装头逐项匹配，实际 `/proc/PID/maps`、ldd、导出符号和 CMake 来源均已归档；未执行专用 ABI checker，不把结构布局未变宣称为工具级 ABI 证明。

新 core 下 Task1 在 `task2/task1-requal` 独立重建，CTest5/5、实际 native Action/替换/取消/停稳/yaw保持/关闭通过，正式规划同时满足 outcome0/PLAN_FOUND/has_reference。Task1 terrain bridge 与 task worker 实际加载上述新 core；全部 worker 退出。此轮 Task1 验证绑定节点缓存修改前提交80d5a37，后续仅 Task2 map-node/统计/测试变化，Task1/core 未变；依 root 裁定不重复整轮。里程计仍为脚本发布，不能据此声称独立 plant 闭环。

命令原件 `red-run.sh`、`green5-run.sh`、`task1-requal-run.sh`；受影响包 colcon 独立 build/install/log，`ctest --test-dir <package-build> --output-on-failure -j1`。最终 `build5.txt`、`ros-ctest5.txt`、`core-ctest3.txt`、`final-map/ros-map-smoke.json`、`task1-requal/native-smoke.json`、`installed-identity.json` 见任务外产物。源代码通过已提交 bundle 导入唯一获准远端测试 checkout，远端 detached01b895b，未操作 origin。详细命令/接口/各阶段证据在 ignored SDD `task-2-report.md`。

本地证据根 `D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029/task2`；远端根 `/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027/exploration-integration-20261029/task2`。`receipts-final.tgz` 和 `remote-receipts` 保存日志/JSON/CDR/测试 XML；构建与安装留远端，不提交 Git。

NOT_RUN：Jazzy、Orin、实车、真实 P3/生产 DDS、跨机 DDS、rosbag/RViz、前沿/DRL接线、独立 plant 运动闭环、长期滚动资源上界、正式训练。本 Task 无训练/硬件/真实 T5，无 owner merge/push/PR/upstream/删除。处置为已提交待 root 独立审查/待合入，资源保留待授权回收；源码/operator/ROS 槽交回 root。


### Task2 scoped 修复 round1：I1 共用投影几何遍历

基线20d196a0；先行测试635aa42c，产品修复6280ba206e278665cb28ad81fd7b02e19fe4df6b。仅提取匿名命名空间 `ProjectImpl<Aggregation>`，统一几何校验、输出分配、坐标换算、索引裁剪和两层交叠遍历；`AnyFreeAggregation` 保留旧 any-FREE 内外层提前退出，`ConservativeAggregation` 保留全面积遍历、BLOCKED面积大于容差优先及完整FREE覆盖规则。两公开API、1e-9面积容差、原生物理阈值和全部原断言不变；没有改core/Task1/前沿/DRL。

这是结构重复修复，没有功能RED：先在未重构实现上运行行为对照，projector6例与地图8例（CTest2/2）通过。新增3例覆盖混合FREE/BLOCKED/UNKNOWN及FREE先后次序、非对齐原点/负索引/部分覆盖、两接口不兼容frame拒绝。重构后独立Humble Release构建通过，受影响ROS包串行CTest15/15、131用例全通过（5.99s）。core和Task1未变，本轮不重复其完整suite，旧资格仍绑定旧执行记录。

实际新安装节点再运行一轮原8阶段/stable20Hz/真实TF变化/服务与退出harness，唯一脚本变化为可执行文件指向round1安装，输入和断言未变。全部通过：dense .01首帧238.571ms；20Hz1.029465s/CPU.01s/额外Apply0，当前anchor吻合；真实TF使fine5→6；未知起点无假支撑。PID800790 SIGINT exit0且消失，私有图无Action/真实T5。新ROS库实际加载 `task2/fix-round1/install`，SHA256 `617d4d71d4d21e72c983a05497e17766f5380e949fe729d33a3bd2cc3c846861`；core仍从task2/install加载SHA `b63079f301e2f1b1d6a0fc126d86eda152d98b6d30baa0b530b7da28260a9ce4`，旧ROS安装SHA64115eb0...未覆盖。两处产物均位于原Task2根的fix-round1子目录，含独立build/install/log、focused-run.sh/final-run.sh、focused-before.txt/ctest-final.txt、actual-map/ros-map-smoke.json及maps、installed-identity.json与receipts.tgz。

状态：I1实现者修复及验证完成，待原reviewer scoped复审，不自行宣称独立质量门槛已通过。M1仍deferred；原NOT_RUN/长期资源与P3边界不变。显式提交，无owner合入/推送/删除，资源保留；源码/operator/ROS槽交回root。
