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


## Task3：显式前沿与新导航真实短闭环

BASE451aced7b60267a4011a6d6aea4e816a6c747f9a；产品03f5ff6347660dcb7ef7a71388dd6440dec565c3。
新增nav1cm_exploration.launch.py默认none/controllerfalse，保持旧入口/训练默认；frontier只选目标，共用真实NavigateToPose、标准Path、已安装MPPI与独立Twist植物执行。DRL仅可选hook，未实现/运行推理或训练。
WAITING_FOR_PATH复用原map wait取消/重选策略；BUFFER_UNAVAILABLE（可携带制动candidate）反馈连续path wait，NOT_DUE保留上一真实周期语义，缺world/state/route仍优先等待；实际到达/取消停稳与generation规则保留。新组合map/task/controller共享可选reliable/volatile地图QoS，普通planner/controller旧best_effort/volatile默认及odom/TF保持。

冻结合成短窗最终结果：

| 场景 | 固定窗 / 实测odomHz | 正式global0/PLAN_FOUND/has_reference | GOAL_REACHED / 不同(x,y,yaw) / CANCELED_STOPPED | 新观测源格 | 独立净空下界m |
|---|---|---|---|---|---|
| 原前沿progress | 75s / 49.468 | 45 | 6 / 7 / 1 | 316 | 1.491926 |
| ROOT支撑修正v2未知等待重选 | 60s / 49.524 | 5 | 2 / 5 / 3 | 0 | 3.852103 |
| 原外部任务取消 | 30s / 49.528 | 7 | 0 / 1 / 1 | 0 | 4.044363 |
| 大帧无目标接收 | 15s / 49.596 | 无目标 | 0 / 0 / 0 | 0 | 仅接收资格 |

物理运行harness7b33d9d9；终态自审收紧后以grader15eba300严格离线重算全部仍通过。保持只使用<=探针收到ACTION_RESULT的实际样本，8次到达均xy<=.1m、yaw<=.05rad、v/w<=.01，保持约.321s；不声称获得server内部终态时间。外部CANCEL在实测运动中发起且后续无新goal。progress7个不同pose含3个xy位置，v2五个候选为同xy不同yaw，不能等同五次位移到达。

三运动轮源10m/.1/10000格，真实local计算5m×5m/.01/500²；大帧源5m/.01/250000格实际进入三消费者，0目标/0运动，不能称运动成功。车体1.182×.818、.2净空、.15响应和原速度/加速度/曲率不变。私有domain92/localhost1/cmd，所有最终图无T5 publisher、一个Action/Path源，过程正常退出、无升级信号/残留worker；不是Orin/实车验收。

v2仅按ROOT裁定将未知缝x[.9,1.3]移为[1.6,2.0]，其它几何/60s/物理不变；原v1因初始硬包络支撑无效而失败保留。v2无运动真实native和observed-only policy中心均FREE/eligible、assumed0；源SHA372a222cb0eafa41821dddfeb758068d03229cf1314d146ef7159431a2e85fc0与最终raw一致。该轮动态障碍条件未触发，global线段与未知缝交集0；没有验证动态障碍响应、未知捷径冲突或已知绕行优先，保留待验，不扩场景。

验证：Windows Task3最终focused45passed；Humble纯Task13、Runtime12通过，planner CTest6/6（20用例）、controller14/14（442用例）。前沿首轮11/12 target，其中旧PlanMotion合成OutsideToInside用例5s超时；单例及完整16例target定向复验通过，当前XML247用例0fail。原超时未稳定复现、根因未确定，不能称首次整包全绿；legacy代码/test/库归属对BASE无diff，未做干净BASE重建对照。Linux最终静态/launch/grade41passed；Windows两项旧launch测试rsplit('/')路径假设失败分列，不倒写Windows全绿。Python编译、bash语法、UTF8读回、diffcheck通过；没有执行hardware wrapper。M1/M2环境噪声deferred。

失败账全部保留：缺依赖/缺bundle prerequisite/安装RPATH身份比较；UUID旁观wire类型；高频反馈驱动odom；单spin节流积压；原v1初始UNKNOWN支撑；Windows计时纯测试量化；旧合成超时。首次QoS RED还遗漏私有cmd：两个controller仅构造不spin/无goal/plant，但destroy_node发起默认T5零Twist发布尝试；当时domain92/localhost1，未记录订阅者/送达，不能称无消息或无订阅者。该轮不作合格图，后续私有RED/GREEN和最终图独立验证。

安装：60项source/install Python/launch SHA或ELF BuildID匹配，8项native实际/proc maps与ldd归档；shared wheel.yaml SHA23b93ebaf83466eb51991881cab045f14a80e871811b08f6dd39a5a768d91fe6。qualified core仍b63079f3、map ROS617d4d71，Task3独立install，旧frozen runtime未覆盖。原始/失败约528.8MB、压缩39.7MB回收D盘，174项SHA核对；固定窗405s及提前失败/构建/清理成本如实保留，无训练账本变化。

本地ART D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029/task3；远端同任务根exploration-integration-20261029/task3。完整命令、UUID/实测xyyawstop、各轮身份、声明/失败及ROOT rulings在ignored SDD task-3-report.md。公开操作与可选DRL参数见操作指令新增段。

NOT_RUN：Jazzy、Orin/实车目标环境（Ubuntu22.04/Humble）、真实P3/生产/跨机DDS、rosbag/RViz、100m行驶/100m全源1cm、DRL推理/权重/训练。当前处置已提交待ROOT独立审查/owner合入，资源待授权回收；无merge/push/PR/upstream/delete/reset/stash。自审由作者执行，ROOT独立审查未被替代。


### Task3最后补证：一次真实障碍地图更新

ROOT在实跑前依据既有raw将新增case初始测量选为已合格progress源（无额外未知缝），不用无法达到>=1.6m触发条件的近目标v2；原四窗不动。冻结harness dee06601，前沿仍自然选点，sourceSHA启动前匹配70d26980e9c32fd604f8578f5bac3895f79860c49a08be2c9625e8b1f707889f；只运行一次固定60s，没有改产品/安全距离/0.2净空/0.15响应/运动约束。

obstacle-update-once通过：当前UUID10eafaff2e8851ff410061999435b73c逻辑接受后14.25ms，t3.046878s、body-goal1.883933395m时加入同0.6m/2m实际障碍；下一源测量高度数据实际变化，随后探索导航occupied0→193，匹配UUID WAITING_FOR_PATH→CANCELED_STOPPED→新逻辑接受UUID67ea0bdcb2228d551164a97b3d49e3a9，XY从(-.1,-1.8813)到(-.1,+1.8813)，相距3.762555m。source/map/UUID顺序均有不可变raw；平顶内部intrinsicFREE不等于整车通行。

该窗0GOAL_REACHED、2不同XY、2CANCELED_STOPPED；不将新goal接受写为到达。实际odom49.278849Hz，最大v.199809/w.871312、a/alpha<=.5，独立净空下界.859790m，全部正式规划/实际运动/私有graph/安装/无异常/cleanup及严格事件门禁通过，owned remaining=[]。未知捷径/绕行优先仍未覆盖。

补证grader10项RED→GREEN，最后Windows focused55passed、Linux合同51passed；未重建/重复未变产品包或原四窗。追加固定60s使总固定窗465s，补充原始12.18MB/压缩2.07MB已回收D盘并核对14文件SHA；之前所有失败和174项原始回执保留。完整次序/UUID/源SHA/成本见task-3-report.md与obstacle-update-once。


### Task3 scoped审查修复 round1：静态输入与终态身份

本节取代前文旧运动窗的可复用资格。旧summary/raw原样保留，离线最新suffix重算到达/停稳计数未变，但所有旧运动raw缺权威接受通知，不能由current_goal跨topic顺序证明UUID目标归属；旧1cm无目标预检仅接收资格不受影响。本轮仅修开放I1-I3，独立复审仍待ROOT安排。

产品68f52d2f9daade228e4b815ef23ba1f53df0a3ce：frontend接可配置reliable/transient_local tf_static_topic并复用PoseResolver；原动态订阅保留，launch给策略传相同静态topic。Runtime唯一owner逻辑接受后通过既有锁外notice捕获TASK_ACCEPTED(task_id,generation,adapted map x/y/yaw)，不增topic/Action/Path/schema。runner只凭exactID通知绑定，current_goal只旁观，缺失/冲突failclosed且不触发障碍；终态改为最新实测端点的连续有效suffix与.15s新鲜度，取消只查最新样本，不留历史best/any。

先行Python7red/42pass；C++早发布零stamp静态TF且frontend晚启动反例红4.018s→绿21ms。Windows最终focused71passed；Humble planner6/6（22例）、frontend12/12（248例）串行通过，Linuxlaunch/grade48passed，最后默认notice/pendingyaw追加后Runtime定向10passed。只构建planner/frontend独立task3/fix-round1 prefix（30.3s），core/MPPI/controller数值未改不重复全套。

冻结harness5b179e0，原固定75/60/30/60各一次全部PASS：GOAL_REACHED6/2/0/1，不同pose7/5/1/7，不同XY3/1/1/3，CSTOP1/3/1/6；正式global0/PLAN_FOUND/has_reference45/5/7/33。实测odom49.335/49.444/49.425/49.349Hz，独立扫掠净空下界1.495108/3.850028/4.074838/.859851m>=.2。75s全链仅启动前一次latched零stamp静态TF，无dynamic publisher；20个接受/终态UUID全有权威通知，无冲突。225s运动窗全部私有domain92/localhost1/cmd，cleanup exit0、无升级/owned残留。

新障碍60s精确通知UUID53bb80d7f254a6445791f0ee6e1284d4、距离1.883933m后加障碍，source/map occupied0→193、匹配WAIT/CSTOP、新UUID889af5ca0d0cbb3f30509c532087ce1d不同XY（3.762555m）链通过；该轮1到达/3XY，不能用旧轮0到达/2XY替代。v2仍无实际障碍更新、global未知缝交集0，未知捷径/绕行优先未触发。

新安装60项source/install与8native核对；新incremental.so SHAa80a185e38825a26f7992f44cc2310b4ab6eabd19d94c8caac3745af4d2b39e0，runtime.py SHA5fd57cd774fd63209149d4e3f288bdbd38caeced34f99416638440e4ede57f33。原Task3/install60文件+8native逐项SHA未变；新build/install/log/cache与raw保存在fix-round1，67文件归档SHA已核对。详细命令、全部UUID/最新xyyawstop、旧raw差异/限制、固定窗原始数据、安装身份与收尾见同SDD task-3-report.md追加段。

原frontend首次11/12/未定legacy5s超时、旧Windows2路径失败、M1/M2、默认T5零Twist发布尝试（无送达/订阅者记录）及全部失败保留，不能倒写首次全绿。NOT_RUN仍含设备/生产DDS/真实P3/DRL与未知捷径；本轮只完成实现者验证，待同reviewer scoped复审/owner合入/授权回收，无merge/push/PR/upstream/delete/reset/stash。


### Task4公共依赖C1：连续观测FREE浮点接缝修复（待独立审查）

本轮BASE `0ebc43ea5a0f7f2849039a146bc3ffc49fd4c80a`，先行测试 `01f02bb9`，产品 `734838aa9e9f4955ee3c42bfdf9e058a6d25bb2e`。Task4原固定权重window02保留：45.346s无运动、44次正式 `0/PLAN_FOUND/has_reference=true`、重复QP3，不能算实际导航通过。ROOT对原709源格初帧独立重建后证明公共P3MapStore误造厘米UNKNOWN行，Actor yaw转换正确；C1据此只修公共依赖，不改变Actor/目标/场景/物理限制。

精确反例 `TerrainUpdate(.2,(-3.,-3.),0.,[[2,11,29,20,FREE]])` 应覆盖x[-260,280)、y[-80,100) cm。旧版179行且缺-80，因为minY=-79.99999999999999进入内部事件，数值薄片中点落边界导致整行拒绝。修复只在选择条带内部事件时采用已有 `1e-9 cm` 边界容差；原顶点/真实观测支持和连续分量判定均保留。新增90/180度旋转、非整数原点、1e-9m与.1mm真实缺口；原旋转断开分量/部分厘米/来源历史和BLOCKED优先回归继续通过。

| 本轮检查 | 真实结果 |
|---|---|
| Windows最小RED | 3 failed / 3 passed，.55s；精确反例及两旋转失败 |
| Windows受影响P3/map/geometry/cache/UNKNOWN/reference组 | 96 passed / 9 skipped / 1 error，4.50s；error为--basetemp父目录不存在的WinError3，明确建D盘父目录后仅该单例1 passed/.49s。9 native skip不算Windows原生验证 |
| Linux709真实native RED | 新增独立core oracle逐格展开与旧P3投影相差2660/250000；1 failed，7.29s。先前scripts命名遮蔽与默认user-site NumPy2/SciPy1失败另存，不充作几何RED |
| Humble新安装构建 | 仅lunar_nav1cm_planner Release，1包2.66s；无core/controller/训练数值重建 |
| Linux受影响地图/native/cache/reference | 168 passed，19.15s；含709 disk原生流水线、真实缺口/旋转/来源历史。系统pytest保留1条旧pythonpath配置不识别warning |
| planner包CTest | `ctest -j1 --output-on-failure`，6/6，6.53s；domain92/localhost1，Action mockworker、无controller/真实运动窗 |
| 安装级原SensorModel709重放 | elevation31x31、origin(-3,-3)、.2m，与固定disk掩码逐值/字节相同，SHA63a944371264ebbc4ea934f6b465292f45196e25af622643ec5d00e984a232d0。FREE220140→222800，UNKNOWN29860→27200，恢复恰2660格；y=-80内侧连续FREE |
| 固定初始硬圆 | native/Python距离场最大差0；中心净空.8858578643762691→2.487928932188135 m，要求.9217224777339302 m不变，可行性false→true；只是初帧几何支持证明 |

远端ROOT=`/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027`，本轮所有source/build/install/log位于 `ROOT/exploration-integration-20261029/task4-common-map-fix`，原Task4和Task3安装不覆盖。按操作指令末节source顺序，最后新common install；38个Python安装文件SHA及Gitblob与734838aa源码一致，实际导入全部来自新安装；`/proc/self/maps`实际core SHA b63079f301e2f1b1d6a0fc126d86eda152d98b6d30baa0b530b7da28260a9ce4。mapROS解析到task2/fix-round1的617d4d71（本诊断未运行mapROS节点）；3个planner native与旧安装SHA完全相同，1574个旧安装文件前后SHA不变。系统 `/usr/bin/python3`、NumPy1.21.5与SciPy1.8.0，关闭user-site，不注入整个用户site。

本地证据 `D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029/task4-common-map-fix`，逐字命令/原始输出、Windows路径错误、三次远端环境诊断、初次五层payload维度诊断错误、shell CRLF错误/遗留路径、身份及受保护安装清单见同SDD `task-4-c1-report.md`。所有原失败保留；诊断脚本只读原SensorModel，不运行Actor、ROS节点、solver求解或训练。Task4原45s真实motion仍待独立审查后复验，不能用C1替代；Orin/实车/真实P3流/生产DDS/RViz/rosbag NOT_RUN。专题与资源已提交待审查/owner合入/授权回收，无合并/推送/PR/upstream/删除。


## 任务级审查与DRL交接更新

Task3原独立审查I1–I3已由68f52d2/5b179e0修复并复审关闭；C1由734838aa修复并独立审查关闭。DRL专题以a094b902为基线，产品9149f3c9、legacy验证帧744ee2bb、接口修复4ebcbf24/任务审查HEAD874c1527；Task4 map_frame和infer配置白名单两项阻挡均已关闭，定向安装12PASS。新后端45s有真实运动/观测推进/取消停稳，未到达；旧后端8s只作兼容smoke。原“待Task4复验”条目保留为当时状态，不改变旧原始资格。当前统一状态和逐项边界见[交付汇总](2026-10-29-exploration-nav1cm-closeout.md)，两owner专题仍待合入，最后跨分支审查进行中。
