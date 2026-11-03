# 地图统一源码实施与验证记录

日期：2026-11-03；任务 MAP-UNIFICATION-20261103。

## 来源与处置

- 基线：98fb60812bf1a541f92e31844393c3150756efe2，保留最新 TCP 全局路径优化。
- 专题：hk/map-resolution-unification-20261103。
- 工作树：D:/Codexworktrees/map-resolution-unification-20261103/Lunar-Navigation。
- 公共 owner：integration/pure-planner-orin；TCP 应用资格由 app/orin-unreal-simulation 承接。
- 实现提交：3b17fd48（同格网导出）、57d3f743（分辨率与几何版本）、d47a7724（不可变事实及历史复用）、273d7558（探索派生）、bf053a33（生产者及运行接线）。中间格式修正提交保留审计来源。
- 状态：已提交待目标验证、待合入。未 push、合并、切换正式安装或重启远端节点。受保护原工作树未改动。

配置、职责及协议详见[地图统一与分辨率配置](../地图统一与分辨率配置.md)。

## 本地实际证据

产物根目录 D:/CodexDownloads/lunar_navigation/map-unification-20261103/，不进入 Git。

|验证|结果|证据|
|---|---|---|
|受影响 Python 导航、地图、launch、发布测试，实际 native DLL|1551 passed / 29 skipped|final-python-all-native.xml、同名 log|
|C++ 地图、投影、策略导出及发布快照纯算法测试|38/38|resolution/，MSVC + 既有 ROS 类型测试替身|
|C++ 事实协议解析及时间戳范围|5/5|cpp-facts-codec/final-time.xml|
|独立复审复跑|Python 135、C++ 38、协议 5 通过|各子任务回执及上述回归产物|
|额外 DRL 接口集合|21 passed / 10 failed / 2 skipped|drl-interface.xml；生成的 lunar_planning_msgs 与 lunar_drl_terrain_native 不可用，不作通过声明|

Python 测试通过 python -m pytest 从仓库根运行。使用 D:/APP/Anaconda/2025/python.exe；native terrain/reference、geometry、distance-field DLL 来自本专题 D 盘隔离构建。早期未设置 native 环境或 Windows 导出不全的失败日志保留；配置修正后最终全量结果见上表。此处 DLL 和测试替身验证不等于 ROS 包构建。

回归覆盖 20/5 cm 源尺度切换、自定义尺度、滚动原点、旋转与部分覆盖、同格网几何、UNKNOWN 与旧 BLOCKED 分离、障碍清除、空快照撤销、跨几何版本请求、sink 失败隔离与退出顺序。已捕获缺陷的 RED/GREEN 产物包括服务旧版本兼容、格网原点、细碎多边形及 ROS 时间戳溢出。

## 分批复审与性能

独立复审已关闭：几何变更仍返回无变化、同尺度原点重建、远端证据重复拷贝、全对全精确相交、有效细碎观测误拒、时间戳转 ROS 后回绕。最终复审未发现剩余 P1/P2；不是目标环境验收结论。

同一 Windows 纯 ApplyFacts 冻结用例，3600 个不相交碎片的验证中位耗时由 1210.10 ms 降至 4.44 ms；精确相交调用由 6478200 降至 0。普通 1/16 碎片用例约 2.3 ms。证据 overlap-performance/results.json 含源码/驱动/产物哈希。这不是 DDS 或整车端到端耗时；大量 AABB 重叠时最坏复杂度仍可能为二次。

## 尚未完成的资格与限制

1. 当前共享是完整快照传输；变更快照的探索投影仍重建。dirty-tile 全链路增量尚未完成，不宣称所有局部更新都避免全图工作。
2. 现有 TCP 启动入口是轮式配置。5 cm 数据能力已测，轮式/足式实车应用的完整启动链尚未验证；不将纯格网测试冒充足式适配。
3. 当前组合启动保留探索地图视图常驻行为；未做纯导航视图按需启动。
4. 固定 Actor 的尺度应按原模型输入合同显式指定，不能因自动跟随源尺度而宣称模型兼容；未训练、未加载模型、未验收 Actor 效果。
5. 192.168.100.216 SSH 超时/认证前关闭，远程台式连接也超时；本机无可用 Linux/ROS 运行环境。因此实际 ROS 节点构建、Humble 包测试、DDS 跨进程、TCP 导航/探索闭环、实车 Orin、5 cm bag 回放、DRL wire/Actor 均为 NOT_RUN。

## 交接顺序

先由唯一部署会话核对专题提交及目标工作树，再在隔离 Humble 构建受影响 ROS 包；实际启用 shared_map_facts 验证只存在一个 P3 地形生产入口，核对同帧源/派生版本、20 cm 与 5 cm、自定义尺度和完整空快照撤销。测量大图快照字节数、地图线程与探索派生耗时后，决定是否实施进一步增量投影。TCP 导航/常规探索、固定 Actor、两类实车分别验收。通过后先审查公共模块晋级，再适配应用线；本专题不直接移动长期分支或覆盖运行安装。


## 测试机恢复连接后的补充验证

用户通知恢复连接后，已在 yanfa@192.168.100.216（yanfa-dog）执行隔离验证。只写 /home/yanfa/P4/build/map-unification-cd27fa78，上传提交 cd27fa78 的必要源码、配置与测试，未修改 P3/Env_X，未切换或终止正式 P4/P3。

- 实际 ROS_DISTRO=humble，三个受影响包 lunar_incremental_navigation_core、lunar_incremental_navigation_ros、lunar_nav1cm_planner 构建成功，耗时6分24秒；依赖既有 lunar_planning_msgs underlay。全新 build/install/logs 未与原安装混用。
- 核心包 CTest 串行 19/21 组通过；fine_traversability_builder 的非法包络测试抛异常退出，request_local_start_patch 的假设坡面软代价相差约0.08。核心 src/test 与基线98fb6081完全一致，本轮核心仅新增独立convex_polygon头；尚不认定为本轮回归，也未修复。
- ROS包 CTest 15/16 组通过；elevation_pipeline_test 的ObservedOnlyDeltaMatchesFullWithUnchangedNavigationFields断言观测应为BLOCKED，实际FREE，待分析。地图统一的exploration_map_node/projector/exporter/facts_codec测试通过。
- 规划器 CTest 10/13 组通过。失败组：shutdown（SciPy对NumPy1.25的警告污染READY输出）、execution_feedback_runtime（预检判据断言）、configuration（能力配置断言）。不能作全包通过结论。
- 本轮Python事实编码/发布专项在实际Humble Python环境51通过，1条NumPy/SciPy兼容警告。
- 实际DDS跨进程：ROS_DOMAIN_ID=174、localhost私有域，发布逐格平地观测、模拟odom和map→odom静态TF。20cm→5cm→20cm均ready=true，epoch=777，fine_revision依次1/2/3，均完整快照。图内只有/map_unification_test/*及ROS标准诊断话题，没有车辆控制发布。测试节点均已退出。首轮探针遗漏TF导致INPUT_UNAVAILABLE，补齐测试输入后通过，未修改实现判据。

远端证据：build.log、core-test.log、ros-test.log、planner-test.log、facts-python-final.log、dds-result.json、dds-probe-final.log及dds-node.log，均在上述隔离目录。DDS探针是合成地图接口测试，不是实时P3输入、导航规划成功或车辆闭环。尚未进行真实P3共享生产链、完整launch构建、固定Actor、Orin实车、bag回放或导航/探索闭环；这些仍为NOT_RUN。保留正式版本，先定位整包失败再考虑部署。


## 失败项根因复核（同日）

前次仅据测试组摘要，曾将 execution_feedback_runtime 表述为“预检判据断言失败”。完整日志证明六个用例均在导入前 StopIteration，尚未执行判据；该表述以本节更正。

|失败项|查实的问题与原因|归属与影响|
|---|---|---|
|execution_feedback_runtime 六用例|隔离上传只含 scripts/navigation/one_cm，漏传 scripts/simulation/p3_input_timing.py。补齐同提交 simulation 源文件后整个测试组通过。|本次验证打包遗漏，不能据此认定运行时判据错误。|
|configuration 的平台合成导入|同样漏传 scripts/simulation，最初 ModuleNotFoundError。补齐后实际暴露下述配置断言。|验证输入不完整。|
|configuration 发布器数量|测试写死 node.count(create_publisher)==5；本轮新增可选事实发布器后实际6。|本轮测试未同步，需改为验证话题及条件启用行为，不能把数量当职责或正确性合同。|
|configuration 速度断言|断言仍期待0.2/0.1，基线 tcp_wheel_speed.yaml 已在ade931b3统一为0.1/0.1，合成代码确实使用覆盖配置，制动参数未变。|旧测试期望失配，不应改回车辆速度以迎合断言。|
|shutdown|把stderr并入stdout，假定第一行一定READY。先被NumPy1.25/SciPy版本警告污染；屏蔽Python警告后又被CycloneDDS的lo无多播提示污染。仅在隔离测试副本跳过READY前诊断后，两个真实SIGINT用例均通过。|测试协议脆弱；不能据此判定运行时停机顺序有错。依赖版本警告是独立环境问题，未修改全局Python环境。|
|fine_traversability_builder非法包络|正确抛出了invalid_argument，但没有展开到EXPECT_THROW。O0重编调用者及实现仍中止；gdb显示大帧展开损坏。独立、不依赖项目的最小C++程序在当前aarch64 GCC11.4环境中：1KB局部栈数组时异常捕获成功，1MB时直接terminate。|真实大栈帧异常展开故障；不精确归咎于某一个编译器/运行库组件。派生函数持有多份tile状态/代价数组，应消除大栈帧，再复测所有异常与正常路径。仅提前验证参数不能解决后续其他异常。|
|request_local_start_patch斜坡代价|普通FineBuilder用MakePlatformWindowElevationEvaluator，按平台尺度测坡并去除背景坡度后算起伏；BoundedSupportEvaluator仍用旧MakePlatformElevationEvaluator，其3x3邻域直接以最高最低高程差算relief。测试斜坡每格高差0.02m，3格跨度0.04m，relief归一化平方乘权重2形成约0.08额外代价。|真实继承的算法口径不一致；起点假设地形与普通地形须统一评价方法，同时保持假设只限请求局部、不写回观测。|
|elevation_pipeline observed-only增量|测试给平坦高程塞入人工relief=2并期待BLOCKED；现有普通导出使用平台窗口从高程重测，平地结果是FREE。测试与实现都未在本专题新引入。|该测试的人工测量前提已与平台窗口来源合同不符；不能用此fixture证明观测增量有错。应使用真正改变评价结果的观测/高程构造变化，并继续验证增量与全量逐字段一致。共享事实路径继续消费已提交分类，不应为修测试重开一套评价器。|

追加证据仍在远端同一隔离目录：planner-diagnosis.log（补齐文件后回归）、shutdown-probe-result.log（2通过）、exception-stack.log、exception-diagnosis/large-frame.cpp及frame1024/frame1048576、both-o0.log。最小异常复现未使用ROS、gtest或项目算法。检查Core src/test与基线无差异，不能因此忽略真实继承问题。

建议修复顺序：补齐验证打包与更新失配测试；消除大栈帧；统一起点假设地形评价；用符合当前观测合同的用例重验增量导出。随后完整串行跑Humble包测试，再接实时P3。此次只作原因定位与隔离实验，未修改产品算法、未切正式P4、未发布速度。


## 用户裁决后的实施与目标复测

提交：114d930c（测试合同）、bd576867（消除大栈缓冲及按值参数）、b4a81b86（取消起点补图）、10b6b46c（对齐既有静止转向标签）。全部在原专题推进，不合并/推送；正式P4/P3未切换。

- 用户不需要起点盲区补图：删除增量起点平面拟合、假设高程及UNKNOWN覆盖；legacy补图源/头逐文件删除，planner、缓存身份、CMake与bundle同步清理。保留已知FREE的正常连接与既有轮式静止转向，不新增足式能力或准入条件。旧blind margin兼容忽略；没有统一或继续维护那套起点假设地形评价器。
- 发布器不作固定数量限制：改为按话题及backend/controller/execution条件验证；速度从基础配置+TCP覆盖配置计算，当前TCP配置0.1m/s及0.1rad/s，另测自定义速度。未为迎合旧断言改变配置。
- 观测测试以现有算法为准：用真实高程起伏及中心清平表达本征分类变化；保持导航膨胀状态/代价相同，仍逐字段验证delta/full重建一致。没有重新引入人工特征覆盖真实高程的分类来源。
- shutdown仅修测试有界日志标记等待，保留诊断、子进程错误、真实SIGINT及ROS context顺序验证。

工具链定位：测试机aarch64，GCC11.4.0-1ubuntu1~22.04.3，libgcc-s1/libstdc++6为12.3.0-1ubuntu1~22.04.3。具体故障在GCC AArch64栈探测的CFI/CFA偏移生成；不是凭版本不同认定libgcc冲突。最小程序中1KB栈捕获成功、1MB中止；同1MB程序显式-fstack-clash-protection失败，-fno-stack-clash-protection成功。反汇编/展开表显示先保存寄存器后探测时未累计先前栈偏移，符合[GCC PR119610官方修复](https://gcc.gnu.org/pipermail/gcc-patches/2025-April/680637.html)。

产品修复保留默认-fstack-clash-protection和-fstack-protector-strong：工作数组堆化、比较immutable旧tile而非再复制旧数组、两个tile构造改const&避免大值参数。实际-O3 -fstack-usage比较DeriveImpl：1180768→1376字节；新库完整FineBuilder27/27通过。系统工具链未升级/打补丁，独立大栈复现仍会失败，不能宣称系统GCC已修好；本轮解决项目对缺陷的触发。

Humble实际复测（远端目录仍/home/yanfa/P4/build/map-unification-cd27fa78）：

|范围|结果|产物|
|---|---|---|
|四包隔离构建（含legacy pure core）|4包成功，9分6秒|repaired-build.log|
|增量核心全部串行CTest|21/21组通过|repaired-core-final-test.log|
|ROS地图/消息/导出全部CTest|16/16组通过|repaired-ros-final-test.log|
|legacy pure core全部CTest|21/22组通过；anytime_wheel组超时|repaired-pure-test.log|
|legacy300m性能用例无编译负载单独重试|仍未通过原有1秒断言|repaired-pure-300m-alone.log|
|Linux bundle生成与脚本语法|1通过|repaired-bundle-final-test.log|
|本机受影响Python回归|1551通过/29跳过|本轮终端输出（52.92秒）；native DLL边界沿前述记录|

完整legacy性能资格仍未通过：该组包含750m长场景，遵守原有时限未放宽。除此之外的dual_mode/cache等组已通过。早期编译负载下ROS调试话题查询失败，在新安装包整组复跑后16/16通过，未改该无关测试或运行逻辑。

独立复审覆盖起点接口、UNKNOWN不补/真FREE正向路径、缓存闭包与堆缓冲生命周期，未发现新增P1/P2。当前仅隔离源码构建和测试，真实P3链路、车辆导航/探索闭环、足式实车/Actor资格仍未验收。

最终同一新安装包补验：规划器CTest13/13组通过（repaired-planner-final-test.log，37.27秒）；私有DDS20→5→20cm仍全部ready=true，fine_revision=1/2/3（repaired-dds-final.log）。测试节点均退出，无车辆指令发布。先前缺脚本与日志污染导致的测试失败已闭合；唯一保留未通过范围为legacy长距离性能组。
