# 可信度开发线兼容主线验证（2026-10-16）

## 范围与身份

所属长期分支 dev/terrain-confidence；专题 chore/confidence-integration-20261016。
开发基线 883175a7f7d6952a8a5841deca3990440e0a220e，合入公共主线
490333080a25c78b2298dc26a94c7e2b827ef207。此文记录专题候选结果，最终回合由主任务独立审查。
源码位于 /home/kai/CodexDownloads/lunar_navigation/branch-integration-20261016/worktrees/confidence-integration；
构建、安装和日志位于同一 stage 下 task-2-humble，未复用旧源码缓存、安装目录或 Jazzy 输出。

## 冲突与兼容决策

- AGENTS.md：采用已授权六长期线规则，保留可信度专题文档入口。
- core/CMakeLists.txt：保留观测估计及三组候选拟合测试，纳入 rolling_update_policy 和 phase_motion 等主线源及测试，无重复注册。
- phase_aware_path_simplifier.cpp：使用共享 CertifySegment 并传播 SearchControl，保留 TerrainSegmentCost 累加与捷径代价不增加约束。
- wheel_local_astar.cpp：使用主线线段认证取代旧对角邻格判断，保留 RequestCellCache 与可信度遍历代价。
- platform_elevation_physics.cpp：默认构造保留按 profile 选择方法的共享观测缓存；注入构造保留主线支撑评价器与空指针拒绝语义。
- incremental_navigation_node.cpp：同时保留 terrain_background、confidence_weight、terrain_fit_method 与主线 goal tolerance、异步 safe-prefix、确认到达逻辑。
- incremental_navigation_node_test.cpp：同时保留参数/拟合后端与部署默认/到达/异步回归。

自动合并还涉及 request_local_start_patch.cpp：BoundedSupportEvaluator 原先默认工厂会把
请求选择退回 baseline，现显式传递背景窗和 fit_method；支撑平面采样仍按既有 baseline 逻辑，
派生支撑环只用于求导，不扩展运动假定域。MakePlatformElevationEvaluator 新增可选方法参数，
旧调用仍默认 baseline。未恢复滤波、未改候选拟合算法。

新增两个行为回归覆盖默认 FineCellEvaluator 与请求级支撑的候选方法传递。
分别人为退回 baseline 后，两项均失败（mutation-fine-red.log、mutation-support-red.log），
恢复正确实现后完整构建和全部测试通过；测试验证曲面代价/高程行为而非源码文本。

## 当前 Humble 验证

本机 /opt/ros/humble，Release，构建 -j2、CTest -j1。消息依赖只读使用
/mnt/data/WS/lunar-navigation/.runtime/region-relation-v2/native-humble/install/lunar_planning_msgs。
ROS 测试使用 ROS_DOMAIN_ID=224、ROS_LOCALHOST_ONLY=1，日志写 task-2-humble/ros-logs。

| 检查 | 结果 | 日志 |
|---|---|---|
| core configure/build/install、CTest | 29组、418用例，零失败/错误/禁用 | logs/configure-core.log、build-core-final.log、install-core.log、test-core.log |
| ROS configure/build/install、CTest | 13组、107用例，零失败/错误/禁用 | logs/configure-ros.log、build-ros.log、install-ros.log、test-ros.log |
| 接口/隔离/消息/launch pytest | 68通过 | logs/pytest-contracts-final.log |
| controller execution/tracking pytest | 90通过 | logs/pytest-controller-final.log |
| diff空白、文本UTF-8 | 通过 | 提交前检查 |

静态契约修正：独立节点32m与统一YAML64m是两个不同启动入口，测试执行实际launch参数合成函数，
验证YAML覆盖确实传入导航节点；OccupancyGrid测试按成员、主题、QoS绑定及可选发布开关
验证探索图与局部细图两个通道，不以全文件固定数量替代接口契约。
额外复现旧 isolation 白名单遗漏主线已有 wheel_orin.yaml/legged_orin.yaml，
补入明确配置并继续检查它们没有被排除的运行输入。生产参数未为通过测试而更改。

初次 controller 收集因独立消息包 Python 路径未加载而失败；显式设置消息包
local/lib/python3.10/dist-packages、动态库和ament前缀后90项通过，原失败日志保留。
standalone CMake安装没有顶层local_setup.bash，未改动共享运行环境。

## 同输入开发线合并前后对照

基线883175a7用 git archive 导出至 task-2-humble/baseline-source，重新构建独立核心库；
两版均重新编译仓库既有 probe.cpp。采用现有protocol生成4组81×81、5cm输入：
平地密集、平地缺测50%、凸面1.5cm噪声、15cm隐藏沟遮挡；固定种子101。
五方法、两版本，共40次串行探针，每次含足式与轮式。配置固定权重2、最低评分0.2、
残差0.03；未启动训练、车辆或仿真服务。

20个输入/方法配对的ground六通道、足/轮state/confidence及路径CSV逐字节一致，
规划status/reason/reached/path_points一致。输入与探针/库哈希、完整命令、各次计时保存于
fixed-comparison/comparison.json和logs/fixed-comparison.log。
这是 dev旧实现→兼容合并后 的有限对照，不是无可信度主线→可信度效果实验。
计时为当前宿主共享负载下单次采样，不代表隔离性能或Orin时延；本探针未测增量更新耗时。
核心既有增量一致性、切换缓存、障碍/共线/空白/平台/路径代价与取消回归同时通过。

## 应用来源82项

沿用已核验 confidence_source_triage.json/.txt：69已采用、9应用适配、4已替代，零遗漏。
本轮每个存在的应用/类别组合抽取1项，共8项，当前SHA256全部匹配原备份；
证据 task-2-app-samples.json。未重迁应用旧实现，未改三个原应用工作树。
应用配置补丁保留在 docs/terrain-confidence/application-adapters/，应用落账由所属任务处理。

## 晋级边界

已完成公共修复兼容合回可信度开发线所需候选提交与当前Humble验证，默认仍为baseline，
四种方法仍为可选；当前证据不足以在本任务中直接决定整模块晋级主线。
历史P3冻结目录在当前主机指定旧路径及有限查找范围内不可用，P3重跑 NOT_RUN。
完整矩阵、同输入全量/增量性能对照及无机制主线效果对照 NOT_RUN；
Orin性能、真实DDS输入、同步回放、TCP动力学和车辆闭环 NOT_RUN。
本机localhost ROS测试不等于上述部署验证，也不恢复历史Jazzy完成声明。
主任务可审查此兼容结果并回合dev；向主线晋级需据完整门槛作独立决定。
