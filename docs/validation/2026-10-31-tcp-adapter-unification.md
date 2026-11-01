# TCP运行/测试入口统一

基线ee85651，hk/p3-pose-wait-20261030/main-worktree，ROOT单写入。用户按既定顺序继续处理TCP入口重复。只改本地平台工具及测试，不连接/上传/重启远端，不切换原运行安装，不发车辆命令，不修改tcp2候选或其他工作树。

## 原问题

p4_tcp_runtime.py与check_tcp_navigation.py各自维护低速profile、launch参数、环境、限速、auto/park、租约心跳、通信断档和停稳取证。测试脚本还在速度转发前要求Action handle和TrackingStatus.reference_accepted，并二次检查定位流；这些已经偏离现运行入口只负责平台传输的职责。其结果是同一控制器输出可能在运行入口转发、测试入口却被扣住。

## 统一方式

沿用p4_tcp_runtime.py作为公共平台工具位置，不新建服务、ROS消息、任务身份、权限协议或计时器。TcpCommandRelay替代两套内嵌的armed、lease、命令与telemetry时间状态，而非叠加另一层状态。ROS发布器以函数注入，组件本身不导入ROS，可离线验证真实转发逻辑。

- low_speed_profile：由所选能力生成低速副本，不改原配置、不把低于调试档的能力提高。运行档仍为0.1m/s、0.1950770225306774rad/s，测试档仍为0.05m/s、0.1rad/s；实际各轴上限来自所选配置，含正反向分量。
- launch_settings：两入口共用map/odom/TF/UNKNOWN、既有时序预算、起步forward及控制参数装配。测试仅传私有prefix/goal_topic，不再自行另拼一套默认值；运行RViz仍用/goal_pose，测试私有目标不受其影响。
- runtime_environment：共同装配域10、RMW、SDK/venv路径和隔离日志；测试私有节点launch副本及目标/时限/取证仍由runner持有。
- TcpCommandRelay：统一合法指令校验、auto切换、原消息转发、zero/park、原租约心跳、通信断档停车、真实telemetry停稳证据。数值比较容差由同一函数作用于配置上限，不再在两份脚本另写限速数字。

测试入口删除Action handle/reference_accepted及里程计再判活的转发门禁。TrackingStatus仍可观察记录，预检仍按导航/跟踪运行合同核对P3；这些数据不再成为平台转发的第二份导航判定。平台仍要求原有车辆控制租约、控制器命令与车辆telemetry持续更新，没有将P3源数据延迟当作命令流延迟。

## 停车和退出

command_ready沿用0.5秒预算，时钟不单调/命令或telemetry断档即park；未新增超时或恢复等待。实测停稳沿用telemetry年龄严格小于0.5秒、vx/vy小于0.01、四轮速小于0.05的条件，并显式拒绝非有限数，避免NaN被max比较掩盖。

有界测试在take-control前维持relay.leased=False，preflight-only即使收到控制器消息，最终也不发布速度、模式或租约。运行入口--check-only仍仅输入桥，不创建转发组件。

收尾先begin_stop，取消/等待期间仍发送原租约心跳和zero/park；此时新的速度回调不能重新auto。释放后所有运动输入直接拒绝。独立复审捕获的“NaN/越限队列消息再次抛错打断finally”已补2项RED并修复：未接管/正在停止的判断先于读取/校验运动消息。活动运行中的非法指令仍停车并上报异常，未放行。

原auto_mode_entered日志保留，断流观察为relay_transport_gap_park；测试结果的auto_mode_entered改由实际非零转发记录派生，避免清理后的armed=False抹掉曾进入auto的事实。测试75秒上限、0.8m位移界、目标取消/终态取证和预检45秒总时限保留，不将这些测试时限变成长期导航任务失败策略。

## 验证与限制

初轮16项RED、8项原测试通过，确认缺共享接口及测试回调未调用平台适配。实现后24通过。新增遥测NaN停稳2项RED与收尾非法输入2项RED均修复。最终五个测试文件147项通过：

- tests/test_p4_tcp_runtime.py
- tests/test_p3_input_timing.py
- tests/test_p3_planning_debug.py
- tests/test_navigation_guard_responsibilities.py
- tests/launch/test_nav1cm_exploration_launch.py

覆盖私有接线、低速能力不提升、正反向限速、非有限/超限、未租约不发布、零命令、0.5秒断流、停止后合法/非法回调、释放与遥测停止、P3延迟/断流策略及启动合同。测试入口command_input通过AST执行真实方法体隔离ROS顶层，不用车辆替身结果冒充节点联调。独立只读复审确认收尾缺陷闭合，并复跑其两项专项回归通过。

UTF-8、Python编译与git diff --check通过。未改ROS消息或包依赖；未做Humble节点/真实DDS/GUI租约/TCP车辆运动验证，均NOT_RUN。平台脚本共同部署前需同步两份入口及既有debug_p3_planning/p3_input_timing依赖；不能只上传其中一个文件。原先台式圆模型SDK通过不代表本次TCP接线已通过。

## 处置

本地源码/测试/计划/本记录及原任务登记提交待合入。长期线不移动，不push/merge；调试机原文件与运行进程未修改。本轮不处理双向加减速算法、圆包络、探索地图/失败记忆或参数热更新。
