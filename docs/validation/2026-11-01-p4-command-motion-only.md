# P4指令认证与原始接口交接记录

沿P4-CONTINUOUS-GOAL-ACCEPTANCE-20261101，专题hk/tcp-continuous-handoff-20261101，ROOT唯一部署。用户要求去掉指令“响应→持令→额外停车”中的附加停车，并将2.5s持令恢复1.2s；后又要求恒速诊断、原始Env_X接口，最后明确“不用验证，打开RViz手测”。

## 已完成源码与配置

7276afe恢复config/tcp_p3_feedback.yaml的command_hold_s=1.2及对应既有默认断言（55项本地通过）。692365e使command_hold_proof只包含响应+持令，两处指令消费者都显式关闭认证器隐式停车尾。真实模式切换响应、取消和到达制动、实际轨迹动力学/能力/扫掠检查保持；已有Certification增一个范围布尔值，motion-only证书不能满足默认完整证明，未增实体或ROS消息。

TDD先观察RED，再定向115通过，独立复审105通过无阻断。4个旧test_pose_goal::test_arrival_requires_braking_endpoint_inside_pose_tolerances [yaw-False/yaw-True/position-False/position-True]在7276afe独立基线真实复现失败，不属于本批；未修改其历史断言。基线导出位于D:/CodexDownloads/lunar_navigation/command-no-tail-baseline-7276afe-20261101。

实际生产包采用8e35ae4已部署源清单，加本轮3份源码差异及1.2配置，刻意排除567f3a4几何缓存。冻结D:/CodexDownloads/lunar_navigation/command-motion-only-upload-692365e-20261101.json保存7份必要源码/测试/配置和SHA。P4新隔离源129项校验、Humble构建、368通过5跳过、11/11 CTest和域169实际DDS通过。回执P4/artifacts/command-motion-validation-692365e-20261101.json。停稳后仅替换4份生产文件，operator144494运行tcp-interactive-command-motion-692365e-20261101。

2米Action自然GOAL_REACHED；P3实际误差.0422108m/.0170435rad。399条命令，132次同目标交接无认证插零，初始起步完成1次；仍有3次GOAL_BRAKING后恢复运动，不冒称所有停顿解决。认证p50约29.76ms、p95约44.29ms，场景与旧回合不同非严格性能因果。回执warm-handover-field[-summary]-692365e-20261101.json，SHA52f4362f96d1731a02943788530d2f04a587495877634a98aa69d969d07a92a9；实际3模块SHA在回执中核对。

## 被动采样与用户终点问题暂放

60秒speed-stutter-692365e-20261101：605条P4/car命令完全同序；前40秒无插零，末20秒13次MODE归零全部用冻结速度目标/P3估计在空图离线复现。数值线速度约1e-11已判为spin，但线加速度硬判据仍拒绝；这不是碰撞，也不是已移除停车尾。用户明确暂不处理终点MODE，未作修复。命令间隔p50 .0688s、p95 .3252s、max .7324s；P3有效源帧间隔中位1.451s，源龄中位2.373s，未新增新鲜度门禁。

追加20秒只读轮指令：100个5Hz输出，68个最近旋转指令匹配样本中45个四轮零速，符合现有3s转向准备逻辑；仅诊断，不用作导航反馈。旧接口源码硬编码20Hz定时器分频4得到5Hz。相对P3源时轨迹，稳定命令区间位移比例约.609、估计沿前进方向偏前中位.218m；三条速度链尚未校准，不能据此直接添加倍率。原始telemetry wz单位/语义未校对，不作为角速度真值。

用户要求无导航恒速：域10唯一/car发布者，20Hz .2m/s、w0持续20s，393条值完全一致，指令间隔max .057s；稳定段95个轮指令全为1.6666666rad/s、转角0、无轮速插零。用户仍感顿挫。再只读加载start_gui使用的原始安装LunarCarGuiNode（SHA1de59825c46aa037e79f3d480314c0a76403240a9f5dc8441918bd25dcf32435），10Hz、auto一次，同样20s，用户确认仍顿挫。两轮均停车并恢复入口，旧目标不自动重放。对应P4/artifacts/constant-v02-*、gui-reference-v02-*。未修改Env_X文件。

## TCP重复连接与最终接口

TCP快照发现两条6668连接：52191（P4_legacy接口，域10）与151018（Env_X原始节点，域57）。两者都5Hz发送；后者启动于1793480847.81，晚于第一轮恒速测试1793480184.59，故双连接不能解释所有历史顿挫，但会污染后续原GUI对照。源端6665采集连接独立，控制口RTT约30–42ms、队列通常0/偶发53字节，快照没有证明重传导致顿挫。

用户指定仅使用Env_X/InterFace，并停止进一步验证、改由RViz手测。已停旧operator151431、两个车辆实例52191/151018；只读执行/home/yanfa/Env_X/InterFace/install/lunar_car_ctrl/lib/lunar_car_ctrl/lunar_car_node，域10，唯一6668连接为vehicle PID153802。没有构建/改写Env_X或P3。参数/环境/日志只在P4/run/vehicle-envx-domain10-20261101：max_linear_speed=.2、spin_max_linear_speed=.1*hypot(.815,.622)/2=.0512618，与P4平台上限对齐；原始启动默认max_linear_speed=.1，不能把GUI输入.2当作必定执行.2。保留原3s准备、5度步进和1s watchdog；移除旧接口环境搜索路径。

当前P4 operator153825，运行P4/run/tcp-interactive-envx-domain10-20261101，navigation launch153850、RViz153851；RViz OpenGL成功且地图211x214已加载。控制域10/P3源域57，持令1.2s、无附加指令停车尾，N20/.2离散/.1规划/.05控制、线速.2/角速.1。没有自动目标或非零运动测试。原始Env_X节点不含P4私有UE path7/8扩展，当前用户入口为RViz；未宣称保留该私有TCP目标功能。

切换停车辅助脚本初次因显式Context却使用默认executor而失败，车已停车；仅P4脚本移除无必要的spin_once后继续交接。旧失败回执envx-domain10-handover-first-failure-20261101.json保留。最终回执envx-domain10-handover-20261101.json确认两个旧PID退出、唯一新连接和motion_test_run=false。用户已明确后续自行测试，未继续验证。

源码/配置与本记录已提交，待长期线合入；没有merge/push或删除工作树。TCP仿真/P3组件证据不等于Orin/实体车辆验收。后续恢复从当前Env_X单接口配置继续，不能重启旧部署脚本里的52191或恢复双连接。

## 2026-11-01 TCP、P3、P4整套重启

按用户明确重启请求，运行目录更新为 `/home/yanfa/P4/run/restart-tcp-p3-p4-20261101-053837`。Env_X原始车辆接口PID156544、采集156545、P3 pipeline156548/relay156550/monitor156552/viewer156553、P4 operator156555。控制与原始采集域10，P3融合域57，由现有relay完成10→57图像归一化及传感器桥接。

首轮053402错误复用采集域57，导致relay源域10收不到图像；虽收到31帧雷达融合，之后视觉无输入、定位失效。已保留首轮证据，纠正P4启动脚本的采集域为10后整套重新初始化，避免续用失去视觉期间的融合状态。第二轮实读localization_valid=true、continuous_visual_pose、视觉已锚定，relay融合计数20→22、地图已生成14帧，RViz收到地图。TCP6665和6668各一条ESTAB。未修改P3或Env_X文件，未改变算法、运动上限和1.2秒持令配置。

无自动目标、无旧目标重放、无自主运动复测；RViz日志出现用户新选目标。启动回执 `P4/artifacts/restart-tcp-p3-p4-latest-20261101.json`，修正启动脚本 `P4/artifacts/restart_tcp_p3_p4_envx_source10_20261101.py`。此处仅证明重启与数据恢复，不新增导航到达验收。
