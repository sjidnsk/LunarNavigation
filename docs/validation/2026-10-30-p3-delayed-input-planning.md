# P3延迟输入无控制联调记录

任务 P3-PLANNING-DELAY-20261030；owner integration/pure-planner-orin；专题 hk/p3-delayed-input-20261030，基线 eabbfc4。本轮仅新P4可写，P3与其他目录只读；未启动跟踪器、TCP车辆桥或发送车辆控制。不训练。

## 输入实测与适配边界

216 ROS2 Humble/aarch64，domain57，CycloneDDS。输入为 /Car/T3/mapping/grid_map、/Car/T3/localization/odometry、/tf、/tf_static。45秒样本：地图22条，源年龄min4.053/p50 4.510/p95 4.930/max5.161秒，最大接收间隔2.659秒；定位52条，源年龄min2.055/p50 2.222/p95 2.883/max3.074秒，最大接收间隔1.954秒，17次重复源戳、0次倒退。年龄由同机ROS接收时刻减源戳得到，不等同独立标定的传感器物理延迟。

map与odom消息frame均odom，子坐标base_link；观测到map->odom固定单位静态变换。当前脚本只接受此条件，变换改变会暂停，未实现或宣称动态TF历史补偿。

真实故障：Python规划节点静态TF订阅深度1，晚加入时漏收同一发布端较早分批发送的map->odom边，造成MAP_WAITING_TF。生产修复仅将该订阅深度改为100，与现有C++消费者一致。真实DDS回归先发布12条分批静态TF再加入节点；修复前1失败2通过，修复后3通过。

新增只读调试脚本区分源年龄、接收断流与源戳冻结；重复戳不算推进，时钟倒退与源回退拒绝。不重标时间戳。地图8秒、定位6秒为新观测工具的有效性预算，未改变执行器超时。高负载下仍有超期并正确取消/暂停的失败记录，不能称严格运动时延补偿。

## 实际结果与未通过项

P4/run/p3-planning-20261030 保存measure-r1和各轮失败原始日志。r1暴露TF丢失；r2/r3澄清GetPolicyMap.ready还包含起点静止支撑认证，并取消重复大地图服务轮询。r4-r7暴露观测门禁、真实输入断续与取消失败；均保留，未用放宽执行门禁获得成功。

r8全局正式结果满足planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true；收到25条全局路径、0条局部路径，真实反馈NO_CERTIFIED_ACTION。细地图有revision76及1个tile，但anchor_center_state=0 UNKNOWN、stationary_support_ready=false。探索检测到2个前沿簇、0候选、0可达候选，无选定目标。起点UNKNOWN与零候选仅是同时观测的事实，尚未证明唯一因果。r8旧脚本曾误把Action状态0当终态，该判断已修复并有回归；r8不能作为完整取消验收证据。

observer-r10只保留暂停观察，未发人工目标；2前沿、0候选、PAUSED，整体ok=false。脚本不模拟到达，完整成功仍要求局部参考与真实探索目标交接，不能用前沿显示代替。

用户告知tcp2已完成平台/UNKNOWN调整后，核对TCP应用线eca3087：速度0.1m/s、角速度0.1950770225306774rad/s；共享入口allow_unknown默认false，TCP OBJ入口显式true，规划器与控制器同源。tcp2独立测试记录139条局部路径，但EXECUTION_INPUT_UNAVAILABLE:missing_or_stale_feedback、未起步；该结果不属于本轮执行验收。旧默认UNKNOWN=false的r8结论不能代表新TCP配置。tcp2正在独立专题处理控制侧反馈时序，本任务不覆盖其配置、源码或安装态。

## 验证与处置

纯时间健康与无控制脚本测试46 passed。216受影响规划包构建通过，包测试colcon汇总48 tests、0 errors/failures/skipped；日志P4/log/p3-delay-package-test.log与p3-delay-package-result.log。真实静态TF回归包含在包测试内，不能将不同层级简单相加为闭环数量。独立只读审查无新增实质阻断。

RViz配置scripts/simulation/p3_planning_observation.rviz；浏览器入口http://127.0.0.1:6087/vnc.html?autoconnect=true&resize=scale。原始P3点云/定位/TF与隔离规划前沿显示；软件GL的Map索引着色器不兼容，因此禁用该显示，用现有真实task_map_markers展示栅格。未伪造地图。显示服务与缓存仅P4/display，无系统安装。

为避免与tcp2运行接口重叠，已对命令身份核验后SIGTERM本任务observer-r10 PID49207；其监督器有界取消并退出自己的launch进程组。RViz及P3保留，不停止其他会话控制/测试。最终observer-r10/result.json为FINISHED、operator_stopped=true、launch_returncode=0；记录过地图源年龄8.044秒触发SOURCE_TOO_OLD，保留stay_fault，不能把初始READY快照写成持续通过。退出时两个命令话题发布者均为空。本轮源码本地提交待合入，不push、不移动main/TCP应用线；专题与工作树保留待授权处置。

剩余：按新TCP配置分别复核探索候选过滤和起点支撑；控制反馈时序交由tcp2当前任务，避免双写。自然到达、车辆控制、移动时定位外推、动态TF历史、Orin部署和实车均NOT_RUN。当前成果是延迟输入诊断/观测适配与静态TF修复，不是完整导航探索闭环通过。
