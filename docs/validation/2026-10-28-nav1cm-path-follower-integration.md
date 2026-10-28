# NAV1CM 与标准 Path 真实跟踪器联合验证

任务归属 integration/pure-planner-orin；专题 hk/nav-1cm-feasibility-20261026。
产品基线 46f635e339fa70fdf116454bc1d287733efa395b。默认入口保持不变。

## 链路与隔离

真实安装 PlannerNode/create_resources/LocalWorker → nav_msgs/Path →
真实安装 PureWheeledControllerNode/StandardPathFollower → geometry_msgs/Twist →
独立 plant 积分 → nav_msgs/Odometry、零 stamp map←odom 静态 TF。
地图为实际 grid_map_msgs/GridMap，direct_p3 由实际 native 地形与距离场处理。

所有跨进程均固定 ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1、PYTHONNOUSERSITE=1。
命令仅 /nav1cm_integration/cmd_vel，实际 ROS 图必须没有 /Car/T5/Car_Cmd_Vel 发布者。
图记录保留发布者/订阅者、类型及 QoS，不把旧消费者结果当真实跟踪器结果。

运行前 source 已验证 SDK、core underlay 和 task2-fix1-build-r1/install/setup.bash，
移除全部旧源码 PYTHONPATH，保留必需 acados_template。安装 receipt 中50个Python文件
逐SHA核对；各进程另记录 pid/pgid/start_ticks/路径/参数与实际 native SHA。
harness-sha256.json绑定两组验证工具源码；生产代码只从安装目录加载。

## plant 与判据

plant 只读取实际收到的 Twist，不根据 Path 或 goal 生成控制。
完整墙钟间隔细分为≤.01s，纵/角速度以.5的加速度上限逼近命令，
中点积分并独立记录末速；这是理想有界执行模型，不能推断实车响应。
控制器 nominal_response_time_s=0，保留其余生产保守预算。
原始命令、路径、诊断、全步状态、地图/目标事件均按进程追加写入。

独立完整矩形 SAT 使用1.182×.818m车体，对解析障碍进行细分扫掠；
以相邻姿态间最大角向/线向运动界膨胀，覆盖采样之间的车体运动。
不调用产品 certify，不以中心点、可视化或参考轨迹代替真实车体安全。
原始速度≤.2、角速度≤1、加速度/角加速度≤.5，行进曲率≤1。
到达要求xy≤.1m、yaw≤.05rad、实际v/w≤.01连续保持≥.3s。
正式规划必须0/PLAN_FOUND/true且请求goal、revision/generation、route接受身份一致。

独立门禁拒绝无真实运动、错误ROS类型、无正式来源、未安装来源、
碰撞/限值、提前退出（包括0）、worker残留和监督异常。有限局部终点不等于全局完成。
Supervisor仅清理自身创建的精确进程组；升级信号或残留仍否决验收。

## 场景和运行

run.py支持 forward_spin、reverse、obstacle、mapchange、empty_smoke。
前四项为真实联合链；empty_smoke明确由fixture注入标准Path，不能声称planner产生empty。
正向含真实worker请求250ms延迟、滚动Path和实际planner停止；mapchange不重发目标且热换源分辨率。
反向明确检查负速度；障碍场景必须真的越过障碍及转角。
每次输出使用不存在的新目录，失败后不得覆盖summary或原始行。

入口：python3 path_follower_validation/run.py --output FRESH_RESULT
--build-dir ROOT/build/mpcc-tests
--installed-identity INTEGRATION/task2-fix1-build-r1/installed-identity.json
--cases forward_spin。

具体原始标签、失败归因和当前验收结果见同任务 task-3-report.md。
本文件不宣称所有场景通过。Jazzy、Orin、实车、真实rosbag与RViz均NOT_RUN。
源码、测试及本文可提交；运行日志、native库、地图、导出结果保留在工作树外。

## 实际 QoS 证据

验证器同时记录真实运行节点的publisher/subscription对象qos_profile，以及DDS发现端点的结构化QoS。
Path生产发布者、标准follower订阅者要求RELIABLE/VOLATILE/KEEP_LAST/depth1；
tf_static双方要求RELIABLE/TRANSIENT_LOCAL。所有记录绑定实际话题、node_name与方向。
DDS发现可能不披露history/depth，此时保留UNKNOWN/null，用对应运行实体的真实profile补证；
缺实体证据或已知QoS冲突均否决结果。此检查只在外部验收器，不改变产品准入、同步或控制逻辑。
旧r1-r4只有QoS文本，不能追认已通过新增结构化QoS门禁。
