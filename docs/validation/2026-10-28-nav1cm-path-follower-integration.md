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

## 当前实际验证结果（r5）

当前生产提交052aa55497fde3a9217ae11e1f4125199c876c7e，
Humble安装t3f01-fix1-build-r1。Task3状态NEEDS_CONTEXT，不能切换默认入口。

r5正向45s通过全部门禁：12次正式规划、780条非空local Path，
实际原目标于17.025s满足连续.3s终端hold；恢复运动后planner退出和陈旧Path停车通过，
250ms真实worker延迟继续运动、连续曲率、完整车体SAT、真实结构化QoS均通过。
实际最大线速.198853m/s、角速.525927rad/s，a/alpha最大.5；
follower compute P95=42.238ms、max105.880ms。

r5障碍48s仍失败：初始旋转从旧版本.079rad推进到1.652rad，
实际位移只有.003302m。非空Path在7.18s后停止，planner持续
NO_FEASIBLE_MPCC_CANDIDATE/FIXED_INITIAL_CIRCLE_INFEASIBLE，
制动拒绝UNKNOWN；独立车体SAT、数值约束、监督和QoS均通过。
该失败归planner局部地图/认证及MPCC owner进一步复现，不以正式规划成功或停稳代替绕障通过。

两个case实际core.so均来自ROOT/install/core-humble-r1/
lunar_incremental_navigation_core/lib/liblunar_incremental_navigation_core.so，
SHA256为0092ca1863b642f9bc0cbc7bb443b22446c26d1b29f153a3e217ce376e9bb4ec。
DDS发现不披露Path history/depth（UNKNOWN/null），运行实体直接证实
planner发布者与follower订阅者均RELIABLE/VOLATILE/KEEP_LAST/depth1。
两case所有进程和worker正常退出、无残留。新版本reverse/empty/mapchange最终matrix尚NOT_RUN。

完整原始证据保存在外部ART/task3-joint-r5-result，
archive SHA256为79cdd40198e2a87e2b82eb9651146a6db0cf8b8a89bb4d894dadbcbfa7850052。
实际图ART/task3-joint-r5-actual-motion-comparison.png已亲视。
旧r1-r4失败与历史通过均完整保留，不归入当前生产版本通过数量。

## 当前后续结果（r6，覆盖r5为最新验收状态）

F02生产963cba45811ab2947a6d13aff67c7e6c16e1de23，
真实安装t3f02-build-r1；harness导入隔离提交ad26ae64。
原完整core两项全局evaluate污染失败已修：focused43项通过，最终完整core
583passed/1skipped（57.15s），原SQP_RTI诊断与历史失败均保留。

r6正向45s与障碍48s仍FAIL；安全、正式来源、结构化QoS和清理通过，
但运动/终端未通过，Task3继续NEEDS_CONTEXT。
正向首次在13.122s报告MPPI_STOPPING_UNSAFE，
实际yaw=.05212而收到单调纯spin参考终点.6，随后FAILED585次。
障碍地图修复后非空Path持续、333次LOCAL_ACCEPTED，
但在9.411s首次报告同一停车失败，随后FAILED773次，实际总位移.003218m。
两项TRACKING_STALLED均为0。不能以旧r5正向PASS代替当前版本，也不把新失败归因旧UNKNOWN。

r5/r6生产50Python仅maps.py有变化，controller全部SHA、实际policy/参数一致。
实际core.so路径与SHA仍如前节，Path/tf_static QoS均有真实实体补证。
r6独立plant禁入检查误伤nav1cm_joint_validation等验证包名；
现已精确排除实际生产nav1cm及controller包及其子模块，focused44项通过，
原r6 raw/summary不改写，观察修复尚未实跑。
新963版本reverse/empty/mapchange仍NOT_RUN。须follower-owner复现停车认证失败后重新全matrix。

当前不可变原始结果ART/task3-joint-r6-result，
archivecccb8f631194a515f07634e0cf5d3b58269f44e8f62051ec71a5fbfe5d3c700c。
实际图ART/task3-joint-r6-stopping-failure.png已亲视，所有进程/worker均正常回收。

重复FAILED输出可能来自锁存，不证明每轮重新认证；需follower-owner区分首次停车认证失败和后续状态。
后续UPDATE增加调用前时钟/phase/failure、真实最近12条指令history和map identity小型只读快照。
plant真实产品禁入精确包含nav1cm、lunar_nav1cm_planner、lunar_pure_wheeled_controller及子模块，
不把验证工具别名当作生产代码；该增强观测尚待后续实际case验证。

## 最新候选结果（r7/F03）

生产b6e1ad2d791a3d5fe0b496a3b14b8aaaf217e73d，安装t3f03-build-r1；
F03独立spec/quality Approved后运行，451包检查/50安装Python身份已核对。
Task3仍NEEDS_CONTEXT，当前矩阵没有全部通过。

r7 forward45s全部门禁PASS，11次正式规划、780非空Path，
15.957s满足连续终端hold；250.325ms真实worker延迟继续运动、
恢复运动后planner停止/陈旧Path停车、完整车体/曲率/QoS/监督均通过。
增强before记录直接证明一次MPPI_STOPPING_UNSAFE瞬态恢复到FINAL_ALIGN/
非零角命令，之后完成到达；compute P95=34.218ms/max96.992ms。

r7 obstacle48s仍FAIL，尽管342次LOCAL_ACCEPTED、894非空Path持续，
实际位移仅.002338m/yaw1.66649。首次t7.33885s实测v/w0、before.failure=None，
返回FAILED/MPPI_STOPPING_UNSAFE；后续784次FAILED且恢复0次。
715条实际诊断明确stopping_retry=true/stop_certified=false/measured_stopped=true，
最后真实最近12条history均0仍不恢复；这不能简单归为“没有重试”。
完整before dt/now/state_age、command history与map origin/revision在follower.jsonl，
需原follower-owner量化复现停车恢复认证为何不许可。

两case实际独立plant生产禁入、结构化QoS、安装/native/core身份、
完整车体SAT/约束与进程/worker清理均PASS。旧r6观察误报不改写。
当前b6e1ad2版本reverse/empty/mapchange短矩阵尚NOT_RUN，不用历史结果补齐。
原始ART/task3-joint-r7-result保持不变，
archivef22a81286831bd88c2ff527bd615a399b84b35c4e2d917eb8b7e92b3e8737bc8；
实际图ART/task3-joint-r7-recovery-and-failure.png已亲视。


## T3F04 共享 intrinsic 栅格修复

F02 只修复 extract_window，center_support 仍逐 FREE 矩形 contained 裁剪，
分数原点导致接缝再经完整车体支撑膨胀。现两消费者统一使用 maps.rasterize_world，
保持精确原点/厘米格/500²及650² halo，FREE 并集全覆盖、BLOCKED 相交优先。
真实 UNKNOWN 孔洞、1cm缝隙、场边界、不可变快照、allow_unknown资格及native距离下界不变。

r7最终实际位姿(-.00022610096342229037,.0023270243419165264,1.66648642142486)，
8m P3/.05m源/elevation及原障碍经实际 NativeTerrain/生产create_p3_source构建。
epoch1及planner digest 7b7c8d05f3d1e8f2b976002128d05aeeca905c4f981c7cb7bf674965f25bf3e4
与原r7请求353等匹配；origin精确匹配revision7的记录。
中心支持 UNKNOWN 86066→37691，FREE101666→150041，BLOCKED62268不变，
实际末12条零命令时钟与state_age .002820814下完整certify_publication false→true。
此为具有地图身份匹配的受限历史重建，不是完整事件历史重放；原始r7证据未修改。

TDD最小RED2失败/4通过、P3/native RED1失败；GREEN31通过，独立面积oracle及
500²/650²/650×713边界22通过，P3/扫掠26通过；完整core609通过/1跳过，64.61s。
测试末尾有SQP_RTI错误状态3输出，pytest退出0且无失败，不将求解器日志抹除。
实际三包重建/CTest/Python与native身份完整回执见外部ART/task3-product-fix4-report.md。
本次没有改C++/ABI、停止证书或参数、harness、默认及应用入口。
新候选联合闭环、Jazzy、Orin、真实P3流/rosbag/实车NOT_RUN；独立复审和短联合验收交root。


## 当前完整矩阵 r8 / F04（覆盖旧结果为当前验收状态）

当前结论 **NEEDS_CONTEXT**：五例同次完整运行，4 PASS / obstacle FAIL；不具备默认切换条件。
生产与harness冻结提交均86115024bc4073fe9ea767ee0b0a8fc6af8fd290，
实际安装t3f04-build-r1；F04独立spec/quality PASS由root交接，452项包检查/50项Python身份由root复核。
本次helper入口再逐文件核对安装Python及三个native SHA，实际各进程INSTALL/NATIVE_LOADED记录保留；
产品从install加载，无source fallback。harness archive SHA256
`e56476d74e3178c44762a66b4f271908918537d877b2db4702f1fd313298e1b1`。

只执行一次：`task3_joint_humble.py task3-joint-r8 --cases forward_spin reverse obstacle mapchange empty_smoke --installed-label t3f04-build-r1`。
保持45/20/48/34/12秒原场景、原障碍/goal/plant/容差/门禁；本轮未改任何harness或产品。
Windows本机无ROS；实际运行环境KAI Ubuntu22.04/Humble/x86_64。完整raw已取回
`ART/task3-joint-r8-result`，远端同label保留；helper进程session61658自然exit1。
所有14角色exit0、无提前退出/升级信号/进程组残留，worker/map线程清理门禁均true。
原r1..r7、F03恢复与历史失败原样保留。

| 场景 | 结果 | 正式规划 / 非空local Path | 首次连续.3s终端hold(s) | 实际最终(x,y,yaw) | compute ms 中位 / P95 / max |
| --- | --- | --- | --- | --- | --- |
| forward_spin | PASS | 12 / 780 | 16.719487 | (1.910900311, 0.384290965, 0.572432970) | 0.558987 / 35.867488 / 100.288419 |
| reverse | PASS | 1 / 340 | 8.598105 | (-0.702315255, 0.000000000, -0.000000000) | 0.563198 / 34.893200 / 77.194830 |
| obstacle | FAIL | 16 / 886 | 未到达 | (1.096187999, 1.338832056, 0.040051461) | 1.023985 / 42.665533 / 106.431382 |
| mapchange | PASS | 2 / 609 | 24.076510 | (2.504812339, 0.000000000, 0.000000000) | 6.212226 / 42.068312 / 108.937647 |
| empty_smoke | PASS | 0 / 133 | 10.468737 | (0.704817684, 0.000000000, 0.000000000) | 5.575165 / 28.722916 / 71.577318 |

compute统计来自各case实际solve_result diagnostics（n依次875/378/932/655/218），独立于到达/协议/安全门禁，不承诺50ms实时期限。
forward原goal在16.719487s满足hold，38s恢复目标后42s停止planner，最终停在恢复运动途中，不能拿最终位姿计算原goal误差作为到达否决。
真实worker延迟250.296542ms期间继续运动/滚动参考，停发陈旧Path停车门禁PASS。
reverse负向速度门禁PASS；mapchange单goal、7..13s临时墙及17s .05→.03源分辨率均按原fixture执行且通过。
empty_smoke为fixture注入Path/真实follower，正式planner计数0；summary formal=true是此smoke明确豁免，不能声称正式规划成功。
前四场正式成功仍需0/PLAN_FOUND/has_reference=true及真实请求/接受身份匹配。

所有case实际完整矩形SAT/角向连续扫掠、速度/加速度/行进曲率、结构化QoS、生产禁入独立plant、
零stamp TF和私有命令/ROS图均PASS。DDS发现history/depth仍UNKNOWN/null，实体实际profile补证KEEP_LAST1/RELIABLE/VOLATILE，
tf_static为TRANSIENT_LOCAL；无T5 publisher。
各场最大|v|依次.199973702/.195432584/.198882692/.196092858/.194749526，
最大|w|依次.531527150/近0/.907384938/0/0；最大a/alpha均不超.5。
实际core.so仍为ROOT/install/core-humble-r1/lunar_incremental_navigation_core/lib/liblunar_incremental_navigation_core.so，
SHA256 `0092ca1863b642f9bc0cbc7bb443b22446c26d1b29f153a3e217ce376e9bb4ec`。
三个native依次terrain `ff300175aff7d80aba6d5b995ef21e39612a85fbeabde92ec5e5fd5f7033c3ef`、
geometry `f8a2366727dad8775a5f2528d93cf5ad5ea18886e782f84f5e18f27cbfeefaa2`、
distance_field `e79c02bc87ce67c70d1f788650fed7794d5efe68442897044aae38c6cbf45c4e`。

### obstacle失败时间线与owner

本轮故障是TRACKING_STALLED，MPPI_STOPPING_UNSAFE实际UPDATE计数0；不能混称旧r7停车证明失败。
实际位移从旧r7约.00234m推进至1.730347m，但未越过x1.5，最终距goal1.505055498m。
20..24.5s实际沿轨迹运动.758790984m、vmax.198882692，89非空/0空Path、33次LOCAL_ACCEPTED。
t24.508362147957087首次从TRACKING变BRAKING：实测(1.0566821624098728,1.3372489491163821,yaw.040044254924018426)，
v=.1957271252615095/w=.0008326115191740618，before.failure=None，dt=.049960818，
now_s1793184823.4288917/state_age_s=.004807973；map origin[-1.4530346938214613,-1.1631401821319909]/res.01/revision409/epoch1。
此时diagnostic distance_m=.7440259081226951、valid_samples/weighted_updates均0，compute_ms=.997006893；
下一次before.failure明确TRACKING_STALLED。之后受限制动，最后非零运动t24.895861299941316。

t24.90801115287468首次返回FAILED/TRACKING_STALLED，实测已停在
(1.0961879990869865,1.3388320555347317,.040051461443987364)，v/w0；
before.phase=BRAKING/failure=TRACKING_STALLED，dt=.050067737，now_s1793184823.8332012/state_age_s=.000544554，
map origin[-1.403985671926278,-1.161174903964848]/res.01/revision422/epoch1。
真实末12条history先有(.19331205,.00043358)、(.19335950,.00042089)、(.19387240,.00041786)、(.19572713,.00083261)，
随后8条(0,0)；完整精确时钟/值见原follower.jsonl及派生audit first_failed，未清空history。
此后463条UPDATE返回FAILED/TRACKING_STALLED，最后before.map revision424，origin[-1.4038120009130135,-1.1611679444652683]，
now_s1793184846.921651/state_age_s=.01212123，最近12条真实history全0。

| 连续时段(s) | 实际路程(m) | 最大速度绝对值 | 非空 / 空Path | LOCAL_ACCEPTED | UPDATE phase |
| --- | --- | --- | --- | --- | --- |
| 25..38 | 0 | 0 | 259 / 0 | 99 | FAILED 260条 |
| 38..48 | 0 | 0 | 193 / 0 | 77 | FAILED 200条 |

因此是持续非空参考/已接受局部候选下的锁停，不能解释为48s截止时仍合法前进但尚未到达。
首次制动前最近收到local Path为t24.474408449s附近（monotonic1291150.382302832），94 poses，
首(1.046965306,1.336859818)、末(1.800090551,1.367556034)，xy整体方向.040735900rad；
这是最近收到的Path，不能等同内部activepath/anchor。
最后收到local Path仍从实际停点伸向(1.831193115,1.368722240)，96点；
最后正式GLOBAL_RESULT仍0/PLAN_FOUND/true、requested_goal[2.6,1.4]/goal_revision1/accepted_generation1，
requested_epoch1/version45，最后局部sequence350/route_version23持续被接受。

owner交原follower停滞/滚动进展状态维护，由root安排最小复现。
源码线索为mppi.py:358–372固定anchor/travel点积超时，以及StandardPathFollower切段/接收参考时的anchor更新；
此为定位假设。原raw没有no_progress_s、_last_pose或内部activepath方向，不能宣称已证实其根因，
也没有新的停车cert拒绝原因可归因。本Task不跨owner修、不加时、不调fixture或阈值、不重新执行。

派生审计保存在ART/task3-joint-r8-derived-audit.json，原始逐文件哈希为ART/task3-joint-r8-raw-sha256.json；
分析脚本ART/task3-r8-audit.py，不覆盖summary/raw。由于非全PASS，本轮不交付到达图替代验收。
未变源码无需重复core/QoS单测；历史609pass/1skip属于F04既有回执，本轮新增证据只有上述真实五场与文档diff检查。
plant nominal_response_time_s=0理想有界执行模型边界保留；Jazzy、Orin、DDS跨机、真实P3流/rosbag/RViz及实车均NOT_RUN。
处置为已提交待owner修复/独立审查及合入；默认/应用入口未改，未合并/推送/PR/删除，资源保留待回收。
提交后本Task交回源码写入权给root，Task3仍NEEDS_CONTEXT。


## T3F05：已停稳参考切换的物理锚点

基线e193273，产品限定修改StandardPathFollower._accept_pending：不兼容参考在实测已停稳时不经过switching制动，因此也必须像该制动完成分支一样调用既有_orient_progress_anchor。旧旋转阶段的平移方向为零；若沿用它，真实平移点积仍为零。仅重绑定新有序阶段，不刷新停滞时间，不清故障，不改共享MPPI/地图/参数/默认入口。

真实Humble TDD：t3f05-red-r1准备姿态已在容差内，6通过，未复现；修正为先经历未完成旋转后，t3f05-red-r2在直线及曲线真实反馈前进约.06m时0.4s错误TRACKING_STALLED，2失败/4通过。修复后t3f05-green-r1标准Path41通过；加入静止不兼容反复切换时钟/历史保持后，t3f05-controller-r1完整controller436通过。保留静止换Path、侧移、反向、往返、旧故障锁存、有序自交、密集yaw及最新pending测试。

r8受限重建以plant记录的Path接收时间与follower实际UPDATE状态合序，测试free地图/快速MPPI/age0替代原运行环境，不能恢复原callback先后、内部activepath或精确15s阈值。实际首制动状态x1.056682162/y1.337248949/v.195727125处，旧源码重建travel=[0,0]且clock14.165990324s，新源码travel=[.999176782,.040567956]且clock.049960818s。此为机制佐证，不冒称完整replay或已通过真实障碍案例。

三包Release重建/CTest、50个Python安装身份及native SHA以ART/task3-product-fix5-report.md与t3f05-build-r1回执为准。独立审查和实际受影响obstacle联合运行交root后续执行，Task3仍NEEDS_CONTEXT；Jazzy、Orin/实车、真实P3流/rosbag与DDS跨机均NOT_RUN。
