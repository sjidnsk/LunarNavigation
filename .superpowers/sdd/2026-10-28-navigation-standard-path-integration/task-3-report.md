# Task3 真实 installed planner—follower 联合验证报告

状态：NEEDS_CONTEXT，已提交待产品修复/待独立审查/待合入，绝非 DONE。
任务：navigation-standard-path-integration/Task3。
归属：integration/pure-planner-orin；专题 hk/nav-1cm-feasibility-20261026。
执行者：nav_actual_joint_impl，Windows唯一源码写入者，隔离台式Humble执行；未派生agent。
工作树：D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree。
产品基线：46f635e339fa70fdf116454bc1d287733efa395b。
本报告、harness、测试和正式验证文档显式提交；progress为原任务共享本地登记。
源码写入权在本次提交后交回root；当前运行资源已回收，工作树保留用于产品修复后的Task3恢复。

## 实现范围

新增 scripts/navigation/one_cm/path_follower_validation/ 下8个模块：
evidence唯一名称导入既有Recorder/formal_acceptance/apply_supervision_gate、解析SAT；
planner_probe包装真实已安装create_resources和LocalWorker；follower只读包装真实已安装
PureWheeledControllerNode/StandardPathFollower.update，返回值、速度、命令历史不变；
plant只接收实际Twist并积分反馈P3 Odometry/TF；grade/evaluate独立计算实际车体判据；
run监督精确本任务进程组、安装身份、异常、退出与worker清理。未实例化旧PathFollower控制律。

跨进程链为真实installed planner→nav_msgs/Path→真实installed follower→Twist→独立plant→反馈。
所有进程固定ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1；命令只发/nav1cm_integration/cmd_vel。
每轮记录真实ROS图publishers/subscribers/types/QoS，检查无/Car/T5/Car_Cmd_Vel发布者。
消息地图为真实grid_map_msgs/GridMap；direct_p3和实际native map；标准零stamp map←odom TF。
UNKNOWN策略默认false；地图8m、源分辨率.05及热变.03，未跑1km或触碰DRL/实车/足式默认。

plant为有界理想执行模型：只订阅Twist，不读取Path作控制，完整墙钟间隔≤.01s小步，
线/角加速度独立限制.5，中点积分并保存真实末速。nominal_response_time_s=0；
未删除生产其余响应/迟延认证约束。完整1.182×.818m矩形SAT独立扫掠，
按线/角运动上界给中间采样加膨胀，碰撞接触即失败。既有motion模块仅复用State/integrate/SAT，
没有调用旧consumer运动控制律或生产certify。该模型不能代表实际底盘执行验收。

门禁要求真实位移>.3m、至少30非空局部Path、正确DDS类型/来源、正式全局0/PLAN_FOUND/true
以及goal/revision/generation/route_accept身份一致；到达xy≤.1/yaw≤.05、实际v/w≤.01连续≥.3s。
速度≤.2/角速度≤1、加速度与alpha≤.5、移动曲率≤1。监督异常、提前退出（包括0）、残留、
升级信号、缺记录或碰撞独立否决；WAITING和局部终点不能代替全局到达。
empty_smoke明确是fixture注入Path，不伪称planner产生empty，也不把formal bypass=true当正式规划证据。

## RED→GREEN与验证边界

- 实现前Windows RED：验收器无条件通过stub，对15种破坏证据产生15 failed/1 passed；
  覆盖无真实运动、错误图类型、无正式来源、假安装、worker/进程异常、无路径、碰撞、
  速度/角速度/加速度/alpha超限及监督失败。工具输出保留；不是仅import错误。
- GREEN：19 passed，包括旋转车体碰角、连续停稳判据、实际图错误类型与T5发布者反例。
- 最终真实Humble/Python3.10快照 task3-grader-humble-r1：19 passed/.03s、exit0。
  archive SHA256=505c64f7b0be06d651c216cd5cd7c6b5592f222b99f9010c8b2394c402d29891。
- Windows显式UTF8读取、8模块compile(source)和git diff --check通过。
  先前compileall因沙盒不允许外部工作树写__pycache失败，随后改用不写缓存的compile；
  一次pytest命令未指定cwd而找不到测试路径，随后在明确WORK重跑19项通过，未计作产品失败。
- 本Task未改产品源码，不为绘图重复完整core545/controller411套件或重复Release构建。
  真实安装复用Task2已验证的Humble Release产物，其436包检查属于Task2，不冒称本Task重跑。
- Jazzy、Orin、实际车辆、真实rosbag、RViz和T5控制：NOT_RUN。

## 安装身份与不可变原始证据

台式ROOT=/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027。
INTEGRATION=ROOT/integration-20261028。
安装=INTEGRATION/task2-fix1-build-r1/install；Git源46f635e。
本地ART=D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/integration-20261028。
每轮fresh label，归档、manifest、joint-receipt、runner日志及完整-result原始目录均保留。
运行前逐50安装Python SHA与回执核对；helper把源码Git对象与安装回执核对，
移除SDK遗留/source-map-v1搜索路径，未用source nav1cm替代生产安装。
planner记录3个libnav1cm_*安装SHA；follower实际maps加载terrain+distance_field两个库，
两个均与安装回执吻合，planner三个也逐一吻合。每进程identity含pid/start_ticks/domain；
follower记录实际executor类、模块路径、全部参数、policy与安装Python SHA。

最后按root要求增加/proc/self/maps中liblunar_incremental_navigation_core.so路径与SHA观测，
以及supervisor入口三native回执核对。这仅增加观测/启动身份检查，不改决策；
该新增core加载观测尚未经历新真实运动运行，待修复后fresh label验证，不能追认旧轮次已记录core。
旧轮次安装receipt中的libnav1cm身份和加载路径仍保留，不替代实际core.so来源证据。

| 标签 | 结果 | archive SHA256 |
|---|---|---|
| task3-joint-r1 | 启动/清理失败，未形成运动验收 | 54e93de2f139571edd2005ab8f921d819d32e968726114380cfcb9a6a1ca757f |
| task3-joint-r2 | forward_spin FAIL，真实运动但terminal/bounds失败 | f1f6c470b6ab835049fdf6f52057774d14bb26a86331e2dcaa0357c4b418bd7c |
| task3-joint-r3 | reverse PASS；empty、obstacle、mapchange FAIL并保留 | 8f91a16ecd7f7ff215847283834f6c5d658b3c4f323e832906be39db4109d90a |
| task3-joint-r4 | 仅fixture修复后重跑empty/mapchange，二者PASS | 42bef6f20491cb2bce070aa446754b7f7675fbf23abdbc764635633527b0b84e |

每项详见ART/标签-result/场景/summary.json、plant/planner/follower.jsonl和console.log。
所有summary保持各次实际结果，不用新PASS覆盖旧FAIL。

## 现场结果、失败原因与owner

r1：planner因SDK残留/source-map-v1命中禁止源码搜索路径断言提前exit1；
监督器回收时plant默认ROS SIGINT已shutdown又重复shutdown报错；
follower尚在构造时收到监督SIGINT而exit-2。
仅19条state、0位移、0路径，不称为产品运动失败。root修外部环境过滤；
本任务修plant SignalHandlerOptions.NO及if rclpy.ok的shutdown。
r2以后所有进程exit0、无信号升级、无剩余pgid/worker，清理修复有实际结果。

r2 forward_spin，32s：
- 正式规划13次、5610 local/509非空、16902实际state；实际最大位移1.746604m。
- vmax=.19536994、wmax=.13589734、a/alpha最大.5，完整车体无碰撞。
- DDS图、formal、安装身份、worker、真实运动、rolling、标准Path合同、独立plant、
  zeroTF、delayMotion、plannerStopMoving、staleStop均PASS；terminal/bounds为FAIL。
- 真实LocalWorker sequence15延迟1275227.580695606→1275227.831280703，250.585ms；
  期间非空路径继续且真实车体位移>.001m。follower compute中位12.397ms、
  P95=70.408ms、max124.670ms（603条），未因>50ms单独制动。
- 原目标(1.4,0,.6)。t25恢复目标时实际(1.30397,-.00570,.45371)，尚未完成原目标yaw，
  不把最终resume位置(1.72822,.25278,.58946)当原目标误差。
  场景后续改为45s、38sresume、42s停planner，容差不变；这个新时序当前NOT_RUN。
  延长时序不能掩盖下述滚动spin产品风险。
- 连续运动曲率19行超限：首次t26.34556169，v=.00460418、w=-.00919650；
  违例max|v|=.02253399、max|w|=.02422876，超出.01停稳阈值，不能按“停稳”豁免。
  原始实际命令1275248.841976852为(.0033843377,0)，随后1275248.89025823为
  (-.0148655328,-.0148655328)，独立.5slew中途穿过v0而w未归0。
  root已裁定owner=follower连续运动认证/换向屏障；不得放松plant或曲率门禁。

r3 reverse，20s：PASS。正式规划1、336非空、12016state，最终
(-.70334012,约0,约0)、v/w0，8.72276s完成连续.3s到达hold；
vmax=.19567834，确有负速度、无碰撞/曲率/加速度违例，全部进程正常清理。

r3 obstacle，48s：FAIL，不能宣称具备障碍跟踪能力。
- 正式全局1、808非空、346 LOCAL_ACCEPTED、808 BUFFER_EXECUTING；
  实际位移0、最终yaw=.07948377、v/w0，最大w=.05。
- 首条多点局部Path98poses，全部xy0，yaw0→1.51388；
  中段96poses，全部xy0，yaw.03317→1.50174；
  末段95poses，全部xy0，yaw.07948→1.50135。真实body48秒未完成初始转向，更未越过障碍。
- follower诊断TRACKING826、FINAL_ALIGN30、BRAKING37；
  BRAKING原因REFERENCE_DISCONTINUITY。路径不是未发布，planner局部结果持续被接受。
- owner=follower滚动原地旋转消费/有序阶段进度。
  源码候选原因：StandardPathReference把每个重复xy且yaw变化的采样独立生成spin stage，
  StandardPathFollower._accept_pending每次rolling设stage_index=0；需产品fixer用RED最小复现确认，
  此报告将其与raw关联，不伪称已完成产品根因修复。

r3 empty_smoke：运动/emptyStop/恢复/terminal与所有安全检查均PASS，
仅fixture注入两pose的stamp相同不满足记录器严格递增Path合同，原FAIL保留。
本任务修为独立header、第一pose等于Pathstamp、第二pose+50ms；
没有改产品Path准入或放宽合同。r4 empty_smoke PASS：
133非空、2976state，空路径前t5.99 v=.147741、x=.511833；
t6.5实际停车于x=.546195，保持至8s恢复；t8.6 v=.107499，
10.51918s达到终端hold，最终x=.70512443/v=w0。formal_count=0，明确注入链。

r3 mapchange：真实碰撞FAIL保留，不能把最终到达反推安全通过。
第一次碰撞t7.1310029、x=.7093149、v=.194029；车头约1.300315，
当时外生墙在x1.3，墙刚于7s出现，首次地图发布monotonic1275619.107650559约t7.029；
剩余车体间隙/停止距离不足。1333行碰撞持续到墙移除前t12.995。
fixture突然在刹车距离以内生出障碍，属于本任务场景设计缺陷，不能归咎未观测产品。
修为墙x1.8..2.1、目标2.6（原目标1.8会在墙内），保持相同7..13s阻塞与所有SAT门禁。
r4 mapchange PASS：正式规划2、598非空、16374state，单次目标不重发，
.05→.03热分辨率已由planner MAP_APPLIED确认，阻塞期间9..12s实际
x=.79966771恒定/v0；移除后恢复，23.96159s达到hold，最终x2.50401365/v=w0。
vmax=.19609340，零碰撞/限值违例、进程全清理。

## 图与原始指标

ART/task3-joint-r2-actual-motion.png：真实xy、yaw时序及26.3s换向曲率超限局部。
ART/task3-joint-r3-obstacle-stall.png：实际yaw与收到的local endpoint yaw并列、实际位移0。
两图均由原始plant.jsonl派生；两图均已亲视。
图是证据补充，绝不以收到的参考端点当作车体到达或碰撞安全证明。
地图npz未单独导出，本轮解析障碍和源分辨率完整保留在MAP行；地图fixture源码由manifest绑定。

## 恢复与剩余条件

1. root安排follower产品owner复现并修复连续换向曲率与滚动spin进度，产品提交单独审查和适用回归。
2. 重新构建真实Humble Release，更新明确新安装prefix/回执，保留46f635e旧结果身份。
3. 本任务复用原Task3/专题分支恢复，fresh label重跑受影响forward_spin/obstacle，
   核对新增实际core.so加载路径/SHA；产品修改影响范围决定是否重复其他case。
4. 全部跨进程case和独立监督门禁PASS后，root再安排独立规格/质量审查；本agent未派reviewer。
5. Task4默认入口仍不切换。无推送/合并/PR/upstream修改/分支或worktree删除；
   当前处置为已提交待修复/待合入，工作树待后续收尾，不能报告全部完成。
