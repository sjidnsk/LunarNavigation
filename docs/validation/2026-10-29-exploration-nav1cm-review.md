# 前沿与DRL接入：最终审查记录

最终结论：两项Important均已修复并经同一审查员限定复审关闭，0新增C/I/M；可进入owner源码集成审查，未授权合并，不代表生产部署就绪。以下保留原审查和后续复审原文。

## 第一阶段：总审查原文（提出两项修复）

# NAV-1CM-AUTO-20261029 最终跨分支审查

审查员 auto_nav_final_review，2026-10-29。独立单次全任务审查，使用 superpowers:requesting-code-review / code-reviewer.md；未派子审查员。

**0 Critical / 2 Important / 0 新增 Minor。Ready for OWNER source integration review? With fixes。** 两项 Important 关闭前，不建议以“产品阻挡清零”提交 owner 验收。不授予合并权限，不代表生产或实车部署就绪。

## 范围与方法

- MAIN：185ece29a83df770bb0f3011c08e54b421a52047..eae70febe3b58ba03cc6a72de9df4387b9ae5bea；根 D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree。
- DRL：a094b90206db9c9bef2b6cf075db55cf210ccc22..b16c8ca8a8914dfc4a06c280d301e310eee61329；根 D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation。
- 两个完整 review packet（MAIN 385668 bytes、DRL 130496 bytes）已分段读至末尾，并补读工具截断处。阅读批准设计/计划、brief、正式 closeout、progress deferred/parked、Task1–4/C1 完整报告、各初审及 fix1 scoped review。任务级批准仅作为上下文。
- 周边核对 Runtime→session→task coordinator→Action、map-only/旧core/export、公共launch→前沿等待取消、DRL factory→owned RosEnv/attach→canonical准备→配置记录/恢复语义，以及相关测试与 runner 身份判据。
- 本轮仅静态审查，没有测试、构建、ROS、Actor、网络或训练动作；没有产品/证据/Git/index/HEAD修改。ROOT 已独立完成保护审计，本轮不重复。唯一输出为本报告。

## Strengths

1. 职责基本清楚：策略选点，共同任务管理UUID/代际，planner输出两个标准Path，MPPI执行。没有用旧ACK、Path存在或Action传输接受冒充实际到达；有效替换和无效目标保留语义明确。
2. Runtime单所有者、不可变提案、输入竞争线性化与锁外事件交付方向正确。原Task1 goal/cancel竞争已修；I1是当前终态边界尚未覆盖的另一类输入。
3. map-only复用core，observed-only与legacy出口分开；保守源格投影/真实UNKNOWN/静态TF有针对性覆盖。C1只修数值边界，保留真实缺口反例，没有扩大FREE来取得运动。
4. 默认none/controller=false和legacy默认配置/恢复identity保持；显式nav1cm有独立identity。canonical v5准备复用、infer优先级与map_frame修复合理。
5. 精确UUID/generation通知、终态最新suffix、旧归因降级、失败及成本记录有价值；DRL45秒未到达明确披露，没有将取消停稳写成到达。

## Critical

无。

## Important

### I1 — 终态提交未否决计算期间已接收的新里程计，可提交过期到达或停稳

主定位：[runtime.py:190](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/runtime.py:190)，提交条件193–194行。

可达链：同文件56–60行offer_odometry在回调线程更新消息和_odometry_sequence；128–138行tick截取一次odom/TF快照，随后锁外处理目标/事件并执行session.tick。187–189行observe仍用旧快照和sequence。提交锁仅检查_goal_sequence，没有检查odom变化。[navigation_task.py:88](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree/scripts/navigation/one_cm/nav1cm/navigation_task.py:88)据此形成终态；119–126行提交后，下一轮84行因outcome已有值直接返回，不能撤回。

具体触发：当前目标已合格保持约0.3秒，tick取到停止且xy/yaw合格的新样本；在session.tick或之后提交前，ROS又接收到移动/越界样本。goal sequence不变，旧提案仍提交GOAL_REACHED。CANCELING同理能从旧停稳样本提交CANCELED_STOPPED，而Runtime已收到非停稳样本。不需要旧stamp、特殊硬件或丢包，正常回调/tick并发即可达到。结果让前沿/DRL进入后续决策，违反当前实测完成/停稳合同。本轮未运行反例，结论来自具体锁与调用顺序；既有固定窗不能排除该交错。

建议：将终态提案绑定测量版本，并在同一提交临界区确认版本；若有更新，由唯一owner以最新输入重新观察或推迟终态。涉及位姿转换的TF版本也应考虑一致性。无需新增消息时间戳/协方差门槛，不能锁住native计算；应避免高频odom令普通反馈与终态永久饥饿。最小确定性回归在snapshot和commit之间注入“合格→移动/越界”，覆盖GOAL_REACHED及CANCELED_STOPPED被否决、保持重置及后续真实合格可完成。保留原goal/cancel竞争、锁外回调合同。

相关测试：[test_navigation_task_runtime.py](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/test/test_navigation_task_runtime.py)。

### I2 — owned RosEnv 新后端忽略已记录的到达容差，配置/resume identity与实际执行不一致

主定位：[ros_env.py:408](D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation/ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/ros_env.py:408)和[navigation_backend.py:24](D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation/ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/navigation_backend.py:24)。

参数链：TrainingConfig在config.py:270–271公开默认.05m/pi/12rad，并接受合法自定义值；config_record保留字段，resume_semantics在472、494–495行记录对应语义。_start_ros的nav1cm分支405–411行仅传platform/build/use_sim_time，随后return，旧分支412–415行的容差透传不执行。owned_nav1cm_specs的24–38行也无相应参数。因此实际map-only默认.1/.05（MAIN exploration_map_node.cpp:84–87）、planner默认.1/.05（MAIN planner/node.py:94）、标准Path controller默认.1/.05（MAIN controller/node.py:221–223）。DecisionCore.observe还用snapshot实际.1覆盖history tolerance，运行不会遵守记录中的.05。

影响：显式选nav1cm并提供合格.2m clearance profile，其余到达配置不变的owned环境，记录.05m却实际使用.1m。例如停止在0.08m且yaw合格即可依共同任务合同成功，环境记录却承诺更严的位置阈值；自定义yaw同样被静默替换。影响后续transition、实验复现和resume identity。本次真实窗口没有到达，本finding不声称发生过实测错误到达，而是新增入口静态可证的参数失配。

范围区分：公共nav1cm_exploration.launch.py没有承诺TrainingConfig容差，而且三方默认一致；不要求修改公共默认，也不将attach/console的NOT_RUN当缺陷。

建议：明确owned模式合同，解析后把同一position/yaw值传给map、planner（goal_tolerance_m）和controller（goal_position_tolerance_m）；若新模式必须固定.1/.05，则启动前明确拒绝不一致配置，并令记录/resume identity反映实际值。保持legacy默认序列化不变。用纯config/specs测试覆盖默认显式新模式和非默认容差的三方参数或拒绝行为，不必启动完整RosEnv/训练或重跑Actor窗口。

相关测试：[test_navigation_backend_selection.py](D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation/ros2_ws/src/lunar_drl_exploration/test/test_navigation_backend_selection.py)。

## Minor

无新增。原M1/M2等deferred保持原判定，不借本次重开共享环境治理。

## Declined to judge

以下逐项保留，不等同通过：

1. M1旧pytest pythonpath warning、M2共享colcon重名mixin：已披露，无--mixin选择造成错误产物的证据；共享环境修复不在范围，不升级产品阻挡。
2. 旧legacy OutsideToInside首次5秒超时：后来scoped/完整16项通过，根因仍未知；不凭后续绿解释首次失败，也不重审未变legacy全部历史。
3. Windows两项POSIX路径假设失败：Linux资格另列，旧平台测试问题已登记，没有本次新增回归证据。
4. 未知捷径/已知绕行优先：NOT_TRIGGERED，没有有效实测就不评判交互，也不擅自追加场景。
5. 长期滚动地图资源上界、100m全源1cm行驶：当前短窗和250000格接收不足证明；需独立任务，也不据此臆断泄漏或实时性故障。
6. window03每子进程live /proc/maps与少量历史argv缺失：降低身份/复现证明强度；预检加载+安装+argv/graph/raw不伪装完整现场映射，不补写/重跑。
7. 最早QoS RED默认T5零Twist发布尝试：domain92/localhost1且无plant，但订阅者/送达UNKNOWN；当前私有graph不能证明历史绝无送达，也不能无证据推断车辆动作。
8. Task4误选测试：111/227组确有临时ReplayBuffer、两次learner.update、ControlledEnv run_training一条假transition、客户端domain205。是违反intended no-training-construction边界的历史事件，不能洗为纯测试。47MB原件/成本与后续guard尝试必须保留。“无正式job启动/恢复/切换”与该违规分开判断；不要求未授权训练补验，不以历史事件自动否定当前产品源码。
9. Frozen/ledger：ROOT权重/config/正式manifest保持审计可引用为ROOT证据；ledger未打开，拒绝给出其字节前后不变结论。
10. 策略质量、新旧短窗性能比较、Orin/实车部署：地图构造及窗口时长不同、新后端未到达，无依据给出胜负或部署结论。

## CannotVerify / 验证边界

本轮reviewer未执行验证。以下仅为读取的既有证据，不能说成本轮测试通过：

- Task1 native Action及纯合同；Task2 core20/20、ROS15/15及250000-cell RECEIVE-only，大帧没有运动资格。
- Task3最终75/60/30/60秒合成物理窗：20精确UUID、9到达、11cancel-stop、90正式0/PLAN_FOUND/has_reference=true，含早发布零stamp静态TF与正向障碍更新。旧无权威绑定raw已降级，不能借新窗反填。
- C1原709源格恢复2660伪UNKNOWN、保留27200真UNKNOWN；初始支持与连续运动资格分开。
- Task4既有v5 Actor新后端45秒：实际1.629m、44正式规划、当前UUID取消停稳、4child exit0；NO ARRIVAL，xy误差.1758m且yaw未满足。旧默认8秒约.987m、UUID关联和取消停稳、3child exit0，只作兼容smoke。
- 最终frame/CLI修复12项安装级Humble targeted pure PASS，没有重复未变物理窗。processed_stamp181含初始0，实际非零180。
- 仍NOT_RUN：console main/public task topic实际ROS生命周期、完整新RosEnv episode、真实P3、生产/跨机DDS、Jazzy、Humble容器、Orin、实车、真实rosbag/RViz、长期策略效率/覆盖/到达效果及正式训练。两项finding不依赖把NOT_RUN当执行或失败。

## Plan alignment

共用任务/地图→前沿→DRL顺序、两个owner专题、可选后端、标准Path/MPPI职责、observed-only地图、默认兼容总体符合批准设计。Task3静态TF/最新suffix/权威UUID、C1数值接缝、Task4frame/CLI修复均有明确限定原因；没有整支MAIN合入DRL。现closeout保留Ruling、原失败与NOT_RUN。

未满足的源码合同是I1最新测量终态提交一致性和I2 owned环境配置/实际执行一致性。历史误选测试违反执行边界已发生，报告保留且不可撤销；后续纯白名单与无正式job行为不能改写成“从未训练构造”。宜用一次协调修复波关闭两项，只做受影响最小验证与scoped复审，不重复已完成窗口或全部历史。

## Ready for OWNER source integration review?

**With fixes。** 先关闭2项Important，再进入owner源集成验收。无新的Critical/Minor仅限本次源码与证据范围。没有merge/push/PR/upstream/删除授权；专题、安装、失败证据保留待owner处置。source integration结论不能写成实车部署就绪。

本报告写入并读回后，唯一final-review.md写槽归还ROOT；reviewer不再修改产品、证据或Git。


## 第二阶段：单次协调修复后的限定复审原文

**I1 最新 odom/TF 终态竞争 — ADDRESSED。** [runtime.py:192](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/runtime.py:192)在原目标输入门禁通过后，比较本轮 odometry 与 TF 版本；195–209 行在同一输入锁内用当前消息和 TF 重做轻量状态转换、重新 observe，再在210行提交。旧提案不再越过已经接收的新运动/越界测量。重观察提交非终态会清除旧 hold；转换失败设置 state=None，错误 notice 在锁外214行交付。持续等价输入会重观察并提交，而非一律拒绝，避免反馈/终态饥饿。session.tick 仍在187行锁外，临界区没有 native/solver、外部 event sink 或 notice；原 goal/cancel sequence 门禁和锁外事件交付保持。

**I1 回归覆盖 — 已核对。** [test_navigation_task_runtime.py:242](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/test/test_navigation_task_runtime.py:242)的 moving/outside/tf 三项在提案形成后注入更新，检查旧到达被否决、hold 重启和后续真正到达；277行取消测试检查移动否决后仍能实测停稳完成；293行检查每轮等价 odom/TF 更新仍产生反馈并最终到达。既有取消入队、真实线程交错、替换及回调重入测试保留。保存的 main-red.txt 是4 failed/11 passed，三次错误 GOAL_REACHED 和一次错误 FINISHED 均为本 finding 的功能反例；main-green.txt 为15 passed。Humble记录 CTest2/2，共 runtime6 + task_runtime15，ROOT XML审计0fail/0error；没有用窗口运动替代竞争测试。

**I2 owned RosEnv 配置容差失配 — ADDRESSED。** [ros_env.py:408](D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation/ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/ros_env.py:408)现在把配置中的 position/yaw 传入真实 helper；[navigation_backend.py:24](D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation/ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/navigation_backend.py:24)新增兼容默认参数，并将 position 映射到 map/follower 的 goal_position_tolerance_m、planner 的 goal_tolerance_m，三方共享 goal_yaw_tolerance_rad。公共无参 helper 仍为.1m/.05rad；owned 使用配置默认时为.05m/pi/12rad，自定义合法值同样透传。公共launch、legacy启动字典、config_record/resume源码没有改动；开发manifest仅改两个实际产品文件的SHA。

**I2 回归覆盖 — 已核对。** [test_owned_nav1cm_tolerances.py:19](D:/Codexworktrees/drl-nav1cm-adapter-20261029/Lunar-Navigation/ros2_ws/src/lunar_drl_exploration/test/test_owned_nav1cm_tolerances.py:19)覆盖helper兼容默认、.05/pi12和.03/.07三方参数；32行起提取并执行真实_start_ros的完整owned AST分支，调用真实helper，用fake process recorder观察参数，能发现调用者漏传。它不是完整RosEnv/ROS生命周期测试，报告没有越界升级。保存的drl-red.txt为4 failed/1 passed，两个不支持kwargs和两个实际仍为.1的caller反例；drl-green.txt为5 passed。Humble精确8项包含新增5项及3项原default/resume/specs检查，输出8 passed/0.17s，ROOT XML审计0fail/0error。

## New Breakage in the Fix Diff

**None — 0 Critical / 0 Important / 0 Minor。** 在限定修复差异中未发现新增破坏。I1没有把求解搬入锁内、改变任务身份/取消顺序或引入stamp门槛；I2参数名与现有三节点接口对应，没有修改公共默认、策略网络/奖励或legacy恢复语义。新增文档明确待复审和旧物理窗不回填；两项开发SHA更新与产品范围一致。

## Checks / 审查范围

**完整阅读 — 已完成。** 原final-review.md的I1/I2、final-fix-review-brief.md、完整final-fix-report.md和以下两个完整log/stat/-U10包均已读完：MAIN eae70febe3b58ba03cc6a72de9df4387b9ae5bea..6e21fdf65c65685537ccf09a1c9bfabf2a320cec（25864 bytes）；DRL b16c8ca8a8914dfc4a06c280d301e310eee61329..6678837ff25c8947725ac1596fd7ae1e3b80c55f（24178 bytes）。按同一原reviewer的ONE scoped复审执行，不是新一次全分支审查。

**证据对照 — 已完成。** 除报告外，只读本地main/drl RED/GREEN原输出、实际run_humble.sh及归档内main-test.txt/drl-test.txt；确认覆盖命令、测试名称、功能RED和GREEN相符。周边只核对本次临界区调用的state_from_odometry/LatestTransforms及提交语义。ROOT提供的125安装、87manifest、5native/coreb630、21+8XML、legacy AST/config exactBASE、frozen/旧安装/旧motionarchive核对作为ROOT独立审计引用，本reviewer没有重新执行远端审计或把ROOT操作称为自己的执行。

**验证边界 — 保持。** 本次reviewer没有运行测试、构建、ROS、Actor、训练、网络或子代理，没有改产品、Git/index/HEAD或raw。实现者本波资格限纯定向测试、Humble x86_64构建/安装身份；旧真实运动窗仍绑定其旧产品提交，不能写成a1e4c469/072f75d6或文档HEAD的新运行。首次identity旧安装路径错误、CRLF diffcheck修复及mixin警告保留。

## Out-of-Scope Observations

**新增观察：None。** 不重开未修改的全分支或历史审查。

**继承边界：非本修复阻断，原样保留。** 原报告10条Declined及ROOT逐条Ruling/误判代价不因本次通过消失；尤其Task4误选ReplayBuffer/两次learner.update/ControlledEnv run_training及47MB成本、后续guard尝试、domain205、默认T5零Twist尝试未知送达、legacy首次超时、M1/M2、Windows路径失败和未知捷径NOT_TRIGGERED，均不能洗为新执行资格或全绿。ledger未打开，不作字节不变证明。

**NOT_RUN：保持。** 原45秒NO ARRIVAL；console main/public task topic真实生命周期、完整新RosEnv episode、真实P3/生产DDS、Jazzy/Humble容器、Orin/实车/rosbag及长期策略效果/正式训练仍未由本次验证。不存在正式训练切换或生产部署验收声明。

## Verdict

**Fix round: All findings addressed, no new Critical/Important breakage。** I1、I2全部关闭；开放finding为0，新C/I/M为0/0/0。

**Ready for OWNER source integration review? Yes，限源码集成审查。** 无merge/push/PR/upstream/删除授权，不能称为生产或实车部署就绪；资源与历史证据继续按ROOT/owner现有安排保留。

**写槽归还 — 完成本报告写入及UTF-8读回后，唯一final-fix-review.md写槽归还ROOT；reviewer停止写入。**
