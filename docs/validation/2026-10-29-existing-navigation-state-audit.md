# 现有导航状态、完成条件与进展台账

2026-10-29，基线389896d，继续EXECUTION-CONTRACT任务。原则：如无必要勿增实体，保持模块职责。当前不新增maneuver_id、状态类、消息字段或新状态枚举。

## 现有身份足够表达本轮反馈归属

任务沿用Action UUID；每次局部发布沿用reference_revision、完整Path摘要、intent与时间戳。任务层按已发布参考目录核对反馈，控制器按配对后实际接受的参考发布状态。相同session内再次FINAL_ALIGN有不同revision，因而不需要仅靠session+intent归属。

迟到的已接受旧参考FAILED可单独锁存，而不回退最新正常反馈门禁；同revision失败不能被后来COMPLETED覆盖。已退休session、未知revision、摘要/intent错误和过期状态均拒绝。现有回归已经覆盖较旧revision/stamp的已接受失败、同版本失败后成功、取消时原session身份等场景。当前没有一个已证明必须加新身份字段才能解决的归属实例。

目录有界128项，ROS反馈新鲜度默认2秒。超出保留/新鲜度范围的输入拒绝，这是明确的保留策略，不证明身份体系缺字段。若未来确实需要跨更长延迟恢复或审计，先证明具体输入无法用原版本目录处理，再讨论最小扩展。

## 各层状态转换和完成对象

|层/现有转换|触发与判据|职责与结果|
|任务：接受→PLANNING|合法目标被session接受，保存原generation/UUID|最终目标与任务归属由导航协调器持有|
|任务：PLANNING↔EXECUTING|地图/位姿/route/candidate及规划结果就绪情况|表示任务调度状态；不能据此宣称跟踪已接受或到达|
|规划：FOLLOW/GOAL_BRAKING/GOAL_TURNING/ARRIVED|位置/朝向、真实初态与候选停车尾部认证|产生当前运动参考；ARRIVED是规划侧候选结论|
|Runtime：状态→现有intent|无候选INVALIDATED；ARRIVED为HOLD；GOAL_TURNING为FINAL_ALIGN；brake/cancel为BRAKE；其余FOLLOW|传递已有意图，不自行求解运动或发布任务最终成功|
|控制器：收到→配对→待接管|Path和元信息stamp/digest严格匹配|接收不等于接受，不把传输等待当运动完成|
|控制器：待接管→实际接受|新session/首次终端接管满足实测停稳与独立停车认证；普通滚动按既有有序几何规则|更新实际execution_reference；保留真实指令历史|
|控制器：不兼容几何→BRAKING→新stage|确需制动，停稳后实际切换|重绑比较方向/角度，不凭切换清停滞余额|
|控制器：当前stage COMPLETED→下一stage或WAITING|该几何目标的位置/yaw、安全支持与实测停稳满足|只完成本地几何；stage推进时重绑锚，不能替代任务终点|
|控制器：BRAKE/HOLD/INVALIDATED→WAITING|受控停车及对应认证|WAITING表示当前停车请求完成；INVALIDATED不能作为到达证据|
|任务：→GOAL_REACHED|同任务/代际、新鲜已接受终端反馈、独立实测目标位置/yaw及停稳，连续保持至少0.3秒|只有此层产生最终导航成功|
|任务：取消→CANCELING→FINISHED/CANCELED|取消规划，后续新鲜独立里程计确认停稳|无需满足目标位置/yaw；原任务身份不被cancel generation替换|
|任务：跟踪故障/反馈超时→ERROR_STOPPING→FINISHED/INTERNAL_ERROR|先取消规划，再由新鲜独立里程计确认停车|保留原失败原因，不因新参考清故障|

COMPLETED/WAITING在TrackingStatus中描述实际执行器当前参考处理结果，并不承诺“最终任务已完成”。参考接受也不是对整条Path逐点执行完毕的证明：既有滚动规则会保留未完成旧屏障并接入兼容未来几何。最终任务仍由独立目标判据把关。

## 进展与计时的唯一归属

MppiExecutor._bind_progress_anchor构造当前位置、当前几何行进方向及当前目标角误差，不写停滞余额/故障/历史。_observe_progress集中原有计算：真实有向位移大于0.03m，或位置保持范围内朝向误差较固定锚累计减少大于0.01rad，才更新物理锚并清no_progress_s；否则累计min(dt,0.2)。阈值、条件与计时语义均未调整。

StandardPathFollower只负责何时进入新的实际几何阶段：

|事件|绑定进展锚|清停滞计时|清故障/历史|
|同路径或兼容滚动更新|否|否|否|
|同向FINAL_ALIGN窗口续接|否|否|否|
|普通路径首次开始|地图可用、进入物理进展观测后初始化|否|否|
|合同首次/显式退休后实际接管|是，保留原合同交接判定；FOLLOW恢复不额外要求停稳|否|否|
|已停稳的不兼容几何接管|是，由_accept_pending处理|否|否|
|不兼容路径先制动、随后实测停稳接管|是，由_stop(switching=True)调用同一绑定函数|否|否|
|当前真实有序stage结束并推进|是，指向下一stage|否|否|
|同阶段新鲜实测有效进展|按原规则更新|是|否|
|新session在原安全门槛后实际接受|按新任务初始化|是|按原任务恢复规则|

现行no_progress_s是行进求解周期累计量，_stop不累计它，不能把它报告成整个任务墙钟超时。反馈超时由任务层单独检查；停车期间不在此次行为保持归并中另加超时或放宽计时。这一语义保留并明确记录，后续若要改变必须独立给出需求与回归。

## 消除的重复与保留的独立检查

消除：基类初始化锚、有效进展后更新锚、标准跟踪器阶段锚原本三处重复构造公式，现共用_bind_progress_anchor；进展与计时只在_observe_progress实现。实际接管、停车后换向及有序stage推进的事件判定保留原位置：普通首次开始须等待地图可用，而合同授权退休后恢复FOLLOW原本允许运动中重新绑定，二者不是重复判据。只有共用的数学实现被归并，不为合并条件新增标志或改变初始化时机。

保留：规划候选尾部预测、控制器实测执行完成与发布安全认证、任务最终目标保持。这些输入/时刻/完成对象不同，独立检查具有职责，不能视为冗余。控制器几何兼容性和运动认证仍属于控制器；任务层不介入MPPI求解。

本轮两个控制器源码之外不改运行代码；现有字段、状态、消息、配置、阈值、优化代价和训练均保持。现有同向续接对历史窗口覆盖的保守限制也未借本轮扩大；尚无无法归属的迟到反馈实例，因此没有必要新增实体。

## 验证与处置

重构前新增特征回归与原合同测试32 passed；最终全部相关纯控制器197 passed，任务到达/取消/故障13项、现有反馈归属与配对20项通过。Windows环境没有生成ROS消息，直接收集测试报缺依赖，未计作算法失败；纯测试入口仅提供禁止实例化的消息类型导入占位，没有模拟消息或ROS行为。实际消息/ROS包测试另在隔离Humble执行。


归并审查中的反例与处置：首版将两个首次接管条件合并，r10实际第二目标TRACKING_STALLED，完整闭环FAIL（不能使用r9历史通过替代）。只读审查和两个先RED的反例确认了可观察差异：运动中BRAKE→FOLLOW漏绑定，以及普通路径无地图等待时提前绑定、可能将等待期间位移误计为进展。最终撤回这部分条件归并，恢复389896d全部转换判据，只保留计算实现共享。两个反例都保留为回归，不调任何阈值或超时。r10失败不能单凭新反例回填为唯一运行根因；原始证据完整保留。
最终源码仅两处计算实现归并，独立只读复审通过。另对基线389896d和最终代码做六类场景145步同输入对照，命令、phase/failure、几何stage游标、progress_m/no_progress_s及锚一致（数值容差1e-12）。覆盖普通/合同平移与旋转、运动中制动后恢复、无地图等待后恢复；不是实车或ROS证据。

最终三个源码/测试文件按state-consolidation-r2-manifest.json逐项SHA256核对上传，成套新隔离目录execution-state-r2的Humble三包构建/549 tests通过，0 errors/failures/skipped；旧execution-r4、execution-state-r1及首版r10记录保留。

r11原Actor/配置/profile/容差和90秒预算下，两个不同目标自然到达，4次真实决策，取消/复位/再次取消完成。自然到达位置误差0.099814033m和0.099766666m、yaw误差0.037876167rad和0.037515878rad，终态实测v/w均0。但verdict仍FAIL，唯一错误no fresh decision after completed feedback：第2次完成与第3次决策processed_stamp_ns都为1793276935293950078，revision均27。不能把“确实发生新决策”替代“使用完成之后的新观测”的严格条件，更不能引用历史r9通过覆盖本轮结果。

源码可区分该实例：InferenceRuntime在结果返回时已调用require_fresh_snapshot，后者按请求序号要求新响应，不保证地图处理stamp严格增加；poll_snapshot已有minimum_map_stamp_ns参数，本Runtime调用未传入。本例是两处“fresh”判据不一致的后续核查项，现有时间戳和接口足以表达，不能据此论证新增机动实体。尚未证明是控制器计算归并造成；本轮不改变DRL门禁、验收器或验收阈值。

r11最终state=CANCELED/stopped=true/inflight=false；四直接子进程exit0，无强制退出、剩余worker或训练尝试，三个冻结输入hash未变。证据本地execution-tests/run-r11及台式analysis-artifacts/nav1cm-drl-inference-closure-20261029/run-r11；首版r10失败及两个RED反例原样保留。本轮处置为现有状态审查及控制器计算归并已验证候选，已提交待合入；严格完整DRL闭环本轮未通过，长期线不晋级。Orin/实车/坡面打滑/Isaac在线/训练均NOT_RUN。原主线与DRL长期线不移动，原目录dirty内容及旧artifact保留。