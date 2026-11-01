# 终端停滞深入诊断（2026-10-29）

任务GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE；基线fd476a4。用户要求再次深入分析run-r4距目标0.1118845932m停滞的具体原因。本轮仅增加隔离进程观测包装和离线重放，不修改产品算法、配置、目标、容差、模型或训练。

## 证据边界

run-r4原始记录显示最小距目标0.1116318602m，最后明显运动/非零指令约50.4498s，随后至91.85s停稳且持续零指令。原始日志未保存跟踪器内部failure/stage/候选成本，不能追溯确认r4唯一根因，不能将下面r5时序回填到r4。

在相同已构建源码、冻结Actor/config/profile、差速模型和90秒窗口下，run-r5用只观察返回值的包装器记录worker原始候选、planner结果、实际发布Path与StandardPathFollower内部状态。异步执行/观测开销使数值轨迹不同：本轮最终距目标约0.0363m，仍未自然到达。它证实了一条真实终端交接和故障回传缺陷，不是原0.112m停点的精确重复。

原始远程目录：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-drl-inference-closure-20261029/run-r5/。trace/下两进程JSONL、close-snapshot.npz、replay-summary.json及原有runtime/events/plant日志保留；观测脚本在同级scripts-r5与本地D:/CodexDownloads/lunar_navigation/git-preservation-20261029/terminal-diagnosis/。不上传模型/原配置/数据，不训练、不改原安装。

## 已证实的时间线

以下时间以首个follower观测为零点，与原始wall时间可关联：

|时间|规划侧实际状态/参考|跟踪侧实际状态|
|---|---|---|
|约23.55s|实测进入0.1m终点位置范围；GOAL_BRAKING，制动候选仅位移0.000451041m、4点|仍TRACKING旧平移stage0/2；旧终点(0.098887943,1.892707246,yaw2.05920817)|
|约28.31s|持续基于最新状态提供制动/终端参考|旧终点剩余距离0.03790m、yaw误差0.08634rad；32有效采样、weighted输出；后续命令逐渐趋零|
|约43.31s|实际已在全局终点位置范围内，但yaw未满足|距旧终点0.02982417m、yaw误差0.08099998rad；no_progress_s=15.000029269，锁存TRACKING_STALLED/FAILED|
|约43.35s以后|GOAL_TURNING；实际发布41点纯旋转Path，从yaw1.97820819到3.92699082|仍保留旧平移stage0/2，failure=TRACKING_STALLED，持续零指令|
|90秒窗末|Action仍RUNNING/GOAL_TURNING，result_pending=true，tracking=null|旧阶段FAILED，未执行最终旋转|

实际Path记录确认最终旋转并未在ROS转换时丢失，同xy的41个yaw点完整发布；全局规划没有断路。启动时仅4帧missing_or_stale_map；停滞前持续有32有效采样，不能把长期停车归为持续无地图或所有动作碰撞不安全。

## 根因链一：旧参考屏障与终端制动的执行语义冲突

standard_path_follower.py:80-84用公共0.1m容差判定已在旧终点附近；若旧yaw尚未完成且新参考起点靠近旧终点，将旧stage插回新参考首位，以保护未完成姿态。这一保护没有区分普通行进路径与终端制动/位置保持/最终旋转意图。

插回后有两个stage；同文件:160-164又将中间stage位置容差收紧到最多0.005m。结果是：规划器已经要求在当前位置制动，跟踪器却仍要求精确完成旧局部几何终点，再执行新的制动/旋转。

离线确定性重放原输入，不改函数：新制动参考总位移0.000451041m；old_stage_preserved=true、stages=2；旧终点距当前0.093490501m、yaw误差0.113618993rad。公共位置容差0.1m，插回后有效中间容差0.005m。重放与在线观察一致，证实责任合同冲突。

这不表示所有旧屏障均可直接丢弃：普通拐角、换向、碰撞/制动约束仍需保留。缺失的是终端参考的明确替换/接受权限，而非简单将5mm改成10cm。

## 根因链二：跟踪器故障没有进入导航Action反馈

mppi.py:368-382按物理进展计时，最终触发TRACKING_STALLED。故障锁存后:341-343提前返回；StandardPathFollower._recover_failure仅处理MPPI_STOPPING_UNSAFE，收到新Path并不自动清除TRACKING_STALLED。这一锁存本身不宜随便取消。

但标准Path模式的node.py:296-300不创建TrackingStatus发布器；标准循环:657-660仅发布诊断与Twist。planner节点没有跟踪器状态订阅；navigation_task.py:82-116只依据位姿、停稳和planner packet结果决定Action状态。于是planner已发出旋转路径，tracker却已失败，Action仍根据规划结果显示GOAL_TURNING/EXECUTING，DRL只能等待。

active_segment_revision是全局route.version，不是局部候选接受次数。BUFFER_EXECUTING/GOAL_TURNING只能证明规划侧阶段，不能证明跟踪器已接受或正在执行。

## 尚未闭合：MPPI为何在旧阶段逐渐趋零

在线可以确认有安全候选、weighted更新和极小输出，不能据此声称优化正确或所有更优动作都不可行。用捕获的旧阶段/停滞前状态调用原_rollout和_tracking_cost（不含地图认证、不加入新控制策略）比较：零指令成本0.2158254，v=.003/w=0成本0.1264803，v=.01/w=.01成本0.1204556。因此“原代价只能选择停车”并不成立。

源码mppi.py:425-432还显示：先计算加权平均序列，再只要该序列安全认证通过就选用，不比较它是否比最优采样或上一轮更能推进。因此weighted不是成本下降或收敛证明；这是一项算法性质，不等于已找到本轮每次趋零的数值原因。

该离线比较未重建原随机发生器状态、完整_nominal、命令历史与所有采样/加权成本，不证明这些对照序列当时进入候选集或通过原在线地图认证。若要继续解释趋零的数值机制，需捕获采样序列→各项成本→权重→加权序列→认证/实际首指令的完整链，不能直接把降低yaw权重作为已验证修复。

MPCC的虚拟s与实际xy由软代价关联确有一般风险，但本轮末次worker候选实长约0.102168m、Δs约0.102056m，尚不能用“虚拟进度冒充运动”解释这一终端交接故障。

## 后续设计建议（本轮未实施）

1. 局部规划拥有终端位置/姿态与阶段安排；标准Path几何接口可保留，但需要可识别的制动、位置保持、最终旋转及参考替换/接受语义。跟踪器不应把已退休的普通局部终点升级为必须5mm完成的任务。
2. 全局终点位置完成集合和转向期间位置保持集合由同一责任方定义；不靠宽松识别旧终点、随后悄然收紧来处理交接。
3. 跟踪器以会话/局部参考身份上报ACCEPTED/ACTIVE/STOPPED/COMPLETED/FAILED；任务层消费匹配且新鲜的状态，不能继续将规划buffer当执行成功。故障和实测停稳分别记录；恢复必须是明确协议，不能新Path一来就清故障。
4. 在改终端优化代价前，补齐上述执行合同和数值成本证据；不扩大容差、不重训、不跳过认证来制造到达。

## 处置

run-r5完整闭环仍FAIL，0自然到达；冻结输入前后hash不变、guard_attempts=[]，四直接子进程exit0，无强制退出/残留worker。产品源码未修改，主线/开发线未晋级。本诊断与任务登记本地提交保存，观测脚本和大体积运行artifact留在仓库外。Orin/实车/坡地与真实滑移恢复NOT_RUN。
