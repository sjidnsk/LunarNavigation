# 终端参考接管与执行反馈合同

任务 GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE/EXECUTION-CONTRACT；公共owner integration/pure-planner-orin，复用hk/drl-integration-20261029隔离工作树，基线70ef6f3。用户已明确要求先落实终端替换权限、统一阶段完成条件与跟踪状态回传，再处理优化代价。

## 设计

保留nav_msgs/Path唯一几何来源；新增配套PathExecutionReference元信息消息（不复制Path）及TrackingStatus附加绑定字段。元信息以Path header stamp+完整Path SHA256绑定，含session_id（Action UUID或确定性topic身份）、单调局部reference_revision、intent(FOLLOW=0,BRAKE=1,HOLD=2,FINAL_ALIGN=3,INVALIDATED=4)、路径digest及位置/yaw/停稳容差。TrackingStatus.segment_revision在此模式表示局部reference_revision，附reference_digest/reference_intent/reference_accepted；旧incremental_reference语义不改。global route.version不作为局部接受身份。

FOLLOW保留普通有序几何屏障；BRAKE/HOLD/INVALIDATED由任务所有者授权退休旧阶段并执行有界停车；FINAL_ALIGN授权终端接管，实测未停稳时先有界停车再接管。终端接管不得再将旧平移stage插回；位置、yaw、停稳阈值从同一owner元信息获取，终端旋转使用owner位置保持集合，不把退休旧终点变成5mm要求。真拐角/换向屏障保留。相同session的FAILED不因新Path自动清除；新session可在真实停稳且停车认证通过后显式开始，不伪造指令历史。

Path和元信息可乱序到达：控制器有界缓存按stamp/digest配对，未配对不执行；拒绝旧revision、身份/摘要错误及未知intent；每次配对不能冒充已接受，status的reference_accepted以执行器实际接管为准。等待配对/过期/缺观测只能发安全停车请求，不能报执行完成。

任务层维护有界已发布参考登记和新鲜状态门禁，拒绝旧session/未知revision/摘要/intent不匹配、回退和陈旧状态。已知当前跟踪失败触发取消规划并等待独立里程计停稳，再Action INTERNAL_ERROR/原tracking reason；不能继续EXECUTING。启动宽限与持续反馈超时均有界(默认2s)。到达仍需任务自己的位置/yaw/独立里程计停稳保持.3s，另需当前session新鲜已接受的停止/完成跟踪状态。取消按原合同独立确认。默认为兼容旧纯规划API关闭合同，当前组合控制入口、nav1cm DRL owned specs开启。

不改MPCC/MPPI优化代价、目标、容差、训练或能力模型。消息改变后在隔离Humble重建msgs/planner/controller及受影响DRL代码，旧安装不改；固定Actor/config/profile不变。所有未知层级标NOT_RUN，不合并/push。

## 分工与验收

- [x] A：纯execution_contract.py（Path hash、immutable reference/status、session身份）和navigation_task.py状态门禁/失败归因，新增纯测试；不写ROS节点或controller。
- [x] B：StandardPathFollower显式offer_execution_reference(points,reference)、实际execution_reference/last_execution_state、终端权限/停稳接管/新session恢复；新增纯回归（r5旧stage+微制动、延迟历史、失败锁存/明确新session、角度跨界/多段旋转）；不写node/metadata模块。
- [x] 主agent：消息、ROS双端pairing/发布/接受状态、Runtime队列和任务接线、组合launch/DRL specs，以及真实ROS合同测试。
- [x] 审查：篡改/乱序/旧session、状态时效、失败停稳再终结、参考只发布未接受不能到达、终端接管不跳真实制动/碰撞、普通Path兼容。
- [x] 验证：RED/GREEN与相关纯回归；隔离Humble构建/包测；原Actor有界90秒闭环和真实失败/取消/复位检查。若仍不收敛，保留完整成本证据而不调代价。

## A/B对接API

A模块nav1cm.execution_contract提供：FOLLOW/BRAKE/HOLD/FINAL_ALIGN/INVALIDATED int常量；execution_session_id(task_id,generation)->32hex；path_fingerprint(ROS Path duck object)->sha256；ExecutionReference(session_id,revision,path_digest,intent,stamp_ns,position_tolerance_m=.1,yaw_tolerance_rad=.05,stopped_linear_mps=.01,stopped_angular_rps=.01)；ExecutionStatus(session_id,revision,path_digest,intent,state:int,accepted:bool,stamp_ns,reason='',linear_speed_mps=0.,angular_speed_radps=0.)。两者冻结并校验基本类型/范围。状态state沿用TrackingStatus 0WAITING,1BRAKING,2ALIGNING,3TRACKING,4FINAL_ALIGN,5COMPLETED,6FAILED。

NavigationTaskCoordinator提供configure_execution_feedback(required=False,timeout_s=2.)；register_execution_reference(ref,now_s)；offer_execution_status(status,now_s)（只tick线程调用）；check_execution_fault(now_s)->str|None；fail_tracking(reason,now_s)取消session并进入ERROR_STOPPING，observe等独立新odom停稳返回INTERNAL_ERROR。submit重置新任务门禁；到达需新鲜已接受WAITING/COMPLETED且实测目标/停稳满足。旧默认无需反馈的测试保持行为。

B方法offer_execution_reference(points,reference)->bool（收到合法请求，非执行接受）；execution_reference为实际接受的ref或None；last_execution_state使用上述int，提供reference_accepted状态；FOLLOW用旧rolling规则，BRAKE/HOLD显式受控停车，FINAL_ALIGN明确退休旧stage；新session需认证停稳接管，旧sessionfailure不清除。

## r7发现与续接协议修订

真实固定Actor r7首目标自然到达，但第二目标的4秒局部旋转时域末yaw随实测状态变化，被误当作新终端目标，反复停车接管；完整闭环仍FAIL，不能归因优化代价。Action接受后的session绑定不可原地修改的最终目标，局部Path末yaw只是当前时域几何，不承担最终目标身份。不同目标必须新session；BRAKE后重新FINAL_ALIGN仍执行首次停稳接管。

连续FINAL_ALIGN仅在同session/同owner阈值和位置保持集合、实测平移停稳且独立停车认证通过时考虑续接。逐相邻yaw累计连续角度，旧剩余与新参考均须为单向纯旋转；新起yaw匹配当前反馈且不能跳过旧当前阶段。保留当前未完成旋转阶段固定目标、真实命令历史和物理停滞时钟；新参考必须覆盖旧剩余全部承诺转角，未来同向纯旋转后缀按最新几何归并为不超过pi/2的表示分段。时域尾点不是实际反向/换向屏障，不能将每次20Hz更新尾点累积成新停车点；跨pi保留累计角度和圈数。反转、混合平移、位置集合变化、owner阈值变化或覆盖/连续性无法证明时，先真实停稳再接管。不改变MPCC/MPPI代价或目标容差。

## 本轮收尾与剩余工作

本轮实现切片与回归已完成，545项隔离Humble包测通过，r9固定Actor完整闭环通过（2自然目标、4决策、取消/复位/再次取消、正常退出）。原r6/r7/r8失败完整保留。用户要求转向本质分析后不再继续逐症状调整；本切片保存待架构审查，不移动owner/main/apps。稳定执行机动身份与统一进展生命周期仍是下一阶段协议设计事项，见docs/validation/2026-10-29-navigation-execution-root-cause.md。
