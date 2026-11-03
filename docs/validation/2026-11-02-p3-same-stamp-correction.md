# P3同时间戳位姿修正接收

P3-POSE-CORRECTION-20261102；owner integration/pure-planner-orin，专题hk/p3-pose-correction-20261102，基线b1575f9b；ROOT，Windows/TCP ARM64隔离。

用户确认三类输入：完全重复忽略；同戳有效修正更新状态但不增加完成/停稳/判活观测；旧戳不覆盖当前状态。

现有接收入口offer_odometry和应用入口_apply_pose_input均用<=拒绝了同戳消息。另有源回退故障会把迟到旧包变为不可用，违背当前确定的处理规则。改为旧包在进入缓存/应用前忽略，同戳先比较完整测量或转换后VehicleState，相同状态不重写，不同状态通过现有坐标/数值转换路径更新。使用已有_applied_pose_stamp_ns与_observed_pose_stamp_ns区分已应用和已消费，无新增接口、版本或状态实体。
完成证据对有源戳的两种控制模式都按严格推进判定；无源戳的旧测试/演示输入保留接收序号兼容。接收判活高水位仍只在源时间推进时更新，同戳修正不能持续续期。没有改真正的ROS时钟回退、未来时间或数值/坐标异常处理。
状态修正仍进入原有规划/认证消费路径，不设置新的制动或全局重规划事件。原有位置阶段锁存保持不变；朝向修正使到达不成立时清除旧保持证据，后续真正新观测仍能完成任务。

测试先复现同戳丢弃及重复观察问题。新增覆盖直出与Path链的状态修正、重复、旧戳、新帧、判活不续期，以及朝向修正撤销到达证据、修正本身不完成、后续新帧可完成。两项旧测试的断言原本要求丢弃修正/旧包触发故障，按用户新规则更新；没有修改判定阈值或扩大超时。
Windows最终1340通过95条件跳过（one_cm及受影响runtime/task/feedback/prediction）；Windows全ROS测试的rclpy/ament缺失不作通过证据。Humble ARM64隔离包构建成功，全包120通过；隔离使用域88、FastDDS，未接入生产控制话题。首次环境夹具缺文件和Cyclone回环诊断干扰测试READY读取的日志保留，补齐既有测试夹具并在隔离FastDDS重验；生产DDS配置未改。

产物：本机D:/CodexDownloads/lunar_navigation/global-planning-perf-20261101/pose-correction-local-final.log；远端/home/yanfa/P4/tmp/p3-pose-correction-20261102/build-test-v2.log、test-final.log及隔离install。只上传必要源码/测试配置，不改P3/Env_X。
处置：已提交待合入；正式P4尚未切换，车辆闭环NOT_RUN，未push/PR或删除工作树。正式切换需重启P4并清除当前目标，等待用户选择运行切换时机。
