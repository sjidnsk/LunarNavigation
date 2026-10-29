# nav1cm运动能力统一审查与验证（2026-10-29）

任务GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE/MOTION-CONTRACT；公共修复owner integration/pure-planner-orin，执行于hk/drl-integration-20261029，基线7fa0417。用户确认车辆可边走边转、可原地旋转，指令v/w独立限幅，实测滑移超速不裁剪。

## 统一结果

新增显式motion_model：differential_drive和curvature_limited。当前config/wheel.yaml与wheel_orin.yaml显式差速；未指定模型的旧API/旧配置保持curvature_limited。保留1/m数值仅供旧模型兼容，不用巨大曲率数值绕过限制。旧motion_reference执行器保持原合同，不自动继承差速能力；未知模型拒绝。

|层级|审查与改动|
|---|---|
|平台/入口|同一wheel文件→WheelLimits及TrackingPolicy；组合launch将同一模型传至两端；独立标准Path node按平台文件读取，显式覆盖可用|
|局部MPCC|差速不再加入行进曲率硬约束；初始种子、候选复核、制动尾部使用同一模型；既有方向/加速度/速度界保留|
|轨迹认证|差速不再以w/v拒绝合法转向，保留速度/加速度、动力学、整车扫掠与碰撞；证书digest绑定模型|
|求解缓存|模型经limits.digest进入solver指纹，新旧模型不共享错误求解缓存|
|标准Path MPPI|采样投影、预测筛选、连续折点检查、最终指令和发布前停车认证一致；差速去除曲率和横向加速度耦合，v/w独立限幅|
|实测反馈|原始速度进入响应与停车尾部预测；实测超速不能靠裁剪过关。测试2m/s实测停车距离超出自由空间时仍拒绝发布，输出停车意图|

加速度平滑、停稳/换向与旧参考衔接、新鲜度和障碍认证保留。本次未重写MPCC末端yaw代价/终端集合，也不宣称实现完整滑移恢复。

## 验证

- Windows本地：674 passed、126 skipped、6 deselected。六项需要本机缺失native库的测试移至台式执行；跳过的ROS/SDK等环境门槛不能当通过。已有D盘scipy复用，无新增依赖下载。
- 台式Ubuntu/ROS2 Humble：受影响lunar_nav1cm_planner、lunar_pure_wheeled_controller独立构建成功；colcon test-result为507 tests、0 errors、0 failures、0 skipped，含新增真实节点模型读取/覆盖和组合launch同源模型测试。
- 台式真实acados/native定向：87 passed、1 skipped、3 deselected。包括独立差速初态v=.01,w=.08真实求解→认证通过、旧曲率模型拒绝同候选；既有前进/倒车/原地旋转、净空/停车与native/Python一致性回归。三个未选测试依赖未上传的旧Orin操作脚本/配置，已在本地原树测试覆盖；一项显式skip保留原门槛。
- 标准Path跨层测试包括实际update连续五轮输出w/v>1.5且certify_publication通过，同动作旧模型拒绝，实测超速停车尾部保守检查；不只是内部函数断言。
- 两次独立只读源码审查通过，审查者未重跑依赖环境测试，其意见不作为运行证据。

台式构建/运行产物在/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-terminal-approach-20261029/的source-r3、motion-r3、build-remote-r3.log、pure-remote-r3-final2.log。源码/测试上传沿用既有授权；不上传模型、数据或原项目配置，隔离配置由台式既有wheel.yaml派生，仅添加模型字段，原文件/安装不修改。早期验证脚本的ROS scripts包命名冲突、缺NAV1CM_BUILD_DIR、部分源码快照缺旧Orin文件均已保留日志；最终按实际适用输入完成验证，未改产品适应测试。

## 完整DRL推理复核仍失败

复用同一Actor/config/profile冻结hash和12m合成场景，在原授权目录nav1cm-drl-inference-closure-20261029/run-r4执行90秒固定Actor推理。验证入口新增显式--motion-model，声明记录覆盖值并同时传给planner/follower；原冻结平台文件不改。node1日志已确认WheelLimits.motion_model='differential_drive'。

本轮行驶累计1.7164117526m、转角累计2.5476177108rad，无碰撞；最终位置(0.1455766303,1.7978191164)、yaw=2.1135874288，距首目标0.1118845932m。0自然到达、1 Actor决策，仍未满足0.1m位置和最终朝向/停稳合同；verdict.ok=false。原0.241m停滞不能被当作仍受旧曲率约束，但本轮进展也不能证明终端收敛。末端最后反馈为BUFFER_EXECUTING，尚需原在线候选/标准Path阶段证据，不能把剩余问题直接归因于单个代价项。

四直接子进程exit0、forced_exit=false、workers_remaining=[]、guard_attempts=[]、frozen_unchanged=true。失败清理取消不能冒充完整取消/复位验收；未训练、不读改原训练账本。

## 处置与下一步

本切片运动能力一致性已完成代码审查与上述适用验证，本地提交待合入；完整DRL反馈闭环仍FAIL，owner/main/apps不晋级。下一步按已讨论方向设计完整终端位姿优化，并采集剩余停滞的在线证据，不放宽容差、不手工指定目标、不通过重训掩盖。真实P3、高程坡地/滑移恢复、Orin、实车和策略效果仍NOT_RUN。当前及历史失败artifact/worktree均保留。
