# P4 控制统一域10

任务CONTROL-DOMAIN10，owner integration/pure-planner-orin，ROOT单写入，复用hk/p3-pose-wait-20261030/main-worktree，基线1dad3dc。用户明确要求统一控制域10；远端仅P4写入，P3与Env_X只读。实现scripts/simulation/p4_tcp_runtime.py，既有有界测试入口check_tcp_navigation.py也改为单控制域10。

## 接线与职责

P3源仍在57；P4只有输入读取节点订阅其地图、定位、动态/静态TF，单向原样转发至10，不改源戳、不插值、不转发控制回57。TF/定位输出提供可靠QoS兼容RViz，源订阅兼容既有发布端，地图/静态TF保留transient_local。导航、跟踪、P4 RViz、车辆适配及原TCP接口都在10。

/goal_pose由普通RViz与P4 RViz发布，导航订阅；控制器输出/p4_p3_debug/cmd_vel供P4车辆适配订阅；适配通过/car/cmd_vel和/car/set_mode交给原TCP节点。/car/external_control_lease为Env_X GUI已有接管心跳：P4发送时GUI不覆盖速度；1.5s失去心跳GUI恢复并停车。本项目只使用既有机制，未改GUI。适配沿用测试中已有0.5s指令/车辆反馈保护与原低速调试上限，跟踪状态无效时零速/park，退出释放租约。启动不恢复旧目标；不修改导航、认证或MPPI算法。

## 验证与当前运行

本机和216/Humble定向54项通过，语法检查和diff检查通过。首次目标机组合52通过1失败是NAV1CM_BUILD_DIR回退干扰空目录用例，移除仅测试进程变量后53通过；随后TF兼容回归新增后54通过。两次只接入检查均收到四类数据且无非零指令；最终12s记录地图7、定位15、动态TF15、静态TF6。

核对PID及命令后，停止旧P4导航22555及P4专用RViz45125；保留普通RViz39088、P3与原TCP车辆节点。新P4 operator53013，导航启动组53031，RViz53032；身份及准确命令写在P4/run/control-domain10-current.json。运行日志位于P4/run/control-domain10-live-20261030。重启应先核对该身份并停止旧operator，不并行启动多个接管者；环境仍通过P4/source/scripts/simulation/p4_environment.sh设置，未改用户或系统ROS环境。

真实图确认：域10/goal_pose有两个RViz发布且导航订阅；域10控制器输出有P4适配订阅；域10/car/cmd_vel有lunar_car_node接收。域57没有P4控制命令发布者。比对12条P3源与域10转发消息序列化CRC完全一致，包括原时间戳。源码/静态、Humble真实图证据仅证明域与消息接通，不证明导航性能或车辆到达。

用户在新RViz选择目标后出现正式PLAN_FOUND、非零控制已转发；但随后跟踪器TRACKING_STALLED锁存停车。一次25s只读观测记录428条控制器指令仅4条非零，TCP端774条指令中同样4条非零，45个车辆反馈样本存在微小运动；末态控制零、车辆近零、mode=park、tracking state6/accepted=true。不能将“非零曾转发”报告为正常起步或持续行驶。人工目标目前规划仍继续但跟踪器失败，属于故障生命周期待定位问题；本次没有改无进展阈值或故障逻辑。长距离、实际绕障及自然到达未通过，仍待后续修复验证。

远端只上传必要源码和测试，不放设计文件；D盘证据execution-tests/control-domain10-20261030保存图、输入原样比对、25s原始观测及当前诊断。代码本地提交待合入，长期线未移动，不push。
