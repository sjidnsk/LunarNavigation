# 轮式控制器标准 Path 地图入口

`controller_backend=mppi_path_follower` 与 `input_mode=incremental_path` 共同选择
标准 Path 跟踪。标准节点默认 `standard_map_source=direct_p3`，直接订阅
`p3_map_topic=/Car/T3/mapping/grid_map`，不要求 planner 提供 GetPolicyMap、
PathReference、Action 或 ACK。旧 MPPI/PathReference 仍沿用 policy_service；
标准模式也可显式选择 `standard_map_source=policy_service` 使用旧服务地图。

标准模式未显式指定 `platform_config` 时，读取已安装
`lunar_nav1cm_planner/share/lunar_nav1cm_planner/config/wheel.yaml`。
速度、加速度、曲率和 intrinsic 地形阈值从同一能力文件读取。车体尺寸取
`body_extent_m` 的长宽（当前 1.182×0.818 m）；正值 `body_length_m`、
`body_width_m` 可显式覆盖，0 表示使用能力文件。能力配置与未知策略为启动参数，
更改后需要重启。默认最大线速 .2 m/s、角速 1 rad/s、加速度与角加速度 .5、
曲率 1/m；标准节点位置/yaw容差 .1 m/.05 rad，停稳阈值 .01 m/s、.01 rad/s。
旧 launch 仍显式传入历史速度/容差参数；通过该 launch 使用标准模式时，应显式
传入标准参数值，节点缺省不会覆盖 launch 中的显式值。

单地图 worker 使用 P3MapStore/真实 NativeTerrain，在实际源分辨率进行地形判定。
任意正源分辨率及热变化、旋转、滚动窗口由共享 P3 几何处理。纯共享模块
`nav1cm.follower_map` 先抽取超过实际车体外接圆半径的 halo，再以显式 native
距离场计算闭格全域支撑，最后裁成 500×500、.01 m、5×5 m。保守外接圆允许
任意 yaw，包括原地转向；不伪造 stationary_anchor。下游只再增加执行净空，
不重复膨胀车体。窗口之外的 source 未知仍是未知。

nav1cm UNKNOWN/FREE/BLOCKED=-1/0/1 显式映射为控制器 0/1/2。
默认 `allow_unknown=false`。true 只改变独立 eligibility 掩码，不把 UNKNOWN
写成实测 FREE，也不借用 StartAssumedFree。实际已知障碍永远不能借此通过。
保守支撑格状态表示完整车体是否受已知自由支撑，而非 intrinsic 中心像素。

Path 输入为 reliable/volatile/depth1。TF 使用最新图，订阅 `/tf` 与
`tf_static_topic=/tf_static`（reliable/transient_local），接受零/相同 stamp
及链式变换；不要求与 Odometry stamp 配对。Odometry 保留有符号车体速度；
尚无 TF 的输入保留待重试，不更新有效接收活性。只做必要结构/有限数检查。

活性用 monotonic：`path_timeout_s=.35`、`odometry_timeout_s=.5`、
`p3_map_timeout_s=3.0`。成功处理的同 epoch 新观测也刷新有效地图接收时间；
坏消息、未转换消息不能续命。地图接收和处理时间分开记录。模型时间与实际发布
命令历史统一使用 ROS clock；缺 stamp 用当次 ROS 接收时刻，不能伪造旧 source age。
标准反馈年龄预算使用配置的 Odometry timeout；实际 source age 仍进入延迟制动证明。
静态 TF 不按 stamp 老化，超过50 ms只记录计算耗时，不单独剥夺执行权限。

优化后重取当前地图，若 revision/epoch/origin 改变，在新图上重新认证所选指令及
完整延迟制动尾部。缺状态/地图只发送保底零目标，并发布
`certificate_unavailable, certified_stop=false, measured_stopped=null`，不能视作
已认证停车。正常结果单独记录实测停稳；有限局部参考到头只 WAITING，不报告整次导航完成。
销毁节点先发布停车目标，再 join worker，最后释放 native 地形资源。

本任务只在隔离 Humble 测试域92、ROS_LOCALHOST_ONLY=1、验证命令话题验证。
Jazzy、Orin、实际 DDS 端到端、真实 rosbag、RViz 与实车均需分别验收，未执行为 NOT_RUN。
