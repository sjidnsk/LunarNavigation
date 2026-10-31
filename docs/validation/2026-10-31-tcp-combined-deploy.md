# TCP/tcp2 组合部署与验证

用户要求暂停共享基础地图实现，先整理 tcp 与 tcp2 优化并同步 192.168.100.216。组合分支 hk/tcp-combined-deploy-20261031，提交143a11fd2c0eba0e20ab003b9045335707041a2b，双亲eac7d14及2338d7b。未包含共享地图3679aa8之后实现，无push或长期线移动。

本会话：职责精简、连续旋转、统一完成证据、真实制动接转向、局部投影、圆包络、TCP入口共用、双向slew及启动参数冻结；tcp2：认证的全局参考平滑及独立参考可视化。五处冲突逐项组合，独立复审APPROVED。

本地导航720通过123跳过；平滑69项属于该总数。216/Orin ROS2 Humble三个包独立构建成功：lunar_nav1cm_planner、lunar_pure_wheeled_controller、lunar_pure_exploration_ros。实际规划器11组62项，控制器26组633项通过。首轮因漏上传平滑测试夹具而配置失败，补齐后构建通过；shutdown测试先后被NumPy版本警告及CycloneDDS回环提示抢占READY行，以隔离域157正常网卡并过滤UserWarning后通过，未修改产品逻辑或导航参数。

全部机器写入位于/home/yanfa/P4。29项必要源码/配置按清单同步，旧版本逐文件备份artifacts/tcp-pre-sync-backup-20261031，回执artifacts/tcp-sync-receipt-143a11f.json。候选源码artifacts/tcp-combined-candidate-20261031，Humble安装artifacts/tcp-combined-install-20261031；本轮环境显式source该overlay，不覆盖原安装。旧车辆消息安装仅只读使用。

P3原算法/配置只读复用，绕过会写P3状态的原总入口；本次所有状态、配置副本、数据库、日志、传感器中继包装及可视化状态位于run/p3-live-20261031。UE端点192.168.10.23，车辆初始park；用户重启UE后连续采集、融合及建图恢复，原RViz/高程监视器已在桌面:0打开。未改P3文件。

低速有界复测：run/tcp-combined-143a11f-20261031/navigation，目标当前车头前0.5m，v上限0.05m/s、w上限0.1rad/s；域10，P3源域57只读桥。正式PLAN_FOUND且has_reference=true，起步INITIAL_START_COMPLETED，已发非零并实际前进。75秒未自然到达，最终P3距目标0.05927m，最大P3位移0.46058m、TCP位移0.44547m；起步/跟踪PASS、到达NOT_REACHED、停稳PASS。127条非零命令转发，认证过期18次、认证中地图变化12次、BRAKING32次、FINAL_ALIGN16次；已取消收尾。未放宽超时或跳过认证。此次为短直线实测，长距离/绕障和平滑转角实测仍NOT_RUN。P3建图与可视化保留运行，导航控制测试已结束。
