# 导航连续性与RViz切换（2026-10-31）

任务NAV-REFERENCE-CONTINUITY-20261031。隔离分支hk/tcp-combined-deploy-20261031；基线bdf12aa，产品4238010。共享基础地图暂停；未push或移动长期线。

已删除路线刷新时无条件清空有限执行来源、P3同世界移窗引起的证明撤销、可重算发布失败时清MPPI初始序列。原source身份/版本保持，当前图整段及前缀认证继续；真实目标/政策变化、取消、碰撞、来源耗尽仍退休制动。新结果仍严格匹配当前请求。公共clip仅改同内核array方法，不改变数值算法。

用户另授权TCP compute .55秒、指令等待.60秒；遥测/停稳仍.50，地图timeout0、定位推进判活4.5、horizon140及能力不变。fresh-state原模型持令15步=.75秒，覆盖名义轮询.65/.70秒，不是Python调度或独立1秒watchdog的硬界证明。

RViz显示起点、选定目标、全局参考、局部认证路径与P3观测轨迹。显示独立线程、两单槽邮箱、5Hz、最多2000点；全局源戳推进、目标前合法点/目标后首点锁存、Marker DELETE。显示不参与控制，无固定source-age门禁。原两个最终复审finding均ADDRESSED。

## 证据

- 本地第一批定向70通过2跳过；全集754通过126跳过6缺native环境失败，不宣称全集通过。
- 216/Orin Humble两包实际构建；planner62、controller647项通过。性能独立14项已正式CTest注册执行；661个controller独立用例不重复统计手工及CTest。
- 最终显示/时序57项目标机通过。真实ROS/DDS验证late join、Marker ADD/DELETE与A→Goal→B起点绑定，无车辆命令发布。
- 2000点、50Hz观测、20Hz dummy relay，4238010最大心跳间隔195.51毫秒；不是车辆或硬实时验收。旧无限频.60097546秒失败保留，.60判据未放宽。
- 代表性离线完整update两轮42组输出/phase一致，中位性能收益3–8%；新.55预算三组中位约98/99/104毫秒，不能推断现场到达通过。

## 当前运行

用户授权P3定位丢失后重启：旧P4 operator12651先停车，measured_stopped=true，导航/RViz退出0；只停自有P3 pipeline6369与relay6375，保留采集/轮速/车辆park接口。新P3运行目录/home/yanfa/P4/run/p3-live-r2-20261031/results/live，定位和地图恢复推进。P3源码配置只读，所有产物P4内，旧地图日志保留，旧目标未重放。

产品4238010六必要文件已逐项备份同步P4/source，回执artifacts/continuity-deploy-receipt-4238010.json，备份artifacts/pre-continuity-deploy-20261031。运行overlay为P4/artifacts/nav-continuity-install-20261031，operator31078，run/tcp-interactive-4238010-20261031；新RViz配置source/scripts/simulation/p4_navigation.rviz。已核对域10、compute=.55、地图0、定位4.5及140steps，定位/地图接收正常，WAITING_INPUT:goal，初始forwarded_nonzero0。等待用户选点，无自动目标。P3原UI保留。

新版自然到达、长距离与绕障现场验证NOT_RUN。旧版本0.5米任务未自然到达、误差5.93厘米的证据不改写。主线合入待后续安排。

tcp2已交付dc334b1的两个目标入口模块与测试，隔离环回通过，生产尚未启动。只读PointStamped为/p4_p3_debug/tcp_goal_point，不回写/goal_pose。已协调统一入口，新P3坐标会话不可复用旧锚点；未覆盖tcp2来源。

原始报告在本工作树.superpowers/sdd/2026-10-31-navigation-reference-continuity及artifacts，目标机P4/log/nav-continuity-*与display-*。一次早期小型pytest默认产物C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-114用于fixture，确认无需证据后可删除；后续测试全部显式D/P4临时路径，本轮不删除。
