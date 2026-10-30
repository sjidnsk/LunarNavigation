# 当前探索默认20cm落地记录

任务P3-EXPLORATION-20CM-20261030；用户明确要求“换成20cm分辨率，与P3高程地图一致”。owner integration/pure-planner-orin，专题hk/p3-delayed-input-20261030，基线f166c94。

## 实现范围

当前nav1cm_exploration.launch.py的coarse_resolution_m默认由1.0改0.2；独立ExplorationMap构造与ExplorationMapNode节点默认同步0.2；DRL owned_nav1cm_specs中的同一地图节点显式覆盖同步0.2。policy_resolution_m原本0.2，保持；DRL消费GetPolicyMap而非探索OccupancyGrid，不改策略特征/模型。

仅改变探索输出栅格尺度。保守完整FREE覆盖、车体包络/净空、UNKNOWN/已知障碍、地图源戳、控制超时与能力保持。旧incremental_v2导航的同名coarse_resolution_m还控制全局引导网格；旧config/exploration_navigation.yaml、incremental RViz和历史演示不机械替换，以免把全局规划计算尺度混入探索修改。

## 证据

- 两个新增入口回归修改前均失败（实际1.0而非0.2），修改后当前组合/DRL规格及相关无控制统计测试26 passed。
- 新增C++默认构造回归确保0.2m投影保留同尺度细图FREE格，不靠比较配置字符串。216 Humble/aarch64隔离地图包完整构建/安装通过；受影响地图与保守投影CTest串行2组、GTest实际15项（9+6），0失败/错误。
- 真实ROS重放domain88、私有/p4_resolution_20cm话题；只地图节点，没有规划器/控制器/车辆桥或目标。默认参数查询coarse_resolution_m=0.2、policy_resolution_m=0.2；实际输出float32分辨率0.20000000298023224，98x99格，1122 FREE、0 OCCUPIED。监督器ok=true、节点正常退出0；没有控制发布者。输入为之前保存的真实CDR，源时间戳保持，不能称当前在线运动状态。
- 同帧生产探索核心：1m为20m²/2前沿/0候选；20cm为44.88m²/2前沿/4候选。三个尺度的实际机器人格均UNKNOWN；已知FREE连通起点未成立，不宣称候选从当前车位可达或执行闭环通过。详见2026-10-30-p3-map-layer-audit.md。

## 隔离与运行处置

代码、配置、构建、安装、测试和实际回执仅216的/home/yanfa/P4/run/p3-exploration-20cm-20261030；没有覆盖tcp2正在调试的/home/yanfa/P4/source、共享安装或其进程，也未改P3。真实节点验证已退出。新增verify_resolution.py可复现参数与输出验证。

首次隔离复制缺少test目录导致配置失败；补齐必要测试源码。新增测试最初位于命名空间外导致编译错误，已修复。低并发冷构建经身份核验后停止，仅本任务进程组，复用缓存低优先级四线程继续；全部日志保留，最终build-r4.log完成。验证器初版使用较新rclpy参数客户端，Humble没有该模块，已改实际GetParameters服务，verify-r2/result.json通过。失败未删除或计通过。

源码本地提交待合入，未移动main/TCP应用线、未push或删除。tcp2现有运行进程不宣称已切换20cm；其后续启动可显式传coarse_resolution_m:=0.2消费，应用线源码/部署由同一写入者衔接。本轮不因改配置开启车辆控制。旧全局引导算法、DRL策略效果、完整在线探索/自然到达和实车验证NOT_RUN。
