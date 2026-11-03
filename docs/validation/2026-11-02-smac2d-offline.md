# Smac2D离线对照（2026-11-02）

任务 GLOBAL-GRAPH-REUSE-20261102 / Smac 子任务。owner integration/pure-planner-orin；执行者 Codex Smac 子代理，Windows + 192.168.100.216 Humble ARM64；分支 hk/global-smac-comparison-20261102；工作树 D:/CodexDownloads/lunar_navigation/global-smac-comparison-20261102；基线 0edfaa4f。状态：已提交待合入，ROOT统一部署；本子任务不部署、不merge/push，不修改局部/控制/地图/正式配置。

远端未安装Nav2。隔离下载官方ROS Humble arm64 deb并解包，调用 libnav2_smac_planner.so 1.1.20 的 AStarAlgorithm<Node2D>（TWOD）。不是自写A*，也不是完整Smac插件导航栈/平滑器。依赖与二进制均只在 `/home/yanfa/P4/tmp/global-smac-comparison-20261102`；未apt安装、未改P3/Env_X。构建入口 `build_smac2d_offline.sh`，运行仅ROS域217/local host，未启动导航或发布控制。首次构建实际遇到缺OMPL/bond/JSON头等依赖，已通过隔离解包补齐。

现场输入 `/home/yanfa/P4/tmp/global-graph-reuse-20261102/request.json`：最新3帧64m/20cm P3重建，36障碍/145free；**不等于正式累计epoch219地图**。原始套件/结果在隔离目录 `suite.json`、`results-suite.json`。本机副本 D:/CodexDownloads/lunar_navigation/smac-{request,suite,results-suite}-20261102.json。

10cm栅格，显式边界[-35,-40,35,30]，约49万格。硬半径75cm，加整格对角线14.14cm保守裕量，保证相邻中心连线及端点连接；50cm软带，8位量化，Smac cost_penalty=1。图的UNKNOWN没有额外罚则，故栅格UNKNOWN映射可通行free并initialize allow_unknown=true。与原图连续距离积分、原生栅格代价/角步长不是严格等价；栅格裕量可封闭原图可通行窄道。有限栅格边界可能阻止原图边界外绕行。未做Smac平滑。

|场景|搜索ms|栅格化ms|进程+I/O ms|长度m|
|---|---:|---:|---:|---:|
|首次|0.379|16.59|480.08|3.530|
|重复|0.388|16.17|477.10|3.530|
|多目标0 (-4,-9)|0.567|13.95|480.97|3.628|
|多目标1 (-6,-5)|0.766|20.53|472.77|6.456|
|多目标2 (0,-10)|0.495|13.64|470.57|2.745|
|局部改图|1.288|14.64|470.40|5.316|

局部改图增加矩形[-2.6,-9.3,-2.4,-9.1]并递增epoch。6例均返回路径，本机使用既有独立 segment_hits_round_box 对全部连续路径段及原始起终点连接段检查75cm半径，无碰撞。此为离线几何通过，不包装为项目正式 planning_outcome/PLAN_FOUND/has_reference 合同通过。每请求独立进程，重复/多目标结果**没有复用Smac对象**；进程I/O含ROS初始化与文本栅格传输，不能以亚毫秒搜索宣称端到端亚毫秒。构建、栅格化和进程创建不纳入Smac内部搜索budget，与项目总预算边界不同。

ROOT同机旧图构图profile约935ms、搜索0.3ms（ROOT提供，详细证据由ROOT归档），因此当前重点确实是构图。Smac搜索没有显示比现有图搜索明显更快；需比较端到端和首次/暖复用收益，不能以此表推断替换必要性。

验证：Windows指定Python `-m pytest tests/test_smac2d_raster.py -q` 1 passed；Humble ARM64真实官方库编译与6例离线执行通过。现有拓扑fixtures全套、精确代价等价、持久Smac对象、多地图/分辨率统计、多次置信区间、Humble包/ROS接口测试、Orin/车辆闭环均 NOT_RUN。专题资源保留待ROOT审查；未回收。首次/重复不是系统冷缓存实验。
