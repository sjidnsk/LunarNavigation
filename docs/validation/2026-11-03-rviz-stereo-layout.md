# P4 RViz双相机布局

任务TCP-RVIZ-STEREO-20261103，owner app/orin-unreal-simulation，ROOT复用独立TCP专题70b96be7；用户选定右侧上下相机、中间完整地图。修改仅config/p4_topics.yaml、scripts/simulation/p4_navigation.rviz及相应配置测试，不改P3/车辆/采集/导航算法。

复用已有/fusion/left与/fusion/right，实收640x536 mono8（P3可视化当前也使用的已正常化图像），同域57直接订阅，无新图像处理或转发节点。内置rviz_default_plugins/Image两视图，Best Effort/Volatile/KeepLast/depth1。统一topic来源新增p3_left_image/right_image。Qt原生QMainWindow State保存右侧上下dock；左侧Displays与导航探索tab切换，默认导航面板在前，中央保持地图。

操作前保存当前RViz未保存设置，发现用户已手工加入/T3/mapping/lidar_map与/T3/mapping/elevation_cloud；保留样式和启用状态、用户视角及工具，在模板中统一两个话题键，重命名为P3点云地图/高程点云。P3 odometry Keep1保持。保存当前窗口1850x1016，原生Qt状态离屏验证右侧两个dock及中央宽度。源配置不覆盖其他模块。

91项相关回归通过，5项Windows平台跳过；配置YAML/TopicKey完整渲染及diff检查通过。Humble实际RViz热加载原生Image，两路画面已目视确认；DDS端点确认p4_planning_rviz左右各一depth1 BestEffort Volatile订阅。原operator24344、RViz24524均保持，不重启导航，不发布目标/任务/速度。界面重载不作为车辆到达或探索验收。用户当前点云地图层未启用、高程点云启用，按原设置保留。

正式源码与活动run/p4-20261103-114117-24344/p4_navigation.rviz已更新；备份/P4/build/rollback-rviz-stereo-20261103（含源码、topic、活动配置），另/P4/tmp/rviz-stereo-layout-20261103/live-ui-backup.rviz保留当前GUI原始快照。Qt布局JSON、DDS检查和截图在同一tmp目录，本机D:/CodexDownloads/lunar_navigation/rviz-stereo-layout-20261103；截图after-loaded.png显示右侧双相机与左侧导航面板，final.png为用户随后切换Displays和缩放后的实际画面。

处置：已提交并部署、当前窗口已生效；不push/长期线合并，无额外安装或P3修改。只证明显示接入与布局，不宣称导航/实车闭环。
