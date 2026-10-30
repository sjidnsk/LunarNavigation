# TCP应用与主线差异处理

任务GIT-PRESERVE-20261029/TCP-SYNC；owner app/orin-unreal-simulation；Codex Windows单写入者。专题hk/tcp-main-sync-20261030复用现有D:/CodexDownloads/lunar_navigation/git-preservation-20261029/worktree，原治理分支90b233c保留。

## 固定基线与范围

- 应用来源2d9a8ce1620d951f110c36ab13e59d3bb478df16，接收主线90b233c410ecd317fa5a2cd0da0ad9b7a9b124c8。
- 保留TCP协议/车辆接口、OBJ仿真、P4启动与导出、地图索引加速及应用配置；同步公共导航、消息/控制配对与DRL适配。
- 九个Git冲突：README、DRL入口、当前入口、开发源码清单、course_two_parallel及两份DRL测试、phase_aware_path_simplifier、fine_traversability_builder_test。
- 本轮先处理源码差异，不启动新测试、构建、仿真或训练，不上传远端。调试机将来仅向/home/yanfa/P4写必要代码/配置/脚本，设计/报告留本地。

## 处理计划

1. 文档以主线当前入口为准，保留TCP身份、部署入口及历史来源。
2. DRL冲突采用主线完整后继实现及对应清单/测试；核对旧DomainBusy处理未丢，不改变训练运行。
3. 简化器保留应用BoundaryDistanceIndex精确查询，同时保留主线CertifySegmentWithClearance及margin；禁止用整边选择丢失任一条件。
4. 地形测试同时保留应用可见中心证据与主线投影版本隔离用例；自动合并的核心/ROS变化逐项审查。
5. 只读核对来源、清单、冲突、TCP必要文件及配置；一次独立整合审查后保存候选，不将源码静态完成写成TCP/ROS运行通过。

Ruling：TCP原净空0.4m、3m盲区及滚动规划参数属于应用覆盖，先保留；主线0.2m不得因自动同步替换TCP现场参数。能力motion_model仍接收主线差速定义。后续运行配置另按实际TCP接线确认。

## 冲突裁定与审查修正

九项显式冲突已解决。README保留TCP分支身份；DRL/当前入口、开发清单和DRL实现/测试采用主线后继版本，DomainBusy逻辑保留。简化器组合应用精确索引与主线物理净空，地形测试保留双方用例及独立函数边界。

独立只读审查另外发现三项自动合并遗漏并修正：

- 普通FineTraversability派生继续使用三参FineCellEvaluator，保留profile背景窗口与共享地形观测；只有Projected派生注入调用方证据。
- 独立terrain native桥源清单增加TCP的terrain_observation_estimator.cpp，避免依赖定义缺失。
- wheel_orin.yaml与wheel.yaml均保留TCP原0.4m净空和3m盲区；motion_model仍采用主线差速定义。

TCP deployment/p4_unreal_target、deployment/p4_vehicle_gui及OBJ仿真包共90个文件与TCP基线Git blob完全相同。DRL开发清单87个Python文件SHA匹配，formal清单不改写。上述为源码/身份核对，不是新构建或闭环证据。

## 继承缺口与下一边界

旧P4导出工具的固定runtime-files及CMake模板滞后，缺phase_motion、policy exporter与应用地形估计/索引等文件。此项作为后续部署入口适配缺口保留；本轮不调用旧导出器、不把保留原文件写成导出可构建，也不向新P4上传不可确认的旧部署包。

本轮仅保存主线同步源码候选，测试/构建/ROS/TCP仿真/部署/闭环均NOT_RUN；不沿用主线测试冒充应用组合通过。应用owner和main保持原指针，待后续P4必要代码集整理及适用构建后再决定owner接收。专题工作树保留，禁止自动清理。

三项修正的独立限定复审已通过，无剩余源码阻断；九项冲突及未解决索引均清空，差异空白检查通过。处置：本地提交保存，待应用构建/适配与owner接收，不部署、不push。

源码合并提交：ffdad931fe459a2ddb25c246f11a272dafb37a1e（双亲TCP基线2d9a8ce、主线90b233c），专题hk/tcp-main-sync-20261030；后续收尾提交仅记录本SHA，不扩大验证资格。
