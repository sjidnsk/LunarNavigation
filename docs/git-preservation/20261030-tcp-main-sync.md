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

## 用户修订：算法与平台参数统一主线

用户随后明确：TCP应用需要的算法与主线一致，平台参数先与主线轮式一致，仅TCP协议需要专用适配。这覆盖前述保留0.4m净空、背景窗口/索引算法差异的旧ruling；前述内容作为处理历史保留。

新阶段基线b8588f9，继续同一任务/专题/工作树。对ros2_ws/src、scripts、config下与主线同路径的差异文件，精确采用90b233c版本，消除公共算法、原生桥、公共测试与轮式参数分叉；不修改main本身，不删除旧应用独有文件。旧应用独有算法辅助文件仅留历史，不进入主线CMake/source入口；若后续发现仍被运行入口引用，必须先处理接线，不能宣称运行能力已统一。

TCP车辆接口是控制/反馈报文转换与连接层；OBJ虚拟观测是可选仿真输入，不属于规划算法。旧OBJ启动器仍使用incremental_reference链，不能作为新P4的nav1cm+MPCC+标准Path MPPI启动入口。当前未部署，待明确所需输入组件后按共享主线入口接线，不能暗中启动旧链。

## 用户新增：车辆接口及OBJ均保留并在TCP机调试

用户明确这两项均需适配新算法，并授权在TCP机调试。本次新增调试授权覆盖此前不追加运行验证的限制，仅作用于本TCP任务。目标机yanfa@192.168.100.216，Ubuntu22.04/ROS Humble/aarch64，Jetson R36.4.7；原进程盘点无活动车辆/导航进程。

执行计划：

1. 完成43个公共同路径文件与main90b233c一致，wheel/wheel_orin及公共算法不再分叉；旧应用独有文件不删除，不纳入主线构建。
2. OBJ入口直接Include公共nav1cm_exploration.launch.py，桥/观测仅负责TCP和地图输入；采用主线轮式配置，不再启旧增量规划/扫描执行器；先补入口回归RED。
3. 新P4只部署必要包的源码、共享launch、配置、构建/运行/调试脚本和必要依赖；不带docs/计划/报告/论文/旧导出模板，不上传模型。旧地图从P4_legacy只读引用，不修改。
4. 全部ROS/编译/求解缓存、临时目录、日志及Python依赖显式留新P4；acados_v0.4.0及CasADi3.6.7隔离安装，不改系统环境或其他目录。主线代码保持原字节。
5. 先在TCP机完成Humble构建和本机回环TCP+OBJ+新算法调试，核对规划正式成功/动作/跟踪/停稳；不连接外部UE发送运动指令。外部真实TCP联调必须先确认对应端口/协议和输入新鲜度。

旧车辆C++接口/目标路径显示功能保留为必要协议适配源码；与Python直连桥不能争用同一车辆控制连接。目标机器只存必要代码/配置及真实调试产物，设计/报告保留本地仓库。

入口切片进展：共享Include代替旧导航及扫描执行器；OBJ已改用现有GetPolicyMap的processed_stamp确认真实输入处理，不再依赖旧诊断计数。入口与回执先3/4项RED，后15项GREEN；核心OBJ/协议组52通过2缺ROS跳过。Windows扩展组曾4项ROS/POSIX收集错误、22项失败，原证据保留；其中7项为真实NumPy标量shape写入npy头的兼容问题，已转Python整数并覆盖回归，其余留Humble核对，不称Windows全绿。

新P4隔离CasADi3.6.7、acados_v0.4.0及ARM原生Tera renderer已构建完成；官方三个子模块按v0.4.0精确提交获取。Git smart协议两次TLS失败保留，改用官方固定归档获取。Rust工具链无profile修改，Python/Rust/SDK/缓存都在新P4；尚未运行车辆/新导航闭环。

## 2026-10-30 静态地图与输入职责修复

前述 GetPolicyMap 分块回执方案已被替代：它只证明地图 owner 处理，不能证明导航/跟踪独立消费者都接收完整历史。现 nav/explore 一律对新收到的真实反馈位姿调用 TerrainMap.observe，完整发布当前局部窗口，保持原反馈时间戳，不可见区域保留 UNKNOWN；不再宣称 KNOWN_MAP_READY 或已知全图。nav 远处未知目标可以被主线拒绝，这是输入语义调整，不放开 UNKNOWN 通行。历史窗口不保证三消费者完全一致。

公共修复已由 main e9b37bb 接收：现有 p3_map_timeout_s=0 仅禁用 direct_p3 接收年龄过期，默认仍3秒；TCP静态OBJ入口显式传0。缺地图、位姿/动态输入年龄、覆盖/碰撞/revision/epoch/origin认证继续生效。三个公共 C++ 解析入口支持可选合法 motion_model，重复/非法值仍拒绝，不改车辆能力数值。44实际入口探针、3个正式gtest、2个真实ROS静态策略测试通过；完整ament包测未跑，精简部署缺其他测试源的配置失败已保留。

目标机 Humble 11包构建成功（8分41秒）；公共4包增量构建成功；OBJ新观测回归7通过，配置/共享入口9通过。所有写入在新P4，其他目录只读；无外部UE连接或训练。只有必要源码/配置/脚本、依赖及实际构建调试产物，设计与本记录均只在本地。

### 回环失败保留与当前阻塞

- r1 配置字段unknown motion_model导致地图owner退出，修复后消除。
- r2 进入新链，失败 EXECUTION_INPUT_UNAVAILABLE:stale_feedback_during_recertification，未运动、正常停稳。
- r3 独立wrapper采样暴露调试脚本只等mapowner而未等controller地图就绪，失败missing_or_stale_map。脚本现须看到controller自身有效地图后的solve_result，再发目标；退出后统一检查returncode/forced_shutdown/vehicle_errors/停稳，不能提前标PASS。
- r4 已收到3条全局路由、9条执行参考、5条匹配接收状态，但仍认证反馈超时。独立profile显示某次tick持续540.5ms、反馈年龄541.8ms，其中追加认证62.5ms超过50ms覆盖窗；地图revision=1/epoch=1未变化。单线程控制回调期间无法处理Odometry，追加认证可能追赶自身耗时。此为剩余计算/调度问题；未放宽反馈门槛，未修改公共控制算法，不能归因于静态地图接收年龄。
- 四轮均非完整闭环PASS；r4车辆位置未变、退出速度零、无TCP错误、launch返回0。原始日志P4/run/tcp-nav1cm-r1至r4与P4/log保留，不把ARM回环当外部UE物理/实车证据。

当前处置：TCP专题已保存源码待应用接收，应用owner仍2d9a8ce；公共main e9b37bb已落地。未push、不删除分支/工作树。剩余跟踪器认证计算与回调时序应作为公共问题继续定位，不能以TCP私有算法绕过。
