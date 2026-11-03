# TCP公共成果晋级与三应用同步

## 授权与范围

2026-11-03：按公共算法、公共交互、TCP专用逐批独立复审后合并，再同步轮式/足式；不重启、不切换正式P4，不push、不回收资源。
来源02b7ec45，正式payload7fcc7362；主线起点b1575f9b。以逐批文件清单JSON追溯最终树delta，避免重复重放同戳修正和参数中间版本。

## 批次1 公共算法

实现443b9eb2，含平台物理窗口、精确75cm圆碰撞、全局按需代价/认证优化、共享事实完整生产消费链、局部参考修复、计算与持续发布解耦。可选已发命令预测默认关闭；按用户最新要求只应用严格递增P3源戳。主线wheel20cm起伏能力不变，未接收TCP30cm、限速、预测启用值或正式构建脚本。

- 实施适用回归1599 passed / 156 skipped / 1 deselected；独立复审另182 passed，无阻塞P1/P2。
- 根agent补跑实际native地形/观测/窗口103/103；使用observation-only-20261103/native及exact-collision ABI3几何库。
- Humble：148个公共C++源码/构建文件SHA与正式构建逐项相同，复用同源二进制在隔离域93逐组执行：core21组通过；ROS16组适用项通过，其中关闭的legacy-debug snapshot用例明确排除，不宣称全包全绿。
- 首次planning_debug_publisher的立即DDS图查询一例失败，原参数单独重跑7/7通过，原失败日志保留，不修改产品逻辑或阈值。
- 证据/home/yanfa/P4/tmp/branch-sync-20261103/public-native-humble-results.json及同目录日志；正式active/previous未变，未启动导航/发目标/车辆控制。
- Windows缺ROS/bash/acados的用例分开记录，未假装通过；新增主线构建、车辆到达/探索闭环、轮式实车和足式实车仍NOT_RUN。

## 环境与产物

本任务工作树/外部验证产物均在D盘，ignored ledger不提交。实施agent前期默认pytest在C:/Users/77634/AppData/Local/Temp/pytest-of-77634产生测试临时文件（查到pytest-258/259/260但不能唯一确认对应轮次）；这些是pytest临时目录，测试结束后经确认无其他测试使用可以删除，本轮不擅自删除。后续均显式使用D盘basetemp。

## 批次2 公共交互

实现29c5d70d，十路径最终源码与02b7ec45一致。探索清理后端、RViz专用图层DELETEALL、默认关闭任务图和重复候选显示、odometry Keep1、已有左右相机与点云显示接线。

- 定向静态143pass/4skip/1deselect；独立复审无P1/P2，另话题13/13通过。
- 42个C++源码逐项同源，Humble隔离域93直接运行RViz3组及活动探索3组全部通过；测试日志interaction-humble-results.json在前述P4/tmp目录。
- 清理只作用探索持有的导航句柄和专用图层；相机只订阅现有话题，不新增采集节点。
- 无正式RViz界面操作/现场交互验收，不重启正式P4。

## 批次3 TCP应用

专题hk/tcp-app-sync-20261103：49c9418c合部署来源，183c865c接公共算法，23708429接公共交互，dc4585c0接最终公共验收。独立复审无P1/P2；旧TCP应用专用目录和接口保留。基础wheel配置与正式来源一致，实际速度由TCP overlay限制为0.1m/s及0.1rad/s，30cm起伏只在TCP应用保留。

Windows入口/配置/runtime187pass9skip。ARM Linux隔离源码入口112/112通过。首次111pass1fail来自测试夹具ready仅看文件存在、在JSON写完前读取；5836a8f6只等待完整JSON，不改产品逻辑/5秒期限，已独立定向复审通过，原失败日志保留。
正式active/previous保持formal-20261103-unified/formal-baseline-20261103，未部署本次Git合并、不重启、不发目标。Linux证据P4/tmp/branch-sync-20261103/tcp-entry-linux-fixed.log。

## 最终复审 fix wave：平台配置消费者

最终复审发现P1：足式launch传legged_orin.yaml，但共享地图节点硬编码按wheel加载，构造失败。公共修复efded1fc复用PlatformProfileFor既有能力变体/轮廓及净空，仅将已有platform_selector贯穿map/frontier，不新增地图实体或阈值。轮式默认及原地图权重保持。

新增真实ROS目标exploration_map_consumer_test：实际足式配置创建节点、发布facts、接OccupancyGrid；含默认wheel与非法selector。Windows launch RED4failed后GREEN84pass；ARM独立CMake构建地图库及节点成功，旧源码RED足式platform wheel mismatch，新源码GREEN3/3。首次GREEN误加载正式旧库；显式LD_LIBRARY_PATH选择隔离build并ldd确认后全部通过，原日志保留。

隔离路径P4/tmp/branch-sync-20261103/platform-map-fix，编译与测试未更改正式source/install。足式候选75389f22已接修复且独立最终复审确认P1关闭，轮式/TCP接收相同公共修复后再结案。
