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
