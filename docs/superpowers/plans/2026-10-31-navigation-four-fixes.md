# 导航四项职责修复计划

> 执行方式：使用executing-plans逐项顺序执行，沿用原任务独立工作树，单写入。

**目标：**按用户顺序修复旋转边界、统一完成条件、真实制动接转向、局部投影统一。
**架构：**复用已有有序stage、协调器结果、制动候选及局部几何缓存。无新任务/机动实体，无新计时器，无超时或安全阈值放宽。
**技术：**Python/NumPy纯算法及现有pytest；ROS/Humble/TCP验证后续执行，本轮不连接远端。
**基线：**666db9e，hk/p3-pose-wait-20261030，main-worktree。

- [x] 1. standard_path_reference拆稀疏大角使每段不超过π/2；先用半圈微反偏/跨界/顺逆转RED，再回归标准Path/执行合同。
- [x] 2. coordinator/async_coordinator提供当前认证的到达结论，navigation_task只汇总任务与执行终态；P3 NOT_DUE与输入更新不能沿用失效证据。按现有packet/result字段实现，先完整读取session/runtime生命周期；补无候选/失败/制动末态/NOT_DUE回归。DRL nav1cm消费已确认终态，不再重复1秒门槛；旧后端保持。
- [x] 3. seeds/pose_goal/local_mpcc复用实际初态制动与轨迹接续；末端微残速可生成制动+转向，保留初值、完整认证和最终姿态。先补微速顺逆/残余角速/碰撞尾段RED。
- [ ] 4. mppi_path_follower提供同次局部投影所用segment/fraction，standard_path_follower按其插值车体yaw；复用原缓存，删除全路径二次投影。补回折/跨界/倒车/滚动路径回归。
- [ ] 每项定向验证通过后保存独立本地提交并更新任务记录；最终运行受影响算法与导航/DRL适配回归，UTF-8、语法及diff检查。不开训练、不改tcp2候选、不push/merge。
