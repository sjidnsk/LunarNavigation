# 2026-09-23 足式 P3 接口核对与 P4 适配

## 范围与发现

用户授权读取 192.168.100.237 的 P3 源码并适配 P4，明确禁止修改 P3 文件。
基线为足式分支 5eff5aee，部署目录 /home/yanfa/P4。轮式分支未修改。
只读 P3 路径 /home/yanfa/P3/lidar_visual_fusion；start_dog.sh → hardware237.yaml → fusion.launch.py。
发现真正的部署差异是 DDS：足式 P3 输出域 69、传感器域 29；原 P4 两硬件脚本域 59，其他脚本依赖外部环境。
P3 高程 GridMap、正式 Odometry、静态 map→odom 的话题/消息/坐标系/QoS 已匹配 P4，不需改消息或绕过 Guard。
P3 elevation_only 不提供旧 global_overview，P4 现有高程派生探索地图链符合该边界。

## 实现

新增共用 p3_environment.sh 和本包 CycloneDDS 配置，七个正式入口统一加载域 69。
显式 ROS_DOMAIN_ID、P3_DOG_DOMAIN 和 CYCLONEDDS_URI 覆盖均保留；移除硬件脚本里的轮式域和绝对 DDS 配置依赖。
保留静态 TF 原值/时间戳转发及里程计独立超时。未更改导航算法、能力上限、控制速度或 P3 代码。
完整契约、启动步骤和旧终端环境纠正在 ../P3_LEGGED_INTEGRATION.md。

## 验证

证据：/home/kai/CodexDownloads/lunar_navigation/p4-p3-adapt-20260923/

- 实际执行脚本并拦截末端 ros2/flock，核对默认域、P3_DOG_DOMAIN 及显式域/DDS 覆盖：RED 19 failed / 2 passed → GREEN 21 passed。
- 加上已有启动/配置/接口回归：70 passed；外部接口边界检查通过。bash 语法与 UTF-8 检查通过。
- 本机 Jazzy 现有地图适配器原生测试：5 passed，包含列主序、坐标、缺失层和 NaN。P3 消息生成源码与解码索引逐项比对，不宣称真实数据通过。
- 目标 Orin 域 69 只读发现检查：本次 3 s 窗口内未发现地图、里程计、/tf 或 /tf_static 发布者（remote-publishers.json），没有为测试启动 P3。
- 11 个相关 P3 源码/配置/消息文件的 SHA-256 与读取快照完全一致（p3-source-hashes.json）；所有远端写入仅 P4 或临时验证目录。
- 同步后目标 Humble/ARM64 环境烟测通过：实际 source 共用环境、Humble 与 P4 overlay，域为 69；新 DDS XML 可创建临时检查节点，GridMap/Odometry/TFMessage 和三个 P4 包均可解析（remote-environment-smoke.txt）。未启动导航、探索或控制器。
- 用户已在目标机运行的 P4 编译未被打断。之后观察到该进程结束及 install/setup.bash 生成；本轮只改启动脚本、网络配置和文档，不重编译算法。

实时地图/位姿样本、正式规划与到达、真实底盘执行仍 NOT_RUN。P3 run_state.json 为历史状态，不作为当前节点运行证据。
