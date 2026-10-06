# 2026-10-06 可信度滤波移除记录

## 结果与范围

用户明确要求“既然滤波没有收益就去除滤波”。在 `dev/terrain-confidence` 专用工作树执行，
以 `db3833f9` 为修改前基线；恢复滤波加入前 `adef7f6d` 的行为。

- 移除中值、双边滤波实现、类型、配置校验、缓存身份和估计器额外滤波处理。
- 移除 ROS `terrain_filter_method` 参数和探针的 FILTER 参数。
- 移除滤波专用单元/集成测试、十五组合实验工具、滤波 P3 工具和滤波报告生成器，共 17 个专用文件。
- 11 个修改过的原有代码、构建、测试和探针文件与 `adef7f6d` 逐字节一致。
- 保留 baseline、weighted_plane、mls、grouped_ransac、gaussian_process 五种拟合方法，默认 baseline。
- 原始观测保护、连续可信度、危险判断和四种候选拟合的回归测试保留。

删除范围仅为本轮新增的滤波代码与专用工具。保留历史滤波设计、实验报告及其 Git 历史，
并加上已移除说明；共享 PDF 渲染工具保留。以下冻结产物目录未修改：

`/home/kai/CodexDownloads/lunar_navigation/terrain-filter-comparison-20261005`

历史报告内源码路径对应旧提交，不应当作当前可运行配置。需要追溯时可读取旧提交或归档
`implementation-a1d24bfb.tar.gz`、`analysis-p3-db3833f9.tar.gz`。

## 重新验证

本机 ROS 2 Jazzy，Release，BUILD_TESTING=ON；使用全新的隔离构建目录：

`/tmp/terrain-filter-removal-20261006/{core,ros,install,quick}`

日志：`/tmp/terrain-filter-removal-*.log`。

- 核心包重新构建、安装通过；串行 CTest 26 个测试程序、358 个用例通过，无失败、错误或禁用。
- ROS 包重新构建、安装通过；串行 CTest 13 个测试程序、86 个用例通过。
  首次运行漏掉 ROS 安装步骤，节点测试找不到包索引，且沙箱限制网络接口；补齐安装后，
  通过允许 localhost 通信的本机测试重跑全部 13 个程序通过。未修改源码以规避失败。
- 原五种拟合方法对比探针重新编译通过（既有缺省成员初始化警告保留）；quick 矩阵 13 个输入、57/57 次运行完整完成，freeze_mismatch 为空。
- 原对比工具的协议与分析测试 10 项通过。
- `rg 'terrain_filter|terrain_sample_filter|TerrainFilter' ros2_ws tools` 无匹配（退出码 1）。
- 11 个原文件与 `adef7f6d` 精确比对通过；`git diff --check` 通过。
- 静态接口检查：33 通过，1 个既有失败。执行
  `python3 -m pytest -q tests/test_external_interface_contract.py tests/test_isolation_contract.py tests/test_incremental_navigation_message_contract.py tests/test_incremental_navigation_interface_contract.py`。
  失败为 `test_stack_config_launch_and_cpp_keep_the_exploration_map_contract_closed`：断言地图发布器源码只含一次
  `create_publisher<nav_msgs::msg::OccupancyGrid>`，实际两次。
  该测试与 `incremental_map_publisher.cpp` 均与 `adef7f6d` 完全一致，本次未修改。

## 验证边界

本次验证移除后的构建、配置接入及算法回归；不重新解释历史实验数值。
Humble、Orin、DDS 设备联调、真实雷达回放、多帧观测和车辆闭环均为 NOT_RUN。
只更新可信度开发分支，未合并、推送或部署；集成主线及三个应用分支未同步。
