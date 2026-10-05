# 可信度机制迁入开发分支

日期：2026-10-05。目标为 `dev/terrain-confidence`，初始流程提交 `c1963c29`，从
`integration/pure-planner-orin@6868236f` 建立。用户要求将现有可信度代码迁入，先在专题分支
集成测试，合格后再同步主线和应用。

## 迁入范围与来源

来源 HEAD：足式 `0209ca23`、实车轮式 `c4866408`、TCP 轮式 `17ebae41`，以及三个工作树的
未提交可信度改动。迁入前保存 HEAD、状态、完整差异和未跟踪文件哈希，源工作树只读。

1. 必要核心依赖单独提交为 `9589002b`：从 `767ac450`、`ab6dfdbd`、`5eff5aee`、
   `a5a27303`、`407292ae`、`f88f3824`、`64057f2c`、`e7f7996c` 精确迁入 core 路径。
   包括地形/边界评价、足式背景平面、查询缓存、物理尺度窗口和增量影响范围。
2. 迁入足式来源的完整可信度核心增量与四个新增公共文件，含连续证据分数、观测估计器、
   有效高程快照、路径代价及路径简化、起终点/支撑检查和回归测试。
3. 三来源公共估计器和路径代价帮助文件字节一致，仅保留一份。足式来源测试覆盖轮式来源全部
   用例，并额外覆盖足式行为。未启用的轮式应用 legacy 足式实现未覆盖公共实现。
4. ROS 接入连续可信度参数，以及所需物理尺度窗口参数和启动校验。迁入相关默认值、覆盖、
   非法输入与运行中禁止修改测试。通用 navigation 配置默认 `confidence_weight: 2.0`。
5. 首次 ROS 回归暴露 `767ac450` 配套流水线漏迁，随后精确迁入同提交的
   `elevation_pipeline.cpp` 和 `elevation_pipeline_test.cpp`：原始地图仅扩边而无 dirty 高程时
   仍更新派生；地形回归使用足够大的地图，能验证 FREE/BLOCKED 引导变化。
6. 三份历史实施计划、七份历史验证/审查记录原样迁入；各应用专有配置增量保存在
   `docs/terrain-confidence/application-adapters/`，对应补丁与来源差异逐字节核对。

未整体合入应用分支，未迁入控制部署、滚动规划 session 或车辆运行参数等无关改动。原应用
分支、集成主线指针及三个来源工作树改动保留。本机代码迁入不代表可立即发布。

## 来源一致性与测试

16 个可信度核心来源/测试文件与足式当前来源逐字节一致；CMake 的差异仅排除滚动 session
模块与对应测试。ROS 参数按集成基线适配，保留其原有平台选择和调用流程。

本机 ROS 2 Jazzy，Release，仓库外独立构建与安装目录。核心：23 组 CTest、331 个用例全部
通过。依赖迁入阶段的 22 组核心测试也通过。ROS 最终 13 组 CTest、85 个用例全部通过，
合计 416 个 core/ROS 用例零失败。修正所需流水线依赖后的日志为 `test-ros-final.log`；首次
失败保留在 `test-ros.log`。核心和 ROS 测试结果 XML 中的 failures/errors 均为 0。

静态接口/launch/隔离检查为 67 项通过、1 项失败。失败项为
`tests/test_incremental_navigation_interface_contract.py:223`：旧断言要求整个 publisher 文件
只有一个 OccupancyGrid 发布器，当前基线已有探索图和局部细图两个发布器。在干净的
`integration/pure-planner-orin@6868236f` 工作树复现同样失败；相关 publisher 和测试文件未被
本次修改。本次保留失败并记录，不能将静态检查写成全通过，主线同步资格仍需处理此项。

独立只读审查核对了核心迁移、平台用例、依赖、ROS 参数、配置补丁和历史文档，未发现迁移
阻断问题；补入的流水线两文件另行审查，与来源逐字节一致。审查结果不能代替运行验证。

## 固定 P3 单帧请求

用迁入后的库重新编译既有固定请求探针，读取此前保存的 320×320、5 cm 原始 float32 高程，
沿用同一足式能力和配置。起终点无直接高程，但有效高程可用；核心 `kPlanFound`、16 个路径点、
`reaches_final_goal=true`，无需起点临时假设。整机 UNKNOWN/BLOCKED/FREE 为
56,450 / 44,290 / 1,660。该结果来自当前包含局部突变保护的代码，不套用历史阶段的地图计数。

这是核心单帧离线请求，位姿与高程时间不同步；没有运行 ROS action、控制器、同步 bag 回放或
实车导航。无回波窄坑仍可能被周边平面估计补平，可信度不是通行概率，不能作为安全认证。

## 证据与重跑入口

产物目录：`/home/kai/CodexDownloads/lunar_navigation/terrain-confidence-migration-20261005/`。

- `source-baseline.json`、三个来源 `.patch`、`source-untracked/`：迁入前只读来源快照。
- `*-core.patch`、`pipeline-prerequisite.patch`：按路径提取的依赖。
- `source-equivalence.json`：可信度核心文件一致性。
- `configure-*.log`、`build-*.log`、`install-*.log`、`test-*.log`：独立构建与回归，首次失败保留。
- `fixed-probe`、`fixed-request.txt`、`fixed-map/`：迁入后固定请求和逐格状态/可信度输出。

从专题工作树进入，`source /opt/ros/jazzy/setup.bash` 后，以本目录的 `core/`、`ros/` 和
`install/` 为独立通道构建；ROS 配置显式指向该安装目录的 core CMake 包，以及本机已有的
Jazzy `lunar_planning_msgs`。运行 ROS 测试时使用 `ROS_DOMAIN_ID=224 ROS_LOCALHOST_ONLY=1`。
源码变更后重新构建再分别运行两个构建目录的 `ctest --output-on-failure`。

提交前 `git diff --check` 与暂存差异检查通过，全部修改的文本按 UTF-8 读取通过。再次核对
三个来源 HEAD、状态、完整差异和未跟踪文件哈希均未改变，各应用配置补丁仍与来源一致。

## 合格边界

算法已迁入开发分支，设备资格未达成。Humble、Orin 性能与稳定性、DDS、同步回放、TCP
动力学和车辆闭环为 `NOT_RUN`。本次不合入主线或应用分支，不推送、不部署。
