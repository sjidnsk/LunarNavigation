# 公共导航离线诊断工具

本目录只包含基于当前 C++ 核心的离线对比驱动和既有输入变更/恢复契约测试。Python 仅依赖标准库；
运行环境为 Linux（CPU affinity、RSS 单位与 `.so` 约定）。不依赖 DRL、ROS 节点或车辆控制器。
`--help` 只读且不创建输出；执行对比会启动显式指定的离线基准二进制并写入指定目录。

## 构建与契约

以下在仓库根目录运行，`NAV_ART` 必须是新的、仓库外的明确路径。Humble 环境只提供 CMake/ament
构建依赖，基准自身不启动 ROS。不要覆盖旧实验目录，也不要改正在运行的安装库。

```bash
source /opt/ros/humble/setup.bash
export NAV_SRC="$PWD"
export NAV_ART=/home/kai/CodexDownloads/lunar_navigation/navigation-offline-new-run
cmake -S "$NAV_SRC/ros2_ws/src/lunar_incremental_navigation_core" \
  -B "$NAV_ART/build-core" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$NAV_ART/build-core" --target lunar_adaptive_navigation_benchmark -j2
export NAV_BENCHMARK="$NAV_ART/build-core/lunar_adaptive_navigation_benchmark"
export LD_LIBRARY_PATH="$NAV_ART/build-core:${LD_LIBRARY_PATH:-}"
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider scripts/navigation/test_comparison_contract.py
python3 scripts/navigation/compare_navigation.py --help
```

现有契约使用 `rough_cost 64 .2 13 1`，核对三种算法、两种预算中新增障碍改变输入哈希，移除后
恢复原输入；它不是到达、性能或车辆就绪证明。缺少 `NAV_BENCHMARK` 时测试明确失败，不能把
环境缺失当作通过。

## 对比驱动

需要执行基准时，把二进制及其 `liblunar_incremental_navigation_core.so` 一起复制到新的冻结目录，
再显式传入 `--binary` 和新的 `--output`。本轮整理只执行契约、CLI 与离线小型验证，不执行完整
6300 行性能矩阵、ROS 闭环、外部仿真或训练。

```bash
python3 scripts/navigation/compare_navigation.py \
  --binary "$NAV_ART/frozen/lunar_adaptive_navigation_benchmark" \
  --output "$NAV_ART/quick-results" --quick
```

`--quick` 为 12 个小场景，完整模式为 150 组输入、三算法、两预算及七请求，共 6300 行。
`pipeline` 把地图表示构建与搜索计入 500ms，`search` 在表示准备后开始预算，仍记录地图耗时。
`--cpu N` 限定当前允许 CPU；`--resume` 要求二进制及同目录核心库哈希一致。必须保持冻结文件
不变。驱动用标准库读取并核对成对输入，保留原始 JSONL、stderr、manifest 和摘要。

安全 AVAILABLE 与 NO_ROUTE、输入不足、TIMEOUT 分开解释；网格 fine A* 不等于连续空间最优。
完整对比还需要阅读底层 benchmark 的独立路线检查语义。该驱动按历史契约原样迁入；不要对不可信
日志使用 `python -O`，其断言会被关闭。当前仅允许新的输出目录；输出覆盖/续跑不提供通用事务保证。
本机 x86 离线数据不能代替 Jazzy/Humble ROS、Orin、真实 bag、动力学或实车证据。

原始 14 文件逐项审阅、哈希与未迁入理由见 [分支整合台账](../../docs/分支整合台账.md)。
