# P3 历史单帧滤波补充实验

已完成：在主矩阵20752次及串行计时405次完成并收到启动通知后，本轮45次真实核心探针
严格串行执行完毕，exit=0、complete=true。五种方法的15次none与旧P3地面/状态/评分/路径
及旧语义summary逐位一致；十五组合各三次的输出逐位稳定。实际链接库、输入、探针、
模型、profile和驱动冻结前后相等。运行日志及最终JSON在
`/tmp/terrain-filter-comparison-20261005/p3/results/`，完整日志在上一层`run.log`。
本目录不在两套主实验source_hash的工具根中，未修改冻结的核心源码或库。

旧baseline的1660 FREE、44290 BLOCKED、56450 UNKNOWN、16个路径顶点全部复现。
本帧36次补丁结果NOT_NEEDED、9次RANSAC结果UNRESOLVED，READY为0；45次assumed_cells
和path_assumed_points全部为0。因此接受滤波补丁进入规划请求的验证状态为NOT_RUN。
内部拒绝分支没有插桩，不能根据UNRESOLVED推断patched evaluator的内部执行覆盖。
GP两种滤波在本帧从NO_PATH变为PLAN_FOUND，RANSAC三种设置均NO_PATH；没有地面真值，
以上不代表精度或安全改善。独立解释及最终证据索引见artifact中的`review.md`和
`execution-provenance.json`。

历史只读来源：
`/home/kai/CodexDownloads/lunar_navigation/terrain-fitting-comparison-20261005/p3`。
fixed_compare.cpp和能力文件从该目录复制。capabilities.hpp保持逐字节一致，SHA256为
`b85130422802738980dac769cc09c79c8b7e6373027951d77004abf2d6c3ce30`。
原始320×320、5 cm、小端float32输入复制到本轮artifact目录，409600字节，SHA256为
`6ca99369ee0173930d4e5375e56d43ad61c89fffb65b0e8cbb045d5e5f74f780`。
该帧17438格有观测，84962格缺测，原stamp=1789097424.9736662。

全部十五组合使用同一能力、profile、起点和终点。起点
(7.6133745667,7.9915921022)，yaw=-3.022577；终点本地图X正方向1 m。
包络0.68×0.33 m，背景半宽.2 m、准入分数.2、残差.03 m、confidence_weight=2，
起点盲区余量3 m，核心规划统一5秒截止。path.csv保持原来的列和本地图坐标，不添加
(+2.2,-3.25)展示变换，以便none路径文件与旧结果逐字节比较。

新驱动接口：

```text
fixed_compare INPUT_F32 OUTPUT_PREFIX METHOD FILTER [REPEAT=0]
```

METHOD仅接受baseline/weighted_plane/mls/grouped_ransac/gaussian_process，FILTER必须显式为
none/median/bilateral；非法名称拒绝。FILTER传给profile和独立地面估计器。
起点补丁支撑平面拟合及原始样本过滤仍共用baseline，但patched FineCellEvaluator按当前
method/filter进行整机复核；其中假定高度可能作为输入参与滤波。记录assumed_cells、补丁
状态、路径来源和端点情况，不把起点补丁的假定格当作真实测量或普通地面补值。

```bash
p3_repo=/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/terrain-confidence
p3_artifact=/tmp/terrain-filter-comparison-20261005/p3
p3_core=/tmp/terrain-filter-comparison-20261005/core
g++ -std=c++20 -O3 -DNDEBUG -Wall -Wextra -Wpedantic \
  -I"$p3_repo/ros2_ws/src/lunar_incremental_navigation_core/include" \
  -I"$p3_repo/ros2_ws/src/lunar_incremental_navigation_core/src" \
  "$p3_repo/tools/terrain_filter_p3/fixed_compare.cpp" \
  -L"$p3_core" -llunar_incremental_navigation_core -Wl,-rpath,"$p3_core" \
  -o "$p3_artifact/fixed_compare"
```

等两个正式阶段完成后串行执行。选择新的results目录，运行器不覆盖旧产物；异常保留全部
原文件及complete=false，无自动挑选最好重复。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 "$p3_repo/tools/terrain_filter_p3/run_p3.py" \
  --probe "$p3_artifact/fixed_compare" --input "$p3_artifact/raw.f32" \
  --output "$p3_artifact/results" \
  --main-artifact /tmp/terrain-filter-comparison-20261005/full \
  --timing-artifact /tmp/terrain-filter-comparison-20261005/timing
```

运行器严格验证主/计时完成状态与数量、实际链接库及全部模型源码一致。输入、库、探针、
模型源码、能力、profile和P3源码冻结；每次调用前及完成后复核。本次45探针串行，每组合
独立新进程三次；重复顺序轮换。每次输出完整stdout/stderr、地面候选、状态、评分、路径
和JSON。不能把同一帧的45次运行称为45个独立场景。

none的15次输出与旧P3逐份比较ground.bin、fine.i8、confidence.f32和path.csv字节；旧语义
JSON全部原字段按规范化字节比较，只有时间和重复ID排除，新增诊断单列。十五组合的三份
新产物也分别逐字节一致，去掉时间/重复ID的新JSON全部字段需一致。任一冻结、none对照
或重复稳定性失败阻断complete=true并保留反例。

正式输出为manifest.json、none-parity.json、repeat-artifact-equality.json、
comparison-summary.json、changes-vs-none.json、analysis.json和analysis.md。
每组合的example保留实际FREE/BLOCKED/UNKNOWN、候选/规划补值、补丁/假定格、端点评分、
核心枚举、原始reason_code、非空reference与路径来源。核心status=0加非空路径才输出
plan_found；local_status_name=PLAN_FOUND是该核心枚举的文字表示，未伪造ROS Action原因。

map_ms为实际全量构图；start_patch_ms独立计时且不包含在5秒核心规划截止时间内；plan_ms
为核心规划；fit_and_export_loop_ms包含独立拟合及状态计数读取，不能称为纯拟合时延。
三次样本P95不代表长期尾延迟，后台负载仍未隔离。

历史位姿比高程帧晚约2.2秒。没有独立高度真值，不报告RMSE、错误FREE或安全改善。
状态/补值/高度改变仅说明本帧软件行为。路径计数只有导出顶点，不能证明整段扫掠支撑。
本机核心请求不代替同步回放、ROS Action、Humble、Orin、真实雷达或闭环实车验证。

轻量协议自检，不运行真实P3：

```bash
cd "$p3_repo/tools/terrain_filter_p3"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_p3_protocol
```
