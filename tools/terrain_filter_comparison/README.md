# 加入滤波后的冻结对比实验

本目录比较五种拟合后端与三种样本滤波的十五种组合。滤波为 `none`、`median`、`bilateral`，
参数在看结果前固定：物理半径0.10 m、空间标准差0.075 m、高度标准差0.03 m、至少3个
真实邻点。滤波在当前局部拟合窗内对真实样本高度执行一次。滤波不会生成额外观测点、不会
写回原始图；原始高度区间、突变、支持及可信度证据继续用于检查。

输入和解析真值复用已经完成的旧产物：
`/home/kai/CodexDownloads/lunar_navigation/terrain-fitting-comparison-20261005/full`。
旧目录只读，新程序实际重跑所有组合，不把旧输出充作新的 `none` 结果。
原manifest保留旧执行时路径，新协议根据案例名称解析durable目录内文件，再核对原SHA256。

## 构建与运行

先在专用开发工作树以Release构建支持 `terrain_filter_method` 的核心包，再重新编译探针。
探针的位置与原构建说明相同：`tools/terrain_confidence_comparison/probe.cpp`。新增末尾参数：

```text
probe INPUT_F32 OUTPUT_PREFIX N RES METHOD fit|integrated [CONF_WEIGHT=2] [MIN_CONF=.2] [MAX_RESIDUAL=.03] [FILTER=none]
```

原调用省略末尾参数时仍为 `none`。主对比固定接受分数0.2、残差0.03 m、评分代价权重2。
方法与滤波经估计器和平台profile传递；集成阶段使用足式、轮式实际通行图和核心规划器。

所有源码、脚本、输入与实际共享库冻结后再执行：

```bash
terrain_source=/home/kai/CodexDownloads/lunar_navigation/terrain-fitting-comparison-20261005/full
terrain_new=/tmp/terrain-filter-comparison-20261005
terrain_probe="$terrain_new/probe"

python3 tools/terrain_filter_comparison/runner.py \
  --source "$terrain_source" --probe "$terrain_probe" \
  --output "$terrain_new/quick" --suite quick --workers 4
python3 tools/terrain_filter_comparison/runner.py \
  --source "$terrain_source" --probe "$terrain_probe" \
  --output "$terrain_new/full" --suite full --workers 4
```

支持 `--phase fit/integrated/all`、`--resume` 和 `--max-cases` 有限冒烟。
workers可以设为1。各探针只读同一输入并使用独立前缀；只有主线程写manifest与运行记录。
组合顺序按案例索引轮转。线程池只负责调度，每次估计/规划仍在独立新进程中执行。

主矩阵的并行时延只保留为诊断，**不作为算法性能结论**。并行CPU竞争也可能影响统一5秒
规划截止时间。若任何平台返回TIMEOUT，原结果保留；主阶段结束后将该输入的全部十五组合
串行复核，结果另存 `timeout-rechecks`，不会悄悄替换并行输出。
分析另输出 `recheck-platform-results` JSON/CSV，列出同组合的parallel/serial状态、到达和
规划成功，以及串行地图和路径危险诊断；它们不进入主150案例口径。没有TIMEOUT时保留
空JSON、CSV表头和零计数摘要。

正式完成要求：全部预期探针有有效身份和完整二进制尺寸、冻结复核通过、`none`结果与旧
地面及通行状态二进制逐位相同，以及所需串行超时复核执行完成。任何 `none` 差异独立保留，
阻断 `complete=true`。路径与时延受运行环境影响，因此位级对照不比较路径或summary文件。
中断或有限冒烟保留 `complete=false`。失败日志保留，续跑不混用新源码、库或参数。

## 运行量

| 阶段 | 输入记录 | 组合 | 探针运行 |
|---|---:|---:|---:|
| 拟合 | 1232 | 5拟合×3滤波 | 18480 |
| 集成扰动 | 150 | 5拟合×3滤波 | 2250 |
| 集成全观测软件参考 | 22 | 仅baseline+none | 22 |
| 正式主矩阵 | 1404 | — | 20752 |
| 冒烟 | 9拟合+2集成扰动+2参考 | 同规则 | 167 |

输入覆盖原11种地形、12种观测模式、5个固定种子及5/10 cm两分辨率。
遮挡五种子相同、条带只有两个相位；仍单列唯一输入SHA数量，运行次数不是独立样本数量。
拟合全观测运行十五组合；只有集成全观测软件参考仅运行baseline+none。

旧6932次串行主矩阵约27分钟。十五组合串行仅按倍数约81分钟，实际滤波增加计算。
workers4理想下约20分钟，但滤波开销、后台负载与任务调度会改变耗时；初步估计30–90分钟，
应以新冒烟和正式进度修正。这个估计不是Orin实时性证明。

## 独立重复计时

主矩阵和超时复核结束后，才执行405次串行计时；必须使用同一程序与核心库。

```bash
python3 tools/terrain_filter_comparison/runner.py \
  --source "$terrain_source" --probe "$terrain_probe" \
  --output "$terrain_new/timing" --suite timing --workers 1 \
  --reference "$terrain_new/full"
```

预先选定9个旧输入：5 cm拟合平地的缺测50%、0.5/1.5/4 cm噪声，凸面的缺测50%和1.5 cm
噪声，岩块离群点；5 cm集成缺测50%平地；10 cm集成隐藏15 cm沟遮挡。每组合新进程重复
3次。输出逐次时间、平均、中位、样本P95及最小/最大。三次样本P95不能代替长期尾延迟；
背景负载未隔离，本机结果不能外推到Orin。
计时参考必须是全部20752次完成的 `full/all` 主矩阵，且没有冻结或执行失败。脚本核对同一
程序、核心库、源码，以及九个输入与真值SHA；同时记录参考manifest的SHA并在计时结束
再次核对。完整冒烟不能充作正式计时参考。

## 指标与公平性

`fit-results`保留每个案例的缺测接受覆盖率、高度误差、有用/有害补值、梯度、评分、方差与
后备比例；`scene-results`按方法、滤波、地形、模式和分辨率分层。主要分母始终是缺测目标。
`platform-results`保留实际FREE/BLOCKED/UNKNOWN、独立解析危险诊断、全观测软件对照、
路径≤1 cm段内采样和规划状态。平台汇总排除仅baseline+none运行的密集参考。

比较每个方法的滤波与其自身 `none` 时同时提供：

- 各自覆盖、误差和错误率，防止通过拒绝更多位置获得漂亮RMSE。
- `filter-common-support-results`：同拟合方法、同输入下滤波与none共同接受的缺测格。
- `all-fifteen-common-results`：十五组合都接受的缺测格；每个方法对自身none比较。
  共同集合可能只剩容易位置，空集合保留计数0和null误差，不能写作零误差。
- 五种子簇配对区间：逐案例差先按种子平均，再重采样10000次；随机案例才进入区间，
  排除密集、重复遮挡和条带。各自接受误差与共同目标误差的区间分别报告，不能混用。

另保留观测格指标，区分“降低已测高度的噪声”和“补未知位置”：
`observed_estimate_accepted_rate`、`observed_estimate_rmse_m`与
`raw_observed_rmse_same_accepted_m`。后两项比较每个组合自己相同的已接受观测位置；不同组合
接受集合仍可能不同。这些格子的原始危险证据没有被替换，不能据此宣称已全面清理传感器噪声。

评分仍为证据估计，方差仍为局部模型条件诊断。解析危险掩码与密集基线是不同口径，均不是
车辆安全认证。核心 `kPlanFound` 和非空路径只证明本地核心请求结果，不代表ROS Action、
同步传感器、Humble/Orin或车辆执行；未执行层级标为NOT_RUN。

## 协议与分析测试

```bash
cd tools/terrain_filter_comparison
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest -v test_protocol_analysis
python3 analyze.py "$terrain_new/full"
```

测试覆盖主矩阵数量、唯一输出、轮转、固定参数、重复计时、none位级变化检测、滤波身份、
共同目标与空集合、观测格同位置对照、排除确定性重复的配对种子统计。
