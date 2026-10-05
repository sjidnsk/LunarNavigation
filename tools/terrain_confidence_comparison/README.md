# 可信度拟合方法对比实验

本目录保留实验驱动、固定平台能力、独立合成真值、扰动生成、统计分析和图表代码。
所有方法读取完全相同的 float32 高程输入；方法名称为
`baseline`、`weighted_plane`、`mls`、`grouped_ransac`、`gaussian_process`。
默认参数固定为可信度代价权重 2、最低评分 0.2、拟合残差上限 0.03 m。

这些实验检验本机算法行为，不能替代真实雷达、Humble、Orin、DDS 或实车验证。

## 构建与执行

先以 Release 构建修改后的 `lunar_incremental_navigation_core`。探针只需要该包的头文件
和共享库，使用独立输出目录，避免将构建和实验产物写入 Git 工作树。

```bash
terrain_repo=/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/terrain-confidence
terrain_pkg="$terrain_repo/ros2_ws/src/lunar_incremental_navigation_core"
terrain_artifact=/tmp/terrain-confidence-comparison
mkdir -p "$terrain_artifact"
source /opt/ros/jazzy/setup.bash
cmake -S "$terrain_pkg" -B "$terrain_artifact/core" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build "$terrain_artifact/core" -j4
g++ -std=c++20 -O3 -DNDEBUG -Wall -Wextra -Wpedantic \
  -I"$terrain_pkg/include" -I"$terrain_pkg/src" \
  "$terrain_repo/tools/terrain_confidence_comparison/probe.cpp" \
  -L"$terrain_artifact/core" -Wl,-rpath,"$terrain_artifact/core" \
  -llunar_incremental_navigation_core -o "$terrain_artifact/probe"
```

先跑冒烟，再串行执行正式对比。不要与其他 CPU 密集测试或训练同时计时。

```bash
python3 "$terrain_repo/tools/terrain_confidence_comparison/runner.py" \
  --probe "$terrain_artifact/probe" --output "$terrain_artifact/quick" --suite quick
python3 "$terrain_repo/tools/terrain_confidence_comparison/runner.py" \
  --probe "$terrain_artifact/probe" --output "$terrain_artifact/full" --suite full
```

也可以拆成 `--phase fit` 和 `--phase integrated` 两个独立产物目录。
`--scene flat convex --pattern noise015` 可选择固定子集，`--max-cases 3` 用于有限冒烟。
`--resume` 仅跳过大小与 JSON 字段均正确的完整结果。可执行文件、实际链接的核心共享库、
源码或参数发生变化时，脚本拒绝把旧结果与新结果混合，需要选择新的产物目录。
`--max-cases` 的有限运行保留完整预期数量，`manifest.complete` 为 `false`。
运行结束还会复核可执行文件、实际共享库、源码和全部输入的 SHA256；发现变化会保留结果并
记录 `freeze_mismatch`，拒绝将该产物标记为完成。

探针也可单独执行：

```text
probe INPUT_F32 OUTPUT_PREFIX N RES METHOD fit|integrated [CONF_WEIGHT=2] [MIN_CONF=.2] [MAX_RESIDUAL=.03]
```

最后两个参数只控制独立拟合阶段的估计器。集成阶段拒绝非默认门槛，地形推导保持生产默认的 0.2/0.03；
集成阶段可以改变评分代价权重，且方法选择沿生产推导接口传递。
因此若修改最后两个参数，应将其解释为拟合门槛敏感性实验，不能说集成流水线也使用了新门槛。

## 固定矩阵

正式拟合矩阵为 11 种真值地形、12 种输入模式、5 个固定随机种子、2 种分辨率。
全观测没有随机扰动，只生成一次。合计 1232 个输入、6160 个方法运行。
随机种子为 101、211、307、401、503，在查看结果前固定，不根据评分表现重新挑选或调参。
脚本记录每个输入的 SHA256，并另外统计不同字节输入数量；运行数量不等于独立样本数量。
`occlusion` 没有随机步骤，五个种子会产生相同输入；`stripes` 在所选五种子中只有两个相位。
它们用于检查重复执行的一致性，不能算作五个独立随机实验，也不进入种子簇置信区间。

| 真值类型 | 定义 |
|---|---|
| 平地 | 高程为零 |
| 三种斜坡 | 15°、25°、35°；轮式最大坡度 20°、足式 30° |
| 凸面、凹面 | 振幅 20 cm 的解析余弦曲面，解析导数作为坡度真值 |
| 台阶 | 高差 30 cm |
| 石块 | 高 65 cm 的两处圆形凸起 |
| 坑 | 深 70 cm、半径 80 cm 的截断抛物坑 |
| 两种沟 | 深 65 cm、宽 15 cm 或 40 cm |

| 输入模式 | 定义 |
|---|---|
| dense | 无缺失、无噪声 |
| missing25/50/75 | 独立随机缺失率 25%/50%/75% |
| noise005/015/040 | 高斯高度噪声标准差 5/15/40 mm，另有 50% 缺失 |
| outlier05 | 5% 观测添加 ±20 cm 离群误差，另有 50% 缺失 |
| stripes | 每四条扫描线保留一条 |
| range_sparse | 保留率随 x 从约 70% 下降到 10% |
| occlusion | 中央宽 35 cm、长 2 m 遮挡；沟场景另将所有沟底观测移除 |
| mixed | 15 mm 噪声、2% 的 ±15 cm 离群、60% 随机缺失和中央遮挡 |

拟合地图为 81×81（5 cm）或 41×41（10 cm）；集成地图为 121×121（5 cm）或
61×61（10 cm）。两种分辨率保持近似相同物理范围，但栅格采样会有变化。特别是 15 cm 沟
在 10 cm 栅格上只能占据一个完整采样列，这个差别必须作为分辨率效应解释。

集成矩阵选择 15 个地形/扰动组合，5 个随机种子和 2 种分辨率，150 个输入分别运行五方法。
另生成 22 个仅用 baseline 的全观测软件参考，共 772 次探针运行，每次包含足式和轮式两平台。
五方法总计 6932 次探针运行。方法执行顺序按输入轮转，全部串行，不进行并行抢占计时。
正式矩阵的完整性以 `manifest.complete=true`、`completed_runs=expected_runs` 为准。

## 输出与指标

每次运行生成 `-ground.bin`，每格六个 float32：估计高度、证据评分、x/y 梯度、预测方差、
是否使用后备拟合。集成模式另生成足式/轮式的格子状态与评分、路径 CSV、构图/规划时间和
规划状态。拟合时间仅包含完整地图估计调用，不包含二进制文件写入；构图和规划时间分别记录。
路径规划有 5 s 截止时间，超时必须单独分析。

`analyze.py` 生成 JSON、CSV 和三张图：

- `fit-results`：逐输入的接受覆盖率、高度 MAE/RMSE/P95/最大误差、解析梯度 RMSE、评分排序、
  高分大误差、后备比例和耗时；边界梯度不定义的格子不计入梯度误差。
- `scene-results`：按方法、地形、扰动、分辨率分层，避免合并平均掩盖遮挡或沟边失效。
- `confidence-bins`：评分分箱与误差。评分≥0.6作为固定“高分”诊断门槛；0.6不表示60%安全。
- `common-support-results`：每个方法与 baseline 都接受的缺失格子，高度误差在完全相同目标上比较。
- `all-five-common-support-results`：五个方法同时接受的缺失格子；`summary.all_five_common_support`
  汇总共同目标误差。这个子集可能更容易，必须与每个方法自己的覆盖率一起阅读。
- `platform-results`：FREE/BLOCKED/UNKNOWN、独立解析危险区域被标为 FREE 的数量、全观测
  软件参考的 BLOCKED→FREE 数量、路径长度与危险采样。路径段内按最多1 cm间隔采样，不能
  只检查路径节点。
- `summary.platform_aggregate_excluding_dense`：平台汇总排除仅 baseline 运行的全观测参考，
  保持五方法相同的扰动案例数量。
- `summary.paired_seed_bootstrap`：方法减 baseline 的逐输入差值先按种子平均，再将五个
  种子作为簇重采样 10000 次，输出95%区间。这个区间表示种子间敏感性，不把相邻格子视作
  独立样本，也不能外推到未覆盖的月表或传感器条件。其统计单位是逐案例宏平均，可能与表中
  按格子数量加权的总体误差不同。
  区间仅包含随机扰动模式，排除 dense、固定 occlusion 和重复 stripes；这些确定性场景仍
  单独保留误差、覆盖率和危险诊断结果。
- `paired_common_support_seed_bootstrap` 和 `all_five_common_support_seed_bootstrap`：分别在
  两两共同接受、五方法共同接受的目标上计算配对误差差值区间。前述各自接受格子的 RMSE
  差值区间与这些共同目标区间含义不同，不能混用。

接受覆盖率的分母为所有缺失目标；“误差≤3 cm且已补值”的恢复率也以所有缺失目标为分母。
因此少补值不能靠排除困难格子获得完整的优势结论。条件 RMSE 只对接受格子计算，拒绝不会
当作零误差。高分大错同时给出高分格子总量，不能仅比较百分数而忽略样本数量。

独立危险诊断根据真值坡度、相邻高度跳变和平台外接圆范围计算，与拟合评分、生产地图输出
相互独立，但它只是一种保守诊断：没有模拟脚落点、步态、轮胎接触、动力学、跨沟动作和身体
姿态。因此这些计数可以暴露估计穿过隐藏危险的现象，不能直接当作车辆事故概率。
全观测 baseline 图是软件行为参考，同样不能充当物理安全真值。

GP 的方差区间覆盖率检验的是当前局部曲面模型条件下的不确定性，不能反映模型之外、完全
没有观测到的沟。即使区间很窄，也不证明地面存在或足够承重。

## 协议校验

```bash
cd "$terrain_repo/tools/terrain_confidence_comparison"
python3 -m unittest -v test_protocol test_analysis
python3 analyze.py "$terrain_artifact/full"
```

校验覆盖输入可复现性、缺失率、解析坡度、15 cm沟尺寸、遮挡保留独立真值、统计分母、
路径段内危险采样、带并列值的排序，以及种子簇配对统计。
