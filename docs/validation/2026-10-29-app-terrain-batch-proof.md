# APP 批量地形三态充分条件与验证边界

任务 NAV-1CM-20261026，APP owner app/orin-real-vehicle。最终修复波的 root 明确批准：bridge 可用保守充分条件提前证明原 wheel scalar 必为 FREE；完整 Evaluate 的 slope/relief/confidence 不能由近似值冒充。其余输入回退原 scalar，足式不启用证书。实际完整命令和原始输出在外部 `integration-20261028/final-whole-branch-fix-report.md`。

## 接口与不变项

`SnapshotTerrainClassifier` 是新加的三态接口，仅被私有 terrain bridge 消费。`MakeSnapshotElevationEvaluator` 仍返回原算法完整数值，旧 public 类型布局和旧工厂未改。dense 查询和所有缓存均绑定一次不可变 view，不跨 revision/源复用。最多四百万格的查询缓存，超出时回退稀疏查询；不存在将未知填成高程的路径。

changed_cells 先按行合并，再按完整 core dependency 半径膨胀，裁到真实 snapshot 范围，最后并集逐格分类。直接有限观测即使 duplicate 也继续发布，不能跳过来源权限。取消是终止性 atomic 标记；apply 每个行程/格检查，关闭者等待 owner 退出再 destroy。PersistentElevationMap::Apply、dense 准备和单格 scalar 不是可抢占任务，不能声称任意大图固定关闭时限；完整500²在途关闭有单独实测。

## 精确重复工作消除

1. 常量窗口首次仍运行原 scalar。缓存键包括相对 clipped 边界和原 float32 常量值，复用成功结果；不缓存失败。
2. 若原始整个行或列的所有可选 ElevationRange 逐值相同，轴坐标及相对窗口证明样本序列完全相同，也可复用结果。它不单独承担一般二维地图性能资格。
3. IRLS 新旧 weights 逐值相等时，fit 输入、累加顺序和结果已固定，直接停止；最终 inlier weights 未变也不用重复同一 fit。
4. 仅 batch view：全部点都是 inlier，中心和四个轴向邻点实测存在，足以证明 nearest=0、最大角间隙<=pi/2。省去 atan2/sort；缺点/拒绝点仍原排序。平方残差和 confidence 计算保持原样。

## FREE 证书的数学范围

仅 wheel、0.01<=resolution<=0.1、宽高3..4096、候选中心的完整 dependency 窗口内每个累计格有有限零宽度区间且 |z|<=16m。缺测、非零宽度区间或残差超限只否决相交依赖窗口；不会缩小halo。三个参考锚点无效及超出全局几何界限时证书停用。中心还须在几何内且四方向均有一格空间；边界和有 TerrainMeasurementsAt 覆盖的中心继续 scalar。以下只是充分条件，不能反向将失败解释为 BLOCKED。

从本 snapshot 的左下、右下、左上原端点构造参考平面 p=c+a*x+b*y；遍历所有原始端点，在残差超过4e-6*S或端点无效时记bad，用整数前缀和证明中心完整background+observation依赖矩形中bad计数为0。它不是假定地图平面，也不写回地图。令 S=max(1,参考平面各分量绝对值)，delta=5e-6*S，其中4e-6*S为已验证的端点残差界，1e-6*S为算术余量。float32端点提升到double无损；额外预算覆盖下面有界double算术，未放宽任何地形能力阈值。

### 矩形与算术界

本证书域 observation最大半径R<=30；窗口至多61²点。即使 clipped，只要中心两侧各有证据，两个轴至少各3点，采样仍是完整笛卡尔积。均匀轴采样令 covariance_xy=0，轴方差为 h²(n²-1)/12。最薄3点与最长61点的方差比不超过465；det/(xx+yy)²仍大于0.002，远大于原0.0001奇异门槛。背景窗口半径2，同理远离其1e-8判别门槛。

对 |端点误差|<=delta，OLS梯度误差按 sum(|x-mean|)/sum((x-mean)²) 估计；取保守每轴6*delta/h。任意窗口点相对拟合的残差界取 (2+24R)*delta，截断窗口中心偏离样本均值也已计入。此界有意大于最紧最小二乘界。

IEEE double单位舍入 u=2^-53。每个矩累加至多3721项，含乘加按gamma_12000=12000u/(1-12000u)计；均匀矩形的上述方差/逆矩界允许用2^17*gamma_12000*S覆盖去中心化、行列式、梯度和回代在高度尺度的误差，每个参考分量至多S，窗口原值至多3*S+端点余量，故该界小于5.4e-7*S。验证参考平面和median背景的有限次算术也在所选1e-6*S预算内。没有fast-math/reassociation开关。本项是源码手工界定与对照验证，不是机器形式化浮点证明；任一生产编译改为不满足该算术模型时须重新审查。

### 原估计器与能力判定

强制 (2+24R)*delta <0.01。于是原preferred窗口的OLS所有残差严格小于0.015，IRLS weights均为1且固定；最终所有点在0.03 inlier界内。原0.10端点、0.02 jump快速守卫、0.04中心守卫均通过。中心及四方向实测使 nearest=0、角覆盖充分；点数至少9，confidence>=1/(1+0.01/0.02)=2/3>0.20。故原地面估计必存在，不需要推断缺点，也不会走remote-jump fallback。

背景median相邻导数每轴误差<=2*delta/h；半径2下median截距误差<=9*delta，各端点相对背景残差<=18*delta。因此局部 relief 和 positive rise 均<=36*delta，背景加权拟合也远离尺度地板和奇异边界。最终原wheel slope由有效ground梯度给出，上界是 atan(hypot(a,b)+6*sqrt(2)*delta/h)。只有这个上界严格小于maximum_slope，且36*delta严格小于wheel relief及underbody rise两个能力，才返回FREE。零能力和近阈值有余量不足时回退scalar，不能被数值近似清成FREE。

该接口不返回参考平面的slope/confidence；调用者需要完整数值时必须使用旧Evaluate或完整batch evaluator。对孔洞/孤立高返回/台阶/不满足充分条件的微扰的完整halo内，仍使用原scalar的UNKNOWN/BLOCKED支持规则。旧NaN历史、source权限、P3三态合成在上层独立验证。

## 验证门槛

`terrain_batch_test.cpp` 比较旧 scalar 与完整 batch 的state和全部double，并比较新classifier的state。覆盖常量0/.3、轴向/对角斜面、边界、孔洞、台阶、孤立高返回、1e-6非均匀扰动、坡度能力附近、非零宽度区间和零能力。每次完整源码改动后重新运行相应原core及消费者套件；通过定向性能后才跑最终完整回归和实际链路。Jetson/实车/真实P3传感器和任意地形最坏耗时仍NOT_RUN。
