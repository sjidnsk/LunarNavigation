# 全局首次请求与参考平滑性能

任务GLOBAL-PLANNING-PERF-20261101；owner integration/pure-planner-orin；专题hk/global-planning-perf-20261101；基线6979586a，复用nav-tcp-integration-20261101。ROOT负责实现/复审，无并行写入。用户允许优化效率与增加平滑预算；本轮不重启正式P4、不发目标/控制。

## 现场证据与测量边界

最新真实日志158次PLAN_FOUND、142次平滑BUDGET_EXHAUSTED、16次SMOOTHED。此前238次实际全局日志只能得到部分提交→日志结果时间，首次约2.3..2.4s；缺少worker计时，无法把这2.4s全归因到搜索或精确还原历史最大平滑时间。
被动捕获control domain10的P3地图一次，原始CDR保留P4/tmp。当前缺少可接收的map<-odom TF，原任务路径话题为空；未猜测坐标对齐、不重启P3。地图按其自身odom坐标生成29个障碍矩形；只在这份真实地图上安排6个诊断穿越查询，非原任务精确回放，不能充当现场闭环验收或历史最坏耗时。

## 原因

GlobalWorker原ProcessPoolExecutor采用spawn且懒启动，在第一目标后才创建/重新导入launcher依赖。相同地图诊断首请求810ms，queue802ms、纯规划2.9ms；第二次5.5ms。
平滑圆碰撞线段验证占带profiler示例40/47ms和51/61ms；曲线检查与最终参考检查重复调用相同线段/相同快照。带profiler的61ms不作为净耗时指标。

## 修改

1. GlobalWorker创建时提交既有进程的prepare_worker，提前spawn、导入并查找可选native代价符号；不规划/发布，真实任务仍单在途。全局启动发生在等待目标期，并非消除了总启动成本。无效协调器参数先验证，不为错误配置启动子进程。
2. 平滑与原始规划同归全局worker，内部请求携带现有goal yaw、开关与预算；保持原始路线、曲线凸包证明、目标/版本/epoch身份。主线程只接纳/当前图变化时复验；新图只挡住曲线时可保留已经重新认证的原路线，不在主线程重算平滑。
3. global_smoothing_budget_s默认0.2s（原生产25ms），通过ROS启动参数/参数文件配置，内部reference_smoothing_budget_s同源传递。全局搜索预算仍1s。
4. 同次shape_route内缓存已通过全部障碍检查的精确线段几何证明，最终检查复用；失败/未完成检查不缓存，不跨调用/地图复用。未降低曲线或采样折线碰撞检查要求，不改净空公式/权重及路径接纳准则。
5. GLOBAL_RESULT记录已有timing、queue/handoff/elapsed及worker总计算/平滑耗时；GLOBAL_REFERENCE_SHAPED记录smoothing_ms/adoption_ms。无新增ROS消息或任务身份。

## ARM64实测对照

相同冻结地图/6查询，每项5次，无profiler。原版最长尝试46.119ms，候选29.166ms（约减少36.8%）；成功平滑最长36.789→22.639ms。200ms窗内没有预算耗尽；仍有一项诊断的5次结果为CLEARANCE_COST，其余直线无需整形或成功曲线；本次未更改此选路规则。
原版冷首请求773.607ms（queue766.918ms，plan2.676ms）；候选先等待prepare完成862.574ms，再提交首请求8.706ms（含worker平滑），第二次7.770ms。此对照模拟节点先就绪再接目标；若目标立即到来，准备开销仍需支付，不能承诺首目标恒为9ms。
六项输入的raw路径及每次reference_points/接纳reason逐值一致。原版与候选查询构图/搜索同算法；一项原构图约212ms、搜索1.4ms，不用平滑提速掩盖该项构图开销。
以上不是历史运行最长时间，也不保证所有更复杂地图都在200ms完成。

## 验证与处置

Windows相关one_cm回归1244通过95条件跳过；日志字段追加回归40通过。ARM64 Ubuntu22.04/Humble候选lunar_nav1cm_planner构建成功，受影响算法/平滑/编排/ROS测试224通过；补齐新增日志与预算参数断言后45测试通过。固定边界的当前图碰撞、只拒曲线不拒原线、曲线/原路线身份不一致、精确证明复用、无效预算和后台接纳均有回归。
Humble使用既有core库，未改core/native ABI。源码/算法级与隔离ROS测试不能声称实际车辆闭环通过。已知环境SciPy/NumPy版本警告保留，未安装依赖或修改其他项目。
产物：本机D:/CodexDownloads/lunar_navigation/global-planning-perf-20261101；远端仅/home/yanfa/P4/tmp/global-planning-perf-20261101，含原CDR、冻结JSON、prof与候选构建。正式P4源码/安装/进程未改。
处置为已提交待合入；不push/PR/merge，其他应用分支未同步此候选。专题和产物保留待授权回收。


## 后续：共享1秒预算及20/10/5cm复杂场景（本节替代独立200ms配置）

按用户最新确认，搜索、构图、路线处理与平滑共用global_budget_s=1s。
worker收到请求后建立同一perf_counter截止时间，搜索接收剩余预算，平滑仅使用其后剩余预算及同一绝对截止时间；移除global_smoothing_budget_s/reference_smoothing_budget_s重复配置。
搜索已成功而平滑耗尽时保留经过认证的原路线；搜索失败不能记为平滑成功。GLOBAL_RESULT补充total_budget_s/smoothing_budget_s。
预算仍是协作式限制，不是硬实时：ARM本轮最慢整个worker约1011.45ms，超出部分含收尾和检查粒度，不能声称每次精确不超过1000ms。未增加第二段预算，也未提高总预算。

新增可复用脚本scripts/navigation/one_cm/benchmark_global_resolution.py。TCP ARM64实测4种封闭交错挡墙场景，每种20/10/5cm，各重复3次，共36次。碰撞圆0.75m、软净空范围0.4m、权重1；通道开口约4m，未使用无法通过生产车体的窄门夹具。
三档地图来自相同解析障碍的保守栅格化，并合并为相同数量稀疏矩形。本测试衡量几何量化与规划，**不衡量P3稠密格处理、原始地图传输、地图融合或实际导航闭环**。UNKNOWN允许通行，没有伪造FREE。

|场景|20cm|10cm|5cm|
|---|---|---|---|
|100m、6挡墙|3/3有效路线，最长352.94ms；CLEARANCE_COST回退原线|3/3，347.91ms；全部平滑|3/3，325.29ms；全部平滑|
|300m、16挡墙|3/3，1002.40ms；平滑全部耗尽回退|3/3，981.82ms；全部平滑|3/3，1003.66ms；1次平滑、2次预算回退|
|750m、30挡墙|0/3，预算耗尽|0/3，预算耗尽|0/3，预算耗尽|
|180m、30挡墙密集场景|0/3，预算耗尽|0/3，预算耗尽|0/3，预算耗尽|

表中时间为worker_compute_ms样本最大值，不含启动/IPC。18/36正式规划成功；有输出路线均通过独立连续线段-圆包络碰撞复核，失败18次均返回一致的无参考状态，并非18次导航成功。100m场景最终路线长度20/10/5cm分别104.96/104.44/103.93m。
复杂场景揭示构图及搜索后的缩短/代价处理仍是瓶颈（密集场景构图约0.68–0.74s，搜索约0.022s，其后耗尽预算）。不能根据小地图毫秒级结果推断长复杂地图平滑预算充足。5cm几何更细不保证整体更快；同面积理论源格数是20cm的16倍，本脚本不模拟这部分负载。

验证：Windows one_cm 1245通过、95条件跳过；TCP ARM64 Ubuntu22.04/Humble隔离包构建通过，相关算法/编排/ROS启动与日志135通过、1条件跳过。SciPy/NumPy原有版本警告保留，依赖未修改。
新证据在本机D:/CodexDownloads/lunar_navigation/global-planning-perf-20261101/resolution-long-complex.json、shared-budget-pytest.log；远端同任务tmp目录resolution-long-complex.json/.log、shared-budget-build-test.log。
正式P4进程和安装未切换，P3未修改。P3原生10cm/5cm输出及车辆闭环NOT_RUN。本轮已提交待合入，性能瓶颈保留为下一批；不得因状态一致就宣称全部场景通过。
