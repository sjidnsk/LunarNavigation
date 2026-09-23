# 高分辨率搜索优化与丰富回归验证

日期：2026-09-23。工作树：app/orin-legged-vehicle。
性能测量基线：dfa3d819d2e44145346959e729389d3757db9379；优化提交：f88f3824。
整理提交前足式已更新至7c0eb97b（滚动规划），本节历史性能数据不改写；新基线的部署同步验证见末节。
实施范围来自用户确认：足式边缓存、请求内重复查询、轮式掩码与捷径空间查询；要求充分测试。

## 最终结论

三项优化已实现。保持原碰撞模型、UNKNOWN/盲区阶段、代价、路径认证和控制接口。
最终5700对基线/优化结果完全一致；本轮另有空间索引消融和执行中取消/超时验证。
两轮完整ABBA矩阵均显示：5 cm足式中位数减少约42%～49%，轮式减少约37%～44%。
绝对耗时存在批次波动，不能将一个批次的绝对数值作为Orin性能保证。

## 实现与回归中修正的方案

1. 足式无界unordered_map边缓存改为64槽定长缓存，完整key匹配后复用。
   碰撞只影响命中率，不会复用不同边；命中前仍检查取消/截止。
   最初完全去缓存触发已有 AdjacentGoalIn320By320WindowOnlyEvaluatesExpandedStateWork
   测试失败：9次认证超过原有8次上限。最终保留近期边复用，原测试未放宽。
2. 新增仅属于一次Plan的RequestCellCache，按需分配256格页面，独立缓存Source、Cost、
   ElevationRange。避免短路径先扫描全窗口；缓存不会跨请求或地图修订复用。
   盲区override随view绑定，Normal不能返回StartPrefix。窗口外高程仍按原base查询，
   不能与窗口外Source=UNKNOWN、Cost=0混淆。足式和轮式搜索均接入。
3. 轮式后处理保留Build已生成的allowed掩码用于supercover访问，并用BVH查询边界距离。
   保留long double原始线段距离公式、supercover覆盖及原净空接受条件；包围盒只做下界剪枝。
   起点阶段与正常阶段各自构建，构建和查询中均检查取消/截止。

代码集中在core三个源文件和两个内部头文件，不改变公开ROS消息或碰撞/能力配置。
性能试验阶段没有修改P3或其他部署分支。后续按用户授权提交并同步通用优化，详见末节；未推送或部署，主线保持不变。

## 最终性能矩阵

Ryzen 9 9955HX、本机x86_64、Release/C++20，CPU6串行。
地图22.2×22.2 m，3个随机迷宫、连续折返、封闭终点。
0.2/0.1/0.05 m，native各自能力/细图、matched相同轮式细图，两平台分别测试。
每个版本每组合10次，采用baseline→optimized→optimized→baseline四块，每块正式5次、
每组合另预热一次。复用规划器对象；计时包括请求缓存、索引构建、搜索、后处理和Plan收尾，
不含预先派生地图和结果序列化。每版本600次，最终矩阵合计1200次。
P95取nearest-rank；下表仅汇总四个可达场景，每格40次，不把无路场景混入。

|分辨率|平台|中位数ms 基线→优化|减少|P95 ms 基线→优化|
|---|---|---:|---:|---:|
|0.2 m|轮式|5.30 → 3.95|25.4%|9.16 → 7.42|
|0.2 m|足式|5.72 → 3.50|38.7%|7.42 → 4.57|
|0.1 m|轮式|18.06 → 12.19|32.5%|24.69 → 17.86|
|0.1 m|足式|29.56 → 18.31|38.1%|40.72 → 24.17|
|0.05 m|轮式|80.59 → 50.90|36.8%|102.15 → 69.99|
|0.05 m|足式|159.05 → 81.05|49.0%|219.64 → 104.38|

最终每版本480次可达均找到路径，120次封闭终点均无路，无非预期超时。
最终5cm轮式后处理中位数32.05→25.63ms；足式1.05→0.59ms，各自分项取中位数不能简单相加。
原生足式更小的包络会探索更多区域；不能以原生总耗时推断两算法本质效率。

前一轮完整同条件矩阵也保留在initial-matrix/：
5cm足式119.93→70.01ms（41.6%），轮式80.77→45.46ms（43.7%）。
最终轮次足式159.05→81.05ms（49.0%），轮式80.59→50.90ms（36.8%）。
未测量CPU频率/温度变化的归因，不将波动归结为特定原因，也不跨轮配对计算收益。

## 正确性与覆盖分层

|层级|规模|检查与结果|
|---|---:|---|
|Release完整core CTest|20目标、274用例|全部通过|
|ASan+UBSan完整core CTest|20目标、274用例|全部通过，启用泄漏/错误终止|
|CSV对照工具自身测试|5用例|正确性字段改变、漏样本/重复样本均拒绝；单独耗时变化允许|
|缓存独立契约测试|3用例|负索引/跨页、缺测、override阶段、独立快照、窗口外高程|
|空间索引独立测试|4用例|15000次与穷举距离对照，含反向、大坐标、内部点、空集合和中断|
|随机查询差分|4800对，9600次运行|状态、expanded、最终到达标记、原因、raw/final路径完全一致|
|台阶/沟隙完整地图链|300对，600次运行|20场景×3分辨率×5次；状态、地图统计和路径一致|
|迷宫完整地图链|600对，1200次运行|三分辨率、两地图模式、两平台状态和路径一致|
|空间索引单独消融|400次运行|保留其余优化，索引/线性距离查询路径一致|
|执行中中断探针|400次运行|确认expanded>0后，均按预期Canceled/Timeout且不返回部分路径|

上述主差分为5700对、11400次；表中的缓存/索引用例已包含在274个core用例中，不重复累加。
早期试跑和initial-matrix重复轮不纳入最终主差分数量。
路径比较是完整hexfloat位置、四元数和phase串，包含raw_path及最终path，并非仅哈希或路长。
统计evaluated_transitions不要求一致：定长缓存淘汰可能重新认证，实际展开节点和路径要求一致。

随机查询工具使用受控的细图/高程快照，隔离测试搜索，并非所有场景都经地图派生。
覆盖10个种子、8类场景、3种分辨率、连续两次地图/代价变化、两平台、五类请求：
正向、反向、非格心短终点、预取消、预过期；四种最终朝向，含负格索引及负tile。
场景含平地、非均匀代价、随机障碍、未知区、台阶高差、起点盲区、封闭终点、通道关闭。
每版本2340次成功、540次无路、960次预取消、960次预超时；无路含主动构造的不可达情况。
这些预停止请求不代表执行中中断覆盖，执行中行为由单独探针及可注入时钟单测验证。

台阶/沟隙20场景经完整高程→细图→规划链，每版本155次成功、145次无路，与基线完全一致。
仍不能跨未认证沟隙；没有通过放宽UNKNOWN或几何阈值制造更高成功率。
具体场景见2026-09-23-legged-step-gap-benchmark.md。
已有core用例还覆盖区域目标、终点朝向、阶段切换、连续端点和安全净空。

## 索引单独收益与取消响应

5cm消融对照保留定长边缓存、请求缓存和allowed掩码，只替换BVH/原线性边界查询。
四个可达native迷宫中，轮式后处理中位数分别：
37.76→34.75ms、16.16→14.61ms、24.70→22.32ms、33.58→22.67ms，减少约8/10/10/32%。
聚合中位数24.70→22.64ms；总耗时52.36→50.75ms。
这是独立批次，不能与完整优化表直接相减；构建索引的成本已包含。

中断探针在调用开始约5ms后从独立线程发取消，或设置5ms截止。
所有样本都断言expanded_states>0，从而证明不是尚未搜索就退出。
优化版取消后返回延迟最大：轮式0.099ms、足式0.267ms；
截止后返回延迟最大：轮式0.022ms、足式0.195ms（向上取整）。
轮式取消P95从0.033升至0.073ms，超时P95从0.0026升至0.0156ms，
请求缓存清理等也在返回时间内；记录绝对增量，不宣称每项延迟均减少。
未做实时调度证明，以上不外推到Orin实时上限。

## 内存、短路径与反例检查

/usr/bin/time -v测整个基准进程的峰值RSS，包含两种派生地图、规划器和输出缓冲：
迷宫基线最高55768KiB（54.5MiB），优化49300KiB（48.1MiB），降低约11.6%。
随机快照13636→13568KiB；台阶11984→12192KiB，增加208KiB。
这些不是单个缓存对象的独立内存测量，也不代表所有地图内存都下降。

短路径控制组的中位数改善；尾部存在微秒级波动。第一轮5cm轮式短路径P95
0.1327→0.1361ms，保留该反例，不笼统宣称所有小场景P95均改善。
最终轮次的完整分组在validation-summary.json中，短路径/取消/超时分别统计。

另在仓库外构建两个故意错误的变体：缓存总返回UNKNOWN、边界距离总返回0。
新增测试均能识别并失败（mutation-checks.json），未把变体应用到生产源码。
首次去缓存的失败及修正记录在计划文件中；没有放宽已有正确性/性能断言。

## 审查与证据边界

独立只读审查未发现阻塞级正确性问题，核查了缓存生命周期、phase不可回流、
完整key匹配、mask索引边界、原long-double距离公式及包围盒下界。
审查指出随机取消样本是预停止；已明确区分，并补充执行中400次探针。

这些属于本机核心库证据。ROS Action planning_outcome/PLAN_FOUND/has_reference、
Humble、Orin、真实P3 bag、DDS、导航/探索控制闭环、实机实时性能均为NOT_RUN。
未改变碰撞模型和能力配置；未实现跨沟动作。通用代码随后同步至轮式/TCP部署分支；未合入主线。

## 复现与文件索引

本轮外部证据根目录：
`/home/kai/CodexDownloads/lunar_navigation/search-optimization-20260923/`。

- baseline-core/为改动前源码快照；baseline动态库路径见manifest.json。
- build/、sanitized/分别保存Release与ASan+UBSan构建，ctest-*.log及test_results保存完整结果。
- prepare_benchmarks.py生成maze/steps测试程序；capabilities.hpp由已核对的部署YAML生成。
- tools/benchmark_search_regression.cpp为可提交的随机查询差分工具，需用C++20链接对应core库。
- run_validation.py按CPU6串行运行；以LD_LIBRARY_PATH选择基线/优化库。
- tools/compare_search_optimization.py ROOT校验完整样本与路径，再生成validation-summary.json。
- prepare_index_ablation.py/run_extra_validation.py、prepare_cancel_probe.py及extra-validation-summary.json保存消融/中断证据。
- mutation_checks.py保存故意错误变体的测试结果；formatter/仅是外部开发工具，不参与产品依赖。
- manifest.json记录源码/库哈希、编译器与环境；所有构建、CSV和大产物均未加入仓库。

构建示例（在本工作树根目录；路径保持独立）：

```bash
source /opt/ros/jazzy/setup.bash
cmake -S ros2_ws/src/lunar_incremental_navigation_core -B /home/kai/CodexDownloads/lunar_navigation/search-optimization-20260923/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/kai/CodexDownloads/lunar_navigation/search-optimization-20260923/build -j 8
ctest --test-dir /home/kai/CodexDownloads/lunar_navigation/search-optimization-20260923/build --output-on-failure -j 1
python3 -m pytest -q tests/test_search_optimization_comparison.py
python3 tools/compare_search_optimization.py /home/kai/CodexDownloads/lunar_navigation/search-optimization-20260923
```

性能复现前须核对manifest的基线库哈希、部署能力文件和原始迷宫map文件，避免把不同版本误作配对。

## 2026-09-23 工作区整理与部署分支提交

用户明确授权将合适部分提交至对应部署分支。提交前发现三个部署分支已增加滚动规划：
足式7c0eb97b、轮式5141684e、TCP dbef580d；均予保留。
通用核心优化、内部缓存/索引、测试与对照工具共11文件单独提交，并用cherry-pick -x同步。
6份本次方案/试验文档仅整理到足式分支；能力、传感器、控制参数等配置没有跨平台带入。

|部署分支|通用优化提交|重新构建后的core测试|随机查询与原基线对照|
|---|---|---|---|
|足式|f88f3824|21/21目标，284用例通过|4800条完全一致|
|轮式实车|6acb99b8|20/20目标，272用例通过|4800条完全一致|
|TCP仿真|4e97ce33|20/20目标，272用例通过|4800条完全一致|

三个分支均另通过5项Python对照工具测试。三次独立构建均为本机Jazzy/Release，
本节验证没有重跑旧性能矩阵或ASan，不把前文dfa3d819基线上的耗时和ASan结果改记为新基线结果。
新基线查询对照共14400条，比较状态、扩展数、到达标记、原因及完整raw/final路径。

证据目录：`/home/kai/CodexDownloads/lunar_navigation/search-optimization-deployment-sync-20260923/`，
含before.json、common-files.json、sync-validation.json、独立构建/CTest日志和三个4800行CSV。
核查三个配置目录tree hash保持不变、原滚动规划提交均为新HEAD祖先、每个优化提交仅11个指定文件。
集成主线仍为6868236f，未合入任何部署分支；没有推送远端或部署到Orin。
