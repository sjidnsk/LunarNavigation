# 全局按需边代价首批优化

任务 GLOBAL-GRAPH-REUSE-20261102，owner integration/pure-planner-orin；ROOT在TCP专题整合，基线0edfaa4f。实现来源0f08904a、30e5ee14，对应应用提交748fbc64、0eee3749；Smac对照74321468对应a23dd110。

## 改动及职责

保留候选节点、连接和连续硬碰撞认证，只把所有连接的软净空积分从构图推迟到A*实际展开；无向共享同一次积分，并保留原connect端点方向，避免反向浮点差异。积分内和每邻边保留deadline检查。无旧图认证缓存、无新增任务实体、无局部MPCC或控制链修改。

现场重建36个障碍矩形，障碍准备没有进入主要热点。现有domain和ordinate依赖起终点，整图跨目标缓存不能直接成立。因此本批没有增加跨请求整图缓存或持久空间索引，避免只对完全相同起终点有效的冗余设计；现有分区障碍裁剪保持。若后续更大障碍集证明查询是热点，再独立实施空间索引和几何准备复用，不声称三项全部完成。

## 同机证据

/home/yanfa/P4/tmp/global-graph-reuse-20261102/request.json：3帧64m、20cm P3数据经正式地形桥和TF重建，36blocked/145free，**不是正式进程累计epoch219的精确快照**。地图起点约(-0.9067,-7.7914)，终点(-3.4456,-10.0672)。硬半径.75m、软带.5m/权重1、UNKNOWN允许不变。

旧版5次总耗时中位579.02ms（563.90–611.97）；初版懒积分213.48ms，补原方向后最终5次中位230.01ms。所有路线points与旧版完全相同，2707nodes/9147edges不变，仅84条edge积分（0.92%）。这是整体规划时间，不把延后到search_s的成本隐藏。cProfile诊断旧版约935ms包含剖析开销，不能与无剖析时间直接对比。

本机全局/平滑/启动/Smac栅格118passed；Humble隔离最终全局43项初跑42passed、1000m蛇形1项BUDGET_EXHAUSTED。后续同机串行诊断：8秒研究预算旧4.820/4.815s、新4.245/4.237s；正式5秒预算旧4.821、新4.258均PLAN_FOUND，6736nodes/20909edges/expanded2464一致。1000m拉直认证仍约3.35s，是剩余瓶颈，不能宣称全部大地图预算已解决。全one_cm初版1234passed/161skip/6本机缺native库环境KeyError，不作全绿声明。

## tcp2交接与部署

tcp2直接远端修改无本地提交，已按交接精确保存源码：去除平滑CLEARANCE_COST拒绝但保留连续碰撞；共享launch/运行入口透传global_budget_s和coverage_target；TCP配置2秒/99%。本地原有launch测试全部保留，仅追加新测试，旧平滑断言按新策略更新。P4环境与README设备专属改动保持，不覆盖。

用户授权先完成先部署：仅原子替换global_planner.py四副本（source、current/input、current/install、budget2-coverage99/install）；未改P3/Env_X。备份/home/yanfa/P4/build/rollback-global-lazy-20261102/receipt.json。哈希b924bd83eba9c72447d1ff6fe514fde32e9cc64a05521d4fd81be5976302ccf5。当前operator103386，导航launch103442，RViz103443，run/p4-20261102-224153-103386。启动正常，P3源戳推进，运行节点实读global_budget_s=2、coverage_target=.99；80*.2前瞻、预测启用、同戳拒收和既有限速保持。未代发目标，不宣称车辆闭环通过。

Smac使用官方Humble原生Node2D库，六例均成功且独立75cm连续几何通过；详见2026-11-02-smac2d-offline.md。独立进程+栅格传输及栅格近似使其不能与本算法搜索ms直接比较；不接正式控制。

处置：首批已提交并正式部署，Smac离线对照已提交；长期线晋级/广域空间索引/跨目标缓存未实施，不push/不移动长期线/不清理保留证据。
