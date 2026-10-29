# 2026-10-29 DRL 可选 nav1cm 后端验证（接口与合成运动资格）

基线 a094b90206db9c9bef2b6cf075db55cf210ccc22；专题 hk/drl-nav1cm-adapter-20261029，owner dev/drl-exploration。公共 underlay 为已审查 Task3 d1a8a0e596eaf20d2ecab84f2a4e92c0c18c410d，不整支合入开发树。未修改公共安装。后续依赖 ROOT 单独修复的 C1，产品提交 734838aa9e9f4955ee3c42bfdf9e058a6d25bb2e；C1 新 overlay 位于 task4-common-map-fix/install，不能省略或替换为旧安装。

Humble x86_64 新 DRL native 实际加载 Task2 核心 SHA b63079f301e2f1b1d6a0fc126d86eda152d98b6d30baa0b530b7da28260a9ce4；固定 block-01 Actor SHA 6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d，loader 确认 task_graph_v5/candidate_geometry_v3、128/8/6、score_bound 10。同一公共 wheel.yaml SHA 23b93ebaf83466eb51991881cab045f14a80e871811b08f6dd39a5a768d91fe6，实际净空 0.2 m。

window-01 readiness 0.139 s 因测试包装的整个 user-site 污染系统 NumPy/SciPy 而失败，零传感器帧、零运动。所有进程退出，失败保留；修复仅将缺失 typing_extensions 单文件复制到独立 Task4 deps。

window-02 在同一预声明场景/45 s/Actor 下运行：12 m 平地、0.2 m 源格、真实 360°/3 m 传感器，起点 (0.1,0.1,0)。Runtime 实际 canonical v5/a2 40 候选，Actor 选 (0.1,1.9,3.92699)；当前 UUID e5fae21808d84c99b1cd949d5fd6b742，Task 接受 yaw -2.35619（等价归一化）。44 次正式 PLAN_FOUND/0/has_reference=true，但局部求解反复 QP error 3，保持 WAITING_FOR_PATH。186 sensor 帧/709 observed cells，180 processed stamps，map revision 恒 1；实际距离和转角均 0。45.346 s 后当前 Action 返回 CANCELED_STOPPED，速度 0，无 inflight，4 子进程 exit 0，全部公共工作线程退出。故只证明真实固定 Actor 接口与取消，物理运动与新观测推进验收失败，不声明到达、覆盖或策略效果。

测试执行审计必须随结果保留：初次 111 PASS 包含既有 candidate_observation 的临时 ReplayBuffer 测试，readiness 客户端使用原测试域 205。扩大组 227 PASS/10.23 s 误纳入 frontier_identity 与 arrival_contract 的临时 learner/optimizer/replay 测试；ControlledEnv runner 实际 1 条经验、0 次更新，另两个测试各一次 learner.update，全部在 /tmp/pytest-of-kai/pytest-481（47 MB）。未经过批准的执行范围不能用通过数掩盖。目录已原样封存、未删除；没有真实 ROS 训练环境或正式恢复切换。正式账本未打开，未做其前后字节证明。

后续外部 guard 尝试：第一次在 ROS launch_testing 自动扫描时拦截 15 个未选文件的训练导入；关闭自动插件后 187 PASS/1 项在 replay 导入前被拦截。此记录不替换原始 227 或 205 事实，覆盖域的guard结果也不冒充原 default 域行为。最终自审另有两项新增纯配置/逻辑 RED→GREEN：新后端 resume 身份与传输拒绝分类；未再运行 ROS/全组。

原始日志、声明、安装身份、测试 node IDs、失败成本与完整命令在外部 Task4 制品和 task-4-report.md。正式冻结 manifest（79 文件）原字节 SHA 00bf70663cc394768d8ba5fc262a2ce56cdb76c5a84905f911aca8c62586c5cb 不变；仅本专题开发 manifest 更新至 87 文件且 not_deployed。未修改冻结 portable YAML、Actor/config 或公共安装。

C1 原因与边界：原 P3MapStore 近整数行事件舍入制造 y=-0.795 m 的 UNKNOWN 缝；初始净空 0.885858 m 小于不变的 0.921722 m 硬圆要求。ROOT 修复行枚举后净空 2.487929 m，恢复伪 UNKNOWN，保留真实 UNKNOWN。无 Actor、目标或物理限制调整；旧 window-02 的失败不会被覆盖。

产品源码 9149f3c9ab080addfde3b9dcc1338c16be43472b 先提交再跨平台构建。build-c1/install-c1 只构建 DRL（4.99 s）；实际 87 个安装 Python 文件逐字等于该提交与开发 manifest。实际 C1 p3_map.py SHA 4a296477d09585563d39275b987ae1d65c4b4d7f45d8680d38472a38fc726831，核心仍 b630；默认 config_record 与 resume_semantics 对 BASE 动态比较相同。旧 window-02 属历史未提交 source-overlay，不能回填为此提交运行。

window-03 在同 fixture/45 s/只读 Actor 下：声明 SHA 94b94b6bc5a9786b3c97f591fed8ea526c9bbb384722d5a1a8f3196d51158227。Actor 自然选原目标 (0.1,1.9,3.92699)，当前 UUID 91018b8a619c418d91ffcd6568de9b8b；44 次正式 0/PLAN_FOUND/has_reference=true，map revision 1→9、181 processed stamps、188 sensor 帧、709→957 observed cells。实际距离 1.62924466 m、累计转角 7.05257719 rad，无碰撞；命令最大线速 0.19960842 m/s、角速 0.85601674 rad/s，实测两轴加速度不超过 0.5（浮点舍入误差 4e-16）。45.851884 s 后当前 Action outcome 5/CANCELED_STOPPED，零速、无 inflight、4 子进程 exit 0、PID 无残留。

这通过固定 Actor 的新后端对接、真实规划/运动/观测推进与取消清理资格。45 s 内未到达：终点 (0.0830565,1.7250257,2.7548041)，xy 距目标约 0.175793 m，yaw 未对齐。CANCELED_STOPPED 不是导航完成；不声明任务覆盖、策略效果或性能优于旧后端。

legacy 8 s 首窗 INPUT_UNAVAILABLE、无目标，8.272434 s 后 3 子进程 exit 0。根因是验证脚本对旧输入硬编码 map 帧，而既有 RosEnv 与 C++ 旧接口要求 odom。仅验证脚本提交 744ee2bb 改为 legacy-only odom（恒等 TF 下物理量测不变），旧产品默认未改。新声明的 legacy-window-02 同 8 s：只读 Actor 自然选 (1.1,1.7)，当前 UUID 2af823023dba486eab4fa1b6f8057b2b，与旧 PathReference/TrackingStatus 一致；距离 0.986545 m、875 observed cells，8.861201 s 后 outcome 5/CANCELED，实测停稳、无 inflight、3 子进程 exit 0。此为默认兼容 smoke，不作后端性能比较。

上述所有窗口的 Actor/config/wheel 前后 SHA 不变。保留所有失败窗口和误选测试；未重跑成功窗口或未变全组。完整 93 文件原始收据归档 receipts-final.tar.gz SHA d6c9494eb16851d7ea650de0e7262c8fe8fce077755edf26a8b32e408237f4b4。未及时抓到 window-03 活进程 /proc/maps；已具备实际预检导入/加载 native 身份、冻结启动组合与运行日志，不把事后静态安装核对冒充全部运行进程映射。

状态：已提交待 ROOT 独立审查/待合入 dev/drl-exploration，not_deployed。仅有 Humble x86_64 隔离合成闭环证据。Jazzy、Humble 容器、Orin、实车、跨机 DDS、rosbag、正式训练及策略效果均 NOT_RUN。专题/远端独立 source/build/install 与证据保留待审查，未获准删除。
