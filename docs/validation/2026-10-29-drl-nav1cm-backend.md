# 2026-10-29 DRL 可选 nav1cm 后端验证（中期，未完成运动门）

基线 a094b90206db9c9bef2b6cf075db55cf210ccc22；专题 hk/drl-nav1cm-adapter-20261029，owner dev/drl-exploration。公共 underlay 为已审查 Task3 d1a8a0e596eaf20d2ecab84f2a4e92c0c18c410d，不整支合入开发树。未修改公共安装。

Humble x86_64 新 DRL native 实际加载 Task2 核心 SHA b63079f301e2f1b1d6a0fc126d86eda152d98b6d30baa0b530b7da28260a9ce4；固定 block-01 Actor SHA 6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d，loader 确认 task_graph_v5/candidate_geometry_v3、128/8/6、score_bound 10。同一公共 wheel.yaml SHA 23b93ebaf83466eb51991881cab045f14a80e871811b08f6dd39a5a768d91fe6，实际净空 0.2 m。

window-01 readiness 0.139 s 因测试包装的整个 user-site 污染系统 NumPy/SciPy 而失败，零传感器帧、零运动。所有进程退出，失败保留；修复仅将缺失 typing_extensions 单文件复制到独立 Task4 deps。

window-02 在同一预声明场景/45 s/Actor 下运行：12 m 平地、0.2 m 源格、真实 360°/3 m 传感器，起点 (0.1,0.1,0)。Runtime 实际 canonical v5/a2 40 候选，Actor 选 (0.1,1.9,3.92699)；当前 UUID e5fae21808d84c99b1cd949d5fd6b742，Task 接受 yaw -2.35619（等价归一化）。44 次正式 PLAN_FOUND/0/has_reference=true，但局部求解反复 QP error 3，保持 WAITING_FOR_PATH。186 sensor 帧/709 observed cells，180 processed stamps，map revision 恒 1；实际距离和转角均 0。45.346 s 后当前 Action 返回 CANCELED_STOPPED，速度 0，无 inflight，4 子进程 exit 0，全部公共工作线程退出。故只证明真实固定 Actor 接口与取消，物理运动与新观测推进验收失败，不声明到达、覆盖或策略效果。

测试执行审计必须随结果保留：初次 111 PASS 包含既有 candidate_observation 的临时 ReplayBuffer 测试，readiness 客户端使用原测试域 205。扩大组 227 PASS/10.23 s 误纳入 frontier_identity 与 arrival_contract 的临时 learner/optimizer/replay 测试；ControlledEnv runner 实际 1 条经验、0 次更新，另两个测试各一次 learner.update，全部在 /tmp/pytest-of-kai/pytest-481（47 MB）。未经过批准的执行范围不能用通过数掩盖。目录已原样封存、未删除；没有真实 ROS 训练环境或正式恢复切换。正式账本未打开，未做其前后字节证明。

后续外部 guard 尝试：第一次在 ROS launch_testing 自动扫描时拦截 15 个未选文件的训练导入；关闭自动插件后 187 PASS/1 项在 replay 导入前被拦截。此记录不替换原始 227 或 205 事实，覆盖域的guard结果也不冒充原 default 域行为。最终自审另有两项新增纯配置/逻辑 RED→GREEN：新后端 resume 身份与传输拒绝分类；未再运行 ROS/全组。

原始日志、声明、安装身份、测试 node IDs、失败成本与完整命令在外部 Task4 制品和 task-4-report.md。正式冻结 manifest（79 文件）原字节 SHA 00bf70663cc394768d8ba5fc262a2ce56cdb76c5a84905f911aca8c62586c5cb 不变；仅本专题开发 manifest 更新至 87 文件且 not_deployed。未修改冻结 portable YAML、Actor/config 或公共安装。

待办：公共局部零运动原因由 ROOT 只读定位；legacy 真实进程 smoke 尚未运行；最后源变更的完整安装/验证待窗口许可。当前暂停新 ROS/全组执行，等待协调。Jazzy、Humble容器、Orin、实车、真实DDS网络/rosbag 与策略效果均 NOT_RUN；Humble x86_64 合成闭环已运行但运动门失败。
