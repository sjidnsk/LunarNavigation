# 候选几何输入与六层决策网络

2026-10-14。用户已确认采用；可观测图和 Critic 真值图都固定 **128 维、8 头、6 层**，不安排层数对照。

模型身份为 `candidate_geometry_v3`，观测协议为 `task_graph_v5`。旧 `region_relation_v2 / task_graph_v4` 路径保留，默认配置仍指向旧模型，避免已有启动命令静默改变模型。新模型必须在配置中显式选择。

## 输入与动作

| 输入 | 形状 | 含义 |
|---|---|---|
| 真实节点几何 | N×5 | 车体坐标 XY/10、到最近任务区域点的向量/10、原图最短到达距离/10 |
| 真实节点视角 | N×8×3 | 八个物理朝向的区内前沿数/100、区外前沿数/100、实际观测历史位 |
| 朝向标签 | 8×2 | 每状态一份，相对机器人朝向的 cos/sin |
| 策略图 | E×2、E | 已知连接与边长度；进入网络时长度/10 |
| 任务多边形 | M×2 | 完整有序边界，车体坐标/10 |
| 动态状态 | 4 | v/0.2、omega/平台最大转速、range/10、FOV/pi |
| 结构索引 | N、A×2 | 真实节点掩码、当前节点、合法位置和朝向组合 |

冻结世界目标、构造 Pose、地图身份、分辨率、平台标识与传感器安装参数属于执行/协议元数据，不拼入模型输入。原图已有的零收益近邻移动动作保留；最多补充 32 个可达且能观测任务区内前沿的真实位置，各补齐八方向。先扩展动作、计算原图距离，再收缩不承担当前/动作职责且内外收益均为零的连通节点。

虚拟节点只有独立学习的连通表示，不编码占位几何/视角，不参与全局真实节点读取，不成为动作。收缩边权仍是代表点经接口的路径代价，不能覆盖真实节点上冻结的原图距离。导航使用真实目标，仍需沿物理可通行路径行驶。

## 最终结构

```mermaid
flowchart TB
    accTitle: 六层候选位置与朝向决策网络
    accDescr: 原图处理得到任务几何、真实视角和收缩图。三个独立网络使用同样的可观测编码结构，Actor 输出联合动作概率，两个 Critic 加入训练专用真值并产生价值。目标 Critic 为两个在线 Critic 的软更新副本。
    subgraph input_processing[必要输入处理]
        fine_graph[已观测原图与当前 Pose] --> candidate_set[扩展真实远端候选并计算原图距离]
        candidate_set --> policy_graph[A2 收缩图与真实节点掩码]
        task_polygon[任务多边形] --> geometry[真实节点几何 5 维]
        fine_graph --> geometry
    end
    subgraph observed_encoder[可观测编码器 每个网络拥有独立参数]
        geometry --> position[位置 MLP 5 → 128 → 128]
        views[方向观测 3 维与相对方向 2 维] --> view[共享视角 MLP 5 → 128 → 128]
        view --> pool[八方向平均]
        position --> fusion[节点融合 256 → 128]
        pool --> fusion
        fusion --> graph[六层稀疏图注意力]
        policy_graph --> graph
        virtual[虚拟连通表示] --> graph
        graph --> read[当前节点读取全局真实节点]
        task_polygon --> polygon[对称边 MLP 与平均池化]
        polygon --> context[状态融合 516 → 128]
        read --> context
        graph --> context
        position --> context
        dynamic[四维动态量] --> context
    end
    subgraph score_path[每网络内部复用评分投影]
        position --> projected[位置与图投影按候选位置计算一次]
        graph --> projected
        view --> selected_view[读取合法动作视角并投影]
        context --> projected_context[状态投影每状态计算一次]
        projected --> sum[按动作索引相加 加一次偏置]
        selected_view --> sum
        projected_context --> sum
        sum --> nonlinear[GELU → 128 → 1]
    end
    nonlinear --> actor[Actor 合法联合动作 softmax]
    nonlinear --> q[Q1 与 Q2 各输出每动作价值]
    truth[仅 Critic 训练可见的真值图 六层<br/>全局读取 候选支持关系 预计消除前沿] --> sum
    q -.软更新.-> target[对应的两个目标 Critic 无梯度无优化器]
```

图中的可观测编码与评分流程是结构定义；Actor、Q1、Q2 不共享可学习权重。真值箭头只适用于 Q 的评分，不进入 Actor。执行时仅加载 Actor。

Actor 的评分输入概念上为 `[e_i,h_i,v_ih,c]`，512 维；Q 为这些表示加全局真值、局部真值及朝向预计消除前沿，769 维。实现不物化重复的大拼接，而按线性层列块先投影、再通过动作索引复用：

`W concat(e_i,h_i,v_ih,c)+b = W_e e_i + W_h h_i + W_v v_ih + W_c c + b`

两种写法随后采用相同的 GELU 和输出层；不把每块压成独立标量，也不在每块分别添加非线性。位置、视角的基础编码只计算一次，编码结果可同时供图传播和最终评分使用。四维动态量直接进入状态融合，不再使用单独的 4→128 扩维。

实测参数数目：Actor 1,077,937，单个 Critic 1,989,729，包含两个目标 Critic 共 9,036,853（不计温度标量）。这是权重数量，不是训练内存或耗时。

## 训练兼容与范围

- SAC 的双 Q、目标软更新、联合动作分布、位置与条件朝向熵保持原机制；奖励、传感器、场景与课程不随此改动改变。
- 新旧观测禁止混入同一回放或批次。保存和恢复同时检查模型身份、观测协议、完整样本数量和内容哈希；不能通过修改外层版本号伪装旧回放。
- 旧 A2 回放不能恢复收缩前路径距离，所以新结构不导入旧权重、优化器或回放。旧模型与实验原位保留。
- 已支持训练采样侧输入构造、CPU Actor 发布/加载、双 Q 更新与完整恢复。真实部署输入适配仍按用户要求留待模型训练完成后处理。
- 当前实现不自动启动正式训练。候选正式配置仅改变模型版本与独立输出路径，其余设置来自现有训练配置。

## 验证入口

新增行为测试：`test_candidate_observation.py`、`test_candidate_model.py`、`test_candidate_training.py`。覆盖几何、收缩、方向与动作身份、稀疏批次、投影输出/梯度等价、Actor/Q 输入隔离、更新和完整恢复。

独立实现与验证产物位于 `/mnt/data/WS/lunar-navigation/analysis-artifacts/input-action-v5-implementation-20261014/`。`baseline-manifest.json` 记录本次复制保留的既有训练改动；`implementation.patch` 只包含本次增量，避免把之前的未提交修复误认为本次改动。具体测试结果和限制见该目录的 `verification.md`。分批汇总：1436 项通过、16 项跳过；包含新增 24 项行为测试。受沙箱 socket/DDS 限制的 67 项均在主机定向复验通过，旧快照格式断言已修正并通过。CUDA、隔离 Humble 仿真及完整经验恢复通过。

一次更新、随机策略走通仿真、几何梯度存在，均不构成已学会入区的证据；Orin 与实车未验证。
