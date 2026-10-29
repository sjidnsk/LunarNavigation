# 进展生命周期与参考一致MPPI：联合验证

2026-10-29，原基线83426f8，hk/drl-integration-20261029；用户授权先修锚点再改初始化/采样。公共跟踪owner integration/pure-planner-orin，推理验收owner dev/drl-exploration。未合入长期线，无push/PR。

## 最终修改

第一切片45eafd7：在锚点重绑前结转有向物理位移，复用原未用于MPPI进展计算的标量并命名_translation_credit_m；负向运动抵消，原进展门槛达成才同时清credit和时钟；clear、新session及同类复位不继承旧credit。阶段变化仍能更新进展方向，参考刷新自身不刷新停滞预算。见progress-anchor-lifecycle验证记录。

第二切片：标准Path由已有位置弧长及连续展开的车体yaw生成全时域控制种子。线速度使用已有接近/制动限速；角速度前馈来自该速度沿路径推进时的yaw差分，叠加原恒定1.5航向误差反馈。种子替换原固定seed候选槽，并用于空warm start初始化；保留已有warm start候选、原Gaussian扰动分布、importance修正、评分权重、加权更新与fallback认证。原地旋转、已到弧长末点的物理纠偏仍用原seed；曲率模式仍限幅。没有增加固定(v,w)补丁、网络、配置项、消息或领域实体，没有将路线规划下沉到跟踪器。

初版尝试的预测域反馈衰减使既有短窗边走边转回归失败，已撤回并恢复原恒定反馈；失败记录保留。原阶段屏障、5mm中间容差、0.1m/0.05rad任务容差、3cm/0.01rad进展门槛、15s缺省超时及认证安全边界均未改。尾部/真实机动屏障的语义仍留待讨论。

## 定向与包验证

- 进展六项先RED，修复后359纯控制器通过；第二切片四项先RED，补充跨pi及末点纠偏后最终365项纯控制器通过。
- 离线同帧重放：r15最后帧原最优0.586516843、停车0.605105799，新参考种子0.008362692；另外三个保留旧阶段帧的新种子0.0405/0.0449/0.0448，均低于原最优约0.29。r16新种子纯跟踪代价0.007775262，含原importance修正为-0.162442268，原实际地图与完整延迟停车认证True。这是离线候选评分/认证，不单独代表任务到达。
- 隔离Humble受影响控制器包重新构建成功，colcon汇总524测试，0错误/失败/跳过；旧消息和规划器安装复用，未重建的包不冒充本轮重跑。Windows缺ROS消息的reference/incremental_path收集失败保留，其真实环境验证包含在Humble包测中。
- 根目录推理门槛/禁止训练/诊断透明性回归71通过。独立只读代码审查无阻断；审查者本机直接测试缺生成消息，未将其计为独立通过。
- 上传源码44文件与清单逐项SHA一致，安装18个Python文件逐项一致。原安装、模型/config/platform与DRL/formal源码清单未改。

## 固定Actor真实组件闭环

|运行|声明上限|自然到达目标|真实决策|从首目标到最后结束|严格验收|
|---|---|---|---|---|---|
|r17 reference-seed|600s，至少6不同目标|6|8|226.579s|PASS|
|r18 reference-seed-long|600s，至少10不同目标|10|12|378.885s|PASS|

共16次自然到达、10个不同目标位姿、20次真实Actor决策；两个任务分别完成取消、复位、再次取消及停稳退出。每个自然完成后的下一决策使用严格前进的地图帧。两轮均从原始execution-feedback重新计算身份绑定和新鲜终端证据，并在本地独立重算grade，与台式verdict完全一致。此前r14第二目标及r12第六目标的相同目标位姿在r17/r18均自然到达；不声称实时轨迹逐帧相同。

r17位置最大误差0.099578733m、朝向最大0.048055327rad；r18分别0.097967341m、0.046466509rad。所有完成均在原容差内且实测停稳。两轮均四直接子进程exit0，无强制退出、残留worker、碰撞或训练尝试；冻结Actor/config/platform摘要前后不变。r17计算诊断p50约13.9ms、p95约61.5ms，不能据此宣称50ms硬实时保证。

## 部署与保存边界

本轮是Humble、真实acados/MPPI与固定Actor推理，使用12m平坦合成观测/运动反馈场景；不是训练、Isaac、高程坡面或实车试验。Orin/实车、真实坡面/打滑及策略泛化仍NOT_RUN。两轮PASS只覆盖声明场景与门槛，不外推所有场景稳定性。

本地证据D:/CodexDownloads/lunar_navigation/git-preservation-20261029/execution-tests：reference-seed-r17/r18原始轨迹/事件/反馈、verdict/summary/declaration、independent-grade；reference-seed-final-digest.json；reference-seed-source-r1-manifest.json与source-receipt；Humble原日志与test-results归档；纯测试RED/GREEN/XML和同帧重放。台式目录仍在原analysis-artifacts下，控制器新安装execution-reference-seed-r1、源码source-r4/reference-seed-r1，未覆盖execution-state-r2。

处置：两个修复切片及验证证据本地保存，满足本轮声明的扩展闭环门槛；待后续集成审查。owner/main/apps未移动，既有失败与隔离资源保留待后续处理。阶段尾部语义未经讨论不修改。
