# 现有导航状态与进展判断归并计划

> 执行采用superpowers:executing-plans单线程，保持同一执行链的职责一致；不新增实体或消息字段。

Goal：先核清现有转换、完成对象和计时，再消除同一职责内重复代码。复用任务GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE/EXECUTION-CONTRACT；owner integration/pure-planner-orin，工作树hk/drl-integration-20261029，基线389896d，Windows Codex单写入者。

Architecture：保持session UUID、reference_revision/digest、现有intent和状态。MppiExecutor集中物理进展锚构造、进展观测与停滞计时，StandardPathFollower集中实际路径接管后的重绑判定；任务协调器独立负责最终到达、取消和故障终结。不会把分属不同职责的安全认证当成冗余删除。

边界：不改消息/launch/配置/优化代价/阈值/计时语义，不改训练。仅两个控制器源码、一个现有回归文件及任务文档。历史r9结果不得当成本轮重测，不移动长期线、不push/merge/PR。

- [x] 整理状态转换、完成判据、进展基准与反馈归属台账；纠正旧报告中无证明就要求maneuver_id的建议。
- [x] 在原回归文件补特征回归：相同平移/旋转状态在普通与合同接管下具有同一物理进展；绑定不清钟，阶段完成不生成假进展；延迟状态错身份不接受、旧失败不复活。
- [x] 归并：MppiExecutor._bind_progress_anchor/_observe_progress保留原0.03m/0.01rad门槛与min(dt,.2)；StandardPathFollower._orient_progress_anchor转调基类，首次/退休后/不兼容接管的绑定放到_accept_pending，移除update内补偿绑定。既有制动后重绑、stage推进属于不同事件仍保留调用。
- [x] 跑受影响controller全套与任务反馈回归；静态diff检查并核对消息/参数零变化；隔离Humble包测与同Actor最多90秒闭环一次性验收。
- [x] 整理本轮验证边界与任务状态，本地显式路径提交；保留原失败与source-r4产物，不覆盖历史验证目录。

Ruling：这是行为保持的归并，已有193项相关回归和新特征回归先在旧实现为GREEN，再归并保持GREEN；不为追求RED编写绑定调用次数一类实现细节测试。若产生行为变化，先新增真实失败回归再实施。
Ruling（审查修正）：两个RED反例证明首次普通路径初始化与合同授权退休后恢复的绑定时机不同，原计划合并_accept_pending与contract update判定会改变行为。最终保留所有原转换判据，只共享锚构造和进展计量；无需新flag。r10首版FAIL保留，最终版本重新成套构建与闭环，不把前轮通过写成本轮通过。

收尾：最终197项控制器回归、33项任务/反馈/配对回归、549项Humble包测通过；145步基线同输入对照一致。r11两个自然目标与取消复位完成，但严格验收因完成后下一决策观测stamp未增加而FAIL，原判据保持。新增身份无必要性证明；现有minimum_map_stamp_ns与请求序号新鲜度的含义差异登记为后续核查，不在此次计算归并中扩大DRL修改。
