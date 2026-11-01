# P4实际暖接续与2.5秒持令窗口试验

沿P4-CONTINUOUS-GOAL-ACCEPTANCE-20261101，owner integration/pure-planner-orin / app/orin-unreal-simulation；工作树D:/CodexDownloads/lunar_navigation/tcp-continuous-handoff-20261101，ROOT唯一216写入/部署入口。用户明确要求P4实际接续测试，随后要求持令认证1.2→2.5秒。没有改动P3、Env_X、legacy源码或地图共享设计。

## 部署与验证

c1df216：从7ce810f精确引入local_mpcc.py物理相位暖接续与reference_buffer.py等价投影优化，另带两份专题测试。保留551ab08终点修复，认证性能567f3a4未纳入运行包。不是把整个c1df216提交树声明成生产树：运行身份为551ab08基线+两份精确文件，源清单128项逐文件SHA核对。

- 本机105通过2跳过；独立来源复审26通过，无阻断。
- P4 Humble构建、297通过5跳过、11/11 CTest和域169真实DDS通过。
- 旧operator122384确认停稳后切为136924，运行目录P4/run/tcp-interactive-warm-handover-c1df216-20261101；仅两份生产源码替换并使用新隔离overlay。N20/.2秒离散、.1秒规划/.05秒控制、.2m/s/.1rad/s、1.2秒持令保持。

8e35ae4：只将TCP共享config/tcp_p3_feedback.yaml改为2.5秒并更新既有默认断言，不改通用传参测试的历史样例值。保留watchdog、速度、周期、碰撞/制动检查。实际运行包为上批源码+单份配置，未夹带567f3a4。

- 本机98通过；P4 Humble构建、352通过5跳过、11/11 CTest与域169 DDS通过。
- operator136924确认停稳后切为140071，运行P4/run/tcp-interactive-hold25-8e35ae4-20261101。P4仅改一份配置和使用新隔离overlay；实际参数服务读回command_hold_s=2.5，其余保持。

两次P3 pipeline均104042、同一原运行参数，未重启。只在旧任务ARRIVED时切换，未复播已完成旧目标。每轮Action前根据P3实际地图坐标/车头方向设2米目标，最大90秒；记录唯一UUID，超时仅取消自身目标；若RViz用户目标出现则保留用户操作。两轮均未超时、无用户中途抢占，Action status4/outcome0/reason GOAL_REACHED。独立全局结果也均满足planning_outcome0/PLAN_FOUND/has_reference=true。

## 实际结果

|持令窗口|样本/同目标交接|认证插零|首次起步完成|最终位置误差|最终朝向误差|认证耗时p50/p95|实际命令间隔p50/p95|
|---|---|---:|---:|---|---|---|---|
|1.2s|704/262|0|1|0.080447m|0.007917rad|35.93/54.73ms|0.0707/0.2024s|
|2.5s|252/90|0|1|0.091716m|0.005918rad|58.10/72.99ms|0.0924/0.1810s|

两轮起点、朝向、路线、地图/负载不相同，不能用总完成时长或P95差异宣称窗口变长导致更顺。1.2秒认证p50约35.93ms、2.5秒约58.10ms为本次观测，不是严格同工况微基准。两轮持令窗口均可自然到达，但未证明2.5秒改善顿挫；当前按用户要求保留2.5秒供RViz继续操作。

1.2秒：seeds nominal4/warm266，262次交接无认证插零。终点有两个GOAL_BRAKING后恢复运动片段（约3.44/2.51秒），另有GOAL_TURNING过零约.067秒；不应冒充同任务重新进入初始起步。终点之外的全局等待/初次缓冲不可用记录单列。

2.5秒：seeds nominal2/warm90，90次交接无认证插零，未观察到非零→零→再次运动片段。最终零速首次标为WAITING_GLOBAL，随后GOAL_BRAKING/ARRIVED；因此不能把所有零速称作已消除。没有ISSUED_CERTIFICATE_MISMATCH或COLLISION/BRAKING_COLLISION拒绝记录。

## 末段制动证据与认证职责

2.5秒在1793477257.235220前一条指令v=.073403、w=0且certificate VALID。下一条1793477257.264844为WAITING_GLOBAL/BRAKE_SOURCE，route_version2仍在、local_busy=true、buffer剩余0，events为空。状态估计距目标约.023m但v=.0866；代码可能出现“位置尚未满足.02m/停稳规则、路线投影已到终点、参考只剩单点”而落到默认WAITING_GLOBAL并清缓冲的边界。该代码路径与日志一致，原始该周期参考点未记录，尚不能声明已最终复现。后续只读订阅拿到的是之后的路线，旧状态投影出当前窗口，诊断失败保留，不拿它当旧事件证据。未修改阶段条件或另造阈值。

用户质疑停车碰撞检查：同一路径减速通常不增加新占用空间；当前proof却是issued响应+恒v/w持令+独立减速尾，未必沿原MPCC曲线。差速模型独立降低v/w会改变曲率。显式尾已归零时certify隐式尾通常零循环，不是实际重复停车。停车尾仅在认证数组，不会随每条cmd下发；实际只发布issued速度。

direct BUFFER_EXECUTING已跳过整候选/action完整认证；MPCC节点圆约束与真实/估计初态经slew后的连续轨迹检查并非同一对象，不能把实际运动检查一并删掉。独立停车假设是否必要应按真实执行方式审查，不能以泛化安全理由默认保留，更不能将MODE/REFERENCE_REENTRY/WAITING_GLOBAL等误写成碰撞。此次保持单变量实验，没有同时修改制动策略。

## 证据与恢复

P4/artifacts/下：warm-handover-source-manifest-c1df216-20261101.json、warm-handover-validation-c1df216-20261101.json、deploy-warm-handover-c1df216-20261101.json；hold25-source-manifest-8e35ae4-20261101.json、hold25-validation-8e35ae4-20261101.json、deploy-hold25-8e35ae4-20261101.json；warm-handover-field[-summary]-c1df216/8e35ae4-20261101.json，以及hold25-waiting-global-context-20261101.json。

实际输入SHA256：
- hold=1.2: 74652d5ffcc5ae8b04f251e8d0cdfeab2e75559ab1a0003417ff1733c8d1fc20
- hold=2.5: fede1cb24363d52822849c4113f090f892423988e968703dd9a9f12a5e9207bd

生产配置旧值备份在P4/artifacts/pre-hold25-production-8e35ae4-20261101/config/tcp_p3_feedback.yaml；旧源码在pre-warm-handover-production-c1df216-20261101。恢复应先核对用户当前任务并停稳，再显式替换需要的一份配置/两份源码和旧overlay，不自动回退用户后续改动、不批量清理。P3与原车辆接口无变更。代码/配置本地已提交待合入，未merge/push；这是TCP仿真与P3实际组件闭环，非Orin或实体车辆验收。
