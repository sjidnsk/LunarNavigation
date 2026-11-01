# P4 局部朝向与延迟补偿

任务：P4-DIRECT-MPCC-CONTROL-20261031 / P4-LOCAL-REFERENCE-DELAY-20261101。当前切片 b86af67，专题 hk/tcp-combined-deploy-20261031。只修改 P4；P3 / Env_X / legacy 源码只读。

## 已提交源码

- 3edf98e：15秒全局后台更新；有效旧路线保留，同几何suffix保持版本和进度，真实事件即时请求，提交异常不热循环。
- 5466d36：TCP统一角速度0.2 rad/s，正反线速各0.5 m/s，加减速不变。59项测试通过。
- aefcd48：session.tick的planning_state仅供规划，session.state始终真实P3。预测ARRIVED而实测未达时为GOAL_BRAKING，不生成到达证据。会话/任务28项通过。
- f4326ec：每预测节点按自身切线生成yaw，reverse加pi，末段融合绝对最终yaw；spin不因实时yaw变化重置目标；复用既有0.01 rad/s停稳标准。起点MPCC与终点解析转向保留各自生成器，不宣称同一算法。
- de5eafb：有界512条已发命令历史，合并等值命令，按平台加速度模型增量推进P3源时刻至当前；新P3校正；超过30秒模型窗口或历史不全返回实测，不新增停车。诊断明确实测/估计、历史覆盖和模型证明来源。
- 3b80924 + d8c8a73：独立复审发现预测漏掉底盘3秒轮角准备，会虚构尚未发生转动。只读核实legacy main.cpp 253-276及P4现有vehicle-params.yaml spin_prep_time=3。预测每条历史保存ready_s，同向幅值更新/P3校正不重开，zero及角速符号变化按既有底盘语义重开；仅影响估计，不修改发出指令。command_spin_prepare_s默认0，TCP配置3，经P4/launch/node/runtime贯通。

裁定：迟到或重复源戳不更新实测，也不刷新判活；源戳高水位仍用于持续更新检测。真正ROS时钟回退保留原完整性故障。此语义与将每个乱序包都作为定位故障停车不同，独立复审已按用户与ROOT明确设计重新核对。

## 验证

本机最终定向整合：357 passed / 13 skipped（12.87秒）。独立复审全局49通过、P4/launch99通过；局部/预测端到端120通过，无剩余阻断。

512条命令本机微基准：首次预测约0.685ms，增量平均0.107ms（早期不含spin准备修正的测量，不充当目标机性能证据）。历史扫描有界，新P3才从新基线重积分；并非新增求解器。

最终Humble两包构建成功，目标机454 passed / 16 skipped、11组CTest全部通过、域169真实DDS检查通过。第一次Humble测试442通过/16跳过、1失败为配置测试仍断言旧1rad/s，b86af67只更新该断言；50340be补齐普通转向停稳配置经worker透传，两个分层RED→GREEN及独立复审无阻断。真实acados在TCP .5/.2限制下spin-left、spin-right、curve、reverse-curve均生成候选且认证VALID，模型构建13.18秒；这是隔离求解证据。车辆自然到达、过冲与平顺性尚未验收。证书起点使用当前估计时，属于估计响应模型证明，不等于执行器实际状态、已停稳或实测到达。

## P3重启与可视化

首次重启/P4 run路径：/home/yanfa/P4/run/p3-user-restart-20261101 与 tcp-interactive-p3-restart-20261101。最初主窗口切新目录、monitor留旧目录导致“正在加载”占位；ROOT补齐monitor重启并在P4副本把位姿显示从旧/T3/semantic/current_pose改接/Car/T3/localization/odometry。相机6秒左右各5帧，轨迹6帧；主窗口日志Unified RViz/monitor window ready，未以该日志单独证明画面内容。

用户再次要求重启后，新运行 /home/yanfa/P4/run/p3-user-restart-r2-20261101，pipeline97238、relay97239、monitor97241、visual97244，P4 operator97240。旧目标不恢复。重启前状态保存在P4 artifacts/p3-before-restart-r2-20261101.json；全部原日志保留，P3文件未改。

ROOT执行目标机唯一部署，尚未合并/推送或移动长期线。已提交源码与文档独立记录，不以本机或固定worker的DDS结果宣称车辆到达。


## 216最终部署

b86af67已逐项核对ba99332生产哈希后备份13个必要源码/配置并部署，无未知覆盖。旧P4退出未给出有效停稳回执，切换先中止；随后107条新鲜车辆遥测确认实际停稳（末值vx=-1.275e-8、vy=3.008e-12、wz=2.166e-8），保留两份证据后继续。部署回执 /home/yanfa/P4/artifacts/deploy-local-delay-20261101.json。

当前P4 run /home/yanfa/P4/run/tcp-interactive-local-delay-b86af67-20261101，operator101783，末尾覆盖 /home/yanfa/P4/artifacts/local-delay-install-20261101。实际launch记录global_period_s=15、command_spin_prepare_s=3、P3输入和域10保持不变；profile角速0.2、正反线速0.5。未恢复旧目标。P3 pipeline97238/relay97239/monitor97241/visual97244保留运行，新源帧仍持续推进。P3源码和原配置、底盘安装及Env_X均未改。

验证小型临时产物：本机pytest自动使用C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-128、pytest-129、pytest-130及后续测试序号目录，均为测试临时文件，可在确认无测试进程后手动清理；未自动批量删除。远端构建、日志与诊断仅P4内部，不纳入Git。
