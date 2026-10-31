# 按实际执行职责收敛模式与制动认证（2026-10-31）

任务NAV-MODE-PREFIX-UNIFICATION-20261031，公共owner integration/pure-planner-orin，沿TCP任务复用hk/tcp-combined-deploy-20261031；基线5743d80，产品8609360。ROOT唯一部署入口，源码单写入、独立复审。长期线未移动、未push，共享基础地图暂停。

## 最终行为与职责

保留forward/reverse/spin三种规划模式。完整轨迹与短段共用真实初态、方向、非加速、无跨零违规、无路线进度信用的进入过程检查；允许合法减速或前置匀速在段末尚未到零，不要求短段独立完成整个切换。未到零的端点及制动尾按实际运动方向验证，到零后按目标模式检查。动力学、速度/加速度、地图/版本绑定、整车扫掠及完整制动尾仍保留，无新阶段实体或能力阈值调整。

应急制动认证必要性按消费者处理：生产P3 NavigationSession仅消费完整candidate/certificate并发布Path，未执行假设0.05秒action；MPPI从实测状态生成实际指令并验证响应延迟/持令/制动全过程。生产移除无对应执行对象的短动作构造及认证否决；协调器内部need_action默认True供离线动作执行，生产session显式False。取消仍从当前状态及地图构造并完整认证真实制动候选。未修改MPPI、车辆模型或P3。

## 回归、复审与目标机

初稿7e95868通过98项但独立复审发现两例完整VALID/短段MODE：先匀速后减速；控制a=0而中间申报速度有原动力学容差内误差。初稿未部署。8609360撤销新增的严格净降速门禁，两个反例均完整VALID/短段VALID；真实碰撞、动力学失败和离线动作损坏仍拒绝。

最终本机163 passed、4 skipped；独立复审APPROVED并在冻结代码复算上述两反例。216隔离147 passed、3 skipped，Humble planner新包构建通过；11组CTest均通过，其中一组初次缺隔离目录内p3_input_timing依赖、补齐原有文件后单独重跑通过。早期隔离测试另有漏传基准模块及旧测试夹具造成7失败，上传当前版本测试/原基准后通过，未为适配测试修改产品。原始日志保留P4/log/mode-prefix-*。

## 部署

四个产品文件certification.py、coordinator.py、async_coordinator.py、p3_session.py逐项核对旧737658a基线；两文件原始SHA差异仅CRLF/LF，规范换行后内容完全一致。旧operator37288先停车，measured_stopped=true，navigation/rviz退出0。备份/home/yanfa/P4/artifacts/pre-mode-prefix-20261031，回执/home/yanfa/P4/artifacts/mode-prefix-deploy-receipt-8609360.json。新隔离overlay /home/yanfa/P4/artifacts/mode-prefix-install-20261031，operator42688，运行目录/home/yanfa/P4/run/tcp-interactive-8609360-20261031；域10，原低速/定位/地图策略不变。

使用相同目标x5.2410430908203125、y4.095983028411865、yaw1.1590999840656853重新规划，未续用旧轨迹。P3未重启、其源码配置只读。现场取证在motion-after.jsonl与runtime/navigation.log；算法修复、部署与导航自然到达分开验收。

## 本轮现场结果

35秒采样251条控制/转发，其中74零速，最大间隔分别0.454/0.464秒；377条参考为315 FOLLOW、59 BRAKE、3 INVALIDATED。遥测704条，最大间隔0.603秒，仅一次auto、无park。完整日志快照82个已发布BRAKE周期，未见ACTION_REJECTED；仍有BUFFER_EMPTY64次、BUFFER_BRAKING_RESERVE27次、BUFFER_CERTIFICATION_MODE5次、LOCAL_CERTIFICATION_MODE4次、FEEDBACK_DISCONTINUITY1次，部分同周期并存，且快照覆盖时间长于35秒，不能相加或与参考数作同分母。

相同目标在不同起始位姿复测，不能宣称严格A/B性能收益；本轮证明重复假设动作门禁不再阻断生产发布。剩余缓冲与完整模式拒绝、长距离绕障和平顺性、自然到达仍未验收。未放宽阈值，未修改MPPI实际指令认证或P3。运行P4/RViz保持，供用户继续操作。原始证据本地ignored artifacts/motion-after-8609360.jsonl、navigation-8609360.log、mode-prefix-live-analysis-8609360.json及上述P4运行目录。
