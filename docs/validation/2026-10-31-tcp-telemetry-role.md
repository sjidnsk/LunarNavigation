# TCP遥测职责收敛（2026-10-31）

任务TCP-TELEMETRY-ROLE-20261031；分支hk/tcp-combined-deploy-20261031，基线23aeea7，产品737658a（含beef177、21fef99）。仅修改P4；共享基础地图暂停、P3与车辆模型未修改，长期线未移动、未push。

持续armed不再用遥测包接收龄决定park。两个入口复用/car/ue_connected（RELIABLE/TRANSIENT_LOCAL/depth1），真实断连及控制指令超过0.6秒仍停车。初始或重新arm需不超过1秒的遥测；measured_stopped仍使用小于0.5秒的有限实测速率，不放宽停稳证据。遥测仍用于观测与停稳，不成为每周期行驶许可。

实际已发布BRAKE在锁外记录既有事件和有限快照，日志异常不影响发布，不改算法。首轮复审发现REFERENCE_REJECTED遗漏，737658a已修复并覆盖真实coordinator→runtime发布路径；成功发布、拒绝发布、诊断异常三分支均回归。最终独立复审无阻断。

## 验证与部署

- 本机遥测职责57项、日志/任务25项通过；目标机Humble环境遥测职责57项通过。隔离测试初次缺原依赖文件而失败，补齐P4既有依赖后通过，没有改产品迎合测试。
- 216新隔离Humble planner构建通过，串行11组CTest全部通过；未用本机纯测试代替目标机包测。
- 真实DDS只读确认/car/ue_connected收到True，发布者lunar_car_node。
- 旧operator31078先SIGINT，measured_stopped=true、navigation/rviz退出0；逐项备份并同步三个必要源码文件，新operator37288、控制域10。P3保持28980运行，原目标位置5.2410430908203125/4.095983028411865、yaw1.1590999840656853重新发布，重新规划而非续用旧轨迹。
- 运行目录/home/yanfa/P4/run/tcp-interactive-737658a-20261031，overlay /home/yanfa/P4/artifacts/telemetry-role-install-20261031，回执/home/yanfa/P4/artifacts/telemetry-role-deploy-receipt-737658a.json，备份/home/yanfa/P4/artifacts/pre-telemetry-role-20261031。仅写P4。

## 现场结果与未决项

35秒采样：693条遥测最大接收间隔0.578秒；340条控制/转发最大间隔分别0.227/0.330秒。仅一次auto、无park，重复遥测龄停车本轮未复现。旧20秒样本9次park均由0.511–0.575秒遥测龄触发；两次采样起始状态不同，不能据此宣称完整运动性能改善。

353条执行参考中228 BRAKE、123 FOLLOW、2 INVALIDATED；340条指令257零速，说明主动制动仍在。新增实际发布日志明确出现ACTION_REJECTED:MODE及LOCAL_REJECTED:BUFFER_BRAKING_RESERVE。保存的完整日志快照613个BRAKE周期中598含ACTION_REJECTED:MODE、282含BUFFER_BRAKING_RESERVE；此快照覆盖范围长于35秒，不能混为同一分母。

源码证实一种结构冲突路径：完整轨迹允许先真实减速到零再进入主mode，但action_prefix只截一个执行周期且沿用主mode；若截取终点尚未到零，独立certify找不到零速节点会以MODE拒绝。现场尚缺fallback前旧候选mode、初始v/w、前缀末速和完整候选首零时刻，故不能认定每个MODE均由此分支造成。新spin候选、旧buffer候选、最终fallback的reverse分别属于不同对象，不能混同。BUFFER_EXECUTING是前缀认证前状态，也不等于动作已经执行。

本轮只删除重复遥测龄门禁并补真实诊断，未删碰撞/运动能力认证，未改BRAKE算法、缓冲制动余量或车辆摆轮。自然到达、长距离绕障和平顺性仍未验收。原始证据保留在上述运行目录motion-after.jsonl和runtime/navigation.log，本地ignored artifacts/motion-after-737658a.jsonl、navigation-737658a.log及telemetry-role-analysis.txt。
