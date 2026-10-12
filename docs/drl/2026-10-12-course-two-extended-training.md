# 课程二扩大训练量

用户于 2026-10-12 明确要求继续训练并增加训练量。延续上一轮已完成检查点，新增 16384 条经验、4096 次更新；每 4096 条/1024 次更新冻结检查点并进行同一套 60 场固定评测，共四段。性能门槛按原标准记录，按用户授权完成追加预算后复核。数值/资源/运行错误仍按训练系统协议处理。

起点：257497 转移/64000 更新，原检查点 SHA `310691626d756eef3616615aeaa26aa10193cc87fe2e232783e6c73ca817e3b0`。终点：273881 转移/68096 更新。learner（含优化器与温度）、replay、schedule、RNG、collector、curriculum、counters 的校验值在准备前后逐项相等。原基准 block31 与首段检查点、评测结果保留。

继续使用三类普通场景、10–50 m 区外距离、10–30 m 回合固定传感器、五种视场含 360°、20% 10 m/90° 锚点、12 路动态环境和三类/回合均匀抽样。网络、奖励、学习率、更新比例不变。每五分钟保存，P 暂停保存、R 恢复、Q 只退出显示。

新管理目录：`/home/kai/CodexDownloads/lunar_navigation/drl-course-two-distance-sensor-10-30m-extended-20261012/managed`。默认 course_two_trial.sh status/monitor/stop/resume 已指向该目录。原运行目录保留为历史；新运行独立绑定运行时摘要。原 60 场基线结果保留原 proof，通过逐文件 SHA/actor/config/case 核对复用。

四段评测分别为 C3-block01 至 C3-block04；每段检查点 block01.pt 至 block04.pt 保留且恢复时不覆盖，gate-block01.json 至 gate-block04.json 保留判断。终端显示总追加进度、当前段与最近完整评测。每段完成后继续下一段，总预算完成后保存待复核。

回归测试：扩展预算、暂停后只完成剩余预算、历史检查点不变、未通过性能门槛仍遵循明确追加授权，先观察到两项失败，再实现；连同现有控制器、评测并行与终端统计共 37 项通过。git diff --check 通过。准备脚本完成真实检查点继承和基线缓存核对。

具体证据：新实验目录 implementation/continuation-proof.json、green.log、start-status.json 和运行中的 state/progress。启动及进度不代表学习已改善；每段采用相同固定集测量。
