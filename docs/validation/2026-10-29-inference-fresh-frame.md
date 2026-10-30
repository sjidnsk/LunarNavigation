# DRL 推理严格新帧门槛与 r14 验证

2026-10-29；基线7432828，分支hk/drl-integration-20261029，owner dev/drl-exploration。用户要求回到导航及未通过闭环的DRL候选；论文/TARE/地形/Isaac/资源回收保留独立台账，不作为本任务前置。本轮单线程实现、独立只读审查。

## 修改及职责

InferenceRuntime原来要求新响应序号，但新响应仍能携带同一地图帧。现在沿用已有_stamp和poll_snapshot(minimum_map_stamp_ns)，请求_stamp+1；地图服务及客户端已有此下界合同。首次_stamp=-1，请求0。取消/暂停/重启的停稳确认保持独立，不因没有新地图而阻塞。时间戳不前进时等待，不新增身份、状态对象或消息字段，不改规划器、跟踪器、容差、权重或训练。

验证器只订阅已有BEST_EFFORT String mppi_diagnostics并保存独立JSONL；验收判据不动。development87清单更新runtime.py，同时补记navigation_backend.py在39c91fe已提交却漏更新的继承清单漂移；后者源码本轮未改。formal清单原字节不变，SHA256 00bf70663cc394768d8ba5fc262a2ce56cdb76c5a84905f911aca8c62586c5cb。

## 验证

- 实际InferenceRuntime类AST生命周期回归：先2失败/1通过，修复后3通过，覆盖同帧等待、新帧恢复、取消与暂停。
- 本地新帧/启动合同/闭环验收/禁止训练守卫共68通过；源码身份选择回归6通过（2未选择）。增加观测日志后相同68项再次通过。
- 本机完整runtime测试因缺lunar_drl_terrain_native无法收集，未算通过。台式原生Humble隔离环境实际runtime测试4通过。
- 独立审查发现已有后端选择测试替身不接受现有API参数；台式先复现TypeError，仅更新测试替身并断言首次下界0；runtime及该后端选择回归合计5通过。审查关闭，无剩余阻断项。
- 台式新Python overlay的87个文件与development清单逐项SHA匹配，未改原安装。复用execution-state-r2三包安装，本轮未重建，历史549包测不记作本轮重跑。

## r14真实组件固定Actor推理

声明600秒/至少6目标，实际第二目标触发TRACKING_STALLED提前结束：1自然到达、2真实决策，严格验收FAIL，取消/复位流程未走到。第一目标完成后，下一决策地图stamp前进250.045ms；这证明本次一个实际边界使用了新帧，不代表长程全面通过。

第二目标最终距任务目标0.112029358m，朝向误差2.416357rad，已停稳。停滞前诊断有32有效样本、一次加权更新，选择weighted；速度上限约0.048013m/s，测得线速度约0.000411m/s，nominal_first同样趋零。诊断distance约0.096059m是当前控制参考的距离，不能与最终任务距离混用。随后FAILED周期valid_samples=0发生在进展门禁之后，不能反推之前全部采样被拒绝。

当前证据排除这些周期的零限速和全候选无效；尚缺实际stage/end/tolerance/anchor及候选代价与权重，不能断言停车唯一最优或唯一根因。下一步围绕现有阶段解析、阶段保留、MPPI候选覆盖及加权结果收集证据，再讨论必要算法修改。旧r12的微转使平移成为5mm中间阶段是解析事实，live阶段缺日志仍不能等同唯一运行根因。

四子进程exit0，无强制退出、残留worker或训练尝试；Actor/config/platform摘要前后不变。模型、配置、数据未上传，未训练。合成观测运动反馈与真实acados/MPPI/Humble组件验证不代表Isaac、真实高程坡面、Orin或实车验收；这些均NOT_RUN。

## 保存与恢复

本地原始证据：D:/CodexDownloads/lunar_navigation/git-preservation-20261029/execution-tests/fresh-frame-r14；包含verdict、summary、events、declaration、1664行mppi-diagnostics。其余回归XML与源身份回执在同级execution-tests目录。台式运行：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-drl-inference-closure-20261029/run-r14-fresh-frame。

处置：本切片提交待合入；完整闭环仍FAIL，不晋级dev owner、主线或应用线，不push/PR。原失败和所有隔离证据保留。
