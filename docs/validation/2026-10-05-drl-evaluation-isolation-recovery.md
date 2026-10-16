# 固定评测 DDS 隔离与第9块恢复

## 问题和证据

2026-10-05 17:47:39，额外300米月表场景1已经运行；它使用DDS220和`/lunar_training/env_0`。正式四阶段课程的第9块训练在74752转移、18432更新处完成，随后固定评测也以DDS220和同一命名空间启动50米区内洞穴（种子2026100400）。两个环境的地图、里程计、动作和时钟服务因此没有隔离。

17:50:49，两者同时中断：月表场景1为下一状态`INPUT_UNAVAILABLE`，课程评测为初始化`START_OUTSIDE_FINE_MAP`。这不是策略完成率不达标，也不是任务多边形内外的判定。两份记录位于`/home/kai/CodexDownloads/lunar_navigation/drl-moon-300m-20261005/scene1/failure.json`和`/home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-30-100-bonus20-20261004/managed/course-block9-00.log`。

## 修复范围

- ROS环境启动前独占主机上的DDS域；已有新版环境由文件锁保护，旧版原生进程通过`/proc/*/environ`的ROS_DOMAIN_ID检测。明确指定的占用域直接拒绝，固定评测在220–232中自动选择空闲域。
- 租约保持到环境与原生子进程关闭，初始化异常也释放；锁文件保留，避免删除文件导致锁身份变化。
- 导航、传感器、模型输入、奖励、种子、预算及SAC采样更新配置不变。
- 监看按实际状态和有效评测数显示进行中/中断，不再仅凭存在标签宣称评测完成；保留原成功率显示。

## 验证及恢复边界

相关88项Python检查通过，涵盖域互斥、旧进程检测、失败初始化释放、监看提示、评测和课程恢复；Jazzy缓存构建完成。原洞穴种子用冻结的第9块Actor在域232中初始化成功，完成5次有效动作、覆盖率28.81%，同时外部月表测试仍占用220和225。实际尝试域220已被占用检查拒绝。独立审阅和证据位于实验根目录的`recovery-isolation-review.md`、`recovery-native-smoke.json`、`recovery-occupied-domain.json`、`recovery-build.log`。

恢复前manifest、exit、review-gate、course-state和失败日志备份到实验根目录`recovery-before-isolation-20261005/`，包含检查点、配置和Actor SHA。课程状态仅完成8块，其中区内洞穴完成5块；第9块的训练已达到边界，恢复控制器应跳过采集/更新，先补跑同一组10个固定场景。只有全部有效评测结束，才记录第6块洞穴结果并决定后续训练和晋级。

本机Jazzy验证不代表Humble、Orin或实车验证；这些层级本次未运行。原生短验证也不代表第9块完整评测已通过。
