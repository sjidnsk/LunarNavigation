> 历史档案（2026-10-17 归并）：来源 `fix/drl-checkpoint-memory-20261013@4c660a303c864d209e70e0d92c044886625c6531`，原专项基线 `8b0b6319598d0ef7735ae804ba9da1ce3f61a4af`。正文保留历史记录；旧 PID、路径和启动命令不代表当前运行环境，请从 [DRL 开发入口](/docs/DRL开发入口.md) 核对当前开发/部署身份。该实验已被后续版本替代的部分不作为当前协议。

# 检查点保存内存峰值修复与恢复准备

## 事故与范围

2026-10-13 09:37:41 Asia/Shanghai，第三训练段 PID 323289 被内核 global OOM 杀死。最后有效保存为 67584 次更新、271901 条转移；最后运行日志为 67653 次更新。原检查点 SHA256 `7d889ebd36918c1c76af55dd5c1b9fda95e1e36928a6cafd6f8646c5dcfef1ed` 保持不变。

工作树 `fix/drl-checkpoint-memory-20261013` 保留 position-temperature 工作树的现有未提交修改。本次仅修改 replay.py、checkpoint.py、course_two_trial.py 并新增/扩展相关测试。未合并或推送。

## 修复

- 递归编码/解码闭包通过 finally 主动释放 memo、backing 引用并打破自引用；异常路径也清理。避免多 GiB 数据等待不确定的循环垃圾回收。
- 检查点 torch.save 显式使用 pickle protocol 5，直接保存 bytes，避免默认 protocol 2 的 Unicode/Latin1 中间副本。现有 weights_only=False 加载路径继续兼容原检查点。
- capture 完成采样完整性校验后释放临时 replay；严格校验保留。
- continuation 控制器启动训练和评测前释放完整检查点对象，只保留所需计数。
- 8 GiB replay/snapshot、32 GiB 输出预算、奖励、优化器和网络配置不变。

## 验证

新增生命周期测试在修复前复现解码数组和编码 backing 滞留；binary-bytes 测试在旧协议下失败；控制器对象释放测试在原实现下失败。

相关测试最终在主机 Jazzy 环境为 151 passed、1137 deselected（41.70 s）。此前沙箱运行 142 passed、7 failed，失败来自 socket PermissionError；首次最小回归还因缺失 native overlay 失败。最终主机运行均通过。六包隔离构建通过，源码与安装包逐文件对齐；git diff --check 通过。

真实保存点含 4307 条经验、2543764831 字节快照。CPU 流程执行完整恢复、连续两次 capture + validated atomic save、重新加载，验证样本数及 learner/optimizer checksum 一致。已修闭包但仍用 protocol 2 的流程峰值为 18.516 GiB；改为 protocol 5 后为 12.805 GiB。此峰值包含加载旧协议检查点，未包含并行 ROS workers，未覆盖满 8 GiB 经验池。不能据此宣称已完成全容量压力验证或现场恢复验证。

## 部署与驱动阻塞

当前 control.sh 已切到此工作树及 `/home/kai/.cache/lunar-drl-checkpoint-memory/jazzy-20261013`。runtime 从 `a30d7b616568b2bd0089f373d8802580f65d06187f1f94fb52ae5ee480113e24` 迁移为 `010114beede6350d002134a628ff6994d908c5345766b4aed4929f743e0d6298`，保留配置、原评测、检查点和迁移备份。旧评测仍是旧 runtime 的历史结果，不是新版本实测。

驱动包、磁盘模块均为 595.99.02，当前加载模块为 595.84；系统有 reboot-required，桌面占用 GPU，sudo 需要密码。未卸载桌面正在使用的模块，未重启电脑，训练保持 paused。

实验根目录 `/home/kai/CodexDownloads/lunar_navigation/drl-region-query-bound10-20261013` 下已保存 `resume-after-driver-restart.sh`：先执行 nvidia-smi 和真实 CUDA 矩阵计算，再调用 control.sh resume。从保存点恢复仍待用户重启后完成，届时还需观察一次在线定时保存。没有注册自动启动任务。

证据：实验 managed/migrations/checkpoint-memory-20261013/，含部署备份、CPU 探针及 build/test 日志。Humble、Orin、实车 NOT_RUN。

## 后续授权容量收紧

用户确认后，将活动 arm config 的 replay_max_bytes 从 8 GiB 调至 3 GiB，snapshot_max_bytes 从 8 GiB 调至 4 GiB；总输出预算保持 32 GiB。在暂停状态和控制器锁下备份、修改配置及 identity 配置哈希。运行代码与原 resume.pt 哈希不变。

用现有 67584 更新检查点按 3 GiB 上限实际重建经验池：4307 条全部保留，bytes_used=2697350505（约 2.51 GiB），三场景采样 readiness 通过。此验证证明现有保存点能够无样本淘汰恢复，不等于满容量在线保存压力验证。记录在实验 managed/migrations/replay-3g-snapshot-4g-20261013/verification.json。训练仍暂停，等待驱动重启及 CUDA 检查。
