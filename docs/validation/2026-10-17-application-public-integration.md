# TCP 应用公共架构同步验证

本候选从原应用 4273314e4bb2893c7cfd9ecb61cc71955f799cae 出发，先以
253e19f825a5e4e571f420c5ce14daf7984a8557 原样保存 26 个待提交路径，
再兼容合并已审公共输入 d452677ae0c681dc865bac35a4092beab3034f9b。
该文档记录专题候选，实际落地由主 agent 独立审查后完成。

保留 TCP 配置、TF、平台、到达阈值、控制反馈和已有 baseline confidence；
未引入开发线四种可选拟合器。公共 CertifySegment、phase/derivative/local_window、
原地支撑、取消、精确 map stamp 与 delta/full 导出契约均保留。
应用 persistent 保持完整 UNKNOWN 输入边界，native GridView 作同契约最小适配。
FineCellEvaluator 默认入口使用共享 observations；四参数自定义分类器入口保持
自定义对象优先且显式 null 拒绝。支撑和 policy 导出传递所属背景宽度。

Humble CPU：14 包 fresh 构建成功；最终 core 389、ROS 129、controller 205 项通过；
接口/launch 68 项通过；受影响 native 221 passed、2 skipped，
两项真实 producer parity 另以普通 Python 通过。普通启动入口错误参数返回 1，
catch(std::exception) 同时覆盖原 runtime_error 与公共 invalid_argument。
原应用负/零到达参数用例按成熟公共 API 精确断言 invalid_argument，拒绝行为不变。

TCP 专属与操作入口组 129 passed、7 skipped、1 环境失败：
test_scripts.py 的构建参数测试硬编码 Jazzy，本机只安装 Humble。
原 BASE 的脚本和测试逐 SHA 相同，隔离导出重现同一失败；
Humble 假 colcon 参数检查通过。6 个真实 TCP E2E、1 个 ROS local demo 未执行。
不安装 Jazzy，不把跳过或 BASE 环境失败描述为通过。

新增 profile 和 explicit-null 回归先得到真实 RED 后修复为 GREEN。
原地支撑夹具适配完整 bounds/受支持障碍；导出夹具严格建立导航字段不变前提，
并补 ground-only delta/full 回归，未降低阈值或放宽断言。
保留 mixin 重复、Time 显式构造、bytecompile 禁用及 python -O 解释器告警。
未变 SAC/课程/模型复用已审 Task3 源码资格，不重跑训练套件。

完整命令、原始 RED、二进制绑定与来源闭包位于仓库外
/home/kai/CodexDownloads/lunar_navigation/branch-integration-20261016/task-4-tcp-humble。
Orin/实车/GPU/训练/真实外部 TCP/DDS/真实 bag 均 NOT_RUN。
原应用工作树及活动运行代码未写入；没有推送、删除、reset、rebase 或 amend。
