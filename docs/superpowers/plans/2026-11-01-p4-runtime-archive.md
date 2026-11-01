# P4运行目录整理与本机归档

目标：P4仅留有效源码、编译安装、依赖、启动脚本、配置及根说明；调试历史归档D:/CodexDownloads/lunar_navigation/p4-archive-20261101。

当前总量重点run约2.4GiB、artifacts约1GiB。不能整目录搬空：P4依赖旧环境链和5个overlay；P3 SQLite/WAL活写，vehicle与6373速度relay仍用旧参数/输出/环境。保留所有非P4目录只读边界。

- [ ] 只读依赖/活写审计；归档以明确文件清单与SHA校验，不把活SQLite原始复制冒充一致快照。
- [ ] 冷历史文件先归档本机D，读取清单/字节数/归档摘要并验证完整可读；远端暂保留。
- [ ] build/current新建正式构建候选，独立基础环境opt/ros + ros2-humble + 原Env_X，新overlay覆盖planner/RViz/exploration_ros/core/controller五包；构建输入版本f6fc2c5，避免旧run/artifacts引用。
- [ ] 生成干净环境/配置/根README及迁移清单；所有候选先验证，不切运行进程。
- [ ] 最终请用户确认明确批量迁移/清理范围以及活跃P3/vehicle/6373转发的受控交接；批准后停稳、补一致活动归档并校验，再移除已备份旧文件。

允许先行：只读、冷数据归档、全新build/current下准备构建候选。禁止先行：递归删除/批量移动/覆盖旧源码/停止外部owner进程。实际运行状态及原始归档保留在设备和D盘，设计文档不上传设备。
