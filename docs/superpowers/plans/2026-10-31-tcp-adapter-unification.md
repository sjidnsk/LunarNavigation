# TCP运行与测试入口统一计划

基线ee85651，原专题/main-worktree，ROOT单写入。先RED再实现，复用既有p4_tcp_runtime.py作平台工具owner，不新增任务/权限身份或计时器。

- [x] 用发布替身验证共享低速profile、命名空间装配和真实传输判活；AST执行测试入口真实command_input，捕获reference_accepted重复门禁。
- [x] 在p4_tcp_runtime.py集中profile/启动参数/环境装配、转发、auto/park、现有租约及实测停车。一个组件替代两套armed/指令时间状态；收尾开始后不再接运动指令。
- [x] 两入口都调用同一组件，测试入口删除接管/定位再判活的转发门控；预检、日志、时限、取消和停稳取证保留，preflight-only不取控制权。
- [x] 回归无租约/新命令/断流/非有限/超速/零指令/退出不续跑等边界及现有P3/导航职责测试；检查UTF-8/语法/diff。静态检查不能写成ROS或TCP运动验收。
- [x] 独立只读复审后显式路径本地提交，更新登记/验证证据。不push/merge、不部署、不启动真实车辆。
