# P4话题、面板和启动入口收尾

产品提交a6fab57（公共命名3e6d3c9、面板89662bb/ac6e730、TF d30cd0f/ac04047）。P4自有接口统一/Car/T4，配置config/p4_topics.yaml供launch/runtime/适配器/RViz生成复用；P3/Car/T3、TF和车辆/car原生名称保留。

已部署192.168.100.216的P4，运行目录/home/yanfa/P4/run/car-t4-panel-a6fab57-20261101，operator214847、navigation214890、rviz214891。P3/Env_X六个受保护进程及文件不改。实际图验证：目标RViz→planner，任务/状态RViz↔探索，速度planner→P4relay→原生车辆节点；/goal_pose无端点。6秒收到状态22、地图3、定位5、静态TF6；探索IDLE无task，RViz插件实际加载。未自动发目标或START。

```bash
cd /home/yanfa/P4
./start_p4.sh
# 不开RViz
./start_p4.sh --no-rviz
```

脚本前台exec，复用P3/Env_X，Ctrl+C沿用runtime清理停车。非阻塞flock只防重复启动；旧记录核对真实进程，兼容省略--root，过期/复用/僵尸PID不误阻。生产重复调用已正确拒绝。

验证：本地127通过5个Linux专用跳过；Humble增量探索37/37；RViz3组CTest共7用例；Linux启动7/7；安装和私有复制launch --show-args均通过。各批独立复审通过。设备证据在P4 artifacts/{car-t4-panel-20261101,static-tf-20261101,rviz-panel-20261101,startup-tests-20261101}及run/live-audit.json。设计文件未上传，仅20份必要代码/配置逐文件SHA核验。

现场驾驶、区域探索完成、人工目标交接驾驶NOT_RUN；历史C++包基线失败仍见frontier-navigation-adaptation验证文档，未宣称全包无失败。原终点侧偏和20cm内20秒预测未实施；DRL后续适配未开始，共享基础地图暂停。
