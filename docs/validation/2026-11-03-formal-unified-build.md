# P4统一正式构建交付

任务TCP-FORMAL-UNIFIED-20261103；owner app/orin-unreal-simulation。复用hk/tcp-terrain-window-deploy-20261102和原工作树；算法基线0939f1e5，入口/缓存/复审修复bd59719e、8f465bce、7fcc7362。正式payload固定为7fcc73628c5fb8dfc32a4a5ac83d4168966c4e0a；未移动长期线、push或删除工作树。

## 当前正式身份

- 新构建：`/home/yanfa/P4/build/formal-20261103-unified`；796个必要源码文件逐一等于该提交Git规范字节，非Windows CRLF工作区字节。源码、配置、Python/C++安装及生成solver共1839文件封存manifest。
- 活动依赖闭包10个生产包，同源附带1个验证支持包，全部Humble/ARM重新构建；未复用原current/budget2的项目安装文件。
- `config/p4_active_build.json`一次原子记录active/previous，正式环境和启动脚本均由其选定来源。`P4_SOURCE_DIR`、安装和`NAV1CM_BUILD_DIR`指向同一构建；solver已用实际acados预编译。
- 基础依赖保留Humble、P4/ros2-humble、Env_X接口及P4/.venv/acados；不修改P3/Env_X。先清理PATH/PYTHONPATH/LD_LIBRARY_PATH/AMENT/CMAKE/COLCON等旧环境，再建立依赖链。先source旧环境再加载新环境的实测，10个活动包prefix/Python/动态库均来自新构建，未残留current/budget2。
- wheel.yaml、tcp_wheel_speed.yaml、tcp_p3_feedback.yaml与原正式版逐YAML对象相同。限速0.1m/s/0.1rad/s、75cm碰撞、50cm软净空、20度坡度、30cm局部高差、1.6m局部前瞻、15秒刷新/2秒全局预算保持。

## 验收

Windows入口/选择102pass9skip；Linux真实flock/原子选择/启动/跨构建旧进程111pass。独立审查两项P2（空/缺项清单可放行、旧构建无锁operator漏检）均经RED/GREEN修复；新environment显式清理继承旧overlay，实测依赖路径通过。

新构建core CTest21/21、nav1cm14/14、RViz3/3、活动探索3/3通过。地图ROS完整15/16；唯一失败为旧IncrementalNavigationNode.EnabledDebugPublishesCycleSnapshotAndActualStartPatchView依赖LUNAR_INCREMENTAL_NAVIGATION_BUILD_DEBUG（宏LUNAR_BUILD_DEMO），该调试功能在正式构建关闭。其余该节点57项单独通过；原始失败日志保留，不冒称地图包全绿，不为旧用例打开额外调试功能。旧legacy探索/750m性能不在当前活动TCP MPCC/增量探索资格中，本次未重新宣称通过。

同一64m实图三帧apply为1023.96/590.23/585.06ms，world与有效观测hash逐帧等于之前合格结果；分类重复历史为0。完整事实编码仍约0.63秒，不宣称全链路dirty-tile。新安装实际全局结果planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true；80步真实acados三轮候选均连续认证通过，包络半径0.75。DDS持续发布与RViz清理用例均包含在新包验收中。以上不是现场车辆到达证明。

## 正式切换和回退边界

正式环境、根启动入口、选择工具及单一选择文件已整体落地。根/P4/source和旧install完全保留；旧环境及原根入口备份于`/home/yanfa/P4/build/formal-baseline-20261103`，previous已指向此处。清单校验/同一启动锁使运行中的P4不能切换；不进入导航状态机，不增加车控门禁。

用户明确取消实际回退演练，本次没有切回旧正式版；仅保留通过单元测试的回退工具和旧版。当前未启动正式任务、未发送目标或车辆控制，已有旧RViz窗口未强制重载。下一次按原入口启动使用新构建及新RViz插件：

```bash
bash /home/yanfa/P4/start_p4.sh
```

结束P4后，如需回退：

```bash
python3 /home/yanfa/P4/p4_build_selection.py --previous
```

启动指令中的P4退出匹配已同时覆盖旧source和新formal源码位置；未改P3/车辆退出指令。选择回执在新build/deployment-receipt.json，完整清单manifest.json，路径核对qualified-overlay.json，测试/实图原始结果均留该构建。设计文档只在本机仓库，未上传到调试机。

处置：已提交算法/入口、已整体切换正式文件，待owner长期线审查/合入；旧版与本专题保留，未回收。实际回退演练NOT_RUN（用户取消），正式导航/车辆运动/到达闭环NOT_RUN。未自动重启P4或P3。
