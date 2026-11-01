# MPPI、导航与验证工具交付

任务MPPI-20261018，owner为dev/drl-exploration，专题feat/mppi-controller-20261018，基线f7db40fe。代码集成、模型选择和车辆部署分别验收。

## 最终方案

控制器提供pure_pursuit、地图感知mppi和mppi_path_follower三个显式后端。通用默认仍为pure_pursuit；正式对比选择mppi_path_follower，保留运动预测、路径/航向误差、运动约束及到达停稳，不在跟随器内部重复地图候选认证。地图版保留作对照。

局部规划保留车体膨胀、FREE合法性、20cm范围/权重2的软代价及原路径简化净空保护；额外硬净空为0，关闭忽略软代价的长度精化。原1cm或1.5cm方案属于前序候选。保留滚动参考连续性、同时间戳里程计与TF、指令历史和20Hz仿真步握手。40ms求解预算仅为诊断，不因超过该值就停车；输入有效性检查独立保留。

## 显式运行参数

新评测根保存evaluation-navigation.json，模板为config/drl_mppi_evaluation_navigation.json：

```json
{"controller_backend":"mppi_path_follower","simulation_step_sync":true,"mppi_nominal_response_time_s":0.0}
```

参数直接进入环境与实际ROS控制器，不通过OwnedProcesses强制覆盖所有控制器。响应时间0表示本次理想仿真模型，不代表实车延迟为0；实车不得开启仿真步同步。后端选择与步同步独立。

没有该文件的历史普通评测沿用原默认；声明MPPI的comparison-protocol必须提供该文件，防止静默回退。proof绑定解析后参数，八路调度器与单场入口使用同一身份；修改参数不能继续使用旧缓存。

## 可复用的512次检查点比较

入口scripts/drl/compare_checkpoints.py提供prepare、seal、verify、run。配置模板为config/drl_checkpoint_validation_512.json。CHECKOUT为源码；SOURCE_RUN提供原配置及验证清单；ACTORS提供block-01至24.pt；NEW_RUN必须不存在。在正确的Humble/Python环境运行：

```bash
python "$CHECKOUT/scripts/drl/compare_checkpoints.py" prepare \
  --source-root "$SOURCE_RUN" --actors-dir "$ACTORS" --root "$NEW_RUN" \
  --protocol "$CHECKOUT/config/drl_checkpoint_validation_512.json" \
  --navigation "$CHECKOUT/config/drl_mppi_evaluation_navigation.json"
```

prepare逐字节复制24份权重，原验证清单只改每场预算为512，保持地形种子、母图和传感器。完整验证120场、24检查点共2880场，8路且错误计失败继续。达到99%覆盖可提前结束；拒绝覆盖已有输出。不复制test清单。

STAGE为独立冻结运行目录，与NEW_RUN分离；包括当前Python包及四个ROS包的install。ament索引必须把四包解析到STAGE/install，PYTHONPATH首先解析STAGE/source，底层DRL native扩展使用配套Humble环境。在此环境确认导入路径后：

```bash
python "$CHECKOUT/scripts/drl/compare_checkpoints.py" seal --root "$NEW_RUN" --stage "$STAGE"
python "$CHECKOUT/scripts/drl/compare_checkpoints.py" verify --root "$NEW_RUN"
python "$CHECKOUT/scripts/drl/compare_checkpoints.py" run --root "$NEW_RUN"
```

已有identity时拒绝重封。run校验配置、场景、参数、权重、源码及原生安装；恢复时复用相同命令、环境与根目录，合法完成缓存复用。SIGINT/SIGTERM请求暂停，可用systemd用户服务托管。入口不会改变已有服务。

指标包括安全SR99、碰撞、执行异常、终止覆盖率、实际L90/95/99、仿真T90/95/99、覆盖曲线、收尾/无增益成本、单位实际米数新增面积与计算开销。阈值取首次确认的原生观测，未达到保存null。block20先固定每场共同物理AUC预算；失败保留在成功率分母，缺失物理曲线报告可用数量。

排序优先零碰撞、零执行错误，再比较两任务宏平均安全完成率、共同预算时间AUC及共同成功场景L99/T99。selection.json仅为验证集候选，不自动测试或部署。洞穴不确定性应按母图分组，单次确定性比较不冒充重复随机试验。

## 四场实际回放

固定validation-moon_outside-000/004、validation-cave_inside-000/004，每检查点4份，共计划96份。目录visualizations/block-NN/case-id/job-id：

- trajectory.jsonl.gz：每0.05仿真秒实际位姿、里程、速度和控制指令。
- decisions.jsonl.gz：候选目标与概率、选中索引/目标及实际位姿。
- references.jsonl.gz：ROS实际发布参考、会话及地图版本。
- maps.zip及map-events.json：地图增量；每条依次为小端uint32平面索引、uint8观测、uint8原始状态、uint8导航状态。索引=y*width+x，观测按历史并集累计，状态按新值替换。
- scene.npz及manifest.json：参考区域、传感器、原点/分辨率、录制状态与开销。参考位图little-bitorder。真值只作可选背景，默认隐藏未知区域。

时间/里程扣除初始化；轨迹来自动力学积分点，不能用目标连线替代。complete表示文件关闭，成功与否查看评测row。录制有CPU/I/O开销，不将不同负载的墙钟时间直接归因于策略。

## 验证边界

原冻结候选200场为199成功、0碰撞、0基础设施错误，唯一预算耗尽有两点循环，未宣称解决。结果只属于原身份，不因源码提交而更名。稀疏圆弧停滞仍为已知边界，本次不扩大修复。

当前24检查点验证继续使用原冻结快照，不在提交合并时切源。新构建、测试与落地证据记入任务登记及analysis-artifacts/mppi-integration-landing-20261019。模型、轨迹、安装、缓存与结果不纳入Git。Orin/实车以及未执行的应用验证保持NOT_RUN。

独立审查的缓存入口与真实包解析问题已补回归修复。非阻塞待补项：回放当前面向单场固定地图，跨历元重置/全量替换删除旧tile的显示语义尚未覆盖；外层campaign中断恢复与候选排序的组合测试可继续扩展，底层并发缓存/暂停已有真实子进程回归。它们不作为本次静态场景录制之外的能力承诺。
