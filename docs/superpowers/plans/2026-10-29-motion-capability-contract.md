# 导航运动能力统一设计与实施计划

任务 GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE/MOTION-CONTRACT；公共修复owner integration/pure-planner-orin，在现有隔离hk/drl-integration-20261029候选工作树执行，基线7fa0417。用户2026-10-29明确授权统一规划/认证/跟踪器运动能力：可边走边转、可原地旋转，指令v/w独立限幅；实测滑移超速不裁剪或冒充指令违规。未授权改变DRL目标或训练，不合并/push。

## 合同与范围

统一显式motion_model字段：differential_drive表示独立v/w能力；curvature_limited保留旧行进曲率约束兼容。WheelLimits和TrackingPolicy未指定时保留curvature_limited，旧配置未指定也按旧模型解释，避免影响历史应用/验证。当前config/wheel.yaml显式选择differential_drive；组合launch从同一配置解析并同步传递模型，独立node也读取同一字段。未知模型必须拒绝。保留数值曲率字段供旧模型使用，差速模式该字段不产生隐藏限制。

差速模型下，移除MPCC约束、初始种子、候选复核、独立认证、标准Path MPPI采样/预测/最终指令的v/w曲率耦合；保留独立速度、加速度平滑、停稳/换向合同、碰撞与新鲜度检查。模型身份参与认证digest/solver指纹，旧缓存不可混用。Legacy motion_reference执行器保持旧模型合同，不把当前导航选择默默传播给旧应用。跟踪指令必须独立限幅；原始里程计保持不变，不能通过裁剪反馈满足约束。滑移补偿与超界初态的完整可行性恢复不作为本次算法完成声明。

本次不同时重写末端yaw代价/终端集合。先消除跨层能力矛盾，随后才验证并设计完整终点位姿收敛。不能以去掉曲率就声称DRL闭环已成功。

## 任务

- [ ] T1：规划/种子/认证增加模型选择与一致门禁，RED/GREEN覆盖高曲率合法差速、原地旋转、独立超速拒绝、旧模型仍拒绝、hash区别。
- [ ] T2：标准Path MPPI增加同名模型，统一预测/采样/指令限幅，覆盖前进/倒车/近零速度旋转；旧模型保持；不裁剪实测状态。
- [ ] T3：主agent贯通wheel配置、configuration、node、组合launch，补静态跨层合同测试。
- [ ] T4：主agent跑完整相关纯测试、独立审查并修复；台式ROS/acados构建与运动验证仅在实际执行后记录，未执行标NOT_RUN；保存任务证据并本地显式路径提交。

## 审查重点

模型缺省与未知值；bool/string参数类型；倒车及零速旋转；旧模型缓存/证书不能复用；标准Path分段不能丢失yaw；实际反馈超限时指令仍受限且不篡改实测；其他legacy入口不可继承差速配置而假装兼容。

## 实施回执

T1/T2/T3完成，T4适用验证完成：本地674 passed/126 skipped/6 deselected；Humble两包构建和507 tests通过；真实acados/native定向87 passed/1 skipped/3 deselected；独立源码审查通过。新增wheel_orin显式差速，旧缺省兼容；真实节点和组合入口模型一致。完整Actor run-r4仍0到达，停在距目标.1118845932m，完整闭环FAIL，不晋级长期线。详细证据、环境失败与边界见docs/validation/2026-10-29-motion-capability-contract.md。
