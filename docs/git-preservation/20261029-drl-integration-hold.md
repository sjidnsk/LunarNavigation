# DRL 导航适配候选：未完成闭环，暂不晋级

任务GIT-PRESERVE-20261029/DRL-INTEGRATION；owner dev/drl-exploration；候选分支hk/drl-integration-20261029。用户明确提醒DRL导航适配未完成闭环，完整DRL→导航→跟踪执行验证是本批采用门槛，不能用纯参数、源码身份或组件测试代替。

## 实际状态

- 公共导航已落地integration/pure-planner-orin@fe5cef6，落地记录cf93f0f。
- DRL owner仍为dev/drl-exploration@dfd422b7536173a96bd61de92fa4cb0d769014c4，没有移动。
- c23f521仅是独立候选分支接收公共依赖；5c89519适配仅在该候选试合并WIP保存，不推进dev、主线或应用线。
- 现行训练、默认incremental_v2、冻结正式源码79清单、Actor和账本均未切换或重写。

## 已做与未做

本地导航选择/owned容差定向14通过2跳过3缺native失败；确认既有台式232个Python文件与候选一致后，三项纯native依赖单元补验3通过（0.17s）。后续身份/CLI/动作边界组25通过1失败，失败由Windows缺少公共controller包及生成的lunar_planning_msgs引起；补入controller搜索路径后仍缺生成消息，保留两次失败，不伪造全绿。开发87个文件SHA、正式清单不变和默认参数兼容的独立代码检查通过，但该检查不构成闭环资格。

未执行真实Actor接入、完整新RosEnv episode、导航Action/DDS执行链以及原45秒端到端复验；这些全部NOT_RUN。既有MPCC→标准Path MPPI三场组件通过只属于公共导航，不证明DRL决策、目标身份、取消/复位、观测更新和episode生命周期闭合。

## 后续门槛

在另一个明确验证任务中确定检查点及冻结配置、有限预算、隔离ROS domain和全链源码身份，验证真实策略选点→目标发送→nav1cm→标准Path MPPI→实测反馈→下一决策/正常终止，并覆盖取消、失败及复位。不得为过测重训、修改预算或改写旧失败。

当前处置：独立候选已保存，未闭环待验收，不合入owner。依赖此DRL适配的应用同步等待；其余历史论文/训练WIP继续保持已保存待审，不因mtime或本次候选试合整包采用。不推送、删除或切运行源。
