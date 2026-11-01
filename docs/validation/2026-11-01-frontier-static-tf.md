# 探索晚启动的静态 TF 接收修复

## 原因与范围

现场桥接发布者以 depth100 保存6条分开的 TFMessage，包含 map→odom，但其后还有相机变换。探索订阅静态TF使用可靠/TransientLocal/depth1，晚加入只得到最后消息，PoseResolver无法将持续收到的P3 odom位姿变换到map，停留WAITING_FOR_INPUT。

本批仅将探索静态TF订阅历史深度改为100，与既有P4桥一致。动态TF、源时间、地图有效性、车辆限速和状态机不变；不增加延时门禁、不伪造坐标变换。不承诺恢复源发布者已经丢弃的历史或超过100条的任意历史。

## 证据

复现测试先发布所需零戳map→odom，再以同一存活发布者发布5条无关静态变换，最后创建探索节点；不补发动态TF，通过真实假导航Action服务端是否收到目标验证。

目标Humble隔离目录：/home/yanfa/P4/artifacts/static-tf-20261101。
初次构建因快照缺少根launch安装输入失败，该次不作为RED。补齐输入后原产品代码构建成功，red-test.log显示目标数0（预期1），回归失败。修复后green-build.log构建成功，green-test.log中全部增量探索36/36通过，原单消息晚加入用例也通过。原包其他历史失败不由本轮宣称消除。

产品与回归已通过独立只读复审；运行切换将与后续RViz面板统一进行，当前未宣称现场探索恢复、自然到达或车辆闭环通过。P3/Env_X未修改。Orin/实车NOT_RUN。

## 后续状态显示切片

为统一面板显示，在保留WAITING_FOR_INPUT状态及全部原条件的前提下，将原因细分为WAITING_FOR_MAP、WAITING_FOR_REGION、WAITING_FOR_POSE_IN_MAP。没有新增状态、计时器或定位可用性门槛。新增回归在有地图/里程计而没有变换时验证具体原因，补入变换后验证实际提交导航目标。reason-red-test.log准确失败，reason-green-build.log成功、reason-green-test.log全部37/37通过，独立复审批准。运行切换仍待统一部署。
