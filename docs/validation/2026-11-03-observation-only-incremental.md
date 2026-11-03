# 共享观测精简、局部更新与目标机验证

任务MAP-OBSERVATION-ONLY-20261103；基线c5d73954；公共owner integration/pure-planner-orin。
执行专题hk/tcp-terrain-window-deploy-20261102，工作树D:/CodexDownloads/lunar_navigation/git-preservation-20261029/worktree。ROOT单写，tcp2已协调暂停重叠修改。

## 分批结果

1. 0d41dc7e：删除SourceEvidence.classification_fragments、accepted_spans共享证据接口及分类副本裁剪。导航_rows/_source_classifications、来源权限、UNKNOWN撤销FREE/保留BLOCKED不变；只保留有效观测引用的测量。
2. bec7e46d：NaN-aware比较源测量；重复输入复用同一快照，纯值变化复用几何及已有索引；局部新格保留未变化片段对象。删除同义的全图相等性重算及分类副本辅助收集。
3. 88fce470：复用同一包围盒树，只裁剪相交片段；轴对齐完整覆盖直接撤销，真正部分/旋转覆盖仍调用原精确内核。无引用测量交换填洞，避免删除首格导致全图重新编号。
4. ee9b6c0a：ARM FMA收缩数值问题修复，见下文。最后清理不再使用的函数与说明，并用独立Python差集oracle完成最终目标验证。

没有新增ROS实体、版本层级、超时门禁或碰撞容差。75cm精确圆、软净空、导航/MPCC/控制职责及P3/Env_X保持不变。共享事实只服务源测量/有效覆盖，导航world独立维护分类。

## 原机实图同帧对照

在192.168.100.216原地使用先前采集的相同三帧64m×64m、20cm GridMap及TF/平台配置。新旧源码分别在隔离进程运行，真实地图不传出设备。基线源码冻结为c5d73954，目标库位于P4隔离构建。

|帧|旧apply ms|最终新apply ms|新纯编码 ms|
|---|---:|---:|---:|
|0|6519.85|1048.14|620.13|
|1|15185.47|606.77|631.35|
|2|15259.06|609.13|627.43|

三帧世界障碍/自由矩形语义hash逐帧完全相同；有效观测polygon/height/state/slope/relief的排序内容hash逐帧完全相同。59个障碍矩形、19538个观测片段不变，重复分类片段29854→0。此样本测量数组前后均19538条，不能声称测量数减少；新内核保证无引用测量不保留。

前次new.json/old.json中的encode_ms包含诊断排序/JSON解析/hash工作，不作为纯编码性能证据。new-qualified.json中的encode_ms才是纯协议编码时间。结果只是三帧同主机离线实图对照，不是长时吞吐或车辆闭环保证。

产物/home/yanfa/P4/tmp/observation-only-20261103/：old.json、new-qualified.json、对应日志、独立新旧源码、DDS及测试输出；原始地图仍在既有current-no-motion-20261103。正式source/install/env未切换。

## ARM数值验证

Humble额外旋转覆盖回归发现合法旋转矩形被ReadPolygon拒绝。最小-O3程序确认：ARM FMA收缩使Cross(a,b,b)得到约-4.06e-17；-ffp-contract=off时为0。不是依据版本猜测，也没有扩大面积epsilon。共享Cross将两个乘积明确分别舍入后相减，保留零阈值；Windows/ARM使用相同乘积差语义，合法旋转通过，非凸输入仍拒绝。

第一次仅重建消费者仍加载核心包已安装的旧头文件；补齐核心安装、按core→ROS→planner顺序重建后修复生效。失败日志保留。剩余一项失败在测试oracle重复将近共线碎片送回native验证器；已改为独立Python凸多边形差集/交集参考，产品阈值不改。

## 最终验证

- 本机当前源码重建terrain/reference DLL：完整导航/地图/launch/事实/话题集合1586通过、29跳过（42.61秒）；独立oracle更新后本机8项、ARM34项通过。
- Humble三个受影响包隔离构建成功；新核心头依赖安装已核对。
- Humble真实原生库观测/旋转/亚厘米/覆盖回归34/34通过。
- ROS地图包CTest16/16、规划器13/13通过；私有域174 DDS 20→5→20cm均ready，revision1/2/3。没有车辆命令或导航目标发布。
- 本轮没有重跑旧legacy长距离性能未通过项，也不据此宣称该资格已解决。

本机产物D:/CodexDownloads/lunar_navigation/observation-only-20261103，未产生新的C盘测试产物。Windows结果不代替Humble；两者均不是车辆/足式/DRL闭环验收。

## 剩余边界与处置

实现及隔离验收完成，已提交待主线晋级/正式部署；不合并长期线、不push，不删除专题工作树。正式P4和P3未切换，车辆闭环NOT_RUN。

增量复用发生在观测生产端：未变化快照/源/几何复用，真正相交区域才裁剪。为维持现有不可变连续数组接口，值改变时仍需复制数组，几何变化时部分元数据索引仍可能重建；不宣称所有操作严格O(变更格数)。跨进程继续传完整快照，编码约0.62–0.63秒，消费者变更版本投影仍非全链路dirty-tile增量。本轮不额外设计新协议或地图管理实体，不隐瞒这部分后续优化空间。
