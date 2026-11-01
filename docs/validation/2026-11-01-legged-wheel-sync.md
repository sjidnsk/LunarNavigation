# 足式同步轮式公共导航

任务 LEGGED-WHEEL-SYNC-20261101；owner app/orin-legged-vehicle；ROOT Windows。
专题 hk/legged-sync-wheel-20261101；复用 nav-tcp-integration-20261101 工作树。
基线足式 ff75f52，公共来源轮式 9fe638f；不动原受保护脏目录。

## 范围与职责

公共算法同步轮式：nav1cm 全局搜索/拉直/终端平滑、局部 MPCC、连续 cmd_vel、参考认证、P3实测反馈、单向终点阶段、导航/探索互操作和统一话题。权重1、软区40cm、15s全局后台周期、1.2s持令。
复用同一个 wheel_navigation.launch.py 作为平面平台薄入口；platform_type 选择能力文件与少量应用参数，不复制控制器或任务状态机。足式入口不再隐式回到 incremental_v2，旧栈仍可显式选择。
足式 capability 文件 legged.yaml/legged_orin.yaml 不改；体积、速度、坡度/台阶/间隙、未知不可通行保持足式。原探索几何加载器将minimum_body_clearance_m作为平面净空的既有语义，本次兼容保留0.3m；不宣称其物理语义已重设计。新导航圆半径来自足式外接圆加该净空，约0.6779m，不套轮式75cm。
配置适配只映射足式前后速度、yaw及加速度到共用v/w模型；不提供横移、步态、足端或跨沟执行规划。NativeTerrain新增可选ABI v2构造符号，明确调用core::LeggedCapability的已有地形判定；wheel ABI入口和语义不变，旧库在足式选择时明确要求重编，不回退轮式。
标准默认mpcc_cmd_vel，不额外启旧纯跟踪器，T5出口保留；实际足式驱动未接入，默认enable_controller=false。不部署TCP机器正式运行、不自动发送目标。

## 合并决策

38项冲突逐文件决策见同目录2026-11-01-legged-sync-file-decisions.json。通用源码及旧C++声明/实现统一采用轮式来源，避免把旧足式地形置信度声明与新公共实现混编；旧足式实验和完整历史保留在ff75f52，不作为公共算法重新引入。足式旧栈配置orin_hardware与exploration_navigation保留；足式专用历史地形回归仍在源树但不伪称新公共实现已满足其不同算法预期。

## 验证

Windows源码/离线回归1314通过、97条件跳过；使用明确指定的geometry/field DLL，跳过真实acados与ROS环境不可用项。新增足式入口/配置源身份与原生能力分流已覆盖。首轮未预建外部pytest父目录导致临时目录创建失败，随后修正测试启动路径；Windows无ROS导致ament依赖测试不在本机执行。
Humble/Orin/足式实机、DDS和车辆闭环未验证，不由静态测试推断。

- ARM64 Ubuntu22.04/Humble隔离：原生桥独立g++构建成功，新增足式对照7通过/1本机Git身份检查不在隔离目录执行；同一25.8°坡面足式30°能力FREE、轮式20°能力BLOCKED，降低足式自身坡度上限后仍BLOCKED，未借用wheel evaluator。
- Humble两个受影响包lunar_nav1cm_planner与lunar_pure_exploration_ros构建成功；运行时、旧外推离线工具、配置、启动参数、终点保持共72测试通过。首轮一项测试因裁剪上传漏带p4_tcp_runtime纯配置夹具失败，补齐源码后72全通过，无产品修补。
- 隔离路径 /home/yanfa/P4/tmp/legged-sync-20261101，仅该P4子目录写入；未重启正式P4/P3/Env_X，未自动发目标。源码/日志本机 D:/CodexDownloads/lunar_navigation/legged-sync-20261101。Windows MSVC不能编译原有GCC overflow builtin，原生桥编译验收因此使用ARM64 g++，未为此改公共算术实现。
- 测试发现上一轮单向终点的旧测试预期残留，以及同源P3在NOT_DUE前已应用/未消费的混淆。通用修复独立提交5aac81e2，仅增加内部已应用源戳字段，不扩ROS接口/任务实体；同步main/TCP/wheel。旧测试已按用户最终约定更新，新源帧可正常接收，不恢复外推。

处置：算法/足式适配合并c567237a，通用源帧修复5aac81e2；已快进合回app/orin-legged-vehicle（产品合并c567237a、共享修复5aac81e2，登记提交3c8d1116）。无push/PR，无分支或工作树删除；专题待授权回收。足式真实驱动、Orin部署/闭环、横移及足端动作NOT_RUN。

最终核对：与轮式相比，共用全局/MPCC/认证/参考/任务及runtime源码相同，标准控制器源码也相同；有效源码差异仅5项足式适配（配置读取、节点圆半径缺省、native地形构造/包装）。公共源帧修复已落主线aed65c90、轮式95a08047、TCP fde8a16f；main重跑122项定向回归全部通过。所有相关工作树无未提交改动，不推送。
