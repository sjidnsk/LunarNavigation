# VIPER 三区轻量展示 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement task-by-task. No parallel source writers.

**Goal:** 将已审VIPER工程晋级到e572ed3三区场景，保留旧传感规格，修消息热点并提供可用轻量展示。
**Architecture:** 接口兼容的typed-byte消息修复/细分计时；独立轻量USD构建层与区域相机；源/原始证据保留，真实SDK验证后安全合入。
**Tech Stack:** Python/NumPy/USD、Isaac6.0.1、SDK Humble cp312、主机Humble cp310。
**Spec:** docs/superpowers/specs/2026-10-29-isaac-viper-showcase-design.md

## Global Constraints
- 640x480/15Hz双目、10Hz LiDAR、100Hz IMU；14cm/外参/范围/视场与扫描配置保持。
- 物理dt=.01、50kg、项目车能力与安全门、真实时钟不改；domain117/localhost1；不改正式controller/Actor。
- 原场景/资源/已有证据只读，新输出仅仓库外D盘或台式独立目录。
- 用户授权本地合入；不推送/PR/删除/批量移动清理。主线脏登记不纳入本专题diff。

## Review Focus
- typed payload不能改变endianness/shape/ROS fields或泄漏可变SDK缓冲。
- 新轻量视觉与碰撞必须同源，禁止旧不可见collider继续活动。
- USD root metadata不从subLayer自动继承；新落位的COM/frame/源hash不能借旧值冒充。
- 三区标注不能进入C传感器输入，A/B地图和算法未接通不得说已验收。
- 合入不得吞入34个应用历史、主线脏文件或其他暂存内容；最终窗口归属要重新核验。

### Task 1: 消息快速路径与性能计时
**Files:** Modify tools/isaac_viper/ros_bridge.py、run_scene.py；Test tests/isaac_viper/test_sensors.py 或专用test_transport.py。
**Interfaces:** 现有RosBridge/RealSensors调用与ROS wire不变；render_timing增加明确phase_ms，旧字段保留。
- [ ] 写payload roundtrip/alias/empty/non-contiguous数据回归及计时结构检查，RED。
- [ ] Image/PointCloud2 data使用array('B')快路径，保留全部消息校验与内容；不关闭全局assert，不降采样。
- [ ] 分开active products、camera/lidar readback、NPZ first-only、camera/lidar pack/publish；必要时Bridge细分depth math、msg_pack、publish汇总。
- [ ] 聚焦/相关纯测试，现有生成消息setterCPU微对照，不init ROS、不GPU。自审/显式commit/report。
- [ ] 独立task review，确定问题按同任务fix loop关闭。

### Task 2: 轻量三区VIPER构建、实际运行与文档
**Files:** Create tools/isaac_viper/build_showcase.py、config/isaac_viper_showcase.yaml、tests/isaac_viper/test_showcase.py；Modify build_stage.py可选落位/terrain路径、run_scene.py显示panel/viewport；更新工程说明/本专题登记。
**Interfaces:** build_showcase(profile,region_config,source_scene,assets_dir,output_dir,quality='light')->dict；输出同runtime可启动的工程USD、区域JSON、quality/源/面数/误差回执。沿用Task1 phase_ms与canonicalROS。
- [ ] 纯采样/网格/边界/拒绝覆盖/参数测试RED；实际PXR组合正例与旧collider/metadata/标注泄漏变异。
- [ ] 实现默认1m统一terrain与<=128tri岩石代理，去无碰撞碎屑/地形大纹理；保存高度误差与新质量ID，旧原始quality可核对。
- [ ] C候选在新实际mesh重算落位，9body8joint50kg；旧环境/车/render关闭，新metadata、相机/区域UI保留，传感器世界无区域辅助网格。
- [ ] 旧profile实际门通过；源文件hash前后同，新对象/collision/三角数核验。明示A/B地图/Actor/新规划器NOT_RUN。
- [ ] 本地测试/compile/diff、commit；父统一同步新目录，禁止子agent绕SCP审批。
- [ ] 只匹配确属本任务旧Viper/目标三区GUI后按授权切换，保留旧回执；其他进程不动。
- [ ] 顺序有界数据/RTF验证与控制停车smoke；真实12topic数据/相位/传感规格/物理dt验证；全车及三区GUI实际查看。
- [ ] 文档记录新profile不降规格、性能实际值与混杂、几何版本和不可外推边界；更新同任务记录。自审/显式commit/report。
- [ ] task独审与最终全范围审查、修复，最终实际合入与主线用户文件保护由root负责。

## 收尾
- [ ] 主集成线HEAD/脏路径/索引核对；只提交本任务文件与来源清楚的晋级差量。
- [ ] 只在无重叠时合入，前后核用户原有修改/未跟踪项；中央脏登记不覆盖，采用专题登记与台式共享入口。
- [ ] 源码/台式登记记录已合入commit/RTF/验证层级；保留原worktrees、场景和全部新旧产物待回收。
### Task 3: 按用户反馈恢复地形与岩石细节
**Files:** Modify build_showcase.py、config/isaac_viper_showcase.yaml、tests/isaac_viper/test_showcase.py；README/本专题登记按实际验证更新。
- 默认light改0.5m（8Mtri）、320tri岩石（cap512），新增uniform0p5m-rock320-v2；兼容旧1m/80和original冻结资产的门。
- 地形单位顶点法线 + matte非金属regolith视觉材质；独立physics purpose不变，visual/collision同mesh。
- 有意义的RED/GREEN：平面法线、岩石320拓扑/包络、版本与实际拓扑不符拒绝、旧版兼容；旧sensor/control/phase接口不改。
- 实际PXR/旧新门/修改源前后SHA、新版有界传感运动与实图/RTF由root执行；新旧runtime快照分离，不覆盖现存GUI源码。
- Task3独立task review + final whole-branch review后实际合入；不推送/删除，中央dirty登记保护不变。
