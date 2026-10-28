# VIPER 三区轻量展示续作设计

用户于2026-10-29授权将VIPER工程合入，指定场景合并e572ed3、登记1a60611，并允许降低场景精度。随后明确传感器分辨率与频率保持旧平台。本续作将车放进同一A/B/C世界，解决真实速度热点并生成新的轻量场景版本。

## 身份、授权与边界

- 基线1a60611，独立工作树D:/Codexworktrees/isaac-viper-showcase/Lunar-Navigation，分支hk/isaac-viper-showcase-20261029，owner integration/pure-planner-orin；续接ISAAC-VIPER-MIDTERM-20261028/engineering，展示子阶段20261029。
- 已审工程源accf75b来自app/orin-unreal-simulation专题，仅19个本任务文件差量晋级为e15c667，不把34个继承TCP提交带进主线。原专题/全部历史和失败保留。主线docs/任务登记.md含其他任务未提交内容，本阶段不覆盖或提交它；登记用本专题文件和既有台式共享入口。
- 用户本轮授权本地合入；未授权推送、PR或删除。主线已有13个修改/166个未跟踪项，最终仅在无路径重叠且HEAD核对后合入，保护原始字节与索引。
- 传感器沿用640x480@15Hz双目、10Hz LiDAR、100Hz IMU，14cm基线、外参、有效量程/视场、扫描配置保持。物理dt=.01、50kg、轮组/碰撞几何与能力、安全门不改；严禁改变clock或重复缓存冒充提速。
- 原v5、原三区准备USD、原高精资产和旧日志只读；所有模型/生成USD/日志/截图/cache在D:/CodexDownloads/isaac-viper-showcase-20261029或台式对应独立analysis-artifacts目录，不进Git。
- 使用独立domain117/localhost1。只可在验证完整归属后停止本任务旧VIPER和目标旧三区预览以切换新统一窗口；不操作训练或其他进程。不得根据历史PID直接发信号。

## 决策与权衡

选择：先修正确认的传感器消息打包热点，再做明确的轻量新场景；保持传感规格。旧稳态RTF=.0563，94.75%墙钟在render后的传感器管线，候选是生成消息setter对bytes逐字节两遍校验；array('B')有原生快速路径。只减地形不能先验解决该热点，因此分别留CPU打包与真实RTF证据。

仅保持原精度/只修打包不能满足用户降低场景复杂度的要求；降低相机到320档会改变用户刚确认的规格，本轮不做。主线程选择uniform1m地形作为默认轻量档，低面数岩石保留实例位置和包络，去掉无碰撞碎屑和地形大纹理；这是新的展示几何版本，不宣称原高精碰撞等价。保留原质量可选入口供核对。

## 场景与接口

- 同一1km世界与A[235,335,-315,-215]、B[45,120,65,140]、C[-140,100,-420,-180]定义保持。默认车位于C旧候选(-55,-320)，在新碰撞地形重新拟合支撑/零初态；A/B起终点仍未验证，镜头切换不等于车辆换区。
- 源高程来自region JSON指定heightfield_m.npy和3个detail patch。轻量terrain统一1m采样，避免拼接裂缝；视觉与碰撞使用同一新网格。岩石同位置、包络低面数代理（目标每实例<=128tri），去除无碰撞碎屑视觉；不删车体精细外观。保存源/新三角数、源文件SHA、实际高度误差与场景质量ID。
- 新USD引用原始资产但强层禁用全部旧环境显示/碰撞、旧LunarRover/旧Render/旧图，只启用新环境与规范VIPER。新工程root显式保留units/defaultPrim/engineering_metadata_json和新落位/来源，startup实际USD/profile门必须通过。
- 可在build_stage现有API增加可选terrain_mesh_path/spawn_xy_m参数，默认行为保持；新build_showcase模块组织环境、车辆、区域相机与回执。不能直接把缺root metadata的旧准备层当运行入口。
- 保留总览/A/B/C/车辆视角。区域辅助标注只在viewport UI overlay或独立预览层，不进入传感器USD渲染；禁止以guide/noCollision当RTX隔离证明。C仍只应使用局部真实观测；本轮不加载Actor、训练权重或给它全图真值。
- A/B已知地图必须以后从此轻场景实际碰撞重新生成。新规划器、建图、策略适配仍属原任务，本阶段不伪称探索导航算法闭环已通过。

## 验收与交付

- 消息优化前后Image/PointCloud2 payload逐字节、shape/step/fields/frame/stamp一致，typed B数组路径有真实生成消息setter的CPU验证；保留assertions，不用python -O。
- 计时细分readback、首次样本保存、depth计算/消息打包/ROS发布，不把GPU异步等待笼统称网络。真实RTF=delta sim/delta monotonic，warmup/启动耗时分开。
- 原profile规格在新场景正例、规范9body8joint50kg/轮径/碰撞与真实RTX/IMU/odom/TF/clock有效样本通过；不要靠topic存在或截图代替数据。
- CPU/PXR检查新mesh数量/三角/可解析资产、单位、场景源不变、无旧重复物理、相机/元数据、旧质量入口；有意义纯测试。Windows全仓71收集错误和旧Humble211已证边界不重复跑。
- 一次有界同条件打包对照（每臂至多3sim或120wall，顺序执行，必要时复用旧真实RTF但明确历史混杂），再新轻场景一次6sim或120wall的运行/前进制动数据检查。仅在新失败给出具体原因后修复，不盲重试。新GUI保留给用户，实际view全车与三区视角。
- 最终独立审查/UTF8/diff检查后将专题快进或受控合入integration；保留用户脏文件与历史。更新同任务登记为已合入、记录性能与剩余NOT_RUN，不推送、不清理分支/工作树。