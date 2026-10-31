# 导航与TCP修改整合（2026-11-01）

任务 NAV-TCP-INTEGRATION-20261101，用户授权整理已有修改并合理提交合并。公共owner为integration/pure-planner-orin，TCP owner为app/orin-unreal-simulation。ROOT Windows单写入；不push、不删除、不改运行安装、不重启、不发车辆目标、不训练。论文/DRL研究及Isaac已有dirty内容不提交、不清理。

## 接收与保留范围

|来源|处置|内容与资格|
|---|---|---|
|hk/p3-pose-wait-20261030 的 eac7d14 及其祖先|公共候选接收|定位按源帧推进判活、导航统一进展职责、去重复门禁、连续旋转/统一完成、制动接转向、局部投影、圆包络、双向slew、参数生命周期。|
|tcp2 平滑产品2338d7b及本次文档56b8042|接收|全局参考整形、真实局部输入一致、独立参考显示；三份dirty收尾文档先原任务提交，再合并保留双方登记。|
|143a11f至9f43f65、e7a57cf/551ab08、c1df216、692365e|接收|参考保留/当前图认证、直接MPCC速度指令、P3时延预测、逐节点切线、实测终点验收、物理相位暖接续、持令无额外停车尾。|
|7276afe及具名TCP profile|接收|当前P4 .2m/s、.1rad/s、1.2秒持令、15秒后台全局更新、地图不按时间失效；基础平台、碰撞及动力学限制不扩大。|
|hk/tcp-combined-deploy-20261031 的5个独有提交|保留来源，选定提炼|两暖接续产品文件与当前候选逐文件无差异，已由c1df216精确接收；不整支叠加早期终点/runtime版本。|
|567f3a4认证memo|排除当前实现|3829b2b在新整合候选撤除单一实验，原提交/原专题保留。认证器SHA256 e93e0f1b296abf11a91e6ef8a7c606c2c5546212d1f9573ccb3d0b75ff69db87，与生产无memo版本一致。|
|3679aa8之后共享基础地图实现|暂停，不合入|该提交不是公共或TCP候选祖先；原p3-pose-wait工作树与实现保留。|
|hk/tcp-goal-entry-20261031 的dc334b1/af903b9|保留待适配|依赖私有UE目标/路线协议；用户当前选择的原始Env_X接口不具备该扩展，不声称此功能已接入。|

## 平台边界

公共wheel.yaml及legged_orin.yaml相对原主线无变化。TCP应用保留原OBJ/历史wheel.yaml(.1m/s、.195077rad/s)及wheel_orin.yaml边界，不借整理改变未重新验收的旧入口。当前P4走load_tcp_platform_profile：基础wheel加具名tcp_wheel_speed覆盖，生成同一平台配置交导航与relay，实际能力始终.2/.2/.1。不能把历史OBJ默认值当作当前P4生效值。

应用试合只有tcp_p3_feedback.yaml add/add冲突，选择最新已部署语义。原OBJ/TCP实现、部署工具、GUI及历史资料保留在应用线；不反向整体导入公共主线。公共算法及P4 runtime两候选相同；应用planner基础配置测试仅保留其真实基础速度断言。

## 新鲜验证

|范围|本轮结果|
|---|---|
|公共候选完整导航算法，当前源码编译的几何/距离场DLL|981 passed，88 skipped。|
|最终TCP应用完整导航算法|981 passed，88 skipped。|
|应用配置/协议/车体变换/公共组合与P4 profile专项|94 passed，2 skipped。|
|Windows相关TCP/公共launch入口|183 passed；最终profile不变断言另55 passed。|
|DRL nav1cm adapter|6 passed；不代表完整训练/DRL推理闭环。|
|Humble域169新隔离构建|lunar_nav1cm_planner与lunar_pure_wheeled_controller两包成功，48.6秒。|
|新隔离安装分包串行pytest|controller 701 passed；planner 113 passed；入口最终183 passed。|
|独立只读审查|终点三文件65 passed；配置/relay两文件65 passed；源码/范围/身份审查无阻断。数字有交集，不相加。|

本机原生库新编译于D:/CodexDownloads/lunar_navigation/nav-tcp-native-20261101；测试日志及临时目录均D盘。远端必要代码/配置/测试285项约594KB，仅写P4/artifacts/nav-tcp-integration-20261101；最终补齐测试引用的已有navigation_backend.py，无模型/原数据/设计文档。远端原安装、P3、Env_X与控制进程不修改。

保留失败证据：首次本机混跑ROS测试缺rclpy/生成消息；首次完整导航15失败中6项native环境、9项已过时终点断言。补原生库并将9项与已授权验收语义对齐；未放宽算法、阈值或加入新完成通路。旧command_ready门禁测试引用已删除函数，清除后由统一relay断流/连接用例覆盖。制动净空夹具显式声明本例.2m/s能力，避免历史应用低速档先LIMIT遮蔽净空断言。Humble初次混跑两包同名test_shutdown收集冲突，改分包串行；入口初期少后端文件/继承build_dir环境，补齐并隔离后通过；最终只加载新overlay local_setup避免重放历史underlay。旧日志不删除。Windows整套OBJ中的ROS、POSIX/bash和可视化依赖项不能在此环境完成，不写成旧OBJ全套通过。

## 落地与剩余事项

整合产品基线dbc80bd；应用接收公共来源并保留历史平台配置。已核对原主线7dfa199、DRL90b233c和TCP3b0b05a未被他人移动，实际公共主线与DRL快进ea8080032b436660ec322456f7ade456064bcfba，TCP应用快进596b13f0aee363f962e81b1227eae0159be81d45。本段收尾及入口/台账更新为文档补记，随后同步相关长期线；最终指针与干净状态记录在D:/CodexDownloads/lunar_navigation/navigation-tcp-landing-20261101.json。

未进行新TCP车辆运动验收；既有2米自然到达、132次交接无认证插零属于历史已保存证据，不能覆盖全部目标。终点极小残余速度MODE插零、终点制动后再修正、用户感知顿挫、私有目标入口仍分别保留，不宣称全部解决。Jazzy、Orin/实体车辆及完整长期DRL闭环本轮NOT_RUN。所有专题/工作树保留，已接收者待授权回收，暂停者保持原来源。
