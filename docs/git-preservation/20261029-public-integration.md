# 公共导航试合与顺序整合记录

任务GIT-PRESERVE-20261029/PUBLIC-INTEGRATION；用户授权按公共导航→DRL→应用线顺序推进。坡面动力学议题搁置，不追加行为修改。

## 试合身份

- 原Windows主线integration/pure-planner-orin：1532cf0981a2665b70d8c0f2a8ff2eb420d54e73，原工作区仍有已保护的论文/DRL未提交内容。
- 治理分支hk/git-preservation-20261029：试合前235c976，包含第一阶段成果保存记录。
- 公共导航来源hk/nav-1cm-feasibility-20261026：eb10ff2670ed87dd1edfc4fcc22995a620fe66bb，含已审查末端姿态修复。
- 实时origin主线b206870；本次未fetch/push。公共专题包含远端主线的来源历史。

使用现有干净治理工作树merge --no-ff --no-commit进行实际试合。唯一冲突为docs/任务登记.md追加段，保留双方完整块后解决；没有产品冲突。原脏root未暂存、覆盖、清理或移动分支。当前主线落地需保护该root的既有工作区与index。

## 产品与入口核对

137个公共导航/跟踪器/组合入口的合并index Git blob全部等于eb10ff2；10个原专题与试合checkout的字节差异均只为CRLF/LF，记录在仓库外public-trial-identity.json。主线Isaac已有内容按正常三方合并保留，不整支覆盖。

公共轮式入口scripts/orin/start_navigation.sh提供nav1cm选择；nav1cm_wheel组合默认不启动controller，显式启用时采用标准Path MPPI。legacy与incremental_v2保留显式入口；scripts/start_all.sh仍是原legacy打包入口，本次不顺手切换。源代码整合不等于部署安装或启动车辆。

## 当前验证

- 实际试合树Windows定向：127通过13跳过（3.85s），包括末端yaw、全局、局部、同步/异步协调器、轮式入口及配置。
- 实际试合树部署配置静态检查：1通过（0.52s），wheel.yaml/wheel_orin.yaml净空均0.2m。
- 台式此前已授权的隔离source-r2与eb10ff2局部源码一致：使用真实native库，六项原环境失败专项6通过（0.28s），不再把原缺路径失败当算法失败。
- 台式相关完整组45通过1失败1跳过；唯一失败是隔离源码ZIP未包含wheel_orin.yaml。配置上传被自动审批拒绝（原授权未明确包含该配置），本轮采用不外发配置的替代：同一配置静态检查本地通过、native专项远端独立通过。不把该完整组记为全绿。
- 原真实acados定向45通过3跳过及三场MPCC/标准Path MPPI闭环证据保留；本次未重跑无改动的长验证。

台式native库来自nav1cm-p3-interface-20261027/integration-20261028/final-fix-main-r1/install/lunar_nav1cm_planner/lib。六项回执在D:/CodexDownloads/lunar_navigation/git-preservation-20261029/native-six.log；原环境失败、缺配置失败均保留。

以上是源码与组件资格；试合后的完整ROS/DDS、生产安装、真实P3/rosbag、Orin/实车仍NOT_RUN。没有切换正式训练、修改Actor/账本或重放恢复脚本。

## 顺序及待办

1. 公共导航候选提交后，独立审查通过再安全落地主线；原dirty目录须保留可靠分支身份和全部原字节，不能强制checkout/reset。
2. DRL先接收公共依赖，再审查5c89519接入，按模块选择保存切片；旧WIP不整包覆盖现行五方法源码。
3. 应用线各自适配，轮式保留自有terrain/ABI/平台配置；仿真、足式与terrain按能力相关性决定同步，分别验收。
4. 所有分支/worktree/ignored运行依赖保留，未获删除或push授权，不回收。

独立限定复审通过：HEAD和MERGE_HEAD登记非空行缺失均0，无冲突标记，产品范围及职责通过。用户已明确授权把原脏root切到同HEAD保护分支，保留所有文件、未跟踪与暂存内容，再由独立D工作树接管并快进main；不绕过保护直接update-ref。

## 本地主线落地

用户明确授权同HEAD保护分支方案后，原目录已切至hk/protected-main-working-20261029@1532cf0；180个修改/未跟踪文件SHA及暂存树逐项保持。主线由D:/CodexDownloads/lunar_navigation/git-preservation-20261029/main-worktree接管，已正常ff-only落地fe5cef6ea5649c5c371e4431fef499236b172aec，工作树干净。外部protected-root-receipt.json保存保护证据。无push或部署切换。
