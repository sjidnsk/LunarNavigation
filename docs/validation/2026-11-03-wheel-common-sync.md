# 轮式应用公共成果同步

owner app/orin-real-vehicle；原95a08047，候选3a8b8b96，接公共主线a97fd590。用户授权只同步代码，不重启正式P4，不部署实车。

合并保留轮式planner_params_file、非TCP硬件入口、平台配置9项blob完全不变；runtime两处冲突合公共严格递增P3源戳，结果与主线运行代码字节一致。公共地图/碰撞/持续发布/交互均接收，不带TCP30cm起伏、限速或命令预测启用。

适用轮式入口与runtime73/73通过，独立复审重跑73通过，无阻塞P1/P2。Windows缺bash的hardware_selector2项、rclpy的staticTF、ament_index_python的app terrain未以Windows通过冒充ROS通过。公共同源Humble验收见2026-11-03-tcp-common-promotion.md；本轮轮式完整ROS包重建、实车设备部署/闭环NOT_RUN。

历史nav1cm-shared-source-provenance.json及11月1日file-decisions保留当时证据，当前同步身份以本文与2026-11-03总整合清单为准。无push、不改远端，不回收旧专题。
