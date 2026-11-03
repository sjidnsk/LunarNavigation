# 命令连续性与MPCC缓存晋级

继承任务TCP-COMMAND-CONTINUITY-20261103；用户授权将已正式切换的4a4c9e23同步适当分支。
公共owner integration/pure-planner-orin；执行ROOT/Windows，复用hk/tcp-common-promotion-20261103及D:/CodexDownloads/lunar_navigation/tcp-common-promotion-20261103，基线b68551ad。来源实现4a4c9e23fb9b28941c28b8d18a7db7fc7e7c5c01，部署证据8f5be5ad7eed7688891e854e8f565a2221c2890c。

1. 公共晋级：只接4a实现14文件及8f两份部署证据追加，解决任务登记的追加冲突时保留双方记录，不恢复旧树；旧认证到期自动归零历史说明标注已替代。保持独立publisher/取消/目标切换/定位中断/异常处理，新期限删除不能改名保留。缓存仅源码，不提交求解器二进制或机器缓存。
2. 公共适用Python/native与Humble/DDS定向回归，独立复审接线/缓存版本身份/取消与连续发布；源码与部署身份分别记录，车辆闭环NOT_RUN。不再把1.2秒模型检查范围解释为发布截止。
3. 公共通过后合主线，再分别三方合入TCP、轮式、足式，保留b685已完成的平台消费者修复、足式窗口ABI/P3域环境/各应用平台参数。三应用批合并后独立复核配置和公共代码一致性，运行适用回归。
4. 最终写主线与三应用落地SHA和任务处置；不push，不重启/替换正式P4，不写P3/Env_X，不删分支/工作树。formal-20261103-continuity仅只读核对，不切回unified，不进行车辆测试。

验收范围：本机适用测试及同源Humble/DDS可运行回归；原4a/8f真实acados数据是来源证据，不包装为本轮现场复测。地图CPU/日志Path临界区热点保持待定位，未纳入本次实现。
