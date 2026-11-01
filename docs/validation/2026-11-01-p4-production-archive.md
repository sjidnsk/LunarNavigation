# P4停机归档与正式入口收敛

用户授权最近改动本地合并、退出TCP/P3/P4、归档删除非必需文件，并明确正式启动不再依赖调试目录。

## 合并与停机

公共专题4639d0e已快进主线integration/pure-planner-orin及dev/drl-exploration；TCP应用合并提交30a10da保留平台历史差异，主线与应用导航/相关runtime各1224通过95跳过。未推送、未改其他应用线/保护脏工作树。

P4 operator、TCP GUI、capture、P3 visual/monitor/relay/pipeline、旧wheel relay、原Env_X vehicle共9角色SIGINT正常退出，无强杀。对应子进程退出，6665/6668无残留TCP连接。原P3/Env_X文件未改。

## 归档

D:/CodexDownloads/lunar_navigation/p4-archive-20261101/final-stopped/p4-final-stopped-20261101.tar.gz，858485045 bytes，SHA256 6f7969876f8a4f38aefaa29f5b7ce5c633b944e5c2bbd51667dba5995abdaab3。45909项逐条内容/链接/元数据核验，归档后原件重新全量SHA无变，27/27 SQLite readonly integrity_check=ok。build/current、deps、.venv及稳定install保留不归档。原历史1断链/绝对链接和重名路径如实保存，不扁平恢复。跨用户/proc无权限，未声称全系统fd审核。

## 正式启动

production模板399ed72/e2577a2/8420518分别固定环境/入口、清除继承Python Qt根目录定位、补P3显示稳定依赖；独立复审批准。固定source链只有系统Humble、P4 ros2-humble/install、原Env_X接口、P4 build/current/install。P3只读追加原env，monitor/view另读原bundle install_native及插件目录。与mapping/relay/capture分开，窗口退出不关mapping；不再source历史run/artifacts，无新的控制器或任务机制。

build/current149输入取Git4639d0e并逐SHA验证，5生产包构建通过（BUILD_TESTING OFF），5prefix/8产品SHA/5typesupport/7so ldd+ctypes/2launch showargs通过。正式配置实际模拟旧污染环境加载验证5prefix和7so，无历史run/artifacts定位；Bash/AST/YAML/JSON通过。P3必要semantic模块在原稳定overlay导入成功，有原环境SciPy/NumPy范围提示，未改依赖。未重启ROS节点/GUI/车辆，新启动完整联调NOT_RUN。

## 清理状态

精确清单D:/CodexDownloads/lunar_navigation/p4-archive-20261101/cleanup-plan.json，摘要cleanup-summary.md。拟删除已归档34447文件/347链接及空目录，约4.54GB逻辑字节。保留source、build/current、base install、deps/.venv、正式config和启动、唯一README与规则；临时cleanup-preparation回执本机保存后移除。依AGENTS批量删除规则，当前原件尚未删除，等待用户确认精确清单。
