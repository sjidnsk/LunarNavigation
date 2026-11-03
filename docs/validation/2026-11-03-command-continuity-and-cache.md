# 移除认证到期归零、复用求解器缓存与运行等待分析

任务TCP-COMMAND-CONTINUITY-20261103。owner integration/pure-planner-orin；TCP部署owner app/orin-unreal-simulation；基线02b7ec45，复用hk/tcp-terrain-window-deploy-20261102工作树。用户明确删除认证到期自动归零，允许撤回计算/发布解耦；最终保留独立发布以免恢复真实断令，仅移除时间期限及其proof_sink内部接口。用户已有目标/取消/定位中断/计算异常逻辑不变，模型1.2秒仅定义有限预测检查范围，不再提供无限持令的认证保证。

## 实现与审查

- runtime pending只保存速度与目标输入序号，无valid_until；发布循环不因时间推移撤销命令。计算线程仍可明确发布停车。取消、换目标、关闭、定位中断、tick异常继续撤销。没有替换为另一种超时。
- 原4项红测全部因到期归零失败；修改后跨检查范围、3.2秒原地转向准备、迟到结果与连续slew均通过；取消/输入中断并发回归保持。
- local_mpcc按配置+SDK模板/动态库/ABI/CasADi/模型源码摘要匹配编译缓存；匹配只load，缺失/损坏才build。使用SDK库名/目录函数；单标准库锁保护生成/清单提交，错误不回退旧模型。现有LocalWorker已启动异步预热并有ready_probe，本轮未加ready实体、固定等待或假地图预热。
- 独立审查发现并修复Windows DLL无lib前缀、运行DLL在bin、模型源码未参与缓存身份三项。加载失败直接报告。旧SDK模型装配替身只调整隔离边界，不修改产品迎合替身；缓存另由14项测试及真实acados验证。

## 当前验证

- 初始受影响Windows121pass；最终适用完整回归1433passed、143skipped（31.42s），见本机windows-full-final.log。Windows不含ROS的范围单独记录；直接收集全部ROS用例产生缺rclpy/ament失败，不作为产品通过。
- 216私域175 Humble受影响runtime/direct/cache及真实DDS：127passed（21.82s）。此前13个缓存setup错误是旧pytest nose把setup helper当无参xunit，改名cache_case后通过，原日志保留。
- DDS计算完全阻塞：3.223s/65条非零，中位50.337ms/max51.220ms，无插零；取消2.347ms。滚动慢计算：3.224s/65条，中位50.371ms/max51.326ms，取消1.261ms。均私域随机话题，未接车辆。
- 216真实acados N80/0.2s/正式平台参数：首次三模型构造13069.92ms，第二独立进程59.17ms，全部JSON/so mtime不变。配置改变、模型源码或ABI变化仍需一次构建；部署可提前在目标位置预编译，不将构建放入首目标关键路径。
- 真实forward/reverse/spin候选分别22.535/21.662/20.032ms，三者完整认证通过（check_braking_tail=False沿用正式配置）。构造只用隔离目录，不代表实车到达验收。

## 运行期间等待：证据与边界

在原正式进程PID111224采样6.046s：map线程111279 CPU4.839s（80%单核），tick111280 CPU0.539s且51/60次futex，command111281 CPU0.0505s；运行队列等待分别0.067/0.035/0.009s，不支持整机CPU排队解释。ARRIVED/brake时1031条交接日志p95间隔1.683s/max1.900s，command_total_ms p95仅0.067ms；因此运行慢不止solver。

已核对源码：session.tick不等MapStore锁/最新地图；apply在输入锁外。runtime最终交接仍在_condition内同步命令日志和Path发布；node每tick构造三条Path。事实编码每次遍历完整有效历史观测，重新建dict/list、校验并JSON编码。MPCC_COMMAND日志至EXECUTION_BRAKE的782ms只定位到其间的日志返回/Path构造与发布/调度区段，不能断言哪一个独占782ms。

未实证map CPU在native地形、Python几何/来源维护及facts编码之间的占比；未证明stdout阻塞或DDS独占，ctypes.CDLL调用可释放GIL。未安装profiling工具、未gdb暂停线程。后续优化应先测量并将诊断/可视化移出输入临界区、复用不变Path，再决定是否隔离facts编码；本轮不盲改地图架构。

## 部署边界

隔离路径/home/yanfa/P4/build/qualification-command-continuity-20261103。只上传必要源码和测试，原正式封存构建未零散覆盖。P3/Env_X未修改、未发现场目标/速度。现场导航、真实转向与轨迹稳定性NOT_RUN；不能用私域DDS代替车辆验收。后续完整候选构建/选择回执追加记录，不覆盖这些边界。
