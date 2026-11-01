# P4正式启动

本目录是正式安装模板，根目录固定为 `/home/yanfa/P4`。无历史run/artifacts叠加依赖；P3与Env_X只读复用。

安装映射（先检查目标现有内容，再逐文件安装）：

| 本目录 | P4目标 |
|---|---|
| `config/**` | `config/**` |
| `start_tcp.sh` | `start_tcp.sh` |
| `start_p3.sh` | `start_p3.sh` |
| `start_p3_view.sh` | `start_p3_view.sh` |
| `p3_runtime.py` | `p3_runtime.py` |
| `README.md` | `README.md`（根唯一说明） |

另将既有 `source/scripts/simulation/start_p4.sh` 逐文件安装为根 `start_p4.sh`，保持原实现；它调用source中的既有 `p4_tcp_runtime.py`。保留正式源码、`.venv`、`deps/acados-v040`、`ros2-humble/install` 和 `build/current/install`；这些运行依赖与本模板分开部署。正式环境只叠加系统Humble、P4基座、Env_X接口、build/current四层。P3组件额外读取其 `scripts/env.sh`，不调用会在P3写锁的controller入口。

终端1运行TCP：

```bash
bash /home/yanfa/P4/start_tcp.sh
```

P3各组件在独立终端使用**同一个本次新目录**。mapping先启动，再启动relay/capture/wheel；monitor与view可选且独立：

```bash
# 每个终端均设置相同的新目录名称；不要复用旧run。
run=/home/yanfa/P4/run/p3-20261101-session1
bash /home/yanfa/P4/start_p3.sh mapping "$run"
# 其他终端分别运行：
bash /home/yanfa/P4/start_p3.sh relay "$run"
bash /home/yanfa/P4/start_p3.sh capture "$run"
bash /home/yanfa/P4/start_p3.sh wheel "$run"
bash /home/yanfa/P4/start_p3.sh monitor "$run"
bash /home/yanfa/P4/start_p3_view.sh "$run"
```

P4导航/空闲探索沿用现有入口：

```bash
bash /home/yanfa/P4/start_p4.sh
# 无导航窗口：bash /home/yanfa/P4/start_p4.sh --no-rviz
```

每个组件前台运行，Ctrl+C正常退出。关闭view只退出窗口，不会关闭mapping。重启整套P3时使用新的共用run目录。运行日志/状态/生成配置均在P4/run，临时文件在P4/tmp，缓存在P4/.cache。TCP参数仍使用已生效的park初始模式；不会自动发送新任务或改变控制策略。capture只转发不保存数据。P3的模型和算法代码保持原位置读取，不复制或上传模型。

本模板仅做Bash语法、Python语法、配置解析和静态路径核对；未启动进程，未验证ROS/Humble联调、GUI或实机，均为NOT_RUN。有效profile及车辆参数取自停机前配置，未调算法阈值。
