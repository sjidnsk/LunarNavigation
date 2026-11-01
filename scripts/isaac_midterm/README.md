# 月球极区三区准备与预览

这三个脚本固化 2026-10-28 已确认的三区准备和台式预览，实现不依赖当时的 GUI 会话层继续存在。
区域、地图口径和 block-19 身份由 `docs/simulation/isaac-polar-v02-midterm-regions.json` 给定。
本目录仅准备几何和显示，不挂接规划器、探索推理、传感器发布器或底盘命令。

## 源码与资源

- `prepare_regions.py`：读取原始高程和区域 JSON，在明确指定的新目录生成标注 USD、选区图及身份回执。
- `verify_regions.py`：使用现有 USD 库静态核对区域、相机和新增对象，更新核对回执与产物清单；不改写场景或生成器源码。
- `preview_regions.py`：通过 Isaac Sim `--exec` 在新进程打开标注入口，暂停时间线，在会话层创建加亮边界和总览/A/B/C 取景按钮。

约 1.2 GB 的原地形、纹理、模型、block-19 权重、导出的 USD、图片、日志、缓存和设备回执保留在仓库外。
默认区域 JSON 对应台式上现有资源路径和已核对的 SHA；换机器或资源版本后须显式更新身份并重新验证。

## 在台式重新生成

在仓库根目录执行，`OUT` 必须指向本次任务自己的新目录：

```bash
REPO_ROOT="$PWD"
OUT=/mnt/data/WS/lunar-navigation/analysis-artifacts/isaac-polar-midterm-20261029
/usr/bin/python3 -s "$REPO_ROOT/scripts/isaac_midterm/prepare_regions.py" \
  --regions "$REPO_ROOT/docs/simulation/isaac-polar-v02-midterm-regions.json" \
  --output-dir "$OUT"
```

台式的系统 NumPy 1.21.5 与 Matplotlib 3.5.1 已用于本场景生成；`-s` 避开不兼容的用户级 NumPy 2 包。
脚本不安装或改变依赖；已有结果会拒绝覆盖，失败产物应保留并使用新目录重新准备。

## 静态验证

以下版本路径来自台式现有 Isaac Sim 6.0.1：

```bash
ISAAC_ROOT=/home/kai/isaacsim-6.0.1
USD_EXT="$ISAAC_ROOT/extscache/omni.usd.libs-1.0.3+f9bf0dda.lx64.r.cp312"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$USD_EXT${PYTHONPATH:+:$PYTHONPATH}" \
LD_LIBRARY_PATH="$USD_EXT/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$ISAAC_ROOT/python.sh" "$REPO_ROOT/scripts/isaac_midterm/verify_regions.py" --output-dir "$OUT"
```

解析通过只证明标注结构、坐标和身份。资源组合、感知、车辆运动与闭环仍需分别验证。
所有脚本的 `--help` 可用普通 Python 查看，不依赖加载 Kit 或 USD 库。

## 暂停预览

先检查参数；这一步不创建目录或打开 GUI：

```bash
/usr/bin/python3 -s "$REPO_ROOT/scripts/isaac_midterm/preview_regions.py" \
  --scene "$OUT/Midterm_Polar_V02_Regions.usda" \
  --output-dir "$OUT/view-01" --dry-run
```

然后从台式桌面终端启动新的窗口：

```bash
"$ISAAC_ROOT/isaac-sim.sh" --exec "$REPO_ROOT/scripts/isaac_midterm/preview_regions.py" \
  --scene "$OUT/Midterm_Polar_V02_Regions.usda" --output-dir "$OUT/view-01"
```

该命令启动新进程；不要在持有他人未保存场景的现有进程中手工执行脚本。
左侧按钮切换总览、A 岩石绕障、B 陨坑坡地、C 自主探索视角。状态保存在指定 `view-01/stage-status.json`，
纯场景缓冲图保存为 `regions-viewport.png`；这不是桌面截图。已有预览目录会被拒绝，包含只有旧图片而没有状态文件的部分恢复目录。

标注和出生点是选区预览内容，边界本身不执行区域限制。无碰撞或 `guide` 不单独证明 RTX 雷达已排除标注。
正式感知采集使用干净月球车入口，视频中的区域边界从区域 JSON 后期叠加。
A/B 起终点、C 在线 v5 适配、建图、新规划器接线、运动与录制仍是后续关口。

## 验证边界

2026-10-28 的 GUI 预览已实际核对三区可见及时间线暂停；对应回执与原失败记录保留在旧产物目录。
本次合并将命令行输入路径从临时目录改为显式参数；实际合并版的重建与静态验证结果见任务登记。
预览脚本不加载检查点，已知地图也不会传入 C 的 Actor。Jazzy、Humble 车辆闭环、Orin 和实车验收不由这些静态检查推断。
