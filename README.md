# 足式应用启动

```bash
bash scripts/orin/build.sh
export NAV1CM_BUILD_DIR=/absolute/path/to/solver-cache
bash scripts/orin/start_hardware_navigation.sh
```

打开 RViz：

```bash
bash scripts/orin/start_navigation.sh legged false "" true nav1cm
```

导航与探索共用常驻入口，在 RViz 面板中开始或暂停探索。
