```bash
# 终端1：TCP车辆接口
bash /home/yanfa/P4/start_tcp.sh
```

```bash
# 终端2：传感器采集
ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 P3_RUNTIME_ROOT=/home/yanfa/Env_X/InterFace \
bash /home/yanfa/P3/lidar_visual_fusion/scripts/start_capture_source.sh --host 192.168.10.23
```

```bash
# 终端3：P3建图定位、转发与可视化
bash /home/yanfa/P3/lidar_visual_fusion/start_live.sh --profile /home/yanfa/P4/config/p3/profile.yaml
```

```bash
# 终端4：P4导航、探索与RViz
bash /home/yanfa/P4/start_p4.sh
```
