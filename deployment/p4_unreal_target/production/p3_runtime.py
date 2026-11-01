#!/usr/bin/env python3
"""仅适配既有P3 relay的运行状态路径；不导入其controller.main。"""
import json
import os
from pathlib import Path
import sys

sys.dont_write_bytecode = True
mode, run_arg = sys.argv[1:]
run = Path(run_arg).resolve()
if run.parent != Path('/home/yanfa/P4/run') or not run.name.startswith('p3-'):
    raise SystemExit('运行目录必须是P4/run下的p3-*直属目录')
out = run / 'results/live'
sys.path.insert(0, '/home/yanfa/P3/lidar_visual_fusion/scripts')
import control
control.STATE = run / 'run_state.json'
if mode == 'mapping':
    # exec保留PID，状态恰好供原relay核对正在运行的launch使用。
    control.save(dict(output=str(out), domain='57', localhost_only='0',
                      processes=dict(pipeline=dict(pid=os.getpid(), identity=control.identity(os.getpid())))))
    os.execv('/usr/bin/python3', ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
             '/home/yanfa/P3/lidar_visual_fusion/scripts/fusion.launch.py',
             'profile:=/home/yanfa/P4/config/p3/profile.yaml', 'output_dir:=' + str(out),
             'use_sim_time:=false'])
elif mode == 'relay':
    import live_sensor_relay
    live_sensor_relay.ROOT = run
    sys.argv = [sys.argv[0], '--output', str(out)]
    live_sensor_relay.main()
else:
    raise SystemExit('仅支持mapping或relay')
