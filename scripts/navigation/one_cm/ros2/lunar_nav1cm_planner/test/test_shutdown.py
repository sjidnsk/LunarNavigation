"""真实SIGINT必须在worker停止之后才销毁ROS context。"""
import os
import subprocess
import signal
import time
import sys


def test_main_stops_resources_while_context_is_alive():
    # 用最小ROS节点隔离昂贵规划资源；运行真实main、rclpy和SIGINT处理。
    code = """
import os,signal,rclpy
from rclpy.node import Node
import lunar_nav1cm_planner.node as module
class Probe(Node):
    def __init__(self):
        super().__init__('planner_shutdown_probe')
        print('READY',flush=True)
    def stop_runtime(self):
        assert rclpy.ok(), 'context closed before runtime stop'
        print('RUNTIME_CONTEXT_ALIVE',flush=True)
        return {'threads_alive':[]}
module.PlannerNode=Probe
module.main()
assert not rclpy.ok()
"""
    child=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    try:
        assert child.stdout.readline().strip()=='READY'
        time.sleep(.3)
        child.send_signal(signal.SIGINT)
        output,_=child.communicate(timeout=3)
        assert child.returncode==0,output
        assert 'RUNTIME_CONTEXT_ALIVE' in output
    finally:
        if child.poll() is None:
            child.terminate();child.wait(timeout=3)
