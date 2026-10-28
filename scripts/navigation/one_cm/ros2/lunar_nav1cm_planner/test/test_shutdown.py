"""真实SIGINT必须在worker停止之后才销毁ROS context。"""
import os
import subprocess
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
        self.create_timer(.1,lambda:os.kill(os.getpid(),signal.SIGINT))
    def stop_runtime(self):
        assert rclpy.ok(), 'context closed before runtime stop'
        print('RUNTIME_CONTEXT_ALIVE',flush=True)
        return {'threads_alive':[]}
module.PlannerNode=Probe
module.main()
assert not rclpy.ok()
"""
    result=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'RUNTIME_CONTEXT_ALIVE' in result.stdout
