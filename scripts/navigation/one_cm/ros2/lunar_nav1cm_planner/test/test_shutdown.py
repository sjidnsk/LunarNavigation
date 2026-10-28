"""真实SIGINT必须在worker停止之后才销毁ROS context。"""
import os
import subprocess
import signal
import time
import sys


def test_main_stops_resources_while_context_is_alive():
    # 用最小ROS节点隔离昂贵规划资源；运行真实main、rclpy和SIGINT处理。
    code = """
import os,signal,rclpy,time
from rclpy.node import Node
import lunar_nav1cm_planner.node as module
class Probe(Node):
    def __init__(self):
        super().__init__('planner_shutdown_probe')
        print('READY',flush=True)
    def stop_runtime(self):
        print('STOPPING',flush=True)
        time.sleep(.3)
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
        assert child.stdout.readline().strip()=='STOPPING'
        child.send_signal(signal.SIGINT)
        output,_=child.communicate(timeout=3)
        assert child.returncode==0,output
        assert 'RUNTIME_CONTEXT_ALIVE' in output
    finally:
        if child.poll() is None:
            child.terminate();child.wait(timeout=3)


def test_sigint_during_subscription_returns_callback_before_shutdown():
    code="""
import os,signal,rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lunar_nav1cm_planner.node as module
class Probe(Node):
    def __init__(self):
        super().__init__('subscription_shutdown_probe')
        self.pub=self.create_publisher(String,'/nav1cm_shutdown/subscription',10)
        self.create_subscription(String,'/nav1cm_shutdown/subscription',self.callback,10)
        self.create_timer(.05,lambda:self.pub.publish(String()))
    def callback(self,message):
        os.kill(os.getpid(),signal.SIGINT)
        print('CALLBACK_RETURNED',flush=True)
    def stop_runtime(self):
        assert rclpy.ok()
        return {'threads_alive':[]}
module.PlannerNode=Probe
module.main()
"""
    result=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'CALLBACK_RETURNED' in result.stdout
