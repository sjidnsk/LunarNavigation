"""真实SIGINT必须在worker停止之后才销毁ROS context。"""
import queue
import subprocess
import signal
import threading
import time
import sys

import pytest


class ChildOutput:
    """持续收集完整诊断；标记等待不受首行警告影响，也不会无限阻塞。"""

    def __init__(self, child):
        self.child = child
        self.lines = []
        self.pending = queue.Queue()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.child.stdout:
                self.lines.append(line)
                self.pending.put(line)
        finally:
            self.pending.put(None)

    def wait_for(self, marker, timeout=10):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                pytest.fail(f'timed out waiting for {marker}; output:\n{self.text}')
            try:
                line = self.pending.get(timeout=remaining)
            except queue.Empty:
                pytest.fail(f'timed out waiting for {marker}; output:\n{self.text}')
            if line is None:
                pytest.fail(f'child exited before {marker}; returncode={self.child.poll()}; '
                            f'output:\n{self.text}')
            if line.strip() == marker:
                return

    @property
    def text(self):
        return ''.join(self.lines)

    def finish(self, timeout=3):
        try:
            self.child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pytest.fail(f'child did not stop; output:\n{self.text}')
        self.reader.join(timeout=timeout)
        assert not self.reader.is_alive(), f'child output did not close:\n{self.text}'
        assert self.child.returncode == 0, self.text
        return self.text


def test_marker_wait_keeps_diagnostics_and_propagates_child_failure():
    code = "print('dependency warning', flush=True); print('READY', flush=True); raise RuntimeError('probe failed')"
    child = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
    output = ChildOutput(child)
    try:
        output.wait_for('READY')
        with pytest.raises(AssertionError, match='probe failed'):
            output.finish()
        assert 'dependency warning' in output.text
        with pytest.raises(pytest.fail.Exception, match='exited before STOPPING'):
            output.wait_for('STOPPING')
    finally:
        if child.poll() is None:
            child.terminate()
            child.wait(timeout=3)


def test_marker_wait_has_a_deadline():
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(10)'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    output = ChildOutput(child)
    try:
        with pytest.raises(pytest.fail.Exception, match='timed out waiting for READY'):
            output.wait_for('READY', timeout=.1)
    finally:
        child.terminate()
        child.wait(timeout=3)


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
    transcript = ChildOutput(child)
    try:
        transcript.wait_for('READY')
        time.sleep(.3)
        child.send_signal(signal.SIGINT)
        transcript.wait_for('STOPPING')
        child.send_signal(signal.SIGINT)
        output = transcript.finish()
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
