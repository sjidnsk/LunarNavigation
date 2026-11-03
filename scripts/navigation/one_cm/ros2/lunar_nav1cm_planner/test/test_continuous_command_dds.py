"""仅私域/测试话题的真实DDS发布连续性；不连接车辆或P3。"""
import json
import statistics
import threading
import time
from uuid import uuid4

import pytest
rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from test_runtime import continuous_runtime, wait_runtime


@pytest.mark.parametrize('rolling',[False,True])
def test_slow_computation_keeps_dds_command_stream_and_cancel_stops(monkeypatch,rolling):
    rclpy.init()
    node=Node('continuous_command_probe_'+uuid4().hex[:8])
    topic='/nav1cm_continuous_probe/run_'+uuid4().hex+'/cmd_vel'
    publisher=node.create_publisher(Twist,topic,10)
    received=[]
    subscription=node.create_subscription(Twist,topic,
        lambda m:received.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin=threading.Thread(target=executor.spin,daemon=True)
    spin.start()
    runtime,sent,entered,release,calls=continuous_runtime(monkeypatch,hold_s=1.2)
    def publish(v,w):
        sent.append((time.monotonic(),v,w,threading.current_thread().name))
        message=Twist();message.linear.x=v;message.angular.z=w
        publisher.publish(message)
    runtime.publish_command=publish
    if rolling:
        import nav1cm.execution as execution
        original=execution.certified_velocity_command
        def slow(*args,**kwargs):
            time.sleep(.18)
            return original(*args,**kwargs)
        monkeypatch.setattr(execution,'certified_velocity_command',slow)
        release.set()
    try:
        assert wait_runtime(lambda:publisher.get_subscription_count()==1,2.)
        runtime.start()
        assert entered.wait(2.)
        assert wait_runtime(lambda:len([m for m in received if m[1:] == (.05,.02)])>=7,.6)
        if rolling:
            assert wait_runtime(lambda:len(received)>1 and
                received[-1][0]-next(m[0] for m in received if m[1:]==(.05,.02))>=3.2,3.5)
        moving=[m for m in received if m[1:] == (.05,.02)]
        assert all(m[1:]==(.05,.02) for m in received if m[0]>=moving[0][0])
        gaps=[b[0]-a[0] for a,b in zip(moving,moving[1:])]
        assert statistics.median(gaps)<.09
        assert max(gaps)<.3  # 测试机调度上限，不是新增运行门禁。
        if rolling:assert len(calls)>=12
        else:assert len(calls)==2  # 第二次仍阻塞，持续发布没有重做认证。
        mark=time.monotonic()
        runtime.request_cancel('continuous')
        assert wait_runtime(lambda:any(m[0]>=mark and m[1:]==(0.,0.) for m in received),.3)
        stopped=next(m[0] for m in received if m[0]>=mark and m[1:]==(0.,0.))
        assert wait_runtime(lambda:len([m for m in received if m[0]>=stopped])>=3)
        assert all(m[1:]==(0.,0.) for m in received if m[0]>=stopped)
        print('CONTINUOUS_DDS '+json.dumps(dict(rolling=rolling,span_s=moving[-1][0]-moving[0][0],moving_samples=len(moving),
            median_gap_ms=statistics.median(gaps)*1000,max_gap_ms=max(gaps)*1000,
            cancel_zero_ms=(stopped-mark)*1000,computation_calls=len(calls))))
    finally:
        release.set()
        assert not runtime.close()['threads_alive']
        executor.shutdown(timeout_sec=2.);spin.join(2.)
        node.destroy_subscription(subscription);node.destroy_node();rclpy.shutdown()
