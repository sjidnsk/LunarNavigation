"""SIGINT只请求退出，避免在ROS绑定内部异步抛异常或提前关闭context。"""
import signal
import threading


def spin_until_sigint(node):
    import rclpy
    stopped=threading.Event()
    previous=signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT,lambda signum,frame:stopped.set())
    try:
        while rclpy.ok() and not stopped.is_set():
            rclpy.spin_once(node,timeout_sec=.1)
    finally:
        signal.signal(signal.SIGINT,previous)
