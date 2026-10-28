"""SIGINT协作请求覆盖spin与资源释放；共享层无节点或包反向依赖。"""
from contextlib import contextmanager
import signal
import threading


@contextmanager
def sigint_guard():
    stopped=threading.Event()
    previous=signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT,lambda signum,frame:stopped.set())
    try:
        yield stopped
    finally:
        signal.signal(signal.SIGINT,previous)


def spin_until_sigint(node,stopped):
    import rclpy
    while rclpy.ok() and not stopped.is_set():
        rclpy.spin_once(node,timeout_sec=.1)
