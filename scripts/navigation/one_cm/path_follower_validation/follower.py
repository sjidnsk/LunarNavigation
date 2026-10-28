"""只读观测真实安装控制器，不改优化、决策或命令。"""
import hashlib
import sys
from pathlib import Path
from dataclasses import asdict
import rclpy
from rclpy.signals import SignalHandlerOptions
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode
import lunar_pure_wheeled_controller
from evidence import Recorder,CMD,DIAG

def run(args):
    rec=Recorder(args.output,'follower')
    params={'input_mode':'incremental_path','controller_backend':'mppi_path_follower',
            'standard_map_source':'direct_p3','command_topic':CMD,'mppi_diagnostics_topic':DIAG,
            'mppi_nominal_response_time_s':0.0}
    cli=['--ros-args']
    for k,v in params.items():
        cli+=['-p',k+':='+str(v)]
    rclpy.init(args=cli,signal_handler_options=SignalHandlerOptions.NO)
    node=PureWheeledControllerNode()
    paths=list(Path(lunar_pure_wheeled_controller.__file__).parent.glob('*.py'))
    rec.emit('INSTALL',executor=type(node._executor).__name__,module=lunar_pure_wheeled_controller.__file__,
             pythonpath=sys.path,production_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
             parameters={p.name:p.value for p in node.get_parameters(list(node._parameters))},
             policy=asdict(node._policy))
    assert type(node._executor).__name__=='StandardPathFollower'
    assert '/install/' in lunar_pure_wheeled_controller.__file__
    original=node._executor.update
    def observe(*a,**kw):
        result=original(*a,**kw)
        rec.emit('UPDATE',state=asdict(a[0]),result=asdict(result))
        return result
    node._executor.update=observe
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rec.emit('NATIVE',sha256={str(Path(line.split()[-1])):hashlib.sha256(Path(line.split()[-1]).read_bytes()).hexdigest() for line in Path('/proc/self/maps').read_text().splitlines() if ('libnav1cm_' in line or 'liblunar_incremental_navigation_core.so' in line) and Path(line.split()[-1]).is_file()})
        rec.emit('CLOSE',map_worker_alive=node._p3_source._worker.is_alive())
        if rclpy.ok():rclpy.shutdown()
        rec.close()
