"""真实已安装 PlannerNode；内部 factory 仅增加观测和单请求 250 ms 延迟。"""
from dataclasses import asdict
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import sys
import time

import nav1cm
from ament_index_python.packages import get_package_prefix
from lunar_nav1cm_planner.node import PlannerNode, create_resources
import rclpy
from rclpy.parameter import Parameter

from evidence import Recorder


class DelayOneRequest:
    def __init__(self, output):
        self.output = str(output)

    def __call__(self, runner, request):
        if request['identity']['sequence'] == 15:
            path = Path(self.output) / 'delay.jsonl'
            with path.open('x', encoding='utf-8') as stream:
                for kind in ('DELAY_START', 'DELAY_END'):
                    stream.write(json.dumps(dict(kind=kind, monotonic_s=time.monotonic(), pid=os.getpid(),
                                                 request_identity=request['identity']))+'\n')
                    stream.flush()
                    if kind == 'DELAY_START':
                        time.sleep(.25)
        return runner(request)


def run(args):
    rec = Recorder(args.output, 'planner')
    assert '/install/' in nav1cm.__file__, nav1cm.__file__
    assert not any('/source-' in p for p in sys.path), sys.path
    prefix = Path(get_package_prefix('lunar_nav1cm_planner'))
    production = list(Path(nav1cm.__file__).parent.glob('*.py'))
    production += list((prefix/'lib').glob('libnav1cm_*.so'))
    rec.emit('INSTALL', nav1cm_file=nav1cm.__file__, pythonpath=sys.path,
             production_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in production})
    rclpy.init(args=[])

    def factory(limits, capability, settings):
        session, store = create_resources(limits, capability, settings)
        rec.emit('RESOURCES', limits=asdict(limits), settings=settings)
        if args.case == 'forward_spin':
            session.coordinator.local_worker._request_wrapper = DelayOneRequest(args.output)
        tick = session.coordinator.tick

        def observed(*values, **kwargs):
            result = tick(*values, **kwargs)
            rec.emit('TICK', status=result.status, events=result.events,
                     global_result=None if result.global_result is None else {
                         k: result.global_result.get(k) for k in ('planning_outcome', 'reason_code', 'has_reference',
                                                                  'requested_epoch', 'requested_goal', 'requested_version')},
                     state=asdict(values[1]), goal=values[2], generation=session.accepted_generation,
                     goal_revision=session.coordinator.goal_revision)
            return result

        session.coordinator.tick = observed
        return session, store

    params = [Parameter('build_dir', value=args.build_dir)]
    if args.case == 'legacy_tf':
        params.append(Parameter('tf_topic', value='/P4/input/map_to_odom'))
    node = PlannerNode(resource_factory=factory, parameter_overrides=params)
    previous_notice = node.runtime.notice

    def notice(kind, detail):
        rec.emit('NOTICE', notice=kind, detail=detail)
        previous_notice(kind, detail)

    node.runtime.notice = notice
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rec.emit('NATIVE_LOADED', production_sha256={
            str(Path(line.split()[-1])):hashlib.sha256(Path(line.split()[-1]).read_bytes()).hexdigest()
            for line in Path('/proc/self/maps').read_text().splitlines()
            if ('libnav1cm_' in line or 'liblunar_incremental_navigation_core.so' in line)
            and Path(line.split()[-1]).is_file()})
        result = node.stop_runtime()
        rec.emit('CLOSE', result=result, remaining_children=[p.pid for p in multiprocessing.active_children()],
                 forbidden_modules=[n for n in sys.modules if 'verification_plant' in n])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        rec.close()
        assert not result['threads_alive'], result
