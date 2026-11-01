"""隔离观测包装必须保持指令、warm start、进度与RNG完全一致。"""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('points',[
    ((0.,0.,0.),(.12,0.,0.)),
    ((0.,0.,0.),(-.12,0.,0.),(-.12,0.,-.01)),
    ((0.,0.,0.),(0.,0.,.4)),
])
def test_trace_preserves_solver_state_and_outputs(monkeypatch,tmp_path,points):
    monkeypatch.syspath_prepend(str(ROOT/'ros2_ws/src/lunar_pure_wheeled_controller/python'))
    if 'lunar_planning_msgs.msg' not in sys.modules:
        msg=ModuleType('lunar_planning_msgs.msg');msg.MotionReference=type('MotionReference',(),{})
        monkeypatch.setitem(sys.modules,'lunar_planning_msgs',ModuleType('lunar_planning_msgs'))
        monkeypatch.setitem(sys.modules,'lunar_planning_msgs.msg',msg)
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower
    from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState
    from lunar_pure_wheeled_controller.mppi import MppiPolicy
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    spec=importlib.util.spec_from_file_location('solver_trace',ROOT/'scripts/drl/trace_standard_path_solver.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    def run():
        e=StandardPathFollower(TrackingPolicy(motion_model='differential_drive',goal_position_tolerance_m=.01),
            MppiPolicy(batch_size=12,time_steps=12,nominal_response_time_s=0.))
        e.set_map(GridMap(np.ones((30,30),np.uint8),.1,(-1.,-1.)))
        e.set_standard_path(points);e.no_progress_s=14.85
        results=[]
        for i in range(4):
            r=e.update(TrackingState(0.,0.,0.),.05,now_s=i*.05)
            e.note_command(r.command.linear_x_mps,r.command.angular_z_radps)
            results.append((r,e._nominal.copy(),e.progress_m,e.no_progress_s,json.dumps(e.rng.bit_generator.state)))
            e.set_standard_path(points)
        return results
    expected=run();path=tmp_path/'trace.jsonl';uninstall=module.install(path)
    try:actual=run()
    finally:uninstall()
    for a,b in zip(expected,actual):
        assert a[0]==b[0] and a[2:]==b[2:]
        np.testing.assert_array_equal(a[1],b[1])
    rows=[json.loads(line) for line in path.read_text().splitlines()]
    scores=[r for r in rows if r['event']=='score'];assert scores
    for r in scores:
        np.testing.assert_allclose(np.array(r['endpoint'])+r['lateral']+r['yaw']+r['smooth'],r['cost'],rtol=1e-14,atol=1e-12)
        assert abs(sum(r['weights'])-1)<1e-12
    assert any(r['event']=='result' and r['context']['failure']=='TRACKING_STALLED' for r in rows)
    assert path.with_suffix('.map.npz').exists()
    with np.load(path.with_suffix('.map.npz')) as saved:
        np.testing.assert_array_equal(saved['states'],np.ones((30,30),np.uint8))
