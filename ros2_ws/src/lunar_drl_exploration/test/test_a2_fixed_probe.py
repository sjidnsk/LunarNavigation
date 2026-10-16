from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts/drl/task_frontier'))


def test_fixed_rebuild_applies_same_a2_graph_and_rejects_unknown_mode():
    import pytest
    import fixed_probes as p
    from lunar_drl_exploration.contracts import Pose, SensorSpec
    from lunar_drl_exploration.decision import DecisionCore
    from lunar_drl_exploration.policy_graph import prepare_policy_graph
    from test_task_analysis import snapshot, task
    measured = np.zeros((17, 17), np.uint8)
    measured[2:15, 2:15] = 1
    core = DecisionCore(task(0, 0, 17, 17), SensorSpec(range_m=3))
    s = snapshot(measured, measured, pose=Pose(8.5, 8.5, 0))
    before = p.capture_before_observe(core, s, (0., 0.))
    native, report = core.observe(s)
    bundle = p.finish_observe(before, core, native, report)
    bundle['policy_graph'] = dict(mode='a2', max_remote_positions=32)
    actual = prepare_policy_graph(native, max_remote_positions=32).observation
    rebuilt = p.rebuild_fixed_observation(bundle)
    p.validate_exact_alignment(rebuilt, actual)
    bundle['policy_graph']['mode'] = 'unsupported'
    with pytest.raises(ValueError, match='policy graph'):
        p.rebuild_fixed_observation(bundle)
