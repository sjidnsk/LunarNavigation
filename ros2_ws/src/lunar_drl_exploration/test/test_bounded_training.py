"""Training limits count admitted and already dispatched work together."""
from dataclasses import replace
import json

import pytest


@pytest.mark.parametrize('admitted,inflight,maximum,allowed', [
    (1000, 0, 8, True),
    (1006, 1, 8, True),
    (1007, 1, 8, False),
    (1008, 0, 8, False),
    (1007, 0, 8, True),
    (1008, 8, None, True),
])
def test_dispatch_counts_inflight_against_new_admission_limit(admitted,inflight,maximum,allowed):
    from lunar_drl_exploration.training import within_admission_budget
    assert within_admission_budget(admitted, inflight, baseline=1000, maximum=maximum) is allowed


def test_bounded_training_persists_reward_counts_and_resumes(tmp_path):
    pytest.importorskip('torch')
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration.replay import loads_transport

    config = replace(TrainingConfig(), environments=2, warmup=0,
        output_dir=str(tmp_path/'run'), artifact_root=str(tmp_path),
        total_output_max_bytes=512*1024**2, reward_diagnostics=True,
        system_reserve_bytes=0, save_interval_s=1800.)
    result = run_training(config, device='cpu', max_transitions=7,
        env_factory='worker_fixtures:ControlledEnv', probe_extent=60, probe_budget=3)
    assert result['new_transitions'] == 7
    first = json.loads((tmp_path/'run'/'reward-summary.json').read_text())
    assert first['totals']['admitted'] == 7
    assert first['totals']['reward_sum'] == pytest.approx(-.007)
    state = CheckpointManager(config.output_dir).load(config).record
    assert 'reward_diagnostics' in loads_transport(state['counters'])
    assert state['schedule']['unfinished_reservations'] == 0
    resumed = run_training(config, resume=True, device='cpu', max_transitions=3,
        env_factory='worker_fixtures:ControlledEnv', probe_extent=60, probe_budget=3)
    assert resumed['new_transitions'] == 3
    second = json.loads((tmp_path/'run'/'reward-summary.json').read_text())
    assert second['totals']['admitted'] == 10
    assert second['totals']['reward_sum'] == pytest.approx(-.010)
    assert (tmp_path/'run'/'reward-summary.json').stat().st_size <= 1024**2
    assert (tmp_path/'run'/'reward-cases.json').stat().st_size <= 16*1024**2
