"""Read-only smoke telemetry must not change numerical state or disabled runs."""
import importlib.util
import json
from pathlib import Path
import sys
import torch


def load_entry():
    path = Path(__file__).resolve().parents[4] / 'scripts/drl/task_frontier/train_live.py'
    spec = importlib.util.spec_from_file_location('smoke_train_entry', path)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    return entry


def test_disabled_hook_keeps_original_methods(monkeypatch):
    from lunar_drl_exploration.sac import SACLearner
    original = SACLearner.update
    monkeypatch.delenv('DRL_SMOKE_TELEMETRY', raising=False)
    assert load_entry().install_smoke_telemetry() is None
    assert SACLearner.update is original


def test_enabled_readonly_snapshot_preserves_rng_and_records_actual_pid(tmp_path):
    import os
    entry = load_entry()
    path = tmp_path / 'telemetry.jsonl'
    before = torch.get_rng_state().clone()
    result = entry.smoke_snapshot(path, 'test', updates=3, transitions=76)
    assert result is None
    assert torch.equal(before, torch.get_rng_state())
    row = json.loads(path.read_text())
    assert row['pid'] == os.getpid()
    assert row['updates'] == 3 and row['transitions'] == 76
    assert row['torch_cpu_rng_sha256']


def test_enabled_real_update_keeps_return_state_and_rng(tmp_path, monkeypatch):
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.config import ModelConfig, LearningConfig
    from test_sac import transitions, scene
    entry = load_entry()
    path = tmp_path / 'telemetry.jsonl'
    torch.manual_seed(334)
    learner = SACLearner(ModelConfig(width=16, heads=4, layers=1), LearningConfig(), device='cpu')
    initial = learner.state_dict()
    samples = transitions()
    expected = learner.update(samples, {'s': scene()})
    expected_rng = torch.get_rng_state().clone()
    expected_state = learner.state_dict()
    monkeypatch.setenv('DRL_SMOKE_TELEMETRY', str(path))
    undo = entry.install_smoke_telemetry()
    try:
        other = SACLearner(ModelConfig(width=16, heads=4, layers=1), LearningConfig(), device='cpu')
        other.load_state_dict(initial)
        actual = other.update(samples, {'s': scene()})
        assert actual == expected
        # Model construction consumes random numbers; the update itself does not.
        before = torch.get_rng_state().clone()
        entry.smoke_snapshot(path, 'after-test', updates=other.updates)
        assert torch.equal(before, torch.get_rng_state())
        for name in ('actor', 'q1', 'q2', 'target1', 'target2'):
            for key, value in expected_state[name].items():
                assert torch.equal(value, other.state_dict()[name][key])
        assert other.updates == 1
        assert [json.loads(s)['event'] for s in path.read_text().splitlines()][:2] == ['model-created', 'update']
    finally:
        undo()
