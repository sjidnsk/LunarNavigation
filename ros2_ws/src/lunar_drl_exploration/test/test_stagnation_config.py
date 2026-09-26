"""Training collection limits are explicit resume identity, with legacy defaults."""
from dataclasses import replace

import pytest

from lunar_drl_exploration.config import (
    TrainingConfig, config_record, resume_semantics, training_config_from_record)


def test_legacy_config_defaults_keep_resume_identity_until_stagnation_is_enabled():
    legacy = config_record(TrainingConfig())
    for key in ('stagnation_window_steps', 'stagnation_max_positions',
                'stagnation_position_tolerance_m'):
        legacy.pop(key, None)
    disabled = training_config_from_record(legacy)
    assert 'training_stagnation' not in resume_semantics(disabled)
    enabled = replace(disabled, stagnation_window_steps=32)
    before, after = resume_semantics(disabled), resume_semantics(enabled)
    assert after.pop('training_stagnation') == dict(
        mode='observed_no_gain_pose_window_v1', window_steps=32,
        max_positions=8, position_tolerance_m=.25)
    assert before == after
    assert training_config_from_record(config_record(enabled)) == enabled


@pytest.mark.parametrize('changes', [
    {'stagnation_window_steps': -1}, {'stagnation_window_steps': 2.5},
    {'stagnation_window_steps': True}, {'stagnation_window_steps': 1},
    {'stagnation_window_steps': 8}, {'stagnation_max_positions': 0},
    {'stagnation_max_positions': 1.5}, {'stagnation_max_positions': False},
    {'stagnation_position_tolerance_m': 0},
    {'stagnation_position_tolerance_m': float('inf')},
    {'stagnation_position_tolerance_m': float('nan')},
])
def test_invalid_stagnation_limits_are_rejected_before_spawn(changes):
    with pytest.raises(ValueError, match='stagnation'):
        replace(TrainingConfig(), **changes)
