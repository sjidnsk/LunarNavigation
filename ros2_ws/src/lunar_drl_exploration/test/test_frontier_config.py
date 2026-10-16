"""The frontier retraining foundation cannot enable position auxiliary loss."""
import json

import pytest

from lunar_drl_exploration.config import (
    LearningConfig, TrainingConfig, canonical_learning_config, config_record,
    training_config_from_record,
)


def test_frontier_config_zero_position_aux_round_trips():
    config = TrainingConfig(learning=LearningConfig(
        entropy_mode='position_heading', position_aux_weight=0.0))
    record = json.loads(json.dumps(config_record(config)))
    assert record['learning']['position_aux_weight'] == 0.0
    assert training_config_from_record(record) == config
    legacy = dict(record['learning'])
    del legacy['position_aux_weight']
    assert canonical_learning_config(legacy)['position_aux_weight'] == 0.0


@pytest.mark.parametrize('weight', [0.001, 1.0, -0.001, float('nan'),
                                   float('inf'), True, '0'])
def test_frontier_config_rejects_position_aux(weight):
    # Accepting any nonzero or malformed weight would silently advertise a
    # training objective excluded by this retraining contract.
    with pytest.raises(ValueError, match='position_aux_weight.*0'):
        LearningConfig(entropy_mode='position_heading', position_aux_weight=weight)


def test_frontier_config_restore_rejects_positive_position_aux():
    record = config_record(TrainingConfig())
    record['learning']['position_aux_weight'] = 0.001
    with pytest.raises(ValueError, match='position_aux_weight.*0'):
        training_config_from_record(record)
    with pytest.raises(ValueError, match='position_aux_weight.*0'):
        canonical_learning_config(record['learning'])
