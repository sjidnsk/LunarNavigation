"""Retired migrations reject before loading, changing RNG, or writing artifacts."""
import copy
from dataclasses import replace
import pytest
import torch
from lunar_drl_exploration.config import TrainingConfig
from lunar_drl_exploration.region_query_migration import migrate_learner_region_query, fork_region_query


def test_learner_migration_rejects_without_changing_record_or_rng():
    record={'model_config':{'region_query_mode':'none'},'sentinel':torch.ones(2)}
    rng=torch.get_rng_state().clone()
    with pytest.raises(ValueError,match='retired.*region_relation_v2'):
        migrate_learner_region_query(record)
    assert set(record)=={'model_config','sentinel'}
    assert record['model_config']=={'region_query_mode':'none'}
    assert torch.equal(rng,torch.get_rng_state())


def test_full_fork_rejects_before_reading_source_or_creating_output(tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    source=tmp_path/'source.pt'; source.write_bytes(b'unchanged')
    config=replace(TrainingConfig(),output_dir=str(tmp_path/'new'))
    with pytest.raises(ValueError,match='retired.*region_relation_v2'):
        TrainingState.fork_from(source,config,experiment='course-two-region-query')
    assert source.read_bytes()==b'unchanged' and not (tmp_path/'new').exists()
    with pytest.raises(ValueError,match='retired.*region_relation_v2'):
        fork_region_query(None,source,config)
