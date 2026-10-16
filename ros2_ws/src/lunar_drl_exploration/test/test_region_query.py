"""Old additive/offset paths are removed from the only supported architecture."""
import inspect
import pytest
import torch
from lunar_drl_exploration.config import ModelConfig, canonical_model_config
from lunar_drl_exploration.model import Actor, Critic, CurrentRead, RegionRead
from test_model import observation


def test_no_legacy_mode_or_offset_branch():
    assert 'region_query_mode' not in ModelConfig.__dataclass_fields__
    assert list(inspect.signature(CurrentRead.forward).parameters)==['self','query','nodes','groups']
    for kind in (Actor,Critic):
        network=kind()
        assert not any('region_query' in key for key in network.state_dict())
        assert sum(isinstance(m,RegionRead) for m in network.modules())==1
    with pytest.raises(ValueError,match='region_query'):
        canonical_model_config(dict(architecture_version='region_relation_v2',region_query_mode='none'))


def test_polygon_cyclic_permutation_preserves_policy():
    from dataclasses import replace
    obs=observation()
    actor=Actor().eval()
    with torch.no_grad():
        left=actor([obs])[0]; right=actor([replace(obs,polygon=obs.polygon[[2,3,4,0,1]])])[0]
    torch.testing.assert_close(left.logits,right.logits,atol=2e-6,rtol=2e-5)
    assert torch.isfinite(left.probs).all()
    torch.testing.assert_close(left.probs.sum(),torch.tensor(1.))
