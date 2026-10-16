"""Old architecture trial entry points are side-effect-free retirement errors."""
import pytest
from lunar_drl_exploration import region_query_trial


@pytest.mark.parametrize('entry', ['prepare','arm_config'])
def test_trial_entry_rejects_before_reading_or_writing(tmp_path,entry):
    source=tmp_path/'source'; source.mkdir(); (source/'identity.json').write_text('historical')
    root=tmp_path/'new'
    with pytest.raises(ValueError,match='retired.*region_relation_v2'):
        if entry=='prepare': region_query_trial.prepare(root,source_root=source)
        else: region_query_trial.arm_config(root,{})
    assert not root.exists() and (source/'identity.json').read_text()=='historical'
