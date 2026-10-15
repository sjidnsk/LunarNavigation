import torch,pytest
from lunar_drl_exploration.checkpoint import _CheckpointStream
from lunar_drl_exploration.artifacts import atomic_write
from lunar_drl_exploration.metrics import ResourceLimitError

def test_streaming_checkpoint_preserves_bytes_tensors_and_aliases(tmp_path):
    path=tmp_path/'resume.pt';tensor=torch.arange(100,dtype=torch.float32)
    record={'replay':b'x'*200000,'tensor':tensor,'view':tensor[5:9]}
    atomic_write(path,lambda f:torch.save(record,_CheckpointStream(f),pickle_protocol=5,_use_new_zipfile_serialization=False),diagnostic=False)
    assert path.read_bytes()[:2]!=b'PK'
    actual=torch.load(path,weights_only=False)
    assert actual['replay']==record['replay'] and torch.equal(actual['tensor'],tensor)
    assert actual['view'].untyped_storage().data_ptr()==actual['tensor'].untyped_storage().data_ptr()

def test_storage_budget_failure_preserves_last_good_checkpoint(tmp_path):
    path=tmp_path/'resume.pt';path.write_bytes(b'last good')
    with pytest.raises(ResourceLimitError):
        atomic_write(path,lambda f:torch.save({'tensor':torch.zeros(100000)},_CheckpointStream(f),pickle_protocol=5,_use_new_zipfile_serialization=False),max_bytes=4096,diagnostic=False)
    assert path.read_bytes()==b'last good'
    assert not list(tmp_path.glob('.*tmp*'))
