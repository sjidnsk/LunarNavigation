import gc
import weakref
import numpy as np
import pytest
from lunar_drl_exploration.replay import dumps_transport, loads_transport

@pytest.mark.parametrize('prefix', [b'', b'header'])
def test_encoder_releases_array_without_cyclic_gc(prefix):
    gc.collect();gc.disable()
    try:
        value=np.ones(1024)
        ref=weakref.ref(value)
        dumps_transport({'array':value},_prefix=prefix)
        del value
        assert ref() is None
    finally:gc.enable();gc.collect()

def test_decoder_releases_arrays_without_cyclic_gc():
    payload=dumps_transport({'array':np.ones(1024)})
    gc.collect();gc.disable()
    try:
        decoded=loads_transport(payload)
        ref=weakref.ref(decoded['array'])
        del decoded
        assert ref() is None
    finally:gc.enable();gc.collect()

def test_encoder_releases_numeric_backing_without_cyclic_gc():
    import tracemalloc
    value=np.ones(256*1024, dtype=np.float64)
    gc.collect();gc.disable();tracemalloc.start()
    try:
        for _ in range(4):
            dumps_transport(value)
        retained,_=tracemalloc.get_traced_memory()
        assert retained < value.nbytes, retained
    finally:tracemalloc.stop();gc.enable();gc.collect()

def test_controller_drops_loaded_checkpoint_before_training(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from lunar_drl_exploration import course_two_trial as trial
    class State:
        def __init__(self,n):
            self.record={'schedule':{'transitions':n,'updates':0,'credit':'0'},'payload':bytearray(1024)}
    held=[]
    def checkpoint(path):
        state=State(0 if path.name=='prefill.pt' else 4)
        if path.name!='prefill.pt':held.append(weakref.ref(state))
        return state
    monkeypatch.setattr(trial,'checkpoint',checkpoint)
    t=trial.Trial.__new__(trial.Trial);t.root=tmp_path;t.stopping=False;t.state={'results':{}}
    t.identity=dict(continuation_blocks=1,block_admissions=8192,pilot_admissions=8192,pilot_updates=2048,bounded_potential=True)
    t.prefill=lambda _:SimpleNamespace(output_dir=str(tmp_path/'run'))
    t.save=lambda **kw:None
    def spawn(*a,**kw):
        assert all(ref() is None for ref in held)
        return False
    t.spawn=spawn
    assert t.extend([]) is False


def test_controller_drops_completed_checkpoint_before_freeze_export_and_screen(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from lunar_drl_exploration import course_two_trial as trial
    from lunar_drl_exploration import evaluation
    directory=tmp_path/'arms/C3';directory.mkdir(parents=True)
    run=tmp_path/'run';run.mkdir();(run/'resume.pt').write_bytes(b'fake checkpoint')
    (tmp_path/'cases.json').write_text('[]')
    held=[];events=[];trained=False
    class State:
        def __init__(self,transitions,updates):
            self.record=dict(schedule=dict(transitions=transitions,updates=updates,credit='0'),
                payload=bytearray(1024))
    def checkpoint(path):
        state=State(0,0) if path.name=='prefill.pt' else State(8192,2048) if trained else State(4,1)
        held.append(weakref.ref(state))
        return state
    def released(event):
        assert all(ref() is None for ref in held),event
        events.append(event)
    monkeypatch.setattr(trial,'checkpoint',checkpoint)
    def freeze(source,target):
        released('freeze');target.write_bytes(source.read_bytes())
    monkeypatch.setattr(trial.shutil,'copyfile',freeze)
    def export(source,target):
        released('export');target.write_bytes(b'fake actor')
    monkeypatch.setattr(evaluation,'export_actor',export)
    monkeypatch.setattr(trial,'gate',lambda rows,baseline:dict(passed=False))
    t=trial.Trial.__new__(trial.Trial);t.root=tmp_path;t.stopping=False;t.state={'results':{}}
    t.identity=dict(continuation_blocks=1,block_admissions=8192,pilot_admissions=8192,
        pilot_updates=2048,bounded_potential=True)
    t.prefill=lambda _:SimpleNamespace(output_dir=str(run))
    t.save=lambda **values:t.state.update(values)
    def spawn(*args,**kwargs):
        nonlocal trained
        released('train');trained=True;return True
    def screen(*args):
        released('screen');return []
    t.spawn=spawn;t.screen=screen
    assert t.extend([]) is True
    assert events==['train','freeze','export','screen']
    assert len(held)==3
