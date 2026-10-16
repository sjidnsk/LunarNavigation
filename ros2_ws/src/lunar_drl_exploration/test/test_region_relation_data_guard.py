"""Training rejects physical heldout terrain regardless of seed or split labels."""
import json
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import pytest
from lunar_drl_exploration.course_two_control import sha
from lunar_drl_exploration.region_relation_run import physical_terrain_sha256


def terrain(offset=0):
    heights=np.arange(12,dtype=np.float32).reshape(3,4)+offset
    return SimpleNamespace(shape=heights.shape,origin=(0.,0.),resolution_m=1.,heights=heights)


def write(path,value):
    path.write_text(json.dumps(value,sort_keys=True),encoding='utf-8')


@pytest.fixture
def frozen(tmp_path):
    root=tmp_path/'frozen'; root.mkdir()
    write(root/'config.json',{'seed':20261014})
    for split in ('validation','test'):
        write(root/(split+'.json'),{'split':split,'cases':[{'case_id':split+'-case','spec':{'seed':1}}]})
    identity=dict(schema='region_relation_run_identity_v1',run_id='run-one',qualified=True,
        config_sha=sha(root/'config.json'),runtime='frozen-code',
        manifests={s:sha(root/(s+'.json')) for s in ('validation','test')},
        physical_terrain_sha256_by_case={'validation-case':physical_terrain_sha256(terrain()),
            'test-case':physical_terrain_sha256(terrain(100)),
            'train-case':physical_terrain_sha256(terrain(200))})
    write(root/'identity.json',identity)
    return root,identity


def guard(root,**kwargs):
    from lunar_drl_exploration.region_relation_data_guard import TrainingTerrainGuard
    return TrainingTerrainGuard(root,runtime='frozen-code',**kwargs)


@pytest.mark.parametrize('offset',[0,100])
def test_physical_duplicate_with_different_seed_or_fake_split_rejected(frozen,offset):
    root,_=frozen
    instance=guard(root)
    with pytest.raises(ValueError,match='heldout physical terrain'):
        instance.check(terrain(offset),{'seed':99999,'split':'train','scenario_id':'novel-label'},'ep-one',0)
    assert not (root/'training-terrain-audit').exists()


def test_distinct_training_terrain_is_admitted_with_bounded_per_env_journal(frozen):
    root,identity=frozen
    instance=guard(root,config_sha=identity['config_sha'])
    actual=terrain(200)
    expected=physical_terrain_sha256(actual)
    assert instance.check(actual,{'seed':42,'split':'test','scenario_id':'x'*10000},'ep-one',0)==expected
    instance.check(terrain(201),{'seed':43,'mother_graph_id':'mother-a'},'ep-two',0)
    instance.check(terrain(202),{'seed':44},'ep-three','worker_1')
    path=root/'training-terrain-audit/env-0.jsonl'
    lines=path.read_bytes().splitlines()
    assert len(lines)==2 and all(len(line)<=1024 for line in lines)
    row=json.loads(lines[0])
    assert row['terrain_sha256']==expected and row['descriptor']['seed']==42
    assert row['episode_id']=='ep-one' and row['env_id']=='0'
    assert row['run_id']==identity['run_id'] and row['config_sha']==identity['config_sha']
    assert row['runtime']==identity['runtime']
    assert len((root/'training-terrain-audit/env-worker_1.jsonl').read_bytes().splitlines())==1
    assert instance.heldout_hashes==frozenset(identity['physical_terrain_sha256_by_case'][k]
        for k in ('validation-case','test-case'))


@pytest.mark.parametrize('bad',['qualified','config','runtime','manifest','missing_hash'])
def test_frozen_identity_changes_rejected_before_any_journal(frozen,bad):
    root,identity=frozen
    if bad=='qualified': identity['qualified']=False
    elif bad=='config': (root/'config.json').write_text('{}')
    elif bad=='runtime': identity['runtime']='different-code'
    elif bad=='manifest': write(root/'validation.json',{'split':'validation','cases':[]})
    else: del identity['physical_terrain_sha256_by_case']['test-case']
    write(root/'identity.json',identity)
    with pytest.raises(ValueError): guard(root)
    assert not (root/'training-terrain-audit').exists()


def test_supplied_config_identity_and_env_path_are_checked(frozen):
    root,_=frozen
    with pytest.raises(ValueError,match='config'): guard(root,config_sha='wrong')
    instance=guard(root)
    with pytest.raises(ValueError,match='env_id'):
        instance.check(terrain(300),{'seed':1},'ep','../../escape')
    assert not (root/'training-terrain-audit').exists()


def test_guard_never_reads_heldout_performance(frozen):
    root,_=frozen
    (root/'evaluation').mkdir(); (root/'evaluation/test-results.json').write_bytes(b'not readable json')
    assert guard(root).check(terrain(400),{'seed':3},'ep',2)==physical_terrain_sha256(terrain(400))
