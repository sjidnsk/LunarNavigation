import json
import hashlib
from collections import Counter
from pathlib import Path
import shutil
import pytest
from lunar_drl_exploration import region_relation_prepare as module
from lunar_drl_exploration.course_two_control import read,write


@pytest.fixture(scope='module')
def prepared(tmp_path_factory):
    patch=pytest.MonkeyPatch()
    patch.setattr(module,'runtime_digest',lambda:'fixture-runtime')
    from dataclasses import replace
    make_config=module.make_config
    patch.setattr(module,'make_config',lambda *a:replace(make_config(*a),
        course_two_scene_tasks=('moon_outside','moon_inside','cave_inside'),moon_entry_curriculum=''))
    directory=tmp_path_factory.mktemp('prepared')
    protocol=directory/'protocol.json';protocol.write_text('{"protocol_id":"approved"}')
    root=directory/'run';module.prepare(root,protocol)
    yield root,protocol
    patch.undo()


@pytest.fixture
def cloned(prepared,tmp_path):
    root=tmp_path/'run';shutil.copytree(prepared[0],root)
    from lunar_drl_exploration.artifacts import MANIFEST
    marker=read(root/MANIFEST);marker['root']=str(root);(root/MANIFEST).write_text(json.dumps(marker))
    pending=read(root/'preparation.json');pending['root']=str(root);(root/'preparation.json').write_text(json.dumps(pending))
    return root


def test_exact_counts_anchors_and_preallocated_backups(prepared):
    root,_=prepared
    for split,count in [('validation',60),('test',100)]:
        requests=read(root/f'{split}-requests.json')['requests']
        assert len(requests)==count*3
        for task in module.TASKS:
            group=[r for r in requests if r['case']['capability']==task]
            assert len(group)==count
            anchors=0
            for request in group:
                specs=request['backup_specs'];assert len(specs)==8
                assert len({s['seed'] for s in specs})==8
                assert all(s['sensor']==specs[0]['sensor'] for s in specs)
                anchors+=int(specs[0]['sensor']=={'range_m':10.,'fov_deg':90.})
                if task=='cave_inside':
                    assert all(s['mother_graph_id']==specs[0]['mother_graph_id'] and s['dataset_split']==split for s in specs)
            assert anchors==count//5
    train=read(root/'train-cave-requests.json')['requests']
    assert len(train)==64
    train_mothers={r['case']['spec']['mother_graph_id'] for r in train}
    assert len(train_mothers)==64
    val={r['case']['spec']['mother_graph_id'] for r in read(root/'validation-requests.json')['requests'] if r['case']['capability']=='cave_inside'}
    test={r['case']['spec']['mother_graph_id'] for r in read(root/'test-requests.json')['requests'] if r['case']['capability']=='cave_inside'}
    assert not train_mothers&val and not val&test and not train_mothers&test
    assert not (root/'readiness.json').exists()


def fake_factory(root,slots,*,failed=False):
    pending=read(root/'preparation.json')
    class Process:
        def __init__(self,args,**kwargs):
            self.slot=int(args[args.index('--slot')+1]);slots.append(self.slot)
            request=read(args[args.index('--request')+1])
            output=Path(args[args.index('--output')+1])
            case=dict(request['case'],spec=request['backup_specs'][1])
            evidence=dict(initial_completed=False,collision=False,initial_coverage=.1,
                          actions=1,reference_area_m2=10,inside=case['capability']!='moon_outside',
                          terrain_sha256=hashlib.sha256(case['case_id'].encode()).hexdigest())
            output.write_text(json.dumps(dict(status='failed' if failed else 'passed',accepted_attempt=1,case=case,evidence=evidence,
                proof=dict(request=request,config_sha=pending['config_sha'],runtime=pending['runtime']))))
        def poll(self):return 0
    return Process


def test_parallel_qualification_freezes_exact_quick_subset(cloned):
    slots=[];identity=module.qualify(cloned,popen_factory=fake_factory(cloned,slots))
    assert set(slots)==set(range(8)) and len(slots)==544
    assert identity['qualified']
    val=read(cloned/'validation.json')['cases'];quick=read(cloned/'quick.json')['cases']
    assert quick==[case for task in module.TASKS for case in [c for c in val if c['capability']==task][:20]]
    assert len(quick)==60 and len(read(cloned/'test.json')['cases'])==300
    assert read(cloned/'state.json')['completed_blocks']==0
    assert not (cloned/'readiness.json').exists()
    assert module.qualify(cloned,popen_factory=lambda *a,**k:pytest.fail('no relaunch'))==identity


def test_changed_source_rejects_cache_before_any_launch(cloned,monkeypatch):
    monkeypatch.setattr(module,'runtime_digest',lambda:'changed')
    with pytest.raises(ValueError,match='source/config changed'):
        module.qualify(cloned,popen_factory=lambda *a,**k:pytest.fail('must not launch'))


def test_failed_qualification_never_freezes(cloned):
    with pytest.raises(ValueError,match='cache failed'):
        module.qualify(cloned,popen_factory=fake_factory(cloned,[],failed=True))
    assert not (cloned/'identity.json').exists()
    assert not (cloned/'validation.json').exists()


def test_refuses_nonempty_fresh_output(tmp_path):
    (tmp_path/'unknown.txt').write_text('owned by someone else')
    protocol=tmp_path/'proto.json';protocol.write_text('{}')
    with pytest.raises(ValueError,match='nonempty unmanaged'):
        module.prepare(tmp_path,protocol)


def test_refuses_changed_immutable_request(cloned):
    request=cloned/'validation-requests.json';request.write_text('{}')
    with pytest.raises(ValueError,match='input changed'):
        module.qualify(cloned,popen_factory=lambda *a,**k:pytest.fail('must not launch'))


def test_cached_proof_change_rejected_before_subprocess(cloned):
    cid=read(cloned/'preparation.json')['request_ids'][0]
    factory=fake_factory(cloned,[])
    factory(['--request',str(cloned/'requests'/f'{cid}.json'),'--output',str(cloned/'qualification'/f'{cid}.json'),'--slot','0'])
    path=cloned/'qualification'/f'{cid}.json';record=read(path)
    record['proof']['runtime']='foreign-source';path.write_text(json.dumps(record))
    with pytest.raises(ValueError,match='cache failed or source changed'):
        module.qualify(cloned,popen_factory=lambda *a,**k:pytest.fail('must not relaunch bad cache'))


def test_heldout_stratified_marginals_and_fixed_backups(prepared):
    root,_=prepared
    for split,count in [('validation',60),('test',100)]:
        requests=read(root/f'{split}-requests.json')['requests']
        for task in module.TASKS:
            group=[r for r in requests if r['case']['capability']==task]
            extents=Counter(r['backup_specs'][0]['extent'] for r in group)
            assert len(extents)==5 and set(extents.values())=={count//5}
            expanded=[r['backup_specs'][0] for i,r in enumerate(group) if i%5]
            for key in ('range_m','fov_deg'):
                histogram=Counter(s['sensor'][key] for s in expanded)
                assert max(histogram.values())-min(histogram.values())<=1
            for request in group:
                specs=request['backup_specs']
                assert all(s['extent']==specs[0]['extent'] and s['episode_budget']==specs[0]['episode_budget'] for s in specs)
            if task=='moon_outside':
                bands=Counter((r['backup_specs'][0]['outside_distance_min_m'],r['backup_specs'][0]['outside_distance_max_m']) for r in group)
                assert len(bands)==4 and set(bands.values())=={count//4}
                for r in group:
                    assert all(s['outside_distance_min_m']==r['backup_specs'][0]['outside_distance_min_m'] and s['outside_distance_max_m']==r['backup_specs'][0]['outside_distance_max_m'] for s in r['backup_specs'])


def test_duplicate_physical_terrain_prevents_freeze(cloned):
    factory=fake_factory(cloned,[])
    class Duplicate(factory):
        def __init__(self,args,**kwargs):
            super().__init__(args,**kwargs)
            path=Path(args[args.index('--output')+1]);result=read(path)
            if result['case']['case_id'] in ('validation-moon_inside-000','test-moon_inside-000'):
                result['evidence']['terrain_sha256']='a'*64
                path.write_text(json.dumps(result))
    with pytest.raises(ValueError,match='duplicate physical terrain'):
        module.qualify(cloned,popen_factory=Duplicate)
    assert not (cloned/'identity.json').exists()
    assert not (cloned/'validation.json').exists()
