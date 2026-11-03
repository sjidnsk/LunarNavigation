"""构建选择仅影响部署入口，不进入导航任务或控制判定。"""
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/simulation/p4_build_selection.py'


def api():
    assert SCRIPT.is_file(), '缺少整套正式构建选择与回退入口'
    spec=importlib.util.spec_from_file_location('p4_build_selection',SCRIPT)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def make_build(root,name,*,qualified=True):
    import hashlib
    target=root/'build'/name
    (target/'source').mkdir(parents=True)
    (target/'install').mkdir()
    (target/'install/local_setup.bash').write_text('# test install\n')
    env=target/'environment.sh';env.write_text('export BUILD_TEST='+name+'\n')
    digest=hashlib.sha256(env.read_bytes()).hexdigest()
    (target/'manifest.json').write_text(json.dumps(dict(qualified=qualified,
        environment_sha256=digest,files={'environment.sh':digest})))
    return target


def test_invalid_or_modified_build_cannot_be_selected(tmp_path):
    module=api();root=tmp_path/'P4';target=make_build(root,'formal-new',qualified=False)
    with pytest.raises(ValueError,match='qualified'):module.validate_build(root,target)
    manifest=target/'manifest.json';data=json.loads(manifest.read_text());data['qualified']=True
    manifest.write_text(json.dumps(data));(target/'environment.sh').write_text('changed')
    with pytest.raises(ValueError,match='digest'):module.validate_build(root,target)


def test_build_outside_p4_is_rejected(tmp_path):
    module=api();root=tmp_path/'P4';root.mkdir()
    target=make_build(tmp_path/'outside','formal-new')
    with pytest.raises(ValueError,match='outside'):module.validate_build(root,target)


def test_manifest_file_cannot_escape_build(tmp_path):
    module=api();root=tmp_path/'P4';target=make_build(root,'formal-new')
    p=target/'manifest.json';data=json.loads(p.read_text());data['files']['../outside']='bad'
    p.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='outside'):module.validate_build(root,target)


@pytest.mark.skipif(sys.platform!='linux',reason='real Linux flock deployment')
def test_switch_and_rollback_change_one_selection_atomically(tmp_path):
    module=api();root=tmp_path/'P4'
    first=make_build(root,'formal-first');second=make_build(root,'formal-second')
    module.switch_build(root,target=first)
    module.switch_build(root,target=second)
    state=json.loads((root/'config/p4_active_build.json').read_text())
    assert state==dict(active=str(second),previous=str(first))
    module.switch_build(root,previous=True)
    state=json.loads((root/'config/p4_active_build.json').read_text())
    assert state==dict(active=str(first),previous=str(second))
    module.switch_build(root,target=first)
    assert json.loads((root/'config/p4_active_build.json').read_text())==state
    assert first.is_dir() and second.is_dir()


@pytest.mark.skipif(sys.platform!='linux',reason='real Linux flock deployment')
def test_switch_refuses_live_start_lock_without_changing_selection(tmp_path):
    import fcntl
    module=api();root=tmp_path/'P4'
    first=make_build(root,'formal-first');second=make_build(root,'formal-second')
    module.switch_build(root,target=first)
    original=(root/'config/p4_active_build.json').read_bytes()
    with (root/'run/p4-start.lock').open('a') as held:
        fcntl.flock(held,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with pytest.raises(RuntimeError,match='P4'):module.switch_build(root,target=second)
    assert (root/'config/p4_active_build.json').read_bytes()==original
