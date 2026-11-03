"""启动互斥在Linux使用真实flock和阻塞假runtime验证。"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/simulation/start_p4.sh'
LINUX=sys.platform.startswith('linux') and shutil.which('flock') and shutil.which('bash')


@pytest.fixture
def fake_p4(tmp_path):
    if not LINUX:pytest.skip('requires Linux flock and /proc')
    root=tmp_path/'p4';(root/'config').mkdir(parents=True);(root/'run').mkdir();(root/'.venv/bin').mkdir(parents=True);(root/'source/scripts/simulation').mkdir(parents=True)
    (root/'.venv/bin/python').symlink_to(sys.executable)
    (root/'config/p4_environment.sh').write_text('unset P4_SOURCE_DIR P4_FORMAL_BUILD P4_FORMAL_COMMIT\nexport P4_TEST_ENV=loaded\n')
    runtime=root/'source/scripts/simulation/p4_tcp_runtime.py'
    runtime.write_text('import json,os,sys,time\nfrom pathlib import Path\nroot=Path(sys.argv[sys.argv.index("--root")+1])\n(root/"run"/str(os.getpid())).write_text(json.dumps([os.environ["P4_TEST_ENV"],*sys.argv]))\nwhile True:time.sleep(.05)\n')
    processes=[]
    def start(*args):
        child=subprocess.Popen(['bash',str(SCRIPT),*args],env=dict(os.environ,P4_ROOT=str(root)),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        processes.append(child);return child
    def ready(child):
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            if (root/'run'/str(child.pid)).exists():return
            if child.poll() is not None:pytest.fail(child.communicate()[1])
            time.sleep(.02)
        pytest.fail('fake runtime did not start')
    yield root,start,ready
    for child in processes:
        if child.poll() is None:child.terminate()
        child.communicate(timeout=5)


def test_first_start_blocks_second_and_stop_allows_restart(fake_p4):
    root,start,ready=fake_p4
    first=start();ready(first)
    args=json.loads((root/'run'/str(first.pid)).read_text())
    assert args[0]=='loaded' and args[-2:]==['--rviz-config',str(root/'source/scripts/simulation/p4_navigation.rviz')]
    second=start();second.wait(timeout=5);assert second.returncode!=0
    assert not (root/'run'/str(second.pid)).exists()
    first.terminate();first.wait(timeout=5)
    third=start('--no-rviz');ready(third)
    assert '--rviz-config' not in json.loads((root/'run'/str(third.pid)).read_text())


def test_concurrent_start_has_exactly_one_runtime(fake_p4):
    root,start,ready=fake_p4
    children=[start(),start()]
    deadline=time.monotonic()+5
    while all(c.poll() is None for c in children) and time.monotonic()<deadline:time.sleep(.02)
    alive=[c for c in children if c.poll() is None]
    assert len(alive)==1
    ready(alive[0]);assert len([p for p in (root/'run').iterdir() if p.name.isdigit()])==1


def test_unrelated_live_pointer_does_not_block(fake_p4):
    root,start,ready=fake_p4
    (root/'run/p4-current.json').write_text(json.dumps(dict(operator_pid=os.getpid(),run=str(root/'run/old'))))
    child=start();ready(child)


def test_deployment_pointer_blocks_matching_existing_runtime_without_lock(fake_p4):
    root,start,ready=fake_p4
    foreign=subprocess.Popen([sys.executable,str(root/'source/scripts/simulation/p4_tcp_runtime.py'),'--root',str(root),'--artifact',str(root/'run/deployed')],env=dict(os.environ,P4_TEST_ENV='loaded'))
    try:
        ready(foreign)
        (root/'run/p4-current.json').write_text(json.dumps(dict(operator_pid=foreign.pid,run=str(root/'run/deployed'))))
        child=start();child.wait(timeout=5);assert child.returncode!=0
        assert not (root/'run'/str(child.pid)).exists()
    finally:foreign.terminate();foreign.wait(timeout=5)


def test_invalid_option_does_not_start_runtime(fake_p4):
    root,start,ready=fake_p4
    child=start('--unexpected');child.wait(timeout=5)
    assert child.returncode!=0 and not (root/'run'/str(child.pid)).exists()


@pytest.mark.parametrize('default_root',[False,True])
def test_pointer_probe_ignores_reused_and_zombie_pid_but_detects_runtime(tmp_path,monkeypatch,default_root):
    monkeypatch.delenv('P4_SOURCE_DIR',raising=False)
    source=SCRIPT.read_text(encoding='utf-8')
    probe=source.split("<<'PY'\n",1)[1].split('\nPY',1)[0]
    root=Path('/home/yanfa/P4').resolve() if default_root else tmp_path/'p4'
    pointer=tmp_path/'p4-current.json';pointer.write_text(json.dumps(dict(operator_pid=42,run=str(root/'run/deployed'))))
    proc=tmp_path/'proc';(proc/'42').mkdir(parents=True)
    original_read=Path.read_text;original_bytes=Path.read_bytes
    def mapped(path):
        value=str(path).replace('\\','/')
        return proc/value.removeprefix('/proc/') if value.startswith('/proc/') else path
    monkeypatch.setattr(Path,'read_text',lambda self,*a,**kw:original_read(mapped(self),*a,**kw))
    monkeypatch.setattr(Path,'read_bytes',lambda self:original_bytes(mapped(self)))
    monkeypatch.setattr(sys,'argv',['-',str(pointer),str(root)])
    def process(state,command):
        (proc/'42/stat').write_text('42 (python) '+state+' 1 0')
        (proc/'42/cmdline').write_bytes(b'\0'.join(x.encode() for x in command)+b'\0')
    actual=[sys.executable,str(root/'source/scripts/simulation/p4_tcp_runtime.py')]
    if not default_root:actual+=['--root',str(root)]
    actual+=['--artifact',str(root/'run/deployed')]
    process('S',actual)
    with pytest.raises(SystemExit):exec(compile(probe,'<pid-probe>','exec'),{})
    process('Z',actual);exec(compile(probe,'<pid-probe>','exec'),{})
    process('S',[sys.executable,'other.py']);exec(compile(probe,'<pid-probe>','exec'),{})



def test_selected_build_supplies_runtime_and_rviz_without_editing_legacy_source(fake_p4):
    root,start,ready=fake_p4
    selected=root/'build/formal-test/source'
    folder=selected/'scripts/simulation';folder.mkdir(parents=True)
    old=root/'source/scripts/simulation/p4_tcp_runtime.py'
    (folder/'p4_tcp_runtime.py').write_bytes(old.read_bytes())
    (root/'config/p4_environment.sh').write_text(
        'export P4_TEST_ENV=loaded\nexport P4_SOURCE_DIR="'+str(selected)+'"\n')
    child=start();ready(child)
    args=json.loads((root/'run'/str(child.pid)).read_text())
    assert args[1]==str(folder/'p4_tcp_runtime.py')
    assert args[-1]==str(folder/'p4_navigation.rviz')
    assert old.exists()



def test_pointer_blocks_unlocked_old_operator_from_other_source(fake_p4):
    root,start,ready=fake_p4
    legacy=root/'source/scripts/simulation/p4_tcp_runtime.py'
    selected=root/'build/formal-new/source/scripts/simulation';selected.mkdir(parents=True)
    (selected/'p4_tcp_runtime.py').write_bytes(legacy.read_bytes())
    (root/'config/p4_environment.sh').write_text('export P4_TEST_ENV=loaded\nexport P4_SOURCE_DIR="'+str(selected.parents[1])+'"\n')
    foreign=subprocess.Popen([sys.executable,str(legacy),'--root',str(root),'--artifact',str(root/'run/legacy')],env=dict(os.environ,P4_TEST_ENV='loaded'))
    try:
        ready(foreign)
        (root/'run/p4-current.json').write_text(json.dumps(dict(operator_pid=foreign.pid)))
        child=start();child.wait(timeout=5)
        assert child.returncode!=0
        assert not (root/'run'/str(child.pid)).exists()
    finally:
        foreign.terminate();foreign.wait(timeout=5)
