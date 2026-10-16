import json
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from scripts.drl.region_contraction import pressure_plot, pressure_report


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')


def fixture(tmp_path):
    cases = [dict(name='one', family='moon', extent=50, seed=1, budget=256),
             dict(name='two', family='moon', extent=50, seed=2, budget=256)]
    save(tmp_path/'protocol.json', dict(cases=cases,modes=['baseline','remote'],frozen={}))
    save(tmp_path/'run-state.json',dict(status='running',updated_unix=1))
    for mode in ['baseline','remote']:
        save(tmp_path/'one'/mode/'summary.json', dict(status='finished',mode=mode,
            family='moon',extent_m=50,seed=1,budget=256,completed=True,collisions=0,
            final_coverage=.99,decisions=10,distance_m=40,
            timing={'episode_wall_seconds':100},
            graph={'mean_fine_nodes':100,'mean_policy_nodes':50,'node_ratio':.5,'edge_ratio':.4},
            provenance=dict(actor_sha='a',pairing_config_digest='cfg',terrain_id='t',reference_id='r',
                semantic_id='s',polygon_digest='p',initial_pose=[0,0,0],pairing_digest='pair',config_sha='c',
                source_commit='commit',source_root='/source',wrapper_sha='wrap',
                runtime={'package_sha':'rt'},experiment_sources={'contraction.py':'frozen'})))
    calls=[]
    def plotter(root):
        calls.append(1)
        return pressure_plot.collect(root)[0]
    return tmp_path,calls,plotter


def test_no_policy_progress_or_resource_only_change_does_not_redraw(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    supervisor=pressure_report.Supervisor(root,plotter=plotter)
    assert supervisor.tick()['plot_updated']
    save(root/'run-state.json',dict(status='running',updated_unix=999))
    save(root/'maxima.json',dict(schema='pressure_resource_maxima_v1',root=str(root),samples=10,
        cases={'one/remote':dict(samples=3,complete_samples=3,rejected_samples=0,rss_max_kib=1024,pss_max_kib=None)}))
    assert not supervisor.tick()['plot_updated']
    assert len(calls)==1
    report=(root/'pressure-report.md').read_text()
    assert '进行中' in report
    assert 'PSS' in report and '未知' in report
    assert '50.00%' in report


def test_partial_tail_alone_is_not_progress_but_complete_decision_is(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    supervisor=pressure_report.Supervisor(root,plotter=plotter)
    supervisor.tick()
    path=root/'two'/'baseline'/'steps.jsonl'
    path.parent.mkdir(parents=True)
    path.write_text('{"step":1,"coverage":')
    assert not supervisor.tick()['plot_updated']
    path.write_text('{"step":1,"coverage":0.2}\n')
    assert supervisor.tick()['plot_updated']
    assert len(calls)==2


def test_complete_bad_json_is_not_ignored(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    supervisor=pressure_report.Supervisor(root,plotter=plotter)
    supervisor.tick()
    path=root/'two'/'baseline'/'steps.jsonl'
    path.parent.mkdir(parents=True)
    path.write_text('{bad}\n')
    with pytest.raises(ValueError,match='complete'):
        supervisor.tick()


def test_terminal_dispatcher_forces_last_plot_and_incomplete_means_interrupted(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    supervisor=pressure_report.Supervisor(root,plotter=plotter)
    supervisor.tick()
    save(root/'run-state.json',dict(status='error',error='dispatcher failed'))
    result=supervisor.tick()
    assert result['done'] and result['status']=='异常'
    assert len(calls)==2
    report=(root/'pressure-report.md').read_text()
    assert '实验中断' in report and '2/4' in report
    assert '—' in report


def test_all_episode_data_waits_for_dispatcher_terminal(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    for mode in ['baseline','remote']:
        row=json.loads((root/'one'/mode/'summary.json').read_text())
        row.update(seed=2)
        row['provenance'].update(terrain_id='t2',reference_id='r2',pairing_digest='pair2')
        save(root/'two'/mode/'summary.json',row)
    supervisor=pressure_report.Supervisor(root,plotter=plotter)
    assert not supervisor.tick()['done']
    save(root/'run-state.json',dict(status='finished'))
    result=supervisor.tick()
    assert result['done'] and result['status']=='完成' and len(calls)==2
    assert '不能据此断言 A2 在所有场景普遍更好' in (root/'pressure-report.md').read_text()


def test_wrong_root_resource_data_rejected(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    save(root/'maxima.json',dict(schema='pressure_resource_maxima_v1',root='/wrong',cases={}))
    with pytest.raises(ValueError,match='资源.*root'):
        pressure_report.Supervisor(root,plotter=plotter).tick()


def test_stable_bad_document_raises_and_retry_can_recover_race(tmp_path):
    path=tmp_path/'document.json'
    path.write_text('{bad}\n')
    with pytest.raises(ValueError,match='完整 JSON'):
        pressure_report.read_json(path,pause=lambda _:None)
    def repair(_):
        save(path,dict(ok=True))
    assert pressure_report.read_json(path,pause=repair)=={'ok':True}


def test_watch_waits_60_seconds_and_exits_on_dispatcher_error(tmp_path,monkeypatch):
    root,calls,plotter=fixture(tmp_path)
    sleeps=[]
    def wait(seconds):
        sleeps.append(seconds)
        save(root/'run-state.json',dict(status='error',error='interrupted'))
    monkeypatch.setattr(pressure_report.pressure_plot,'run',plotter)
    monkeypatch.setattr(pressure_report.time,'sleep',wait)
    assert pressure_report.main(['--root',str(root),'--watch'])==1
    assert sleeps==[60.]
    assert len(calls)==2
    assert '实验中断' in (root/'pressure-report.md').read_text()


def test_cli_bad_complete_record_writes_explicit_failure_report(tmp_path,monkeypatch):
    root,calls,plotter=fixture(tmp_path)
    path=root/'two'/'remote'/'steps.jsonl'
    path.parent.mkdir(parents=True)
    path.write_text('{bad}\n')
    monkeypatch.setattr(pressure_report.pressure_plot,'run',plotter)
    assert pressure_report.main(['--root',str(root),'--once'])==1
    text=(root/'pressure-report.md').read_text()
    assert '状态：异常' in text and 'complete' in text and '旧图/指标' in text
    assert calls==[]


def test_failed_endpoint_not_reported_as_success_efficiency(tmp_path):
    root,calls,plotter=fixture(tmp_path)
    row=json.loads((root/'one'/'baseline'/'summary.json').read_text())
    row.update(completed=False,final_coverage=.5,truncated=True)
    row['timing']['episode_wall_seconds']=99999.
    save(root/'one'/'baseline'/'summary.json',row)
    result=pressure_report.Supervisor(root,plotter=plotter).tick()
    group=result['metrics']['strata'][0]['modes']['baseline']
    assert group['successes']==0 and group['ended']==1
    assert group['both_successful_ended']==0 and group['both_successful_wall_seconds'] is None
    text=(root/'pressure-report.md').read_text()
    success_table=text.split('## 双方均成功的描述性效率',1)[1].split('## 图规模与压缩',1)[0]
    assert '99999' not in success_table
    assert '—（未知）' in success_table
