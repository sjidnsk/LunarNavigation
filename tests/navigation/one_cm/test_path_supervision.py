"""监督器退出门禁：纯 mock 进程/时钟，不启动 ROS 或真实子进程。"""
import argparse
import csv
from importlib import import_module
import json
from pathlib import Path

import pytest


@pytest.mark.parametrize('early_exit', [True, False])
def test_supervisor_error_dominates_analysis_while_normal_finish_stays_pass(tmp_path, monkeypatch, early_exit):
    harness = Path(__file__).resolve().parents[3] / 'scripts/navigation/one_cm/p3_path_validation'
    monkeypatch.syspath_prepend(str(harness))
    runner = import_module('scripts.navigation.one_cm.p3_path_validation.run')
    evaluator = import_module('scripts.navigation.one_cm.p3_path_validation.evaluate')

    class Clock:
        milliseconds = 0

        def monotonic(self):
            return self.milliseconds / 1000

        def sleep(self, seconds):
            self.milliseconds += round(seconds * 1000)

    clock = Clock()

    class Process:
        def __init__(self, command, **kwargs):
            self.role = command[command.index('--role') + 1]
            start = float(command[command.index('--start') + 1])
            self.exit_at = start + (5. if early_exit and self.role == 'consumer' else 7.)
            self.pid = 101 if self.role == 'consumer' else 102
            self.returncode = None

        def poll(self):
            if clock.monotonic() >= self.exit_at:
                self.returncode = 0
            return self.returncode

        def wait(self, timeout):
            clock.milliseconds = max(clock.milliseconds, round(self.exit_at * 1000))
            self.returncode = 0
            return 0

        def send_signal(self, signum):
            self.returncode = 0

    def optimistic_analysis(folder, case, start, results, stop_s):
        # 模拟其他所有门禁通过；监督失败必须独立否决且保留这些诊断。
        assert len(results) == 2 and all(p['returncode'] == 0 for p in results)
        return dict(case=case, pass_=True, checks=dict(process_exit=True), diagnostic='preserved')

    import sys
    from types import SimpleNamespace
    monkeypatch.setitem(sys.modules,'evaluate',SimpleNamespace(
        analyze=lambda *a: (_ for _ in ()).throw(AssertionError('wrong generic evaluator'))))
    monkeypatch.setattr(runner, 'time', clock)
    monkeypatch.setattr(runner.subprocess, 'Popen', Process)
    monkeypatch.setattr(runner, 'group_pids', lambda group: [])
    monkeypatch.setattr(evaluator, 'analyze', optimistic_analysis)
    for key, value in dict(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='91', ROS_DISTRO='humble').items():
        monkeypatch.setenv(key, value)
    output = tmp_path / 'result'
    code = runner.supervise(argparse.Namespace(output=str(output), cases=['empty_smoke'], build_dir=str(tmp_path / 'unused')))
    suite = json.loads((output / 'summary.json').read_text(encoding='utf-8'))
    case = suite['cases'][0]
    assert case['diagnostic'] == 'preserved' and case['checks']['process_exit']
    if early_exit:
        assert 'premature exit 0' in (output / 'empty_smoke/supervisor-error.txt').read_text(encoding='utf-8')
        assert code != 0 and not suite['pass_'] and not case['pass_']
        assert not case['checks']['supervisor_integrity']
    else:
        assert code == 0 and suite['pass_'] and case['pass_']
        assert not (output / 'empty_smoke/supervisor-error.txt').exists()


@pytest.mark.parametrize('error_text', ["RuntimeError('empty_smoke/consumer premature exit 0')", ''])
def test_offline_analysis_rejects_existing_supervisor_error_even_exit_zero(tmp_path, error_text):
    from scripts.navigation.one_cm.p3_path_validation.evaluate import analyze
    topics = {'/Car/T3/localization/odometry': ('nav_msgs/msg/Odometry', 'nav1cm_bounded_path_consumer'),
              '/Car/T4/planning/local_path': ('nav_msgs/msg/Path', 'nav1cm_bounded_fixture')}
    graph = dict(kind='GRAPH', types={k:[v[0]] for k,v in topics.items()},
                 publishers={k:[dict(topic_type=v[0],node_name=v[1])] for k,v in topics.items()})
    rows = [graph, dict(kind='PATH_RECEIVED', stream='local', monotonic_s=2.,
                       message=dict(frame='map', stamp_ns=2_000_000_000, poses=[])),
            dict(kind='EXIT', forbidden_modules=[])]
    (tmp_path / 'consumer.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows), encoding='utf-8')
    with (tmp_path / 'state.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['monotonic_s','elapsed_s','x','y','yaw','v','w','a','alpha','mode','path_age_s','collision_free'])
        writer.writerow([2.,2.,.1,0.,0.,.2,0.,0.,0.,'translate',0.,True])
        writer.writerow([2.4,2.4,.14,0.,0.,0.,0.,0.,0.,'brake',.4,True])
    results = [dict(role=role, returncode=0, escalated=False, remaining_group_pids=[]) for role in ('consumer','fixture')]
    assert analyze(tmp_path,'empty_smoke',0.,results,None)['pass_']
    (tmp_path / 'supervisor-error.txt').write_text(error_text, encoding='utf-8')
    scored = analyze(tmp_path,'empty_smoke',0.,results,None)
    assert not scored['pass_']
    assert not scored['checks']['supervisor_integrity']
    assert scored['supervisor_error'] is not None
