import json
import signal
import subprocess

import pytest

from scripts.drl.region_contraction.pressure import budget_for_extent, build_cases, validate_terminal, close_owned


def test_area_extrapolation_exact_protocol():
    assert [budget_for_extent(x) for x in (50, 100, 150, 200, 300)] == [256, 512, 960, 1536, 3264]


@pytest.mark.parametrize('extent', [0, 49, 301, float('nan'), float('inf')])
def test_invalid_extent_rejected(extent):
    with pytest.raises(ValueError):
        budget_for_extent(extent)


def test_matrix_fixed_seeds_and_order():
    cases = build_cases()
    assert len(cases) == 20 and len({r['seed'] for r in cases}) == 20
    assert len({r['name'] for r in cases}) == 20
    assert cases[0] == dict(name='moon050-r1', family='moon', extent=50, seed=2026100600,
                           budget=256, repeat=0, mode_order=['baseline', 'remote'])
    assert cases[2]['mode_order'] == ['remote', 'baseline']
    assert cases[-1]['name'] == 'cave300-r2' and cases[-1]['budget'] == 3264


@pytest.fixture
def terminal(tmp_path):
    case = build_cases()[0]
    proof = dict(actor_sha='a', config_sha='c', source_commit='base', experiment_commit='exp',
                 wrapper_sha='w', experiment_sources={'evaluate.py': 'e', 'contraction.py': 'g'}, runtime={'package_sha':'r'})
    (tmp_path / 'dispatch.json').write_text(json.dumps(dict(case=case, mode='baseline', frozen=proof)))
    summary = dict(status='finished', error=None, mode='baseline', family='moon', extent_m=50,
                   seed=case['seed'], budget=256, ended=1, provenance=proof.copy())
    (tmp_path / 'summary.json').write_text(json.dumps(summary))
    return tmp_path, case, proof, summary


def test_missing_terminal_not_reused(terminal):
    root, case, proof, _ = terminal
    assert validate_terminal(root / 'missing', case, 'baseline', proof) is None


def test_finished_terminal_identity_checked(terminal):
    root, case, proof, summary = terminal
    assert validate_terminal(root, case, 'baseline', proof) == summary


@pytest.mark.parametrize('key', ['actor_sha', 'config_sha', 'source_commit', 'experiment_commit',
                                'wrapper_sha', 'experiment_sources', 'runtime'])
def test_terminal_source_drift_rejected(terminal, key):
    root, case, proof, summary = terminal
    summary['provenance'][key] = 'changed'
    (root / 'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='identity'):
        validate_terminal(root, case, 'baseline', proof)


def test_missing_finished_provenance_rejected(terminal):
    root, case, proof, summary = terminal
    del summary['provenance']['actor_sha']
    (root / 'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError):
        validate_terminal(root, case, 'baseline', proof)


def test_error_is_preserved_but_not_policy_finished(terminal):
    root, case, proof, _ = terminal
    summary = dict(status='error', error='InputUnavailable', mode='baseline', family='moon',
                   extent_m=50, seed=case['seed'], budget=256, ended=0)
    (root / 'summary.json').write_text(json.dumps(summary))
    assert validate_terminal(root, case, 'baseline', proof)['ended'] == 0


def test_error_without_dispatch_proof_not_reused(tmp_path):
    case = build_cases()[0]
    (tmp_path / 'summary.json').write_text(json.dumps(dict(status='error')))
    with pytest.raises(ValueError):
        validate_terminal(tmp_path, case, 'baseline', {})


class FakeProcess:
    def __init__(self, timeouts):
        self.timeouts = timeouts
        self.calls = []
    def poll(self):
        return None
    def send_signal(self, value):
        self.calls.append(('signal', value))
    def terminate(self):
        self.calls.append(('terminate',))
    def kill(self):
        self.calls.append(('kill',))
    def wait(self, timeout):
        self.calls.append(('wait', timeout))
        if self.timeouts:
            self.timeouts -= 1
            raise subprocess.TimeoutExpired('owned', timeout)
        return 0


def test_owned_cleanup_all_timeouts_are_bounded():
    process = FakeProcess(2)
    close_owned(process)
    assert process.calls == [('signal', signal.SIGINT), ('wait', 15), ('terminate',),
                             ('wait', 5), ('kill',), ('wait', 5)]


def test_owned_cleanup_does_not_signal_finished_child():
    process = FakeProcess(0)
    process.poll = lambda: 0
    close_owned(process)
    assert process.calls == []
