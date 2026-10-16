import json
from pathlib import Path
import sys

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from scripts.drl.region_contraction import pressure_plot


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def case(name, seed):
    return dict(name=name, family='moon', extent=50, seed=seed, budget=256)


def summary(mode, seed, completed=True, **changes):
    row = dict(mode=mode, family='moon', extent_m=50., seed=seed, budget=256,
        status='finished', completed=completed, collisions=0, error=None,
        final_coverage=.99 if completed else .6, decisions=20, distance_m=40.,
        zero_gain_ratio=.2, max_zero_gain_run=2, navigation_failures=0,
        timing={'episode_wall_seconds': 100.}, graph={'mean_fine_nodes': 200., 'mean_policy_nodes': 100.},
        provenance=dict(actor_sha='a', pairing_config_digest='cfg', terrain_id=f'terrain-{seed}',
            reference_id=f'ref-{seed}', semantic_id='sem', polygon_digest='poly', initial_pose=[0.,0.,0.],
            pairing_digest=f'pair-{seed}', config_sha='cfg-file', source_commit='commit',
            source_root='/source', wrapper_sha='wrapper', experiment_sources={'contraction.py':'core'},
            runtime={'package_sha':'runtime'}))
    row.update(changes)
    return row


def setup_root(tmp_path):
    save(tmp_path / 'protocol.json', dict(cases=[case('finished', 1), case('pending', 2)],
        modes=['baseline', 'remote'], frozen={}))
    for mode in ('baseline', 'remote'):
        save(tmp_path / 'finished' / mode / 'summary.json', summary(mode, 1))
    return tmp_path


def test_pending_is_unknown_and_full_protocol_strata_keep_planned_denominator(tmp_path):
    root = setup_root(tmp_path)
    metrics, curves = pressure_plot.collect(root)
    assert metrics['expected_episodes'] == 4
    assert metrics['terminal_episodes'] == 2
    assert metrics['phase'] == 'partial'
    assert metrics['strata'][0]['planned_cases'] == 2
    assert metrics['strata'][0]['paired_cases'] == ['finished']
    assert metrics['strata'][0]['modes']['baseline']['successes'] == 1
    assert metrics['strata'][0]['modes']['baseline']['ended'] == 1
    assert metrics['strata'][0]['modes']['baseline']['success_rate'] == 1.
    pending = next(row for row in metrics['episodes'] if row['case'] == 'pending' and row['mode'] == 'baseline')
    assert pending['status'] == 'pending'
    assert pending['final_coverage'] is None
    assert pending['wall_seconds'] is None


def test_failed_wall_is_not_averaged_into_successful_efficiency(tmp_path):
    root = setup_root(tmp_path)
    save(root / 'finished' / 'baseline' / 'summary.json',
         summary('baseline', 1, completed=False, truncated=True,
                 timing={'episode_wall_seconds': 99999.}))
    metrics, curves = pressure_plot.collect(root)
    baseline = metrics['strata'][0]['modes']['baseline']
    assert baseline['ended'] == 1
    assert baseline['successes'] == 0
    assert baseline['both_successful_wall_seconds'] is None
    assert baseline['both_successful_ended'] == 0


def test_curves_stream_complete_jsonl_records_and_discard_unfinished_tail(tmp_path):
    path = tmp_path / 'steps.jsonl'
    with path.open('w') as stream:
        for step in range(1, 1301):
            stream.write(json.dumps(dict(step=step, coverage=step/2000, unused_large_field='x'*200)) + '\n')
        stream.write('{"step": 1301, "coverage":')
    curve = pressure_plot.read_curve(path, max_points=64)
    assert curve['read_records'] == 1300
    assert curve['partial_tail'] is True
    assert curve['points'][0] == [1, .0005]
    assert curve['points'][-1] == [1300, .65]
    assert len(curve['points']) <= 64


def test_complete_malformed_record_is_visible_instead_of_silently_skipped(tmp_path):
    path = tmp_path / 'steps.jsonl'
    path.write_text('{bad}\n')
    with pytest.raises(ValueError, match='complete'):
        pressure_plot.read_curve(path)


def test_run_generates_two_images_and_readable_metrics_for_partial_data(tmp_path):
    root = setup_root(tmp_path)
    metrics = pressure_plot.run(root)
    assert metrics['phase'] == 'partial'
    for name in ('pressure-comparison.png', 'pressure-coverage.png'):
        assert (root / name).read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
    saved = json.loads((root / 'pressure-metrics.json').read_text())
    assert saved['terminal_episodes'] == 2
    assert (root / 'pressure-metrics.csv').is_file()
    assert 'pending' in (root / 'pressure-metrics.txt').read_text()


def test_planned_identity_mismatch_cannot_be_plotted_as_pair(tmp_path):
    root = setup_root(tmp_path)
    for mode in ('baseline', 'remote'):
        save(root / 'finished' / mode / 'summary.json', summary(mode, 99))
    with pytest.raises(ValueError, match='protocol seed'):
        pressure_plot.collect(root)


def test_runtime_mismatch_is_rejected_instead_of_claiming_pair(tmp_path):
    root = setup_root(tmp_path)
    changed = summary('remote', 1)
    changed['provenance']['runtime']['package_sha'] = 'changed runtime'
    save(root / 'finished' / 'remote' / 'summary.json', changed)
    with pytest.raises(ValueError, match='pairing mismatch.*runtime'):
        pressure_plot.collect(root)


def test_both_arms_changed_from_frozen_protocol_are_rejected(tmp_path):
    root = setup_root(tmp_path)
    protocol = json.loads((root / 'protocol.json').read_text())
    protocol['frozen']['actor_sha'] = 'frozen original'
    save(root / 'protocol.json', protocol)
    with pytest.raises(ValueError, match='frozen protocol mismatch actor_sha'):
        pressure_plot.collect(root)
