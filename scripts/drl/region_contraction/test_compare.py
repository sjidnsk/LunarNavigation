"""Fair aggregate denominators and same-state graph diagnostic regressions."""
import csv
import json
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.drl.region_contraction import compare


def summary(mode, *, completed=True, status='finished', **changes):
    result = dict(status=status, mode=mode, family='moon', extent_m=50., seed=42,
        budget=64, successes=int(completed), ended=int(status == 'finished'),
        success_rate=float(completed), infrastructure_failures=0, error=None,
        error_category=None, completed=completed, final_coverage=.99 if completed else .7,
        decisions=10, distance_m=20., zero_gain_ratio=.2, max_zero_gain_run=2,
        navigation_failures=0, collisions=0,
        timing={'actor_seconds': .25, 'contraction_seconds': .1},
        graph={'mean_fine_nodes': 50, 'mean_policy_nodes': 20},
        provenance=dict(actor_sha='actor-A', pairing_config_digest='config-A',
            terrain_id='terrain-A', reference_id='reference-A', semantic_id='semantic-A',
            polygon_digest='polygon-A', initial_pose=[0., 0., 0.],
            initial_observation_hash='observation-A', pairing_digest='pair-A',
            config_sha='config-file-A', source_commit='source-commit-A', source_root='/original/source',
            wrapper_sha='wrapper-A', experiment_sources={'contraction.py': 'contraction-A'},
            runtime={'navigation': {'planner': {'sha': 'native-planner-A'}}, 'package_sha': 'package-A'}))
    result.update(changes)
    return result


def save(root, case, mode, row):
    path = root / case / mode / 'summary.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(row), encoding='utf-8')


def test_running_and_infrastructure_errors_never_become_policy_failures(tmp_path):
    save(tmp_path, 'moon50', 'baseline', summary('baseline'))
    save(tmp_path, 'moon50', 'contracted', summary('contracted', completed=False))
    save(tmp_path, 'moon50', 'remote', summary('remote', status='running'))
    for mode in compare.MODES:
        save(tmp_path, 'cave50', mode, summary(mode, status='error', completed=False,
            ended=0, successes=0, infrastructure_failures=1, error='worker disappeared',
            error_category='infrastructure'))
    result = compare.aggregate(tmp_path)
    assert result['groups']['baseline']['ended'] == 1
    assert result['groups']['baseline']['successes'] == 1
    assert result['groups']['contracted']['success_rate'] == 0.
    assert result['groups']['remote']['ended'] == 0
    assert result['groups']['remote']['success_rate'] is None
    assert len(result['infrastructure_errors']) == 3
    assert result['paired_cases'] == []
    assert result['paired_groups']['baseline']['ended'] == 0


@pytest.mark.parametrize('key', ['actor_sha', 'pairing_config_digest', 'terrain_id',
    'reference_id', 'semantic_id', 'polygon_digest', 'initial_pose', 'pairing_digest'])
def test_pairing_rejects_changed_input_even_if_digest_was_not_updated(tmp_path, key):
    for mode in compare.MODES:
        save(tmp_path, 'moon50', mode, summary(mode))
    row = summary('contracted')
    row['provenance'][key] = [1., 0., 0.] if key == 'initial_pose' else 'changed'
    save(tmp_path, 'moon50', 'contracted', row)
    with pytest.raises(ValueError, match=key):
        compare.aggregate(tmp_path, cases=['moon50'])


@pytest.mark.parametrize('component', ['navigation', 'experiment'])
def test_changed_code_cannot_be_treated_as_a_paired_policy_comparison(tmp_path, component):
    for mode in compare.MODES:
        save(tmp_path, 'moon50', mode, summary(mode))
    row = summary('remote')
    if component == 'navigation':
        row['provenance']['runtime']['navigation']['planner']['sha'] = 'different-planner'
    else:
        row['provenance']['experiment_sources']['contraction.py'] = 'different-algorithm'
    save(tmp_path, 'moon50', 'remote', row)
    with pytest.raises(ValueError, match='runtime|experiment_sources'):
        compare.aggregate(tmp_path, cases=['moon50'])


def test_missing_pairing_evidence_is_rejected(tmp_path):
    for mode in compare.MODES:
        row = summary(mode)
        del row['provenance']['reference_id']
        save(tmp_path, 'moon50', mode, row)
    with pytest.raises(ValueError, match='reference_id'):
        compare.aggregate(tmp_path, cases=['moon50'])


def test_initial_scan_variation_is_reported_without_hiding_pairing_evidence(tmp_path):
    for mode in compare.MODES:
        row = summary(mode)
        if mode == 'contracted':
            row['provenance']['initial_observation_hash'] = 'native-scan-jitter'
        save(tmp_path, 'moon50', mode, row)
    result = compare.aggregate(tmp_path, cases=['moon50'])
    assert result['paired_cases'] == ['moon50']
    assert result['pairing_diagnostics'][0]['initial_observations_equal'] is False
    assert result['pairing_diagnostics'][0]['initial_observation_hashes']['contracted'] == 'native-scan-jitter'


def test_collision_overrides_a_wrong_success_flag_and_outputs_a_readable_dataset(tmp_path):
    for mode in compare.MODES:
        save(tmp_path, 'moon50', mode, summary(mode))
    row = summary('remote', collisions=1)
    save(tmp_path, 'moon50', 'remote', row)
    result = compare.run_comparison(tmp_path, cases=['moon50'])
    assert result['paired_cases'] == ['moon50']
    assert result['paired_groups']['remote']['successes'] == 0
    assert result['paired_groups']['baseline']['success_rate'] == 1.
    rows = list(csv.DictReader((tmp_path / 'comparison.csv').open(encoding='utf-8')))
    assert len(rows) == 3
    assert rows[0]['case'] == 'moon50'
    assert float(rows[0]['timing.actor_seconds']) == .25
    assert json.loads((tmp_path / 'comparison.json').read_text())['groups']['remote']['successes'] == 0


def test_non_infrastructure_exception_is_visible_without_entering_finished_counts(tmp_path):
    save(tmp_path, 'moon50', 'baseline', summary('baseline', status='error',
        error='invalid contraction', error_category='implementation', ended=0))
    result = compare.aggregate(tmp_path, cases=['moon50'])
    assert result['groups']['baseline']['ended'] == 0
    assert result['infrastructure_errors'] == []
    assert len(result['other_errors']) == 1


def test_hop_statistics_count_disconnected_frontiers_and_six_layer_limit():
    features = np.zeros((10, 27), np.float32)
    features[[6, 7, 9], 3] = 1.
    observation = SimpleNamespace(features=features, current_index=0,
        edges=np.asarray([[i, i + 1] for i in range(7)] + [[8, 9]], np.int64))
    stats = compare.frontier_hop_statistics(observation)
    assert stats['positive_task_frontiers'] == 3
    assert stats['reachable_positive_task_frontiers'] == 2
    assert stats['unreachable_positive_task_frontiers'] == 1
    assert stats['within_6_layers'] == 1
    assert stats['beyond_6_layers'] == 1
    assert stats['hop_max'] == 7
    assert stats['hop_median'] == 6.5


def test_same_physical_goal_can_keep_identity_when_action_index_changes():
    first = SimpleNamespace(goals=np.asarray([[2., 3., 0.], [5., 6., 1.]]))
    second = SimpleNamespace(goals=np.asarray([[5., 6., 1.], [2., 3., 0.]]))
    assert compare.goal_changed(first, 0, second, 1) is False
    assert compare.goal_changed(first, 0, second, 0) is True


def test_benchmark_uses_real_frozen_actor_and_all_modes_on_one_observation(tmp_path):
    torch = pytest.importorskip('torch')
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.contracts import DecisionObservation
    from lunar_drl_exploration.feature_schema import OBSERVATION_SCHEMA, frontier_semantics
    from lunar_drl_exploration.model import Actor
    from lunar_drl_exploration.replay import dumps_transport
    model = Actor(ModelConfig(width=8, heads=2, layers=1)).eval()
    actor_path = tmp_path / 'actor.pt'
    torch.save(dict(schema=OBSERVATION_SCHEMA, frontier_semantics=frontier_semantics(),
        model_config=dict(width=8, heads=2, layers=1, actor_score_bound=0.),
        version=1, state_dict=model.state_dict()), actor_path)
    features = np.zeros((8, 27), np.float32)
    features[:, 2] = 1.
    features[7, 3] = 1.
    positions = np.column_stack((np.arange(8), np.zeros(8))).astype(np.float32)
    obs = DecisionObservation(node_ids=np.arange(8), positions=positions, features=features,
        edges=np.asarray([[i, i + 1] for i in range(7)]), edge_lengths=np.ones(7),
        current_index=0, polygon=np.asarray([[-1., -1.], [9., -1.], [9., 1.], [-1., 1.]]),
        context=np.zeros(8), action_nodes=np.asarray([0, 1]), action_yaws=np.zeros(2),
        goals=np.asarray([[0., 0., 0.], [1., 0., 0.]]), epoch='test', revision=0)
    probe_path = tmp_path / 'probe.bin'
    probe_path.write_bytes(dumps_transport(dict(observation=obs,
        metadata={'case': 'test', 'initial_pose': np.asarray([0., 0., 0.])})))
    output = tmp_path / 'benchmark.json'
    result = compare.benchmark_snapshot(probe_path, actor_path, output, repeats=3)
    assert result['device'] == 'cpu'
    assert result['cpu_threads'] == 2
    assert result['fine_graph_construction'] == 'NOT_MEASURED'
    assert result['metadata']['initial_pose'] == [0., 0., 0.]
    assert 'compare.py' in result['provenance']['experiment_sources']
    assert len(result['provenance']['experiment_sources']['contraction.py']['sha256']) == 64
    assert result['provenance']['original_package']['modules']['lunar_drl_exploration.model']['path'].endswith('model.py')
    assert result['provenance']['host']['machine']
    assert len(result['measurement_order']) == 9
    assert {row['mode'] for row in result['modes']} == set(compare.MODES)
    modes = {row['mode']: row for row in result['modes']}
    assert modes['baseline']['fine_nodes'] == modes['remote']['fine_nodes'] == 8
    assert modes['contracted']['policy_nodes'] == 4
    assert modes['remote']['action_count'] == 10
    assert modes['baseline']['frontier_hops_before']['beyond_6_layers'] == 1
    assert modes['contracted']['frontier_hops_after']['within_6_layers'] == 1
    assert all(row['probs_finite'] for row in result['modes'])
    assert all(len(row['samples']) == 3 for row in result['modes'])
    assert 'microbenchmark' in result['limits']
    assert json.loads(output.read_text())['snapshot_sha256'] == result['snapshot_sha256']
