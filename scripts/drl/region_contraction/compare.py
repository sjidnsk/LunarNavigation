"""Paired frozen episodes and same-state CPU snapshot microbenchmarks.

Run ``python -m scripts.drl.region_contraction.compare --root ROOT`` or
``python -m scripts.drl.region_contraction.compare --probe SNAPSHOT --actor ACTOR
--output REPORT`` in the original runtime's owned Python environment. This module
does not choose an overlay, build native code, or change the production runtime.
"""
import argparse
from collections.abc import Mapping
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time

import numpy as np

MODES = ('baseline', 'contracted', 'remote')
PAIR_FIELDS = ('actor_sha', 'pairing_config_digest', 'terrain_id', 'reference_id',
    'semantic_id', 'polygon_digest', 'initial_pose', 'pairing_digest', 'config_sha',
    'source_commit', 'source_root', 'wrapper_sha', 'runtime', 'experiment_sources')
CASE_FIELDS = ('family', 'extent_m', 'seed', 'budget')
METRICS = ('final_coverage', 'decisions', 'distance_m', 'zero_gain_ratio',
    'max_zero_gain_run', 'navigation_failures', 'collisions')


def _classification(row):
    if row.get('infrastructure_failures', 0) or row.get('error_category') == 'infrastructure':
        return 'infrastructure_error'
    if row.get('error') or row.get('status') == 'error':
        return 'other_error'
    return 'finished' if row.get('status') == 'finished' else 'incomplete'


def _success(row):
    # Never promote coverage alone to success; it must be a collision-free
    # terminal episode as recorded by the unchanged environment.
    return bool(row.get('completed')) and not row.get('collisions', 0)


def _groups(records):
    result = {}
    for mode in MODES:
        rows = [record['summary'] for record in records
                if record['mode'] == mode and record['classification'] == 'finished']
        count = len(rows)
        group = dict(ended=count, successes=sum(_success(row) for row in rows),
            success_rate=sum(_success(row) for row in rows) / count if count else None)
        for key in METRICS:
            values = [float(row[key]) for row in rows if row.get(key) is not None]
            if any(not math.isfinite(value) for value in values):
                raise ValueError(f'nonfinite finished metric: {mode}.{key}')
            group[f'mean_{key}'] = statistics.fmean(values) if values else None
        for category in ('timing', 'graph'):
            keys = sorted({key for row in rows for key in row.get(category, {})})
            group[category] = {}
            for key in keys:
                values = [float(row[category][key]) for row in rows
                          if isinstance(row.get(category, {}).get(key), (int, float))]
                if any(not math.isfinite(value) for value in values):
                    raise ValueError(f'nonfinite finished metric: {mode}.{category}.{key}')
                if values:
                    group[category][f'mean_{key}'] = statistics.fmean(values)
        result[mode] = group
    return result


def _require_pair_identity(case, records):
    identities = []
    for record in records:
        row, identity = record['summary'], {}
        for key in CASE_FIELDS:
            if row.get(key) is None:
                raise ValueError(f'{case}/{record["mode"]}: missing pairing field {key}')
            identity[key] = row[key]
        provenance = row.get('provenance', {})
        for key in PAIR_FIELDS:
            if provenance.get(key) is None or provenance.get(key) == '':
                raise ValueError(f'{case}/{record["mode"]}: missing pairing field {key}')
            identity[key] = provenance[key]
        identities.append(identity)
    first = identities[0]
    for index, identity in enumerate(identities[1:], start=1):
        differences = [key for key in first if first[key] != identity[key]]
        if differences:
            raise ValueError(f'{case}: pairing mismatch {records[0]["mode"]} vs '
                             f'{records[index]["mode"]}: {", ".join(differences)}')


def aggregate(root, *, cases=None):
    """Keep finished denominators, errors, and complete paired triples separate."""
    root = Path(root)
    cases = list(cases) if cases is not None else ['moon50', 'cave50']
    if len(cases) != len(set(cases)):
        raise ValueError('duplicate comparison case')
    records, missing, paired_cases, paired_records, pairing_diagnostics = [], [], [], [], []
    for case in cases:
        current = []
        for mode in MODES:
            path = root / case / mode / 'summary.json'
            if not path.is_file():
                missing.append(dict(case=case, mode=mode, path=str(path)))
                continue
            row = json.loads(path.read_text(encoding='utf-8'))
            if row.get('mode') != mode:
                raise ValueError(f'{path}: mode does not match artifact directory')
            record = dict(case=case, mode=mode, path=str(path),
                classification=_classification(row), summary=row)
            records.append(record)
            current.append(record)
        finished = [record for record in current if record['classification'] == 'finished']
        # Validate every available finished arm, even before the final arm ends.
        if finished:
            _require_pair_identity(case, finished)
            initial_hashes = {record['mode']: record['summary']['provenance'].get('initial_observation_hash')
                              for record in finished}
            pairing_diagnostics.append(dict(case=case, initial_observation_hashes=initial_hashes,
                initial_observations_equal=all(initial_hashes.values()) and len(set(initial_hashes.values())) == 1,
                note='Initial native scan observations may vary with arrival timing. Hard pairing '
                     'requires unchanged terrain/reference, original spawn, configuration and Actor.'))
        if len(finished) == len(MODES):
            paired_cases.append(case)
            paired_records.extend(finished)
    errors = lambda kind: [record for record in records if record['classification'] == kind]
    return dict(schema='region_contraction_comparison_v1', root=str(root.resolve()),
        requested_cases=cases, paired_cases=paired_cases,
        complete_pairing=len(paired_cases) == len(cases),
        groups=_groups(records), paired_groups=_groups(paired_records), records=records,
        pairing_diagnostics=pairing_diagnostics,
        missing=missing, incomplete=errors('incomplete'),
        infrastructure_errors=errors('infrastructure_error'), other_errors=errors('other_error'),
        limits='Only finished collision-free terminal episodes can succeed. Infrastructure '
               'errors do not enter ended denominators. Compare policy outcomes using paired_groups; '
               'remote also changes the action set. These local frozen episodes do not establish '
               'retrained policy quality or vehicle readiness.')


def _flatten(value, prefix='', result=None):
    result = {} if result is None else result
    for key, item in value.items():
        name = f'{prefix}.{key}' if prefix else key
        if isinstance(item, dict):
            _flatten(item, name, result)
        elif isinstance(item, (list, tuple)):
            result[name] = json.dumps(item, ensure_ascii=False, allow_nan=False)
        else:
            result[name] = item
    return result


def _write_json(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_plain(record), ensure_ascii=False, allow_nan=False, indent=2) + '\n',
                    encoding='utf-8')


def _plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def run_comparison(root, *, cases=None):
    result = aggregate(root, cases=cases)
    dataset = []
    for record in result['records']:
        row = _flatten(record['summary'])
        row.update(case=record['case'], mode=record['mode'], classification=record['classification'],
                   paired=record['case'] in result['paired_cases'],
                   eligible_success=_success(record['summary']) and record['classification'] == 'finished')
        dataset.append(row)
    _write_json(Path(root) / 'comparison.json', result)
    _write_json(Path(root) / 'comparison.dataset.json', dataset)
    keys = sorted({key for row in dataset for key in row})
    columns = ['case', 'mode', 'classification', 'paired', 'eligible_success']
    columns.extend(key for key in keys if key not in columns)
    with (Path(root) / 'comparison.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(dataset)
    return result


def frontier_hop_statistics(observation, layers=6):
    """Undirected original traversability edges; no spatial shortcuts or truth."""
    if layers < 1:
        raise ValueError('positive message-passing layer count required')
    from lunar_drl_exploration.feature_schema import IN_GAIN_SLICE, OUT_GAIN_SLICE
    count = len(observation.features)
    neighbors = [[] for _ in range(count)]
    for source, target in observation.edges:
        neighbors[int(source)].append(int(target))
        neighbors[int(target)].append(int(source))
    hops = np.full(count, -1, np.int64)
    hops[observation.current_index] = 0
    pending = deque([observation.current_index])
    while pending:
        source = pending.popleft()
        for target in neighbors[source]:
            if hops[target] == -1:
                hops[target] = hops[source] + 1
                pending.append(target)
    positive = np.any(observation.features[:, IN_GAIN_SLICE] > 0., axis=1)
    reachable = hops[positive & (hops >= 0)]
    return dict(message_passing_layers=layers, graph_nodes=count,
        current_reachable_nodes=int(np.count_nonzero(hops >= 0)),
        positive_task_frontiers=int(positive.sum()),
        positive_outside_frontiers=int(np.any(observation.features[:, OUT_GAIN_SLICE] > 0., axis=1).sum()),
        reachable_positive_task_frontiers=int(reachable.size),
        unreachable_positive_task_frontiers=int(np.count_nonzero(positive & (hops < 0))),
        within_6_layers=int(np.count_nonzero(reachable <= 6)),
        beyond_6_layers=int(np.count_nonzero(reachable > 6)),
        within_encoder_layers=int(np.count_nonzero(reachable <= layers)),
        beyond_encoder_layers=int(np.count_nonzero(reachable > layers)),
        hop_min=int(reachable.min()) if reachable.size else None,
        hop_median=float(np.median(reachable)) if reachable.size else None,
        hop_p90=float(np.percentile(reachable, 90)) if reachable.size else None,
        hop_max=int(reachable.max()) if reachable.size else None,
        reachable_hops=sorted(map(int, reachable)))


def goal_changed(first, first_action, second, second_action):
    return not np.array_equal(first.goals[first_action], second.goals[second_action])


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _load_snapshot(path):
    path = Path(path)
    if path.suffix == '.npz':
        from .evaluate import load_observation
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(str(archive['_metadata_json'].item())) if '_metadata_json' in archive else {}
        return load_observation(path), metadata
    from lunar_drl_exploration.replay import loads_transport
    from lunar_drl_exploration.contracts import DecisionObservation
    record = loads_transport(path.read_bytes())
    if isinstance(record, DecisionObservation):
        return record, {}
    if not isinstance(record, dict) or not isinstance(record.get('observation'), DecisionObservation):
        raise ValueError('probe must contain a complete native DecisionObservation')
    return record['observation'], _plain(record.get('metadata', {}))


def _adapt(observation, mode):
    from .contraction import contract_observation, expand_frontier_actions
    start = time.perf_counter()
    expanded = observation
    expansion_seconds = 0.
    if mode == 'remote':
        expanded = expand_frontier_actions(observation, max_positions=32)
        expansion_seconds = time.perf_counter() - start
    contraction_seconds = 0.
    result = None
    if mode != 'baseline':
        contraction_start = time.perf_counter()
        result = contract_observation(expanded)
        policy_observation = result.observation
        contraction_seconds = time.perf_counter() - contraction_start
    else:
        policy_observation = observation
    # These action invariants make same-state probabilities interpretable.
    prefix = len(observation.goals)
    if not np.array_equal(policy_observation.goals[:prefix], observation.goals):
        raise ValueError(f'{mode}: original physical action prefix changed')
    if not np.array_equal(policy_observation.action_yaws[:prefix], observation.action_yaws):
        raise ValueError(f'{mode}: original action heading prefix changed')
    if mode == 'contracted' and len(policy_observation.goals) != prefix:
        raise ValueError('contracted: original action count changed')
    return policy_observation, result, dict(expansion_seconds=expansion_seconds,
        contraction_seconds=contraction_seconds,
        adaptation_seconds=time.perf_counter() - start)


def _timing_summary(values):
    return dict(median=float(np.median(values)), p90=float(np.percentile(values, 90)),
                min=float(min(values)), max=float(max(values)))


def _load_actor(path):
    """Use the same Actor-only contract without importing native ROS runtime."""
    import torch
    from lunar_drl_exploration.config import ModelConfig, canonical_model_config
    from lunar_drl_exploration.feature_schema import OBSERVATION_SCHEMA, require_frontier_semantics
    from lunar_drl_exploration.model import Actor
    record = torch.load(path, map_location='cpu', weights_only=True)
    required = {'schema', 'frontier_semantics', 'model_config', 'version', 'state_dict'}
    if set(record) != required or record['schema'] != OBSERVATION_SCHEMA:
        raise ValueError('expected compatible frozen Actor-only publication')
    require_frontier_semantics(record)
    if type(record['version']) is not int or record['version'] < 0:
        raise ValueError('Actor version must be a nonnegative integer')
    actor = Actor(ModelConfig(**canonical_model_config(record['model_config'])))
    actor.load_state_dict(record['state_dict'], strict=True)
    if any(not torch.isfinite(value).all() for value in actor.state_dict().values()):
        raise ValueError('Actor weights must be finite')
    return actor.cpu().eval()


def _benchmark_provenance():
    import importlib
    import torch
    package = importlib.import_module('lunar_drl_exploration')
    modules = {}
    for name in ('lunar_drl_exploration', 'lunar_drl_exploration.model',
                 'lunar_drl_exploration.batch', 'lunar_drl_exploration.contracts',
                 'lunar_drl_exploration.config', 'lunar_drl_exploration.feature_schema',
                 'lunar_drl_exploration.replay'):
        path = Path(importlib.import_module(name).__file__).resolve()
        modules[name] = dict(path=str(path), sha256=_sha256(path))
    package_root = Path(package.__file__).resolve().parents[4]
    commit = subprocess.run(['git', '-C', str(package_root), 'rev-parse', 'HEAD'],
        capture_output=True, text=True, check=False)
    experiment_sources = {}
    for name in ('compare.py', 'contraction.py', 'evaluate.py'):
        path = Path(__file__).with_name(name).resolve()
        if path.is_file():
            experiment_sources[name] = dict(path=str(path), sha256=_sha256(path))
    cpu_model = platform.processor()
    cpu_info = Path('/proc/cpuinfo')
    if cpu_info.is_file():
        cpu_model = next((line.split(':', 1)[1].strip() for line in cpu_info.read_text().splitlines()
                          if line.startswith('model name')), cpu_model)
    return dict(experiment_sources=experiment_sources,
        original_package=dict(path=str(Path(package.__file__).resolve()),
            version=getattr(package, '__version__', None), source_root=str(package_root),
            source_commit=commit.stdout.strip() if commit.returncode == 0 else None, modules=modules),
        python=platform.python_version(), numpy=np.__version__, torch=str(torch.__version__),
        host=dict(platform=platform.platform(), hostname=platform.node(), machine=platform.machine(),
            cpu_model=cpu_model, logical_cpus=os.cpu_count(), loadavg_before=list(os.getloadavg()),
            exclusive_host=False),
        thread_environment={name: os.environ.get(name) for name in
            ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS')})


def benchmark_snapshot(probe, actor_path, output, *, repeats=7):
    """Same CPU threads, weights and native observation, two warmups per arm.

    Actor timing includes observation tensor packing. Combined timing includes
    remote expansion, contraction and Actor; fine-graph generation and native
    navigation are not measured. No privileged input is passed to the Actor.
    """
    if isinstance(repeats, bool) or not isinstance(repeats, int) or repeats < 1:
        raise ValueError('positive integer benchmark repeat count required')
    import torch
    torch.set_num_threads(2)
    actor = _load_actor(actor_path)
    observation, metadata = _load_snapshot(probe)
    provenance = _benchmark_provenance()
    if not len(observation.goals):
        raise ValueError('probe must contain at least one valid real action')
    prepared = {mode: _adapt(observation, mode) for mode in MODES}
    samples = {mode: [] for mode in MODES}
    outputs = {}
    measurement_order = []
    with torch.inference_mode():
        for _ in range(2):
            for mode in MODES:
                adapted, _, _ = _adapt(observation, mode)
                actor([adapted])
        # Rotate order across rounds to reduce one arm's cache/order advantage.
        for round_index in range(repeats):
            modes = MODES[round_index % 3:] + MODES[:round_index % 3]
            for mode in modes:
                start = time.perf_counter()
                adapted, _, timings = _adapt(observation, mode)
                actor_start = time.perf_counter()
                policy = actor([adapted])[0]
                actor_seconds = time.perf_counter() - actor_start
                combined_seconds = time.perf_counter() - start
                probabilities = policy.probs.cpu().numpy().astype(np.float64)
                timings.update(actor_seconds=actor_seconds, combined_seconds=combined_seconds,
                    argmax_action=int(probabilities.argmax()))
                samples[mode].append(timings)
                outputs[mode] = probabilities
                measurement_order.append(dict(round=round_index, mode=mode))
    baseline_action = int(outputs['baseline'].argmax())
    layers = actor.config.layers
    before = frontier_hop_statistics(observation, layers=layers)
    rows = []
    for mode in MODES:
        adapted, contraction, _ = prepared[mode]
        probabilities = outputs[mode]
        action = int(probabilities.argmax())
        finite = bool(np.isfinite(probabilities).all())
        normalized = bool(finite and np.all(probabilities >= 0.)
                          and np.isclose(probabilities.sum(), 1., atol=1e-5))
        if len(probabilities) != len(adapted.goals):
            raise ValueError(f'{mode}: Actor output count differs from real action count')
        timings = {key: _timing_summary([sample[key] for sample in samples[mode]])
                   for key in ('expansion_seconds', 'contraction_seconds', 'adaptation_seconds',
                               'actor_seconds', 'combined_seconds')}
        prefix = len(observation.goals)
        baseline_padded = np.pad(outputs['baseline'], (0, len(probabilities) - prefix))
        row = dict(mode=mode, fine_nodes=len(observation.node_ids), fine_edges=len(observation.edges),
            policy_nodes=len(adapted.node_ids), policy_edges=len(adapted.edges),
            node_ratio=len(adapted.node_ids) / len(observation.node_ids),
            edge_ratio=len(adapted.edges) / len(observation.edges) if len(observation.edges) else 1.,
            virtual_nodes=int(contraction.virtual_mask.sum()) if contraction is not None else 0,
            action_count=len(probabilities), remote_action_count=len(probabilities) - prefix,
            probs_finite=finite, probs_normalized=normalized, probability_sum=float(probabilities.sum()),
            max_probability=float(probabilities.max()), argmax_action=action,
            selected_goal=list(map(float, adapted.goals[action])),
            goal_changed=goal_changed(observation, baseline_action, adapted, action),
            argmax_stable_across_repeats=len({sample['argmax_action'] for sample in samples[mode]}) == 1,
            total_variation_vs_baseline=float(.5 * np.abs(probabilities - baseline_padded).sum()),
            added_action_probability_mass=float(probabilities[prefix:].sum()),
            frontier_hops_before=before,
            frontier_hops_after=frontier_hop_statistics(adapted, layers=layers),
            timing=timings, samples=samples[mode], probabilities=list(map(float, probabilities)))
        if contraction is not None:
            row['contraction_diagnostics'] = contraction.diagnostics
        rows.append(row)
    result = dict(schema='region_contraction_snapshot_microbenchmark_v1',
        snapshot=str(Path(probe).resolve()), snapshot_sha256=_sha256(probe),
        actor=str(Path(actor_path).resolve()), actor_sha256=_sha256(actor_path),
        device='cpu', cpu_threads=torch.get_num_threads(), warmup_rounds=2, repeats=repeats,
        actor_layers=layers, actor_timing_includes_tensor_packing=True,
        fine_graph_construction='NOT_MEASURED', native_navigation='NOT_MEASURED',
        privileged_actor_input=False, metadata=metadata, provenance=provenance,
        measurement_order=measurement_order, modes=rows,
        limits='A fixed-snapshot microbenchmark measures graph adaptation and Actor computation '
            'only. It is not a full-coverage episode, does not measure original fine-graph '
            'construction or gain calculation, and does not establish higher policy quality. '
            'Remote changes the action set. Hop counts describe local message-passing reach; '
            'the Actor also reads global node attention. Interleaved arms share a host which '
            'may have other workloads; timing does not imply exclusive CPU access.')
    provenance['host']['loadavg_after'] = list(os.getloadavg())
    final_provenance = _benchmark_provenance()
    if (final_provenance['experiment_sources'] != provenance['experiment_sources']
            or final_provenance['original_package'] != provenance['original_package']):
        raise ValueError('benchmark source identity changed during timing')
    _write_json(output, result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    task = parser.add_mutually_exclusive_group(required=True)
    task.add_argument('--root', type=Path)
    task.add_argument('--probe', type=Path)
    parser.add_argument('--actor', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--repeats', type=int, default=7)
    parser.add_argument('--cases', nargs='+', default=['moon50', 'cave50'])
    args = parser.parse_args(argv)
    if args.probe:
        if args.actor is None or args.output is None:
            parser.error('--probe requires --actor and --output')
        result = benchmark_snapshot(args.probe, args.actor, args.output, repeats=args.repeats)
        print(json.dumps(dict(output=str(args.output), modes=[dict(mode=row['mode'],
            nodes=row['policy_nodes'], actions=row['action_count'], goal_changed=row['goal_changed'],
            actor_seconds=row['timing']['actor_seconds']) for row in result['modes']]),
            ensure_ascii=False, allow_nan=False))
    else:
        result = run_comparison(args.root, cases=args.cases)
        print(json.dumps(dict(paired_cases=result['paired_cases'], groups=result['groups'],
            infrastructure_errors=len(result['infrastructure_errors']),
            other_errors=len(result['other_errors'])), ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
