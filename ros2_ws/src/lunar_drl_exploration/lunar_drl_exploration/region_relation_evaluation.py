"""Single-model manifest evaluation, strict three-task gate and validation selection."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
from .evaluation_error_policy import valid_coverage

TASKS = ('moon_inside', 'moon_outside', 'cave_inside')
COUNTS = {'validation': 60, 'test': 100, 'quick': 20}


def manifest_digest(manifest):
    return hashlib.sha256(json.dumps(manifest, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':')).encode()).hexdigest()


def validate_manifest(manifest):
    split = manifest['split']
    if split not in COUNTS:
        raise ValueError('unknown evaluation split')
    cases = manifest['cases']
    ids = [c['case_id'] for c in cases]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate manifest case ids')
    counts = Counter(c['capability'] for c in cases)
    tasks = tuple(manifest.get('active_tasks', TASKS))
    if tasks not in (TASKS, ('moon_outside', 'cave_inside')):
        raise ValueError('unsupported evaluation scene tasks')
    if counts != Counter({task: COUNTS[split] for task in tasks}):
        raise ValueError('manifest task counts differ from protocol')
    for case in cases:
        if case['kind'] != 'simple':
            raise ValueError('single-model evaluation requires simple case specs')
        if case['capability'] == 'cave_inside' and not _mother(case):
            raise ValueError('cave mother graph identity missing')
    return cases


def _mother(case):
    return case.get('mother_graph_id') or case.get('spec', {}).get('mother_graph_id')


def _success(row):
    coverage = row.get('final_coverage')
    return (not row.get('error') and row.get('completed') is True and isinstance(coverage, (int, float))
            and math.isfinite(coverage) and 0.99 <= coverage <= 1.0
            and row.get('collisions') == 0)


def evaluate_manifest(root, actor, manifest, output_dir, *, slot=0, evaluate_case_fn=None):
    """Explicit closed-loop runner; never runs baseline or selects on test data."""
    from .course_two_control import write
    if evaluate_case_fn is None:
        from .course_two_eval import evaluate_case
        evaluate_case_fn = evaluate_case
    cases = validate_manifest(manifest)
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    results = []
    for index, case in enumerate(cases):
        request = directory / f'{index:04d}-request.json'
        output = directory / f'{index:04d}-result.json'
        write(request, {'actor': str(actor), 'case': case})
        result = evaluate_case_fn(Path(root), request, output, slot=slot)
        row = result['row']
        row.update(capability=case['capability'], dataset_split=manifest['split'])
        if _mother(case):
            row['mother_graph_id'] = _mother(case)
        write(output, result)
        results.append(result)
    summary = summarize(manifest, results)
    write(directory / 'summary.json', summary)
    return results, summary


def summarize(manifest, results):
    cases = validate_manifest(manifest)
    tasks = tuple(manifest.get('active_tasks', TASKS))
    expected = {case['case_id']: case for case in cases}
    rows = [result.get('row', result) for result in results]
    ids = [row['case_id'] for row in rows]
    if len(ids) != len(set(ids)) or set(ids) != set(expected):
        raise ValueError('evaluation case ids differ from manifest')
    for row in rows:
        case = expected[row['case_id']]
        if row.get('capability') != case['capability']:
            raise ValueError('evaluation task identity differs from manifest')
        if case['capability'] == 'cave_inside' and row.get('mother_graph_id') != _mother(case):
            raise ValueError('evaluation cave mother graph identity differs from manifest')
        counted=row.get('evaluation_failure_kind')=='infrastructure' and isinstance(row.get('error'),str) and bool(row['error']) and row.get('completed') is False
        if (row.get('error') and not counted) or row.get('initial_completed') is not False:
            raise ValueError('invalid evaluation episode: ' + row['case_id'])
        if row.get('dataset_split', manifest['split']) != manifest['split']:
            raise ValueError('evaluation split identity changed')
        if counted and row.get('final_coverage') is None and row.get('collisions') is None:continue
        if counted and row.get('final_coverage') is None and type(row.get('collisions')) is int and row['collisions']>=0:continue
        if not isinstance(row.get('final_coverage'), (int, float)) or not math.isfinite(row['final_coverage']):
            raise ValueError('missing/nonfinite final coverage')
        if not 0 <= row['final_coverage'] <= 1 or (not (counted and row.get('collisions') is None) and (not isinstance(row.get('collisions'), int) or row['collisions'] < 0)):
            raise ValueError('invalid coverage/collision metric')
    by_task = {}
    for task in tasks:
        group = [row for row in rows if row['capability'] == task]
        successes = sum(_success(row) for row in group)
        coverage=[row['final_coverage'] for row in group if valid_coverage(row.get('final_coverage'))]
        by_task[task] = {'episodes': len(group), 'successes': successes,
            'success_rate': successes / len(group),
            'mean_final_coverage': sum(coverage)/len(coverage) if coverage else None,
            'coverage_known_episodes':len(coverage),
            'infrastructure_failures':sum(bool(row.get('error')) for row in group),
            'unknown_collision_episodes':sum(row.get('collisions') is None for row in group),
            'collisions': sum(row.get('collisions') or 0 for row in group)}
    clusters = {}
    for row in rows:
        if row['capability'] == 'cave_inside':
            group = clusters.setdefault(row['mother_graph_id'], {'episodes':0, 'successes':0})
            group['episodes'] += 1
            group['successes'] += int(_success(row))
    passed = manifest['split'] == 'validation' and all(by_task[t]['successes'] >= 54 for t in tasks)
    return {'split': manifest['split'], 'manifest_sha256': manifest_digest(manifest),
        'episodes':len(rows), 'by_task':by_task, 'cave_mother_graph_clusters':clusters,
        'cave_mother_graph_cluster_count':len(clusters), 'candidate_gate_passed':passed,
        'macro_success_rate':sum(by_task[t]['success_rate'] for t in tasks)/len(tasks)}


def select_validation_candidate(manifest, candidates, *, run_id):
    """Rank same-run full validation: gate, macro success, common-case cost.

    Candidates carry checkpoint, run_id and results. Test/quick manifests are
    refused. Efficiency uses only cases successful for every candidate; an
    empty common set cannot establish an efficiency advantage.
    """
    if manifest['split'] != 'validation':
        raise ValueError('checkpoint selection requires full validation only')
    if not candidates:
        raise ValueError('no validation candidates')
    checked = []
    common = None
    for candidate in candidates:
        if candidate['run_id'] != run_id:
            raise ValueError('candidate from another training run')
        summary = summarize(manifest, candidate['results'])
        rows = {result.get('row',result)['case_id']:result.get('row',result)
                for result in candidate['results']}
        successes = {case_id for case_id,row in rows.items() if _success(row)}
        common = successes if common is None else common & successes
        checked.append((candidate, summary, rows))
    ranked = []
    for candidate, summary, rows in checked:
        distance = decisions = 0.0
        for case_id in sorted(common):
            row = rows[case_id]
            for metric in ('distance_m', 'decisions'):
                if not isinstance(row.get(metric),(int,float)) or not math.isfinite(row[metric]) or row[metric] < 0:
                    raise ValueError('invalid common-case efficiency metric')
            distance += row['distance_m']
            decisions += row['decisions']
        divisor = max(1,len(common))
        rank = (int(summary['candidate_gate_passed']), summary['macro_success_rate'],
                -distance/divisor, -decisions/divisor)
        ranked.append((rank,candidate,summary))
    # Stable input order resolves a true tie; no test metric is consulted.
    _, chosen, summary = max(ranked,key=lambda item:item[0])
    return {'checkpoint':chosen['checkpoint'], 'run_id':run_id,
        'validation_summary':summary, 'common_successful_case_count':len(common),
        'selection_rule':'gate_then_macro_success_then_common_case_distance_then_decisions'}
