#!/usr/bin/env python3
"""Validate all differential cases before publishing performance summaries."""
import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

csv.field_size_limit(10_000_000)


def read(path, count):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != count:
        raise ValueError(f"{path}: expected {count} rows, got {len(rows)}")
    return rows


def equivalent(a, b, keys, fields):
    def keyed(rows):
        output = {}
        for row in rows:
            key = tuple(row[k] for k in keys)
            if key in output:
                raise ValueError(f"Duplicate key: {key}")
            output[key] = row
        return output
    left, right = keyed(a), keyed(b)
    if left.keys() != right.keys():
        raise ValueError("Differential input keys differ")
    for key in left:
        different = [f for f in fields if left[key][f] != right[key][f]]
        if different:
            raise ValueError(f"Output mismatch {key}: {different}")
    return len(left)


def summarize(rows, keys):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[k] for k in keys)].append(row)
    results = []
    for key, group in sorted(groups.items()):
        values = sorted(float(r['plan_ms']) for r in group)
        entry = dict(zip(keys, key))
        entry.update(n=len(group), median_ms=statistics.median(values),
                     p95_ms=values[math.ceil(.95 * len(values)) - 1])
        if 'postprocess_ms' in group[0]:
            entry['postprocess_median_ms'] = statistics.median(
                float(r['postprocess_ms']) for r in group)
        results.append(entry)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    out = args.directory
    result = {'equivalent_pairs': {}, 'performance': {}, 'status_counts': {}}
    for name, count, keys, fields in [
        ('regression', 4800, ['seed', 'scene', 'epoch', 'resolution', 'platform', 'query'],
         ['status', 'expanded', 'reaches_final_goal', 'reason', 'path_signature']),
        ('steps', 300, ['scene', 'resolution', 'repeat'],
         ['status', 'expanded', 'length_m', 'free', 'blocked', 'unknown', 'represented_gap_m', 'path_signature'])]:
        a = read(out / f'{name}-baseline.csv', count)
        b = read(out / f'{name}-optimized.csv', count)
        result['equivalent_pairs'][name] = equivalent(a, b, keys, fields)
        result['status_counts'][name] = dict(Counter(r['status'] for r in b))
        result['performance'][name] = {
            v: summarize(rows, ['resolution', 'status'] + (['platform', 'query'] if name == 'regression' else []))
            for v, rows in [('baseline', a), ('optimized', b)]}
    variants = defaultdict(list)
    reference = None
    for i, variant in enumerate(['baseline', 'optimized', 'optimized', 'baseline']):
        rows = read(out / f'maze-{i}-{variant}.csv', 300)
        if reference is None:
            reference = rows
        else:
            equivalent(reference, rows, ['scene', 'resolution', 'mode', 'platform', 'repeat'],
                       ['status', 'expanded', 'length_m', 'points', 'reason', 'path_signature'])
        variants[variant].extend(rows)
    result['equivalent_pairs']['maze'] = 600
    result['status_counts']['maze'] = dict(Counter(r['status'] for r in variants['optimized']))
    result['performance']['maze'] = {
        v: summarize(rows, ['resolution', 'mode', 'platform', 'status'])
        for v, rows in variants.items()}
    result['peak_rss_kib'] = {}
    for path in sorted(out.glob('*.resource.txt')):
        for line in path.read_text().splitlines():
            if 'Maximum resident set size (kbytes)' in line:
                result['peak_rss_kib'][path.name] = int(line.rsplit(':', 1)[1])
    (out / 'validation-summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print('Equivalent baseline/candidate pairs:', result['equivalent_pairs'])
    print('Optimized result counts:', result['status_counts'])
    print('Maze native reachable medians / P95:')
    for variant, rows in result['performance']['maze'].items():
        for r in rows:
            if r['mode'] == 'native' and r['status'] == '0':
                print(variant, r)
    print('Peak RSS KiB:', result['peak_rss_kib'])


if __name__ == '__main__':
    main()
