"""One-shot, protocol-backed static plots for the frozen pressure experiment.

Run with system Python: ``python3 pressure_plot.py --root EXPERIMENT_ROOT``.
This independent reader imports no native runtime and never starts an episode.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

if __package__:
    from . import compare
else:
    # Direct-file invocation keeps the frozen reader alongside this script.
    import compare

LABELS = {'baseline': 'A0 基线', 'remote': 'A2 粗图+远程点', 'contracted': 'A1 图收缩'}
COLORS = {'baseline': '#2563a6', 'remote': '#dc7d20', 'contracted': '#21845b'}
PHASES = {'partial': '进行中', 'final': '已完成', 'final_with_errors': '已结束（含错误）'}
STATUSES = {'pending': '进行中', 'infrastructure_error': '基础设施错误', 'other_error': '执行错误'}


def _number(value):
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('nonfinite metric')
    return result


def read_curve(path, max_points=512):
    """Read one JSONL record at a time; keep a bounded, decimated scalar trace.

    A writer's incomplete final record is omitted. Malformed complete records
    are errors. First and last valid points survive online thinning.
    """
    if max_points < 3:
        raise ValueError('max_points must be at least 3')
    result = dict(points=[], read_records=0, partial_tail=False)
    path = Path(path)
    if not path.is_file():
        return result
    previous_step = None
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if not line.endswith('\n'):
                result['partial_tail'] = True
                break
            try:
                row = json.loads(line)
                step, coverage = _number(row['step']), _number(row['coverage'])
                if step is None or coverage is None or step < 0 or not 0 <= coverage <= 1.000001:
                    raise ValueError('invalid step/coverage')
                if previous_step is not None and step <= previous_step:
                    raise ValueError('steps must increase')
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f'{path}: malformed complete record {result["read_records"] + 1}: {error}') from error
            previous_step = step
            result['read_records'] += 1
            result['points'].append([int(step) if step.is_integer() else step, coverage])
            if len(result['points']) > max_points:
                last = result['points'][-1]
                result['points'] = result['points'][::2]
                if result['points'][-1] != last:
                    result['points'].append(last)
    return result


def _protocol(root):
    protocol = json.loads((root / 'protocol.json').read_text(encoding='utf-8'))
    cases, modes = protocol['cases'], protocol['modes']
    if not cases or not modes or len(modes) != len(set(modes)):
        raise ValueError('protocol requires cases and unique modes')
    names = [case['name'] for case in cases]
    if len(names) != len(set(names)):
        raise ValueError('duplicate protocol case')
    for case in cases:
        if Path(case['name']).name != case['name'] or case['name'] in ('.', '..'):
            raise ValueError('protocol case must be a directory basename')
        for key in ('family', 'extent', 'seed', 'budget'):
            if key not in case:
                raise ValueError(f'{case["name"]}: missing protocol {key}')
        if _number(case['extent']) <= 0 or _number(case['budget']) <= 0:
            raise ValueError('protocol extent and budget must be positive')
    return protocol


def collect(root):
    """Use only complete, provenance-matched pairs for comparative outcomes."""
    root = Path(root).resolve()
    protocol = _protocol(root)
    cases, modes = protocol['cases'], protocol['modes']
    compared = compare.aggregate(root, cases=[case['name'] for case in cases], modes=modes)
    records = {(row['case'], row['mode']): row for row in compared['records']}
    definitions = {case['name']: case for case in cases}
    for record in compared['records']:
        row, case = record['summary'], definitions[record['case']]
        for actual, planned in (('family', 'family'), ('extent_m', 'extent'), ('seed', 'seed'), ('budget', 'budget')):
            if row.get(actual) != case[planned]:
                raise ValueError(f'{record["case"]}/{record["mode"]}: summary differs from protocol {actual}')
        if record['classification'] == 'finished':
            provenance = row['provenance']
            for key in ('actor_sha', 'config_sha', 'source_commit', 'experiment_commit',
                        'wrapper_sha', 'experiment_sources', 'runtime'):
                if key in protocol.get('frozen', {}) and provenance.get(key) != protocol['frozen'][key]:
                    raise ValueError(f'{record["case"]}/{record["mode"]}: frozen protocol mismatch {key}')
    episodes, curves = [], []
    for case in cases:
        for mode in modes:
            record = records.get((case['name'], mode))
            summary = record['summary'] if record else {}
            status = record['classification'] if record else 'pending'
            if status == 'incomplete':
                status = 'pending'
            finished = status == 'finished'
            terminal = status in ('finished', 'infrastructure_error', 'other_error')
            curve = read_curve(root / case['name'] / mode / 'steps.jsonl')
            curves.append(dict(case=case['name'], mode=mode, **curve))
            episodes.append(dict(case=case['name'], family=case['family'], extent_m=case['extent'],
                seed=case['seed'], budget=case['budget'], mode=mode, status=status, terminal=terminal,
                successful=compare._success(summary) if finished else None,
                final_coverage=_number(summary.get('final_coverage')) if finished else None,
                decisions=_number(summary.get('decisions')) if finished else None,
                distance_m=_number(summary.get('distance_m')) if finished else None,
                wall_seconds=_number(summary.get('timing', {}).get('episode_wall_seconds')) if finished else None,
                mean_fine_nodes=_number(summary.get('graph', {}).get('mean_fine_nodes')) if finished else None,
                mean_policy_nodes=_number(summary.get('graph', {}).get('mean_policy_nodes')) if finished else None,
                error=summary.get('error'), curve_records=curve['read_records'], partial_tail=curve['partial_tail']))
    strata = []
    keys = list(dict.fromkeys((case['family'], case['extent']) for case in cases))
    for family, extent in keys:
        names = [case['name'] for case in cases if (case['family'], case['extent']) == (family, extent)]
        selected = [record for record in compared['records'] if record['case'] in names]
        paired = [name for name in names if name in compared['paired_cases']]
        both = [name for name in paired if name in compared['both_successful_cases']]
        groups = compare._groups([record for record in selected if record['case'] in paired], modes)
        successful_groups = compare._groups([record for record in selected if record['case'] in both], modes)
        available = compare._groups(selected, modes)
        strat_modes = {}
        for mode in modes:
            mode_episodes = [episode for episode in episodes if episode['case'] in names and episode['mode'] == mode]
            strat_modes[mode] = dict(groups[mode],
                available_ended=available[mode]['ended'], available_successes=available[mode]['successes'],
                available_success_rate=available[mode]['success_rate'],
                both_successful_ended=successful_groups[mode]['ended'],
                both_successful_wall_seconds=successful_groups[mode]['mean_wall_seconds'],
                both_successful_distance_m=successful_groups[mode]['mean_distance_m'],
                both_successful_decisions=successful_groups[mode]['mean_decisions'],
                pending=sum(episode['status'] == 'pending' for episode in mode_episodes),
                infrastructure_errors=sum(episode['status'] == 'infrastructure_error' for episode in mode_episodes),
                other_errors=sum(episode['status'] == 'other_error' for episode in mode_episodes))
        strata.append(dict(family=family, extent_m=extent, planned_cases=len(names),
            paired_cases=paired, both_successful_cases=both, modes=strat_modes))
    terminal_count = sum(episode['terminal'] for episode in episodes)
    expected = len(episodes)
    errors = sum(episode['status'] in ('infrastructure_error', 'other_error') for episode in episodes)
    phase = 'partial' if terminal_count < expected else ('final_with_errors' if errors else 'final')
    metrics = dict(schema='region_pressure_plot_v1', root=str(root), phase=phase,
        generated_utc=datetime.now(timezone.utc).isoformat(), expected_episodes=expected,
        terminal_episodes=terminal_count, pending_episodes=expected-terminal_count,
        error_episodes=errors, planned_cases=len(cases), paired_cases=compared['paired_cases'],
        modes=modes, cases=cases, strata=strata, episodes=episodes,
        pairing_diagnostics=compared['pairing_diagnostics'],
        protocol_sha=hashlib.sha256((root/'protocol.json').read_bytes()).hexdigest(),
        plot_source_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        compare_source_sha=hashlib.sha256(Path(compare.__file__).read_bytes()).hexdigest(),
        limits='Successes/ended, coverage and decisions compare complete provenance-matched pairs only. '
               'Errors are separate and excluded from ended. Pending is unknown, never zero. '
               'Coverage/decisions include all finished endpoints, including budget failures. '
               'Wall/path to success uses the jointly successful subset; this selection is descriptive, '
               'not a population efficiency effect. Per-family/extent sample sizes are small. '
               'Curves are observed prefixes, bounded to 512 points with first/last preserved. '
               'Means weight episodes equally; graph means are means of per-episode means.')
    return metrics, curves


def _plots(metrics, curves, root):
    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir()) / 'lunar-pressure-mpl'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib import font_manager

    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    font_manager.fontManager.addfont(str(font))
    plt.rcParams['font.family'] = [font_manager.FontProperties(fname=str(font)).get_name()]
    plt.rcParams['axes.unicode_minus'] = False

    families = list(dict.fromkeys(case['family'] for case in metrics['cases']))
    extents = sorted(set(case['extent'] for case in metrics['cases']))
    modes = metrics['modes']
    strata = {(row['family'], row['extent_m']): row for row in metrics['strata']}
    fig, axes = plt.subplots(4, len(families), figsize=(max(8, 5.9*len(families)), 11.2), squeeze=False)
    rows = [('success_rate', '成功回合 / 结束回合（%）', 100.),
            ('mean_final_coverage', '终态覆盖率（%）', 100.),
            ('mean_decisions', '决策步数\n（全部终态配对）', 1.),
            ('both_successful_wall_seconds', '墙钟时间（s）\n（双方均成功）', 1.)]
    offsets = [(i-(len(modes)-1)/2)*.24 for i in range(len(modes))]
    for col, family in enumerate(families):
        for row_index, (key, label, multiplier) in enumerate(rows):
            ax = axes[row_index, col]
            for i, extent in enumerate(extents):
                stratum = strata.get((family, extent))
                if stratum is None:
                    continue
                selected_count = len(stratum['both_successful_cases'] if row_index == 3 else stratum['paired_cases'])
                if not selected_count:
                    pending = any(value['pending'] for value in stratum['modes'].values())
                    ax.text(i, .045, '进行中' if pending else '无有效配对',
                        ha='center', va='bottom', fontsize=8, color='#666666', transform=ax.get_xaxis_transform())
                ax.text(i, 1.09, f'{"b" if row_index == 3 else "n"}={selected_count}/{stratum["planned_cases"]}',
                    ha='center', fontsize=8, color='#555555', transform=ax.get_xaxis_transform())
                for mode_index, (mode, offset) in enumerate(zip(modes, offsets)):
                    group = stratum['modes'][mode]
                    value = group.get(key)
                    if value is None:
                        continue
                    height = value * multiplier
                    ax.bar(i+offset, height, width=.22, color=COLORS[mode], zorder=3)
                    text = f'{group["successes"]}/{group["ended"]}' if row_index == 0 else f'{height:.1f}'
                    ax.annotate(text, (i+offset, height), xytext=(0, 3+9*(mode_index%2)), textcoords='offset points',
                        ha='center', fontsize=8, color=COLORS[mode])
            ax.set_xticks(range(len(extents)), [str(extent) for extent in extents])
            ax.set_xlim(-.6, len(extents)-.4)
            ax.set_ylabel(label, fontsize=10)
            ax.set_xlabel('场景尺度（m）')
            ax.grid(axis='y', color='#dddddd', linewidth=.6, zorder=0)
            ax.spines[['top','right']].set_visible(False)
            if row_index in (0,1):
                ax.set_ylim(0, 112)
            else:
                # Family columns share the same metric scale and label headroom.
                values = [group[key] for stratum in metrics['strata']
                          for group in stratum['modes'].values() if group.get(key) is not None]
                ax.set_ylim(0, max(values)*1.3 if values and max(values)>0 else 1.)
            if row_index == 0:
                ax.set_title({'moon':'月面', 'cave':'洞穴'}.get(family,family), fontsize=12, pad=37)
    handles = [Line2D([0],[0], color=COLORS[mode], lw=7, label=LABELS[mode]) for mode in modes]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5,.955), ncol=len(modes), frameon=False)
    fig.suptitle(f'压力测试对比 — {PHASES[metrics["phase"]]} | 终态 {metrics["terminal_episodes"]}/{metrics["expected_episodes"]}'
                 f' | 完整配对 {len(metrics["paired_cases"])}/{metrics["planned_cases"]}', fontsize=13, y=.988)
    fig.text(.5,.018,'n：完整配对；b：双方均成功的配对。进行中值留空；错误不计入结束回合。'
             '\n覆盖率/步数包含失败终态；墙钟来自成功子集，仅作小样本描述。',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.067,1,.92), h_pad=1.7)
    fig.savefig(root/'pressure-comparison.png', dpi=140)
    plt.close(fig)

    columns = min(4, len(metrics['cases']))
    rows_count = math.ceil(len(metrics['cases'])/columns)
    fig, axes = plt.subplots(rows_count, columns, figsize=(4.3*columns, 2.8*rows_count), squeeze=False)
    curve_index = {(row['case'], row['mode']): row for row in curves}
    episode_index = {(row['case'], row['mode']): row for row in metrics['episodes']}
    for index, case in enumerate(metrics['cases']):
        ax = axes.flat[index]
        has_data = False
        for mode in modes:
            episode = episode_index[(case['name'], mode)]
            curve = curve_index[(case['name'], mode)]
            status = episode['status']
            label = f'{LABELS[mode]}：{STATUSES.get(status,status)}'
            if status == 'finished':
                label = f'{LABELS[mode]}：{"成功" if episode["successful"] else "失败终态"}'
            points = curve['points']
            if points:
                has_data = True
                ax.plot([point[0] for point in points], [100*point[1] for point in points],
                    color=COLORS[mode], linewidth=1.4, linestyle='--' if mode == 'remote' else '-', label=label)
                ax.scatter(points[-1][0], 100*points[-1][1], s=13,
                    marker='x' if status.endswith('error') else 'o', color=COLORS[mode])
            else:
                ax.plot([], [], color=COLORS[mode], label=label)
            if episode['final_coverage'] is not None and episode['decisions'] is not None:
                has_data = True
                ax.scatter(episode['decisions'],100*episode['final_coverage'],s=17,color=COLORS[mode],marker='s')
        if not has_data:
            pending = all(episode_index[(case['name'], mode)]['status']=='pending' for mode in modes)
            ax.text(.5,.45,('进行中——' if pending else '')+'暂无已观测步数',
                ha='center',transform=ax.transAxes,color='#666666',fontsize=9)
        ax.axhline(99.,color='#888888',linewidth=.6,linestyle=':')
        ax.set_ylim(0,103)
        ax.set_xlim(0,case['budget'])
        ax.set_title(f'{case["name"]} | 种子 {case["seed"]}',fontsize=10)
        ax.set_xlabel(f'决策步数（预算 {case["budget"]}）',fontsize=8)
        ax.set_ylabel('覆盖率（%）',fontsize=8)
        ax.tick_params(labelsize=8)
        ax.grid(color='#e8e8e8',linewidth=.5)
        ax.legend(fontsize=7,loc='lower right',framealpha=.8)
    for ax in list(axes.flat)[len(metrics['cases']):]:
        ax.set_visible(False)
    fig.suptitle(f'已观测覆盖率曲线 — {PHASES[metrics["phase"]]} | 终态 '
                 f'{metrics["terminal_episodes"]}/{metrics["expected_episodes"]}',fontsize=14,y=.993)
    fig.text(.5,.009,'水平虚线：99% 目标。进行中曲线只到最新已观测步数；未外推。'
             '横轴上限采用各场景预算；曲线使用有界抽稀。',ha='center',fontsize=9)
    fig.tight_layout(rect=(0,.027,1,.969),h_pad=1.5)
    fig.savefig(root/'pressure-coverage.png',dpi=140)
    plt.close(fig)


def _write_metrics(metrics, root):
    (root/'pressure-metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    fields = ['family','extent_m','mode','planned_cases','paired_cases','successes','ended','success_rate',
        'available_successes','available_ended','mean_final_coverage','mean_decisions','mean_distance_m',
        'mean_wall_seconds','mean_fine_nodes','mean_policy_nodes','both_successful_ended',
        'both_successful_wall_seconds','both_successful_distance_m','pending','infrastructure_errors','other_errors']
    with (root/'pressure-metrics.csv').open('w',newline='',encoding='utf-8') as stream:
        writer = csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader()
        for stratum in metrics['strata']:
            for mode, group in stratum['modes'].items():
                row = dict(group,family=stratum['family'],extent_m=stratum['extent_m'],mode=mode,
                    planned_cases=stratum['planned_cases'],paired_cases=len(stratum['paired_cases']))
                writer.writerow({key: row.get(key) for key in fields})
    lines = [f'{metrics["phase"]}: terminal {metrics["terminal_episodes"]}/{metrics["expected_episodes"]}; '
             f'pending {metrics["pending_episodes"]}; errors {metrics["error_episodes"]}; '
             f'complete pairs {len(metrics["paired_cases"])}/{metrics["planned_cases"]}',metrics['limits'],'']
    def value(number, scale=1):
        return 'pending/n.a.' if number is None else f'{number*scale:.3f}'
    for stratum in metrics['strata']:
        lines.append(f'{stratum["family"]} {stratum["extent_m"]}m: pairs '
                     f'{len(stratum["paired_cases"])}/{stratum["planned_cases"]}; '
                     f'joint successes {len(stratum["both_successful_cases"])}')
        for mode, group in stratum['modes'].items():
            lines.append(f'  {LABELS[mode]}: paired success {group["successes"]}/{group["ended"]} '
                f'({value(group["success_rate"],100)}%); coverage {value(group["mean_final_coverage"],100)}%; '
                f'decisions {value(group["mean_decisions"])}; jointly successful wall '
                f'{value(group["both_successful_wall_seconds"])}s/path {value(group["both_successful_distance_m"])}m; '
                f'nodes fine/policy {value(group["mean_fine_nodes"])}/{value(group["mean_policy_nodes"])}; '
                f'available successes/ended {group["available_successes"]}/{group["available_ended"]}; '
                f'pending {group["pending"]}, infrastructure {group["infrastructure_errors"]}, other errors {group["other_errors"]}')
    (root/'pressure-metrics.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def run(root):
    root = Path(root).resolve()
    metrics, curves = collect(root)
    _plots(metrics,curves,root)
    _write_metrics(metrics,root)
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args = parser.parse_args(argv)
    metrics = run(args.root)
    print(f'{metrics["phase"]}: terminal {metrics["terminal_episodes"]}/{metrics["expected_episodes"]}, '
          f'pairs {len(metrics["paired_cases"])}/{metrics["planned_cases"]}; plots and metrics in {metrics["root"]}')


if __name__ == '__main__':
    main()
