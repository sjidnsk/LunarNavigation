"""On-demand or 60-second pressure report supervisor; no native process control.

System Python: ``python3 pressure_report.py --root ROOT --watch``.
Use ``--once`` to recompute final figures and the Chinese Markdown report.
Only plot/metrics/report artifacts are written. No runtime, Actor inference,
process launches, signals, navigation or training are performed.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

if __package__:
    from . import pressure_plot
else:
    import pressure_plot

TERMINAL = {'finished', 'finished_with_errors', 'error'}


def _stat(path):
    try:
        info = Path(path).stat()
        return info.st_ino, info.st_size, info.st_mtime_ns
    except FileNotFoundError:
        return None


def read_json(path, *, optional=False, attempts=3, pause=time.sleep):
    """Retry bounded document races, but never accept a persistent bad document."""
    path = Path(path)
    error = None
    for attempt in range(attempts):
        before = _stat(path)
        if before is None and optional:
            return None
        try:
            text = path.read_text(encoding='utf-8')
            value = json.loads(text)
            if _stat(path) != before:
                raise ValueError('文件读取期间发生变化')
            if not isinstance(value, dict):
                raise ValueError('JSON 文档必须是对象')
            return value
        except (FileNotFoundError, json.JSONDecodeError, ValueError) as exc:
            error = exc
            if attempt+1 < attempts:
                pause(.05)
    raise ValueError(f'{path}: 无法读取完整 JSON 文档: {error}') from error


def _atomic_report(root, content):
    fd, temporary = tempfile.mkstemp(prefix='.pressure-report.',suffix='.md',dir=root)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as stream:
            stream.write(content)
            stream.flush()
        os.replace(temporary,root/'pressure-report.md')
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _fmt(value, *, scale=1., unit='', digits=2):
    if value is None:
        return '—（未知）'
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('报告指标必须有限')
    return f'{number*scale:.{digits}f}{unit}'


def _escape(value):
    return str(value).replace('|','\\|').replace('\n',' ')


def _status(metrics, state):
    dispatcher = state.get('status') if state else None
    complete = metrics['terminal_episodes'] == metrics['expected_episodes']
    if dispatcher in TERMINAL and not complete:
        return '异常', '实验中断：调度器已结束，但计划回合尚未全部生成终态记录。'
    if metrics['error_episodes'] or dispatcher in {'error','finished_with_errors'}:
        return '异常', '存在基础设施/执行错误；错误回合独立列示，不能当作正常失败或补零。'
    if dispatcher == 'finished' and complete:
        return '完成', '计划回合已全部终止，调度器终态已核对。'
    if complete:
        return '进行中', '回合数据已齐，等待调度器终态核对。'
    return '进行中', '计划回合仍有进行中或未开始项；本报告是部分结果。'


def _validate(metrics, root, protocol, maxima):
    if metrics.get('schema') != 'region_pressure_plot_v1' or metrics.get('root') != str(root):
        raise ValueError('metrics schema/root 不匹配')
    expected = len(protocol['cases'])*len(protocol['modes'])
    if metrics.get('expected_episodes') != expected or metrics.get('cases') != protocol['cases'] or metrics.get('modes') != protocol['modes']:
        raise ValueError('metrics 与 protocol 计划不一致')
    if not 0 <= metrics['terminal_episodes'] <= expected:
        raise ValueError('metrics 终态数量不合法')
    if metrics.get('protocol_sha') != hashlib.sha256((root/'protocol.json').read_bytes()).hexdigest():
        raise ValueError('metrics/protocol SHA 不一致，需重算')
    if maxima is not None:
        if maxima.get('schema') != 'pressure_resource_maxima_v1' or maxima.get('root') != str(root):
            raise ValueError('资源 maxima schema/root 不匹配')
        if not isinstance(maxima.get('cases'),dict):
            raise ValueError('资源 maxima 缺少 cases')


def render_report(root, metrics, protocol, state=None, maxima=None):
    """Render one validated metric snapshot, keeping endpoint populations explicit."""
    root = Path(root).resolve()
    _validate(metrics,root,protocol,maxima)
    status, explanation = _status(metrics,state)
    modes = metrics['modes']
    successful = sum(episode.get('successful') is True for episode in metrics['episodes'])
    ended = sum(episode['status']=='finished' for episode in metrics['episodes'])
    infrastructure = sum(episode['status']=='infrastructure_error' for episode in metrics['episodes'])
    other = sum(episode['status']=='other_error' for episode in metrics['episodes'])
    lines = ['# A0 / A2 压力测试报告','',f'**状态：{status}**。{explanation}',
        f'检查时间：{datetime.now(timezone.utc).isoformat()}；指标快照：{metrics["generated_utc"]}。',
        f'计划 {metrics["planned_cases"]} 个场景、{metrics["expected_episodes"]} 回合；终态 '
        f'{metrics["terminal_episodes"]}/{metrics["expected_episodes"]}；正常结束成功 '
        f'{successful}/{ended}；基础设施错误 {infrastructure}；其他错误 {other}；'
        f'进行中/未开始 {metrics["pending_episodes"]}；完整配对 '
        f'{len(metrics["paired_cases"])}/{metrics["planned_cases"]}。','',
        '成功由冻结环境的无碰撞 completed 终态判定。错误不进入 successes/ended；'
        '进行中覆盖率、步数和墙钟保持未知，不能补零。',
        '下表只使用完整身份配对；单臂已结束的原始 successes/ended 另列。'
        '覆盖率和步数包含成功及预算失败等全部正常终态。','',
        '## 按场景类型与尺度分层','',
        '| 类型 | 尺度 m | 模式 | 完整配对/计划 | 配对成功/结束 | 成功率 | 平均覆盖率 | 平均决策步数 | 单臂原始成功/结束 | 进行中 | 基础设施/其他错误 |',
        '|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for stratum in metrics['strata']:
        family = {'moon':'月面','cave':'洞穴'}.get(stratum['family'],stratum['family'])
        for mode in modes:
            group = stratum['modes'][mode]
            lines.append(f'| {_escape(family)} | {stratum["extent_m"]} | {pressure_plot.LABELS[mode]} | '
                f'{len(stratum["paired_cases"])}/{stratum["planned_cases"]} | '
                f'{group["successes"]}/{group["ended"]} | {_fmt(group["success_rate"],scale=100,unit="%")} | '
                f'{_fmt(group["mean_final_coverage"],scale=100,unit="%")} | {_fmt(group["mean_decisions"])} | '
                f'{group["available_successes"]}/{group["available_ended"]} | {group["pending"]} | '
                f'{group["infrastructure_errors"]}/{group["other_errors"]} |')
    lines += ['', '## 双方均成功的描述性效率','',
        '墙钟和路径仅来自 A0/A2 均达到完成条件的同一场景子集。'
        '单臂预算耗尽的全路径/墙钟不等于达到 99% 的效率，不能混入此表。'
        '成功子集存在选择边界，不能据此断言 A2 在所有场景普遍更好。','',
        '| 类型 | 尺度 m | 模式 | 双成功配对数 | 平均墙钟 s | 平均路径 m |',
        '|---|---:|---|---:|---:|---:|']
    for stratum in metrics['strata']:
        for mode in modes:
            group = stratum['modes'][mode]
            family = {'moon':'月面','cave':'洞穴'}.get(stratum['family'],stratum['family'])
            lines.append(f'| {_escape(family)} | {stratum["extent_m"]} | {pressure_plot.LABELS[mode]} | '
                f'{group["both_successful_ended"]} | {_fmt(group["both_successful_wall_seconds"])} | '
                f'{_fmt(group["both_successful_distance_m"])} |')
    lines += ['', '## 图规模与压缩','',
        '下表仍采用完整配对，先按每回合求图均值，再等权汇总。'
        '压缩率为 1−每回合 policy/fine 比值的均值；并非不同回合节点均值相除。'
        'A2 包含图收缩和远程动作扩展；冻结 Actor 不变。细图观察仍然执行，压缩不代表细图构建成本消失。','',
        '| 类型 | 尺度 m | 模式 | 平均细图节点 | 平均策略节点 | 节点压缩率 | 边压缩率 |',
        '|---|---:|---|---:|---:|---:|---:|']
    for stratum in metrics['strata']:
        for mode in modes:
            group = stratum['modes'][mode]
            node_ratio = group.get('graph',{}).get('mean_node_ratio')
            edge_ratio = group.get('graph',{}).get('mean_edge_ratio')
            family = {'moon':'月面','cave':'洞穴'}.get(stratum['family'],stratum['family'])
            lines.append(f'| {_escape(family)} | {stratum["extent_m"]} | {pressure_plot.LABELS[mode]} | '
                f'{_fmt(group["mean_fine_nodes"])} | {_fmt(group["mean_policy_nodes"])} | '
                f'{_fmt(None if node_ratio is None else 1-node_ratio,scale=100,unit="%")} | '
                f'{_fmt(None if edge_ratio is None else 1-edge_ratio,scale=100,unit="%")} |')
    lines += ['', '## 资源采样与边界','',
        'RSS 是进程树常驻页之和，会重复计入共享页；PSS 按全系统共享者分摊。'
        '两者分别记录采样峰值，PSS 缺失保持未知，不用 RSS 替代。'
        '每约 10 秒采样可能遗漏瞬时峰值；以下数值是采样最大值而非真实峰值保证。'
        '采样器在实验开始后启动，前三个终态未覆盖完整；其余回合也不能据样本数声称全时段覆盖。','',
        f'资源采样总次数：{maxima.get("samples", "未知") if maxima else "未知（暂无 maxima）"}；'
        f'最小宿主可用内存 MiB：{_fmt(maxima.get("minimum_host_available_kib") if maxima else None,scale=1/1024)}。',
        '宿主内存/负载与其他系统工作共享；本次并行/串行资源条件不同，不是排他的机器资源测量。','',
        '| 场景 | 模式 | 总样本 | 完整样本 | 归属拒绝样本 | RSS 采样最大 MiB | PSS 采样最大 MiB |',
        '|---|---|---:|---:|---:|---:|---:|']
    resources = maxima.get('cases',{}) if maxima else {}
    for case in protocol['cases']:
        for mode in modes:
            row = resources.get(case['name']+'/'+mode,{})
            lines.append(f'| {_escape(case["name"])} | {pressure_plot.LABELS[mode]} | '
                f'{row.get("samples","—（未知）")} | {row.get("complete_samples","—（未知）")} | '
                f'{row.get("rejected_samples","—（未知）")} | '
                f'{_fmt(row.get("rss_max_kib"),scale=1/1024)} | {_fmt(row.get("pss_max_kib"),scale=1/1024)} |')
    lines += ['', '## 身份与结论边界','',
        '严格配对要求场景类型/尺度/seed/预算、Actor、配置、原始出生姿态、地形、参考真值、'
        '工作区域、运行包/native 文件、实验源码和 wrapper 一致。'
        '初始化扫描的时序/姿态变化会影响初始观察哈希；该哈希只作诊断，不构成硬配对条件。',
        '每个类型×尺度通常仅 2 个 seed；分档结果是本计划的描述性观测，'
        '不能把混合尺度总均值或部分结果解释为 A2 普遍更优、重训练策略质量或实车验证。',
        '预算规则：`'+_escape(protocol.get('budget_rule','未记录'))+'`；起点模式：`'+_escape(protocol.get('start_mode','未记录'))+'`。',
        '预算按面积公式外推并向上取整至 32 的倍数，属于本压力协议；不等同于旧课程预算。','',
        '| 冻结/辅助身份 | SHA / 记录 |','|---|---|']
    for key in ('actor_sha','config_sha','source_commit','experiment_commit','wrapper_sha'):
        lines.append(f'| {key} | `{_escape(protocol.get("frozen",{}).get(key,"未知"))}` |')
    frozen = protocol.get('frozen',{})
    for name, sha in frozen.get('experiment_sources',{}).items():
        lines.append(f'| experiment_source/{_escape(name)} | `{_escape(sha)}` |')
    runtime = frozen.get('runtime',{})
    lines.append(f'| runtime/package_sha | `{_escape(runtime.get("package_sha","未知"))}` |')
    for category in ('modules','navigation'):
        for name, entry in runtime.get(category,{}).items():
            lines.append(f'| runtime/{_escape(name)} | `{_escape(entry.get("sha","未知"))}` |')
    for name in ('python','numpy','scipy'):
        lines.append(f'| runtime/{name} 版本 | `{_escape(runtime.get(name,"未知"))}` |')
    for key in ('protocol_sha','plot_source_sha','compare_source_sha'):
        lines.append(f'| {key} | `{metrics[key]}` |')
    lines.append(f'| report_source_sha | `{hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}` |')
    lines.append(f'| resource_collector_sha | `{_escape(maxima.get("collector_sha","未知") if maxima else "未知")}` |')
    lines += ['', '[分档对比图](pressure-comparison.png) · [覆盖率曲线](pressure-coverage.png) · '
              '[完整指标](pressure-metrics.json) · [分层 CSV](pressure-metrics.csv)','',
              '监督器只读取数据并生成上述文件；不会启动/停止任何 native、导航、Actor 推理或训练进程。','']
    return '\n'.join(lines)


class Supervisor:
    def __init__(self,root,*,plotter=None,pause=time.sleep):
        self.root = Path(root).resolve()
        self.plotter = plotter or pressure_plot.run
        self.pause = pause
        self.metrics = None
        self.fingerprint = None
        self.curve_cache = {}

    def _progress(self,protocol):
        rows = [hashlib.sha256((self.root/'protocol.json').read_bytes()).hexdigest()]
        for case in protocol['cases']:
            for mode in protocol['modes']:
                base = self.root/case['name']/mode
                summary = read_json(base/'summary.json',optional=True,pause=self.pause)
                summary_sha = hashlib.sha256(json.dumps(summary,sort_keys=True,allow_nan=False).encode()).hexdigest()
                path = base/'steps.jsonl'
                stat = _stat(path)
                cached = self.curve_cache.get(path)
                if cached is None or cached[0] != stat:
                    curve = pressure_plot.read_curve(path,max_points=3)
                    progress = (curve['read_records'],tuple(curve['points'][-1]) if curve['points'] else None)
                    self.curve_cache[path] = (stat,progress)
                rows.append((case['name'],mode,summary_sha,self.curve_cache[path][1]))
        return tuple(rows)

    def _plot(self):
        # Frozen writers use atomic documents. Retry transient rename/read races;
        # complete malformed JSONL remains ValueError and is never swallowed.
        error = None
        for attempt in range(3):
            try:
                return self.plotter(self.root)
            except (json.JSONDecodeError,FileNotFoundError) as exc:
                error = exc
                if attempt<2:
                    self.pause(.05)
        raise ValueError(f'绘图快照读取失败: {error}') from error

    def tick(self,*,force=False):
        protocol = read_json(self.root/'protocol.json',pause=self.pause)
        state = read_json(self.root/'run-state.json',optional=True,pause=self.pause)
        fingerprint = self._progress(protocol)
        terminal = bool(state and state.get('status') in TERMINAL)
        plotted = force or self.metrics is None or fingerprint != self.fingerprint or terminal
        if plotted:
            metrics = self._plot()
            # If protocol was replaced during the operation, the snapshot must
            # match the newly read protocol before any report can be accepted.
            protocol = read_json(self.root/'protocol.json',pause=self.pause)
            self.metrics = metrics
            self.fingerprint = fingerprint
        maxima = read_json(self.root/'maxima.json',optional=True,pause=self.pause)
        report = render_report(self.root,self.metrics,protocol,state,maxima)
        _atomic_report(self.root,report)
        status,_ = _status(self.metrics,state)
        return dict(status=status,done=terminal,plot_updated=plotted,metrics=self.metrics)

    def failure_report(self,error):
        message = ('# A0 / A2 压力测试报告\n\n**状态：异常**。数据读取或身份校验失败；'
                   '监督器已退出，未操作任何实验进程。\n\n'+_escape(error)+'\n\n'
                   '旧图/指标可能仍是先前快照，不能当作本次校验通过的结果。\n')
        _atomic_report(self.root,message)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--watch',action='store_true',help='每60秒检查，仅实际数据变化时重画')
    mode.add_argument('--once',action='store_true',help='立即重算一次；默认行为')
    args = parser.parse_args(argv)
    supervisor = Supervisor(args.root)
    try:
        while True:
            result = supervisor.tick(force=not args.watch)
            metrics = result['metrics']
            print(f'{result["status"]}: 终态 {metrics["terminal_episodes"]}/{metrics["expected_episodes"]}, '
                  f'完整配对 {len(metrics["paired_cases"])}/{metrics["planned_cases"]}, '
                  f'图更新={result["plot_updated"]}',flush=True)
            if not args.watch or result['done']:
                return 1 if result['status']=='异常' else 0
            time.sleep(60.)
    except KeyboardInterrupt:
        return 130  # Stops only this independent supervisor.
    except (OSError,ValueError,KeyError,TypeError) as error:
        try:
            supervisor.failure_report(error)
        except OSError:
            pass
        print(f'监督器异常: {error}',file=sys.stderr,flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
