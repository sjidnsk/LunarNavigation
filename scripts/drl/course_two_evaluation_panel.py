"""Read-only, cached evaluation evidence for the terminal monitor.

Keep this outside the training package: presentation changes must not change the
runtime identity of a running experiment. No model loading or gate mutation.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time


KINDS = ('new', 'old', 'diagnostic', 'visible')
ROW_FIELDS = ('case_id', 'kind', 'completed', 'collisions', 'error', 'decisions',
    'initial_roi_visible', 'first_roi', 'final_coverage', 'distance_m',
    'truncation_reason', 'max_zero_gain_run', 'zero_gain_two_point_loop_steps')


def compact_row(row):
    return {key: row.get(key) for key in ROW_FIELDS}


def compact_result(value):
    return dict(status=value.get('status'), proof=value.get('proof', {}),
        row=compact_row(value.get('row', {})), wall_s=value.get('wall_s'))


def compact_summary(value):
    return dict(status=value.get('status'), actor_sha=value.get('actor_sha'),
        rows=[compact_row(row) for row in value.get('rows', [])])


def succeeded(row):
    return bool(row.get('completed')) and not row.get('collisions') and not row.get('error')


def aggregates(rows, cases):
    groups = {kind: dict(total=sum(c['kind'] == kind for c in cases), done=0,
        success=0, failed=0, blind=0, approached=0, initial_visible=0) for kind in KINDS}
    failures = Counter()
    for row in rows:
        group = groups[row['kind']]
        success = succeeded(row)
        group['done'] += 1
        group['success'] += int(success)
        group['failed'] += int(not success)
        group['initial_visible'] += int(row.get('initial_roi_visible') is True)
        if row.get('initial_roi_visible') is False and not row.get('error'):
            group['blind'] += 1
            group['approached'] += int(bool(row.get('first_roi')))
        if not success:
            reason = ('ERROR' if row.get('error') else 'COLLISION' if row.get('collisions')
                else row.get('truncation_reason') or 'INCOMPLETE')
            failures[reason] += 1
    return groups, dict(failures)


class EvaluationReader:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.cache = {}

    def _read(self, path, transform=None, *, digest=False):
        """Cache compact records, never hundreds of full trajectory arrays."""
        path = Path(path)
        key = (path, digest)
        try:
            stat = path.stat()
            stamp = (stat.st_mtime_ns, stat.st_size)
            old = self.cache.get(key)
            if old is not None and old[0] == stamp:
                return old[1]
            if digest:
                with path.open('rb') as stream:
                    value = hashlib.file_digest(stream, 'sha256').hexdigest()
            else:
                value = json.loads(path.read_text(encoding='utf-8'))
                if transform is not None:
                    value = transform(value)
            self.cache[key] = (stamp, value)
            return value
        except (OSError, ValueError, TypeError, AttributeError, KeyError):
            self.cache.pop(key, None)
            return None

    def _cases(self, label, stage):
        name = getattr(self,'case_file',None) or f'cases-{stage}.json'
        if label.startswith('B-easy-'):
            name = 'cases-B-easy-' + label.split('-')[2] + '.json'
        cases = self._read(self.root/name)
        if not isinstance(cases, list):
            return []
        if label == 'baseline' and not getattr(self,'full_baseline',False):
            cases = [case for case in cases if case.get('kind') in ('old', 'diagnostic')]
        return [case for case in cases if isinstance(case, dict)
            and isinstance(case.get('case_id'), str)
            and Path(case['case_id']).name == case['case_id']
            and case['case_id'] not in ('.', '..') and case.get('kind') in KINDS]

    def _summary(self, label, stage, actor_sha):
        summary = self._read(self.root/getattr(self,'directory','evaluation')/label/'summary.json', compact_summary)
        cases = self._cases(label, stage)
        if not summary or not cases or summary['status'] != 'finished' or not actor_sha:
            return None
        ids = [row['case_id'] for row in summary['rows']]
        expected = {case['case_id']: case['kind'] for case in cases}
        if (summary['actor_sha'] != actor_sha or len(ids) != len(expected)
                or set(ids) != set(expected)
                or any(row['kind'] != expected[row['case_id']] for row in summary['rows'])):
            return None
        return summary

    def snapshot(self, state, *, now=None):
        now = time.time() if now is None else now
        self.directory=state.get('evaluation_directory','evaluation')
        self.case_file=state.get('evaluation_case_file')
        self.full_baseline=bool(state.get('reevaluation'))
        for name in (self.directory,self.case_file):
            if name is not None and (Path(name).name!=name or name in ('.','..')):
                raise ValueError('invalid evaluation evidence path')
        stage = state['stage']
        label = state.get('evaluation_label') or f"{stage}-block{state.get('stage_block', 0)+1}"
        if Path(label).name != label or label in ('.', '..'):
            raise ValueError('invalid evaluation label')
        label_stage = label.split('-')[0]
        if label_stage in ('A1', 'A2', 'B', 'C', 'D'):
            stage = label_stage
        cases = self._cases(label, stage)
        actor = self.root/'frozen/block31.pt' if label == 'baseline' else self.root/(label+'.pt')
        actor_sha = self._read(actor, digest=True)
        config_sha = self._read(self.root/'config.json', digest=True)
        identity = self._read(self.root/'identity.json') or {}
        rows, warnings = [], []
        if not cases:
            warnings.append('未读取到本轮场景清单，暂不汇总结果')
        if not actor_sha:
            warnings.append('未读取到本轮模型，暂不接收结果')
        for case in cases:
            path = self.root/self.directory/label/(case['case_id']+'.json')
            if not path.exists():
                continue
            record = self._read(path, compact_result)
            if record is None:
                warnings.append(case['case_id']+' 结果暂不可读')
                continue
            proof = record['proof']
            row = record['row']
            if (not actor_sha or not config_sha or not identity.get('runtime')
                    or proof.get('actor_sha') != actor_sha or proof.get('case') != case
                    or proof.get('config_sha') != config_sha
                    or proof.get('runtime') != identity['runtime']
                    or row['case_id'] != case['case_id'] or row['kind'] != case['kind']
                    or record['status'] not in ('finished', 'failed')):
                warnings.append(case['case_id']+' 结果身份不符，未计入')
                continue
            rows.append(row)
        groups, failures = aggregates(rows, cases)
        summary = self._summary(label, stage, actor_sha)
        complete = bool(summary and cases and len(rows) == len(cases) and not warnings)

        # Request, live snapshot, model and controller child must identify one case.
        request = self._read(self.root/'evaluation-request.json') or {}
        requested = request.get('case', {})
        current_case = next((case for case in cases if case == requested), None)
        current = None
        if current_case is not None and request.get('actor') == str(actor):
            cid = current_case['case_id']
            spec = current_case.get('case', current_case.get('spec', {}))
            live = state.get('evaluation', {})
            active = state.get('status') == 'running' and state.get('phase') == 'evaluation'
            matched = (active and live.get('case_id') == cid and live.get('actor') == str(actor)
                and state.get('child_label') == 'eval-'+label+'-'+cid
                and live.get('updated_unix', 0) >= state.get('updated_unix', 0)
                and cid not in {row['case_id'] for row in rows})
            age = max(0., now-live['updated_unix']) if matched else None
            current = dict(case_id=cid, kind=current_case['kind'], spec=spec,
                budget=spec.get('episode_budget', current_case.get('budget')),
                live=dict(live) if matched else None, age_s=age,
                stale=bool(age is not None and age > 15), active=active,
                elapsed_s=max(0., now-state.get('updated_unix', now)) if matched else None)

        workers=[]
        if 'evaluation_workers' in state:
            current=None  # Parallel workers never inherit the old global snapshot.
            by_id={case['case_id']:case for case in cases}
            completed={row['case_id'] for row in rows}
            active=state.get('status')=='running' and state.get('phase')=='evaluation'
            for worker in state['evaluation_workers']:
                cid=worker.get('case_id');case=by_id.get(cid)
                if case is None or cid in completed:continue
                request_path=(self.root/worker['request']).resolve()
                progress_path=(self.root/worker['progress']).resolve()
                if not request_path.is_relative_to(self.root.resolve()) or not progress_path.is_relative_to(self.root.resolve()):
                    warnings.append('并行评测路径无效');continue
                request=self._read(request_path) or {};live=self._read(progress_path) or {}
                if request.get('actor')!=str(actor) or request.get('case')!=case:
                    warnings.append(cid+' 并行请求身份不符');continue
                matched=(active and live.get('case_id')==cid and live.get('actor')==str(actor)
                    and live.get('slot')==worker['slot'] and live.get('job_id')==worker['job_id']
                    and live.get('domain_id')==worker['domain_id']
                    and live.get('updated_unix',0)>=worker['started_unix'])
                age=max(0.,now-live['updated_unix']) if matched else None
                spec=case.get('case',case.get('spec',{}))
                workers.append(dict(worker,kind=case['kind'],spec=spec,
                    budget=spec.get('episode_budget',case.get('budget')),live=dict(live) if matched else None,
                    age_s=age,stale=bool(age is not None and age>15),active=active,
                    elapsed_s=max(0.,now-worker['started_unix'])))

        history = []
        baseline_actor = self._read(self.root/'frozen/block31.pt', digest=True)
        baseline = self._summary('baseline', stage, baseline_actor)
        if baseline:
            history.append(dict(label='block31', groups=aggregates(baseline['rows'],
                self._cases('baseline', stage))[0], gate=None))
        for gate in state.get('history', [])[-4:]:
            name = f"{gate['stage']}-block{gate['block']}"
            saved = self._summary(name, gate['stage'], gate.get('actor_sha'))
            history.append(dict(label=name, groups=aggregates(saved['rows'],
                self._cases(name, gate['stage']))[0] if saved else None, gate=gate))
        gate = next((entry['gate'] for entry in history if entry['label'] == label), None) if complete else None
        return dict(stage=stage, label=label, done=len(rows), total=len(cases) or state.get('evaluation_total', 0),
            complete=complete, current=current, workers=workers,groups=groups, failures=failures,
            rows=rows, history=history, gate=gate, warnings=warnings)
