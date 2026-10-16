from lunar_drl_exploration.course_two_parallel import evaluation_proof_matches
"""Read-only evidence for the C3 monitor, outside the frozen training runtime."""
from collections import Counter, deque
import hashlib
import json
import os
from pathlib import Path
import shutil
import time

from course_two_trial_rewards import EpisodeRewards


GROUPS = ('moon_outside','moon_inside','cave_inside','new', 'old', 'diagnostic', 'visible', 'moon_cover', 'cave_cover', 'moon_finish', 'cave_finish',
    'cover', 'finish', 'return', 'detour')
NAMES = dict(new='新任务', old='旧课回归', diagnostic='D1–D8', visible='初始化检查',
    cover='持续覆盖', approach='接近区域', finish='覆盖收尾', **{'return': '返回区域'}, detour='区域外绕行',
    moon_inside='月表·区内', moon_outside='月表·区外', cave_inside='洞穴·区内', cave_outside='洞穴·区外',
    success_finish='成功收尾',global_batch_capacity='全局批次供给')
NAMES.update(moon_cover='月表·覆盖',cave_cover='洞穴·覆盖',moon_finish='月表·收尾',cave_finish='洞穴·收尾')
ROW_KEYS = ('case_id', 'kind', 'capability', 'completed', 'collisions', 'error', 'decisions',
    'initial_roi_visible', 'initial_coverage', 'first_roi', 'final_coverage', 'distance_m',
    'truncation_reason', 'max_zero_gain_run', 'zero_gain_two_point_loop_steps','promotion_required','task_scope',
    'initial_completed','scene_profile','evaluation_role','sensor','start_boundary_distance_m')


def name(value):
    value = str(value or '—')
    for prefix in ('c2_A1_', 'new_'):
        if value.startswith(prefix):
            value = value[len(prefix):]
    return NAMES.get(value, value)


def succeeded(row):
    return bool(row.get('completed')) and not row.get('collisions') and not row.get('error')


def aggregate(rows, cases):
    groups = {key: dict(total=0, done=0, success=0, failed=0, errors=0,
        blind=0, approached=0, initial_visible=0,initial_completed=0,policy_total=0) for key in GROUPS}
    failures = Counter()
    for case in cases:
        key = case.get('capability') or case.get('kind')
        if key in groups:
            groups[key]['total'] += 1
    for row in rows:
        key = row.get('capability') or row.get('kind')
        if key not in groups:
            continue
        g = groups[key]
        g['done'] += 1
        error = bool(row.get('error'))
        success = succeeded(row)
        initial=bool(row.get('initial_completed')) and not error
        g['errors'] += int(error)
        g['initial_completed']+=int(initial)
        g['policy_total']+=int(not error and not initial)
        g['success'] += int(success and not initial)
        g['failed'] += int(not error and not success)
        g['initial_visible'] += int(row.get('initial_roi_visible') is True and not error)
        if row.get('initial_roi_visible') is False and not error:
            g['blind'] += 1
            g['approached'] += int(bool(row.get('first_roi')))
        if not success:
            failures['运行错误' if error else '碰撞' if row.get('collisions')
                else row.get('truncation_reason') or '未达覆盖目标'] += 1
    return groups, dict(failures)


def process(pid, now):
    try:
        data = Path(f'/proc/{int(pid)}/stat').read_text().rsplit(')', 1)[1].split()
        if data[0] in ('Z', 'X'):
            return None
        uptime = float(Path('/proc/uptime').read_text().split()[0])
        elapsed = max(0., uptime - int(data[19]) / os.sysconf('SC_CLK_TCK'))
        return dict(pid=pid, elapsed=elapsed, started=now-elapsed)
    except (OSError, ValueError, TypeError, IndexError):
        return None


class Countdown:
    """Monitor-local throughput; never an input to scheduling or promotion."""
    def __init__(self):
        self.key = None
        self.points = deque()

    def estimate(self, key, done, total, now, active):
        if not active:
            self.points.clear()
            self.key = None
            return None
        if key != self.key or (self.points and done < self.points[-1][1]):
            self.points.clear()
            self.key = key
        self.points.append((now, done))
        while len(self.points) > 2 and self.points[1][0] < now-180:
            self.points.popleft()
        span = now-self.points[0][0]
        delta = done-self.points[0][1]
        if span < 30 or delta <= 0:
            return None
        return dict(seconds=max(0, total-done)*span/delta, span=span, count=delta)


class Reader:
    def __init__(self, root, owner_fn):
        self.root = Path(root).resolve()
        self.owner_fn = owner_fn
        self.cache = {}
        self.countdown = Countdown()
        self.system_time = 0
        self.system = {}
        self.cpu = None
        self.episode_rewards = EpisodeRewards(self.root/'arms/C3/training/run/metrics.jsonl')

    def read(self, path, compact=None):
        path = Path(path)
        key = (path, compact)
        try:
            stat = path.stat()
            stamp = (stat.st_mtime_ns, stat.st_size)
            if key not in self.cache or self.cache[key][0] != stamp:
                payload = path.read_bytes()
                value = json.loads(payload)
                if compact == 'result':
                    value = dict(status=value.get('status'), proof=value.get('proof', {}),
                        row={k: value.get('row', {}).get(k) for k in ROW_KEYS}, wall_s=value.get('wall_s'))
                elif compact == 'qualification':
                    value = dict(status=value.get('status'), proof=value.get('proof', {}),
                        rejections=value.get('rejections', []), wall_s=value.get('wall_s'))
                elif compact == 'summary':
                    value = dict(status=value.get('status'), actor_sha=value.get('actor_sha'),
                        rows=[{k: r.get(k) for k in ROW_KEYS} for r in value.get('rows', [])])
                elif compact == 'run':
                    final = value.get('observations',{}).get('final',{})
                    value = {k:final.get(k) for k in ('stop_reason','owned_children_closed')}
                self.cache[key] = (stamp, value, hashlib.sha256(payload).hexdigest())
            return self.cache[key][1]
        except (OSError, ValueError, TypeError, AttributeError):
            self.cache.pop(key, None)
            return None

    def digest(self, path):
        path = Path(path)
        key = (path, 'digest')
        try:
            stat = path.stat()
            stamp = (stat.st_mtime_ns, stat.st_size)
            if key not in self.cache or self.cache[key][0] != stamp:
                with path.open('rb') as stream:
                    hasher = hashlib.sha256()
                    for block in iter(lambda: stream.read(1024**2), b''):
                        hasher.update(block)
                    digest = hasher.hexdigest()
                self.cache[key] = (stamp, digest)
            return self.cache[key][1]
        except OSError:
            return None

    @staticmethod
    def age(path, now):
        try:
            return max(0., now-Path(path).stat().st_mtime)
        except OSError:
            return None

    def resources(self, now):
        if now-self.system_time < 5:
            return self.system
        self.system_time = now
        result = {}
        try:
            memory = {r.split(':')[0]: int(r.split()[1])*1024 for r in Path('/proc/meminfo').read_text().splitlines()}
            result.update(memory=memory['MemTotal'], available=memory['MemAvailable'],
                swap_used=memory['SwapTotal']-memory['SwapFree'], swap_total=memory['SwapTotal'])
            cpu = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:9]))
            total, idle = sum(cpu), cpu[3]+cpu[4]
            if self.cpu and total > self.cpu[0]:
                result['cpu'] = 1-(idle-self.cpu[1])/(total-self.cpu[0])
            self.cpu = (total, idle)
            result['load'] = os.getloadavg()[0]
            result['cpus'] = os.cpu_count()
            result['disk_free'] = shutil.disk_usage(self.root).free
        except (OSError, ValueError, KeyError):
            pass
        self.system = result
        return result

    def qualification(self, identity, config_sha, state, live, now):
        groups, requests, valid, warnings = {}, {}, {}, []
        for split, label in (('training', 'rebuilt'), ('development', 'development')):
            for request in self.read(self.root/'frozen'/f'{split}-requests.json') or []:
                key = (split, request.get('capability',request['role']))
                group = groups.setdefault(key, dict(total=0, done=0, active=0, attempts=0, rejections=0))
                group['total'] += 1
                path = self.root/'qualification'/f"{label}-{request['request_id']}.json"
                requests[str(path)] = request
                result = self.read(path, 'qualification')
                if result is None:
                    if path.exists():
                        warnings.append(path.name[:28]+' 验收结果暂不可读')
                    continue
                proof = dict(runtime=identity.get('runtime'), config_sha=config_sha, request=request, original=False)
                actual = result.get('proof', {})
                adopted = identity.get('qualification_cache_adoptions', {}).get(path.name)
                accepted = actual == proof or (adopted == dict(
                    sha=self.cache[(path, 'qualification')][2], from_runtime=actual.get('runtime'),
                    to_runtime=proof['runtime']) and dict(actual, runtime=proof['runtime']) == proof)
                if result.get('status') != 'passed' or not accepted:
                    warnings.append(path.name[:28]+' 验收身份/状态不符，未计入')
                    continue
                valid[str(path)] = result
                group['done'] += 1
                group['attempts'] += len(result.get('rejections', []))+1
                group['rejections'] += len(result.get('rejections', []))
        workers = []
        for worker in sorted(state.get('qualification_workers', []), key=lambda w: w['slot']):
            request = requests.get(worker['output'], {})
            p = self.read(worker['progress']) or {}
            matched = p.get('request_id') == request.get('request_id') and p.get('slot') == worker['slot']
            current = p if matched else {}
            proc = process(worker.get('pid'), now) if live else None
            split = 'development' if request.get('split') == 'development' else 'training'
            accepted = worker['output'] in valid
            attempts = self.read(worker['output']+'.attempts.json') or {}
            attempt_proof = dict(runtime=identity.get('runtime'),config_sha=config_sha,request=request,original=False)
            rejections = attempts.get('rejections',[]) if attempts.get('proof')==attempt_proof else []
            key=(split,request.get('capability',request.get('role')))
            if key in groups and proc and not accepted:
                groups[key]['active'] += 1
            workers.append(dict(worker, request=request, progress=current,
                split=split, alive=bool(proc), accepted=accepted, rejections=rejections,
                elapsed=proc['elapsed'] if proc else None,
                age=max(0, now-current['updated_unix']) if current.get('updated_unix') else None))
        return dict(groups=groups, done=len(valid), total=len(requests), workers=workers, warnings=warnings,
            rejections=sum(g['rejections'] for g in groups.values())
                +sum(len(w['rejections']) for w in workers if not w['accepted']))

    def evaluation(self, label, identity, config_sha, state, live, now):
        cases = self.read(self.root/'cases.json')
        planned = not isinstance(cases, list)
        if planned:
            cases = list(self.read(self.root/'frozen'/('cases-base.json' if identity.get('task_scope') else 'cases-v2.json')) or [])
            cases += [dict(case_id='dev-'+r['request_id'], kind='capability', capability=r.get('capability',r['role']))
                for r in self.read(self.root/'frozen/development-requests.json') or []]
        actor = self.root/'frozen/block31.pt' if label == 'baseline' else self.root/(label+'.pt')
        actor_sha = self.digest(actor)
        rows, warnings = [], []
        directory = self.root/'evaluation'/label
        if not planned:
            for case in cases:
                path = directory/(case['case_id']+'.json')
                result = self.read(path, 'result')
                if not result:
                    if path.exists():
                        warnings.append(case['case_id']+' 结果暂不可读')
                    continue
                proof = dict(actor_sha=actor_sha, config_sha=config_sha, runtime=identity.get('runtime'), case=case)
                row = result.get('row', {})
                if (not actor_sha or not identity.get('runtime') or not evaluation_proof_matches(self.root,path,result,proof,identity)
                        or result.get('status') not in ('finished', 'failed')
                        or row.get('case_id') != case['case_id'] or row.get('kind') != case['kind']
                        or (row.get('capability') != case.get('capability') and not row.get('error'))):
                    warnings.append(case['case_id']+' 结果身份不符，未计入')
                    continue
                if row.get('error') and case.get('capability'):
                    row = dict(row, capability=case['capability'])
                rows.append(row)
        groups, failures = aggregate(rows, cases)
        summary = self.read(directory/'summary.json', 'summary') or {}
        complete = bool(cases and not planned and len(rows) == len(cases) and not warnings
            and not any(r.get('error') for r in rows) and summary.get('status') == 'finished'
            and summary.get('actor_sha') == actor_sha and summary.get('rows') == rows)
        workers = []
        if state.get('evaluation_label') == label and state.get('phase') == 'evaluation':
            by_id = {c['case_id']: c for c in cases}
            for worker in sorted(state.get('evaluation_workers', []), key=lambda w: w['slot']):
                request = self.read(self.root/worker['request']) or {}
                p = self.read(self.root/worker['progress']) or {}
                case = by_id.get(worker['case_id'], {})
                matched = (case and request.get('case') == case and request.get('actor') == str(actor)
                    and p.get('actor') == str(actor) and p.get('case_id') == worker['case_id']
                    and p.get('job_id') == worker.get('job_id') and p.get('slot') == worker['slot'])
                current = p if matched else {}
                proc = process(worker.get('pid'), now) if live else None
                workers.append(dict(worker, case=case, progress=current, alive=bool(proc),
                    elapsed=max(0, now-worker.get('started_unix', now)),
                    age=max(0, now-current['updated_unix']) if current.get('updated_unix') else None))
        return dict(label=label, total=len(cases), done=len(rows), rows=rows, groups=groups,
            failures=failures, warnings=warnings, workers=workers, complete=complete, planned=planned,
            actor_sha=actor_sha)

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        state = self.read(self.root/'state.json') or {}
        identity = self.read(self.root/'identity.json') or {}
        warnings = []
        try:
            live = self.owner_fn(self.root)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            live = None
            warnings.append('控制器身份核查失败：'+str(exc))
        status = state.get('status', 'missing')
        if status == 'running' and not live:
            status = 'interrupted'
            warnings.append('状态文件仍为 running，但控制器已退出；旧指标不是实时进度。')
        config_sha = self.digest(self.root/'config.json')
        qualification = self.qualification(identity, config_sha, state, live, now)
        phase = state.get('phase')
        baseline = self.evaluation('baseline', identity, config_sha, state, live, now)
        candidate = self.evaluation(state.get('candidate_label','C3'), identity, config_sha, state, live, now)
        evaluation = baseline if state.get('evaluation_label', 'baseline') == 'baseline' else candidate
        run = self.root/'arms/C3/training/run'
        progress = self.read(run/'progress.json') or {}
        progress_age = self.age(run/'progress.json', now)
        child = process(state.get('child_pid'), now) if live else None
        fresh = bool(child and progress_age is not None and now-progress_age >= child['started']-1)
        active_training = bool(status == 'running' and phase in ('training', 'prefill') and fresh)
        current = qualification if phase == 'qualification' else evaluation
        eta = self.countdown.estimate((live and live['pid'], phase, evaluation['label']),
            current['done'], current['total'], now, bool(status == 'running' and live
                and phase in ('qualification', 'evaluation') and not current['warnings']))
        for part in (qualification, baseline, candidate):
            warnings += part['warnings']
        gate = self.read(self.root/'arms/C3/gate.json') or state.get('results', {}).get('C3') or {}
        if gate and (not candidate['complete'] or not baseline['complete']
                or gate.get('actor_sha') != candidate['actor_sha']):
            warnings.append('已有门槛记录缺少本轮完整评测/模型证据，暂不显示通过结论。')
            gate = {}
        checkpoint_path = run/'resume.pt'
        reward_boundary = ((identity.get('reward_adjustment') or {}).get('boundary') or {}).get('transitions', 0)
        episode_rewards = self.episode_rewards.snapshot(progress, boundary=reward_boundary)
        return dict(root=str(self.root), now=now, state=state, identity=identity, live=live, status=status,
            state_age=self.age(self.root/'state.json', now), warnings=warnings,
            qualification=qualification, baseline=baseline, candidate=candidate, evaluation=evaluation,
            progress=progress, progress_age=progress_age, active_training=active_training,
            episode_rewards=episode_rewards, episode_rewards_age=self.age(run/'metrics.jsonl', now),
            resources=self.resources(now), eta=eta,
            limits={kind:(self.read(self.root/(kind+'-settings.json')) or {}).get('workers')
                for kind in ('qualification', 'evaluation', 'training')},
            checkpoint_age=self.age(checkpoint_path, now), checkpoint_path=str(checkpoint_path),
            prefill=self.read(self.root/'arms/C3/prefill-proof.json') or {},
            gate=gate,
            exit=self.read(run/'run.json','run') or {})

    def log_tail(self, maximum=18):
        try:
            with (self.root/'logs/controller.log').open('rb') as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell()-8192))
                return stream.read().decode('utf-8', errors='replace').splitlines()[-maximum:]
        except OSError:
            return []
