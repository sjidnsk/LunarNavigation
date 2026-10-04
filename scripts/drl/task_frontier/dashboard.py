"""Fixed curses terminal dashboard over files; never drives policy decisions."""
import curses,json,math,os,signal,time,unicodedata
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
ROOT=Path('/home/kai/CodexDownloads/lunar_navigation/drl-task-frontier-20261003/managed')

def read(path,default=None):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return default

def duration(seconds):
    seconds=max(0,int(seconds));return f'{seconds//3600:02}:{seconds//60%60:02}:{seconds%60:02}'

def clip(text,width):
    output=[];used=0
    for char in text:
        size=2 if unicodedata.east_asian_width(char) in ('W','F') else 1
        if used+size>width:break
        output.append(char);used+=size
    return ''.join(output)

def latest_progress():
    path=ROOT/'run/metrics.jsonl'
    try:
        with path.open('rb') as f:
            f.seek(0,2);end=f.tell();f.seek(max(0,end-128*1024));lines=f.read().decode('utf-8',errors='replace').splitlines()
        for line in reversed(lines):
            try:r=json.loads(line)
            except ValueError:continue
            if r.get('event')=='progress':return r
    except OSError:pass
    return {}

def completed_cases(root,label,manifest=None):
    """Only terminal, matching results contribute to coverage statistics."""
    rows={}
    expected=read(root/'cases.json',{}).get('cases',[])
    for case in expected:
        index=case['case_index'];record=read(root/f'{label}-case-{index:02}.json',{})
        if record.get('status')!='finished' or record.get('exit_code')!=0:continue
        results=record.get('cases',[])
        if len(results)!=1:continue
        row=results[0];coverage=row.get('final_coverage')
        if not all(row.get(key)==case.get(key) for key in ('case_index','family','seed','extent_m','budget')):continue
        if (row.get('error') is not None or row.get('exit_code')!=0
                or not (row.get('terminated') or row.get('truncated'))
                or type(coverage) not in (int,float) or not math.isfinite(coverage) or not 0<=coverage<=1):continue
        if manifest and (row.get('run_id')!=manifest.get('run_id') or row.get('actor_sha')!=manifest.get('actor_sha')):continue
        rows[index]=row
    return rows

def coverage_lines(manifest):
    label=manifest.get('label',f"stage{manifest.get('stage',0)}")
    if label=='baseline':label='baseline'
    current=completed_cases(ROOT,label,manifest if manifest.get('phase')=='evaluation' else None)
    baseline=completed_cases(ROOT,'baseline')
    if not current and manifest.get('phase')!='evaluation':return []
    cases=read(ROOT/'cases.json',{}).get('cases',[])
    def pct(row):return f"{row['final_coverage']:6.2%}" if row else ' 待评 '
    result=['',f'已完成场景覆盖率 · {label}（括号内为 26856 基线）',
            '组   尺寸(m)      月表覆盖率(基线)       洞穴覆盖率(基线)']
    for group in range((len(cases)+1)//2):
        pair=cases[group*2:group*2+2]
        values=[f'{pct(current.get(c["case_index"]))} ({pct(baseline.get(c["case_index"]))})' for c in pair]
        result.append(f'{group+1:2}   {pair[0]["extent_m"]:7.2f}      '+ '       '.join(values))
    for family,name in (('moon','月表'),('cave','洞穴')):
        rows=[r for r in current.values() if r['family']==family]
        mean=f"{sum(r['final_coverage'] for r in rows)/len(rows):.2%}" if rows else '—'
        result.append(f'{name}：已评 {len(rows)}/10 · 平均覆盖率 {mean} · 完成 {sum(bool(r["completed"]) for r in rows)}/{len(rows)}')
    return result

def lines(manifest,progress,evaluation):
    stage=manifest.get('stage',0);phase=manifest.get('phase','等待状态')
    output=['任务内外前沿 · 从零训练监看',
        '位置辅助损失 0  |  独立位置/朝向熵  |  月表、洞穴 50–100m  |  512 决策',
        '─'*70,
        f"状态：{manifest.get('status','等待')}   阶段：{'预热' if stage==0 else str(stage)+'/10'}   任务：{phase}"]
    if phase=='evaluation':
        output.extend([f"已完成：{manifest.get('eval_completed',0)}/20   当前组：{manifest.get('case_index',0)//2+1}/10",
            f"当前地图：{manifest.get('family','—')} {manifest.get('extent_m',0):.2f}m seed={manifest.get('seed','—')}"])
        # The old episode's telemetry must not be presented as the new episode.
        expected=f"{manifest.get('family','')} {manifest.get('extent_m',0):g}m"
        age=time.time()-evaluation.get('updated_unix',0)
        valid=telemetry_valid(manifest,evaluation)
        if valid:
            coverage=evaluation.get('coverage')
            output.extend([f"覆盖率：{'等待观测' if coverage is None else f'{coverage:.2%}'}   决策：{evaluation.get('step',0)}/512   连续零收益：{evaluation.get('zero',0)}",
                f"当前动作：{evaluation.get('phase','—')}   动作耗时：{duration(evaluation.get('action_wall_s',0))}   回合耗时：{duration(evaluation.get('episode_wall_s',0))}"])
        else:output.extend(['覆盖率：等待本回合实时数据','已完成结果已保存；等待当前回合加载或遥测更新。'])
    elif phase in ('training','warmup'):
        transitions=progress.get('transitions',manifest.get('actuals',{}).get('transitions',0))
        updates=progress.get('updates',manifest.get('actuals',{}).get('updates',0))
        budget=1024 if stage==0 else 8192
        current=max(0,min(budget,transitions-(0 if stage==0 else 1024+(stage-1)*8192)))
        output.extend([f'本阶段：{current:,}/{budget:,} ({current/budget:.1%})',
            f'本轮转移：{transitions:,}/82,944   新增更新：{updates:,}/20,480',
            f"速度：{progress.get('transitions_per_s',0):.2f} 条/s   回放经验：{progress.get('replay_count',0):,}",'',
            '环境   类型       尺寸       决策     已知面积(m²)    状态'])
        for key,e in sorted(progress.get('environments',{}).items(),key=lambda kv:int(kv[0])):
            output.append(f"{key:>3}    {e.get('family','—'):<7} {e.get('extent_m',0):>6.1f}m  {e.get('steps',0):>4}/512   {e.get('known_area_m2',0):>9.1f}    {e.get('state','—')}")
    output.extend(coverage_lines(manifest))
    if manifest.get('error'):output.extend(['错误：'+manifest['error']])
    checkpoint=ROOT/'run/resume.pt'
    stamp=datetime.fromtimestamp(checkpoint.stat().st_mtime,ZoneInfo('Asia/Shanghai')).strftime('%m-%d %H:%M:%S') if checkpoint.exists() else '—'
    output.extend(['','─'*70,'最近检查点：'+stamp,
        '曲线：训练统计 / 固定实测状态 Q与策略诊断 / 完整20回合评测',
        f'曲线 CSV+PNG：{ROOT}/curves | 日志：{manifest.get("detail_log","—")}','',
        'q：退出监看（任务继续）   Ctrl+C：请求保存退出任务'])
    return output

def telemetry_valid(manifest, evaluation, now=None):
    age=(time.time() if now is None else now)-evaluation.get('updated_unix',0)
    return (0<=age<15 and evaluation.get('pid')==manifest.get('eval_pid')
            and bool(manifest.get('run_id')) and evaluation.get('run_id')==manifest['run_id']
            and evaluation.get('case_index')==manifest.get('case_index'))


def fit_lines(output,height,width):
    if height<len(output)+1:
        output=output[:max(0,height-2)]+['q退出 | Ctrl+C保存停止']
    return [clip(line,max(0,width-1)) for line in output[:max(0,height-1)]]


def display(screen,root=ROOT):
    global ROOT
    ROOT=Path(root)
    try:curses.curs_set(0)
    except curses.error:pass
    screen.timeout(1000)
    request={'stop':False};notice=''
    original=signal.signal(signal.SIGINT,lambda *_:request.update(stop=True))
    try:
        while True:
            manifest=read(ROOT/'manifest.json',{})
            if request['stop']:
                from controller import request_stop
                try:request_stop(ROOT);notice='已请求保存停止；等待检查点完成。'
                except ValueError as error:notice=str(error)
                request['stop']=False
            output=lines(manifest,latest_progress(),read(ROOT/'eval-progress.json',{}))
            if notice:output.insert(4,notice)
            screen.erase();height,width=screen.getmaxyx()
            for index,line in enumerate(fit_lines(output,height,width)):
                try:screen.addstr(index,0,line)
                except curses.error:pass
            screen.refresh();key=screen.getch()
            if key==3:request['stop']=True
            if key in (ord('q'),ord('Q')):return
    finally:signal.signal(signal.SIGINT,original)
