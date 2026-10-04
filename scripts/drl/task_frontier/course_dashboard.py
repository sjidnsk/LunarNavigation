"""Per-scene coverage and durable four-task training allocation monitor."""
import curses
from datetime import datetime
from pathlib import Path
import signal
import time
from zoneinfo import ZoneInfo
from controller import read, request_stop
from dashboard import clip, telemetry_valid
from course_controller import TASKS

NAMES=dict(zip(TASKS,('区内月表','区内洞穴','区外月表','区外洞穴')))


def safe_read(path,default):
    try:return read(path)
    except (OSError,ValueError):return default


def latest_progress(root):
    path=Path(root)/'run/metrics.jsonl'
    try:
        with path.open('rb') as stream:
            stream.seek(max(0,path.stat().st_size-131072))
            lines=stream.read().splitlines()
        import json
        for line in reversed(lines):
            try:record=json.loads(line)
            except ValueError:continue
            if record.get('event')=='progress':return record
    except OSError:pass
    return {}


def lines(root,manifest,progress,course,evaluation):
    root=Path(root);phase=manifest.get('course_phase',0);task=TASKS[phase]
    state=safe_read(root/'course-state.json',{});plan=safe_read(root/'course-plan.json',{})
    config=safe_read(root/'config.json',{})
    extents=config.get('curriculum_extents_m',[[50.,100.]])[0]
    extent_label=f'{extents[0]:g}–{extents[1]:g}m'
    targets=manifest.get('targets',{});actuals=manifest.get('actuals',{})
    t=progress.get('transitions',actuals.get('transitions',0));u=progress.get('updates',actuals.get('updates',0))
    training=manifest.get('phase') in ('warmup','training')
    output=['四阶段 SAC 课程 · 逐场景覆盖率',
            f'区内月表 → 区内洞穴 → 区外月表 → 区外洞穴  |  训练{extent_label} / 256–512决策 · 固定评测50–100m',
            '─'*100,
            f"状态 {manifest.get('status','等待')} · 第{phase+1}/4阶段 {NAMES[task]} · {manifest.get('phase','等待')} · 本阶段训练块 {state.get('phase_blocks',0)}/{plan.get('max_blocks',[20,10,20,10])[phase]}",
            f"累计转移 {t:,} · 更新 {u:,} · 本块目标 {targets.get('transitions',0):,}条 / {targets.get('updates',0):,}次 · 连续评测通过 {state.get('streak',0)}/2"]
    if training:
        output += [f"速度 {progress.get('transitions_per_s',0):.2f}条/s · replay {progress.get('replay_count',0):,}条",'',
                   '环境  起点/地形  尺寸    覆盖率                决策    状态']
        for key,e in sorted(progress.get('environments',{}).items(),key=lambda kv:int(kv[0])):
            coverage=e.get('coverage');coverage=e.get('reference_coverage',e.get('coverage_ratio')) if coverage is None else coverage
            fraction=min(1.,max(0.,coverage or 0.));filled=round(fraction*15)
            bar='━'*filled+'─'*(15-filled)
            label=NAMES.get(e.get('curriculum_task'),e.get('family','—'))
            output.append(f"E{key:<2}   {label:<6} {e.get('extent_m',0):5.1f}m  {'—' if coverage is None else f'{coverage:6.2%}'} {bar}  {e.get('steps',0):3}/{e.get('budget',e.get('episode_budget',512)):3}  {e.get('state','—')}")
    elif manifest.get('phase')=='evaluation':
        valid=telemetry_valid(manifest,evaluation)
        output += [f"评测 {manifest.get('eval_completed',0)}/{manifest.get('eval_total',5)} · {manifest.get('family','—')} / {manifest.get('start_mode','—')} · {manifest.get('extent_m',0):.1f}m"]
        if valid:
            coverage=evaluation.get('coverage')
            output += [f"覆盖 {'—' if coverage is None else f'{coverage:.2%}'} · 决策 {evaluation.get('step',0)}/{manifest.get('eval_budget',512)} · 连续零收益 {evaluation.get('zero',0)} · 动作 {evaluation.get('action_wall_s',0):.1f}s"]
        else:output += ['等待本回合遥测']
    course=progress.get('course') if training and progress.get('course') else course
    current_statistics=course.get('phase')==phase
    phase_t=course.get('phase_transitions',{}) if current_statistics else {}
    phase_s=course.get('phase_samples',{}) if current_statistics else {}
    if not current_statistics:output += ['等待本阶段统计；累计回合与转移仍显示已保存值。']
    total_t=sum(phase_t.values());total_s=sum(phase_s.values())
    stats=course.get('by_task',{})
    coverage_stats=course.get('coverage_stats',{})
    def coverage_text(key):
        c=coverage_stats.get(key,{});n=c.get('count',0);recent=c.get('recent',[])
        mean=f"{c.get('sum',0.)/n:.2%}" if n else '—'
        near=f"{sum(recent)/len(recent):.2%}" if recent else '—'
        return mean,near,n,len(recent)
    mean,near,n,recent_n=coverage_text(task)
    missing=max(0,stats.get(task,{}).get('ended',0)-n)
    output += ['',f'当前任务平均最终覆盖率 {mean}（已记录{n}回合） · 最近20均值 {near}（实际{recent_n}回合）']
    if missing:output += [f'历史缺{missing}个最终覆盖率，均值只使用可核对记录；后续结束回合完整累计。']
    output += ['', '任务        结束回合  成功/结束   累计转移   本阶段转移占比  本阶段采样占比  平均步数   平均覆(样本)  近20(样本)']
    for key in TASKS:
        v=stats.get(key,{})
        tf=phase_t.get(key,0)/total_t if total_t else 0.;sf=phase_s.get(key,0)/total_s if total_s else 0.
        ended=v.get('ended',0);mean=v.get('episode_steps',0)/ended if ended else 0.
        cov,near_cov,cov_n,near_n=coverage_text(key)
        if cov_n:cov+=f'({cov_n})'
        if near_n:near_cov+=f'({near_n})'
        output.append(f"{NAMES[key]:<6} {ended:8}  {v.get('successes',0):3}/{ended:<4} {v.get('transitions',0):10,}  {tf:12.1%}  {sf:12.1%}  {mean:8.1f}  {cov:>14}  {near_cov:>14}")
    if current_statistics and course.get('missing_replay_tasks'):output.append('新任务经验尚在收集：'+','.join(course['missing_replay_tasks'])+'；暂按可用类更新，实际比例如上。')
    output += ['结束回合含成功和有效终止/截断；中断与在途回合不计。起点标签在整回合内固定。']
    label=manifest.get('label')
    if label:
        cases=safe_read(root/'cases.json',{}).get('cases',[])
        output += ['',f'本轮已完成评测 · {label}']
        for c in cases:
            row_path=root/f'{label}-case-{c["case_index"]:02}.json'
            value=safe_read(row_path,{})
            if value.get('status')!='finished':continue
            row=value.get('cases',[{}])[0]
            output.append(f"{c['case_index']+1:2} {NAMES.get(c.get('curriculum_task'),'—')} {c['extent_m']:5.1f}m  覆盖 {row.get('final_coverage',0):6.2%}  {'成功' if row.get('completed') else '未完成'}  {row.get('decisions',0)}/{c['budget']}步")
    checkpoint=root/'run/resume.pt'
    stamp=datetime.fromtimestamp(checkpoint.stat().st_mtime,ZoneInfo('Asia/Shanghai')).strftime('%m-%d %H:%M:%S') if checkpoint.exists() else '—'
    output += ['', '─'*100, '最近检查点 '+stamp, f'曲线 {root}/curves','q：退出监看，训练继续   Ctrl+C：请求保存并停止训练']
    if manifest.get('error'):output += ['错误：'+manifest['error']]
    return output


def display(screen,root):
    root=Path(root)
    try:curses.curs_set(0)
    except curses.error:pass
    screen.timeout(1000);request={'stop':False};notice=''
    original=signal.signal(signal.SIGINT,lambda *_:request.update(stop=True))
    try:
        while True:
            if request['stop']:
                try:request_stop(root);notice='已请求保存停止，等待检查点。'
                except ValueError as exc:notice=str(exc)
                request['stop']=False
            output=lines(root,safe_read(root/'manifest.json',{}),latest_progress(root),
                         safe_read(root/'run/course-summary.json',{}),safe_read(root/'eval-progress.json',{}))
            if notice:output.insert(4,notice)
            screen.erase();height,width=screen.getmaxyx()
            if len(output)>=height:output=output[:max(0,height-2)]+['窗口过小，请放大；q退出 / Ctrl+C保存停止']
            for i,line in enumerate(output[:height-1]):
                try:screen.addstr(i,0,clip(line,width-1))
                except curses.error:pass
            screen.refresh();key=screen.getch()
            if key==3:request['stop']=True
            elif key in (ord('q'),ord('Q')):return
    finally:signal.signal(signal.SIGINT,original)
