#!/usr/bin/env python3
"""Responsive, read-only presentation; P/R delegate to the owned controller."""
import argparse
from collections import deque
import curses
import math
from pathlib import Path
import textwrap
import time
import unicodedata

from lunar_drl_exploration.course_two_control import (
    DEFAULT_ROOT, read, request_stop, start_background, status,
)
from lunar_drl_exploration.terminal import clip, duration, number
from course_two_evaluation_panel import EvaluationReader

STAGES={'A1':'接近入门','A2':'扩展距离','B':'扩大量程','C':'扩展视场','D':'联合巩固'}
STATES={'running':'运行中','paused':'已保存 · 已暂停','ready':'准备就绪','failed':'运行失败',
    'needs_attention':'等待处理','finished':'课程完成','interrupted_unverified':'进程已退出 · 需核查'}
PHASES={'training':'采集与学习','evaluation':'固定场景评测','saved_exit':'保存完成',
    'sensor_preflight_required':'等待传感器预检','pilot_no_region_gain':'小试未获得区域收益',
    'stage_budget_exhausted':'本段预算已用完','range_budget_exhausted':'量程验收未通过',
    'error':'查看详情中的错误','course_complete':'课程完成','reevaluation_complete':'重评完成 · 等待分析'}


def cells(text):
    return sum(2 if unicodedata.east_asian_width(c) in ('W','F') else 0 if unicodedata.combining(c) else 1 for c in str(text))


def pad(text,width):
    value=clip(str(text),width)
    return value+' '*max(0,width-cells(value))


def table(values,widths):
    return '  '.join(pad(v,w) for v,w in zip(values,widths))


def percent(value):
    return f'{value:.1%}' if isinstance(value,(float,int)) else '—'


def task_name(task):
    return {'moon_inside':'旧课 · 月内','moon_outside':'旧课 · 月外',
        'cave_inside':'旧课 · 洞内','cave_outside':'旧课 · 洞外'}.get(task,task.replace('c2_','新课 '))


def env_state(row):
    if row.get('reason_code')=='COLLISION':return '碰撞终止','bad'
    if row.get('terminated'):return ('覆盖达标','good') if row.get('completed') else ('耗尽未完成','warn')
    if row.get('truncated'):
        reason={'APPROACH_BUDGET':'接近预算到限','STAGNATION':'停滞截断','DECISION_BUDGET':'决策预算到限'}.get(row.get('truncation_reason'),'预算到限')
        return reason,'warn'
    state=row.get('state')
    return {'EXECUTING':('执行目标','good'),'READY':('等待决策','dim'),
        'RESETTING':('准备新场景','warn'),'INITIALIZING':('初始化','warn'),
        'CLOSED':('已关闭','dim')}.get(state,('等待数据','dim'))


def summary(counts):
    total=sum(c.get('ended',0)-c.get('zero_decisions',0) for c in counts)
    successes=sum(c.get('successes',0) for c in counts)
    approaches=sum(c.get('first_roi',0) for c in counts)
    initial=sum(c.get('initial_completed',0) for c in counts)
    rate=f'{successes/total:.0%}' if total else '—'
    return f'策略成功 {successes}/{total} ({rate})   首次区域观测 {approaches}   初始已完成 {initial}'


class TrainingCountdown:
    """A monitor-local estimate of the current collection budget, never a gate."""
    def __init__(self):
        self.points=deque();self.run=None

    def line(self,state,*,width,now=None):
        now=time.time() if now is None else now
        if state['status']!='running' or state['phase']!='training':
            self.points.clear();self.run=None
            if state['status']=='finished':return '课程已结束','good'
            if state['status']=='running' and state['phase']=='evaluation':
                return '本块训练已结束 · 正在评测，评测耗时不计入训练倒计时','dim'
            return '本块预计剩余 — · '+STATES.get(state['status'],state['status']),'dim'
        p=state.get('progress',{});age=p.get('file_age_s',math.inf)
        elapsed=p.get('wall_s');total=p.get('transitions')
        if (not p or not isinstance(age,(int,float)) or not math.isfinite(age) or age>15
                or not isinstance(elapsed,(int,float)) or not math.isfinite(elapsed)
                or not isinstance(total,int) or now-max(0,age)<state.get('updated_unix',0)):
            self.points.clear();self.run=None
            return '本块预计剩余 — · 等待新指标（过期超过15秒时暂停估计）','warn'
        remaining=max(0,state['target_transitions']-total)
        if not remaining:return '本块采集预算已到 · 等待更新、保存或评测','good'
        run=(state['stage'],state['target_transitions'],state.get('child_pid'))
        if run!=self.run or (self.points and (elapsed<self.points[-1][0] or total<self.points[-1][1])):
            self.points.clear();self.run=run
        if not self.points or elapsed>self.points[-1][0]:self.points.append((elapsed,total))
        while len(self.points)>2 and self.points[1][0]<=elapsed-180:self.points.popleft()
        span=elapsed-self.points[0][0]
        rate=p.get('transitions_per_s');source='本次均速'
        if span>=30:
            rate=(total-self.points[0][1])/span;source=f'近{span:.0f}秒均速'
        if elapsed<30 or not isinstance(rate,(int,float)) or not math.isfinite(rate) or rate<=0:
            return '本块预计剩余 — · 速度采样中 / 近期无新增经验','dim'
        # Between fresh snapshots, count down only up to their freshness limit.
        seconds=max(1,math.ceil(remaining/rate-max(0,age)))
        finish=time.strftime('%m-%d %H:%M',time.localtime(now+seconds))
        text=f'本块预计剩余 {duration(seconds)}  预计结束 {finish}（不含评测）'
        if width>=112:text+=f'  {source} {rate:.2f}条/s'
        return text,'title'


def selected_view(state, view='auto'):
    if view != 'auto':return view
    return 'evaluation' if (state.get('reevaluation') or state['phase']=='evaluation' or
        state['phase']=='stage_budget_exhausted' or
        state['status']=='paused' and str(state.get('child_label','')).startswith('eval-') or
        state['status']=='finished' and state.get('evaluation_label')) else 'training'


FAILURES={'APPROACH_BUDGET':'接近预算','DECISION_BUDGET':'决策耗尽',
    'STAGNATION':'停滞截断','COLLISION':'碰撞','ERROR':'运行错误','INCOMPLETE':'其他未完成'}
KINDS={'new':'新任务','old':'旧课回归','diagnostic':'D1–D8','visible':'初始化检查'}
BEARINGS=('东','东北','北','西北','西','西南','南','东南')


def check_name(key):
    names={'complete_records':'记录完整性','no_collisions':'无碰撞','initial_visibility':'初始可见性分类',
        'approach':'首见总率','coverage':'新任务覆盖','old_completion':'旧任务完成率',
        'diagnostic_retention':'D1–D8能力保留','old_decision_cost':'旧任务决策代价'}
    if key.startswith('approach_bearing_bin_'):
        return BEARINGS[int(key.rsplit('_',1)[1])]+'向首见'
    if key.startswith('approach_range_m_'):return key.rsplit('_',1)[1]+'m量程首见'
    if key.startswith('approach_fov_deg_'):return key.rsplit('_',1)[1]+'°视场首见'
    if key.startswith('pair_'):return '传感器组合 '+key[5:]
    return names.get(key,key)


def wrapped(text, width, tone='normal'):
    line=''
    for char in text:
        if line and cells(line+char)>max(1,width-1):
            yield line,tone
            line=''
        line+=char
    if line:yield line,tone


def evaluation_layout(state,identity,config,*,width,details=False):
    panel=state.get('evaluation_panel',{})
    stage=panel.get('stage',state['stage'])
    label=panel.get('label',state.get('evaluation_label','尚无评测'))
    done=panel.get('done',state.get('evaluation_done',0));total=panel.get('total',state.get('evaluation_total',0))
    fraction=done/total if total else 0.;n=22 if width>=100 else 12
    bar='█'*min(n,int(fraction*n))+'░'*max(0,n-int(fraction*n))
    status_text=STATES.get(state['status'],state['status'])
    tone='good' if state['status']=='running' else 'bad' if state['status']=='failed' else 'warn'
    p=state.get('progress',{});checkpoint=state.get('checkpoint',{})
    lines=[(f"课程二   {stage} · {STAGES[stage]}   {status_text}   |   评测{'详情' if details else '概览'}",tone),
        (f"{label}  {bar}  {done}/{total} ({fraction:.1%})   待完成 {max(0,total-done)}",'title'),
        (f"累计训练 {p.get('transitions',identity['baseline_transitions'])-identity['baseline_transitions']:,} 条 / "
         f"{p.get('updates',identity['baseline_updates'])-identity['baseline_updates']:,} 次更新   检查点 {duration(checkpoint.get('age_s',0))} 前",'dim')]
    current=panel.get('current')
    if 'evaluation_workers' in state:
        workers=panel.get('workers',[]);limit=state.get('evaluation_concurrency',1)
        queue=max(0,total-done-len(workers))
        plan=state.get('reevaluation',{});labels=plan.get('labels',[])
        text=f"并行 {len(workers)}/{limit}   排队 {queue}"
        if labels:text+=f"   冻结模型重评 {min(plan.get('next',0)+1,len(labels))}/{len(labels)} · 不更新网络"
        lines.append((text,'title'))
        widths=[4,24,10,12,10,12] if width>=110 else [3,19,8,10,8,9]
        lines.append((table(['槽位','场景','覆盖','决策/预算','已用','指标年龄'],widths),'dim'))
        for worker in workers:
            live=worker.get('live') or {};age=worker.get('age_s')
            lines.append((table([worker['slot'],worker['case_id'],percent(live.get('coverage')),
                f"{live.get('decisions','—')}/{worker.get('budget') or '—'}",duration(worker['elapsed_s']),
                f'{age:.0f}s' if age is not None else '初始化'],widths),'warn' if worker.get('stale') else 'normal'))
        if not workers:lines.append(('暂无运行场景 · 已完成结果保存在下表','dim'))
        if details:
            for worker in workers:
                spec=worker['spec'];sensor=spec.get('sensor') or config.get('sensor',{})
                text=f"槽位{worker['slot']}  {worker['case_id']}  DDS {worker['domain_id']}  PID {worker['pid']}"
                if sensor:text+=f"  固定 {sensor['range_m']:g}m/{sensor['fov_deg']:g}°"
                if 'roi_width_m' in spec:text+=f"  任务区 {spec['roi_width_m']:.1f}×{spec['roi_height_m']:.1f}m"
                lines.extend(wrapped(text,width,'dim'))
    elif current:
        spec=current['spec'];live=current.get('live');family={'moon':'月面','cave':'洞穴'}.get(spec.get('family'),'')
        title='当前场景' if current['active'] else '最近场景'
        lines.append((f"{title}  {current['case_id']}   {KINDS.get(current['kind'],current['kind'])} {family}",'title'))
        sensor=spec.get('sensor') or config.get('sensor',{})
        sensor_text=f"{sensor['range_m']:g}m / {sensor['fov_deg']:g}°" if sensor else '—'
        geometry=(f"   任务区 {spec['roi_width_m']:.1f}×{spec['roi_height_m']:.1f}m"
            if 'roi_width_m' in spec else '')
        bearing=spec.get('bearing_bin');direction='   目标'+BEARINGS[bearing] if bearing in range(8) else ''
        lines.append((f"固定传感器 {sensor_text}{geometry}{direction}",'normal'))
        if live:
            coverage=live.get('coverage')
            lines.append((f"覆盖 {percent(coverage)} / 99%   决策 {live.get('decisions',0)}/{current.get('budget') or '—'}"
                f"   本场已用 {duration(current.get('elapsed_s',0))}",'normal'))
            first=live.get('first_roi')
            first_text=(f"从零首见：第{first['decisions']}次决策 / {number(first.get('distance_m'))}m"
                if first else '从零首见：尚无记录；初始化已可见的场景不产生此记录')
            lines.append((first_text,'dim'))
            age=current.get('age_s',0)
            lines.append((f"本场指标 {age:.0f}s 前更新"+
                (' · 暂无新快照，可能仍在执行动作' if current.get('stale') else ' · 正在逐场评测'),
                'warn' if current.get('stale') else 'dim'))
        else:
            lines.append(('等待本场景的新快照；上一场景数据不作当前进度' if current['active'] else
                '当前没有正在执行的评测场景，以下为已保存结果','dim'))
    else:
        cid=state.get('evaluation',{}).get('case_id','—')
        lines.append((f"场景 {cid} · 等待与本轮身份一致的评测快照",'dim'))
    lines.append(('本轮已结束结果  ·  完整成功要求覆盖≥99%且无碰撞','title'))
    widths=[14,12,8,8,9]
    lines.append((table(['组别','已结束/计划','完整成功','未完成','成功率'],widths),'dim'))
    for kind in ('new','old','diagnostic','visible'):
        counts=panel.get('groups',{}).get(kind)
        if counts is None:continue
        lines.append((table([KINDS[kind],f"{counts['done']}/{counts['total']}",counts['success'],
            counts['failed'],percent(counts['success']/counts['done'] if counts['done'] else None)],widths),
            'warn' if counts['failed'] else 'normal'))
    new=panel.get('groups',{}).get('new',{})
    if new:
        rate=percent(new['approached']/new['blind'] if new['blind'] else None)
        lines.extend(wrapped(f"新任务从零首见 {new['approached']}/{new['blind']} ({rate}) · 已结束且初始不可见；未首见 {new['blind']-new['approached']} 场",width))
        visible=panel.get('groups',{}).get('visible',{})
        if visible.get('total'):
            lines.extend(wrapped(f"分组：新任务 {new['total']} 场 + 初始化检查 {visible['total']} 场；初始化检查不计入新任务成功率和首见率。",width,'dim'))
        lines.extend(wrapped(f"已结束新任务中：初始已可见 {new['initial_visible']} 场；首见分母 {new['blind']} 场（初始不可见且无运行错误）。",width,'dim'))
        if new['initial_visible']:
            lines.extend(wrapped("旧口径提示：新任务内初始已可见场景已从首见率排除，但仍计入上表完整成功率，不能把后者当作盲接近成功率；验收保留原判定。",width,'warn'))
    failures=panel.get('failures',{})
    lines.extend(wrapped('未完成原因  '+('  '.join(f"{FAILURES.get(k,k)} {v}" for k,v in failures.items())
        if failures else '暂无未完成记录'),width,'warn' if failures else 'dim'))
    gate=panel.get('gate')
    verdict=(('通过' if gate['passed'] else '未通过') if gate else '待验收' if panel.get('complete')
        else '场景已齐，等待汇总核对' if done==total and total else '未完成，暂不判定')
    gate_label='本模型验收' if state.get('reevaluation') else '本轮晋级'
    suffix=' · 重评不自动晋级' if state.get('reevaluation') else f"   连续通过 {state.get('streak',0)}/2"
    if label=='baseline':verdict='原模型参照，不作晋级'
    lines.append((f"{gate_label}：{verdict}{suffix}",'good' if gate and gate['passed'] else 'warn'))
    history=panel.get('history',[])
    latest=next((entry for entry in reversed(history) if entry['gate'] and entry['groups']),None)
    if latest:
        failed=[check_name(k) for k,v in latest['gate'].get('checks',{}).items() if not v]
        if failed:
            text=f"{latest['label']} 未通过："+'、'.join(failed if details else failed[:4])
            if not details and len(failed)>4:text+=f" 等{len(failed)}项（Tab详情）"
            lines.extend(wrapped(text,width,'warn'))
    if history:
        lines.append(('完整评测对比  ·  当前未完成轮次不混入','title'))
        widths=[16,12,12,10,10]
        lines.append((table(['模型','新任务成功','旧课成功','D1–D8','晋级'],widths),'dim'))
        for entry in history:
            groups=entry['groups'] or {}
            values=[]
            for kind in ('new','old','diagnostic'):
                group=groups.get(kind)
                values.append(f"{group['success']}/{group['total']}" if group and group['total'] else '—')
            result=('待核对' if entry['groups'] is None else '基线' if entry['gate'] is None
                else '通过' if entry['gate']['passed'] else '未通过')
            lines.append((table([entry['label'],*values,result],widths),'normal'))
    if details:
        lines.append(('本轮未完成场景  ·  覆盖 / 决策 / 最长零收益段','title'))
        for row in panel.get('rows',[]):
            if row.get('completed') and not row.get('error') and not row.get('collisions'):continue
            reason='运行错误' if row.get('error') else '碰撞' if row.get('collisions') else FAILURES.get(row.get('truncation_reason'),'其他未完成')
            lines.extend(wrapped(f"{row['case_id']}  {percent(row.get('final_coverage'))}  {row.get('decisions','—')}次 / "
                f"{row.get('max_zero_gain_run','—')}次零收益  {reason}",width,'warn'))
            if row.get('error'):lines.extend(wrapped(row['error'],width,'bad'))
        lines.extend([('运行与保存','title'),(f"控制器 {state.get('live_pid') or '—'}   评测进程 {state.get('child_pid') or '—'}",'dim')])
        lines.extend(wrapped(str(checkpoint.get('path','—')),width,'dim'))
    for warning in panel.get('warnings',[]):lines.extend(wrapped(warning,width,'warn'))
    if state.get('error'):lines.extend(wrapped(state['error'],width,'bad'))
    return [(clip(text,max(0,width-1)),tone) for text,tone in lines]


def layout(state,identity,config,readiness,*,width,height,details=False,countdown=None,view='auto'):
    """Render state without modifying training; update only the local ETA window."""
    if selected_view(state,view)=='evaluation':
        return evaluation_layout(state,identity,config,width=width,details=details)
    p=state.get('progress',{});saved=state.get('saved_schedule',{}) if state['status']=='paused' else {}
    total=saved.get('transitions',p.get('transitions',identity['baseline_transitions']))
    updates=saved.get('updates',p.get('updates',identity['baseline_updates']))
    done=max(0,total-state['stage_start']);budget=state['target_transitions']-state['stage_start']
    fraction=min(1,done/max(1,budget));bar_width=22 if width>=100 else 12
    bar='█'*int(fraction*bar_width)+'░'*(bar_width-int(fraction*bar_width))
    stage=state['stage'];color='good' if state['status']=='running' else 'bad' if state['status'] in ('failed','interrupted_unverified') else 'warn'
    lines=[(f"课程二   {stage} · {STAGES[stage]}    {STATES.get(state['status'],state['status'])}   |   {'详情' if details else PHASES.get(state['phase'],state['phase'])}",color),
        (f"本段  {bar}  {done:,} / {budget:,}  {fraction:.1%}    {'A1 小试' if not state['pilot_checked'] else '有界训练'}",'title'),
        (f"课程新增 {total-identity['baseline_transitions']:,} 条   更新 {updates-identity['baseline_updates']:,} 次   采集 {number(p.get('transitions_per_s'),2)} 条/s   学习 {number(p.get('updates_per_s'),2)} 次/s",'normal')]
    lines.insert(2,(countdown or TrainingCountdown()).line(state,width=width))
    if state['phase']!='training':
        lines[0]=(lines[0][0]+' · 训练末次快照',lines[0][1])
    checkpoint=state.get('checkpoint',{});age=checkpoint.get('age_s',0)
    resource=p.get('resources',{});gpu=p.get('gpu_memory') or {}
    gpu_gb=gpu['allocated_bytes']/1024**3 if 'allocated_bytes' in gpu else None
    pss_gb=resource['owned_pss_bytes']/1024**3 if 'owned_pss_bytes' in resource else None
    resources=f"  GPU {number(gpu_gb)}G  内存 {number(pss_gb)}G" if width>=108 and p else ''
    lines.append((f"保存于 {duration(age)} 前   下次 {duration(p.get('save_in_s',0))}   指标 {number(p.get('file_age_s'),0)}s 前{resources}",'dim'))
    if details:
        lines.append(('课程统计  ·  策略成功率不含零决策初始化完成','title'))
        widths=[12,8,9,7,12,8,8]
        lines.append((table(['任务','转移','采样占比','首见','成功/结束','成功率','初始完成'],widths),'dim'))
        tasks=p.get('course',{}).get('by_task',{});sample_count=sum(c['samples'] for c in tasks.values())
        for task,c in tasks.items():
            if not c['transitions'] and not c['ended']:continue
            ended=c['ended']-c['zero_decisions']
            lines.append((table([task_name(task),f"{c['transitions']:,}",percent(c['samples']/sample_count if sample_count else None),
                c['first_roi'],f"{c['successes']}/{ended}",percent(c['successes']/ended if ended else None),c['initial_completed']],widths),'normal'))
        lines.extend([('', 'normal'),(f"进程   控制器 {state.get('live_pid') or '—'}   训练 {state.get('child_pid') or '—'}",'dim'),
            (f"回放   {p.get('replay_count',0):,} 条   {p.get('replay_bytes',0)/1024**2:.1f} MiB   待更新额度 {p.get('credit','—')}",'normal'),
            ('完整检查点','title')])
        lines.extend((line,'dim') for line in textwrap.wrap(str(checkpoint.get('path','—')),max(20,width-2)))
        lines.append(('场景身份  ·  每场景传感器固定','title'))
        for env,row in sorted(p.get('environments',{}).items(),key=lambda item:int(item[0])):
            sensor=row.get('sensor',{});scene=row.get('scenario_id')
            spec=f"{sensor.get('range_m','—')}m / {sensor.get('fov_deg','—')}°"
            lines.append((f"E{env}  {spec:<14}  {str(scene)[:24] if scene else '原课程复习'}",'dim'))
        if state.get('error'):lines.extend((line,'bad') for line in textwrap.wrap(state['error'],max(20,width-2)))
    else:
        lines.append(('环境   ·   覆盖率表示任务区域的已覆盖比例','title'))
        wide=width>=112
        widths=[3,11,11,18,9,8,8,14] if wide else [3,9,10,8,8,14]
        headings=['ID','任务','固定传感器','覆盖率','决策','新增m²','路程m','状态'] if wide else ['ID','任务','传感器','覆盖率','决策','状态']
        lines.append((table(headings,widths),'dim'))
        for env in range(config['environments']):
            row=p.get('environments',{}).get(str(env),p.get('environments',{}).get(env,{}))
            family={'moon':'月','cave':'洞'}.get(row.get('family'),'—')
            task=row.get('curriculum_task') or (f'c2_{stage}' if row.get('scenario_id') else '')
            suffix='内' if task.endswith('_inside') else '外' if task.endswith('_outside') else ''
            task_text=(task[3:]+'·'+family) if task.startswith('c2_') else '复习·'+family+suffix
            sensor=row.get('sensor') or row.get('course_two_case',{}).get('sensor',{})
            sensor_text=f"{sensor['range_m']:g}m/{sensor['fov_deg']:g}°" if sensor else '—'
            coverage=row.get('reference_coverage',row.get('initial_reference_coverage'))
            bar2=('█'*min(8,int(coverage*8))+'░'*(8-min(8,int(coverage*8)))+' '+percent(coverage)) if isinstance(coverage,(float,int)) else '—'
            decisions=f"{row.get('steps',0)}/{row.get('budget','—')}"
            text,tone=env_state(row)
            values=[f'E{env}',task_text,sensor_text,bar2,decisions,number(row.get('new_area_m2')),number(row.get('distance_m')),text] if wide else [f'E{env}',task_text,sensor_text,percent(coverage),decisions,text]
            lines.append((table(values,widths),tone))
        tasks=p.get('course',{}).get('by_task',{})
        lines.extend([('', 'normal'),('新任务   '+summary([v for k,v in tasks.items() if k.startswith('c2_')]),'normal'),
            ('旧课复习 '+summary([v for k,v in tasks.items() if not k.startswith('c2_')]),'dim'),
            (f"晋级验收 {state['streak']}/2 连续通过   已评测 {state['stage_block']} 块   全课程采集上限 163,840 条",'dim')])
        if state['phase']=='evaluation':
            e=state.get('evaluation',{})
            lines.append((f"评测 {state.get('evaluation_done',0)}/{state.get('evaluation_total',0)}  {e.get('case_id','—')}  覆盖 {percent(e.get('coverage'))}",'title'))
        elif readiness.get('blocked_stages'):
            lines.append(('大量程阶段暂锁定：40 m 原生观测缓存超限，当前阶段可运行','warn'))
    return [(clip(line,max(0,width-1)),color) for line,color in lines]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    args=parser.parse_args();root=args.root.resolve()
    identity=read(root/'identity.json');config=read(root/'config.json');evaluation=EvaluationReader(root)
    def display(screen):
        screen.timeout(1000);message='退出仅关闭监看，后台继续。E 切换训练/评测，A 恢复自动视图。';details=False;offset=0;view='auto';last_view=None
        countdown=TrainingCountdown()
        palette={k:0 for k in ('normal','good','warn','bad','dim','title')}
        if curses.has_colors():
            curses.start_color();curses.use_default_colors()
            for i,(name,color) in enumerate([('good',curses.COLOR_GREEN),('warn',curses.COLOR_YELLOW),('bad',curses.COLOR_RED),('dim',curses.COLOR_WHITE),('title',curses.COLOR_CYAN)],1):
                curses.init_pair(i,color,-1);palette[name]=curses.color_pair(i)
        try:curses.curs_set(0)
        except curses.error:pass
        while True:
            height,width=screen.getmaxyx()
            try:
                s=status(root);readiness=read(root/'sensor-readiness.json')
                current_view=selected_view(s,view)
                if current_view!=last_view:offset=0;last_view=current_view
                if current_view=='evaluation':s['evaluation_panel']=evaluation.snapshot(s)
                content=layout(s,identity,config,readiness,width=width,height=height,details=details,countdown=countdown,view=view)
            except (OSError,ValueError) as exc:content=[('暂时无法读取状态：'+str(exc),'bad')]
            available=max(1,height-3);offset=min(offset,max(0,len(content)-available))
            screen.erase()
            for i,(text,tone) in enumerate(content[offset:offset+available]):
                try:screen.addstr(i,0,clip(text,max(0,width-1)),palette[tone]|(curses.A_BOLD if tone=='title' else curses.A_DIM if tone=='dim' else 0))
                except curses.error:pass
            footer='P 保存停  R 恢复  Q 退出  Tab '+('概览' if details else '详情')+'  E 视图  A 自动'
            if len(content)>available:footer+='   ↑↓ 滚动'
            for i,text in [(height-2,footer),(height-1,message)]:
                if i>=0:
                    try:screen.addstr(i,0,clip(text,max(0,width-1)),palette['title'] if i==height-2 else palette['dim'])
                    except curses.error:pass
            screen.refresh();key=screen.getch()
            if key in (ord('q'),ord('Q'),3):return
            if key==9:details=not details;offset=0
            elif key in (ord('e'),ord('E')):view='training' if last_view=='evaluation' else 'evaluation';offset=0
            elif key in (ord('a'),ord('A')):view='auto';offset=0
            elif key in (curses.KEY_DOWN,curses.KEY_NPAGE):offset+=1 if key==curses.KEY_DOWN else available
            elif key in (curses.KEY_UP,curses.KEY_PPAGE):offset=max(0,offset-(1 if key==curses.KEY_UP else available))
            elif key in (ord('p'),ord('P'),ord('r'),ord('R')):
                try:
                    if key in (ord('p'),ord('P')):
                        result=request_stop(root);message='正在保存并停止，请等待状态变为“已暂停”。' if result['status']!='not_running' else '训练当前没有运行。'
                    else:
                        result=start_background(root);message='训练已在运行。' if result['status']=='already_running' else '已从完整检查点恢复训练。'
                except Exception as exc:message='操作未执行：'+str(exc)
    try:curses.wrapper(display)
    except KeyboardInterrupt:pass


if __name__=='__main__':main()
