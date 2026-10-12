#!/usr/bin/env python3
"""Responsive C3 console. Only explicit P/R keys delegate to the controller."""
import argparse
import curses
import math
from pathlib import Path
import time
import unicodedata

from lunar_drl_exploration.course_two_trial import DEFAULT_ROOT, owner, start, stop
from course_two_trial_panel import Reader, name, GROUPS

VIEWS = ('overview', 'workers', 'training', 'evaluation', 'details')
TITLES = ('总览', '并行任务', '训练与采样', '评测与门槛', '诊断与日志')
PHASES = dict(qualification='场景验收', baseline='准备基线评测', evaluation='固定场景评测',
    prefill='经验补采 · 暂不学习', training='采集与学习', training_review_ready='本段已评测 · 待复核')
STATES = dict(running='运行中', paused='已暂停', ready='准备就绪', needs_attention='需要处理',
    interrupted='控制器已退出', missing='未读取到状态', finished='已完成', failed='运行失败')
REASONS = dict(STAGNATION='停滞截断', APPROACH_BUDGET='接近预算到限',
    DECISION_BUDGET='决策预算到限', GOAL_REACHED='到达目标', COLLISION='碰撞',
    NO_PATH='无路径', TIMEOUT='导航超时')


def cells(value):
    return sum(0 if unicodedata.combining(c) else 2 if unicodedata.east_asian_width(c) in ('W', 'F')
        else 1 for c in str(value))


def clean(value):
    return ''.join(c if c.isprintable() else ' ' for c in str(value))


def clip(value, width):
    out, used = '', 0
    for char in clean(value):
        size = cells(char)
        if used+size > width:
            break
        out += char
        used += size
    return out


def pad(value, width):
    text = clip(value, width)
    return text+' '*max(0, width-cells(text))


def wrap(value, width):
    text, rows = clean(value), []
    while text:
        line = clip(text, max(4, width))
        rows.append((line, 'dim'))
        text = text[len(line):]
    return rows or [('', 'dim')]


def num(value, digits=1):
    return f'{value:.{digits}f}' if isinstance(value, (int, float)) and math.isfinite(value) else '—'


def count(value):
    return f'{value:,}' if isinstance(value, int) else '—'


def pct(value, total=None):
    if total is not None:
        value = value/total if total else None
    return f'{value:.1%}' if isinstance(value, (int, float)) and math.isfinite(value) else '—'


def duration(seconds):
    if not isinstance(seconds, (int, float)) or not math.isfinite(seconds):
        return '—'
    seconds = max(0, int(seconds))
    return f'{seconds//3600:02d}:{seconds//60%60:02d}:{seconds%60:02d}'


def gib(value):
    return num(value/1024**3) if isinstance(value, (int, float)) else '—'


def table(values, widths):
    return ' '.join(pad(v, w) for v, w in zip(values, widths))


def section(title, width):
    return [(clip(' '+title+' '+('─'*width), width), 'section')]


def columns(left, right, width):
    if width < 110:
        return left+right
    a = (width-3)//2
    b = width-a-3
    rows = []
    for i in range(max(len(left), len(right))):
        l, ls = left[i] if i < len(left) else ('', 'normal')
        r, rs = right[i] if i < len(right) else ('', 'normal')
        style = 'bad' if 'bad' in (ls, rs) else 'section' if 'section' in (ls, rs) else ls if ls==rs else 'normal'
        rows.append((pad(l, a)+' │ '+pad(r, b), style))
    return rows


def progress_bar(done, total, width=22):
    ratio = min(1., max(0., done/total)) if total else 0.
    n = int(width*ratio)
    return '['+'█'*n+'░'*(width-n)+f'] {done:,}/{total:,}  {pct(done,total)}'


def phase_summary(s):
    phase = s['state'].get('phase')
    if phase == 'qualification':
        q = s['qualification']
        return f"验收 {q['done']}/{q['total']}"
    if phase == 'evaluation':
        e = s['evaluation']
        return f"{e['label']} 评测 {e['done']}/{e['total']}"
    if phase in ('prefill', 'training'):
        return f"累计采集 {count(s['progress'].get('transitions'))}"
    return PHASES.get(phase, phase or '等待启动')


def eta_line(s):
    if s['status'] != 'running' or not s['live']:
        return '预计剩余 — · 当前未运行'
    if s['state'].get('phase') in ('prefill', 'training'):
        p = s['progress']
        if not s['active_training'] or s['progress_age'] is None or s['progress_age'] > 15:
            return '采集预计剩余 — · 等待本轮新指标'
        target = s['state'].get('target_transitions')
        remaining = max(0, target-p.get('transitions', 0)) if target else None
        rate = p.get('transitions_per_s', 0)
        if remaining == 0:
            return '采集预算已到 · 等待更新/保存'
        if rate > 0 and p.get('wall_s', 0) >= 30 and remaining is not None:
            return f'采集预计剩余 ≈ {duration(remaining/rate)} · 本次均速'
        return '采集预计剩余 — · 速度采样中'
    eta = s.get('eta')
    if eta:
        return f"本阶段粗估剩余 ≈ {duration(eta['seconds'])} · 近 {eta['span']:.0f}s 完成 {eta['count']} 项"
    return '本阶段预计剩余 — · 等待至少 30s 完成速率样本'


def resources(s, width):
    r = s['resources']
    gpu = s['progress'].get('gpu_memory') or {}
    return section('资源 / 保存恢复', width)+[
        (f"主机 CPU {pct(r.get('cpu'))} · 1min 负载 {num(r.get('load'))}/{r.get('cpus','—')} 核", 'normal'),
        (f"可用内存 {gib(r.get('available'))}/{gib(r.get('memory'))} GiB · Swap {gib(r.get('swap_used'))} GiB", 'normal'),
        (f"磁盘剩余 {gib(r.get('disk_free'))} GiB · 三池上限 "
            + '/'.join(str(s['limits'].get(k) or '—') for k in ('qualification','evaluation','training')), 'normal'),
        (f"{'训练显存' if s['active_training'] else '最近训练显存'} 已分配/保留 {gib(gpu.get('allocated_bytes'))}/{gib(gpu.get('reserved_bytes'))} GiB"
            if gpu else '训练显存 — · 尚无训练进程指标', 'dim'),
        (f"最近检查点 {duration(s['checkpoint_age'])} 前 · 可按 P 保存停止"
            if s['checkpoint_age'] is not None else '尚未开始 C3 · block31 完整基线已冻结', 'dim')]


def protocol(s, width):
    if s['identity'].get('sampling')=='scene_episode_v1':
        result=section('本轮目标 / 学习安排',width)+[
            ('三类普通场景 · 区外月表 / 区内月表 / 洞穴','normal'),
            ('12 路共享队列 · 空闲进程动态领取新场景','normal'),
            ('每批 64：三类 21/21/22；额外一条随机轮换','normal'),
            ('先均匀选回合、再选经验 · 同回合 ≤4 条','normal'),
            (f"{'追加总量' if s['identity'].get('continuation_blocks') else '首段'} {s['identity'].get('pilot_admissions',2048):,} 条 / {s['identity'].get('pilot_updates',512):,} 次更新",'normal')]
        if s['identity'].get('scene_profile')=='distance_sensor_10_30_v1':
            return result+[
                ('洞内全部属于任务区；新区外月表距区域边界 10–50m','dim'),
                ('量程 10–30m · 视场 60/90/120/180/360° · 每回合固定','accent'),
                ('新回合 20% 保留 10m/90°；80% 混合传感器 · 三类均参与','dim'),
                ('开发集每类：5 个原场景回归 + 15 个传感器组合','dim')]
        return result+[
            ('洞内全部属于任务区；月表区外起点距区域 ≤20m','dim'),
            ('A1：每回合固定 10m / 90° · 传感器扩展后续开放','dim')]
    return section('本轮目标 / 学习安排', width)+[
        ('仅 C3 · 覆盖阶段 + 回合均匀抽样', 'normal'),
        (f"{'追加总量' if s['identity'].get('continuation_blocks') else '首段'} {s['identity'].get('pilot_admissions',2048):,} 条 / {s['identity'].get('pilot_updates',512):,} 次更新", 'normal'),
        ('每批 64：旧课 32 / 覆盖 16 / 接近 8 / 收尾 8', 'normal'),
        ('同回合 ≤4 条 · 收尾含 4 条成功末段样本', 'normal'),
        ('任务范围：整洞纳入任务区 · 无专门离区返回/绕行配额', 'dim')
            if s['identity'].get('task_scope') else ('任务范围：历史 v2', 'dim'),
        ('A1：固定 10m / 90° · 后续传感器课程待开放', 'dim')]


def qualification_groups(s, width):
    q, sides = s['qualification'], []
    for split, title in (('training','训练场景'), ('development','开发场景')):
        rows = section(title+' · 合格/计划 · 在跑 · 待派', width)
        for (group_split, role), g in q['groups'].items():
            if split != group_split:
                continue
            label = '旧课四类' if role == 'old' else name(role)
            rows.append((table((label, f"{g['done']}/{g['total']}", f"在跑 {g['active']}",
                f"待派 {max(0,g['total']-g['done']-g['active'])}"), (12,9,9,9)),
                'good' if g['done'] == g['total'] else 'normal'))
        sides.append(rows)
    return columns(*sides, width)


def worker_rows(s, width, details=False, limit=None):
    phase = s['state'].get('phase')
    out = section('并行任务 · 覆盖为当前场景值；不是成功率', width)
    rows = []
    if phase == 'qualification':
        widths = [4,12,10,16,6,7,8,9,8] if width>=110 else [4,12,6,14,4,6,7,8,8]
        heads = ['槽位','场景','地形','正在执行','尝试','阶段步','覆盖率','耗时','指标龄']
        if width >= 115:
            widths += [10,12]
            heads += ['传感器','区域边长']
        out.append((table(heads,widths), 'dim'))
        for w in s['qualification']['workers']:
            p, r = w['progress'], w['request']
            status = {'reset':'初始化/参考计算','prelude':'前序覆盖','passed':'验收通过',
                'completion_certificate':'验证/前序复演'}.get(p.get('phase'),p.get('phase','等待指标'))
            if not w['alive']:
                status = '进程已退出'
            elif w.get('accepted'):
                status = '结果已合格'
            size = r.get('roi_side_interval_m') or r.get('old_nominal_extent_m')
            sensor = r.get('sensor', {})
            values = [f"E{w['slot']:02}", ('开·' if w['split']=='development' else '训·')+name(r.get('role')),
                {'moon':'月表','cave':'洞穴'}.get(r.get('family'),'—'), status,
                p.get('attempt',0)+1 if p else '—', p.get('decisions','—'), pct(p.get('coverage')),
                duration(w['elapsed']), duration(w['age'])]
            if width >= 115:
                values += [f"{num(sensor.get('range_m'),0)}m/{num(sensor.get('fov_deg'),0)}°",
                    f'{size[0]:g}–{size[1]:g}m' if size else '—']
            style = 'warn' if not w['alive'] or (w['age'] is not None and w['age']>60) else 'normal'
            rows.append((table(values,widths),style))
            if details:
                rows += wrap(f"     PID {w['pid']} · 请求 {r.get('request_id','—')} · "
                    f"当前阶段路程 {num(p.get('distance_m'))}m", width)
                rows += wrap(f"     固定传感器 {num(sensor.get('range_m'),0)}m / {num(sensor.get('fov_deg'),0)}° · "
                    f"区域边长范围 {size or '—'}m · 尝试 {p.get('attempt',0)+1}/{r.get('max_geometry_attempts','—')}",width)
                if w.get('rejections'):
                    rows += [(text,'warn') for text,_ in wrap('     最近一次场景拒绝：'+w['rejections'][-1].get('error','—'),width)]
    elif phase == 'evaluation':
        widths = [4,12,24,9,9,9,9,8] if width>=110 else [4,10,14,8,8,8,8,8]
        out.append((table(['槽位','组别','场景','决策','覆盖率','从零首见','耗时','指标龄'],widths),'dim'))
        for w in s['evaluation']['workers']:
            p, c = w['progress'], w['case']
            spec = c.get('case', c.get('spec', {}))
            budget = spec.get('episode_budget',c.get('budget','—'))
            status = '已首见' if p.get('first_roi') else '—'
            values = [f"E{w['slot']:02}",name(c.get('capability') or c.get('kind')),w['case_id'],
                f"{p.get('decisions','—')}/{budget}",pct(p.get('coverage')),status,duration(w['elapsed']),duration(w['age'])]
            rows.append((table(values,widths),'normal' if w['alive'] else 'warn'))
            if not p:
                label='初始化' if s['identity'].get('sampling')=='scene_episode_v1' else '初始化 / 前序'
                rows.append((f"     E{w['slot']:02} {label}；尚无本次策略指标",'dim'))
            if details:
                rows += wrap(f"     PID {w['pid']} · 场景 {w['case_id']} · Job {w.get('job_id','—')}",width)
                sensor=spec.get('sensor',{})
                if sensor:
                    rows += wrap(f"     整场传感器 {num(sensor.get('range_m'),1)}m / {num(sensor.get('fov_deg'),0)}°"
                        f" · {'原场景回归' if c.get('evaluation_role')=='regression' else '传感器适应'}",width)
    elif phase in ('training','prefill') and s['active_training']:
        widths = [4,14,14,10,9,10,9,9] if width>=110 else [4,12,12,8,8,8,7,7]
        heads = ['槽位','课程任务','状态','决策','覆盖率','新增 m²','奖励','导航倍率']
        if width >= 115:
            widths += [11,12]
            heads += ['传感器','上次导航']
        out.append((table(heads,widths),'dim'))
        for env, p in sorted(s['progress'].get('environments',{}).items(),key=lambda item:int(item[0])):
            status = {'READY':'等待决策','EXECUTING':'执行目标','RESETTING':'准备场景',
                'INITIALIZING':'初始化','CLOSED':'已关闭'}.get(p.get('state'),'等待指标')
            if p.get('terminated'):
                status = '覆盖达标' if p.get('completed') else '未达目标终止'
            if p.get('truncated'):
                status = REASONS.get(p.get('truncation_reason'),'预算截断')
            sensor = p.get('sensor') or {}
            values = [f'E{int(env):02}',name(p.get('curriculum_task')),status,
                f"{p.get('steps',0)}/{p.get('budget','—')}",
                pct(p.get('reference_coverage',p.get('initial_reference_coverage'))),
                num(p.get('new_area_m2')),num(p.get('reward'),2),num(p.get('actual_rtf'))+'×']
            if width >= 115:
                values += [f"{num(sensor.get('range_m'),0)}m/{num(sensor.get('fov_deg'),0)}°",
                    REASONS.get(p.get('reason_code'),p.get('reason_code') or '—')]
            rows.append((table(values,widths),'warn' if p.get('truncated') else 'normal'))
            if details:
                prelude = p.get('prelude') or {}
                detail=f"     场景面积 {num(p.get('reference_area_m2'))}m² · 路程 {num(p.get('distance_m'))}m · 图节点 {p.get('observed_nodes','—')} / 边 {p.get('observed_edges','—')}"
                if prelude:detail+=f" · 前序 {prelude.get('decisions',0)} 步 / {num(prelude.get('wall_s'))}s"
                rows += wrap(detail,width)
                rows += wrap(f"     固定传感器 {num(sensor.get('range_m'),0)}m / {num(sensor.get('fov_deg'),0)}° · "
                    f"上次导航 {REASONS.get(p.get('reason_code'),p.get('reason_code') or '—')}",width)
    if not rows:
        out.append(('当前没有可展示的活动任务；历史结果见训练/评测页。','dim'))
    elif limit is not None and len(rows)>limit:
        out += rows[:limit]+[(f'还有 {len(rows)-limit} 行 · 按 2 查看全部并行任务','dim')]
    else:
        out += rows
    return out


def training_stats(s, width):
    p = s['progress']
    out = section('采集与学习',width)
    if not p:
        return out+[('待开始 · block31 基线评测结束后启动 C3。' if s['identity'].get('sampling')=='scene_episode_v1' else
            '待开始 · 场景验收和 block31 基线评测结束后启动 C3。','dim')]
    mode = '实时' if s['active_training'] else '最近训练快照（非当前活动阶段）'
    out += [(f"{mode} · 指标 {duration(s['progress_age'])} 前 · 本次运行 {duration(p.get('wall_s'))}",
        'normal' if s['active_training'] and (s['progress_age'] or 0)<=15 else 'warn')]
    if s['identity'].get('continuation_blocks'):
        out.append((f"追加训练 · 第 {s['state'].get('block_index',1)}/{s['identity']['continuation_blocks']} 段"
            f" · 每 {s['identity']['block_admissions']:,} 条保存并评测",'accent'))
    total = p.get('transitions',0)
    prefill_schedule = s['prefill'].get('schedule',{})
    phase = s['state'].get('phase')
    if phase == 'prefill' or (p.get('collect_only') and not prefill_schedule):
        amount = total-s['identity'].get('baseline_transitions',total)
        out.append(('补采上限 '+progress_bar(amount,s['identity'].get('prefill_limit',4096)), 'accent'))
        out.append(('满足经验供给后可提前结束；本阶段零优化器更新。','dim'))
    elif prefill_schedule:
        out.append(('本段采集 '+progress_bar(total-prefill_schedule['transitions'],s['identity'].get('pilot_admissions',2048)), 'accent'))
        out.append(('本段更新 '+progress_bar(p.get('updates',0)-prefill_schedule['updates'],s['identity'].get('pilot_updates',512)), 'accent'))
    out += [(f"累计采集 {count(p.get('transitions'))} · 累计更新 {count(p.get('updates'))} · 本次新增 {count(p.get('new_transitions'))}",'normal'),
        (f"本次均速 采集 {num(p.get('transitions_per_s'),2)} 条/s · 更新 {num(p.get('updates_per_s'),2)} 次/s",'normal'),
        (f"保存倒计时 {duration(p.get('save_in_s'))} · 在途决策 {p.get('inflight','—')} · 更新额度 {num(p.get('credit'),2)}",'normal'),
        (f"经验池 {count(p.get('replay_count'))} 条 / {gib(p.get('replay_bytes'))} GiB · 策略版本 {p.get('actor_version','—')}",'normal')]
    return out


def task_table(s, width):
    tasks = s['progress'].get('course',{}).get('by_task',{})
    out = section('课程结果 · 统计有策略动作的已结束回合',width)
    if not tasks:
        return out+[('暂无 C3 采集统计','dim')]
    widths = [15,10,10,13,9,10,10] if width>=110 else [13,9,9,11,7,8,8]
    out.append((table(['任务','接收条数','学习抽样','成功/结束','成功率','零步结束','初始完成'],widths),'dim'))
    for task,r in tasks.items():
        ended = max(0,r.get('ended',0)-r.get('zero_decisions',0))
        success = r.get('successes',0)
        out.append((table([name(task),count(r.get('transitions')),count(r.get('samples')),
            f'{success}/{ended}',pct(success,ended),r.get('zero_decisions',0),r.get('initial_completed',0)],widths),'normal'))
    if s['identity'].get('sampling')=='scene_episode_v1':
        course=s['progress']['course'];slots=course.get('active_slots',{});cost=course.get('action_seconds',{})
        out.append(('动态采集 · 当前进程：'+' / '.join(f'{name(t)} {slots.get(t,0)}' for t in tasks),'accent'))
        out.append(('动作耗时平滑均值：'+' / '.join(f'{name(t)} {num(cost.get(t),1)}s' for t in tasks),'normal'))
        out.append(('累计派发回合：'+' / '.join(f"{name(t)} {course.get('scene_issued',{}).get(t,0)}" for t in tasks),'normal'))
        out.append(('结束回合平均覆盖：'+' / '.join(
            f"{name(t)} {pct(v.get('sum',0),v.get('count',0))} ({v.get('count',0)}场)"
            for t,v in course.get('coverage_stats',{}).items()),'normal'))
        out.append(('新场景数三类均等，采集条数可不同；学习抽样三类各约 1/3。','dim'))
    else:
        old = sum(r.get('transitions',0) for k,r in tasks.items() if not k.startswith('c2_'))
        new = sum(r.get('transitions',0) for k,r in tasks.items() if k.startswith('c2_'))
        out.append((f'实际接收：旧课 {old:,} ({pct(old,old+new)}) / 新课 {new:,} ({pct(new,old+new)}) · 目标各 50%', 'accent'))
    return out+[('成功率分母剔除零决策回合；初始完成单列。进行中的回合不计为失败。','dim')]


def replay_table(s, width):
    b = s['progress'].get('balanced_replay') or {}
    out = section('经验供给 / 实际学习抽样',width)
    simple=s['identity'].get('sampling')=='scene_episode_v1'
    if not b:
        if simple:return out+[('待开始 · 三类各约 1/3，回合均匀抽样。','dim')]
        return out+[('待开始 · 目标旧课/新课各 50%；新课按覆盖阶段分层。','dim')]
    ready = b.get('ready')
    out.append(('经验供给：'+('已就绪' if ready else '未就绪；缺少 '+', '.join(name(k) for k in b.get('missing',[]))),
        'good' if ready else 'warn'))
    widths = [17,12,12,10,10]
    out.append((table(['分层','可用转移','不同回合','目标/批','最近一批'],widths),'dim'))
    batch = b.get('last_batch') or {}
    for key,quota in b.get('phase_quotas',{}).items():
        out.append((table([name(key),b.get('pool_counts',{}).get(key,0),
            b.get('episode_counts',{}).get(key,0),quota,batch.get('phases',{}).get(key,'—')],widths),'normal'))
    if not simple:
        out += [(f"成功收尾 {b.get('successful_finish_episodes',0)} 回合 / {b.get('successful_finish_transitions',0)} 条",'normal'),
            (f"收尾 8 条中，成功末段专用抽样 {batch.get('roles',{}).get('success_finish','—')}/4 条",'normal')]
    out += [(f"最近一批：{batch.get('unique_transitions','—')} 条不重复经验 · {batch.get('episodes','—')} 回合 · "
            f"同回合最多 {batch.get('max_per_episode','—')} 条（上限 4）",'normal'),
        ('所有动作从正常回合初始化开始；不人为准备覆盖进度。' if simple else
            '接收比例与学习抽样比例分别统计；前序移动不算策略训练步。','dim')]
    if b.get('task_scope'):
        out += [(f"旧任务范围的洞穴经验：{b.get('scope_excluded_transitions',0)} 条保留但不参与本轮学习",'dim')]
    return out


def learning_rows(s, width):
    p = s['progress'].get('learning') or {}
    out = section('最近一次学习指标 · 改善需看固定评测',width)
    for label,a,b in [('价值损失','critic1_loss','critic2_loss'),('策略损失 / 温度损失','actor_loss','alpha_loss'),
        ('位置熵 / 目标','position_entropy','position_entropy_target'),
        ('航向熵 / 目标','heading_conditional_entropy','heading_entropy_target'),
        ('位置温度 / 航向温度','alpha_position','alpha_heading')]:
        out.append((f'{label}  {num(p.get(a),4)} / {num(p.get(b),4)}','normal'))
    return out+[(f"位置最大概率 {pct(p.get('position_max_probability'))} · Q 目标均值 {num(p.get('target_mean'),3)}",'normal')]


def evaluation_table(s, width, label=None):
    e = s[label] if label in ('baseline','candidate') else s['evaluation']
    baseline = s['baseline']
    title = 'block31 基线' if e['label']=='baseline' else 'C3 训练后'
    status = '完整结束' if e['complete'] else '部分结果' if e['done'] else '待开始'
    out = section(f"{title}评测 · {status} · {e['done']}/{e['total']}",width)
    widths = [13,12,7,9,7,8,9]
    heads = ['组别','已结束/计划','成功','未完成','错误','初始完成','成功率']
    if width>=110:
        widths += [15]
        heads += ['基线策略成功']
    out.append((table(heads,widths),'dim'))
    for key in GROUPS:
        g = e['groups'][key]
        if not g['total'] and not g['done']:continue
        values = [name(key),f"{g['done']}/{g['total']}",g['success'],g['failed'],g['errors'],g['initial_completed'],
            pct(g['success'],g['policy_total'])]
        if width>=110:
            bg = baseline['groups'][key]
            values += [f"{bg['success']}/{bg['policy_total']}" if bg['done'] else '待开始']
        out.append((table(values,widths),'bad' if g['errors'] else 'normal'))
    simple=s['identity'].get('sampling')=='scene_episode_v1'
    new = e['groups']['moon_outside' if simple else 'new']
    out += [(f"{'区外月表' if simple else '新任务'}从零首见 {new['approached']}/{new['blind']} ({pct(new['approached'],new['blind'])}) · "
        f"剔除初始可见 {new['initial_visible']} / 错误 {new['errors']}",'accent'),
        ('策略成功率分母剔除初始完成和运行错误；从零首见另剔除初始可见。','dim'),
        ('每类固定 20 场；部分结果不代表晋级或退化判定。' if simple else
            '初始化检查与策略新任务分组；部分结果不代表晋级或退化判定。','dim')]
    if s['identity'].get('scene_profile')=='distance_sensor_10_30_v1':
        out+=sensor_evaluation_rows(e['rows'],width)
    if e['failures']:
        out += wrap('未完成原因：'+' · '.join(f'{REASONS.get(k,k)} {v}' for k,v in e['failures'].items()),width)
    return out


def sensor_evaluation_rows(rows,width):
    out=section('传感器分组 · 策略成功 / 有效回合 · 初始完成另列',width)
    groups={key:[] for key in ('10m','20m','30m','60°','90°','120°','180°','360°')}
    for row in rows:
        sensor=row.get('sensor') or {};distance=sensor.get('range_m');fov=sensor.get('fov_deg')
        if isinstance(distance,(float,int)):
            key=f'{distance:g}m'
            if key in groups:groups[key].append(row)
        key=f'{fov:g}°' if isinstance(fov,(float,int)) else ''
        if key in groups:groups[key].append(row)
    for key,group in groups.items():
        errors=sum(bool(r.get('error')) for r in group)
        initial=sum(bool(r.get('initial_completed')) and not r.get('error') for r in group)
        valid=[r for r in group if not r.get('error') and not r.get('initial_completed')]
        good=sum(bool(r.get('completed')) and not r.get('collisions') for r in valid)
        out.append((f'  {key:<10} {good}/{len(valid)} ({pct(good,len(valid))}) · 初始完成 {initial} · 错误 {errors}','normal'))
    return out


def gate_rows(s, width):
    gate = s['gate']
    out = section('阶段门槛 / 当前结论',width)
    if not gate:
        out.append(('追加训练每段保存并评测；完成总预算后待复核。' if s['identity'].get('continuation_blocks') else '待完整 C3 评测后判定；达到首段预算也会先停下复核。','dim'))
    else:
        out.append((f"门槛 {'通过' if gate.get('passed') else '未全部通过'} · "
            f"{'按授权追加预算继续' if s['identity'].get('continuation_blocks') else ('退化检查触发停止' if gate.get('stop') else '保留能力通过')}",'warn' if not gate.get('passed') else 'good'))
        for k,v in gate.get('checks',{}).items():
            label=dict(data='结果无运行错误',initialization='初始完成身份一致',collisions='无碰撞',old_success='旧课成功数',
                old_decisions='旧课决策开销',diagnostics='D 场景保留',approach='从零首见',
                new_complete='新任务完成数',initial_visibility='新任务初始不可见',
                development='三类完成率' if s['identity'].get('sampling')=='scene_episode_v1' else '月表/洞穴覆盖与收尾').get(k,name(k))
            if k.endswith('_decisions') and s['identity'].get('sampling')=='scene_episode_v1':
                label=name(k[:-10])+'决策开销'
            for suffix,description in [('_regression','原场景保留'),('_adaptation','新条件成功率')]:
                if k.endswith(suffix):label=name(k[:-len(suffix)])+description
            for prefix,axis in [('bearing_bin:','方向分组 '),('range_m:','量程分组 '),('fov_deg:','视场分组 ')]:
                if k.startswith(prefix):
                    label=axis+k[len(prefix):]
            out.append((f"  {'通过' if v else '未通过'}  {label}",'good' if v else 'warn'))
    if s['identity'].get('scene_profile')=='distance_sensor_10_30_v1':
        return out+[('固定 60 场：每类 5 原场景 + 15 传感器组合；初始完成单列。','normal'),
            ('各类与新条件有效回合成功率 ≥90%；原场景成功数不下降。','normal'),
            ('退化门槛继续记录；按授权完成追加预算后复核。' if s['identity'].get('continuation_blocks') else '各类比基线少成功两场即停止；成对策略决策比中位数 ≤1.2。','normal')]
    if s['identity'].get('sampling')=='scene_episode_v1':
        return out+[('固定 60 场，每类 20 场；各类成功 ≥18/20，无碰撞、无运行错误。','normal'),
            ('各类比基线少成功两场即停止；成对完成场景决策比中位数 ≤1.2。','normal')]
    return out+[('新任务：从零首见 ≥72/80，完成 ≥64/80；方向/传感器各子组首见 ≥80%。','normal'),
        ('旧课：成功 ≥19/20，成对决策比中位数 ≤1.2；D 场景仅历史诊断。','normal')
            if s['identity'].get('task_scope') else
        ('旧课：成功 ≥19/20，成对决策比中位数 ≤1.2；保留基线已成功的 D 场景。','normal'),
        ('能力组：各 ≥14/16，且不能比基线少成功两项；要求无碰撞、无运行错误。','normal')]


def failure_rows(s, width):
    e=s['candidate'] if s['candidate']['done'] else s['baseline']
    out=section(f"{e['label']} 未完成场景明细",width)
    rows=[r for r in e['rows'] if not r.get('completed') or r.get('collisions') or r.get('error')]
    if not rows:
        return out+[('暂无已记录的未完成场景。','dim')]
    for r in rows:
        reason=r.get('error') or ('碰撞' if r.get('collisions') else REASONS.get(r.get('truncation_reason'),r.get('truncation_reason') or '未达目标'))
        out += wrap(f"{name(r.get('capability') or r.get('kind'))} · {r['case_id']}",width)
        out += wrap(f"  覆盖 {pct(r.get('final_coverage'))} · 决策 {r.get('decisions','—')} · 原因 {reason}",width)
    return out


def body(s, view, width, height=42, log=()):
    phase = s['state'].get('phase')
    if view == 'overview':
        out = [('流程：基线评测 → 补采 → 训练 → 训练后评测 → 复核' if
            s['identity'].get('sampling')=='scene_episode_v1' else
            '流程：验收 → 基线评测 → 补采 → C3 训练 → 训练后评测 → 复核','dim')]
        if phase in ('qualification','evaluation'):
            p = s['qualification'] if phase=='qualification' else s['evaluation']
            out += [(PHASES.get(phase,phase)+' '+progress_bar(p['done'],p['total']), 'accent')]
        out += [(eta_line(s),'accent')]
        compact = width<110 and height<34
        if compact:
            out += [(f"C3 · {'10–30m / 60–360°' if s['identity'].get('scene_profile') else 'A1 10m/90°'} · 12 路上限 · 可用内存 {gib(s['resources'].get('available'))} GiB",'normal')]
        else:
            panel_width = (width-3)//2 if width>=110 else width
            out += columns(protocol(s,panel_width),resources(s,panel_width),width)
        if phase == 'qualification':
            if compact:
                for split,label in (('training','训练场景'),('development','开发场景')):
                    groups=[g for (k,_),g in s['qualification']['groups'].items() if k==split]
                    out.append((f"{label}  合格 {sum(g['done'] for g in groups)}/{sum(g['total'] for g in groups)}"
                        f" · 在跑 {sum(g['active'] for g in groups)}",'normal'))
            else:
                out += qualification_groups(s,width)
        elif phase == 'evaluation':
            out += evaluation_table(s,width)
        elif phase in ('training','prefill'):
            out += training_stats(s,width)
        elif s['gate']:
            out += gate_rows(s,width)
        available = max(3,height-6-len(out)-3)
        out += worker_rows(s,width,limit=available)
        if s['warnings']:
            out.insert(0,(s['warnings'][0],'bad'))
        if s['state'].get('error'):
            out.insert(0,('错误：'+s['state']['error']+' · 按 5 查看详情','bad'))
        return out
    if view == 'workers':
        out = [(eta_line(s),'accent')]
        if phase=='qualification':
            out += qualification_groups(s,width)
            q=s['qualification']
            out += [(f"场景构造/验收历史重试 {q['rejections']} 次；它们不计为策略探索失败。",'dim')]
            out += wrap('阶段步/覆盖为最近一次验收指标；验证后还会复演前序。指标较旧会标黄，进程存活另行核查。',width)
        return out+worker_rows(s,width,details=True)
    if view == 'training':
        return training_stats(s,width)+task_table(s,width)+replay_table(s,width)+learning_rows(s,width)+protocol(s,width)
    if view == 'evaluation':
        return evaluation_table(s,width,'candidate')+evaluation_table(s,width,'baseline')+gate_rows(s,width)+failure_rows(s,width)
    out = resources(s,width)+section('运行证据 / 恢复信息',width)
    for key,value in [('输出目录',s['root']),('检查点',s['checkpoint_path']),
        ('运行时摘要',s['identity'].get('runtime')),('基线模型摘要',s['identity'].get('source_sha')),
        ('控制器状态',s['state'].get('status')),('当前阶段',s['state'].get('phase')),
        ('退出原因',s['exit'].get('stop_reason') or s['state'].get('reason') or '—'),
        ('历史恢复原因',s['state'].get('recovery_reason') or '—')]:
        out += wrap(f'{key}：{value}',width)
    out += section('当前错误 / 证据告警',width)
    warnings = s['warnings']+[s['state']['error']] if s['state'].get('error') else s['warnings']
    if warnings:
        for warning in warnings:
            out += [(text,'bad') for text,_ in wrap(warning,width)]
    else:
        out.append(('当前未记录错误；进程健康不等于策略已经改善。','good'))
    out += section('控制器日志末尾 · 只读',width)
    for line in log or ['暂无日志输出']:
        out += wrap(line,width)
    return out


def frame(s, view='overview', width=144, height=42, scroll=0, message='', log=()):
    width, height = max(1,width-1), max(1,height)
    if width < 36 or height < 10:
        return [(clip(v,width),'warn') for v in ['窗口过小，请放大至至少 80×24',
            phase_summary(s),'Q 关闭面板 · P 保存停止 · R 恢复'][:height]],0,0
    bad = s['status'] in ('interrupted','failed','needs_attention','missing')
    title = '课程二 / C3 训练监看  ·  '+('接近与传感器混合补训'
        if s['identity'].get('scene_profile')=='distance_sensor_10_30_v1' else 'A1')
    clock = time.strftime('%m-%d %H:%M:%S',time.localtime(s['now']))
    first = pad(title,max(0,width-cells(clock)-1))+' '+clock
    header = [(first,'title'),
        (f"{STATES.get(s['status'],s['status'])}  |  {phase_summary(s)}  |  "
            f"PID {s['live']['pid'] if s['live'] else '—'}  |  状态 {duration(s['state_age'])} 前", 'bad' if bad else 'good'),
        ('  '.join(f"{'[' if view==v else ' '}{i+1} {TITLES[i]}{']' if view==v else ' '}" for i,v in enumerate(VIEWS)), 'nav')]
    content = [(part,style) for text,style in body(s,view,width,height,log)
        for part,_ in wrap(text,width)]
    size = max(1,height-6)
    scroll = min(max(0,scroll),max(0,len(content)-size))
    visible = content[scroll:scroll+size]
    visible += [('', 'normal')]*(size-len(visible))
    footer = [('─'*width,'dim'),
        (message or f"{TITLES[VIEWS.index(view)]} · {scroll+1}–{min(len(content),scroll+size)}/{len(content)} 行 · "
            '↑↓/PgUp/PgDn 滚动 · Tab 切页', 'warn' if message else 'dim'),
        ('1–5 切页  Q/Ctrl+C 关面板  P 保存停止  R 恢复', 'nav')]
    return [(clip(text,width),style) for text,style in header+visible+footer],scroll,len(content)


def lines(root, now=None, view='overview', width=144):
    reader = Reader(root,owner)
    s = reader.snapshot(now)
    views = VIEWS if view=='all' else (view,)
    return [text for v in views for text,_ in body(s,v,width,log=reader.log_tail() if v=='details' else ())]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    parser.add_argument('--once',action='store_true')
    parser.add_argument('--view',choices=(*VIEWS,'all'),default='overview')
    parser.add_argument('--width',type=int,default=144)
    args = parser.parse_args()
    reader = Reader(args.root,owner)
    if args.once:
        s=reader.snapshot()
        print(STATES.get(s['status'],s['status'])+' | '+phase_summary(s))
        for v in VIEWS if args.view=='all' else (args.view,):
            print('\n'.join(text for text,_ in body(s,v,args.width,log=reader.log_tail() if v=='details' else ())))
        return

    def show(screen):
        screen.timeout(1000)
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        styles = {'title':curses.A_BOLD,'section':curses.A_BOLD,'accent':curses.A_BOLD,
            'nav':curses.A_REVERSE,'bad':curses.A_BOLD,'dim':curses.A_DIM}
        if curses.has_colors():
            curses.start_color()
            curses.use_default_colors()
            for i,(key,fg) in enumerate([('title',curses.COLOR_CYAN),('section',curses.COLOR_CYAN),
                ('good',curses.COLOR_GREEN),('warn',curses.COLOR_YELLOW),('bad',curses.COLOR_RED),
                ('accent',curses.COLOR_CYAN)],1):
                curses.init_pair(i,fg,-1)
                styles[key]=curses.color_pair(i)|curses.A_BOLD
        view = 'overview' if args.view=='all' else args.view
        scroll, message, deadline = 0, '', 0
        snapshot = reader.snapshot()
        while True:
            height,width = screen.getmaxyx()
            now = time.monotonic()
            if now>=deadline:
                snapshot=reader.snapshot()
                deadline=now+1
            screen.erase()
            rows,scroll,total = frame(snapshot,view,width,height,scroll,message,
                reader.log_tail() if view=='details' else ())
            for y,(text,style) in enumerate(rows):
                try:
                    screen.addstr(y,0,text,styles.get(style,0))
                except curses.error:
                    pass
            screen.refresh()
            key=screen.getch()
            if key in (ord('q'),ord('Q'),3):
                return
            if key in range(ord('1'),ord('6')):
                view=VIEWS[key-ord('1')];scroll=0;message=''
            elif key in (9,curses.KEY_RIGHT,curses.KEY_LEFT,curses.KEY_BTAB):
                step=-1 if key in (curses.KEY_LEFT,curses.KEY_BTAB) else 1
                view=VIEWS[(VIEWS.index(view)+step)%len(VIEWS)];scroll=0;message=''
            elif key in (curses.KEY_DOWN,ord('j')):
                scroll+=1
            elif key in (curses.KEY_UP,ord('k')):
                scroll=max(0,scroll-1)
            elif key in (curses.KEY_NPAGE,curses.KEY_PPAGE):
                scroll+=max(1,height-7)*(1 if key==curses.KEY_NPAGE else -1)
            elif key==curses.KEY_HOME:
                scroll=0
            elif key==curses.KEY_END:
                scroll=total
            try:
                if key in (ord('p'),ord('P')):
                    result=stop(args.root)
                    message='已请求保存并停止，请等待完成。' if result['status']=='save_and_stop_requested' else '控制器当前未运行。'
                    deadline=0
                elif key in (ord('r'),ord('R')):
                    result=start(args.root)
                    message=f"控制器运行中 · PID {result.get('pid','—')}"
                    deadline=0
            except Exception as exc:
                message='操作未执行：'+str(exc)
    try:
        curses.wrapper(show)
    except KeyboardInterrupt:
        pass


if __name__=='__main__':
    main()
