"""纯证据闭环验收；不导入ROS、策略网络或训练模块。"""
import math


def _number(value, *, positive=False):
    return (type(value) in (int,float) and math.isfinite(value)
            and (not positive or value>0))


def _goal(value):
    return isinstance(value,(list,tuple)) and len(value)==3 and all(_number(v) for v in value)


def _identity(event):
    serial,session=event.get('serial'),event.get('session_id')
    if type(serial) is not int or serial<1 or not isinstance(session,str) or not session:
        return None
    return serial,session


def _stopped(event):
    velocity=event.get('velocity')
    return (isinstance(velocity,(list,tuple)) and len(velocity)==2
            and all(_number(v) and abs(v)<=.01 for v in velocity)
            and _number(event.get('pose_age_s')) and 0<=event['pose_age_s']<=1.)


def grade(events,summary,minimum_completed=2):
    """以有序decision/goal/accepted/finish/cancel/reset事件判断真实推理链。

    completed_goals按(serial,UUID)去重且要求不同目标；重复帧不能凑数。
    返回计数和明确失败原因，不通过异常输入推断成功。
    """
    errors=[]
    if type(minimum_completed) is not int or minimum_completed<2:
        return dict(ok=False,errors=['invalid minimum_completed'],completed_goals=0,decisions=0)
    if not isinstance(events,(list,tuple)) or not isinstance(summary,dict):
        return dict(ok=False,errors=['invalid evidence container'],completed_goals=0,decisions=0)
    decisions=[];goals={};accepted={};seen=set();success=[];cancels={};canceled=[];resets=[]
    for position,event in enumerate(events):
        if not isinstance(event,dict):errors.append('invalid event');continue
        kind=event.get('kind')
        identity=_identity(event)
        if kind=='decision':
            if (type(event.get('index')) is not int or event['index']<1
                    or not _number(event.get('stamp_ns'),positive=True)
                    or not _number(event.get('revision'),positive=True)
                    or not _goal(event.get('selected_goal'))):
                errors.append('invalid decision evidence');continue
            if any(e['index']==event['index'] for _,e in decisions):
                errors.append('duplicate decision index');continue
            decisions.append((position,event))
        elif kind=='goal':
            serial=event.get('serial')
            if type(serial) is not int or serial<1 or not _goal(event.get('goal')):
                errors.append('invalid goal evidence');continue
            if not decisions or tuple(decisions[-1][1]['selected_goal'])!=tuple(event['goal']):
                errors.append('goal does not match the latest actor decision')
            if serial in goals:errors.append('goal serial reused')
            goals[serial]=tuple(event['goal'])
        elif kind=='accepted':
            if identity is None or not _goal(event.get('goal')) or goals.get(identity[0])!=tuple(event['goal']):
                errors.append('invalid accepted goal identity');continue
            if any(key[0]==identity[0] and key!=identity for key in accepted):
                errors.append('serial accepted by multiple sessions');continue
            accepted[identity]=(position,tuple(event['goal']))
        elif kind=='finish':
            if identity in seen:continue
            if (identity is None or identity not in accepted or event.get('accepted') is not True
                    or not _goal(event.get('goal')) or accepted[identity][1]!=tuple(event['goal'])):
                errors.append('finish lacks current accepted identity');continue
            seen.add(identity)
            if not _number(event.get('stamp_ns'),positive=True) or not _number(event.get('revision'),positive=True):
                errors.append('finish lacks observation identity');continue
            outcome,reason=event.get('outcome'),event.get('reason_code')
            if type(outcome) is not int:errors.append('invalid outcome');continue
            if outcome==0 and reason=='GOAL_REACHED':
                good=(_stopped(event) and _number(event.get('position_error_m'))
                    and 0<=event['position_error_m']<=.1 and _number(event.get('yaw_error_rad'))
                    and abs(event['yaw_error_rad'])<=.05 and event.get('action_status',4)==4)
                if good:success.append((position,identity,event))
                else:errors.append('arrival violates measured pose/stop/status contract')
            elif outcome==5 and reason=='CANCELED_STOPPED':
                if identity not in cancels or not _stopped(event) or event.get('action_status',5)!=5:
                    errors.append('cancel lacks matching request or measured stop')
                else:canceled.append((position,identity,event))
            else:errors.append('navigation did not reach or cancel cleanly')
        elif kind=='cancel':
            if identity is None or identity not in accepted:errors.append('cancel has stale identity')
            else:cancels[identity]=(position,event)
        elif kind=='reset':
            if (identity is None or identity not in accepted or event.get('accepted') is not True or not event.get('task_id')
                    or not _number(event.get('stamp_ns'),positive=True)):
                errors.append('invalid reset evidence')
            else:resets.append((position,identity,event))
        elif kind=='process':
            command=event.get('command')
            if (not isinstance(event.get('name'),str) or not event['name'].strip()
                    or type(event.get('pid')) is not int or event['pid']<1
                    or not isinstance(command,list) or not command
                    or not all(isinstance(part,str) and part for part in command)):
                errors.append('invalid process metadata')
        elif kind=='task_published':
            if (type(event.get('command')) is not int or event['command'] not in (1,4)
                    or not isinstance(event.get('task_id'),str) or not event['task_id'].strip()):
                errors.append('invalid task publication metadata')
        else:errors.append('unknown event kind')
    distinct=[];targets=set()
    for row in success:
        target=tuple(row[2]['goal'])
        if target not in targets:targets.add(target);distinct.append(row)
    if len(distinct)<minimum_completed:errors.append('insufficient distinct natural arrivals')
    if len(decisions)<minimum_completed+1:errors.append('insufficient real actor decisions')
    for last_position,_,last in distinct[:minimum_completed]:
        next_row=next(((position,event) for position,event in decisions if position>last_position),None)
        if (next_row is None or next_row[1]['stamp_ns']<=last['stamp_ns']
                or any(last_position<cpos<next_row[0] for cpos,_ in cancels.values())):
            errors.append('no fresh decision after completed feedback')
    if len(decisions)<2 or max((e['revision'] for _,e in decisions),default=0)<=decisions[0][1]['revision']:
        errors.append('observed map revision did not advance')
    if not canceled:errors.append('no current-session cancel completion')
    if not any(rpos>cpos and rid[0]>cid[0] and rid[1]!=cid[1]
            and r['stamp_ns']>c['stamp_ns'] for rpos,rid,r in resets for cpos,cid,c in canceled):
        errors.append('no fresh accepted task reset after cancellation')
    if not any(cpos>rpos and cid==rid for rpos,rid,r in resets for cpos,cid,c in canceled):
        errors.append('reset task did not complete its own current-session cancellation')
    expected={'frozen_unchanged':True,'guard_attempts':[],'stopped':True,'inflight':False,
              'state':'CANCELED','forced_exit':False,'workers_remaining':[]}
    for key,value in expected.items():
        if key not in summary or type(summary[key]) is not type(value) or summary[key]!=value:
            errors.append('summary '+key+' violates closure contract')
    expected_processes=summary.get('guard_expected_processes')
    startups=summary.get('guard_startups')
    expected_ids=set();startup_ids=set()
    if not isinstance(expected_processes,list) or len(expected_processes)<4:
        errors.append('guard expected process inventory missing or incomplete')
    else:
        for process in expected_processes:
            if (not isinstance(process,dict) or type(process.get('pid')) is not int or process['pid']<1
                    or not isinstance(process.get('starttime'),str) or not process['starttime'].strip()):
                errors.append('invalid guard expected process identity')
            else:expected_ids.add((process['pid'],process['starttime']))
        if len(expected_ids)!=len(expected_processes) or len(expected_ids)<4:
            errors.append('guard expected process inventory has duplicates or is incomplete')
    if not isinstance(startups,list):
        errors.append('guard startup evidence missing')
    else:
        for startup in startups:
            if (not isinstance(startup,dict) or startup.get('kind')!='startup'
                    or type(startup.get('pid')) is not int or startup['pid']<1
                    or not isinstance(startup.get('detail'),dict)
                    or not isinstance(startup['detail'].get('starttime'),str)
                    or not startup['detail']['starttime'].strip()):
                errors.append('invalid guard startup identity')
            else:startup_ids.add((startup['pid'],startup['detail']['starttime']))
    if not expected_ids.issubset(startup_ids):
        errors.append('guard hook did not cover every expected live process identity')
    exits=summary.get('child_exits')
    if not isinstance(exits,dict) or len(exits)<4 or any(type(v) is not int or v!=0 for v in exits.values()):
        errors.append('child process exit evidence incomplete or failed')
    return dict(ok=not errors,errors=list(dict.fromkeys(errors)),completed_goals=len(distinct),decisions=len(decisions))
