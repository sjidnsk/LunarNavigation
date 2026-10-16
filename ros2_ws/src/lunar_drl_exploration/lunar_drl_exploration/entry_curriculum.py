"""Frozen, learner-owned near-to-far start curriculum; never a policy input."""
from copy import deepcopy
import math

MODE='near_to_far_v1'
SCHEMA='moon_entry_curriculum_v1'
STAGES=((2.,5.,7,12,2.,5.,8192),(5.,15.,5,32,3.,10.,32768))
WINDOW=50
REQUIRED=40

def semantics():
    return dict(mode=MODE,stages=[dict(distance_m=[s[0],s[1]],guided_fraction=s[2]/10,
        entry_decisions=s[3],entry_distance_multiplier=s[4],entry_distance_slack_m=s[5],
        maximum_moon_admissions=s[6]) for s in STAGES],ordinary_distance_m=[10.,50.],
        ordinary_guided_fraction=0.,window=WINDOW,required_efficient_entries=REQUIRED,
        metric='first_actual_robot_entry_v1',promotion='mastery_or_explicit_admission_cap_v1',
        count_scope='moon_admissions_including_prefill_and_warmup',entry_does_not_terminate=True)

class EntryCurriculum:
    def __init__(self,mode,state=None):
        if mode!=MODE:raise ValueError('unsupported entry curriculum')
        self.stage=0;self.admissions=0;self.promotions=[]
        self.stats={str(i):dict(count=0,entered=0,efficient=0,recent=[]) for i in range(2)}
        if state is not None:
            if (not isinstance(state,dict) or set(state)!= {'schema','mode','stage','admissions','stats','promotions'}
                    or state['schema']!=SCHEMA or state['mode']!=MODE):raise ValueError('entry curriculum identity changed')
            if type(state['stage']) is not int or not 0<=state['stage']<=2 or type(state['admissions']) is not int or state['admissions']<0:
                raise ValueError('invalid entry curriculum counters')
            stats=deepcopy(state['stats']);promotions=deepcopy(state['promotions'])
            if not isinstance(stats,dict) or set(stats)!= {'0','1'}:raise ValueError('invalid entry stage statistics')
            for row in stats.values():
                if not isinstance(row,dict) or set(row)!= {'count','entered','efficient','recent'}:raise ValueError('invalid entry statistics')
                if any(type(row[k]) is not int for k in ('count','entered','efficient')) or not 0<=row['efficient']<=row['entered']<=row['count']:
                    raise ValueError('invalid entry counts')
                recent=row['recent']
                if not isinstance(recent,list) or len(recent)!=min(row['count'],WINDOW):raise ValueError('invalid entry window')
                for v in recent:
                    if (not isinstance(v,dict) or set(v)!= {'entered','efficient'} or any(type(x) is not bool for x in v.values()) or v['efficient'] and not v['entered']):raise ValueError('invalid entry result')
                if sum(v['entered'] for v in recent)>row['entered'] or sum(v['efficient'] for v in recent)>row['efficient']:
                    raise ValueError('entry window exceeds totals')
                if row['count']<=WINDOW and (sum(v['entered'] for v in recent)!=row['entered'] or sum(v['efficient'] for v in recent)!=row['efficient']):raise ValueError('entry window differs from totals')
            if not isinstance(promotions,list) or len(promotions)!=state['stage']:raise ValueError('invalid entry promotion history')
            previous=0
            for i,p in enumerate(promotions):
                if (not isinstance(p,dict) or set(p)!= {'from_stage','to_stage','reason','moon_admissions'} or p['from_stage']!=i or p['to_stage']!=i+1 or p['reason'] not in ('mastery','admission_cap') or type(p['moon_admissions']) is not int or not previous<=p['moon_admissions']<=state['admissions']):raise ValueError('invalid entry promotion')
                if p['reason']=='admission_cap' and p['moon_admissions']<STAGES[i][6]:raise ValueError('premature entry cap promotion')
                if p['reason']=='mastery' and (stats[str(i)]['count']<WINDOW or sum(v['efficient'] for v in stats[str(i)]['recent'])<REQUIRED):raise ValueError('entry mastery not supported by window')
                previous=p['moon_admissions']
            self.stage=state['stage'];self.admissions=state['admissions'];self.stats=stats;self.promotions=promotions

    def advance(self,admissions):
        if type(admissions) is not int or admissions<self.admissions:raise ValueError('entry admissions moved backwards')
        self.admissions=admissions
        while self.stage<2:
            row=self.stats[str(self.stage)]
            mastered=row['count']>=WINDOW and sum(v['efficient'] for v in row['recent'])>=REQUIRED
            capped=admissions>=STAGES[self.stage][6]
            if not (mastered or capped):break
            self.promotions.append(dict(from_stage=self.stage,to_stage=self.stage+1,
                reason='mastery' if mastered else 'admission_cap',moon_admissions=admissions))
            self.stage+=1

    def apply(self,spec,ordinal):
        result=dict(spec)
        if spec.get('curriculum_task')=='moon_outside' and self.stage<2 and ordinal%10<STAGES[self.stage][2]:
            result.update(outside_distance_min_m=STAGES[self.stage][0],outside_distance_max_m=STAGES[self.stage][1])
        return result

    def observe(self,stage,steps,start_distance,first_entry,collision):
        if stage!=self.stage or self.stage==2 or type(steps) is not int or steps<=0:return
        entered=first_entry is not None
        efficient=False
        if entered:
            d,n,path=start_distance,first_entry.get('decisions'),first_entry.get('distance_m')
            if (isinstance(d,bool) or not isinstance(d,(int,float)) or not math.isfinite(d) or d<=0
                    or type(n) is not int or not 1<=n<=steps or isinstance(path,bool) or not isinstance(path,(int,float)) or not math.isfinite(path) or path<0):raise ValueError('invalid actual entry metric')
            s=STAGES[self.stage];efficient=not collision and n<=s[3] and path<=s[4]*d+s[5]
        row=self.stats[str(self.stage)];row['count']+=1;row['entered']+=int(entered);row['efficient']+=int(efficient)
        row['recent'].append(dict(entered=entered,efficient=efficient));del row['recent'][:-WINDOW]
        self.advance(self.admissions)

    def state_dict(self):
        return dict(schema=SCHEMA,mode=MODE,stage=self.stage,admissions=self.admissions,
            stats=deepcopy(self.stats),promotions=deepcopy(self.promotions))
