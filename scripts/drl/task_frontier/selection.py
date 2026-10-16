"""One independently resumable best-new checkpoint; learner always uses run/resume.pt.

selection.json is the commit record. A small write-ahead intent permits recovery
across checkpoint replacement and metadata commit without a second saved replay.
The existing artifact writer bounds old+temporary disk peaks and streams copies.
"""
from dataclasses import replace
from pathlib import Path
import copy
import shutil
from controller import read, write, sha, digest, validate_evaluation, PAIR_KEYS, stage_target, validate_case_set


def score(result):
    rows=result['cases']
    return [sum(row['completed'] for row in rows),
            -sum(not row['completed'] and row['max_zero_gain_run']>=128 and row['zero_gain_two_point_loop_steps']>0 for row in rows),
            sum(row['final_coverage'] for row in rows)]


class StageSelection:
    def __init__(self,root):
        from lunar_drl_exploration.config import training_config_from_record, resume_semantics
        from eval_case import effective_identity
        self.root=Path(root)
        self.path=self.root/'selection.json'
        self.pending=self.root/'selection-pending.json'
        self.best_path=self.root/'best-new.pt'
        self.config=training_config_from_record(read(self.root/'config.json'))
        self.training_semantic=digest(resume_semantics(self.config))
        self.evaluation_identity=effective_identity(replace(self.config,stagnation_window_steps=0))
        self.cases=validate_case_set(read(self.root/'cases.json')['cases'])

    def _summary(self,label):
        return read(self.root/f'{label}-summary.json')

    def _entry(self,result,actor=None):
        entry=dict(label=result['label'],stage=result['stage'],actor_sha=result['actor_sha'],
                   semantic_id=result['semantic_id'],score=score(result),
                   summary_sha=sha(self.root/f"{result['label']}-summary.json"))
        if actor is not None:
            sidecar=read(actor.with_suffix('.json'))
            if sidecar['actor_sha']!=sha(actor) or sidecar['actor_sha']!=result['actor_sha']:
                raise ValueError('selection Actor export hash mismatch')
            entry.update(actor=actor.name,checkpoint_sha=sidecar['checkpoint_sha'],
                         training_semantic_id=self.training_semantic,
                         source_checkpoint='run/resume.pt')
        return entry

    def _validate_result(self,result,baseline):
        legacy=result['label']=='baseline'
        expected=[dict(case,**{key:row[key] for key in PAIR_KEYS}) for case,row in zip(self.cases,baseline['cases'])]
        for case in expected:case['config_digest']=(baseline['cases'][case['case_index']]['config_digest'] if legacy
                                                   else self.evaluation_identity['config_digest'])
        validate_evaluation(result,actor_sha=result['actor_sha'],
            semantic_id=baseline['semantic_id'] if legacy else self.evaluation_identity['semantic_id'],cases=expected)
        score(result)  # Missing actual-area loop metrics are not eligible for ranking.

    def _validate_record(self,record):
        if (record.get('schema')!='task_frontier_selection_v1'
                or record.get('best_new_checkpoint')!='best-new.pt'):raise ValueError('selection schema')
        baseline=self._summary('baseline');self._validate_result(baseline,baseline)
        if record['baseline']!=self._entry(baseline):raise ValueError('selection baseline changed')
        entries=[]
        for index,entry in enumerate(record['completed'],1):
            if entry['label']!=f'stage{index}' or entry['stage']!=index or index>10:
                raise ValueError('selection stage ordering')
            result=self._summary(entry['label']);self._validate_result(result,baseline)
            expected=self._entry(result,self.root/f'stage{index}.pt')
            if entry!=expected:raise ValueError('selection stage/summary/Actor changed')
            entries.append(entry)
        best=max(entries,key=lambda entry:entry['score']) if entries else None
        overall=max([record['baseline'],*entries],key=lambda entry:entry['score'])
        if (record['best_new']!=best or record['best_label']!=overall['label']
                or record['status']!=('finished' if len(entries)==10 else 'unfinished')):
            raise ValueError('selection score or completion mismatch')

    def _validate_checkpoint(self,path,entry):
        import torch
        from lunar_drl_exploration.checkpoint import TrainingState
        if not path.exists() or sha(path)!=entry['checkpoint_sha']:
            raise ValueError('selected checkpoint hash mismatch')
        state=TrainingState(torch.load(path,map_location='cpu',weights_only=False))
        state.validate(self.config)
        if len(state.record['replay'])>self.config.snapshot_max_bytes:
            raise ValueError('selected replay exceeds snapshot budget')
        schedule=state.record['schedule'];transitions,updates=stage_target(entry['stage'])
        if (schedule['transitions'],schedule['updates'],str(schedule['credit']))!=(transitions,updates,'0') or schedule.get('unfinished_reservations',0):
            raise ValueError('selected checkpoint not at exact stage boundary')
        actor=torch.load(self.root/entry['actor'],map_location='cpu',weights_only=True)
        learner=state.record['learner']
        if (actor['version']!=updates or actor['model_config']!=learner['model_config']
                or actor['schema']!=state.record['semantics']['observation_schema']
                or actor['frontier_semantics']!=state.record['semantics']['frontier_semantics']
                or set(actor['state_dict'])!=set(learner['actor'])
                or any(not torch.equal(value,actor['state_dict'][key]) for key,value in learner['actor'].items())):
            raise ValueError('selected checkpoint Actor binding mismatch')
        # The full state is released here, before streaming any checkpoint bytes.

    def _copy_checkpoint(self,entry):
        from lunar_drl_exploration.artifacts import atomic_write
        source=self.root/'run/resume.pt'
        self._validate_checkpoint(source,entry)
        def copy_bytes(stream):
            with source.open('rb') as source_stream:shutil.copyfileobj(source_stream,stream,1024**2)
        atomic_write(self.best_path,copy_bytes,diagnostic=False,
                     local_total_max_bytes=self.config.total_output_max_bytes)
        if sha(self.best_path)!=entry['checkpoint_sha']:raise ValueError('checkpoint changed during preservation')

    def _comparison(self,record):
        summaries=[self._summary(entry['label']) for entry in [record['baseline'],*record['completed']]]
        write(self.root/'comparison.json',dict(status=record['status'],summaries=summaries,
            best_label=record['best_label'],best_new_label=record['best_new']['label'] if record['best_new'] else None,
            selection_sha=sha(self.path),tie_rule='retain earlier',
            scope='fixed development cases; no independent holdout or vehicle claim'))

    def verify(self):
        intent=read(self.pending) if self.pending.exists() else None
        if intent is not None:
            record=intent['selection']
            if intent['previous_sha']!=(sha(self.path) if self.path.exists() else None) and (
                    not self.path.exists() or read(self.path)!=record):
                raise ValueError('selection recovery previous record mismatch')
            self._validate_record(record)
            entry=record['best_new']
            if not self.best_path.exists() or sha(self.best_path)!=entry['checkpoint_sha']:
                self._copy_checkpoint(entry)
            self._validate_checkpoint(self.best_path,entry)
            write(self.path,record)
            self._comparison(record)
            write(self.pending,None)
        if not self.path.exists():
            if self.best_path.exists():raise ValueError('orphan selected checkpoint without selection')
            return None
        record=read(self.path);self._validate_record(record)
        if record['best_new']:self._validate_checkpoint(self.best_path,record['best_new'])
        elif self.best_path.exists():raise ValueError('unexpected selected checkpoint')
        self._comparison(record)
        return record

    def start(self,baseline):
        existing=self.verify()
        self._validate_result(baseline,baseline)
        if existing:return
        record=dict(schema='task_frontier_selection_v1',status='unfinished',baseline=self._entry(baseline),
                    completed=[],best_new=None,best_new_checkpoint='best-new.pt',best_label='baseline')
        write(self.path,record);self._comparison(record)

    def complete(self,result,actor):
        record=self.verify()
        if record is None:raise ValueError('baseline selection missing')
        baseline=self._summary('baseline');self._validate_result(result,baseline)
        stage=result['stage']
        if result['label']!=f'stage{stage}' or actor!=self.root/f'stage{stage}.pt':raise ValueError('selection stage Actor path')
        entry=self._entry(result,actor)
        if stage<=len(record['completed']):
            if entry!=record['completed'][stage-1]:raise ValueError('completed selection changed')
            return
        if stage!=len(record['completed'])+1 or stage>10:raise ValueError('selection skipped stage')
        self._validate_checkpoint(self.root/'run/resume.pt',entry)
        updated=copy.deepcopy(record);updated['completed'].append(entry)
        promote=record['best_new'] is None or entry['score']>record['best_new']['score']
        if promote:updated['best_new']=entry
        updated['best_label']=max([updated['baseline'],*updated['completed']],key=lambda row:row['score'])['label']
        updated['status']='finished' if stage==10 else 'unfinished'
        self._validate_record(updated)
        if promote:
            write(self.pending,dict(previous_sha=sha(self.path),selection=updated))
            self._copy_checkpoint(entry)
        write(self.path,updated)
        self._comparison(updated)
        if promote:write(self.pending,None)
