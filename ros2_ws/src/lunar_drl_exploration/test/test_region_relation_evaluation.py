import copy
import pytest
from lunar_drl_exploration.region_relation_evaluation import (
    COUNTS,TASKS,evaluate_manifest,select_validation_candidate,summarize)


def fixture(split='validation'):
    cases=[];rows=[]
    for task in TASKS:
        for index in range(COUNTS[split]):
            case={'case_id':f'{task}-{index}','kind':'simple','capability':task,'spec':{}}
            row={'case_id':case['case_id'],'capability':task,'initial_completed':False,
                 'completed':True,'collisions':0,'error':None,'final_coverage':.99,
                 'distance_m':10.,'decisions':20}
            if task=='cave_inside':
                case['spec']['mother_graph_id']=f'mother-{index%3}'
                row['mother_graph_id']=case['spec']['mother_graph_id']
            cases.append(case);rows.append(row)
    return {'split':split,'cases':cases},rows


def test_full_validation_counts_clusters_and_gate():
    manifest,rows=fixture()
    summary=summarize(manifest,rows)
    assert summary['candidate_gate_passed']
    assert summary['cave_mother_graph_cluster_count']==3
    for row in rows[:7]:row['completed']=False
    assert not summarize(manifest,rows)['candidate_gate_passed']


@pytest.mark.parametrize('split',['quick','test'])
def test_other_splits_report_but_do_not_select(split):
    manifest,rows=fixture(split)
    assert not summarize(manifest,rows)['candidate_gate_passed']
    with pytest.raises(ValueError,match='full validation'):
        select_validation_candidate(manifest,[{'run_id':'one','checkpoint':'a','results':rows}],run_id='one')


@pytest.mark.parametrize('change',['missing','duplicate','error','initial','task','mother','nan'])
def test_rejects_invalid_episodes(change):
    manifest,rows=fixture()
    if change=='missing':rows.pop()
    elif change=='duplicate':rows[-1]=rows[-2].copy()
    elif change=='error':rows[0]['error']='infra failed'
    elif change=='initial':rows[0]['initial_completed']=True
    elif change=='task':rows[0]['capability']='cave_inside'
    elif change=='mother':rows[-1]['mother_graph_id']='foreign'
    elif change=='nan':rows[0]['final_coverage']=float('nan')
    with pytest.raises(ValueError):summarize(manifest,rows)


def test_success_requires_completed_coverage_collision_free():
    manifest,rows=fixture()
    rows[0]['final_coverage']=.989
    rows[1]['collisions']=1
    rows[2]['completed']=False
    assert summarize(manifest,rows)['by_task']['moon_inside']['successes']==57


def test_selection_same_run_and_common_success_efficiency():
    manifest,rows=fixture()
    second=copy.deepcopy(rows)
    for row in second:row['distance_m']=5.
    # Failed case is excluded from efficiency for all candidates.
    rows[0]['completed']=False;second[1]['completed']=False
    candidates=[{'run_id':'one','checkpoint':'a','results':rows},
                {'run_id':'one','checkpoint':'b','results':second}]
    chosen=select_validation_candidate(manifest,candidates,run_id='one')
    assert chosen['checkpoint']=='b'
    assert chosen['common_successful_case_count']==178
    candidates[0]['run_id']='old'
    with pytest.raises(ValueError,match='another training run'):
        select_validation_candidate(manifest,candidates,run_id='one')


def test_single_model_runner_preserves_mother_metadata(tmp_path):
    manifest,rows=fixture('quick')
    calls=[]
    def evaluate(root,request,output,**kwargs):
        import json
        case=json.loads(request.read_text())['case']
        calls.append(case['case_id'])
        row=next(r.copy() for r in rows if r['case_id']==case['case_id'])
        row.pop('mother_graph_id',None)
        return {'row':row}
    results,summary=evaluate_manifest(tmp_path,'actor.pt',manifest,tmp_path/'out',evaluate_case_fn=evaluate)
    assert len(calls)==60
    assert summary['cave_mother_graph_cluster_count']==3
    assert results[-1]['row']['dataset_split']=='quick'
