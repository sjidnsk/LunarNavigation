"""Short paired evaluations retain strict budget, scene and resume identity."""
import pytest
from test_task_frontier_controller import module,paired_fixture,ROOT,local_controller


def shortened(tmp_path):
    c=local_controller(tmp_path);root=tmp_path/'owned';c.prepare(root,ROOT/'config/drl_task_frontier.yaml')
    plan=dict(group_indices=[0,2,4,7,9],budget=256)
    c.write(root/'evaluation-plan.json',plan);(root/'cases.json').unlink()
    cases=c.freeze_cases(c.LEGACY/'artifacts/baseline-summary.json',root/'cases.json',**plan)
    _,baseline=paired_fixture();baseline['cases']=baseline['cases'][:10]
    baseline.update(label='baseline',stage=0)
    for case,row in zip(cases,baseline['cases']):
        row.update(case,decisions=256,max_zero_gain_run=0,zero_gain_two_point_loop_steps=0)
    c.write(root/'baseline-summary.json',baseline)
    return c,root,cases,baseline


def test_five_groups_freeze_exact_original_scenes_and_256_budget(tmp_path):
    c,root,cases,_=shortened(tmp_path)
    assert len(cases)==10 and [x['case_index'] for x in cases]==list(range(10))
    assert [x['seed'] for x in cases[::2]]==[2026100100,2026100102,2026100104,2026100107,2026100109]
    assert cases[2]['extent_m']==61.111111111111114
    assert all(x['budget']==256 for x in cases)
    with pytest.raises(ValueError,match='frozen cases changed'):
        c.freeze_cases(c.LEGACY/'artifacts/baseline-summary.json',root/'cases.json',group_indices=[0,2,4,7,9],budget=512)


@pytest.mark.parametrize('groups,budget',[([0,0],256),([10],256),([],256),([0],0),([True],256),([0],True)])
def test_invalid_short_plan_is_rejected(tmp_path,groups,budget):
    c=local_controller(tmp_path)
    with pytest.raises(ValueError):
        c.freeze_cases(c.LEGACY/'artifacts/baseline-summary.json',tmp_path/'cases.json',group_indices=groups,budget=budget)


def test_short_screen_counts_and_256_result_cannot_reuse_512_decisions(tmp_path):
    c,root,cases,baseline=shortened(tmp_path)
    control=c.Controller(root);control.record(stage=1)
    actor=root/'stage1.pt';actor.write_bytes(b'actor')
    c.write(actor.with_suffix('.json'),dict(actor_sha=c.sha(actor),checkpoint_sha='checkpoint'))
    def spawn(command,label,**kwargs):
        request=c.read(root/'evaluation-request.json');row=dict(baseline['cases'][request['case_index']],actor_sha=c.sha(actor))
        c.write(command[-1],dict(status='finished',exit_code=0,actor_sha=c.sha(actor),semantic_id='semantic',cases=[row]));return True
    control.spawn=spawn;result=control.screen('stage1',actor,baseline)
    assert len(result['cases'])==10
    m=c.read(root/'manifest.json');assert (m['eval_completed'],m['eval_groups'],m['eval_total'],m['eval_budget'])==(10,5,10,256)
    broken=dict(result,cases=[dict(x) for x in result['cases']]);broken['cases'][0]['decisions']=512
    with pytest.raises(ValueError,match='budget'):c.validate_evaluation(broken,actor_sha=c.sha(actor),semantic_id='semantic',cases=[dict(case,**{k:row[k] for k in c.PAIR_KEYS},config_digest='config') for case,row in zip(cases,result['cases'])])


def test_short_selection_and_curves_accept_only_complete_frozen_set(tmp_path):
    c,root,cases,baseline=shortened(tmp_path)
    selection=module('selection').StageSelection(root);selection.start(baseline)
    assert selection.verify()['best_label']=='baseline'
    curves=module('curves');recorder=curves.CurveRecorder(root);recorder.sample()
    assert len(recorder.data['evaluations'])==10
    c.write(root/'baseline-summary.json',dict(baseline,cases=baseline['cases'][:-1]))
    with pytest.raises(ValueError):selection.verify()


def test_terminal_uses_case_count_and_eval_budget_not_training_budget(tmp_path,monkeypatch):
    c,root,cases,baseline=shortened(tmp_path);d=module('dashboard');monkeypatch.setattr(d,'ROOT',root)
    m=dict(phase='evaluation',stage=1,eval_total=10,eval_budget=256,case_index=0)
    text='\n'.join(d.lines(m,{},{}))
    assert '0/10' in text and '/5' in text and '评测 256' in text and '训练 512' in text


from test_frontier_final_fixes import selection_fixture


def test_short_stage_promotion_and_resume_preserve_checkpoint_binding(selection_fixture):
    c,s,root,baseline,stage,config=selection_fixture
    cases=c.read(root/'cases.json')['cases'][:10]
    for case in cases:case['budget']=256
    c.write(root/'cases.json',dict(cases=cases))
    baseline['cases']=baseline['cases'][:10]
    for row in baseline['cases']:row.update(budget=256,decisions=256)
    c.write(root/'baseline-summary.json',baseline)
    selected=s.StageSelection(root);selected.start(baseline)
    result,actor=stage(1,.8);result['cases']=result['cases'][:10]
    for row in result['cases']:row.update(budget=256,decisions=256)
    c.write(root/'stage1-summary.json',result);selected.complete(result,actor)
    checkpoint_sha=c.sha(root/'best-new.pt')
    resumed=s.StageSelection(root).verify()
    assert resumed['completed'][0]['stage']==1
    assert resumed['best_new']['checkpoint_sha']==checkpoint_sha
    assert resumed['best_new']['score'][-1]==pytest.approx(8.)
