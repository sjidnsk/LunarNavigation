"""Size budgets are frozen before the episode, independent of its outcome."""
import pytest
from test_task_frontier_controller import module

RULE=dict(mode='linear_extent',min_extent_m=50.,max_extent_m=100.,round_to=32,min_decisions=256,max_decisions=384)


def test_size_budget_freezes_five_exact_pairs_and_rule_identity(tmp_path):
    c=module('controller');path=tmp_path/'cases.json'
    rows=c.freeze_cases(c.LEGACY/'artifacts/baseline-summary.json',path,group_indices=[0,2,4,7,9],budget=RULE)
    assert [row['budget'] for row in rows[::2]]==[256,288,320,352,384]
    assert all(rows[i]['budget']==rows[i+1]['budget'] for i in range(0,10,2))
    assert c.read(path)['budget_rule']==RULE
    with pytest.raises(ValueError,match='frozen cases changed'):
        c.freeze_cases(c.LEGACY/'artifacts/baseline-summary.json',path,group_indices=[0,2,4,7,9],budget=dict(RULE,max_decisions=512))


@pytest.mark.parametrize('extent,expected',[(25,256),(50,256),(75,320),(100,384),(150,384)])
def test_size_budget_bounds_rounding_and_legacy_fixed_budget(extent,expected):
    c=module('controller');assert c.decision_budget(extent,RULE)==expected
    assert c.decision_budget(extent,512)==512


@pytest.mark.parametrize('rule',[dict(RULE,mode='area'),dict(RULE,min_extent_m=0),dict(RULE,round_to=0),dict(RULE,max_decisions=128),dict(RULE,min_decisions=True),dict(RULE,max_decisions=380)])
def test_invalid_size_rule_is_rejected(rule):
    with pytest.raises(ValueError):module('controller').decision_budget(50.,rule)


def test_terminal_shows_current_large_map_budget_and_range(tmp_path,monkeypatch):
    c=module('controller');c.write(tmp_path/'cases.json',dict(cases=[dict(case_index=0,family='moon',seed=1,extent_m=50.,budget=256),dict(case_index=1,family='cave',seed=1,extent_m=50.,budget=256),dict(case_index=2,family='moon',seed=2,extent_m=100.,budget=384),dict(case_index=3,family='cave',seed=2,extent_m=100.,budget=384)]))
    d=module('dashboard');monkeypatch.setattr(d,'ROOT',tmp_path);monkeypatch.setattr(d,'telemetry_valid',lambda *_:True)
    text='\n'.join(d.lines(dict(phase='evaluation',case_index=2,eval_budget=384),{},dict(step=300,coverage=.8)))
    assert '评测 256–384' in text and '300/384' in text and '训练 512' in text
