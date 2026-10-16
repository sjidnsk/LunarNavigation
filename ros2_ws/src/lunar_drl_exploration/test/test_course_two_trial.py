from copy import deepcopy


def rows():
    result=[]
    for kind,count in [('old',20),('diagnostic',8),('new',80),('visible',18)]:
        for i in range(count):result.append(dict(case_id=f'{kind}-{i}',kind=kind,completed=True,
            collisions=0,error=None,decisions=10,initial_roi_visible=kind=='visible',first_roi={'decisions':2}))
    for capability in ('cover','finish','return','detour'):
        for i in range(16):result.append(dict(case_id=f'{capability}-{i}',kind='capability',capability=capability,
            completed=True,collisions=0,error=None,decisions=10,initial_roi_visible=True,first_roi=None))
    return result


def test_regression_stops_arm_before_additional_training():
    from lunar_drl_exploration.course_two_trial import gate
    baseline=rows();candidate=deepcopy(baseline)
    assert gate(candidate,baseline)['passed']
    candidate[0]['completed']=candidate[1]['completed']=False
    assert gate(candidate,baseline)['stop']
    candidate=deepcopy(baseline);candidate[20]['completed']=False
    assert gate(candidate,baseline)['stop']
    candidate=deepcopy(baseline)
    for r in candidate[:20]:r['decisions']=13
    assert gate(candidate,baseline)['stop']
    candidate=deepcopy(baseline)
    for r in candidate[-2:]:r['completed']=False
    assert gate(candidate,baseline)['stop']


def test_incomplete_or_mismatched_evidence_never_passes():
    import pytest
    from lunar_drl_exploration.course_two_trial import gate
    with pytest.raises(ValueError,match='identity'):gate(rows()[:-1],rows())
    candidate=rows();candidate[-1]['case_id']='changed'
    with pytest.raises(ValueError,match='identity'):gate(candidate,rows())


def test_failing_promotion_does_not_extend_pilot_budget():
    from lunar_drl_exploration.course_two_trial import gate
    baseline=rows();candidate=deepcopy(baseline)
    for r in candidate[28:48]:r['completed']=False;r['first_roi']=None
    result=gate(candidate,baseline)
    assert not result['passed']
    assert result['next_action']=='end_block_and_review'


def test_single_c3_run_has_no_control_qualification_jobs():
    from lunar_drl_exploration.course_two_trial import ARMS, qualification_jobs
    assert ARMS == ('C3',)
    train = [dict(role='old' if i < 64 else 'cover', request_id=f'train-{i}') for i in range(128)]
    development = [dict(role='finish', request_id=f'dev-{i}') for i in range(64)]
    jobs = qualification_jobs(train, development)
    assert len(jobs) == 192
    assert all(not original for _, _, original in jobs)
    assert {label for label, _, _ in jobs} == {'rebuilt', 'development'}


def test_explicit_resource_resize_reuses_only_exact_qualified_records(tmp_path):
    import pytest
    from lunar_drl_exploration import course_two_control as ctl
    from lunar_drl_exploration.course_two_trial import cached_qualification
    path=tmp_path/'qualified.json'
    proof=dict(runtime='before',config_sha='config',request=dict(seed=7),original=False)
    result=dict(status='passed',proof=proof,evidence=dict(coverage=.4))
    ctl.write(path,result)
    expected=dict(proof,runtime='after')
    with pytest.raises(ValueError,match='qualification'):
        cached_qualification(path,expected,{})
    identity=dict(qualification_cache_adoptions={path.name:dict(
        sha=ctl.sha(path),from_runtime='before',to_runtime='after')})
    before=path.read_bytes()
    assert cached_qualification(path,expected,identity)==result
    assert path.read_bytes()==before
    for changed in (dict(expected,config_sha='changed'),dict(expected,request=dict(seed=8))):
        with pytest.raises(ValueError,match='qualification'):
            cached_qualification(path,changed,identity)
    result['evidence']['coverage']=.5;ctl.write(path,result)
    with pytest.raises(ValueError,match='qualification'):
        cached_qualification(path,expected,identity)
