import copy
import json
from pathlib import Path
import pytest


def inputs(tmp_path):
    source=tmp_path/'source';source.mkdir();actors=tmp_path/'actors';actors.mkdir()
    cases=[]
    for task in ('moon_outside','cave_inside'):
        for i in range(60):
            spec=dict(seed=i,episode_budget=256,sensor=dict(range_m=10,fov_deg=90))
            if task=='cave_inside':spec['mother_graph_id']='mother-'+str(i%3)
            cases.append(dict(case_id=f'validation-{task}-{i:03d}',kind='simple',capability=task,spec=spec))
    manifest=dict(split='validation',active_tasks=['moon_outside','cave_inside'],cases=cases)
    (source/'validation.json').write_text(json.dumps(manifest))
    (source/'config.json').write_text(json.dumps(dict(curriculum_budgets=[512,2048,8192],ordered_budget_rule=dict(min_decisions=256,max_decisions=512))))
    for block in range(1,25):(actors/f'block-{block:02d}.pt').write_bytes(bytes([block]))
    project=Path(__file__).resolve().parents[4]
    return source,actors,manifest,project/'config/drl_checkpoint_validation_512.json',project/'config/drl_mppi_evaluation_navigation.json'


def test_prepare_is_validation_only_and_preserves_scene_identity(tmp_path):
    from lunar_drl_exploration.validation_campaign import prepare
    source,actors,manifest,protocol,navigation=inputs(tmp_path);root=tmp_path/'output'
    original=(source/'validation.json').read_bytes()
    prepare(source,actors,root,protocol,navigation)
    expected=copy.deepcopy(manifest)
    for case in expected['cases']:case['spec']['episode_budget']=512
    assert json.loads((root/'validation.json').read_text())==expected
    assert (source/'validation.json').read_bytes()==original
    assert not (root/'test.json').exists()
    assert json.loads((root/'evaluation-settings.json').read_text())==dict(workers=8,error_policy='count_failure')
    assert json.loads((root/'evaluation-navigation.json').read_text())['controller_backend']=='mppi_path_follower'
    p=json.loads((root/'comparison-protocol.json').read_text())
    assert len(p['visualization_cases'])==4 and len(p['actors'])==24 and p['checkpoint_order'][0]==20
    for actor in actors.glob('*.pt'):assert (root/'actors'/actor.name).read_bytes()==actor.read_bytes()
    with pytest.raises(FileExistsError):prepare(source,actors,root,protocol,navigation)


def test_prepare_rejects_test_manifest_before_creating_output(tmp_path):
    from lunar_drl_exploration.validation_campaign import prepare
    source,actors,manifest,protocol,navigation=inputs(tmp_path);root=tmp_path/'output'
    manifest['split']='test';(source/'validation.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError):prepare(source,actors,root,protocol,navigation)
    assert not root.exists()


def test_unachieved_thresholds_and_zero_cost_are_not_success_selection(tmp_path):
    from lunar_drl_exploration.validation_campaign import successful,finite_summary
    assert not successful(dict(completed=True,collisions=0,error=None,final_coverage=.98))
    assert not successful(dict(completed=True,collisions=None,error=None,final_coverage=.99))
    assert successful(dict(completed=True,collisions=0,error=None,final_coverage=.99))
    assert finite_summary([None,float('nan'),0])==dict(available=1,median=0.,p90=0.,mean=0.)


def test_sealed_campaign_rejects_navigation_and_snapshot_changes(tmp_path,monkeypatch):
    import ament_index_python.packages
    from lunar_drl_exploration import validation_campaign as campaign
    source,actors,manifest,protocol,navigation=inputs(tmp_path);root=tmp_path/'output'
    campaign.prepare(source,actors,root,protocol,navigation)
    stage=tmp_path/'snapshot';stage.mkdir();binary=stage/'native.so';binary.write_bytes(b'unchanged')
    monkeypatch.setattr(ament_index_python.packages,'get_package_prefix',lambda name:str(stage/'install'/name))
    monkeypatch.setattr(campaign,'runtime_digest',lambda:'test-runtime')
    frozen_python=stage/'source/lunar_drl_exploration/validation_campaign.py'
    frozen_python.parent.mkdir(parents=True);frozen_python.write_bytes(Path(campaign.__file__).read_bytes())
    monkeypatch.setattr(campaign,'__file__',str(frozen_python))
    from types import SimpleNamespace
    def imported(name):
        file=stage/('source' if name.startswith('lunar_drl_exploration') else 'install')/name.split('.')[0]/'module.py'
        file.parent.mkdir(parents=True,exist_ok=True)
        if not file.exists():file.write_text('# generated module\n')
        return SimpleNamespace(__file__=str(file))
    monkeypatch.setattr(campaign,'import_module',imported,raising=False)
    campaign.seal(root,stage);campaign.verify(root)
    nav=root/'evaluation-navigation.json';original=nav.read_bytes();settings=json.loads(original)
    settings['mppi_nominal_response_time_s']=.15;nav.write_text(json.dumps(settings))
    with pytest.raises(ValueError,match='input changed'):campaign.verify(root)
    nav.write_bytes(original);binary.write_bytes(b'changed')
    with pytest.raises(ValueError,match='snapshot changed'):campaign.verify(root)
    with pytest.raises(FileExistsError):campaign.seal(root,stage)


def test_sealing_cannot_fingerprint_live_checkout_as_frozen_source(tmp_path,monkeypatch):
    import ament_index_python.packages
    from lunar_drl_exploration import validation_campaign as campaign
    source,actors,manifest,protocol,navigation=inputs(tmp_path);root=tmp_path/'output'
    campaign.prepare(source,actors,root,protocol,navigation)
    stage=tmp_path/'snapshot';stage.mkdir()
    monkeypatch.setattr(ament_index_python.packages,'get_package_prefix',lambda name:str(stage/'install'/name))
    monkeypatch.setattr(campaign,'runtime_digest',lambda:'test-runtime')
    with pytest.raises(ValueError,match='Python'):
        campaign.seal(root,stage)
    assert not (root/'identity.json').exists()


@pytest.mark.parametrize('drift',['prefix','interface_import','controller_import','drl_import'])
def test_resume_checks_actual_imports_and_message_package_resolution(tmp_path,monkeypatch,drift):
    import ament_index_python.packages
    from types import SimpleNamespace
    from lunar_drl_exploration import validation_campaign as campaign
    source,actors,manifest,protocol,navigation=inputs(tmp_path);root=tmp_path/'output'
    campaign.prepare(source,actors,root,protocol,navigation)
    stage=tmp_path/'snapshot';frozen=stage/'source/lunar_drl_exploration/validation_campaign.py'
    frozen.parent.mkdir(parents=True);frozen.write_bytes(Path(campaign.__file__).read_bytes())
    monkeypatch.setattr(campaign,'__file__',str(frozen))
    monkeypatch.setattr(campaign,'runtime_digest',lambda:'test-runtime')
    monkeypatch.setattr(ament_index_python.packages,'get_package_prefix',lambda name:str(stage/'install'/name))
    paths={}
    def imported(name):
        if name not in paths:
            file=stage/('source' if name.startswith('lunar_drl_exploration') else 'install')/name.split('.')[0]/'module.py'
            file.parent.mkdir(parents=True,exist_ok=True);file.touch()
            paths[name]=file
        return SimpleNamespace(__file__=str(paths[name]))
    monkeypatch.setattr(campaign,'import_module',imported,raising=False)
    campaign.seal(root,stage)
    if drift=='prefix':
        monkeypatch.setattr(ament_index_python.packages,'get_package_prefix',lambda name:
                            str(tmp_path/'outside') if name=='lunar_planning_msgs' else str(stage/'install'/name))
    else:
        name={'interface_import':'lunar_planning_msgs.srv._get_policy_map',
              'controller_import':'lunar_pure_wheeled_controller.node',
              'drl_import':'lunar_drl_exploration.course_two_control'}[drift]
        paths[name]=tmp_path/'outside/module.py'
    with pytest.raises(ValueError,match='frozen'):
        campaign.verify(root)
