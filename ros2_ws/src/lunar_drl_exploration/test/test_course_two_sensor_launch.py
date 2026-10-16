from dataclasses import replace
import shutil
from test_course_two_distance_sensor import PROFILE
from test_ordered_extent_fork import ordered_checkpoint
from test_course_two_trial_dashboard import ui,root,write


def test_prepare_and_fork_bind_profile_without_changing_frozen_source(ordered_checkpoint,tmp_path,monkeypatch):
    from test_course_two_training import completed_source
    from lunar_drl_exploration import course_two_trial as trial
    from lunar_drl_exploration import course_two_control as ctl
    _,source,original=completed_source(ordered_checkpoint,tmp_path)
    old=tmp_path/'old';(old/'frozen').mkdir(parents=True)
    shutil.copyfile(source,old/'frozen/source-resume.pt')
    (old/'frozen/block31.pt').write_bytes(b'actor')
    (old/'frozen/diagnostic_scenes.py').write_text('# preserved')
    monkeypatch.setattr(trial,'OLD_ROOT',old);monkeypatch.setattr(trial,'SOURCE_SHA',ctl.sha(source))
    target=tmp_path/'new'
    trial.prepare(target)
    identity=ctl.read(target/'identity.json')
    assert identity['scene_profile']==PROFILE and identity['sampling']=='scene_episode_v1'
    assert ctl.sha(target/'frozen/source-resume.pt')==ctl.sha(source)
    assert len(ctl.read(target/'cases.json'))==60
    assert all(c['scene_profile']==PROFILE for c in ctl.read(target/'cases.json'))
    controller=trial.Trial(target);config=controller.initialize_arm('C3')
    assert config.course_two_scene_profile==PROFILE and config.environments==12
    assert trial.checkpoint(target/'arms/C3/training/run/resume.pt').record['schedule']==original.record['schedule']
    assert 'distance-sensor-10-30m' in str(trial.DEFAULT_ROOT)


def test_profile_panel_describes_current_ranges_without_legacy_a1_claim(ui,root):
    write(root/'identity.json',dict(runtime='runtime-one',sampling='scene_episode_v1',scene_profile=PROFILE))
    s=ui.Reader(root,lambda _:None).snapshot()
    text='\n'.join(t for t,_ in ui.protocol(s,144)+ui.gate_rows(s,144))
    assert '10–50' in text and '10–30' in text and '360' in text and '20%' in text
    assert '后续开放' not in text and '18/20' not in text
