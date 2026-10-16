import pytest

from lunar_drl_exploration.region_relation_run import (
    make_config, stage_budget, qualification_eligible, validate_run_identity,
)


def test_budget_excludes_prefill_and_resumes_exact_boundary():
    schedule = dict(prefill_transitions=420, transitions=420, updates=0)
    assert stage_budget(schedule, 0) == 1024
    schedule.update(transitions=420 + 1024 + 7000, updates=1750)
    assert stage_budget(schedule, 1) == 1192
    schedule.update(transitions=420 + 1024 + 8192, updates=2048)
    assert stage_budget(schedule, 1) == 0
    with pytest.raises(ValueError):
        stage_budget(schedule, 0)
    assert stage_budget(dict(schedule, transitions=420), 24) == 197632


def test_new_config_has_no_parent_and_frozen_three_scene_protocol(tmp_path):
    config = make_config(tmp_path, 'a' * 64)
    assert config.seed == 20261014
    assert config.model.architecture_version == 'region_relation_v2'
    assert config.model.actor_score_bound == 10
    assert config.learning.maximum_position_alpha == .001
    assert config.learning.microbatch_size == 8
    assert config.curriculum_extents_m[0] == (30, 100)
    assert config.cave_generator_version == 'cave_topology_v2'
    assert config.course_two_sampling == 'scene_episode_v1'
    assert config.warmup == 1024 and config.environments == 8
    assert config.snapshot_max_bytes == config.replay_max_bytes == 8 * 1024**3
    assert config.stagnation_window_steps == 128


def test_qualification_replaces_initial_success_without_hiding_policy_failure():
    good = dict(initial_coverage=.1, initial_completed=False, collision=False,
                actions=8, reference_area_m2=20, inside=True)
    assert qualification_eligible(good, 'cave_inside')
    for patch in (dict(initial_completed=True), dict(initial_coverage=.99),
                  dict(actions=0), dict(reference_area_m2=0), dict(collision=True),
                  dict(inside=False)):
        assert not qualification_eligible(dict(good, **patch), 'cave_inside')
    assert qualification_eligible(dict(good, inside=False), 'moon_outside')


def test_run_identity_rejects_changed_code_manifests_and_config():
    frozen = dict(runtime='code', config_sha='config', manifests={'validation':'val','test':'test'})
    validate_run_identity(frozen, dict(frozen))
    for changed in (dict(frozen, runtime='new'), dict(frozen, config_sha='new'),
                    dict(frozen, manifests={'validation':'new','test':'test'})):
        with pytest.raises(ValueError):
            validate_run_identity(frozen, changed)


@pytest.mark.parametrize('count', [72, 4096])
def test_prefill_ready_checkpoint_recovers_without_collecting_again(tmp_path, monkeypatch, count):
    import lunar_drl_exploration.region_relation_run as run
    (tmp_path/'run').mkdir()
    (tmp_path/'run/resume.pt').touch()
    owner = object.__new__(run.Run)
    owner.root = tmp_path
    saved = {}
    owner.save = lambda **kw: saved.update(kw)
    checkpoint = dict(transitions=count, prefill_transitions=count, updates=0, prefill_ready=True)
    monkeypatch.setattr(run, 'checkpoint_schedule', lambda *a, **kw: checkpoint)
    monkeypatch.setattr(run.subprocess, 'Popen', lambda *a, **kw: pytest.fail('ready prefill restarted'))
    assert owner.train_to(prefill=True) == checkpoint
    assert saved == dict(prefill_admissions=count, prefill_done=True)


def test_physical_map_hash_ignores_labels_but_preserves_geometry():
    import numpy as np
    from types import SimpleNamespace
    from lunar_drl_exploration.region_relation_run import physical_terrain_sha256
    grid = dict(shape=(2, 3), origin=(0., 1.), resolution_m=.2,
                heights=np.array([[0, 1, np.nan], [0, 2, 3]], dtype=np.float32))
    first = SimpleNamespace(**grid, terrain_id='seed1')
    second = SimpleNamespace(**grid, terrain_id='seed2')
    assert physical_terrain_sha256(first) == physical_terrain_sha256(second)
    second.origin = (1., 1.)
    assert physical_terrain_sha256(first) != physical_terrain_sha256(second)


def test_pilot_needs_every_numeric_and_readiness_evidence():
    from lunar_drl_exploration.region_relation_run import pilot_gate
    learning = dict.fromkeys(('critic1_loss','critic2_loss','actor_loss','alpha_loss',
        'actor_grad_norm','q1_grad_norm','q2_grad_norm','temperature_grad_norm',
        'alpha_position','alpha_heading','log_alpha_position','log_alpha_heading'), .1)
    learning.update(numeric_guard_version=1, numeric_checked_updates=4096)
    progress = dict(updates=4096, balanced_replay={'ready':True}, learning=learning)
    readiness = dict(passed=True, checks=dict.fromkeys(
        ('gradient_isolation','checkpoint_restore','cuda_update','ros_closed_loop'), True))
    assert pilot_gate(progress, readiness)
    for key in learning:
        assert not pilot_gate(dict(progress, learning={k:v for k,v in learning.items() if k!=key}), readiness)
    assert not pilot_gate(dict(progress, learning=dict(learning, actor_grad_norm=float('nan'))), readiness)
    assert not pilot_gate(progress, dict(readiness, checks={}))


def test_readiness_is_bound_to_frozen_code_config_and_evidence(tmp_path):
    from lunar_drl_exploration.region_relation_run import validate_readiness
    from lunar_drl_exploration.course_two_control import sha
    evidence = tmp_path/'checks.txt'
    evidence.write_text('passed')
    identity = dict(run_id='run1', runtime='code1', config_sha='config1')
    ready = dict(identity, passed=True, checks=dict.fromkeys(
        ('gradient_isolation','checkpoint_restore','cuda_update','ros_closed_loop'), True),
        evidence={str(evidence):sha(evidence)})
    validate_readiness(ready, identity)
    for key in identity:
        with pytest.raises(ValueError):
            validate_readiness(dict(ready, **{key:'changed'}), identity)
    with pytest.raises(ValueError):
        validate_readiness(dict(ready, evidence={}), identity)
    evidence.write_text('changed')
    with pytest.raises(ValueError):
        validate_readiness(ready, identity)


@pytest.mark.parametrize('failure', ['report', 'save'])
def test_training_reporting_failure_stops_owned_child(tmp_path, monkeypatch, failure):
    import lunar_drl_exploration.region_relation_run as run
    (tmp_path/'logs').mkdir()
    owner = object.__new__(run.Run)
    owner.root = tmp_path
    def save(**kw):
        if failure == 'save' and 'child_pid' in kw:
            raise RuntimeError('save failed')
    owner.save = save
    owner._report_progress = lambda: (_ for _ in ()).throw(RuntimeError('report failed'))
    class Child:
        pid = 123
        stopped = False
        waited = False
        def poll(self): return 0 if self.stopped else None
        def send_signal(self, sig): self.stopped = True
        def wait(self, timeout): self.waited = True; return 0
    child = Child()
    monkeypatch.setattr(run.subprocess, 'Popen', lambda *a, **kw: child)
    monkeypatch.setattr(run.time, 'sleep', lambda _: None)
    with pytest.raises(RuntimeError, match=failure+' failed'):
        owner.train_to(prefill=True)
    assert child.stopped and child.waited
    assert owner.child is None
