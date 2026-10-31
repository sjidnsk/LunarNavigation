"""编排边界：确定性 runner 不代替正式异步闭环。"""
import ast
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import CellRect, Route, VehicleState, WheelLimits, WorldSnapshot
from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
from scripts.navigation.one_cm.nav1cm.global_worker import make_request, run_request, restore_result


class Runner:
    def __init__(self):
        self.requests = []
        self.ready = False
        self.pending = None
    @property
    def busy(self):
        return self.pending is not None
    def submit(self, request):
        assert not self.busy
        self.pending = request
        self.requests.append(request)
    def poll(self):
        if not self.ready or not self.busy:
            return None
        request, self.pending = self.pending, None
        self.ready = False
        return run_request(request)


class RejectPlanner:
    def __init__(self): self.calls = 0
    def plan(self, *args, **kwargs):
        self.calls += 1
        return SimpleNamespace(candidate=None, reason='QP_FAILED', timings_ms={}, attempts=())


@pytest.fixture
def setup():
    world = WorldSnapshot(1, (), (CellRect(-2000, -2000, 2000, 2000),))
    runner, planner = Runner(), RejectPlanner()
    coordinator = NavigationCoordinator(planner, runner=runner, local_period_s=10., global_period_s=1.)
    return world, runner, planner, coordinator


def test_first_wait_is_certified_and_nonblocking_and_schedule(setup):
    world, runner, planner, c = setup
    state = VehicleState(0, 0, 0, .1, 0)
    first = c.tick(world, state, (3, 0), 0)
    assert first.certificate.matches(first.candidate, first.grid, c.limits, first.candidate.route_version)
    assert first.candidate.source == 'brake' and first.candidate.states[0, 3] == .1
    assert planner.calls == 0 and len(runner.requests) == 1
    skipped = c.tick(world, state, (3, 0), .01)
    assert skipped.status == 'NOT_DUE' and skipped.candidate is None
    runner.ready = True
    c.tick(world, state, (3, 0), 10)
    assert planner.calls == 1 and len(runner.requests) == 2


def test_map_update_recertifies_and_coalesces_busy_requests(setup):
    world, runner, _, c = setup
    state = VehicleState(0, 0, 0, .1, 0)
    a = c.tick(world, state, (4, 0), 0)
    new = WorldSnapshot(2, (CellRect(250, -20, 270, 20),), world.known_free)
    b = c.tick(new, state, (4, 0), 10)
    assert len(runner.requests) == 1 and b.candidate.map_epoch == 2
    assert not a.certificate.matches(a.candidate, b.grid, c.limits, a.candidate.route_version)
    runner.ready = True
    c.tick(new, state, (4, 0), 20)
    assert runner.requests[-1]['world']['epoch'] == 2


def test_route_replacement_reprojects_and_old_goal_rejected(setup):
    world, runner, _, c = setup
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world, state, (4, 0), 0)
    runner.ready = True
    result = c.tick(world, state, (0, 4), 10)
    assert c.route is None and 'STALE_GLOBAL_RESULT' in [e['kind'] for e in result.events]
    runner.ready = True
    result = c.tick(world, VehicleState(0, 1, 0, 0, 0), (0, 4), 20)
    assert c.route.goal == (0., 4.)
    assert result.reference.progress_s == pytest.approx(1.)


def test_unknown_policy_changes_digest_and_failure_never_changes_world(setup):
    _, runner, _, c = setup
    world = WorldSnapshot(1, (), (CellRect(-80, -80, 80, 80),))
    state = VehicleState(0, 0, 0, 0, 0)
    first = c.tick(world, state, (3, 0), 0)
    c.allow_unknown = True
    runner.ready = True
    second = c.tick(world, state, (3, 0), 10)
    assert first.grid.digest() != second.grid.digest()
    assert world.blocked == () and np.any(second.grid.cells == -1)
    assert 'QP_FAILED' in [e.get('reason') for e in second.events]


def test_goal_later_blocked_brakes_real_motion(setup):
    world, runner, _, c = setup
    state = VehicleState(0, 0, 0, .1, 0)
    c.tick(world, state, (3, 0), 0)
    blocked = WorldSnapshot(2, (CellRect(290, -10, 310, 10),), world.known_free)
    result = c.tick(blocked, state, (3, 0), 10)
    assert result.status == 'GOAL_IN_OBSTACLE'
    assert result.candidate.source == 'brake' and result.candidate.states[0, 3] == .1
    assert result.candidate.states[-1, 3] == pytest.approx(0.)


def test_offline_prototype_keeps_periodic_replanning_compatibility(setup):
    world,runner,_,c=setup;c.local_period_s=.05;c.global_period_s=10.
    c.tick(world,VehicleState(0,0,0,0,0),(3,0),0.)
    runner.ready=True;c.tick(world,VehicleState(0,0,0,0,0),(3,0),.05)
    r=c.tick(world,VehicleState(0,0,0,0,0),(3,0),5.)
    assert 'NO_PROGRESS' not in [e.get('reason') for e in r.events]
    assert len(runner.requests)==1
    c.tick(world,VehicleState(0,0,0,0,0),(3,0),10.1)
    assert runner.requests[-1]['request_reason']=='PERIODIC'


def test_worker_restores_immutable_route(setup):
    world, _, _, c = setup
    result = restore_result(run_request(make_request(world, (0, 0), (3, 0), 7, c.limits, .9)))
    assert result['route'].version == 7 and result['worker_pid'] > 0
    with pytest.raises(ValueError): result['route'].points.setflags(write=True)


def test_dependency_directions():
    root = Path(__file__).resolve().parents[3] / 'scripts/navigation/one_cm/nav1cm'
    blocked = {
        'maps': {'global_planner', 'local_mpcc', 'coordinator'},
        'reference': {'global_planner', 'local_mpcc', 'coordinator'},
        'dynamics': {'global_planner', 'local_mpcc', 'coordinator'},
        'certification': {'global_planner', 'local_mpcc', 'coordinator'},
        'local_mpcc': {'certification', 'coordinator'},
        'global_planner': {'local_mpcc', 'coordinator', 'fixtures'},
        'global_worker': {'local_mpcc'},
    }
    for name, forbidden in blocked.items():
        tree = ast.parse((root / (name + '.py')).read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            imports = [node.module or ''] if isinstance(node, ast.ImportFrom) else [x.name for x in node.names] if isinstance(node, ast.Import) else []
            assert not any(part in forbidden | {'benchmark_navigation', 'fixtures', 'rclpy', 'rospy'} for imp in imports for part in imp.split('.'))


def test_execution_brake_substeps_never_reverse_and_initial_state_checked(setup):
    from scripts.navigation.one_cm.nav1cm.execution import action_prefix, execute_action
    world, _, _, c = setup
    state = VehicleState(0, 0, 0, .003, 0)
    result = c.tick(world, state, (3, 0), 0.)
    assert result.action.times[-1] == pytest.approx(.05)
    after = execute_action(result.action, state)
    assert abs(after.v) < 1e-12 and after.x > 0
    assert result.action_certificate.matches(result.action, result.grid, c.limits, -1)
    with pytest.raises(ValueError, match='INITIAL_STATE'):
        action_prefix(result.candidate, VehicleState(1, 0, 0, .003, 0))


def test_offline_prototype_exact_twenty_hz_and_one_hz_schedule(setup):
    world, runner, _, c = setup
    c.local_period_s = .05
    state = VehicleState(0, 0, 0, 0, 0)
    c.tick(world, state, (3, 0), 0)
    runner.ready = True
    c.tick(world, state, (3, 0), .05)
    assert len(runner.requests) == 1
    assert c.tick(world, state, (3, 0), .06).status == 'NOT_DUE'
    for i in range(2, 20): c.tick(world, state, (3, 0), i*.05)
    assert len(runner.requests) == 1
    c.tick(world, state, (3, 0), 1.)
    assert len(runner.requests) == 2 and runner.requests[-1]['request_reason'] == 'PERIODIC'


def test_stale_candidate_rejected_and_no_safe_brake_is_explicit(setup):
    from scripts.navigation.one_cm.nav1cm.seeds import brake_candidate
    from scripts.navigation.one_cm.nav1cm.contracts import LocalReference
    from scripts.navigation.one_cm.nav1cm.maps import extract_window
    world, runner, _, c = setup
    state = VehicleState(0, 0, 0, 0, 0)
    grid = extract_window(world, state, False)
    stale = brake_candidate(VehicleState(.1, 0, 0, 0, 0), grid, LocalReference(1, 1, [[0,0]], [0.]), c.limits)
    c.planner = SimpleNamespace(plan=lambda *a, **k: SimpleNamespace(candidate=stale, reason='CANDIDATE', timings_ms={}, attempts=()))
    c.tick(world, state, (3, 0), 0)
    runner.ready = True
    result = c.tick(world, state, (3, 0), 10)
    assert 'INITIAL_STATE_MISMATCH' in [e.get('reason') for e in result.events]
    assert result.action.states[0,0] == 0.
    collision = WorldSnapshot(2, (CellRect(-10,-10,10,10),), world.known_free)
    result = c.tick(collision, VehicleState(0,0,0,.1,0), (3,0), 20)
    assert result.status == 'NO_CERTIFIED_ACTION' and result.action is None


def test_sdf_cache_reuses_geometry_and_rebinds_current_grid(setup, monkeypatch):
    import scripts.navigation.one_cm.nav1cm.maps as module
    world, runner, _, c = setup
    original = module.build_distance_field
    calls = []
    def build(grid):
        calls.append(grid.digest())
        return original(grid)
    monkeypatch.setattr(module, 'build_distance_field', build)
    state = VehicleState(0,0,0,0,0)
    c.tick(world,state,(3,0),0)
    runner.ready = True
    c.tick(world,state,(3,0),10)
    c.tick(world,VehicleState(0,0,.1,0,0),(3,0),20)
    assert len(calls) == 1
    c.allow_unknown = True
    c.tick(world,state,(3,0),30)
    c.tick(world,VehicleState(.01,0,0,0,0),(3,0),40)
    c.tick(WorldSnapshot(2,(),world.known_free),state,(3,0),50)
    assert len(calls) == 2 and len(set(calls)) == 2


def test_short_moving_candidate_falls_back_to_certified_brake(setup):
    from scripts.navigation.one_cm.nav1cm.contracts import Trajectory
    world, runner, _, c = setup
    state = VehicleState(0,0,0,.1,0)
    candidate = Trajectory(1,1,[0.],[[0,0,0,.1,0,0]],np.empty((0,3)),'forward','short-test')
    c.planner = SimpleNamespace(plan=lambda *a, **k: SimpleNamespace(candidate=candidate, reason='CANDIDATE', timings_ms={}, attempts=()))
    c.tick(world,state,(3,0),0)
    runner.ready = True
    result = c.tick(world,state,(3,0),10)
    assert result.action.source == 'brake'
    assert 'ACTION_PREFIX_INVALID' in [e.get('reason') for e in result.events]


@pytest.mark.parametrize('elapsed_ms', [49., 49.5, 50., 50.001, 60.])
def test_soft_budget_keeps_independently_certified_mpcc_motion(setup, monkeypatch, elapsed_ms):
    import scripts.navigation.one_cm.nav1cm.coordinator as module
    from scripts.navigation.one_cm.nav1cm.contracts import Trajectory
    from scripts.navigation.one_cm.nav1cm.seeds import rollout
    world, runner, _, c = setup
    state = VehicleState(0,0,0,.1,0)
    c.local_period_s = .05
    c.route = Route(1,1,[[0,0],[3,0]],(3,0))
    c._goal, c._epoch, c._policy = (3.,0.), 1, False
    c._pending_reason, c._next_global = None, 999.
    controls = np.zeros((1,3))
    candidate = Trajectory(1,1,[0.,.2],rollout([0,0,0,.1,0,0],controls),controls,'forward','mpcc:test')
    c.planner = SimpleNamespace(plan=lambda *a, **k: SimpleNamespace(candidate=candidate, reason='CANDIDATE', timings_ms={}, attempts=()))
    readings = iter([0.] + [elapsed_ms/1000]*100)
    monkeypatch.setattr(module, 'perf_counter', lambda: next(readings))
    result = c.tick(world,state,(3,0),0)
    assert result.candidate is candidate
    assert result.action.source == 'mpcc:test' and result.action.states[-1,0] > 0
    assert result.certificate.matches(candidate,result.grid,c.limits,1)
    assert result.action_certificate.matches(result.action,result.grid,c.limits,1)
    assert result.action_certificate.trajectory_digest != result.certificate.trajectory_digest
    assert result.timings_ms['total'] == pytest.approx(elapsed_ms)
    assert result.deadline_miss == (elapsed_ms > 50.)
    assert ('LOCAL_DEADLINE_MISS' in [e['kind'] for e in result.events]) == (elapsed_ms > 50.)
    assert 'MOTION_BUDGET_EXCEEDED' not in [e['kind'] for e in result.events]
    assert 'brake' not in result.timings_ms


def test_benchmark_soft_budget_observations(monkeypatch):
    import importlib
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]/'scripts/navigation/one_cm'))
    benchmark = importlib.import_module('benchmark_navigation')
    action = SimpleNamespace(source='mpcc:test')
    result = SimpleNamespace(deadline_miss=True, action=action)
    assert benchmark.TIMING_POLICY == 'soft_period_certified_action'
    observed = benchmark.timing_observations(result, 60., SimpleNamespace(valid=True))
    assert observed['deadline_miss'] and observed['decision_deadline_miss']
    assert observed['late_certified_motion']
    assert not benchmark.timing_observations(result, 60., SimpleNamespace(valid=False))['late_certified_motion']
    result.action = SimpleNamespace(source='brake')
    assert not benchmark.timing_observations(result, 60., SimpleNamespace(valid=True))['late_certified_motion']
    result.action = action
    assert not benchmark.timing_observations(result, 50., SimpleNamespace(valid=True))['late_certified_motion']


def test_mpcc_horizon_configuration_validated_before_dependencies(tmp_path):
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    for kwargs in ({'n':0},{'dt':0},{'dt':float('nan')},{'n':2.5}):
        with pytest.raises(ValueError): MpccPlanner(WheelLimits(),tmp_path,**kwargs)


def test_shared_build_root_keeps_configuration_artifacts_separate(tmp_path, monkeypatch):
    import sys
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    class Symbol:
        def __getitem__(self, key): return self
        def operation(self, *args): return self
        __add__ = __radd__ = __sub__ = __rsub__ = __mul__ = __rmul__ = operation
        __truediv__ = __rtruediv__ = __pow__ = __neg__ = operation
    symbolic = Symbol()
    fake_ca = SimpleNamespace(SX=SimpleNamespace(sym=lambda *args: symbolic),
        cos=lambda *args: symbolic, sin=lambda *args: symbolic,
        vertcat=lambda *args: symbolic, sumsqr=lambda *args: symbolic)
    captured = []
    def capture(ocp, *, json_file, verbose):
        export = Path(ocp.code_export_directory)
        export.mkdir(parents=True, exist_ok=True)
        Path(json_file).write_text(ocp.model.name, encoding='utf-8')
        captured.append((export, Path(json_file)))
        return SimpleNamespace()
    fake_acados = SimpleNamespace(AcadosModel=SimpleNamespace,
        AcadosOcp=lambda: SimpleNamespace(solver_options=SimpleNamespace(), cost=SimpleNamespace(), constraints=SimpleNamespace()),
        AcadosOcpSolver=capture)
    monkeypatch.setitem(sys.modules, 'casadi', fake_ca)
    monkeypatch.setitem(sys.modules, 'acados_template', fake_acados)
    baseline = MpccPlanner(WheelLimits(), tmp_path)
    first = {p for pair in captured for p in pair}
    originals = {p: p.read_bytes() for p in first if p.is_file()}
    captured.clear()
    fast = MpccPlanner(WheelLimits(linear_speed_mps=.5, angular_speed_rps=.5), tmp_path, n=15, dt=.1)
    second = {p for pair in captured for p in pair}
    assert first.isdisjoint(second), 'shared root must not overwrite another configuration'
    assert all(p.read_bytes() == content for p, content in originals.items())
    assert baseline.build_root == fast.build_root == tmp_path.resolve()
    assert baseline.build_dir == baseline.build_root / baseline.model_fingerprint
    assert fast.build_dir == fast.build_root / fast.model_fingerprint
    assert (baseline.N, baseline.dt) == (20, .2)


@pytest.mark.parametrize('status,expected', [
    ('GOAL_CLEARANCE', 'SAFE_STOP_GOAL_REJECTED'),
    ('GOAL_IN_OBSTACLE', 'SAFE_STOP_GOAL_REJECTED'),
    ('ARRIVED', 'ARRIVED'),
    ('CANDIDATE_UNCERTIFIED', 'ARRIVED'),
])
def test_terminal_goal_rejection_precedes_near_stopped_arrival(monkeypatch, status, expected):
    import importlib
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]/'scripts/navigation/one_cm'))
    benchmark = importlib.import_module('benchmark_navigation')
    assert benchmark.classify_terminal(status, VehicleState(0,0,0,0,0), (.05,0)) == expected


def test_terminal_rejected_moving_goal_never_arrives(monkeypatch):
    import importlib
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3]/'scripts/navigation/one_cm'))
    benchmark = importlib.import_module('benchmark_navigation')
    assert benchmark.classify_terminal('GOAL_CLEARANCE', VehicleState(0,0,0,.1,0), (.05,0)) is None


@pytest.mark.skipif(__import__('os').environ.get('NAV1CM_REAL_SOLVER')!='1', reason='real acados required')
def test_fast_profile_real_horizon_and_certified_action():
    import os
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    from scripts.navigation.one_cm.nav1cm.maps import extract_window
    from scripts.navigation.one_cm.nav1cm.reference import extract_reference
    from scripts.navigation.one_cm.nav1cm.execution import action_prefix
    from scripts.navigation.one_cm.nav1cm.certification import certify
    limits=WheelLimits(linear_speed_mps=.5, angular_speed_rps=.5)
    shared_root=Path(os.environ['NAV1CM_BUILD_DIR']).parent/'mpcc-shared-root-regression'
    baseline=MpccPlanner(WheelLimits(),shared_root)
    preserved={p:p.read_bytes() for p in baseline.build_dir.glob('*.json')}
    planner=MpccPlanner(limits,shared_root,n=20,dt=.1)
    assert baseline.build_dir != planner.build_dir
    assert len(preserved)==3 and all(p.read_bytes()==data for p,data in preserved.items())
    world=WorldSnapshot(1,(),(CellRect(-500,-500,500,500),))
    state=VehicleState(0,0,0,0,0)
    grid=extract_window(world,state,False)
    reference,_=extract_reference(Route(1,1,[[0,0],[2,0]],(2,0)),state,grid)
    default_result=baseline.plan(state,grid,reference)
    assert default_result.candidate is not None
    assert default_result.candidate.times[-1]==pytest.approx(4.)
    assert certify(action_prefix(default_result.candidate,state),grid,WheelLimits(),1).valid
    result=planner.plan(state,grid,reference)
    assert result.candidate is not None
    assert result.candidate.times[-1]==pytest.approx(2.)
    assert np.allclose(np.diff(result.candidate.times),.1)
    assert certify(action_prefix(result.candidate,state),grid,limits,1).valid
    assert np.max(np.abs(result.candidate.states[:,3]))<=.5+1e-9

def test_global_submit_failure_still_returns_certified_brake(setup):
    world,runner,_,c=setup
    def reject(request): raise RuntimeError('global unavailable')
    runner.submit=reject
    r=c.tick(world,VehicleState(0,0,0,.1,0),(3,0),0)
    assert r.action.source=='brake'
    assert r.action_certificate.matches(r.action,r.grid,c.limits,-1)
    assert c._pending_reason is None
    assert c._next_global == c.global_period_s
    assert any(e['kind']=='GLOBAL_SUBMIT_ERROR' for e in r.events)
