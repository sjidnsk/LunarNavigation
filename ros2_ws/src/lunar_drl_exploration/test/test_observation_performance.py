"""Cache reuse must preserve measured graph inputs and current-map readiness."""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from lunar_drl_exploration.contracts import Pose,SensorSpec
from lunar_drl_exploration.decision import DecisionCore
from lunar_drl_exploration.ros_messages import MeasurementBuffer
from test_task_analysis import snapshot,task
from test_adapter_readiness import adapter,Client,reply
from test_runtime import Adapter


def test_repeated_ack_skips_scan_but_new_and_older_measurements_are_processed():
    b=MeasurementBuffer((0,0),1)
    hits=SimpleNamespace(cols=[1],rows=[2],heights=[3.],stats=[[0.,0.,0.,1.]])
    b.add(hits,20);b.acknowledge(10);retained=b._cells
    b.acknowledge(10)
    assert b._cells is retained
    b.add(hits,30);b.acknowledge(20)
    assert b._cells[(1,2)][0]==30
    b.add(hits,5);b.acknowledge(20)
    assert not b._cells


def test_sensor_driven_requests_reuse_current_map_and_wait_for_new_stamp(adapter):
    a=adapter;c=Client();a.client=c;s=Adapter().snapshot
    assert a.poll_snapshot(refresh=False) is None
    c.future.response=reply(s)
    assert a.poll_snapshot(refresh=False) is not None
    for _ in range(30):assert a.poll_snapshot(refresh=False) is not None
    assert len(c.requests)==1
    assert a.poll_snapshot(2_000_000_000,refresh=False) is None
    assert len(c.requests)==2 and c.requests[-1].minimum_map_stamp_ns==2_000_000_000
    response=reply(replace(s,revision=s.revision+1));response.processed_stamp.sec=2
    c.future.response=response
    assert a.poll_snapshot(2_000_000_000,refresh=False) is not None
    a.require_fresh_snapshot()
    assert a.poll_snapshot(refresh=False) is None
    assert len(c.requests)==3


def test_unchanged_map_reuses_connections_and_gain_but_updates_anchor(monkeypatch):
    import lunar_drl_exploration.graph as graph
    calls=[];real=graph.MetricGrid.connections
    def connections(*args,**kwargs):calls.append(1);return real(*args,**kwargs)
    monkeypatch.setattr(graph.MetricGrid,'connections',connections)
    m=np.zeros((19,19),np.uint8);m[2:17,2:17]=1
    t=task(0,0,19,19);sensor=SensorSpec(range_m=3)
    core=DecisionCore(t,sensor);s=snapshot(m,pose=Pose(4.7,4.7,0))
    first,_=core.observe(s)
    changed=replace(s,revision=s.revision+1,pose=Pose(4.75,4.75,.6))
    second,_=core.observe(changed)
    assert len(calls)==1
    np.testing.assert_array_equal(first.features[:-1,3:19],second.features[:-1,3:19])
    np.testing.assert_array_equal(second.positions[-1],[np.float32(4.75),np.float32(4.75)])
    assert not np.array_equal(first.features[:,:2],second.features[:,:2])
    core.record_observation(changed.pose,[[4,4]])
    third,_=core.observe(changed)
    assert len(calls)==1
    assert np.any(third.features[:,19:27]!=second.features[:,19:27])


def test_obstacle_change_invalidates_cached_geometry(monkeypatch):
    import lunar_drl_exploration.graph as graph
    calls=[];real=graph.MetricGrid.connections
    def connections(*args,**kwargs):calls.append(1);return real(*args,**kwargs)
    monkeypatch.setattr(graph.MetricGrid,'connections',connections)
    m=np.ones((19,19),np.uint8);core=DecisionCore(task(0,0,19,19),SensorSpec(range_m=3))
    core.observe(snapshot(m,pose=Pose(4.7,4.7,0)))
    m[8:12,8]=2
    changed,_=core.observe(replace(snapshot(m,pose=Pose(4.7,4.7,0)),revision=2))
    assert len(calls)==2
    assert len(changed.edges)>0


def test_gain_cache_reuses_rays_and_invalidates_sensor_and_intrinsic(monkeypatch):
    import lunar_drl_exploration.graph as graph
    real=graph.directional_frontier_counts;calls=[]
    def counts(*args,**kwargs):calls.append(len(args[2]));return real(*args,**kwargs)
    monkeypatch.setattr(graph,'directional_frontier_counts',counts)
    m=np.zeros((19,19),np.uint8);m[2:17,2:17]=1
    core=DecisionCore(task(0,0,19,19),SensorSpec(range_m=3))
    s=snapshot(m,pose=Pose(4.7,4.7,0));first,_=core.observe(s)
    before=len(calls);second,_=core.observe(replace(s,revision=2))
    assert len(calls)==before
    np.testing.assert_array_equal(first.features[:,3:19],second.features[:,3:19])
    core.sensor=SensorSpec(range_m=2)
    core.observe(replace(s,revision=3));assert len(calls)>before
    before=len(calls);m[8:12,8]=2
    core.observe(replace(snapshot(m,pose=Pose(4.7,4.7,0)),revision=4))
    assert len(calls)>before


def test_final_snapshot_requires_request_after_stop():
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env=object.__new__(RosExplorationEnv);env._latest_sensor_ns=123
    events=[];snap=SimpleNamespace()
    class MapAdapter:
        processed_stamp_ns=123
        def require_fresh_snapshot(self):events.append('fresh')
        def poll_snapshot(self,minimum,*,refresh):
            assert events==['fresh'] and minimum==123 and refresh is False
            return snap
    env.adapter=MapAdapter();env._wait=lambda predicate,reason:predicate()
    env.buffer=SimpleNamespace(acknowledge=lambda stamp:events.append('ack'))
    env._apply_known=lambda value:events.append('known')
    assert env._snapshot() is snap and events==['fresh','ack','known']


def test_cached_gain_matches_direct_counts_for_same_and_cross_cell_anchor():
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    from lunar_drl_exploration.feature_schema import IN_GAIN_SCALE,OUT_GAIN_SCALE
    m=np.zeros((19,19),np.uint8);m[2:17,2:17]=1
    core=DecisionCore(task(0,0,19,19),SensorSpec(range_m=3))
    for revision,x in enumerate((4.7,4.75,5.7,4.7),1):
        s=replace(snapshot(m,pose=Pose(x,4.7,.6)),revision=revision)
        obs,_=core.observe(s);w=core.analyzer.workspace
        for front,section,scale in ((core.frontiers.inside,slice(3,11),IN_GAIN_SCALE),(core.frontiers.outside,slice(11,19),OUT_GAIN_SCALE)):
            expected=directional_frontier_counts(front,w,obs.positions,core.sensor,s.resolution_m)
            np.testing.assert_array_equal(obs.features[:,section],(expected/scale).astype(np.float32))


def test_fresh_boundary_rejects_outstanding_response_with_opt_in_cache(adapter):
    a=adapter;c=Client();a.client=c;s=Adapter().snapshot
    assert a.poll_snapshot(refresh=False) is None
    previous=c.future;a.require_fresh_snapshot()
    previous.response=reply(s)
    assert a.poll_snapshot(refresh=False) is None
    assert len(c.requests)==2 and c.future is not previous
    current=replace(s,pose=Pose(3.5,2.5,0.))
    c.future.response=reply(current)
    assert a.poll_snapshot(refresh=False).pose==current.pose
    assert len(c.requests)==2


def test_gain_cache_invalidates_frontier_demands_and_lattice_offset():
    from lunar_drl_exploration.frontiers import directional_frontier_counts
    m=np.zeros((19,19),np.uint8);m[2:17,2:17]=1
    core=DecisionCore(task(0,0,19,19),SensorSpec(range_m=3))
    s=snapshot(m,pose=Pose(4.7,4.7,0));obs,_=core.observe(s)
    w=core.analyzer.workspace;front=core.frontiers.inside;builder=core.builder
    key=(s.epoch,s.resolution_m,tuple(w.bounds),tuple(w.origin),tuple(w.lattice_offset))
    points=obs.positions
    for f in (front,replace(front,demand_ids=front.demand_ids+1),front):
        expected=directional_frontier_counts(f,w,points,core.sensor,s.resolution_m)
        actual=builder._cached_gains('inside',f,w,points,core.sensor,s.resolution_m,key)
        np.testing.assert_array_equal(actual,expected)
    moved=replace(w,lattice_offset=(w.lattice_offset[0]+1,w.lattice_offset[1]))
    moved_key=(*key[:-1],tuple(moved.lattice_offset))
    expected=directional_frontier_counts(front,moved,points,core.sensor,s.resolution_m)
    actual=builder._cached_gains('inside',front,moved,points,core.sensor,s.resolution_m,moved_key)
    np.testing.assert_array_equal(actual,expected)
