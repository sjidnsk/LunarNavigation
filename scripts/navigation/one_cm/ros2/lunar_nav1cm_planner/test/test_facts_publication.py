"""共享事实发布的线程/提交合同；不构成ROS或车辆验证。"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
import pytest
from test_runtime import build_runtime, runtime_api
from nav1cm.contracts import WorldSnapshot


def message(stamp=12_345_678_901):
    sec, nanosec = divmod(stamp, 1_000_000_000)
    return NS(header=NS(frame_id="map", stamp=NS(sec=sec, nanosec=nanosec)))


class Store:
    source_resolution_m = .05
    def __init__(self):
        self.snapshot = WorldSnapshot(3, (), ())
        self.facts = NS(world=self.snapshot, raw_revision=8)
        self.calls = self.captures = 0
        self.fail = False
    def apply(self, message, transform):
        self.calls += 1
        if self.fail: raise ValueError("bad source")
        return self.snapshot
    def capture_facts(self):
        self.captures += 1
        return self.facts
    def close(self): pass


def wait_for(predicate):
    deadline = time.monotonic() + 2.
    while time.monotonic() < deadline:
        if predicate(): return
        time.sleep(.005)
    assert predicate()


def runtime_with_sink(store, sink):
    _, session, _, logs = build_runtime(store)
    runtime = runtime_api().PlannerRuntime(session, store, publish=lambda p: None,
        ros_now_ns=lambda: 999, notice=lambda *args: logs.append(args),
        map_frame="map", facts_sink=sink)
    return runtime, logs


def test_committed_facts_keep_world_identity_stamp_resolution_and_run_outside_lock():
    store, received = Store(), []
    def sink(facts, **metadata):
        assert threading.current_thread().name == "nav1cm-map"
        assert not runtime._condition._is_owned()
        assert runtime._world is store.snapshot is facts.world
        received.append((facts, metadata))
    runtime, logs = runtime_with_sink(store, sink)
    runtime.start()
    try:
        runtime.offer_map(message())
        wait_for(lambda: len(received) == 1)
        runtime.offer_map(message(12_345_678_902))
        wait_for(lambda: len(received) == 2)
        assert received[0][0] is store.facts is received[1][0]
        assert received[0][1] == dict(source_resolution_m=.05, frame_id="map", processed_stamp_ns=12_345_678_901)
        assert received[1][1]["processed_stamp_ns"] == 12_345_678_902
        assert store.calls == store.captures == 2
    finally:
        assert not runtime.close()["threads_alive"]


def test_failed_apply_never_captures_or_sends():
    store, received = Store(), []
    store.fail = True
    runtime, logs = runtime_with_sink(store, lambda *a, **k: received.append(a))
    initial = runtime._world
    runtime.start()
    try:
        runtime.offer_map(message())
        wait_for(lambda: any(k == "MAP_REJECTED" for k, d in logs))
        assert runtime._world is initial
        assert not received and store.captures == 0 and store.calls == 1
    finally:
        runtime.close()


def test_sink_exception_cannot_rollback_world_retry_apply_or_stop_worker():
    store = Store()
    def sink(*args, **kwargs): raise RuntimeError("transport failed")
    runtime, logs = runtime_with_sink(store, sink)
    runtime.start()
    try:
        runtime.offer_map(message())
        wait_for(lambda: any(k == "MAP_FACTS_REJECTED" for k, d in logs))
        assert runtime._world is store.snapshot
        assert store.calls == 1
        runtime.offer_map(message(13_000_000_000))
        wait_for(lambda: store.calls == 2)
        assert runtime._world is store.snapshot
        assert runtime._threads[0].is_alive()
    finally:
        runtime.close()


def test_default_without_sink_does_not_capture_facts():
    store = Store()
    runtime, _, _, logs = build_runtime(store)
    runtime.start()
    try:
        runtime.offer_map(message())
        wait_for(lambda: any(k == "MAP_APPLIED" for k, d in logs))
        assert store.captures == 0
    finally:
        runtime.close()


def test_mismatched_facts_are_not_sent_and_world_stays_committed():
    store, received = Store(), []
    store.facts = NS(world=WorldSnapshot(99, (), ()), raw_revision=8)
    runtime, logs = runtime_with_sink(store, lambda *a, **k: received.append(a))
    runtime.start()
    try:
        runtime.offer_map(message())
        wait_for(lambda: any(k == "MAP_FACTS_REJECTED" for k, d in logs))
        assert not received and runtime._world is store.snapshot and store.calls == 1
    finally:
        runtime.close()


NODE = Path(__file__).resolve().parents[1] / "lunar_nav1cm_planner/node.py"


def test_node_optional_publisher_is_configured_before_start_and_parameter_is_startup_only():
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PlannerNode")
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    defaults = next(n.value for n in ast.walk(init) if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "defaults" for t in n.targets))
    pairs = dict(zip((ast.literal_eval(k) for k in defaults.keys), defaults.values))
    assert ast.literal_eval(pairs["base_map_facts_topic"]) == ""
    configure = next(n for n in ast.walk(init) if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == "_configure_facts_publisher")
    start = next(n for n in ast.walk(init) if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == "start")
    assert configure.lineno < start.lineno
    assert "startup_names=('wheel_config',*overrides,*defaults)" in ast.get_source_segment(NODE.read_text(encoding="utf-8"), init)


def test_node_sink_uses_real_codec_one_lifecycle_epoch_and_retained_qos(monkeypatch):
    import json
    import sys
    from types import MethodType
    from nav1cm.base_map import BaseMapFacts
    tree = ast.parse(NODE.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "PlannerNode")
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
        and n.name in ("_configure_facts_publisher", "_publish_facts")]
    assert len(methods) == 2
    clock_calls, messages, publishers, logs = [], [], [], []
    def clock():
        clock_calls.append(True)
        return 123456789
    monkeypatch.setitem(sys.modules, "std_msgs.msg", NS(String=lambda: NS(data="")))
    namespace = dict(time=NS(time_ns=clock), QoSProfile=lambda **kw: NS(**kw),
        ReliabilityPolicy=NS(RELIABLE="reliable"),
        DurabilityPolicy=NS(TRANSIENT_LOCAL="transient_local"))
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(NODE), "exec"), namespace)
    node = NS(get_logger=lambda: NS(info=logs.append))
    node._publish_facts = MethodType(namespace["_publish_facts"], node)
    def create(kind, topic, qos):
        publishers.append((kind, topic, qos))
        return NS(publish=messages.append)
    node.create_publisher = create
    configure = MethodType(namespace["_configure_facts_publisher"], node)
    assert configure("") is None
    assert not clock_calls and not publishers
    sink = configure("/shared/facts")
    assert len(clock_calls) == 1
    assert publishers[0][1] == "/shared/facts"
    assert vars(publishers[0][2]) == dict(depth=1, reliability="reliable", durability="transient_local")
    facts = BaseMapFacts(WorldSnapshot(9, (), ()), 21, ())
    for stamp in (100, 101):
        sink(facts, source_resolution_m=.05, frame_id="map", processed_stamp_ns=stamp)
    wire = [json.loads(m.data) for m in messages]
    assert [v["processed_stamp_ns"] for v in wire] == [100, 101]
    assert all(v["epoch"] == 123456789 and v["raw_revision"] == 21 and v["source_resolution_m"] == .05 for v in wire)
    assert len(clock_calls) == 1
    assert any("21" in v and "123456789" in v and "0.05" in v for v in logs)
