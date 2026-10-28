"""破坏真实运动/图类型/正式来源/进程/车体证据时必须否决。"""
import importlib.util
from pathlib import Path
import pytest
SPEC=importlib.util.spec_from_file_location("joint_grade",Path(__file__).resolve().parents[3]/"scripts/navigation/one_cm/path_follower_validation/grade.py")
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)

def evidence():
    return dict(states=[dict(t=0.,x=0.,y=0.,yaw=0.,v=0.,w=0.,a=0.,alpha=0.,clear=True),
                        dict(t=1.,x=.4,y=0.,yaw=0.,v=.1,w=0.,a=0.,alpha=0.,clear=True)],
                graph_ok=True,formal=True,installed=True,workers_clean=True,
                paths=35,processes=[dict(returncode=0,premature=False,remaining=[],escalated=False)],
                terminal=True,supervisor_error=None)

def test_accepts_complete_evidence():
    assert M.grade(evidence())["pass_"]

@pytest.mark.parametrize("field,value",[
    ("states",[]),("graph_ok",False),("formal",False),("installed",False),
    ("workers_clean",False),("paths",0),("terminal",False),("supervisor_error","unexpected exit"),
    ("processes",[dict(returncode=0,premature=True,remaining=[],escalated=False)])])
def test_missing_proof_vetoes(field,value):
    e=evidence();e[field]=value
    assert not M.grade(e)["pass_"]

@pytest.mark.parametrize("field,value",[("x",0.),("clear",False),("v",.21),("w",1.1),("a",.6),("alpha",.6)])
def test_raw_motion_violation_vetoes(field,value):
    e=evidence();e["states"][-1][field]=value
    assert not M.grade(e)["pass_"]

def test_independent_sat_catches_rotating_front_corner():
    from scripts.navigation.one_cm.path_follower_validation.evidence import legacy
    geometry=legacy("motion")
    assert not geometry.swept_clear(geometry.State(),geometry.State(yaw=1.57),[(.45,.45,.5,.5)])
    assert geometry.swept_clear(geometry.State(),geometry.State(yaw=.2),[(2.,2.,2.1,2.1)])

def test_terminal_requires_contiguous_stopped_hold():
    from scripts.navigation.one_cm.path_follower_validation.evidence import legacy
    evaluator=legacy("evaluate")
    states=[dict(elapsed_s=i*.1,x=.8,y=0.,yaw=0.,v=.011,w=0.) for i in range(8)]
    assert not evaluator.held_terminal(states,(.8,0.,0.),0.,1.)
    for r in states:r["v"]=0.
    assert evaluator.held_terminal(states,(.8,0.,0.),0.,1.)

def test_wrong_ros_type_and_production_command_graph_veto():
    from scripts.navigation.one_cm.path_follower_validation.evaluate import graph_valid
    from scripts.navigation.one_cm.path_follower_validation.evidence import TOPICS,CMD
    entries={TOPICS["grid"]:("grid_map_msgs/msg/GridMap","nav1cm_joint_plant"),
        TOPICS["odom"]:("nav_msgs/msg/Odometry","nav1cm_joint_plant"),
        TOPICS["local"]:("nav_msgs/msg/Path","nav1cm_joint_plant"),
        CMD:("geometry_msgs/msg/Twist","lunar_pure_wheeled_controller"),
        "/tf_static":("tf2_msgs/msg/TFMessage","nav1cm_joint_plant")}
    graph=dict(types={t:[k] for t,(k,n) in entries.items()},
       publishers={t:[dict(node_name=n,topic_type=k)] for t,(k,n) in entries.items()},
       subscriptions={TOPICS["local"]:[dict(node_name="lunar_pure_wheeled_controller")],
                      CMD:[dict(node_name="nav1cm_joint_plant")]})
    assert graph_valid(graph,"empty_smoke")
    graph["types"][CMD]=["std_msgs/msg/String"]
    assert not graph_valid(graph,"empty_smoke")
    graph["types"][CMD]=["geometry_msgs/msg/Twist"]
    graph["publishers"]["/Car/T5/Car_Cmd_Vel"]=[dict(node_name="unexpected")]
    assert not graph_valid(graph,"empty_smoke")

def qos_evidence():
    from scripts.navigation.one_cm.path_follower_validation.evidence import TOPICS
    profile=dict(reliability="RELIABLE",durability="VOLATILE",history="KEEP_LAST",depth=1)
    static=dict(reliability="RELIABLE",durability="TRANSIENT_LOCAL",history="KEEP_LAST",depth=100)
    rows=[dict(node_name="lunar_pure_wheeled_controller",direction="subscriptions",topic=TOPICS["local"],qos=profile.copy()),
          dict(node_name="nav1cm_joint_plant",direction="publishers",topic=TOPICS["local"],qos=profile.copy()),
          dict(node_name="nav1cm_joint_plant",direction="publishers",topic="/tf_static",qos=static.copy()),
          dict(node_name="lunar_pure_wheeled_controller",direction="subscriptions",topic="/tf_static",qos=static.copy())]
    graph=dict(publishers={},subscriptions={})
    for row in rows:
        graph[row["direction"]].setdefault(row["topic"],[]).append(dict(node_name=row["node_name"],qos=row["qos"].copy()))
    return graph,rows

@pytest.mark.parametrize("field,value",[("reliability","BEST_EFFORT"),("durability","TRANSIENT_LOCAL"),("depth",2)])
def test_qos_wrong_actual_path_entity_veto(field,value):
    from scripts.navigation.one_cm.path_follower_validation.evaluate import qos_valid
    graph,entities=qos_evidence()
    entities[0]["qos"][field]=value
    assert not qos_valid(graph,entities,"empty_smoke")

def test_qos_unknown_graph_depth_requires_real_entity_depth():
    from scripts.navigation.one_cm.path_follower_validation.evaluate import qos_valid
    from scripts.navigation.one_cm.path_follower_validation.evidence import TOPICS
    graph,entities=qos_evidence()
    for direction in graph.values():
        for rows in direction.values():
            for r in rows:r["qos"].update(history="UNKNOWN",depth=None)
    assert qos_valid(graph,entities,"empty_smoke")
    entities[0]["qos"]["depth"]=None
    assert not qos_valid(graph,entities,"empty_smoke")

def test_tf_static_actual_durability_and_graph_reliability_veto():
    from scripts.navigation.one_cm.path_follower_validation.evaluate import qos_valid
    graph,entities=qos_evidence()
    entities[-1]["qos"]["durability"]="VOLATILE"
    assert not qos_valid(graph,entities,"empty_smoke")
    graph,entities=qos_evidence()
    graph["publishers"]["/tf_static"][0]["qos"]["reliability"]="BEST_EFFORT"
    assert not qos_valid(graph,entities,"empty_smoke")

@pytest.mark.parametrize("folder",["p3_path_validation","path_follower_validation"])
def test_direct_script_help_uses_isolated_package(folder):
    import subprocess
    import sys
    script=Path(__file__).resolve().parents[3]/"scripts/navigation/one_cm"/folder/"run.py"
    result=subprocess.run([sys.executable,str(script),"--help"],capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    assert "--role" in result.stdout and "--output" in result.stdout

def test_legacy_loader_and_joint_package_keep_separate_evaluators():
    import importlib
    from scripts.navigation.one_cm.path_follower_validation.evidence import legacy
    old=legacy("evaluate")
    new=importlib.import_module("scripts.navigation.one_cm.path_follower_validation.evaluate")
    assert old is not new
    assert hasattr(old,"apply_supervision_gate")
    import inspect
    assert "start" in inspect.signature(old.analyze).parameters
    assert "error" in inspect.signature(new.analyze).parameters

def test_independent_plant_rejects_real_production_packages_without_rejecting_harness_names():
    from scripts.navigation.one_cm.path_follower_validation.evidence import production_modules
    names=["nav1cm_joint_validation","nav1cm_joint_validation.plant",
           "nav1cm_p3_path_validation.motion","nav1cm","nav1cm.maps",
           "lunar_pure_wheeled_controller.node","lunar_nav1cm_planner.node"]
    assert production_modules(names)==["nav1cm","nav1cm.maps","lunar_pure_wheeled_controller.node","lunar_nav1cm_planner.node"]

def test_executor_context_copies_real_observation_without_mutating_history():
    from types import SimpleNamespace
    import numpy as np
    from scripts.navigation.one_cm.path_follower_validation.evidence import executor_context
    history=[(2.,np.array([.1,.02]))]
    executor=SimpleNamespace(phase="BRAKING",failure=None,_last_command=np.array([.1,.02]),
        _command_history=history,map=SimpleNamespace(origin=(-2.5,-2.5),resolution=.01,revision=7,epoch="measured"))
    snapshot=executor_context(executor,.05,dict(now_s=2.1,state_age_s=.02))
    assert snapshot["dt"]==.05 and snapshot["now_s"]==2.1 and snapshot["state_age_s"]==.02
    assert snapshot["phase"]=="BRAKING" and snapshot["failure"] is None
    assert snapshot["map"]==dict(origin=(-2.5,-2.5),resolution=.01,revision=7,epoch="measured")
    assert snapshot["last_command"]==[.1,.02] and snapshot["command_history"]==[(2.,[.1,.02])]
    snapshot["command_history"][0][1][0]=9.
    assert history[0][1][0]==.1


def complete_joint_analysis_evidence(tmp_path):
    """构造其他门禁全通过的离线证据，单独验证异常和禁入话题否决。"""
    import json
    from scripts.navigation.one_cm.path_follower_validation.evidence import TOPICS, CMD
    _, entities = qos_evidence()
    entities[1]["node_name"] = "lunar_nav1cm_planner"
    entities.extend([
        dict(entities[1], topic=TOPICS["global_route"]),
        dict(entities[-1], node_name="lunar_nav1cm_planner"),
    ])
    entries = {
        TOPICS["grid"]: ("grid_map_msgs/msg/GridMap", "nav1cm_joint_plant"),
        TOPICS["odom"]: ("nav_msgs/msg/Odometry", "nav1cm_joint_plant"),
        TOPICS["goal"]: ("geometry_msgs/msg/PoseStamped", "nav1cm_joint_plant"),
        TOPICS["local"]: ("nav_msgs/msg/Path", "lunar_nav1cm_planner"),
        TOPICS["global_route"]: ("nav_msgs/msg/Path", "lunar_nav1cm_planner"),
        CMD: ("geometry_msgs/msg/Twist", "lunar_pure_wheeled_controller"),
        "/tf_static": ("tf2_msgs/msg/TFMessage", "nav1cm_joint_plant"),
    }
    graph = dict(kind="GRAPH", types={t: [k] for t, (k, _) in entries.items()},
                 publishers={t: [dict(node_name=n, topic_type=k)] for t, (k, n) in entries.items()},
                 subscriptions={TOPICS["local"]: [dict(node_name="lunar_pure_wheeled_controller")],
                                CMD: [dict(node_name="nav1cm_joint_plant")]})
    for entity in entities:
        endpoint = graph[entity["direction"]].setdefault(entity["topic"], [])
        row = next((r for r in endpoint if r["node_name"] == entity["node_name"]), None)
        if row is None:
            row = dict(node_name=entity["node_name"])
            endpoint.append(row)
        row["qos"] = entity["qos"].copy()
    path = dict(frame="map", stamp_ns=100,
                poses=[dict(x=2.6, y=1.4, z=0., q=[0., 0., 0., 1.], stamp_ns=100, frame="map")])
    plant = [graph, dict(kind="ENTITY_QOS", entities=entities), dict(kind="TF", stamp=0),
             dict(kind="EXIT", forbidden=[])]
    plant += [dict(kind="STATE", t=t, x=x, y=y, yaw=0., v=0., w=0., a=0., alpha=0., clear=True)
              for t, x, y in [(0., 0., 0.), (1., 2.6, 1.4), (1.4, 2.6, 1.4)]]
    plant += [dict(kind="PATH", stream="local", message=path) for _ in range(35)]
    follower = [dict(kind="INSTALL", executor="StandardPathFollower", module="/validated/install/follower.py"),
                dict(kind="CLOSE", map_worker_alive=False)]
    planner = [dict(kind="INSTALL"), dict(kind="CLOSE", remaining_children=[],
                    result=dict(threads_alive=[], workers={})),
               dict(kind="TICK", goal=[2.6, 1.4], generation=2, goal_revision=3,
                    global_result=dict(planning_outcome=0, reason_code="PLAN_FOUND", has_reference=True,
                                       requested_goal=[2.6, 1.4], requested_epoch=8, requested_version=4),
                    events=[dict(kind="GLOBAL_RESULT_RECEIVED", accepted_generation_at_submit=2, goal_revision_at_submit=3),
                            dict(kind="ROUTE_ACCEPTED", version=4, map_epoch=8, points=[[0., 0.], [2.6, 1.4]])])]
    actors = dict(plant=plant, follower=follower, planner=planner)
    def write():
        for role, rows in actors.items():
            (tmp_path / (role + ".jsonl")).write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    write()
    return actors, write, [dict(returncode=0, premature=False, remaining=[], escalated=False)]


def test_complete_analysis_allows_normal_infeasible_candidate(tmp_path):
    from scripts.navigation.one_cm.path_follower_validation.evaluate import analyze
    actors, write, processes = complete_joint_analysis_evidence(tmp_path)
    baseline = analyze(tmp_path, "obstacle", processes, None, None)
    assert baseline["pass_"] and all(baseline["checks"].values())
    actors["planner"].extend([
        dict(kind="NOTICE", notice="LOCAL_FAILED", detail="NO_FEASIBLE_MPCC_CANDIDATE"),
        dict(kind="TICK", global_result=None, events=[dict(kind="LOCAL_FAILED", reason="NO_FEASIBLE_MPCC_CANDIDATE")]),
    ])
    write()
    scored = analyze(tmp_path, "obstacle", processes, None, None)
    assert scored["pass_"] and all(scored["checks"].values())


@pytest.mark.parametrize("anomaly, veto", [("tick_error", "runtime_exceptions_absent"),
                                          ("later_t5", "production_command_absent")])
def test_analysis_independent_veto_with_otherwise_all_pass(tmp_path, anomaly, veto):
    from copy import deepcopy
    from scripts.navigation.one_cm.path_follower_validation.evaluate import analyze
    actors, write, processes = complete_joint_analysis_evidence(tmp_path)
    baseline = analyze(tmp_path, "obstacle", processes, None, None)
    assert baseline["pass_"] and all(baseline["checks"].values())
    if anomaly == "tick_error":
        actors["planner"].append(dict(kind="NOTICE", notice="TICK_ERROR", detail="RuntimeError: injected"))
    else:
        late = deepcopy(actors["plant"][0])
        late["publishers"]["/Car/T5/Car_Cmd_Vel"] = [dict(node_name="unexpected")]
        actors["plant"].append(late)
    write()
    scored = analyze(tmp_path, "obstacle", processes, None, None)
    assert not scored["pass_"], scored
    assert scored["checks"][veto] is False
    assert all(value for name, value in scored["checks"].items() if name != veto)
