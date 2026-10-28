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
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[3]/"scripts/navigation/one_cm/path_follower_validation"))
    from evidence import legacy
    geometry=legacy("motion")
    assert not geometry.swept_clear(geometry.State(),geometry.State(yaw=1.57),[(.45,.45,.5,.5)])
    assert geometry.swept_clear(geometry.State(),geometry.State(yaw=.2),[(2.,2.,2.1,2.1)])

def test_terminal_requires_contiguous_stopped_hold():
    from evidence import legacy
    evaluator=legacy("evaluate")
    states=[dict(elapsed_s=i*.1,x=.8,y=0.,yaw=0.,v=.011,w=0.) for i in range(8)]
    assert not evaluator.held_terminal(states,(.8,0.,0.),0.,1.)
    for r in states:r["v"]=0.
    assert evaluator.held_terminal(states,(.8,0.,0.),0.,1.)

def test_wrong_ros_type_and_production_command_graph_veto():
    from evaluate import graph_valid
    from evidence import TOPICS,CMD
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
