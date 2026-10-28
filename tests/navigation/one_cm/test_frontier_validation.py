"""联合证据门禁不可把局部Path或传输接受当实际探索到达。"""
from importlib.util import spec_from_file_location,module_from_spec
from pathlib import Path
import pytest

FILE=Path(__file__).resolve().parents[3]/"scripts/navigation/one_cm/frontier_validation/grade.py"
def load():
    assert FILE.exists(), "前沿联合门禁尚未实现"
    spec=spec_from_file_location("frontier_grade",FILE);m=module_from_spec(spec);spec.loader.exec_module(m);return m

def evidence():
    return dict(case="frontier_progress",common={k:True for k in ("graph","formal","motion","bounds","clearance","cleanup","installed","no_exceptions","source_received")},
                reached=[dict(uuid="current",xy_error=.05,yaw_error=.02,v=0.,w=0.,hold_s=.3,status=4,outcome=0)],
                distinct_goals=2,observed_growth=20,waits=0,canceled_stopped=0,cancel_while_moving=False,no_goals_after_cancel=True)

def test_actual_terminal_requires_uuid_xy_yaw_stop_and_hold():
    grade=load().grade
    e=evidence();assert grade(e)["pass_"]
    for key,bad in [("uuid",""),("xy_error",.2),("yaw_error",.1),("v",.02),("w",.02),("hold_s",.1),("status",6),("outcome",5)]:
        x=evidence();x["reached"][0][key]=bad;assert not grade(x)["pass_"],key

def test_path_or_no_map_cannot_count_as_frontier_completion():
    e=evidence();e["reached"]=[];e["nonempty_paths"]=100
    assert not load().grade(e)["pass_"]

@pytest.mark.parametrize("key",["graph","formal","motion","bounds","clearance","cleanup","installed","no_exceptions","source_received"])
def test_every_common_safety_and_identity_gate_is_independent(key):
    e=evidence();e["common"][key]=False;assert not load().grade(e)["pass_"]

def test_blocked_wait_requires_real_cancel_stop_and_distinct_frontend_goal():
    e=evidence();e.update(case="blocked_wait_reselect",waits=3,canceled_stopped=1,distinct_goals=2)
    assert load().grade(e)["pass_"]
    for k,v in [("waits",0),("canceled_stopped",0),("distinct_goals",1)]:
        x={**e,k:v};assert not load().grade(x)["pass_"]

def test_external_cancel_requires_measured_motion_stop_and_no_reselection():
    e=evidence();e.update(case="task_cancel_stop",canceled_stopped=1,cancel_while_moving=True)
    assert load().grade(e)["pass_"]
    e["no_goals_after_cancel"]=False;assert not load().grade(e)["pass_"]


def test_passive_action_probe_uuid_uses_python_wire_integers():
    import sys
    import numpy as np
    path=FILE.with_name("run.py")
    spec=spec_from_file_location("frontier_runner",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    assert hasattr(module,"uuid_values"), "UUID旁观请求缺少wire整数规范化"
    values=module.uuid_values(np.arange(16,dtype=np.uint8))
    assert values==list(range(16)) and all(type(v) is int for v in values)
