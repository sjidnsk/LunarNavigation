"""profile由运行前外部声明，生产参数不能反向放宽grader。"""
import importlib.util,json
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]

def load():
    path=ROOT/'scripts/navigation/one_cm/path_follower_validation/profiles.py'
    assert path.exists()
    spec=importlib.util.spec_from_file_location('joint_profiles',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def test_strict_profile_independent_bounds_and_hold(tmp_path):
    mod=load()
    expected={'max_v':0.2,'max_w':0.1,'max_a':0.5,'max_alpha':0.5,'curvature':1.0,'xy':0.1,'yaw':0.02}
    assert not mod.within_bounds({'v':0.0,'w':0.11,'a':0.0,'alpha':0.0},expected)
    assert mod.within_bounds({'v':0.0,'w':0.1,'a':0.0,'alpha':0.0},expected)
    rows=[dict(t=i/10,x=0.,y=0.,yaw=.03,v=0.,w=0.) for i in range(5)]
    assert not mod.held(rows,(0.,0.,0.),expected)
    for row in rows: row['yaw']=.01
    assert mod.held(rows,(0.,0.,0.),expected)
