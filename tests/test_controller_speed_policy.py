"""隔离执行节点现有参数合并块；不是 ROS 节点运行验证。"""
import ast
from pathlib import Path
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('cap,override,standard,expected,angular_cap,angular_override,angular',[(.5,None,True,.5,1.2,None,1.2),(.5,.1,True,.1,1.2,.4,.4),(.5,1.,True,.5,1.2,2.,1.2),(.2,None,True,.2,1.,None,1.),(.5,None,False,.2,1.2,None,.5)])
def test_node_policy_merge_uses_platform_speed(cap,override,standard,expected,angular_cap,angular_override,angular):
    path=ROOT/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/node.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PureWheeledControllerNode')
    init=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
    capability_if=next(n for n in init.body if isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='platform_path')
    loop=next(n for n in capability_if.body if isinstance(n,ast.For))
    standard_block=next(n for n in init.body if isinstance(n,ast.If) and ast.unparse(n.test)=='self._standard_mode' and any(isinstance(x,ast.For) for x in n.body))
    names={'max_linear_mps':'maximum_forward_speed_mps','max_reverse_mps':'maximum_reverse_speed_mps','max_angular_radps':'maximum_spin_rate_radps'}
    capability={'maximum_forward_speed_mps':cap,'maximum_reverse_speed_mps':cap,'maximum_spin_rate_radps':angular_cap}
    values={'max_linear_mps':.2,'max_reverse_mps':.2,'max_angular_radps':.5}
    overrides={}
    if override is not None:
        values.update(max_linear_mps=override,max_reverse_mps=override);overrides={'max_linear_mps':override,'max_reverse_mps':override}
    if angular_override is not None:
        values['max_angular_radps']=angular_override;overrides['max_angular_radps']=angular_override
    ns={'self':SimpleNamespace(_standard_mode=standard,_parameter_overrides=overrides),'names':names,'capability':capability,'policy_values':values,'platform_path':'wheel.yaml'}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[loop,standard_block],type_ignores=[])),str(path),'exec'),ns)
    assert values['max_linear_mps']==pytest.approx(expected)
    assert values['max_reverse_mps']==pytest.approx(expected)
    assert values['max_angular_radps']==pytest.approx(angular)
