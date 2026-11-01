"""标准Path不按固定接收间隔作废；配对仍有界且不能重放。"""
from pathlib import Path
from types import SimpleNamespace as S
import importlib.util


def test_delayed_pair_is_accepted_without_ttl_but_cannot_replay():
    p=Path(__file__).resolve().parents[1]/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/execution_pairing.py'
    spec=importlib.util.spec_from_file_location('pairing',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    inbox=m.ExecutionInbox(capacity=2,max_age_s=None)
    reference=S(session_id='a',stamp_ns=1,path_digest='hash',revision=1)
    inbox.add_path(1,'hash','geometry',0.)
    assert inbox.add_reference(reference,10.)
    assert inbox.pop_ready(100.)[:2]==(reference,'geometry')
    assert not inbox.add_reference(reference,101.)
    for i in range(10):inbox.add_path(i+2,str(i),'unused',102.)
    assert len(inbox.paths)==2


def test_reference_ingress_accepts_old_timestamp_but_rejects_future_or_wrong_frame():
    # 只提取真实回调以在无rclpy的本机验证，不替代ROS消息/节点测试。
    import ast
    from unittest.mock import patch
    from scripts.navigation.one_cm.nav1cm.execution_contract import ExecutionReference
    from scripts.navigation.one_cm.nav1cm import execution_ros
    p=Path(__file__).resolve().parents[1]/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/node.py'
    tree=ast.parse(p.read_text(encoding='utf-8'));cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PureWheeledControllerNode')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_on_execution_reference')
    module=ast.Module(body=[method],type_ignores=[]);scope={};exec(compile(module,str(p),'exec'),scope)
    callback=scope['_on_execution_reference'];calls=[]
    n=S(_map_frame='map',get_clock=lambda:S(now=lambda:S(nanoseconds=1000)),
        _execution_inbox=S(add_reference=lambda *args:(calls.append('received') or True)),
        _executor=S(execution_reference=None),request_controlled_stop=lambda reason:None,
        _try_execution_pair=lambda:calls.append('pair'))
    import sys,time
    sys.modules.setdefault('nav1cm.execution_ros',execution_ros)
    ref=ExecutionReference('a'*32,1,'b'*64,0,1)
    with patch.object(execution_ros,'reference_from_message',return_value=ref):
        callback(n,S(header=S(frame_id='map')),received_at=0.)
    assert calls==['received','pair'];calls.clear()
    from dataclasses import replace
    with patch.object(execution_ros,'reference_from_message',return_value=replace(ref,stamp_ns=1001)):
        callback(n,S(header=S(frame_id='map')),received_at=0.)
    assert not calls
    with patch.object(execution_ros,'reference_from_message',return_value=ref):
        callback(n,S(header=S(frame_id='wrong')),received_at=0.)
    assert not calls
