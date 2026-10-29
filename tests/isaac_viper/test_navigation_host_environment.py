"""host前置环境门：错误域不能触碰ROS，独立executor工厂不受main门影响。"""
import builtins
import hashlib
import json
import sys
import numpy as np
import pytest
from tools.isaac_viper import navigation_showcase as nav


class RosImportReached(AssertionError): pass


def setup_main(tmp_path,monkeypatch,environment):
    source=tmp_path/'map.npz'
    np.savez_compressed(source,elevation=np.zeros((2,2),np.float32),bounds_xy_m=[235.,235.4,-315.,-314.6],resolution_m=.2)
    scene=tmp_path/'scene.json'
    scene.write_text(json.dumps(dict(schema='lunar.viper-navigation-scene.v1',region='A',resolution_m=.2,
        map_npz=str(source),map_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),bounds_xy_m=[235.,235.4,-315.,-314.6])),encoding='utf-8')
    output=tmp_path/'host-output'; overlay=tmp_path/'new-overlay'/'paths.json'
    monkeypatch.setattr(sys,'argv',['navigation_showcase','--scene-json',str(scene),'--output-dir',str(output),'--overlay-file',str(overlay)])
    for key in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_DISTRO'): monkeypatch.delenv(key,raising=False)
    for key,value in environment.items(): monkeypatch.setenv(key,value)
    touched=[]; original_import=builtins.__import__
    def guarded_import(name,*args,**kwargs):
        if name=='rclpy' or name.startswith('rclpy.'):
            touched.append(name)
            raise RosImportReached('ROS import reached; init/entities must remain unreachable in bad environment')
        return original_import(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',guarded_import)
    return output,overlay,touched


GOOD={'ROS_DOMAIN_ID':'117','ROS_LOCALHOST_ONLY':'1','ROS_DISTRO':'humble'}

@pytest.mark.parametrize('environment',[
    {},dict(GOOD,ROS_DOMAIN_ID='218'),dict(GOOD,ROS_LOCALHOST_ONLY='0'),dict(GOOD,ROS_DISTRO='jazzy'),
    {k:v for k,v in GOOD.items() if k!='ROS_DOMAIN_ID'},
    {k:v for k,v in GOOD.items() if k!='ROS_LOCALHOST_ONLY'},
    {k:v for k,v in GOOD.items() if k!='ROS_DISTRO'},
])
def test_bad_environment_rejected_before_ros_or_output_creation(tmp_path,monkeypatch,environment):
    output,overlay,touched=setup_main(tmp_path,monkeypatch,environment)
    with pytest.raises(RuntimeError,match='required ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 ROS_DISTRO=humble'):
        nav.main()
    # ROS import被边界替身拦截；未触达import即不可能init、建publisher或ActionClient。
    assert touched==[] and not output.exists() and not overlay.parent.exists()


def test_correct_environment_keeps_original_ros_startup_path(tmp_path,monkeypatch):
    output,overlay,touched=setup_main(tmp_path,monkeypatch,GOOD)
    with pytest.raises(RosImportReached): nav.main()
    assert touched==['rclpy'] and output.is_dir() and overlay.parent.is_dir()


def test_domain218_executor_factory_is_independent_of_main_environment_gate(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','218')
    class LegacyBase:
        def _take_subscription(self,sub): pass
        async def _execute_subscription(self,sub,msg): pass
    async def await_callback(callback,*args): return callback(*args)
    factory=nav.message_info_executor_type(LegacyBase,await_callback)
    assert issubclass(factory,LegacyBase)
