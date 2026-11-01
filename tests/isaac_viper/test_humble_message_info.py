"""针对实机旧Humble单参executor的显示订阅适配；不init ROS、不替代DDS验证。"""
import asyncio
from types import SimpleNamespace
from tools.isaac_viper import navigation_showcase as nav


class Handle:
    def __init__(self,value): self.value=value
    def __enter__(self): return self
    def __exit__(self,*args): return False
    def take_message(self,msg_type,raw): return self.value


async def await_callback(callback,*args):
    result=callback(*args)
    if hasattr(result,'__await__'): return await result
    return result


class OldHumbleSingleArgumentExecutor:
    def __init__(self,**kwargs): pass
    def _take_subscription(self,sub):
        with sub.handle:
            msg_info=sub.handle.take_message(sub.msg_type,sub.raw)
            return msg_info[0] if msg_info is not None else None
    async def _execute_subscription(self,sub,msg):
        if msg: await await_callback(sub.callback,msg)


def subscription(message,info,callback):
    return SimpleNamespace(handle=Handle((message,info)),msg_type=object,raw=False,callback=callback)


def test_info_adapter_preserves_true_take_message_info_and_normal_callbacks():
    assert hasattr(nav,'message_info_executor_type'),'old Humble executor compatibility missing'
    adapter=nav.message_info_executor_type(OldHumbleSingleArgumentExecutor,await_callback)()
    message=object(); info={'source_timestamp':1234567890123,'received_timestamp':999}
    received=[]
    tagged=subscription(message,info,lambda msg,metadata:received.append((msg,metadata)))
    adapter.preserve_message_info(tagged)
    taken=adapter._take_subscription(tagged)
    assert taken[0] is message and taken[1] is info
    asyncio.run(adapter._execute_subscription(tagged,taken))
    assert received==[(message,info)] and received[0][1] is info
    normal=[]; untagged=subscription(message,info,lambda msg:normal.append(msg))
    taken=adapter._take_subscription(untagged)
    assert taken is message
    asyncio.run(adapter._execute_subscription(untagged,taken))
    assert normal==[message]
    tagged.handle.value=None
    assert adapter._take_subscription(tagged) is None
    asyncio.run(adapter._execute_subscription(tagged,None))
    assert received==[(message,info)]


def test_source_timestamp_dict_object_and_missing_info_never_use_wall_time():
    assert hasattr(nav,'dds_source_timestamp'),'true DDS metadata extraction missing'
    assert nav.dds_source_timestamp({'source_timestamp':1234567})==1234567
    assert nav.dds_source_timestamp(SimpleNamespace(source_timestamp=7654321))==7654321
    for info in (None,{},SimpleNamespace(),{'source_timestamp':True},{'source_timestamp':-1},{'source_timestamp':'123'}):
        assert nav.dds_source_timestamp(info)==0
    display=nav.DisplayPlanSession(); display.begin_goal('a'*32,100)
    assert not display.observe_global(dict(stamp_ns=0,points=[[0,0,0],[1,0,0]]),nav.dds_source_timestamp({}))


def test_info_adapter_awaits_async_callback_and_info_object_unchanged():
    assert hasattr(nav,'message_info_executor_type'),'old Humble executor compatibility missing'
    adapter=nav.message_info_executor_type(OldHumbleSingleArgumentExecutor,await_callback)()
    received=[]; metadata=SimpleNamespace(source_timestamp=1001)
    async def callback(msg,info): received.append((msg,info))
    sub=subscription('original-message',metadata,callback); adapter.preserve_message_info(sub)
    asyncio.run(adapter._execute_subscription(sub,adapter._take_subscription(sub)))
    assert received==[('original-message',metadata)] and received[0][1] is metadata
