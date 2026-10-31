"""有界Path/元信息乱序配对；提供请求不等于执行接受。"""
from collections import OrderedDict,deque
import math


class ExecutionInbox:
    def __init__(self,capacity=32,max_age_s=.35):
        if type(capacity) is not int or capacity<1 or (max_age_s is not None and (not math.isfinite(max_age_s) or max_age_s<=0)):
            raise ValueError('invalid execution pairing bounds')
        self.capacity=capacity;self.max_age_s=max_age_s
        self.paths=OrderedDict();self.references=OrderedDict()
        self.session=None;self.revision=0;self.stamp_ns=0;self.retired=deque(maxlen=128)

    def _trim(self,mapping):
        while len(mapping)>self.capacity:mapping.popitem(last=False)

    def add_path(self,stamp_ns,digest,parsed,received_s):
        self.paths[(stamp_ns,digest)]=(parsed,received_s);self._trim(self.paths)

    def add_reference(self,reference,received_s):
        if reference.session_id in self.retired or reference.stamp_ns<=self.stamp_ns:return False
        if reference.session_id==self.session and reference.revision<=self.revision:return False
        self.references[(reference.stamp_ns,reference.path_digest)]=(reference,received_s)
        self._trim(self.references);return True

    def pop_ready(self,now_s):
        if not math.isfinite(now_s):return None
        for mapping in (self.paths,self.references):
            for key,(_,received) in list(mapping.items()):
                if now_s<received or (self.max_age_s is not None and now_s-received>self.max_age_s):mapping.pop(key)
        ready=[(key,value) for key,value in self.references.items() if key in self.paths]
        ready.sort(key=lambda item:item[0][0],reverse=True)
        for key,(ref,received) in ready:
            if ref.session_id in self.retired or ref.stamp_ns<=self.stamp_ns:continue
            if ref.session_id==self.session and ref.revision<=self.revision:continue
            parsed,path_received=self.paths[key]
            if self.session is not None and ref.session_id!=self.session:self.retired.append(self.session)
            self.session=ref.session_id;self.revision=ref.revision;self.stamp_ns=ref.stamp_ns
            for mapping in (self.paths,self.references):
                for old in list(mapping):
                    if old[0]<=self.stamp_ns:mapping.pop(old)
            return ref,parsed,min(path_received,received)
        return None
