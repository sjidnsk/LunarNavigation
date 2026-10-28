"""本任务证据格式；通过唯一模块名加载既有只读证据工具。"""
import importlib.util
from pathlib import Path
def legacy(name):
    path=Path(__file__).resolve().parent.parent/'p3_path_validation'/(name+'.py')
    spec=importlib.util.spec_from_file_location('task3_legacy_'+name,path)
    module=importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    return module
common=legacy('common')
Recorder=common.Recorder
TOPICS=common.TOPICS
path_data=common.path_data
yaw_of=common.yaw_of
CMD='/nav1cm_integration/cmd_vel'
DIAG='/nav1cm_integration/diagnostics'
DURATIONS=dict(forward_spin=45.,reverse=20.,obstacle=48.,mapchange=34.,empty_smoke=12.)
def obstacles(case,t):
    if case=='obstacle':
        return [(1.2,-2.,1.5,.2)]
    if case=='mapchange' and 7.<t<13.:
        return [(1.8,-4.,2.1,4.)]
    return []
def goal(case):
    return {'forward_spin':(1.4,0.,.6),'reverse':(-.8,0.,0.),
            'obstacle':(2.6,1.4,0.),'mapchange':(2.6,0.,0.),'empty_smoke':(.8,0.,0.)}[case]
