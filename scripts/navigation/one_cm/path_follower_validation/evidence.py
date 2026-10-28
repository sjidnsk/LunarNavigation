"""本任务证据格式；通过唯一模块名加载既有只读证据工具。"""
import importlib.util
from pathlib import Path
def legacy(name):
    import importlib
    import sys
    package = 'nav1cm_p3_path_validation'
    folder = Path(__file__).resolve().parent.parent/'p3_path_validation'
    if package not in sys.modules:
        spec=importlib.util.spec_from_file_location(package,folder/'__init__.py',
            submodule_search_locations=[str(folder)])
        module=importlib.util.module_from_spec(spec)
        sys.modules[package]=module
        spec.loader.exec_module(module)
    return importlib.import_module(package+'.'+name)
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

def production_modules(names):
    return [n for n in names if n in ('nav1cm','lunar_pure_wheeled_controller','lunar_nav1cm_planner') or
            n.startswith(('nav1cm.','lunar_pure_wheeled_controller.','lunar_nav1cm_planner.'))]

def executor_context(executor,dt,kwargs):
    grid=executor.map
    return dict(dt=dt,now_s=kwargs.get('now_s'),state_age_s=kwargs.get('state_age_s',0.),
        phase=executor.phase,failure=executor.failure,last_command=executor._last_command.tolist(),
        command_history=[(t,u.tolist()) for t,u in executor._command_history[-12:]],
        map=None if grid is None else dict(origin=grid.origin,resolution=grid.resolution,
            revision=grid.revision,epoch=grid.epoch))
