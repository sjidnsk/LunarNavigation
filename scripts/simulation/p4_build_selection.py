"""原子选择已验证的整套P4构建；只用于部署，不连接ROS或控制车辆。"""
import argparse
import hashlib
import json
import os
from pathlib import Path


# 固定正式部署闭包的关键入口；完整清单另外覆盖全部源码和安装文件。
REQUIRED_FILES=frozenset([
    'environment.sh','install/local_setup.bash',
    'source/scripts/simulation/start_p4.sh',
    'source/scripts/simulation/p4_tcp_runtime.py',
    'source/scripts/simulation/p4_topics.py',
    'source/config/p4_topics.yaml','source/config/wheel.yaml',
    'source/config/tcp_wheel_speed.yaml','source/config/tcp_p3_feedback.yaml',
    'source/scripts/navigation/one_cm/nav1cm/execution.py',
    'source/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/runtime.py',
    'install/lunar_nav1cm_planner/lib/lunar_nav1cm_planner/navigation_task_node',
    'install/lunar_nav1cm_planner/lib/libnav1cm_terrain.so',
    'install/lunar_nav1cm_planner/lib/libnav1cm_geometry.so',
    'install/lunar_nav1cm_planner/lib/libnav1cm_distance_field.so',
    'install/lunar_incremental_navigation_core/lib/liblunar_incremental_navigation_core.so',
    'install/lunar_incremental_navigation_ros/lib/liblunar_incremental_navigation_ros.so',
    'install/lunar_pure_planner_core/lib/liblunar_pure_planner_core.so',
    'install/lunar_pure_exploration_core/lib/liblunar_pure_exploration_core.so',
    'install/lunar_pure_exploration_ros/lib/liblunar_pure_exploration_ros_incremental.so',
    'install/lunar_navigation_rviz/lib/liblunar_navigation_rviz.so'])


def validate_build(root,target):
    root,target=Path(root).resolve(),Path(target).resolve()
    if not target.is_relative_to(root/'build') or not target.name.startswith('formal-'):
        raise ValueError('build is outside P4 formal build area')
    manifest=json.loads((target/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('qualified') is not True:raise ValueError('build is not qualified')
    environment=target/'environment.sh'
    if hashlib.sha256(environment.read_bytes()).hexdigest()!=manifest.get('environment_sha256'):
        raise ValueError('environment digest mismatch')
    if not (target/'source').is_dir():raise ValueError('source missing')
    if manifest.get('mode')!='baseline' and not (target/'install/local_setup.bash').is_file():
        raise ValueError('install missing')
    files=manifest.get('files',{})
    baseline=manifest.get('mode')=='baseline' and target.name.startswith('formal-baseline-')
    if not baseline and (not isinstance(files,dict) or not REQUIRED_FILES.issubset(files)):
        raise ValueError('incomplete source/config/install/library manifest')
    for relative,expected in files.items():
        path=(target/relative).resolve()
        if not path.is_relative_to(target):raise ValueError('manifest file is outside build')
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError('file digest mismatch: '+relative)
    return target


def _live_operator(root):
    pointer=root/'run/p4-current.json'
    if not pointer.exists():return False
    record=json.loads(pointer.read_text(encoding='utf-8'))
    proc=Path('/proc')/str(int(record['operator_pid']))
    try:
        state=proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[0]
        command=[v.decode() for v in proc.joinpath('cmdline').read_bytes().split(b'\0') if v]
    except FileNotFoundError:return False
    if state in ('Z','X'):return False
    return any(v.endswith('/scripts/simulation/p4_tcp_runtime.py') and
        Path(v).resolve().is_relative_to(root) for v in command)


def switch_build(root,*,target=None,previous=False,check_only=False):
    root=Path(root).resolve()
    pointer=root/'config/p4_active_build.json'
    state=json.loads(pointer.read_text(encoding='utf-8')) if pointer.exists() else {}
    requested=state.get('previous') if previous else target
    if requested is None:raise ValueError('no previous build' if previous else 'target required')
    selected=validate_build(root,requested)
    if check_only:return dict(active=str(selected),previous=state.get('active'))
    import fcntl
    (root/'run').mkdir(exist_ok=True)
    with (root/'run/p4-start.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:raise RuntimeError('P4 is running; stop its existing session first') from exc
        # 锁内重读，防止另一个切换者改变active/previous后仍使用旧状态。
        state=json.loads(pointer.read_text(encoding='utf-8')) if pointer.exists() else {}
        if previous:
            if not state.get('previous'):raise ValueError('no previous build')
            selected=validate_build(root,state['previous'])
        if _live_operator(root):raise RuntimeError('P4 operator is still running')
        if state.get('active')==str(selected):return state
        updated=dict(active=str(selected),previous=state.get('active'))
        pointer.parent.mkdir(exist_ok=True)
        temporary=pointer.with_name(pointer.name+'.'+str(os.getpid())+'.tmp')
        try:
            with temporary.open('x',encoding='utf-8',newline='\n') as output:
                json.dump(updated,output,indent=2);output.write('\n');output.flush();os.fsync(output.fileno())
            os.replace(temporary,pointer)
        except BaseException:
            if temporary.exists():temporary.unlink()  # 仅删除本次创建的单个临时文件。
            raise
        return updated


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('/home/yanfa/P4'))
    choice=parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--target',type=Path)
    choice.add_argument('--previous',action='store_true')
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    try:result=switch_build(args.root,target=args.target,previous=args.previous,check_only=args.check_only)
    except (ValueError,OSError,RuntimeError,KeyError) as exc:parser.exit(1,str(exc)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
