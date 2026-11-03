#!/usr/bin/env bash
# 导航和空闲探索前台常驻；P3/Env_X由既有进程提供。
set -e
root="${P4_ROOT:-/home/yanfa/P4}"
rviz=true
case "${1:-}" in
  '') ;;
  --no-rviz) rviz=false; shift ;;
  *) echo "用法: bash start_p4.sh [--no-rviz]" >&2; exit 2 ;;
esac
if (( $# )); then echo "不支持多余参数" >&2; exit 2; fi
if [[ ! -f "$root/config/p4_environment.sh" ]]; then
  echo "缺少P4环境: $root/config/p4_environment.sh" >&2; exit 1
fi
# 文件锁由前台exec继承，进程退出即释放；不改变导航任务语义。
mkdir -p "$root/run"
exec 9>"$root/run/p4-start.lock"
if ! flock -n 9; then
  echo "P4启动入口已经运行，请使用现有会话" >&2; exit 1
fi
source "$root/config/p4_environment.sh"
python="$root/.venv/bin/python"
source_root="${P4_SOURCE_DIR:-$root/source}"
if [[ -f "$root/run/p4-current.json" ]]; then
  "$python" - "$root/run/p4-current.json" "$root" <<'PY'
import json
import os
from pathlib import Path
import sys

root=Path(sys.argv[2]).resolve()
try:
    current=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
    pid=int(current['operator_pid'])
    if pid<=0:raise ValueError('invalid operator_pid')
except (OSError,ValueError,KeyError,TypeError) as exc:
    sys.exit('P4当前运行记录无法解析，请先核对: '+str(exc))
try:
    proc=Path('/proc')/str(pid)
    state=proc.joinpath('stat').read_text().rsplit(')',1)[1].split()[0]
    command=[value.decode() for value in proc.joinpath('cmdline').read_bytes().split(b'\0') if value]
except FileNotFoundError:
    command=[];state='X'
except (OSError,ValueError,IndexError,UnicodeError) as exc:
    sys.exit('无法核对P4运行身份: '+str(exc))
def argument(name):
    for index,value in enumerate(command):
        if value==name and index+1<len(command):return command[index+1]
        if value.startswith(name+'='):return value[len(name)+1:]
    return None
runtimes=[value for value in command if Path(value).as_posix().endswith('/scripts/simulation/p4_tcp_runtime.py')
          and Path(value).is_absolute() and Path(value).resolve().is_relative_to(root)]
root_arg=argument('--root') or str(Path('/home/yanfa/P4').resolve());artifact=argument('--artifact')
matching=(bool(runtimes) and Path(root_arg).is_absolute() and Path(root_arg).resolve()==root
          and artifact is not None and Path(artifact).is_absolute() and Path(artifact).resolve().is_relative_to(root/'run'))
if state not in ('Z','X') and matching:
    sys.exit('P4已经运行，PID '+str(pid)+'，运行目录 '+str(artifact))
PY
fi
artifact="$root/run/p4-$(date +%Y%m%d-%H%M%S)-$$"
args=(--root "$root" --artifact "$artifact")
if [[ "$rviz" == true ]]; then
  args+=(--rviz-config "$source_root/scripts/simulation/p4_navigation.rviz")
fi
exec "$python" "$source_root/scripts/simulation/p4_tcp_runtime.py" "${args[@]}"
