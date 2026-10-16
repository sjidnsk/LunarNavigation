#!/usr/bin/env bash
set -euo pipefail
TRIAL_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export DRL_CACHE=${DRL_CACHE:-/home/kai/.cache/lunar-drl-task-frontier/jazzy-task8-20261004}
if [[ ${1:-} == monitor ]]; then
  shift
  exec bash "$TRIAL_SCRIPT_DIR/task_frontier/run.sh" python "$TRIAL_SCRIPT_DIR/course_two_trial_dashboard.py" "$@"
fi
exec bash "$TRIAL_SCRIPT_DIR/task_frontier/run.sh" python -m lunar_drl_exploration.course_two_trial "$@"
