#!/usr/bin/env bash
set -eo pipefail
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repository_root/scripts/orin/p3_environment.sh"
source /opt/ros/humble/setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
runtime_dir="${P4_RUNTIME_DIR:-$repository_root/deployment-evidence/static-tf-runtime}"
mkdir -p "$runtime_dir/log" "$runtime_dir/tmp"
export ROS_LOG_DIR="$runtime_dir/log" TMPDIR="$runtime_dir/tmp"
exec flock -n "$runtime_dir/adapter-domain-$ROS_DOMAIN_ID.lock" \
  /usr/bin/python3 "$repository_root/scripts/orin/static_tf_input.py" "$@"
