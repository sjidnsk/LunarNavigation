#!/usr/bin/env bash
# P3 与平台 TF 由外部输入；复用轮式正式启动入口。
set -euo pipefail
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repository_root/scripts/orin/p3_environment.sh"
mode="${1:-${LUNAR_NAVIGATION_MODE:-nav1cm}}"
extra_args=()
if (( $# > 1 )); then extra_args=("${@:2}"); fi
config_file=""
if [[ "$mode" == incremental_v2 ]]; then config_file="$repository_root/config/orin_hardware.yaml"; fi
exec bash "$repository_root/scripts/orin/start_navigation.sh" legged false "$config_file" false "$mode" "${extra_args[@]}"
