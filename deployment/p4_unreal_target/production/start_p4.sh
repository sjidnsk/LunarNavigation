#!/usr/bin/env bash
# 正式入口固定使用P4，复用既有导航启动实现。
unset P4_ROOT LUNAR_P4_ROOT
exec /bin/bash /home/yanfa/P4/source/scripts/simulation/start_p4.sh "$@"
