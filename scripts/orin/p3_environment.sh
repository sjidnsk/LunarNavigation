#!/usr/bin/env bash
# Foot P3 start_dog.sh uses domain 69; explicit domain/DDS overrides stay authoritative.
p4_environment_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${P3_DOG_DOMAIN:-69}}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$p4_environment_root/config/cyclonedds_p3.xml}"
