#!/usr/bin/env bash
# Build the ROS 1 catkin workspace in Release mode.
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${ROS_VERSION:-}" != "1" ]]; then
  echo "ROS 1 is not sourced. Run: source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

cd "${repository_root}/ros1_ws"
catkin_make -DCMAKE_BUILD_TYPE=Release
