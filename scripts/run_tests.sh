#!/usr/bin/env bash
# Run Python tests and ROS C++ tests when the catkin environment is available.
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repository_root}"
PYTHONPATH=src pytest -q tests/python

if command -v catkin_make >/dev/null 2>&1; then
  cd ros1_ws
  catkin_make run_tests_ecbf_franka_controller
fi
