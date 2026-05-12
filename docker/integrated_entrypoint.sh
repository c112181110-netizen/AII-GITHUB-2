#!/usr/bin/env bash
set -euo pipefail

safe_source() {
  local setup_file="$1"
  local had_nounset=0

  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "${setup_file}"

  if [[ ${had_nounset} -eq 1 ]]; then
    set -u
  fi
}

safe_source /opt/ros/jazzy/setup.bash

if [[ -f /workspace/integrated_ws/install/setup.bash ]]; then
  safe_source /workspace/integrated_ws/install/setup.bash
fi

export PYTHONPATH="/workspace/ros:/workspace/OpenCV_ROS:${PYTHONPATH:-}"

exec "$@"
