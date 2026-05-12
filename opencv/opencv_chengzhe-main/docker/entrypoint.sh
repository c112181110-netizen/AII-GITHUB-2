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

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  safe_source /opt/ros/jazzy/setup.bash
fi

if [[ -f /workspace/OpenCV_ROS/install/setup.bash ]]; then
  safe_source /workspace/OpenCV_ROS/install/setup.bash
fi

cd /workspace/OpenCV_ROS
exec "$@"
