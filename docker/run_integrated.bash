#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-doctor}"

usage() {
  cat <<'EOF'
Usage:
  bash /run_integrated.bash [doctor|opencv-bridge|moveit|controller-dry-run|controller-once|all-dry-run]

Modes:
  doctor             Check ROS/OpenCV/MoveIt Python modules and built packages.
  opencv-bridge      Run Windows TCP camera sender -> ROS image/point bridge on port 5001.
  moveit             Run PhantomX MoveIt move_group for /compute_ik.
  controller-dry-run Run OpenCV topic -> IK -> AX conversion without opening CM530 serial.
  controller-once    Pull one OpenCV service point -> IK -> AX conversion without serial.
  all-dry-run        Start OpenCV bridge + MoveIt + dry-run controller in one container.

Notes:
  Docker Desktop on Windows usually needs -p 5001:5001 for the camera sender.
  Real CM530 COM4 access from Linux containers is not automatic on Docker Desktop.
  Use dry-run in-container first; run real CM530 motion from a host/WSL environment that can see COM4.
EOF
}

doctor() {
  echo "[integrated] ROS_DISTRO=${ROS_DISTRO:-}"
  echo "[integrated] Python=$(command -v python3)"
  python3 --version

  local modules=(
    rclpy
    geometry_msgs.msg
    sensor_msgs.msg
    moveit_msgs.srv
    cv_bridge
    cv2
    numpy
    serial
  )

  local module
  for module in "${modules[@]}"; do
    python3 -c "import importlib; importlib.import_module('${module}')" \
      && echo "[doctor] ${module}: OK" \
      || { echo "[doctor] ${module}: MISSING"; return 1; }
  done

  ros2 pkg prefix opencv_ros2_bridge_interfaces >/dev/null
  echo "[doctor] opencv_ros2_bridge_interfaces: OK"
  ros2 pkg prefix phantomx_pincher_moveit_config >/dev/null
  echo "[doctor] phantomx_pincher_moveit_config: OK"
  ros2 interface show opencv_ros2_bridge_interfaces/srv/GetObjectPoint >/dev/null
  echo "[doctor] GetObjectPoint service type: OK"
}

run_opencv_bridge() {
  cd /workspace/OpenCV_ROS
  exec python3 windows_stream_bridge_publisher.py --ros-args \
    -p listen_host:=0.0.0.0 \
    -p listen_port:=5001 \
    -p frame_id:=phantomx_pincher_arm_base_link
}

run_moveit() {
  exec ros2 launch /integrated_move_group.launch.py \
    log_level:=info
}

run_controller_dry() {
  exec python3 /workspace/ros/opencv_to_cm530_node.py \
    --dry-run \
    --continuous \
    --no-tf \
    --vision-map test06 \
    --vision-gain-xy 1.0,0.0 \
    --vision-y-limits 0.0,0.0 \
    --vision-z-limits 0.05,0.12 \
    --target-frame phantomx_pincher_arm_base_link \
    --service-timeout-sec 10
}

run_controller_once() {
  exec python3 /workspace/ros/opencv_to_cm530_node.py \
    --dry-run \
    --once \
    --no-tf \
    --vision-map test06 \
    --vision-gain-xy 1.0,0.0 \
    --vision-y-limits 0.0,0.0 \
    --vision-z-limits 0.05,0.12 \
    --target-frame phantomx_pincher_arm_base_link \
    --service-timeout-sec 10
}

run_all_dry() {
  echo "[integrated] Starting OpenCV bridge..."
  python3 /workspace/OpenCV_ROS/windows_stream_bridge_publisher.py --ros-args \
    -p listen_host:=0.0.0.0 \
    -p listen_port:=5001 \
    -p frame_id:=phantomx_pincher_arm_base_link &
  OPENCV_PID=$!

  echo "[integrated] Starting MoveIt..."
  ros2 launch /integrated_move_group.launch.py \
    log_level:=info &
  MOVEIT_PID=$!

  cleanup() {
    kill "${OPENCV_PID}" "${MOVEIT_PID}" 2>/dev/null || true
  }
  trap cleanup EXIT

  echo "[integrated] Waiting for /compute_ik..."
  deadline=$((SECONDS + 60))
  until ros2 service list | grep -qx "/compute_ik"; do
    if (( SECONDS >= deadline )); then
      echo "[integrated][error] /compute_ik did not appear in time" >&2
      exit 1
    fi
    sleep 1
  done

  echo "[integrated] Starting dry-run controller. Click Windows preview to send points."
  python3 /workspace/ros/opencv_to_cm530_node.py \
    --dry-run \
    --continuous \
    --no-tf \
    --vision-map test06 \
    --vision-gain-xy 1.0,0.0 \
    --vision-y-limits 0.0,0.0 \
    --vision-z-limits 0.05,0.12 \
    --target-frame phantomx_pincher_arm_base_link \
    --service-timeout-sec 10
}

case "${MODE}" in
  -h|--help|help)
    usage
    ;;
  doctor)
    doctor
    ;;
  opencv-bridge)
    run_opencv_bridge
    ;;
  moveit)
    run_moveit
    ;;
  controller-dry-run)
    run_controller_dry
    ;;
  controller-once)
    run_controller_once
    ;;
  all-dry-run)
    run_all_dry
    ;;
  *)
    usage
    echo "[integrated][error] Unsupported mode: ${MODE}" >&2
    exit 2
    ;;
esac
