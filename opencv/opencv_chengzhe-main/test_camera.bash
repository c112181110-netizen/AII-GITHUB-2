#!/usr/bin/env bash

# 攝像頭功能測試啟動器

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-menu}"
PYTHON_BIN="python3"

log() {
  echo "[camera-test] $*"
}

warn() {
  echo "[camera-test][warn] $*" >&2
}

fail() {
  echo "[camera-test][error] $*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  bash test_camera.bash [command]

Commands:
  menu              交互式菜單（默認）
  opencv            測試 OpenCV 功能
  simulator         啟動模擬攝像頭發佈者
  subscriber        啟動攝像頭訂閱者
  both              同時運行發佈者和訂閱者
  ros-test          ROS 2 完整測試
  help              顯示此幫助信息

Examples:
  bash test_camera.bash
  bash test_camera.bash opencv
  bash test_camera.bash simulator
  bash test_camera.bash both
EOF
}

# 來自 run.bash 的函數
source_ros_env() {
  if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
    log "Using current ROS environment: ${ROS_DISTRO}"
  else
    # 查找 ROS setup.bash
    local preferred=(jazzy humble iron rolling galactic foxy)
    local distro
    for distro in "${preferred[@]}"; do
      if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
        source "/opt/ros/${distro}/setup.bash"
        log "Sourced ROS: /opt/ros/${distro}/setup.bash"
        return 0
      fi
    done
    fail "Cannot find ROS setup.bash"
  fi
}

test_opencv() {
  log "Testing OpenCV..."
  "${PYTHON_BIN}" "${SCRIPT_DIR}/test_camera.py"
}

launch_simulator() {
  log "啟動模擬攝像頭 (Ctrl+C 停止)..."
  source_ros_env
  "${PYTHON_BIN}" "${SCRIPT_DIR}/mock_camera_publisher.py"
}

launch_subscriber() {
  log "啟動攝像頭訂閱者 (Ctrl+C 停止)..."
  source_ros_env
  
  # 如果是 camera_point_cv_subscriber.py，假設它已配置正確
  if [[ -f "${SCRIPT_DIR}/camera_point_cv_subscriber.py" ]]; then
    "${PYTHON_BIN}" "${SCRIPT_DIR}/camera_point_cv_subscriber.py"
  else
    fail "camera_point_cv_subscriber.py not found"
  fi
}

launch_both() {
  log "啟動模擬和訂閱者..."
  log "模擬攝像頭將在後台運行，訂閱者將在前台運行"
  
  source_ros_env
  
  # 在後台啟動模擬攝像頭
  log "Starting simulator in background..."
  "${PYTHON_BIN}" "${SCRIPT_DIR}/mock_camera_publisher.py" &
  SIMULATOR_PID=$!
  log "Simulator PID: ${SIMULATOR_PID}"
  
  # 等待一秒讓模擬器啟動
  sleep 2
  
  # 啟動訂閱者
  log "Starting subscriber..."
  trap "kill ${SIMULATOR_PID}" EXIT
  
  "${PYTHON_BIN}" "${SCRIPT_DIR}/camera_point_cv_subscriber.py"
}

ros_complete_test() {
  log "Running complete ROS 2 camera test..."
  source_ros_env
  
  echo ""
  log "📋 檢查環境..."
  "${PYTHON_BIN}" -c "
import rclpy
print('✅ ROS 2 Python client ready')
"
  
  echo ""
  log "🎬 測試分為三部分（可選）:"
  echo "  1. 模擬攝像頭發佈者"
  echo "  2. 攝像頭訂閱者"
  echo "  3. OpenCV 功能"
  echo ""
  
  read -p "是否要測試？ (y/n): " -n 1 -r
  echo
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    launch_both
  fi
}

interactive_menu() {
  log "🎥 攝像頭功能測試菜單"
  echo ""
  echo "選擇測試項目:"
  echo "  1) OpenCV 功能測試"
  echo "  2) 啟動模擬攝像頭發佈者"
  echo "  3) 啟動攝像頭訂閱者"
  echo "  4) 模擬者 + 訂閱者 (完整測試)"
  echo "  5) ROS 2 完整測試"
  echo "  6) 顯示幫助"
  echo "  0) 退出"
  echo ""
  
  read -p "請選擇 (0-6): " -n 1 -r choice
  echo
  echo ""
  
  case "$choice" in
    1) test_opencv ;;
    2) launch_simulator ;;
    3) launch_subscriber ;;
    4) launch_both ;;
    5) ros_complete_test ;;
    6) usage ;;
    0) log "已退出"; exit 0 ;;
    *) warn "無效選擇"; interactive_menu ;;
  esac
}

case "${TARGET}" in
  -h|--help|help)
    usage
    exit 0
    ;;
  menu)
    interactive_menu
    ;;
  opencv)
    test_opencv
    ;;
  simulator)
    launch_simulator
    ;;
  subscriber)
    launch_subscriber
    ;;
  both)
    launch_both
    ;;
  ros-test)
    ros_complete_test
    ;;
  *)
    usage
    fail "Unknown command: ${TARGET}"
    ;;
esac
