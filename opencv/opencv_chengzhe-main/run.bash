#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-auto}"
PYTHON_BIN=""

usage() {
  cat <<'EOF'
Usage:
  bash run.bash [auto|main|cv|cv-view|mock-cv|win-bridge|win-cv|win-coord|doctor|test-camera]

Modes:
  auto         Auto-select a runnable node for this machine (default)
  main         Run main.py
  cv           Run local USB camera publisher + viewer
  cv-view      Run viewer only (expects /camera/image_raw publisher)
  mock-cv      Run with simulated camera (ROS 2 mock + viewer)
  win-bridge   Run TCP bridge for Windows camera sender -> /camera/image_raw
  win-cv       Start Windows bridge + OpenCV viewer (auto-launch sender terminal)
  win-coord    Coordinate-only mode (no ROS publish, no WSL OpenCV viewer)
  test-camera  Launch camera testing suite
  doctor       Print environment/module checks only

Examples:
  bash run.bash
  bash run.bash main
  bash run.bash cv
  bash run.bash cv-view
  bash run.bash mock-cv
  bash run.bash win-bridge
  bash run.bash win-cv
  bash run.bash win-coord
  bash run.bash test-camera
  bash run.bash doctor

Windows camera tuning env vars (for win-cv / win-coord):
  WIN_CAMERA_FPS     Capture FPS target (default: 30)
  WIN_SEND_HZ        Max TCP send rate (default: 30)
  WIN_JPEG_QUALITY   JPEG quality 1-100 (default: 80)
  WIN_CAMERA_WIDTH   Capture width (default: 960)
  WIN_CAMERA_HEIGHT  Capture height (default: 540)
  WIN_SENDER_PREVIEW 1/0: show sender preview window on Windows (default: 1)
  WIN_DETECT_BLACK   1/0: detect black object and publish realtime point (default: 0)
  WIN_BLACK_DETECT_HZ Max detect-point publish rate (default: 8)
  WIN_BLACK_V_MAX    HSV V upper bound for black detection (default: 55)
  WIN_MIN_BLACK_AREA Min contour area for black target (default: 600)
  WIN_POINT_SCALE_PX_PER_METER Pixel-to-meter scale used by bridge (default: 100.0)
  WIN_POINT_DEFAULT_Z_M Default output z value in meters (default: 0.0)
  WIN_XYZ_DECIMALS    Decimals for coordinate tuple output (default: 1)
  WIN_COORD_FRAME_ID  Output frame label for coordinate mode (default: base_link)
  WIN_COORD_AXIS_MAPPING Coordinate conversion mode (default: rep103_base_link)
  WIN_COORD_ONLY_MODE 1/0: make win-cv route to coordinate-only mode (default: 1)
  WIN_SENDER_COORD_ONLY 1/0: sender skips JPEG frames and sends only coordinates (default: 1)
EOF
}

log() {
  echo "[run] $*"
}

warn() {
  echo "[run][warn] $*" >&2
}

fail() {
  echo "[run][error] $*" >&2
  exit 1
}

safe_source() {
  local setup_file="$1"
  local had_nounset=0

  # Some ROS setup scripts reference optional env vars and are not strict-set safe.
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

pick_python() {
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python)"
  else
    fail "Python is not installed."
  fi
}

discover_ros_setup() {
  if [[ -n "${ROS_SETUP:-}" && -f "${ROS_SETUP}" ]]; then
    echo "${ROS_SETUP}"
    return 0
  fi

  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "/opt/ros/${ROS_DISTRO}/setup.bash"
    return 0
  fi

  local preferred=(jazzy humble iron rolling galactic foxy)
  local distro
  for distro in "${preferred[@]}"; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      echo "/opt/ros/${distro}/setup.bash"
      return 0
    fi
  done

  local detected
  detected="$(find /opt/ros -mindepth 2 -maxdepth 2 -type f -name setup.bash 2>/dev/null | sort | tail -n 1 || true)"
  if [[ -n "${detected}" ]]; then
    echo "${detected}"
    return 0
  fi

  return 1
}

source_ros_env() {
  if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
    log "Using current ROS environment: ${ROS_DISTRO}"
  else
    local ros_setup
    ros_setup="$(discover_ros_setup || true)"
    if [[ -z "${ros_setup}" ]]; then
      fail "Cannot find ROS setup.bash. Install ROS 2 or set ROS_SETUP=/path/to/setup.bash"
    fi

    safe_source "${ros_setup}"
    log "Sourced ROS: ${ros_setup}"
  fi

  local overlay_setup="${SCRIPT_DIR}/install/setup.bash"
  if [[ -f "${overlay_setup}" ]]; then
    safe_source "${overlay_setup}"
    log "Sourced workspace overlay: ${overlay_setup}"
  fi
}

has_display() {
  [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" || -n "${WSL2_GUI_APPS_ENABLED:-}" ]]
}

camera_device_exists() {
  compgen -G "/dev/video*" >/dev/null
}

topic_has_publishers() {
  local topic_info
  topic_info="$(ros2 topic info /camera/image_raw 2>/dev/null || true)"
  [[ "${topic_info}" =~ Publisher\ count:\ [1-9][0-9]* ]]
}

tcp_port_in_use() {
  local port="$1"

  if command -v ss >/dev/null 2>&1; then
    ss -ltn "sport = :${port}" 2>/dev/null | grep -q LISTEN
    return $?
  fi

  if command -v netstat >/dev/null 2>&1; then
    netstat -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${port}$"
    return $?
  fi

  return 1
}

pick_free_tcp_port() {
  local start_port="$1"
  local span="${2:-50}"
  local port

  for ((port=start_port; port<start_port+span; port++)); do
    if ! tcp_port_in_use "${port}"; then
      echo "${port}"
      return 0
    fi
  done

  return 1
}

check_modules() {
  local missing=()
  local module
  for module in "$@"; do
    if ! "${PYTHON_BIN}" -c "import importlib; importlib.import_module('${module}')" >/dev/null 2>&1; then
      missing+=("${module}")
    fi
  done

  if (( ${#missing[@]} > 0 )); then
    warn "Missing Python modules: ${missing[*]}"
    if printf '%s\n' "${missing[@]}" | grep -qx "cv2"; then
      warn "Install OpenCV Python, for example: sudo apt install python3-opencv"
    fi
    if printf '%s\n' "${missing[@]}" | grep -Eq '^(rclpy|geometry_msgs.msg|sensor_msgs.msg|std_msgs.msg|tf2_ros|tf2_geometry_msgs|cv_bridge)$'; then
      warn "ROS 2 Python modules are missing. Ensure ROS is installed and setup.bash is sourced."
    fi
    return 1
  fi

  return 0
}

doctor() {
  log "Workspace: ${SCRIPT_DIR}"
  log "Python: ${PYTHON_BIN}"
  "${PYTHON_BIN}" --version

  if [[ -n "${ROS_DISTRO:-}" ]]; then
    log "ROS_DISTRO=${ROS_DISTRO}"
  else
    warn "ROS_DISTRO is empty"
  fi

  local mod
  local all_ok=1
  for mod in rclpy geometry_msgs.msg sensor_msgs.msg std_msgs.msg tf2_ros tf2_geometry_msgs cv_bridge numpy cv2; do
    if "${PYTHON_BIN}" -c "import importlib; importlib.import_module('${mod}')" >/dev/null 2>&1; then
      echo "[doctor] ${mod}: OK"
    else
      echo "[doctor] ${mod}: MISSING"
      all_ok=0
    fi
  done

  if [[ ${all_ok} -eq 1 ]]; then
    log "All required modules are available."
    return 0
  fi

  warn "Some modules are missing."
  return 1
}

run_main() {
  check_modules rclpy geometry_msgs.msg sensor_msgs.msg trajectory_msgs.msg tf2_ros tf2_geometry_msgs
  exec "${PYTHON_BIN}" "${SCRIPT_DIR}/main.py"
}

run_cv_view() {
  if ! has_display; then
    fail "No GUI display detected. Use 'main' mode or run in a desktop session for OpenCV window output."
  fi
  check_modules rclpy sensor_msgs.msg std_msgs.msg geometry_msgs.msg cv_bridge numpy cv2
  exec "${PYTHON_BIN}" "${SCRIPT_DIR}/camera_point_cv_subscriber.py"
}

run_cv() {
  if ! has_display; then
    fail "No GUI display detected. Cannot run local USB camera viewer."
  fi

  check_modules rclpy sensor_msgs.msg std_msgs.msg geometry_msgs.msg cv_bridge numpy cv2

  if [[ ! -f "${SCRIPT_DIR}/usb_camera_publisher.py" ]]; then
    fail "usb_camera_publisher.py not found"
  fi

  if ! camera_device_exists; then
    fail "No /dev/video* camera device found. For built-in Windows webcam, install usbipd-win and attach the camera to WSL first, or use 'mock-cv'."
  fi

  log "Starting USB camera publisher..."
  "${PYTHON_BIN}" "${SCRIPT_DIR}/usb_camera_publisher.py" &
  CAMERA_PID=$!

  sleep 2
  if ! kill -0 ${CAMERA_PID} 2>/dev/null; then
    fail "USB camera publisher failed to start"
  fi

  trap "kill ${CAMERA_PID} 2>/dev/null || true" EXIT

  log "USB camera publisher is running (PID: ${CAMERA_PID})"
  exec "${PYTHON_BIN}" "${SCRIPT_DIR}/camera_point_cv_subscriber.py"
}

run_mock_cv() {
  if ! has_display; then
    fail "No GUI display detected. Cannot run mock camera viewer."
  fi
  
  log "Starting mock camera demo..."
  log "  - Mock camera publisher will generate test patterns"
  log "  - Press 'q' in the window to stop"
  
  check_modules rclpy sensor_msgs.msg std_msgs.msg geometry_msgs.msg cv_bridge numpy cv2
  
  # Start mock publisher in background
  "${PYTHON_BIN}" "${SCRIPT_DIR}/mock_camera_publisher.py" &
  MOCK_PID=$!
  
  # Give it time to start
  sleep 2
  
  # Start viewer (foreground)
  if ! kill -0 $MOCK_PID 2>/dev/null; then
    fail "Mock camera publisher failed to start"
  fi
  
  trap "kill $MOCK_PID 2>/dev/null || true" EXIT
  
  log "Mock camera is running (PID: $MOCK_PID)"
  exec "${PYTHON_BIN}" "${SCRIPT_DIR}/camera_point_cv_subscriber.py"
}

launch_windows_sender_terminal() {
  local host="$1"
  local port="$2"
  local camera_index="$3"
  local camera_fps="$4"
  local sender_hz="$5"
  local camera_width="$6"
  local camera_height="$7"
  local jpeg_quality="$8"
  local sender_preview="$9"
  local detect_black="${10}"
  local black_detect_hz="${11}"
  local black_v_max="${12}"
  local min_black_area="${13}"
  local sender_coord_only="${14:-0}"

  if ! command -v cmd.exe >/dev/null 2>&1; then
    warn "cmd.exe is unavailable. Open Windows terminal manually to run windows_camera_ros_sender.py"
    return 1
  fi

  if ! command -v wslpath >/dev/null 2>&1; then
    warn "wslpath is unavailable. Open Windows terminal manually to run windows_camera_ros_sender.py"
    return 1
  fi

  local sender_bat
  sender_bat="$(wslpath -w "${SCRIPT_DIR}/start_windows_ros_sender.bat")"

  local -a sender_cmd=(
    cmd.exe /c start "" "${sender_bat}"
    --host "${host}"
    --port "${port}"
    --index "${camera_index}"
    --fps "${camera_fps}"
    --send-hz "${sender_hz}"
    --width "${camera_width}"
    --height "${camera_height}"
    --jpeg-quality "${jpeg_quality}"
  )

  case "${sender_preview,,}" in
    1|true|yes|on)
      sender_cmd+=(--preview)
      ;;
  esac

  case "${detect_black,,}" in
    1|true|yes|on)
      sender_cmd+=(
        --detect-black
        --black-detect-hz "${black_detect_hz}"
        --black-v-max "${black_v_max}"
        --min-black-area "${min_black_area}"
      )
      ;;
  esac

  case "${sender_coord_only,,}" in
    1|true|yes|on)
      sender_cmd+=(--coord-only)
      ;;
  esac

  "${sender_cmd[@]}" >/dev/null 2>&1
}

run_win_bridge() {
  local bridge_port="${WIN_BRIDGE_PORT:-5001}"
  local point_scale_px_per_meter="${WIN_POINT_SCALE_PX_PER_METER:-100.0}"

  if [[ "${point_scale_px_per_meter}" =~ ^[+-]?[0-9]+$ ]]; then
    point_scale_px_per_meter="${point_scale_px_per_meter}.0"
  fi

  check_modules rclpy sensor_msgs.msg std_msgs.msg geometry_msgs.msg numpy cv2

  if [[ ! -f "${SCRIPT_DIR}/windows_stream_bridge_publisher.py" ]]; then
    fail "windows_stream_bridge_publisher.py not found"
  fi

  if tcp_port_in_use "${bridge_port}"; then
    fail "TCP port ${bridge_port} is already in use. Stop the existing process or run: WIN_BRIDGE_PORT=<port> bash run.bash win-bridge"
  fi

  log "Starting Windows TCP stream bridge on 0.0.0.0:${bridge_port}"
  exec "${PYTHON_BIN}" "${SCRIPT_DIR}/windows_stream_bridge_publisher.py" --ros-args -p listen_port:="${bridge_port}" -p scale_px_per_meter:="${point_scale_px_per_meter}"
}

run_win_cv() {
  local win_coord_only_mode="${WIN_COORD_ONLY_MODE:-1}"
  case "${win_coord_only_mode,,}" in
    1|true|yes|on)
      log "WIN_COORD_ONLY_MODE enabled, routing win-cv -> win-coord"
      run_win_coord
      return
      ;;
  esac

  local preferred_port="${WIN_BRIDGE_PORT:-5001}"
  local bridge_port="${preferred_port}"
  local sender_host="127.0.0.1"
  local camera_index="${WIN_CAMERA_INDEX:-0}"
  local camera_fps="${WIN_CAMERA_FPS:-30}"
  local sender_hz="${WIN_SEND_HZ:-30}"
  local camera_width="${WIN_CAMERA_WIDTH:-960}"
  local camera_height="${WIN_CAMERA_HEIGHT:-540}"
  local jpeg_quality="${WIN_JPEG_QUALITY:-80}"
  local sender_preview="${WIN_SENDER_PREVIEW:-1}"
  local detect_black="${WIN_DETECT_BLACK:-0}"
  local black_detect_hz="${WIN_BLACK_DETECT_HZ:-8}"
  local black_v_max="${WIN_BLACK_V_MAX:-55}"
  local min_black_area="${WIN_MIN_BLACK_AREA:-600}"
  local point_scale_px_per_meter="${WIN_POINT_SCALE_PX_PER_METER:-100.0}"

  if [[ "${point_scale_px_per_meter}" =~ ^[+-]?[0-9]+$ ]]; then
    point_scale_px_per_meter="${point_scale_px_per_meter}.0"
  fi

  if ! has_display; then
    fail "No GUI display detected. Cannot run OpenCV viewer."
  fi

  check_modules rclpy sensor_msgs.msg std_msgs.msg geometry_msgs.msg numpy cv2

  if [[ ! -f "${SCRIPT_DIR}/windows_stream_bridge_publisher.py" ]]; then
    fail "windows_stream_bridge_publisher.py not found"
  fi

  if tcp_port_in_use "${bridge_port}"; then
    local fallback_port
    fallback_port="$(pick_free_tcp_port $((preferred_port + 1)) 50 || true)"
    if [[ -z "${fallback_port}" ]]; then
      fail "TCP port ${preferred_port} is busy and no fallback port was found. Stop stale bridge processes and retry."
    fi
    warn "Port ${preferred_port} is in use. Switching to ${fallback_port}."
    bridge_port="${fallback_port}"
  fi

  log "Starting Windows camera bridge in background on port ${bridge_port}..."
  "${PYTHON_BIN}" "${SCRIPT_DIR}/windows_stream_bridge_publisher.py" --ros-args -p listen_port:="${bridge_port}" -p scale_px_per_meter:="${point_scale_px_per_meter}" &
  BRIDGE_PID=$!

  sleep 2
  if ! kill -0 ${BRIDGE_PID} 2>/dev/null; then
    fail "Windows camera bridge failed to start"
  fi

  trap "kill ${BRIDGE_PID} 2>/dev/null || true" EXIT

  log "Bridge is running (PID: ${BRIDGE_PID})"
  log "Trying to launch Windows sender terminal..."
  if ! launch_windows_sender_terminal "${sender_host}" "${bridge_port}" "${camera_index}" "${camera_fps}" "${sender_hz}" "${camera_width}" "${camera_height}" "${jpeg_quality}" "${sender_preview}" "${detect_black}" "${black_detect_hz}" "${black_v_max}" "${min_black_area}" "0"; then
    local win_dir
    local preview_arg=""
    local detect_black_arg=""
    case "${sender_preview,,}" in
      1|true|yes|on)
        preview_arg=" --preview"
        ;;
    esac
    case "${detect_black,,}" in
      1|true|yes|on)
        detect_black_arg=" --detect-black --black-detect-hz ${black_detect_hz} --black-v-max ${black_v_max} --min-black-area ${min_black_area}"
        ;;
    esac
    win_dir="$(wslpath -w "${SCRIPT_DIR}" 2>/dev/null || echo "C:\\Users\\<user>\\Downloads\\OpenCV_ROS\\OpenCV_ROS")"
    warn "Failed to auto-launch Windows sender. Run these commands in Windows terminal:"
    echo "  cd /d \"${win_dir}\""
    echo "  python windows_camera_ros_sender.py --host ${sender_host} --port ${bridge_port} --index ${camera_index} --fps ${camera_fps} --send-hz ${sender_hz} --width ${camera_width} --height ${camera_height} --jpeg-quality ${jpeg_quality}${preview_arg}${detect_black_arg}"
  fi

  "${PYTHON_BIN}" "${SCRIPT_DIR}/camera_point_cv_subscriber.py"
}

run_win_coord() {
  local preferred_port="${WIN_BRIDGE_PORT:-5001}"
  local bridge_port="${preferred_port}"
  local sender_host="127.0.0.1"
  local camera_index="${WIN_CAMERA_INDEX:-0}"
  local camera_fps="${WIN_CAMERA_FPS:-30}"
  local sender_hz="${WIN_SEND_HZ:-30}"
  local camera_width="${WIN_CAMERA_WIDTH:-960}"
  local camera_height="${WIN_CAMERA_HEIGHT:-540}"
  local jpeg_quality="${WIN_JPEG_QUALITY:-80}"
  local sender_preview="${WIN_SENDER_PREVIEW:-1}"
  local detect_black="${WIN_DETECT_BLACK:-0}"
  local black_detect_hz="${WIN_BLACK_DETECT_HZ:-8}"
  local black_v_max="${WIN_BLACK_V_MAX:-55}"
  local min_black_area="${WIN_MIN_BLACK_AREA:-600}"
  local sender_coord_only="${WIN_SENDER_COORD_ONLY:-1}"
  local point_scale_px_per_meter="${WIN_POINT_SCALE_PX_PER_METER:-100.0}"
  local point_default_z_m="${WIN_POINT_DEFAULT_Z_M:-0.0}"
  local xyz_decimals="${WIN_XYZ_DECIMALS:-1}"
  local coord_frame_id="${WIN_COORD_FRAME_ID:-base_link}"
  local coord_axis_mapping="${WIN_COORD_AXIS_MAPPING:-rep103_base_link}"

  if [[ "${point_scale_px_per_meter}" =~ ^[+-]?[0-9]+$ ]]; then
    point_scale_px_per_meter="${point_scale_px_per_meter}.0"
  fi
  if [[ "${point_default_z_m}" =~ ^[+-]?[0-9]+$ ]]; then
    point_default_z_m="${point_default_z_m}.0"
  fi

  if [[ ! -f "${SCRIPT_DIR}/windows_coordinate_bridge.py" ]]; then
    fail "windows_coordinate_bridge.py not found"
  fi

  if tcp_port_in_use "${bridge_port}"; then
    local fallback_port
    fallback_port="$(pick_free_tcp_port $((preferred_port + 1)) 50 || true)"
    if [[ -z "${fallback_port}" ]]; then
      fail "TCP port ${preferred_port} is busy and no fallback port was found. Stop stale bridge processes and retry."
    fi
    warn "Port ${preferred_port} is in use. Switching to ${fallback_port}."
    bridge_port="${fallback_port}"
  fi

  log "Starting coordinate-only bridge on 0.0.0.0:${bridge_port}..."
  "${PYTHON_BIN}" "${SCRIPT_DIR}/windows_coordinate_bridge.py" \
    --host "0.0.0.0" \
    --port "${bridge_port}" \
    --scale-px-per-meter "${point_scale_px_per_meter}" \
    --default-z-m "${point_default_z_m}" \
    --xyz-decimals "${xyz_decimals}" \
    --frame-id "${coord_frame_id}" \
    --axis-mapping "${coord_axis_mapping}" &
  COORD_PID=$!

  sleep 1
  if ! kill -0 ${COORD_PID} 2>/dev/null; then
    fail "Coordinate-only bridge failed to start"
  fi

  trap "kill ${COORD_PID} 2>/dev/null || true" EXIT

  log "Coordinate bridge is running (PID: ${COORD_PID})"
  log "Coordinate output format: source/frame/(x y z) in separate lines"
  log "Coordinate mapping: frame=${coord_frame_id} axis_mapping=${coord_axis_mapping}"
  log "Trying to launch Windows sender terminal..."
  if ! launch_windows_sender_terminal "${sender_host}" "${bridge_port}" "${camera_index}" "${camera_fps}" "${sender_hz}" "${camera_width}" "${camera_height}" "${jpeg_quality}" "${sender_preview}" "${detect_black}" "${black_detect_hz}" "${black_v_max}" "${min_black_area}" "${sender_coord_only}"; then
    local win_dir
    local preview_arg=""
    local detect_black_arg=""
    local coord_only_arg=""
    case "${sender_preview,,}" in
      1|true|yes|on)
        preview_arg=" --preview"
        ;;
    esac
    case "${detect_black,,}" in
      1|true|yes|on)
        detect_black_arg=" --detect-black --black-detect-hz ${black_detect_hz} --black-v-max ${black_v_max} --min-black-area ${min_black_area}"
        ;;
    esac
    case "${sender_coord_only,,}" in
      1|true|yes|on)
        coord_only_arg=" --coord-only"
        ;;
    esac
    win_dir="$(wslpath -w "${SCRIPT_DIR}" 2>/dev/null || echo "C:\\Users\\<user>\\Downloads\\OpenCV_ROS\\OpenCV_ROS")"
    warn "Failed to auto-launch Windows sender. Run these commands in Windows terminal:"
    echo "  cd /d \"${win_dir}\""
    echo "  python windows_camera_ros_sender.py --host ${sender_host} --port ${bridge_port} --index ${camera_index} --fps ${camera_fps} --send-hz ${sender_hz} --width ${camera_width} --height ${camera_height} --jpeg-quality ${jpeg_quality}${preview_arg}${detect_black_arg}${coord_only_arg}"
  fi

  wait ${COORD_PID}
}

run_test_camera() {
  log "Launching camera testing suite..."
  if [[ ! -f "${SCRIPT_DIR}/test_camera.bash" ]]; then
    fail "test_camera.bash not found. Please create it first."
  fi
  bash "${SCRIPT_DIR}/test_camera.bash" menu
}

auto_select_target() {
  if has_display && check_modules rclpy sensor_msgs.msg std_msgs.msg geometry_msgs.msg cv_bridge numpy cv2; then
    if camera_device_exists; then
      echo "cv"
      return 0
    fi

    if topic_has_publishers; then
      echo "cv-view"
      return 0
    fi

    echo "mock-cv"
    return 0
  fi
  echo "main"
}

case "${TARGET}" in
  -h|--help|help)
    usage
    exit 0
    ;;
  auto|main|cv|cv-view|mock-cv|win-bridge|win-cv|win-coord|doctor|test-camera)
    ;;
  *)
    usage
    fail "Unsupported mode: ${TARGET}"
    ;;
esac

pick_python
NEEDS_ROS=1
if [[ "${TARGET}" == "win-coord" ]]; then
  NEEDS_ROS=0
fi
if [[ "${TARGET}" == "win-cv" ]]; then
  win_coord_only_mode_value="${WIN_COORD_ONLY_MODE:-1}"
  case "${win_coord_only_mode_value,,}" in
    1|true|yes|on)
      NEEDS_ROS=0
      ;;
  esac
fi

if [[ ${NEEDS_ROS} -eq 1 ]]; then
  source_ros_env
fi

if [[ "${TARGET}" == "doctor" ]]; then
  doctor
  exit $?
fi

if [[ "${TARGET}" == "auto" ]]; then
  TARGET="$(auto_select_target)"
  log "Auto mode selected: ${TARGET}"
fi

if [[ "${TARGET}" == "main" ]]; then
  run_main
fi

if [[ "${TARGET}" == "cv" ]]; then
  run_cv
fi

if [[ "${TARGET}" == "cv-view" ]]; then
  run_cv_view
fi

if [[ "${TARGET}" == "mock-cv" ]]; then
  run_mock_cv
fi

if [[ "${TARGET}" == "win-bridge" ]]; then
  run_win_bridge
fi

if [[ "${TARGET}" == "win-cv" ]]; then
  run_win_cv
fi

if [[ "${TARGET}" == "win-coord" ]]; then
  run_win_coord
fi

if [[ "${TARGET}" == "test-camera" ]]; then
  run_test_camera
fi
