# Docker Packaging Guide

This project can run inside a ROS 2 Jazzy Docker container.

## 0. First-Time Host Setup (WSL/Linux)

If you see errors like:
- permission denied while trying to connect to /var/run/docker.sock
- docker: unknown command: docker compose

Run these once on the host:

```bash
sudo usermod -aG docker "$USER"
sudo apt-get update
sudo apt-get install -y docker-compose-v2 docker-buildx
```

Then restart your shell session (or WSL) so new group membership applies:

```bash
newgrp docker
```

Verify:

```bash
docker info
docker compose version
docker buildx version
```

Quick workaround (without group change) is to prefix Docker commands with `sudo`.

## 1. Build Image

```bash
docker build -t opencv_ros:jazzy .
```

Or with Compose:

```bash
docker compose build
```

## 2. Quick Health Check

```bash
docker run --rm -it --network host opencv_ros:jazzy
```

Default command is:

```bash
bash run.bash doctor
```

## 3. Run Common Modes

### 3.1 Mock Camera + Viewer

```bash
docker run --rm -it \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  opencv_ros:jazzy \
  bash run.bash mock-cv
```

### 3.2 USB Camera Mode

```bash
docker run --rm -it \
  --network host \
  --device /dev/video0:/dev/video0 \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  opencv_ros:jazzy \
  bash run.bash cv
```

### 3.3 Windows Sender Bridge Mode

Bridge only:

```bash
docker run --rm -it --network host opencv_ros:jazzy bash run.bash win-bridge
```

Bridge + viewer:

```bash
docker run --rm -it \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  opencv_ros:jazzy \
  bash run.bash win-cv
```

Note:
- Auto-launch of Windows sender (`cmd.exe`) is usually unavailable inside Linux containers.
- In that case, manually run `windows_camera_ros_sender.py` on Windows host.

## 4. Black Object Detection (Windows Sender)

Use env vars with `win-cv` mode:

```bash
docker run --rm -it \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -e WIN_DETECT_BLACK=1 \
  -e WIN_BLACK_DETECT_HZ=8 \
  -e WIN_BLACK_V_MAX=55 \
  -e WIN_MIN_BLACK_AREA=600 \
  -e WIN_POINT_SCALE_PX_PER_METER=80.0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  opencv_ros:jazzy \
  bash run.bash win-cv
```

## 5. Compose Usage

```bash
docker compose up --build
```

Override command:

```bash
docker compose run --rm opencv_ros bash run.bash mock-cv
```

## 6. Verify Point Topic

In another shell:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /camera/object_point
```
