FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    python3-numpy \
    ros-jazzy-cv-bridge \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    iproute2 \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/OpenCV_ROS
COPY . /workspace/OpenCV_ROS

RUN chmod +x /workspace/OpenCV_ROS/run.bash \
    && chmod +x /workspace/OpenCV_ROS/test_camera.bash \
    && chmod +x /workspace/OpenCV_ROS/docker/entrypoint.sh

ENTRYPOINT ["/workspace/OpenCV_ROS/docker/entrypoint.sh"]
CMD ["bash", "run.bash", "doctor"]
