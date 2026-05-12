#!/usr/bin/env python3
"""ROS 2 node that captures USB camera frames and publishes sensor_msgs/Image."""

from __future__ import annotations

import cv2
import os
import rclpy
import time
from rclpy.node import Node
from ros_image_codec import cv_frame_to_image_msg
from sensor_msgs.msg import Image


class UsbCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("usb_camera_publisher")

        self.declare_parameter("device_id", 0)
        self.declare_parameter("publish_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 30)

        self.device_id = self.get_parameter("device_id").get_parameter_value().integer_value
        self.publish_topic = self.get_parameter("publish_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_width = self.get_parameter("frame_width").get_parameter_value().integer_value
        self.frame_height = self.get_parameter("frame_height").get_parameter_value().integer_value
        self.fps = max(1, self.get_parameter("fps").get_parameter_value().integer_value)

        self.publisher = self.create_publisher(Image, self.publish_topic, 10)

        self.capture = self._open_camera()
        self.consecutive_read_failures = 0
        self.last_reopen_attempt = 0.0

        self.timer = self.create_timer(1.0 / float(self.fps), self.publish_frame)
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            f"USB camera publisher started\n"
            f"  Device: /dev/video{self.device_id}\n"
            f"  Topic: {self.publish_topic}\n"
            f"  Requested: {self.frame_width}x{self.frame_height} @ {self.fps} FPS"
        )

    def _candidate_sources(self) -> list[int | str]:
        sources: list[int | str] = [int(self.device_id)]
        device_path = f"/dev/video{self.device_id}"
        if os.path.exists(device_path):
            sources.append(device_path)
        return sources

    def _candidate_backends(self) -> list[tuple[str, int]]:
        backends: list[tuple[str, int]] = []
        if hasattr(cv2, "CAP_V4L2"):
            backends.append(("V4L2", cv2.CAP_V4L2))
        if hasattr(cv2, "CAP_FFMPEG"):
            backends.append(("FFMPEG", cv2.CAP_FFMPEG))
        backends.append(("ANY", cv2.CAP_ANY))
        return backends

    def _candidate_fourccs(self) -> list[tuple[str, int | None]]:
        if not hasattr(cv2, "VideoWriter_fourcc"):
            return [("default", None)]

        return [
            ("default", None),
            ("MJPG", cv2.VideoWriter_fourcc(*"MJPG")),
            ("YUYV", cv2.VideoWriter_fourcc(*"YUYV")),
        ]

    def _configure_capture(self, capture: cv2.VideoCapture, fourcc: int | None) -> None:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.frame_width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.frame_height))
        capture.set(cv2.CAP_PROP_FPS, float(self.fps))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)
        if fourcc is not None:
            capture.set(cv2.CAP_PROP_FOURCC, float(fourcc))

    def _probe_capture(self, capture: cv2.VideoCapture, attempts: int = 25) -> bool:
        for _ in range(attempts):
            ok, frame = capture.read()
            if ok and frame is not None:
                return True
        return False

    def _open_camera(self) -> cv2.VideoCapture:
        attempts: list[str] = []

        for source in self._candidate_sources():
            for backend_name, backend in self._candidate_backends():
                for fourcc_name, fourcc in self._candidate_fourccs():
                    capture = cv2.VideoCapture(source, backend)
                    attempt_label = (
                        f"source={source} backend={backend_name} fourcc={fourcc_name}"
                    )

                    if not capture.isOpened():
                        attempts.append(f"{attempt_label} -> open failed")
                        capture.release()
                        continue

                    self._configure_capture(capture, fourcc)
                    if self._probe_capture(capture):
                        self.get_logger().info(f"Camera opened using {attempt_label}")
                        return capture

                    attempts.append(f"{attempt_label} -> read probe failed")
                    capture.release()

        debug_attempts = "\n  - ".join(attempts[-8:]) if attempts else "none"
        raise RuntimeError(
            f"Failed to open camera stream /dev/video{self.device_id}. Recent attempts:\n  - {debug_attempts}"
        )

    def _try_reopen_capture(self) -> None:
        now = time.monotonic()
        if now - self.last_reopen_attempt < 5.0:
            return

        self.last_reopen_attempt = now
        self.get_logger().warning("Attempting to reopen camera stream...")

        try:
            if self.capture is not None:
                self.capture.release()
            self.capture = self._open_camera()
            self.consecutive_read_failures = 0
            self.get_logger().info("Camera stream recovered.")
        except RuntimeError as exc:
            self.get_logger().error(f"Camera reopen failed: {exc}")

    def publish_frame(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.consecutive_read_failures += 1
            if self.consecutive_read_failures in (1, 30, 120):
                self.get_logger().warning(
                    f"Failed to read frame from camera (count={self.consecutive_read_failures})."
                )

            if self.consecutive_read_failures >= 120:
                self._try_reopen_capture()
            return

        self.consecutive_read_failures = 0

        message = cv_frame_to_image_msg(
            frame=frame,
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id,
            encoding="bgr8",
        )

        self.publisher.publish(message)

        self.frame_count += 1
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.last_log_time = now
            height, width = frame.shape[:2]
            self.get_logger().info(
                f"Publishing {width}x{height} frames on {self.publish_topic} (count={self.frame_count})"
            )

    def destroy_node(self) -> bool:
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    try:
        node = UsbCameraPublisher()
    except RuntimeError as exc:
        print(f"[usb_camera_publisher][error] {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by keyboard interrupt.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
