#!/usr/bin/env python3
"""Utilities for converting between ROS Image messages and OpenCV frames."""

from __future__ import annotations

import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def cv_frame_to_image_msg(frame: np.ndarray, stamp, frame_id: str, encoding: str = "bgr8") -> Image:
    """Convert an OpenCV frame to sensor_msgs/Image without cv_bridge."""
    if frame is None:
        raise ValueError("frame is None")

    if encoding == "bgr8":
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"Expected HxWx3 frame for bgr8, got shape={frame.shape}")
        step = int(frame.shape[1] * 3)
    elif encoding == "mono8":
        if frame.ndim != 2:
            raise ValueError(f"Expected HxW frame for mono8, got shape={frame.shape}")
        step = int(frame.shape[1])
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    msg = Image()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = step
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


def image_msg_to_bgr8(msg: Image) -> np.ndarray:
    """Convert sensor_msgs/Image to a BGR OpenCV frame without cv_bridge."""
    encoding = msg.encoding.lower()

    if encoding in ("bgr8", "rgb8"):
        channels = 3
        row_stride = int(msg.step) if msg.step else int(msg.width * channels)
        pixel_stride = int(msg.width * channels)

        if row_stride < pixel_stride:
            raise ValueError(
                f"Invalid step for {encoding}: step={row_stride}, width={msg.width}, channels={channels}"
            )

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = row_stride * int(msg.height)
        if raw.size < expected_size:
            raise ValueError(
                f"Image data too small for {encoding}: got={raw.size}, expected={expected_size}"
            )

        rows = raw[:expected_size].reshape((int(msg.height), row_stride))
        frame = rows[:, :pixel_stride].reshape((int(msg.height), int(msg.width), channels))

        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame.copy()

    if encoding == "mono8":
        row_stride = int(msg.step) if msg.step else int(msg.width)

        if row_stride < int(msg.width):
            raise ValueError(f"Invalid step for mono8: step={row_stride}, width={msg.width}")

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected_size = row_stride * int(msg.height)
        if raw.size < expected_size:
            raise ValueError(
                f"Image data too small for mono8: got={raw.size}, expected={expected_size}"
            )

        rows = raw[:expected_size].reshape((int(msg.height), row_stride))
        gray = rows[:, : int(msg.width)]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")
