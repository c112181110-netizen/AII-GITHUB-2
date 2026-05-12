#!/usr/bin/env python3
"""ROS 2 node that displays camera images and overlays PointStamped data."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_image_codec import image_msg_to_bgr8
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import tf2_geometry_msgs
    import tf2_ros
except Exception:  # noqa: BLE001
    tf2_geometry_msgs = None
    tf2_ros = None


class CameraPointCvSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("camera_point_cv_subscriber")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("point_topic", "/camera/object_point")
        self.declare_parameter("show_point_overlay", True)
        self.declare_parameter("enable_click_scan", True)
        self.declare_parameter("scale_px_per_meter", 100.0)
        self.declare_parameter("default_z_m", 0.0)
        self.declare_parameter("output_frame_mode", "camera")
        self.declare_parameter("arm_frame_id", "arm_base_link")
        self.declare_parameter("use_tf_for_arm_output", True)
        self.declare_parameter("xyz_output_topic", "/camera/object_xyz")
        self.declare_parameter("xyz_output_decimals", 1)
        self.declare_parameter("tx_topic", "/camera/tx")
        self.declare_parameter("stop_topic", "/camera/stop")
        self.declare_parameter("rx_topic", "/camera/rx")
        self.declare_parameter("bridge_control_topic", "/camera/bridge_control")
        self.declare_parameter("show_debug_metrics", True)
        self.declare_parameter("processing_mode", "none")
        self.declare_parameter("canny_low", 80)
        self.declare_parameter("canny_high", 160)
        self.declare_parameter("min_target_area", 350.0)
        self.declare_parameter("auto_publish_detected_point", False)
        self.declare_parameter("auto_publish_hz", 8.0)
        self.declare_parameter("render_hz", 60.0)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.point_topic = self.get_parameter("point_topic").get_parameter_value().string_value
        self.show_point_overlay = (
            self.get_parameter("show_point_overlay").get_parameter_value().bool_value
        )
        self.enable_click_scan = (
            self.get_parameter("enable_click_scan").get_parameter_value().bool_value
        )
        self.scale_px_per_meter = (
            self.get_parameter("scale_px_per_meter").get_parameter_value().double_value
        )
        self.default_z_m = self.get_parameter("default_z_m").get_parameter_value().double_value
        self.output_frame_mode = self._normalize_output_mode(
            self.get_parameter("output_frame_mode").get_parameter_value().string_value
        )
        self.arm_frame_id = self.get_parameter("arm_frame_id").get_parameter_value().string_value
        self.use_tf_for_arm_output = (
            self.get_parameter("use_tf_for_arm_output").get_parameter_value().bool_value
        )
        self.xyz_output_topic = (
            self.get_parameter("xyz_output_topic").get_parameter_value().string_value
        )
        self.xyz_output_decimals = (
            self.get_parameter("xyz_output_decimals").get_parameter_value().integer_value
        )
        self.tx_topic = self.get_parameter("tx_topic").get_parameter_value().string_value
        self.stop_topic = self.get_parameter("stop_topic").get_parameter_value().string_value
        self.rx_topic = self.get_parameter("rx_topic").get_parameter_value().string_value
        self.bridge_control_topic = (
            self.get_parameter("bridge_control_topic").get_parameter_value().string_value
        )
        self.show_debug_metrics = (
            self.get_parameter("show_debug_metrics").get_parameter_value().bool_value
        )
        self.processing_mode = (
            self.get_parameter("processing_mode").get_parameter_value().string_value.lower().strip()
        )
        self.canny_low = self.get_parameter("canny_low").get_parameter_value().integer_value
        self.canny_high = self.get_parameter("canny_high").get_parameter_value().integer_value
        self.min_target_area = (
            self.get_parameter("min_target_area").get_parameter_value().double_value
        )
        self.auto_publish_detected_point = (
            self.get_parameter("auto_publish_detected_point").get_parameter_value().bool_value
        )
        self.auto_publish_hz = (
            self.get_parameter("auto_publish_hz").get_parameter_value().double_value
        )
        self.render_hz = self.get_parameter("render_hz").get_parameter_value().double_value

        self.window_name = "Camera Viewer (q:quit d:debug s:stop)"
        self.width = 960
        self.height = 720

        if self.scale_px_per_meter <= 0.0:
            self.scale_px_per_meter = 100.0
        if self.canny_low < 0:
            self.canny_low = 0
        if self.canny_high <= self.canny_low:
            self.canny_high = self.canny_low + 1
        if self.min_target_area <= 0.0:
            self.min_target_area = 350.0
        if self.auto_publish_hz <= 0.0:
            self.auto_publish_hz = 8.0
        if self.render_hz <= 0.0:
            self.render_hz = 60.0
        if self.xyz_output_decimals < 0:
            self.xyz_output_decimals = 0
        if self.xyz_output_decimals > 6:
            self.xyz_output_decimals = 6

        self.tf_buffer = None
        self.tf_listener = None
        self.last_tf_warn_time = self.get_clock().now()
        if self.output_frame_mode == "arm" and self.use_tf_for_arm_output:
            if tf2_ros is None or tf2_geometry_msgs is None:
                self.get_logger().warning(
                    "TF modules are unavailable. Output falls back to camera frame."
                )
            else:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_image: np.ndarray | None = None
        self.latest_image_msg: Image | None = None
        self.latest_point_msg: PointStamped | None = None
        self.last_clicked_pixel: tuple[int, int] | None = None
        self.last_detected_pixel: tuple[int, int] | None = None
        self.last_detected_area: float = 0.0
        self.processing_mode_warned = False
        self.last_auto_publish_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.last_tx_message = "TX: N/A"
        self.last_rx_message = "RX: N/A"
        self.publish_fps_log_enabled = True

        now_mono = time.monotonic()
        self.image_hz_measured = 0.0
        self.point_hz_measured = 0.0
        self.render_fps_measured = 0.0
        self._image_window_started = now_mono
        self._point_window_started = now_mono
        self._render_window_started = now_mono
        self._image_count = 0
        self._point_count = 0
        self._render_count = 0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos,
        )
        self.point_subscription = self.create_subscription(
            PointStamped,
            self.point_topic,
            self.point_callback,
            10,
        )
        self.rx_subscription = self.create_subscription(
            String,
            self.rx_topic,
            self.rx_callback,
            10,
        )

        self.point_publisher = self.create_publisher(PointStamped, self.point_topic, 10)
        self.xyz_publisher = self.create_publisher(String, self.xyz_output_topic, 10)
        self.tx_publisher = self.create_publisher(String, self.tx_topic, 10)
        self.stop_publisher = self.create_publisher(String, self.stop_topic, 10)
        self.bridge_control_publisher = self.create_publisher(
            String,
            self.bridge_control_topic,
            10,
        )

        self.timer = self.create_timer(1.0 / self.render_hz, self.render_frame)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        cv2.setMouseCallback(self.window_name, self.on_mouse_event)

        self.get_logger().info(f"Subscribed image topic: {self.image_topic}")
        self.get_logger().info(f"Subscribed point topic: {self.point_topic}")
        self.get_logger().info(f"XYZ output topic: {self.xyz_output_topic}")
        self.get_logger().info(f"TX topic: {self.tx_topic}, STOP topic: {self.stop_topic}")
        self.get_logger().info(f"RX topic: {self.rx_topic}")
        self.get_logger().info(f"Bridge control topic: {self.bridge_control_topic}")
        self.get_logger().info(f"Render refresh rate: {self.render_hz:.2f} Hz")
        self.get_logger().info(f"Scale setting: {self.scale_px_per_meter:.1f} px/m (1:100cm)")
        self.get_logger().info(
            f"Output frame mode: {self.output_frame_mode} "
            f"(arm frame: {self.arm_frame_id})"
        )
        self.get_logger().info(f"Processing mode: {self.processing_mode}")
        if self.enable_click_scan:
            self.get_logger().info(
                "Click-scan enabled: left-click image to publish /camera/object_point."
            )
        if self.auto_publish_detected_point:
            self.get_logger().info(
                f"Auto point publish enabled at <= {self.auto_publish_hz:.2f} Hz"
            )
        self.get_logger().info("Press 'q' quit, 'd' debug, 'f' fps-log toggle, 's' STOP.")

    @staticmethod
    def _normalize_output_mode(raw_mode: str) -> str:
        mode = str(raw_mode).strip().lower()
        if mode in ("arm", "arm000", "robot", "robot_arm"):
            return "arm"
        return "camera"

    def _update_image_rate(self) -> None:
        self._image_count += 1
        now = time.monotonic()
        elapsed = now - self._image_window_started
        if elapsed >= 1.0:
            self.image_hz_measured = self._image_count / elapsed
            self._image_count = 0
            self._image_window_started = now

    def _update_point_rate(self) -> None:
        self._point_count += 1
        now = time.monotonic()
        elapsed = now - self._point_window_started
        if elapsed >= 1.0:
            self.point_hz_measured = self._point_count / elapsed
            self._point_count = 0
            self._point_window_started = now

    def _update_render_rate(self) -> None:
        self._render_count += 1
        now = time.monotonic()
        elapsed = now - self._render_window_started
        if elapsed >= 1.0:
            self.render_fps_measured = self._render_count / elapsed
            self._render_count = 0
            self._render_window_started = now

    def image_callback(self, msg: Image) -> None:
        self._update_image_rate()

        try:
            raw_frame = image_msg_to_bgr8(msg)
            processed_frame = self._process_frame(raw_frame)
            self.latest_image = processed_frame
            self.latest_image_msg = msg
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Failed to convert image message: {exc}")

    def point_callback(self, msg: PointStamped) -> None:
        self._update_point_rate()

        point_for_output = self._transform_for_output(msg)
        self.latest_point_msg = point_for_output
        self._publish_xyz_output(point_for_output)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds > 1_000_000_000:
            self.last_log_time = now
            self.get_logger().info(
                f"Point frame={point_for_output.header.frame_id or 'unknown'} "
                f"{self._format_xyz(point_for_output)}"
            )

    def rx_callback(self, msg: String) -> None:
        self.last_rx_message = f"RX: {msg.data.strip() or '<empty>'}"
        self.get_logger().info(self.last_rx_message)

    @staticmethod
    def _debug_button_rect(frame_w: int) -> tuple[int, int, int, int]:
        x2 = max(140, frame_w - 12)
        x1 = max(10, x2 - 160)
        y1 = 10
        y2 = 44
        return x1, y1, x2, y2

    @staticmethod
    def _fps_log_button_rect(frame_w: int) -> tuple[int, int, int, int]:
        x2 = max(332, frame_w - 184)
        x1 = max(170, x2 - 190)
        y1 = 10
        y2 = 44
        return x1, y1, x2, y2

    def _publish_bridge_fps_log_state(self) -> None:
        command = "fps_log_on" if self.publish_fps_log_enabled else "fps_log_off"
        msg = String()
        msg.data = command
        self.bridge_control_publisher.publish(msg)

        self.last_tx_message = f"TX {command.upper()}"
        self.get_logger().info(
            f"Bridge publishing-fps log -> {'ON' if self.publish_fps_log_enabled else 'OFF'}"
        )

    def _toggle_bridge_fps_log_from_click(self, x: int, y: int) -> bool:
        frame_w = self.latest_image.shape[1] if self.latest_image is not None else self.width
        x1, y1, x2, y2 = self._fps_log_button_rect(frame_w)
        if x1 <= x <= x2 and y1 <= y <= y2:
            self.publish_fps_log_enabled = not self.publish_fps_log_enabled
            self._publish_bridge_fps_log_state()
            return True
        return False

    def _toggle_debug_from_click(self, x: int, y: int) -> bool:
        frame_w = self.latest_image.shape[1] if self.latest_image is not None else self.width
        x1, y1, x2, y2 = self._debug_button_rect(frame_w)
        if x1 <= x <= x2 and y1 <= y <= y2:
            self.show_debug_metrics = not self.show_debug_metrics
            state = "ON" if self.show_debug_metrics else "OFF"
            self.get_logger().info(f"Debug metrics toggled: {state}")
            return True
        return False

    def on_mouse_event(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        del flags, param

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self._toggle_bridge_fps_log_from_click(int(x), int(y)):
            return

        if self._toggle_debug_from_click(int(x), int(y)):
            return

        if not self.enable_click_scan:
            return

        if self.latest_image is None:
            return

        self.last_clicked_pixel = (int(x), int(y))
        self._publish_point_from_pixel(int(x), int(y), source="click")

    def _point_from_pixel(self, x: int, y: int) -> PointStamped:
        h, w = self.latest_image.shape[:2]
        x_m = (float(x) - (w / 2.0)) / self.scale_px_per_meter
        y_m = ((h / 2.0) - float(y)) / self.scale_px_per_meter

        frame_id = "camera_frame"
        if self.latest_image_msg is not None and self.latest_image_msg.header.frame_id:
            frame_id = self.latest_image_msg.header.frame_id

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = frame_id
        point_msg.point.x = float(x_m)
        point_msg.point.y = float(y_m)
        point_msg.point.z = float(self.default_z_m)
        return point_msg

    def _transform_for_output(self, msg: PointStamped) -> PointStamped:
        if self.output_frame_mode != "arm":
            return msg

        if not self.use_tf_for_arm_output or self.tf_buffer is None or tf2_geometry_msgs is None:
            return msg

        src_frame = msg.header.frame_id or ""
        if not src_frame or src_frame == self.arm_frame_id:
            return msg

        try:
            transform = self.tf_buffer.lookup_transform(
                self.arm_frame_id,
                src_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return tf2_geometry_msgs.do_transform_point(msg, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time).nanoseconds > 1_000_000_000:
                self.last_tf_warn_time = now
                self.get_logger().warning(
                    f"TF transform failed ({src_frame} -> {self.arm_frame_id}), "
                    f"using source frame: {exc}"
                )
            return msg

    def _format_xyz(self, point_msg: PointStamped) -> str:
        value_fmt = f"{{:.{int(self.xyz_output_decimals)}f}}"
        p = point_msg.point
        return (
            f"(X y z) ({value_fmt.format(float(p.x))} "
            f"{value_fmt.format(float(p.y))} "
            f"{value_fmt.format(float(p.z))})"
        )

    def _publish_xyz_output(self, point_msg: PointStamped) -> None:
        msg = String()
        msg.data = self._format_xyz(point_msg)
        self.xyz_publisher.publish(msg)

    def _publish_tx_target(self, point_msg: PointStamped, source: str) -> None:
        tx_msg = String()
        tx_msg.data = (
            f"TX TARGET source={source} frame={point_msg.header.frame_id or 'unknown'} "
            f"{self._format_xyz(point_msg)}"
        )
        self.tx_publisher.publish(tx_msg)
        self.last_tx_message = tx_msg.data

    def _publish_stop_signal(self) -> None:
        stop_msg = String()
        stop_msg.data = "STOP"
        self.stop_publisher.publish(stop_msg)

        tx_msg = String()
        tx_msg.data = "TX STOP"
        self.tx_publisher.publish(tx_msg)
        self.last_tx_message = tx_msg.data

        self.get_logger().info(
            f"Published STOP on {self.stop_topic} and TX STOP on {self.tx_topic}"
        )

    def _publish_point_from_pixel(self, x: int, y: int, source: str) -> None:
        if self.latest_image is None:
            return

        if source == "auto":
            dt_ns = (self.get_clock().now() - self.last_auto_publish_time).nanoseconds
            min_interval_ns = int((1.0 / self.auto_publish_hz) * 1e9)
            if dt_ns < min_interval_ns:
                return

        point_msg = self._point_from_pixel(x, y)
        point_msg = self._transform_for_output(point_msg)
        self.point_publisher.publish(point_msg)
        self.latest_point_msg = point_msg

        self._publish_tx_target(point_msg, source=source)

        if source == "auto":
            self.last_auto_publish_time = self.get_clock().now()
            return

        self.get_logger().info(
            f"Clicked pixel=({x},{y}) frame={point_msg.header.frame_id or 'unknown'} "
            f"{self._format_xyz(point_msg)}"
        )

    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        mode = self.processing_mode

        if mode in ("", "none"):
            self.last_detected_pixel = None
            self.last_detected_area = 0.0
            return frame

        if mode == "gray":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.last_detected_pixel = None
            self.last_detected_area = 0.0
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if mode == "canny":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, int(self.canny_low), int(self.canny_high))
            self.last_detected_pixel = None
            self.last_detected_area = 0.0
            return cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        if mode in ("red-target", "red_track", "red"):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_red_1 = np.array([0, 100, 80], dtype=np.uint8)
            upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
            lower_red_2 = np.array([170, 100, 80], dtype=np.uint8)
            upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)

            mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
            mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
            mask = cv2.bitwise_or(mask_1, mask_2)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self.last_detected_pixel = None
            self.last_detected_area = 0.0

            if contours:
                contour = max(contours, key=cv2.contourArea)
                area = float(cv2.contourArea(contour))
                if area >= self.min_target_area:
                    moments = cv2.moments(contour)
                    if moments["m00"] > 0:
                        cx = int(moments["m10"] / moments["m00"])
                        cy = int(moments["m01"] / moments["m00"])
                        self.last_detected_pixel = (cx, cy)
                        self.last_detected_area = area

                        cv2.drawContours(frame, [contour], -1, (0, 255, 255), 2)
                        cv2.circle(frame, (cx, cy), 8, (0, 0, 255), -1)
                        cv2.putText(
                            frame,
                            f"target area={area:.0f}",
                            (max(12, cx - 80), max(20, cy - 16)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

                        if self.auto_publish_detected_point:
                            self._publish_point_from_pixel(cx, cy, source="auto")

            return frame

        if not self.processing_mode_warned:
            self.processing_mode_warned = True
            self.get_logger().warning(
                f"Unknown processing_mode='{self.processing_mode}'. Falling back to raw image."
            )

        self.last_detected_pixel = None
        self.last_detected_area = 0.0
        return frame

    def _draw_waiting_canvas(self) -> np.ndarray:
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        canvas[:] = (20, 20, 20)

        cv2.putText(
            canvas,
            "Waiting for camera image...",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"image topic: {self.image_topic}",
            (30, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (190, 190, 190),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"point topic: {self.point_topic}",
            (30, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (190, 190, 190),
            2,
            cv2.LINE_AA,
        )

        return canvas

    def _draw_point_overlay(self, frame: np.ndarray) -> None:
        if not self.show_point_overlay:
            return

        h, w = frame.shape[:2]
        inset_w = min(300, max(220, w // 3))
        inset_h = min(240, max(180, h // 3))
        margin = 16

        x1 = w - inset_w - margin
        y1 = margin + 40
        x2 = x1 + inset_w
        y2 = y1 + inset_h

        if x1 < 0 or y2 > h:
            return

        roi = frame[y1:y2, x1:x2]
        overlay = roi.copy()
        overlay[:] = (30, 30, 30)
        cv2.addWeighted(overlay, 0.6, roi, 0.4, 0, roi)

        cx = x1 + inset_w // 2
        cy = y1 + inset_h // 2
        cv2.rectangle(frame, (x1, y1), (x2, y2), (120, 120, 120), 1)
        cv2.line(frame, (x1, cy), (x2, cy), (80, 80, 80), 1)
        cv2.line(frame, (cx, y1), (cx, y2), (80, 80, 80), 1)
        cv2.putText(
            frame,
            "XY (top view)",
            (x1 + 10, y1 + 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )

        if self.latest_point_msg is None:
            cv2.putText(
                frame,
                "No PointStamped",
                (x1 + 10, y1 + inset_h - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )
            return

        p = self.latest_point_msg.point
        px = int(cx + p.x * self.scale_px_per_meter)
        py = int(cy - p.y * self.scale_px_per_meter)
        in_bounds = x1 <= px < x2 and y1 <= py < y2
        color = (0, 220, 0) if in_bounds else (0, 180, 255)

        draw_x = min(max(px, x1), x2 - 1)
        draw_y = min(max(py, y1), y2 - 1)
        cv2.circle(frame, (draw_x, draw_y), 6, color, -1)
        cv2.circle(frame, (draw_x, draw_y), 14, color, 2)

        cv2.putText(
            frame,
            self._format_xyz(self.latest_point_msg),
            (x1 + 10, y2 - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )

    def _draw_header_overlay(self, frame: np.ndarray) -> None:
        lines = [
            f"image topic: {self.image_topic}",
            f"point topic: {self.point_topic}",
            f"output mode: {self.output_frame_mode} (arm={self.arm_frame_id})",
            f"processing: {self.processing_mode}",
            "Press q=quit d=debug f=fps-log s=stop",
        ]

        if self.enable_click_scan:
            lines.append("Left-click image: publish object point")

        if self.latest_image_msg is not None:
            stamp = self.latest_image_msg.header.stamp
            stamp_sec = stamp.sec + stamp.nanosec / 1e9
            frame_id = self.latest_image_msg.header.frame_id or "unknown"
            lines.insert(2, f"image frame_id: {frame_id}  stamp: {stamp_sec:.6f}s")

        for idx, line in enumerate(lines):
            y = 62 + idx * 24
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def _draw_click_marker(self, frame: np.ndarray) -> None:
        if self.last_clicked_pixel is None:
            return

        x, y = self.last_clicked_pixel
        h, w = frame.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return

        color = (0, 220, 255)
        cv2.circle(frame, (x, y), 8, color, 2)
        cv2.line(frame, (x - 12, y), (x + 12, y), color, 1)
        cv2.line(frame, (x, y - 12), (x, y + 12), color, 1)

        label = f"px=({x},{y})"
        cv2.putText(
            frame,
            label,
            (x + 10, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_detected_marker(self, frame: np.ndarray) -> None:
        if self.last_detected_pixel is None:
            return

        x, y = self.last_detected_pixel
        h, w = frame.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return

        color = (0, 0, 255)
        cv2.circle(frame, (x, y), 12, color, 2)
        cv2.putText(
            frame,
            f"detected px=({x},{y})",
            (x + 10, min(h - 8, y + 22)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            1,
            cv2.LINE_AA,
        )

    def _draw_debug_button(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        del h
        x1, y1, x2, y2 = self._debug_button_rect(w)

        fill_color = (48, 140, 65) if self.show_debug_metrics else (80, 80, 80)
        cv2.rectangle(frame, (x1, y1), (x2, y2), fill_color, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (220, 220, 220), 1)

        label = "DEBUG ON" if self.show_debug_metrics else "DEBUG OFF"
        cv2.putText(
            frame,
            label,
            (x1 + 12, y1 + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_bridge_fps_button(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        del h
        x1, y1, x2, y2 = self._fps_log_button_rect(w)

        fill_color = (28, 108, 182) if self.publish_fps_log_enabled else (80, 80, 80)
        cv2.rectangle(frame, (x1, y1), (x2, y2), fill_color, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (220, 220, 220), 1)

        label = "PUB FPS ON" if self.publish_fps_log_enabled else "PUB FPS OFF"
        cv2.putText(
            frame,
            label,
            (x1 + 12, y1 + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _draw_debug_overlay(self, frame: np.ndarray) -> None:
        if not self.show_debug_metrics:
            return

        point_frame = "unknown"
        point_xyz = "(X y z) (N/A N/A N/A)"
        if self.latest_point_msg is not None:
            point_frame = self.latest_point_msg.header.frame_id or "unknown"
            point_xyz = self._format_xyz(self.latest_point_msg)

        lines = [
            f"FPS/HZ image={self.image_hz_measured:.1f} point={self.point_hz_measured:.1f} render={self.render_fps_measured:.1f}",
            f"scale_px_per_meter={self.scale_px_per_meter:.1f}  ratio=1:100cm",
            f"bridge publishing-fps-log={'ON' if self.publish_fps_log_enabled else 'OFF'}",
            f"point frame={point_frame}  {point_xyz}",
            self.last_tx_message,
            self.last_rx_message,
        ]

        h, w = frame.shape[:2]
        box_h = 20 + len(lines) * 22
        y1 = max(0, h - box_h - 10)
        x1 = 10
        x2 = min(w - 10, 760)
        y2 = min(h - 10, y1 + box_h)

        roi = frame[y1:y2, x1:x2]
        if roi.size > 0:
            overlay = roi.copy()
            overlay[:] = (16, 16, 16)
            cv2.addWeighted(overlay, 0.55, roi, 0.45, 0, roi)

        for idx, line in enumerate(lines):
            y = y1 + 24 + idx * 22
            cv2.putText(
                frame,
                line,
                (x1 + 10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def render_frame(self) -> None:
        self._update_render_rate()

        if self.latest_image is None:
            frame = self._draw_waiting_canvas()
        else:
            frame = self.latest_image.copy()

        self._draw_bridge_fps_button(frame)
        self._draw_debug_button(frame)
        self._draw_header_overlay(frame)
        self._draw_point_overlay(frame)
        self._draw_click_marker(frame)
        self._draw_detected_marker(frame)
        self._draw_debug_overlay(frame)

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Quit requested from OpenCV window.")
            rclpy.shutdown()
        elif key == ord("d"):
            self.show_debug_metrics = not self.show_debug_metrics
            state = "ON" if self.show_debug_metrics else "OFF"
            self.get_logger().info(f"Debug metrics toggled: {state}")
        elif key == ord("f"):
            self.publish_fps_log_enabled = not self.publish_fps_log_enabled
            self._publish_bridge_fps_log_state()
        elif key == ord("s"):
            self._publish_stop_signal()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraPointCvSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by keyboard interrupt.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
