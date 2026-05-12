#!/usr/bin/env python3
"""ROS 2 main controller: OpenCV point -> MoveIt IK -> CM530 AX positions.

This file intentionally reuses TEST07.CM530Bridge for every CM530 command.
The serial protocol remains owned by TEST07.py and the CM530 firmware spec.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence

from TEST07 import CM530Bridge, DEFAULT_BAUD, DEFAULT_PORT
from ik_to_ax import (
    AxCalibration,
    extract_ordered_joint_positions,
    format_joint_debug,
    radians_to_ax_positions,
)

ROS_IMPORT_ERROR: ImportError | None = None
try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PointStamped, PoseStamped
    from moveit_msgs.msg import MoveItErrorCodes
    from moveit_msgs.srv import GetPositionIK
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError as exc:  # pragma: no cover - depends on sourced ROS env
    ROS_IMPORT_ERROR = exc
    rclpy = None  # type: ignore[assignment]
    Duration = None  # type: ignore[assignment]
    PointStamped = object  # type: ignore[assignment]
    PoseStamped = object  # type: ignore[assignment]
    MoveItErrorCodes = None  # type: ignore[assignment]
    GetPositionIK = None  # type: ignore[assignment]
    ReentrantCallbackGroup = None  # type: ignore[assignment]
    MultiThreadedExecutor = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment]
    JointState = object  # type: ignore[assignment]

try:
    from opencv_ros2_bridge_interfaces.srv import GetObjectPoint
except ImportError:  # pragma: no cover - optional workspace overlay
    GetObjectPoint = None

try:
    import tf2_geometry_msgs
    import tf2_ros
except ImportError:  # pragma: no cover - optional but normally present in ROS env
    tf2_geometry_msgs = None
    tf2_ros = None


ARM_JOINT_NAMES = (
    "phantomx_pincher_arm_shoulder_pan_joint",
    "phantomx_pincher_arm_shoulder_lift_joint",
    "phantomx_pincher_arm_elbow_flex_joint",
    "phantomx_pincher_arm_wrist_flex_joint",
)


@dataclass(frozen=True)
class ControllerConfig:
    point_topic: str
    point_service: str
    compute_ik_service: str
    group_name: str
    target_frame: str
    end_effector_link: str
    quat_xyzw: tuple[float, float, float, float]
    use_tf: bool
    min_command_interval_sec: float
    ik_timeout_sec: float
    service_timeout_sec: float
    send_mode: str
    trajectory_dt_ms: int
    move_enabled: bool
    vision_map: str
    vision_base_xyz: tuple[float, float, float]
    vision_gain_xy: tuple[float, float]
    vision_x_limits: tuple[float, float]
    vision_y_limits: tuple[float, float]
    vision_z_limits: tuple[float, float]


class OpenCvToCm530Node(Node):
    def __init__(
        self,
        config: ControllerConfig,
        bridge: CM530Bridge,
        calibration: AxCalibration,
        once: bool = False,
    ) -> None:
        super().__init__("opencv_to_cm530_node")
        self.config = config
        self.bridge = bridge
        self.calibration = calibration
        self.once = once

        self.callback_group = ReentrantCallbackGroup()
        self.last_command_time = 0.0
        self.completed_once = False

        self.ik_client = self.create_client(
            GetPositionIK,
            self.config.compute_ik_service,
            callback_group=self.callback_group,
        )

        self.point_client = None
        if GetObjectPoint is not None:
            self.point_client = self.create_client(
                GetObjectPoint,
                self.config.point_service,
                callback_group=self.callback_group,
            )

        self.tf_buffer = None
        self.tf_listener = None
        if self.config.use_tf:
            if tf2_ros is None or tf2_geometry_msgs is None:
                self.get_logger().warning("TF modules unavailable; point frame must already match target frame.")
            else:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.point_subscription = None
        if not once:
            self.point_subscription = self.create_subscription(
                PointStamped,
                self.config.point_topic,
                self._point_callback,
                10,
                callback_group=self.callback_group,
            )

        self.get_logger().info(
            "OpenCV -> MoveIt IK -> CM530 controller started\n"
            f"  point_topic: {self.config.point_topic}\n"
            f"  point_service: {self.config.point_service}"
            f"{' (unavailable)' if GetObjectPoint is None else ''}\n"
            f"  compute_ik: {self.config.compute_ik_service}\n"
            f"  target_frame: {self.config.target_frame}\n"
            f"  eef_link: {self.config.end_effector_link}\n"
            f"  send_mode: {self.config.send_mode}\n"
            f"  serial_dry_run: {self.bridge.dry_run}"
        )

    def wait_for_ik_service(self) -> bool:
        self.get_logger().info(f"Waiting for {self.config.compute_ik_service}...")
        ok = self.ik_client.wait_for_service(timeout_sec=self.config.service_timeout_sec)
        if not ok:
            self.get_logger().error(f"{self.config.compute_ik_service} is not available.")
        return ok

    def request_latest_opencv_point(self) -> Optional[PointStamped]:
        if GetObjectPoint is None or self.point_client is None:
            self.get_logger().error(
                "OpenCV GetObjectPoint service type is unavailable. "
                "Build/source opencv_ros2_bridge_interfaces or use --test-point."
            )
            return None

        if not self.point_client.wait_for_service(timeout_sec=self.config.service_timeout_sec):
            self.get_logger().error(f"{self.config.point_service} is not available.")
            return None

        future = self.point_client.call_async(GetObjectPoint.Request())
        if not self._wait_for_future(future, self.config.service_timeout_sec):
            self.get_logger().error("Timed out waiting for OpenCV latest point service.")
            return None

        response = future.result()
        if not response.success:
            self.get_logger().warning(f"OpenCV has no point yet: {response.message}")
            return None

        self.get_logger().info(
            f"OpenCV service point source={response.source or '<unknown>'} "
            f"{self._format_point(response.point)}"
        )
        return response.point

    def _point_callback(self, msg: PointStamped) -> None:
        now = time.monotonic()
        if (now - self.last_command_time) < self.config.min_command_interval_sec:
            return
        self.last_command_time = now
        self.process_point(msg)

    def process_point(self, msg: PointStamped) -> bool:
        point = self._transform_to_target(msg)
        if point is None:
            return False
        point = self._map_vision_point(point)

        self.get_logger().info(f"Target point {self._format_point(point)}")

        joints_rad = self.compute_ik(point)
        if joints_rad is None:
            return False

        ax_positions = radians_to_ax_positions(joints_rad, self.calibration)
        self.get_logger().info(
            f"IK rad j1..j4={format_joint_debug(joints_rad)} -> AX={ax_positions}"
        )

        if not self.config.move_enabled and not self.bridge.dry_run:
            self.get_logger().error("Refusing to move: pass --move or use --dry-run.")
            return False

        ok = self._send_cm530(ax_positions)
        self.completed_once = self.completed_once or ok
        return ok

    def compute_ik(self, target: PointStamped) -> Optional[list[float]]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.config.group_name
        request.ik_request.pose_stamped = self._pose_from_point(target)
        request.ik_request.ik_link_name = self.config.end_effector_link
        request.ik_request.avoid_collisions = False
        request.ik_request.robot_state.joint_state = self._seed_joint_state()
        request.ik_request.timeout = Duration(sec=int(self.config.ik_timeout_sec))
        request.ik_request.timeout.nanosec = int(
            (self.config.ik_timeout_sec - int(self.config.ik_timeout_sec)) * 1_000_000_000
        )

        future = self.ik_client.call_async(request)
        wait_sec = max(self.config.service_timeout_sec, self.config.ik_timeout_sec + 1.0)
        if not self._wait_for_future(future, wait_sec):
            self.get_logger().error("Timed out waiting for MoveIt IK response.")
            return None

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(f"IK failed, MoveIt error_code={response.error_code.val}")
            return None

        names = list(response.solution.joint_state.name)
        positions = list(response.solution.joint_state.position)
        try:
            return extract_ordered_joint_positions(names, positions, ARM_JOINT_NAMES)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return None

    def _wait_for_future(self, future, timeout_sec: float) -> bool:  # noqa: ANN001
        if self.once:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return future.done() and future.result() is not None

        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done() and future.result() is not None

    @staticmethod
    def _seed_joint_state() -> JointState:
        seed = JointState()
        seed.name = list(ARM_JOINT_NAMES)
        seed.position = [0.0, 0.0, 0.0, 0.0]
        return seed

    def _send_cm530(self, ax_positions: Sequence[int]) -> bool:
        if self.config.send_mode == "trajectory":
            return self.bridge.trajectory([ax_positions], dt_ms=self.config.trajectory_dt_ms)
        return self.bridge.ax(ax_positions)

    def _pose_from_point(self, point: PointStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header = point.header
        pose.header.frame_id = self.config.target_frame
        pose.pose.position.x = point.point.x
        pose.pose.position.y = point.point.y
        pose.pose.position.z = point.point.z

        qx, qy, qz, qw = self.config.quat_xyzw
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _transform_to_target(self, msg: PointStamped) -> Optional[PointStamped]:
        src_frame = msg.header.frame_id or ""
        if not src_frame:
            msg.header.frame_id = self.config.target_frame
            return msg

        if src_frame == self.config.target_frame:
            return msg

        if not self.config.use_tf:
            self.get_logger().warning(
                f"Using point without TF transform: {src_frame} -> {self.config.target_frame}"
            )
            msg.header.frame_id = self.config.target_frame
            return msg

        if self.tf_buffer is None or tf2_geometry_msgs is None:
            self.get_logger().error(
                f"Cannot transform point {src_frame} -> {self.config.target_frame}; TF unavailable."
            )
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.config.target_frame,
                src_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return tf2_geometry_msgs.do_transform_point(msg, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warning(f"TF transform failed ({src_frame} -> {self.config.target_frame}): {exc}")
            return None

    def _map_vision_point(self, msg: PointStamped) -> PointStamped:
        if self.config.vision_map == "none":
            return msg

        if self.config.vision_map != "test06":
            self.get_logger().warning(f"Unknown vision map '{self.config.vision_map}', using raw point.")
            return msg

        mapped = PointStamped()
        mapped.header = msg.header
        base_x, base_y, base_z = self.config.vision_base_xyz
        gain_x, gain_y = self.config.vision_gain_xy
        mapped.point.x = _clamp(base_x + msg.point.y * gain_x, self.config.vision_x_limits)
        mapped.point.y = _clamp(base_y + msg.point.x * gain_y, self.config.vision_y_limits)
        mapped.point.z = _clamp(
            base_z if abs(msg.point.z) < 1e-9 else msg.point.z,
            self.config.vision_z_limits,
        )
        self.get_logger().info(
            f"Vision map test06 {self._format_point(msg)} -> {self._format_point(mapped)}"
        )
        return mapped

    @staticmethod
    def _format_point(msg: PointStamped) -> str:
        p = msg.point
        return (
            f"frame={msg.header.frame_id or 'unknown'} "
            f"(x y z)=({p.x:.4f} {p.y:.4f} {p.z:.4f})"
        )


def _clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def _parse_float2(raw: str | None, default: Sequence[float]) -> tuple[float, float]:
    if raw is None:
        return tuple(float(v) for v in default)  # type: ignore[return-value]
    parts = raw.replace(",", " ").split()
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected 2 comma/space separated values")
    return tuple(float(part) for part in parts)  # type: ignore[return-value]


def _parse_float3(raw: str | None, default: Sequence[float]) -> tuple[float, float, float]:
    if raw is None:
        return tuple(float(v) for v in default)  # type: ignore[return-value]
    parts = raw.replace(",", " ").split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("expected 3 comma/space separated values")
    return tuple(float(part) for part in parts)  # type: ignore[return-value]


def _parse_float4(raw: str | None, default: Sequence[float]) -> tuple[float, float, float, float]:
    if raw is None:
        return tuple(float(v) for v in default)  # type: ignore[return-value]
    parts = raw.replace(",", " ").split()
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected 4 comma/space separated values")
    return tuple(float(part) for part in parts)  # type: ignore[return-value]


def _parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="OpenCV point -> PhantomX MoveIt IK -> CM530 AX command node"
    )
    parser.add_argument("--point-topic", default="/camera/object_point")
    parser.add_argument("--point-service", default="/camera/get_object_point")
    parser.add_argument("--compute-ik-service", default="/compute_ik")
    parser.add_argument("--group-name", default="arm")
    parser.add_argument("--target-frame", default="phantomx_pincher_arm_base_link")
    parser.add_argument("--end-effector-link", default="phantomx_pincher_end_effector")
    parser.add_argument("--quat-xyzw", default="1.0,0.0,0.0,0.0")
    parser.add_argument("--no-tf", action="store_true")
    parser.add_argument("--min-command-interval-sec", type=float, default=0.8)
    parser.add_argument("--ik-timeout-sec", type=float, default=0.5)
    parser.add_argument("--service-timeout-sec", type=float, default=5.0)
    parser.add_argument("--send-mode", choices=("ax", "trajectory"), default="ax")
    parser.add_argument("--trajectory-dt-ms", type=int, default=300)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--serial-timeout-sec", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--move", action="store_true", help="Allow non-dry-run CM530 motion")
    parser.add_argument("--once", action="store_true", help="Pull one point from OpenCV service")
    parser.add_argument("--continuous", action="store_true", help="Subscribe and run continuously")
    parser.add_argument("--test-point", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument(
        "--vision-map",
        choices=("none", "test06"),
        default="none",
        help="Map OpenCV point into robot target space. test06 reuses TEST06.py clamp mapping.",
    )
    parser.add_argument("--vision-base-xyz", default="0.15,0.0,0.03")
    parser.add_argument("--vision-gain-xy", default="1.0,1.0")
    parser.add_argument("--vision-x-limits", default="0.08,0.22")
    parser.add_argument("--vision-y-limits", default="-0.08,0.08")
    parser.add_argument("--vision-z-limits", default="0.03,0.12")
    parser.add_argument("--ax-centers")
    parser.add_argument("--ax-offsets-rad")
    parser.add_argument("--ax-directions")
    parser.add_argument("--ax-min")
    parser.add_argument("--ax-max")
    parser.add_argument("--ax-units-per-rad", type=float, default=None)
    return parser.parse_known_args(argv)


def _make_test_point(values: Sequence[float], frame_id: str) -> PointStamped:
    point = PointStamped()
    point.header.frame_id = frame_id
    point.point.x = float(values[0])
    point.point.y = float(values[1])
    point.point.z = float(values[2])
    return point


def main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = _parse_args(sys.argv[1:] if argv is None else argv)

    if ROS_IMPORT_ERROR is not None:
        print(
            "ROS 2 / MoveIt Python modules are unavailable. "
            "Source ROS 2, MoveIt, and the PhantomX workspace before running this node.\n"
            f"Import error: {ROS_IMPORT_ERROR}",
            file=sys.stderr,
        )
        return 2

    if args.once and args.continuous:
        print("Choose only one of --once or --continuous.", file=sys.stderr)
        return 2

    mode_count = sum(bool(value) for value in (args.once, args.continuous, args.test_point))
    if mode_count == 0:
        args.continuous = True

    try:
        calibration = AxCalibration.from_values(
            centers=_parse_float4(args.ax_centers, (512, 512, 512, 512)),
            offsets_rad=_parse_float4(args.ax_offsets_rad, (0, 0, 0, 0)),
            directions=_parse_float4(args.ax_directions, (1, 1, 1, 1)),
            min_positions=_parse_float4(args.ax_min, (0, 0, 0, 0)),
            max_positions=_parse_float4(args.ax_max, (1023, 1023, 1023, 1023)),
            units_per_rad=args.ax_units_per_rad
            if args.ax_units_per_rad is not None
            else AxCalibration().units_per_rad,
        )
        vision_base_xyz = _parse_float3(args.vision_base_xyz, (0.15, 0.0, 0.03))
        vision_gain_xy = _parse_float2(args.vision_gain_xy, (1.0, 1.0))
        vision_x_limits = _parse_float2(args.vision_x_limits, (0.08, 0.22))
        vision_y_limits = _parse_float2(args.vision_y_limits, (-0.08, 0.08))
        vision_z_limits = _parse_float2(args.vision_z_limits, (0.03, 0.12))
    except (ValueError, argparse.ArgumentTypeError) as exc:
        print(f"Invalid numeric configuration: {exc}", file=sys.stderr)
        return 2

    config = ControllerConfig(
        point_topic=args.point_topic,
        point_service=args.point_service,
        compute_ik_service=args.compute_ik_service,
        group_name=args.group_name,
        target_frame=args.target_frame,
        end_effector_link=args.end_effector_link,
        quat_xyzw=_parse_float4(args.quat_xyzw, (1, 0, 0, 0)),
        use_tf=not args.no_tf,
        min_command_interval_sec=max(0.0, float(args.min_command_interval_sec)),
        ik_timeout_sec=max(0.05, float(args.ik_timeout_sec)),
        service_timeout_sec=max(0.1, float(args.service_timeout_sec)),
        send_mode=args.send_mode,
        trajectory_dt_ms=max(0, int(args.trajectory_dt_ms)),
        move_enabled=bool(args.move),
        vision_map=args.vision_map,
        vision_base_xyz=vision_base_xyz,
        vision_gain_xy=vision_gain_xy,
        vision_x_limits=vision_x_limits,
        vision_y_limits=vision_y_limits,
        vision_z_limits=vision_z_limits,
    )

    if not args.dry_run and not args.move:
        print("Refusing to move without --move. Add --dry-run for calculation-only.", file=sys.stderr)
        return 2

    rclpy.init(args=list(ros_args))
    bridge = CM530Bridge(
        port=args.port,
        baud=int(args.baud),
        timeout=float(args.serial_timeout_sec),
        dry_run=bool(args.dry_run),
    )

    node: OpenCvToCm530Node | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        bridge.open()
        node = OpenCvToCm530Node(
            config=config,
            bridge=bridge,
            calibration=calibration,
            once=bool(args.once or args.test_point),
        )
        if not node.wait_for_ik_service():
            return 1

        if args.test_point:
            ok = node.process_point(_make_test_point(args.test_point, config.target_frame))
            return 0 if ok else 1

        if args.once:
            point = node.request_latest_opencv_point()
            if point is None:
                return 1
            ok = node.process_point(point)
            return 0 if ok else 1

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Shutdown requested by keyboard interrupt.")
        return 0
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        bridge.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
