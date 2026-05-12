#!/usr/bin/env python3
"""ROS 2 node that integrates camera point input, MoveIt2, and CM-530 serial control.

Flow:
1) Subscribe PointStamped from /camera/object_point.
2) (Optional) transform point to target frame via TF.
3) Ask MoveIt2 to plan/execute pose goal.
4) Intercept JointTrajectory and relay it to CM-530 firmware with ACK handshake.
5) If MoveIt2 is unavailable, fall back to sending target coordinate command over serial.
"""

from __future__ import annotations

import time
from threading import Lock, Thread

import geometry_msgs.msg
import rclpy
import tf2_geometry_msgs
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
except Exception:  # noqa: BLE001
    serial = None

try:
    from pymoveit2 import MoveIt2
    from pymoveit2.robots import phantomx_pincher
except Exception:  # noqa: BLE001
    MoveIt2 = None
    phantomx_pincher = None


class Cm530ArmController:
    """CM-530 serial bridge with simple ACK-based protocol."""

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        timeout: float = 2.0,
        joint_count: int = 4,
    ) -> None:
        self.ser = None
        self.timeout = max(0.1, float(timeout))
        self.traj_id_counter = 0
        self.joint_count = max(1, int(joint_count))
        self._io_lock = Lock()

        if serial is None:
            print("[cm530][warn] pyserial not installed, serial bridge is disabled.")
            return

        try:
            self.ser = serial.Serial(port, baudrate, timeout=self.timeout)
            print(f"[cm530] Connected serial: {port} @ {baudrate}")
        except Exception as exc:  # noqa: BLE001
            print(f"[cm530][warn] Cannot open serial port: {exc}")
            self.ser = None

    def is_ready(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def close(self) -> None:
        if self.ser is None:
            return

        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    @staticmethod
    def _format_xyz_tuple(point_msg: geometry_msgs.msg.PointStamped) -> str:
        p = point_msg.point
        return f"(X y z) ({float(p.x):.1f} {float(p.y):.1f} {float(p.z):.1f})"

    def _send_and_wait(self, command_str: str, expected_ack_prefix: str) -> bool:
        if not self.is_ready():
            return False

        with self._io_lock:
            try:
                assert self.ser is not None
                if self.ser.in_waiting > 0:
                    self.ser.reset_input_buffer()

                full_cmd = f"{command_str}\n"
                self.ser.write(full_cmd.encode("ascii"))
                self.ser.flush()
                print(f"[cm530][tx] {command_str}")
            except Exception as exc:  # noqa: BLE001
                print(f"[cm530][error] Serial write failed: {exc}")
                return False

            started = time.time()
            while (time.time() - started) < self.timeout:
                try:
                    if self.ser.in_waiting <= 0:
                        continue

                    response = self.ser.readline().decode("ascii", errors="ignore").strip()
                    if response:
                        print(f"[cm530][rx] {response}")
                    if response.startswith(expected_ack_prefix):
                        return True
                    if response.startswith("ERR"):
                        print(f"[cm530][warn] Firmware error: {response}")
                        return False
                except Exception as exc:  # noqa: BLE001
                    print(f"[cm530][error] Serial read failed: {exc}")
                    return False

        print(f"[cm530][error] Timeout waiting for ACK '{expected_ack_prefix}'")
        return False

    def send_stop(self) -> bool:
        return self._send_and_wait("STOP", "OK,STOP")

    def send_target_coordinate(self, point_msg: geometry_msgs.msg.PointStamped) -> bool:
        """Fallback mode: send target coordinate directly when no trajectory is available."""
        x = float(point_msg.point.x)
        y = float(point_msg.point.y)
        z = float(point_msg.point.z)
        cmd = f"TARGET,{x:.4f},{y:.4f},{z:.4f}"
        ok = self._send_and_wait(cmd, "OK,TARGET")
        if ok:
            print(f"[cm530] TARGET sent {self._format_xyz_tuple(point_msg)}")
        return ok

    def execute_trajectory(self, joint_trajectory: JointTrajectory) -> bool:
        if not self.is_ready():
            print("[cm530] Serial not connected, trajectory relay skipped.")
            return False

        points = joint_trajectory.points
        point_count = len(points)
        if point_count == 0:
            print("[cm530][warn] Empty trajectory, nothing to send.")
            return False

        self.traj_id_counter += 1
        traj_id = self.traj_id_counter

        print(f"[cm530] Sending trajectory id={traj_id}, points={point_count}")

        cmd_begin = f"BEGIN,{traj_id},{self.joint_count},{point_count}"
        if not self._send_and_wait(cmd_begin, f"OK,BEGIN,{traj_id}"):
            return False

        prev_time_ms = 0.0
        for idx, point in enumerate(points):
            rads = list(point.positions[: self.joint_count])
            rad_csv = ",".join(f"{rad:.4f}" for rad in rads)

            current_time_ms = (
                float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1e9
            ) * 1000.0
            dt_ms = 0 if idx == 0 else int(round(current_time_ms - prev_time_ms))
            prev_time_ms = current_time_ms

            cmd_pt = f"PT,{idx},{dt_ms},{rad_csv}"
            if not self._send_and_wait(cmd_pt, f"OK,PT,{idx}"):
                self.send_stop()
                return False

        cmd_end = f"END,{traj_id}"
        if not self._send_and_wait(cmd_end, f"OK,END,{traj_id}"):
            return False

        print(f"[cm530] Trajectory {traj_id} transmitted successfully.")
        return True


class CameraPointMoveItCm530Node(Node):
    def __init__(self) -> None:
        super().__init__("camera_point_moveit_cm530")

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter("point_topic", "/camera/object_point")
        self.declare_parameter("trajectory_topic", "/arm_controller/joint_trajectory")
        self.declare_parameter("target_frame", "arm_base_link")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("min_command_interval_sec", 0.8)
        self.declare_parameter("trajectory_timeout_sec", 6.0)
        self.declare_parameter("z_offset_m", 0.03)
        self.declare_parameter("min_z_m", 0.01)
        self.declare_parameter("max_z_m", 0.30)
        self.declare_parameter("cartesian", False)
        self.declare_parameter("use_moveit", True)
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baudrate", 57600)
        self.declare_parameter("serial_timeout_sec", 2.0)

        self.point_topic = self.get_parameter("point_topic").get_parameter_value().string_value
        self.trajectory_topic = (
            self.get_parameter("trajectory_topic").get_parameter_value().string_value
        )
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.use_tf_transform = (
            self.get_parameter("use_tf_transform").get_parameter_value().bool_value
        )
        self.min_command_interval_sec = (
            self.get_parameter("min_command_interval_sec").get_parameter_value().double_value
        )
        self.trajectory_timeout_sec = (
            self.get_parameter("trajectory_timeout_sec").get_parameter_value().double_value
        )
        self.z_offset_m = self.get_parameter("z_offset_m").get_parameter_value().double_value
        self.min_z_m = self.get_parameter("min_z_m").get_parameter_value().double_value
        self.max_z_m = self.get_parameter("max_z_m").get_parameter_value().double_value
        self.cartesian = self.get_parameter("cartesian").get_parameter_value().bool_value
        self.use_moveit = self.get_parameter("use_moveit").get_parameter_value().bool_value
        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.serial_baudrate = (
            self.get_parameter("serial_baudrate").get_parameter_value().integer_value
        )
        self.serial_timeout_sec = (
            self.get_parameter("serial_timeout_sec").get_parameter_value().double_value
        )

        if self.min_command_interval_sec < 0.0:
            self.min_command_interval_sec = 0.0
        if self.trajectory_timeout_sec <= 0.0:
            self.trajectory_timeout_sec = 6.0
        if self.max_z_m < self.min_z_m:
            self.max_z_m = self.min_z_m

        self.cm530 = Cm530ArmController(
            port=self.serial_port,
            baudrate=int(self.serial_baudrate),
            timeout=float(self.serial_timeout_sec),
            joint_count=4,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._state_lock = Lock()
        self._last_goal_wall_time = 0.0
        self._awaiting_trajectory_until = 0.0

        self.moveit2 = None
        self.moveit_ready = self._initialize_moveit2()

        self.point_subscription = self.create_subscription(
            geometry_msgs.msg.PointStamped,
            self.point_topic,
            self.point_callback,
            10,
            callback_group=self.callback_group,
        )

        self.trajectory_subscription = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.trajectory_callback,
            10,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "Camera point -> MoveIt2 -> CM530 node started\n"
            f"  point_topic: {self.point_topic}\n"
            f"  trajectory_topic: {self.trajectory_topic}\n"
            f"  target_frame: {self.target_frame}\n"
            f"  moveit_enabled: {self.moveit_ready}\n"
            f"  serial_ready: {self.cm530.is_ready()}"
        )

    def _initialize_moveit2(self) -> bool:
        if not self.use_moveit:
            self.get_logger().warning("use_moveit=false, will send TARGET over serial directly.")
            return False

        if MoveIt2 is None or phantomx_pincher is None:
            self.get_logger().warning(
                "pymoveit2 is unavailable, fallback to TARGET serial command mode."
            )
            return False

        try:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=phantomx_pincher.joint_names(),
                base_link_name=phantomx_pincher.base_link_name(),
                end_effector_name=phantomx_pincher.end_effector_name(),
                group_name=phantomx_pincher.MOVE_GROUP_ARM,
                callback_group=self.callback_group,
            )
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"MoveIt2 initialization failed, fallback to serial TARGET mode: {exc}"
            )
            self.moveit2 = None
            return False

    def _transform_point_to_target(
        self,
        msg: geometry_msgs.msg.PointStamped,
    ) -> geometry_msgs.msg.PointStamped | None:
        if not self.use_tf_transform:
            return msg

        src_frame = msg.header.frame_id or ""
        if not src_frame or src_frame == self.target_frame:
            return msg

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                src_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            return tf2_geometry_msgs.do_transform_point(msg, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warning(f"TF transform failed ({src_frame} -> {self.target_frame}): {exc}")
            return None

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    def point_callback(self, msg: geometry_msgs.msg.PointStamped) -> None:
        point_in_target = self._transform_point_to_target(msg)
        if point_in_target is None:
            return

        now = time.monotonic()
        with self._state_lock:
            if (now - self._last_goal_wall_time) < self.min_command_interval_sec:
                return
            self._last_goal_wall_time = now

        p = point_in_target.point
        self.get_logger().info(
            f"Target point frame={point_in_target.header.frame_id or 'unknown'} "
            f"(X y z) ({p.x:.1f} {p.y:.1f} {p.z:.1f})"
        )

        if self.moveit_ready and self.moveit2 is not None:
            with self._state_lock:
                self._awaiting_trajectory_until = time.monotonic() + self.trajectory_timeout_sec

            Thread(
                target=self._plan_and_execute,
                args=(point_in_target,),
                daemon=True,
            ).start()
            return

        self.cm530.send_target_coordinate(point_in_target)

    def _plan_and_execute(self, target: geometry_msgs.msg.PointStamped) -> None:
        if self.moveit2 is None:
            self.cm530.send_target_coordinate(target)
            return

        x = float(target.point.x)
        y = float(target.point.y)
        z = self._clamp(float(target.point.z) + self.z_offset_m, self.min_z_m, self.max_z_m)

        try:
            self.moveit2.move_to_pose(
                position=[x, y, z],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                cartesian=self.cartesian,
            )
            self.moveit2.wait_until_executed()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"MoveIt2 execution failed: {exc}")
            self.cm530.send_target_coordinate(target)
            with self._state_lock:
                self._awaiting_trajectory_until = 0.0
            return

        with self._state_lock:
            waiting = time.monotonic() <= self._awaiting_trajectory_until

        if waiting:
            self.get_logger().warning(
                "MoveIt2 completed but no trajectory callback observed in time, "
                "fallback to TARGET command."
            )
            self.cm530.send_target_coordinate(target)
            with self._state_lock:
                self._awaiting_trajectory_until = 0.0

    def trajectory_callback(self, msg: JointTrajectory) -> None:
        if not self.cm530.is_ready():
            return

        now = time.monotonic()
        with self._state_lock:
            if now > self._awaiting_trajectory_until:
                return
            self._awaiting_trajectory_until = 0.0

        if len(msg.points) == 0:
            self.get_logger().warning("Received empty JointTrajectory.")
            return

        ok = self.cm530.execute_trajectory(msg)
        if not ok:
            self.get_logger().warning("Failed to relay JointTrajectory to CM-530.")

    def destroy_node(self) -> bool:
        self.cm530.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = CameraPointMoveItCm530Node()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested by keyboard interrupt.")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
