#!/usr/bin/env python3
"""Coordinate-only TCP bridge for Windows sender packets.

This server accepts the same packet protocol used by windows_camera_ros_sender.py:
    Frame packet:   [4-byte big-endian length][JPEG payload]
    Control packet: [4-byte length with high-bit set][JSON payload]

It ignores image frames and only prints converted coordinate tuples from control packets.

Default conversion mode follows a REP103-compatible top-view assumption for `base_link`:
    x: forward (+)  <- image up direction
    y: left (+)     <- image left direction
    z: up (+)       <- configured default_z_m
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time


def recv_exact(conn: socket.socket, size: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = size

    while remaining > 0:
        try:
            data = conn.recv(remaining)
        except socket.timeout:
            continue
        except OSError:
            return None

        if not data:
            return None

        chunks.append(data)
        remaining -= len(data)

    return b"".join(chunks)


def format_xyz_values(x: float, y: float, z: float, decimals: int) -> str:
    value_fmt = f"{{:.{decimals}f}}"
    return (
        f"({value_fmt.format(float(x))} "
        f"{value_fmt.format(float(y))} "
        f"{value_fmt.format(float(z))})"
    )


def clamp_int(value: int, lower: int, upper: int) -> int:
    return min(max(value, lower), upper)


def pixel_to_xyz(
    x_px: int,
    y_px: int,
    width: int,
    height: int,
    scale_px_per_meter: float,
    default_z_m: float,
    axis_mapping: str,
) -> tuple[float, float, float]:
    if axis_mapping == "rep103_base_link":
        # REP103 base_link (right-handed): x forward, y left, z up.
        x_forward_m = ((height / 2.0) - float(y_px)) / scale_px_per_meter
        y_left_m = ((width / 2.0) - float(x_px)) / scale_px_per_meter
        z_up_m = float(default_z_m)
        return x_forward_m, y_left_m, z_up_m

    # Legacy camera-XY mapping fallback.
    x_m = (float(x_px) - (width / 2.0)) / scale_px_per_meter
    y_m = ((height / 2.0) - float(y_px)) / scale_px_per_meter
    z_m = float(default_z_m)
    return x_m, y_m, z_m


def process_control_payload(
    payload: bytes,
    scale_px_per_meter: float,
    default_z_m: float,
    xyz_decimals: int,
    frame_id: str,
    axis_mapping: str,
) -> bool:
    try:
        message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        print(f"[coord][warn] Invalid control payload: {exc}", flush=True)
        return False

    msg_type = str(message.get("type", "")).strip().lower()
    if msg_type == "stop":
        print("[coord][tx] STOP", flush=True)
        return True

    if msg_type != "click_point_px":
        print(f"[coord][warn] Unsupported control message type: {msg_type or '<empty>'}", flush=True)
        return False

    try:
        x_px = int(message["x"])
        y_px = int(message["y"])
        width = int(message["width"])
        height = int(message["height"])
        source = str(message.get("source", "click")).strip().lower() or "click"
    except (KeyError, TypeError, ValueError) as exc:
        print(f"[coord][warn] Invalid click payload fields: {exc}", flush=True)
        return False

    if width <= 0 or height <= 0:
        print("[coord][warn] Invalid frame dimensions in control packet", flush=True)
        return False

    x_px = clamp_int(x_px, 0, width - 1)
    y_px = clamp_int(y_px, 0, height - 1)

    x_m, y_m, z_m = pixel_to_xyz(
        x_px=x_px,
        y_px=y_px,
        width=width,
        height=height,
        scale_px_per_meter=scale_px_per_meter,
        default_z_m=default_z_m,
        axis_mapping=axis_mapping,
    )

    xyz_text = format_xyz_values(x_m, y_m, z_m, xyz_decimals)
    print("[coord][tx]", flush=True)
    print(f"source: {source}", flush=True)
    print(f"frame: {frame_id}", flush=True)
    print(f"(x y z): {xyz_text}", flush=True)
    print("", flush=True)
    return True


def serve(
    host: str,
    port: int,
    max_frame_bytes: int,
    max_control_bytes: int,
    scale_px_per_meter: float,
    default_z_m: float,
    xyz_decimals: int,
    frame_id: str,
    axis_mapping: str,
) -> int:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, int(port)))
    server.listen(1)
    server.settimeout(1.0)

    print(f"[coord] Coordinate bridge listening on {host}:{port}", flush=True)
    print(
        f"[coord] mode=coordinate-only scale={scale_px_per_meter:.1f} "
        f"default_z={default_z_m:.3f} frame_id={frame_id} axis_mapping={axis_mapping}",
        flush=True,
    )

    frame_count = 0
    control_count = 0
    accepted_points = 0
    last_stats_time = time.monotonic()

    try:
        while True:
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                print(f"[coord][error] Accept failed: {exc}", flush=True)
                continue

            print(f"[coord] Client connected: {addr[0]}:{addr[1]}", flush=True)
            with conn:
                conn.settimeout(1.0)
                while True:
                    header = recv_exact(conn, 4)
                    if header is None:
                        break

                    raw_len = struct.unpack("!I", header)[0]
                    is_control = bool(raw_len & 0x80000000)
                    payload_len = raw_len & 0x7FFFFFFF

                    if payload_len <= 0:
                        print("[coord][warn] Invalid packet length=0. Closing client.", flush=True)
                        break

                    if is_control:
                        if payload_len > max_control_bytes:
                            print(
                                f"[coord][warn] Control packet too large={payload_len}, max={max_control_bytes}. Closing client.",
                                flush=True,
                            )
                            break
                    else:
                        if payload_len > max_frame_bytes:
                            print(
                                f"[coord][warn] Frame packet too large={payload_len}, max={max_frame_bytes}. Closing client.",
                                flush=True,
                            )
                            break

                    payload = recv_exact(conn, payload_len)
                    if payload is None:
                        break

                    if is_control:
                        control_count += 1
                        accepted = process_control_payload(
                            payload=payload,
                            scale_px_per_meter=scale_px_per_meter,
                            default_z_m=default_z_m,
                            xyz_decimals=xyz_decimals,
                            frame_id=frame_id,
                            axis_mapping=axis_mapping,
                        )
                        if accepted:
                            accepted_points += 1
                    else:
                        frame_count += 1

                    now = time.monotonic()
                    if (now - last_stats_time) >= 2.0:
                        print(
                            f"[coord] stats frames={frame_count} control={control_count} points={accepted_points}",
                            flush=True,
                        )
                        last_stats_time = now

            print(f"[coord] Client disconnected: {addr[0]}:{addr[1]}", flush=True)

    except KeyboardInterrupt:
        print("[coord] Interrupted by user", flush=True)
        return 0
    finally:
        try:
            server.close()
        except OSError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Coordinate-only bridge (no ROS publish)")
    parser.add_argument("--host", default="0.0.0.0", help="listen host")
    parser.add_argument("--port", type=int, default=5001, help="listen port")
    parser.add_argument("--scale-px-per-meter", type=float, default=100.0)
    parser.add_argument("--default-z-m", type=float, default=0.0)
    parser.add_argument("--xyz-decimals", type=int, default=1)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument(
        "--axis-mapping",
        choices=["rep103_base_link", "camera_xy"],
        default="rep103_base_link",
    )
    parser.add_argument("--max-frame-bytes", type=int, default=4_000_000)
    parser.add_argument("--max-control-bytes", type=int, default=4096)
    args = parser.parse_args()

    scale_px_per_meter = float(args.scale_px_per_meter)
    if scale_px_per_meter <= 0.0:
        scale_px_per_meter = 100.0

    max_frame_bytes = int(args.max_frame_bytes)
    if max_frame_bytes <= 1024:
        max_frame_bytes = 4_000_000

    max_control_bytes = int(args.max_control_bytes)
    if max_control_bytes <= 64:
        max_control_bytes = 4096

    xyz_decimals = int(args.xyz_decimals)
    if xyz_decimals < 0:
        xyz_decimals = 0
    if xyz_decimals > 6:
        xyz_decimals = 6

    return serve(
        host=str(args.host),
        port=int(args.port),
        max_frame_bytes=max_frame_bytes,
        max_control_bytes=max_control_bytes,
        scale_px_per_meter=scale_px_per_meter,
        default_z_m=float(args.default_z_m),
        xyz_decimals=xyz_decimals,
        frame_id=str(args.frame_id),
        axis_mapping=str(args.axis_mapping),
    )


if __name__ == "__main__":
    raise SystemExit(main())
