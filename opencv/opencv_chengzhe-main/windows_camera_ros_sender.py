#!/usr/bin/env python3
"""Capture frames on Windows and send them to WSL bridge over TCP.

Protocol:
	Frame packet:   [4-byte big-endian length][JPEG payload]
	Control packet: [4-byte length with high-bit set][JSON payload]
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from typing import Iterable

import cv2
import numpy as np


def _candidate_backends() -> list[tuple[str, int]]:
	backends: list[tuple[str, int]] = []

	if hasattr(cv2, "CAP_DSHOW"):
		backends.append(("DSHOW", cv2.CAP_DSHOW))
	if hasattr(cv2, "CAP_MSMF"):
		backends.append(("MSMF", cv2.CAP_MSMF))

	backends.append(("ANY", cv2.CAP_ANY))
	return backends


def open_camera(index: int, width: int, height: int, fps: int) -> tuple[cv2.VideoCapture, str]:
	attempts: list[str] = []

	for backend_name, backend in _candidate_backends():
		cap = cv2.VideoCapture(index, backend)
		if not cap.isOpened():
			attempts.append(f"backend={backend_name}: open failed")
			cap.release()
			continue

		cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
		cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
		cap.set(cv2.CAP_PROP_FPS, float(max(1, fps)))
		cap.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)

		ok, frame = cap.read()
		if ok and frame is not None:
			return cap, backend_name

		attempts.append(f"backend={backend_name}: first frame failed")
		cap.release()

	attempt_text = " | ".join(attempts) if attempts else "no backend attempts"
	raise RuntimeError(f"Cannot open camera index {index}. Attempts: {attempt_text}")


def scan_camera_indices(max_index: int = 10) -> list[tuple[int, str]]:
	found: list[tuple[int, str]] = []

	print("[scan] Probing camera indices...")
	for index in range(max_index):
		opened_backends: list[str] = []
		for backend_name, backend in _candidate_backends():
			cap = cv2.VideoCapture(index, backend)
			if cap.isOpened():
				ok, frame = cap.read()
				if ok and frame is not None:
					opened_backends.append(backend_name)
			cap.release()

		if opened_backends:
			uniq = sorted(set(opened_backends))
			print(f"[scan] index={index}: OK via {', '.join(uniq)}")
			found.append((index, uniq[0]))

	if not found:
		print("[scan] No available camera index detected.")

	return found


def _test_pattern_frames(width: int, height: int, fps: int) -> Iterable[np.ndarray]:
	tick = 0
	period = 1.0 / float(max(1, fps))

	while True:
		frame = np.zeros((height, width, 3), dtype=np.uint8)
		frame[:, :] = (30, 30, 30)

		cv2.putText(
			frame,
			"Windows Test Pattern",
			(20, 42),
			cv2.FONT_HERSHEY_SIMPLEX,
			1.0,
			(0, 255, 0),
			2,
			cv2.LINE_AA,
		)

		cx = int(width * (0.2 + 0.6 * ((np.sin(tick / 15.0) + 1.0) / 2.0)))
		cy = int(height * (0.2 + 0.6 * ((np.cos(tick / 18.0) + 1.0) / 2.0)))
		cv2.circle(frame, (cx, cy), 35, (0, 0, 255), -1)
		cv2.circle(frame, (cx, cy), 42, (0, 255, 255), 2)

		cv2.putText(
			frame,
			f"frame={tick}",
			(20, height - 20),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.6,
			(220, 220, 220),
			1,
			cv2.LINE_AA,
		)

		tick += 1
		yield frame
		time.sleep(period)


def _connect(host: str, port: int, timeout: float) -> socket.socket:
	sock = socket.create_connection((host, port), timeout=timeout)
	sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
	return sock


def _make_control_packet(message: dict[str, object]) -> bytes:
	payload = json.dumps(
		message,
		separators=(",", ":"),
		ensure_ascii=True,
	).encode("utf-8")
	return struct.pack("!I", len(payload) | 0x80000000) + payload


def _detect_black_object(
	frame: np.ndarray,
	black_v_max: int,
	min_black_area: float,
) -> tuple[tuple[int, int, int, int] | None, tuple[int, int] | None, float]:
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_black = np.array([0, 0, 0], dtype=np.uint8)
	upper_black = np.array([180, 255, int(black_v_max)], dtype=np.uint8)
	mask = cv2.inRange(hsv, lower_black, upper_black)

	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	frame_h, frame_w = frame.shape[:2]
	max_frame_area = float(frame_h * frame_w)

	best_bbox: tuple[int, int, int, int] | None = None
	best_area = 0.0

	for contour in contours:
		area = float(cv2.contourArea(contour))
		if area < min_black_area:
			continue

		if area >= (max_frame_area * 0.90):
			continue

		x, y, w, h = cv2.boundingRect(contour)

		# Skip background-like contours attached to image edges.
		if x <= 1 or y <= 1 or (x + w) >= (frame_w - 1) or (y + h) >= (frame_h - 1):
			continue

		if area > best_area:
			best_area = area
			best_bbox = (x, y, w, h)

	if best_bbox is None:
		return None, None, 0.0

	x, y, w, h = best_bbox
	center = (x + (w // 2), y + (h // 2))
	return best_bbox, center, best_area


def main() -> int:
	parser = argparse.ArgumentParser(description="Windows camera sender for WSL bridge")
	parser.add_argument("--host", default="127.0.0.1", help="bridge host (default: 127.0.0.1)")
	parser.add_argument("--port", type=int, default=5001, help="bridge port (default: 5001)")
	parser.add_argument("--index", type=int, default=0, help="camera index (default: 0)")
	parser.add_argument("--width", type=int, default=1280, help="capture width (default: 1280)")
	parser.add_argument("--height", type=int, default=720, help="capture height (default: 720)")
	parser.add_argument("--fps", type=int, default=30, help="capture FPS target (default: 30)")
	parser.add_argument("--send-hz", type=float, default=30.0, help="max send rate over TCP")
	parser.add_argument("--jpeg-quality", type=int, default=85, help="JPEG quality 1-100")
	parser.add_argument("--preview", action="store_true", help="show local preview window")
	parser.add_argument("--source", choices=["camera", "test"], default="camera", help="frame source")
	parser.add_argument("--scan", action="store_true", help="scan camera indices and exit")
	parser.add_argument("--max-index", type=int, default=10, help="max index for --scan")
	parser.add_argument("--reconnect-seconds", type=float, default=2.0, help="reconnect backoff")
	parser.add_argument("--connect-timeout", type=float, default=5.0, help="socket connect timeout")
	parser.add_argument("--duration-seconds", type=float, default=0.0, help="0 means run forever")
	parser.add_argument(
		"--coord-only",
		action="store_true",
		help="send only coordinate control packets, skip JPEG frame packets",
	)
	parser.add_argument("--detect-black", action="store_true", help="enable black object detection")
	parser.add_argument(
		"--black-detect-hz",
		type=float,
		default=8.0,
		help="max publish rate for black target point",
	)
	parser.add_argument(
		"--black-v-max",
		type=int,
		default=55,
		help="HSV V upper bound for black detection (0-255)",
	)
	parser.add_argument(
		"--min-black-area",
		type=float,
		default=600.0,
		help="minimum contour area for black target",
	)
	args = parser.parse_args()

	if args.scan:
		found = scan_camera_indices(max_index=max(1, args.max_index))
		return 0 if found else 1

	jpeg_quality = int(min(100, max(1, args.jpeg_quality)))
	coord_only_mode = bool(args.coord_only)
	send_hz = float(args.send_hz)
	if send_hz <= 0.0:
		send_hz = float(max(1, args.fps))
	send_interval = 1.0 / send_hz

	detect_black_enabled = bool(args.detect_black)
	black_detect_hz = float(args.black_detect_hz)
	if black_detect_hz <= 0.0:
		black_detect_hz = 8.0
	black_detect_interval = 1.0 / black_detect_hz
	black_v_max = int(min(255, max(0, args.black_v_max)))
	min_black_area = float(args.min_black_area)
	if min_black_area <= 0.0:
		min_black_area = 600.0

	cap: cv2.VideoCapture | None = None
	backend_name = "test"
	test_frames = None
	preview_enabled = bool(args.preview)
	preview_window_name = "Windows Sender Preview (press q to quit)"
	pending_clicks: list[tuple[int, int, int, int]] = []
	preview_frame_size = (int(args.width), int(args.height))

	if args.source == "camera":
		try:
			cap, backend_name = open_camera(args.index, args.width, args.height, args.fps)
		except RuntimeError as exc:
			print(f"[sender][error] {exc}")
			print("[hint] If camera was attached to WSL before, run admin PowerShell:")
			print("[hint]   usbipd detach --busid <BUSID>")
			print("[hint]   usbipd unbind --busid <BUSID>")
			print("[hint] Try scan mode: python windows_camera_ros_sender.py --scan")
			return 1

		print(f"[sender] Camera opened index={args.index} backend={backend_name}")
	else:
		test_frames = _test_pattern_frames(args.width, args.height, args.fps)
		print("[sender] Using synthetic test pattern source")

	if coord_only_mode:
		print("[sender] Coordinate-only mode ON (JPEG frame packets disabled)")

	if preview_enabled:
		try:
			cv2.namedWindow(preview_window_name, cv2.WINDOW_NORMAL)
			cv2.resizeWindow(preview_window_name, int(args.width), int(args.height))

			def _on_preview_mouse(event: int, x: int, y: int, flags: int, param: object) -> None:
				del flags, param
				if event != cv2.EVENT_LBUTTONDOWN:
					return

				frame_w, frame_h = preview_frame_size
				if frame_w <= 0 or frame_h <= 0:
					return

				click_x = int(min(max(x, 0), frame_w - 1))
				click_y = int(min(max(y, 0), frame_h - 1))
				pending_clicks.append((click_x, click_y, frame_w, frame_h))
				print(
					f"[sender] Click queued px=({click_x},{click_y}) "
					f"frame={frame_w}x{frame_h}"
				)

			cv2.setMouseCallback(preview_window_name, _on_preview_mouse)
		except cv2.error as exc:
			print(f"[sender][warn] Preview disabled: {exc}")
			preview_enabled = False

	started = time.perf_counter()
	frame_count = 0
	last_log_time = started
	next_send_time = started
	last_black_publish_time = 0.0

	sock: socket.socket | None = None

	try:
		while True:
			if args.duration_seconds > 0 and (time.perf_counter() - started) >= args.duration_seconds:
				print("[sender] Duration reached; stopping.")
				break

			if sock is None:
				try:
					sock = _connect(args.host, args.port, timeout=args.connect_timeout)
					print(f"[sender] Connected to bridge {args.host}:{args.port}")
				except OSError as exc:
					print(
						f"[sender][warn] Connect failed: {exc}; retrying in {args.reconnect_seconds:.1f}s"
					)
					time.sleep(max(0.1, args.reconnect_seconds))
					continue

			now = time.perf_counter()
			remaining = next_send_time - now
			if remaining > 0:
				time.sleep(remaining)
			now = time.perf_counter()
			next_send_time = max(next_send_time + send_interval, now)

			if args.source == "camera":
				assert cap is not None
				ok, frame = cap.read()
				if not ok or frame is None:
					print("[sender][warn] Camera frame read failed")
					time.sleep(0.05)
					continue
			else:
				assert test_frames is not None
				frame = next(test_frames)

			frame_h, frame_w = frame.shape[:2]
			if preview_enabled:
				preview_frame_size = (frame_w, frame_h)

			detected_bbox: tuple[int, int, int, int] | None = None
			detected_center: tuple[int, int] | None = None
			detected_area = 0.0
			if detect_black_enabled:
				detected_bbox, detected_center, detected_area = _detect_black_object(
					frame=frame,
					black_v_max=black_v_max,
					min_black_area=min_black_area,
				)

			if not coord_only_mode:
				ok, encoded = cv2.imencode(
					".jpg",
					frame,
					[int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
				)
				if not ok:
					print("[sender][warn] JPEG encode failed")
					continue

				payload = encoded.tobytes()
				packet = struct.pack("!I", len(payload)) + payload

				try:
					sock.sendall(packet)
				except OSError as exc:
					print(f"[sender][warn] Send failed: {exc}; reconnecting...")
					try:
						sock.close()
					except OSError:
						pass
					sock = None
					continue

			while pending_clicks:
				click_x, click_y, click_w, click_h = pending_clicks.pop(0)
				click_packet = _make_control_packet(
					{
						"type": "click_point_px",
						"source": "click",
						"x": click_x,
						"y": click_y,
						"width": click_w,
						"height": click_h,
					}
				)
				try:
					sock.sendall(click_packet)
					print(
						f"[sender] Click coordinate sent "
						f"px=({click_x},{click_y})"
					)
				except OSError as exc:
					print(f"[sender][warn] Click send failed: {exc}; reconnecting...")
					pending_clicks.insert(0, (click_x, click_y, click_w, click_h))
					try:
						sock.close()
					except OSError:
						pass
					sock = None
					break

			if detect_black_enabled and detected_center is not None and sock is not None:
				now = time.perf_counter()
				if (now - last_black_publish_time) >= black_detect_interval:
					cx, cy = detected_center
					black_packet = _make_control_packet(
						{
							"type": "click_point_px",
							"source": "black_detect",
							"x": int(cx),
							"y": int(cy),
							"width": int(frame_w),
							"height": int(frame_h),
						}
					)
					try:
						sock.sendall(black_packet)
						last_black_publish_time = now
					except OSError as exc:
						print(f"[sender][warn] Black target send failed: {exc}; reconnecting...")
						try:
							sock.close()
						except OSError:
							pass
						sock = None

			frame_count += 1

			if preview_enabled:
				preview = frame.copy()
				cv2.putText(
					preview,
					f"{preview.shape[1]}x{preview.shape[0]} send_hz={send_hz:.1f}",
					(12, 28),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.7,
					(0, 255, 0),
					2,
					cv2.LINE_AA,
				)
				cv2.putText(
					preview,
					"Left click: send coordinate packet",
					(12, 56),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.6,
					(0, 220, 255),
					2,
					cv2.LINE_AA,
				)
				status_text = f"Black detect: {'ON' if detect_black_enabled else 'OFF'} (press b)"
				cv2.putText(
					preview,
					status_text,
					(12, 84),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.6,
					(60, 255, 255) if detect_black_enabled else (180, 180, 180),
					2,
					cv2.LINE_AA,
				)
				if detect_black_enabled and detected_bbox is not None and detected_center is not None:
					x, y, w, h = detected_bbox
					cx, cy = detected_center
					cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 255), 2)
					cv2.circle(preview, (cx, cy), 5, (0, 0, 255), -1)
					cv2.putText(
						preview,
						f"black target px=({cx},{cy}) area={detected_area:.0f}",
						(12, 112),
						cv2.FONT_HERSHEY_SIMPLEX,
						0.55,
						(0, 255, 255),
						2,
						cv2.LINE_AA,
					)
				elif detect_black_enabled:
					cv2.putText(
						preview,
						"black target: not found",
						(12, 112),
						cv2.FONT_HERSHEY_SIMPLEX,
						0.55,
						(0, 200, 255),
						2,
						cv2.LINE_AA,
					)
				cv2.imshow(preview_window_name, preview)
				key = cv2.waitKey(1) & 0xFF
				if key in (ord("q"), 27):
					print("[sender] Preview close requested by user")
					break
				if key in (ord("b"), ord("B")):
					detect_black_enabled = not detect_black_enabled
					state = "ON" if detect_black_enabled else "OFF"
					print(f"[sender] Black detection toggled: {state}")

			now = time.perf_counter()
			if now - last_log_time >= 1.0:
				elapsed = max(now - started, 1e-6)
				fps = frame_count / elapsed
				print(
					f"[sender] sent_frames={frame_count} avg_fps={fps:.2f} "
					f"backend={backend_name} target_send_hz={send_hz:.2f}"
				)
				last_log_time = now

	except KeyboardInterrupt:
		print("[sender] Interrupted by user")
	finally:
		if cap is not None:
			cap.release()
		if preview_enabled:
			cv2.destroyAllWindows()
		if sock is not None:
			try:
				sock.close()
			except OSError:
				pass

	return 0


if __name__ == "__main__":
	raise SystemExit(main())
