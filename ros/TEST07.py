#!/usr/bin/env python3
"""
TEST07 - CM-530 communication baseline for the 15 cm530 test firmware.

Firmware baseline:
  CM530test/CM530_ROS_BRIDGE-main/15 cm530 test/APP/src/main.c
  CM530test/CM530_ROS_BRIDGE-main/15 cm530 test/ROS_CM530_INTERFACE_SPEC.txt

Purpose:
  1. Verify the PC <-> CM-530 serial link is alive.
  2. Verify the formal 4-joint AX position protocol.
  3. Provide a small reusable CM530Bridge class for the later OpenCV/ROS flow.

Default port settings follow the project README:
  COM4 @ 57600, 8N1, ASCII, LF command ending.
"""

from __future__ import annotations

import argparse
import itertools
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence

import serial


JOINT_COUNT = 4
HOME = [512, 512, 512, 512]
SAFE_TEST = [520, 512, 512, 512]
DEFAULT_PORT = "COM4"
DEFAULT_BAUD = 57600


class CM530ProtocolError(RuntimeError):
    pass


class CM530TimeoutError(TimeoutError):
    pass


@dataclass
class CommandResult:
    command: str
    ok: bool
    line: Optional[str]
    rx_lines: List[str]


class CM530Bridge:
    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        timeout: float = 2.0,
        dry_run: bool = False,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.dry_run = dry_run
        self.ser: Optional[serial.Serial] = None
        self._traj_ids = itertools.cycle(range(1, 250))

    def open(self) -> None:
        if self.dry_run:
            print("[DRY RUN] Serial is not opened.")
            return

        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=self.timeout,
        )
        time.sleep(0.2)
        self.flush_input()

    def close(self) -> None:
        if self.ser:
            self.ser.close()
            self.ser = None

    def flush_input(self) -> List[str]:
        lines: List[str] = []
        if self.dry_run:
            return lines

        assert self.ser is not None
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            line = self._readline_once()
            if line is None:
                continue
            lines.append(line)
            print(f"RX <- {line}")
        return lines

    def listen(self, seconds: float) -> List[str]:
        print(f"Listening for {seconds:.1f}s...")
        lines: List[str] = []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            line = self._readline_once()
            if line is None:
                continue
            lines.append(line)
            print(f"RX <- {line}")
        if not lines:
            print("RX <- (no data)")
        return lines

    def send(self, command: str, expected_prefixes: Sequence[str]) -> CommandResult:
        print(f"TX -> {command}")

        if self.dry_run:
            simulated = expected_prefixes[0] if expected_prefixes else None
            return CommandResult(command=command, ok=True, line=simulated, rx_lines=[])

        assert self.ser is not None
        self.ser.write((command + "\n").encode("ascii"))
        self.ser.flush()

        rx_lines: List[str] = []
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            line = self._readline_once()
            if line is None:
                continue

            rx_lines.append(line)
            print(f"RX <- {line}")

            if line == "READY":
                continue

            if line.startswith("ERR,"):
                return CommandResult(command=command, ok=False, line=line, rx_lines=rx_lines)

            if any(line.startswith(prefix) for prefix in expected_prefixes):
                return CommandResult(command=command, ok=True, line=line, rx_lines=rx_lines)

        wanted = ", ".join(expected_prefixes)
        print(f"TIMEOUT waiting for: {wanted}")
        return CommandResult(command=command, ok=False, line=None, rx_lines=rx_lines)

    def expect_ok(self, command: str, expected_prefixes: Sequence[str]) -> str:
        result = self.send(command, expected_prefixes)
        if result.ok and result.line is not None:
            return result.line
        if result.line:
            raise CM530ProtocolError(f"{command} failed: {result.line}")
        raise CM530TimeoutError(f"{command} timed out")

    def ping(self) -> bool:
        return self.send("PING", ["PONG"]).ok

    def home(self) -> bool:
        return self.send("HOME", ["OK,HOME"]).ok

    def stop(self) -> bool:
        return self.send("STOP", ["OK,STOP"]).ok

    def ax(self, positions: Sequence[int]) -> bool:
        values = normalize_positions(positions)
        return self.send("AX," + ",".join(str(v) for v in values), ["OK,AX"]).ok

    def trajectory(self, points: Sequence[Sequence[int]], dt_ms: int = 300) -> bool:
        if not points:
            raise ValueError("trajectory needs at least one point")
        if dt_ms < 0:
            raise ValueError("dt_ms must be >= 0")

        traj_id = next(self._traj_ids)
        point_count = len(points)

        if not self.send(f"BEGIN,{traj_id},{JOINT_COUNT},{point_count}", [f"OK,BEGIN,{traj_id}"]).ok:
            return False

        for seq, point in enumerate(points):
            values = normalize_positions(point)
            cmd = f"PT,{seq},{dt_ms}," + ",".join(str(v) for v in values)
            if not self.send(cmd, [f"OK,PT,{seq}"]).ok:
                return False

        return self.send(f"END,{traj_id}", [f"OK,END,{traj_id}"]).ok

    def _readline_once(self) -> Optional[str]:
        if self.dry_run:
            return None

        assert self.ser is not None
        raw = self.ser.readline()
        if not raw:
            return None

        line = raw.decode("ascii", errors="replace").strip()
        return line or None

    def __enter__(self) -> "CM530Bridge":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def normalize_positions(values: Sequence[int]) -> List[int]:
    if len(values) == 1:
        values = list(values) * JOINT_COUNT
    if len(values) != JOINT_COUNT:
        raise ValueError("expected 1 or 4 AX position values")

    normalized = [int(v) for v in values]
    bad = [v for v in normalized if v < 0 or v > 1023]
    if bad:
        raise ValueError(f"AX position out of range 0..1023: {bad}")
    return normalized


def parse_positions(text: str) -> List[int]:
    text = text.strip()
    if text.upper().startswith("AX,"):
        text = text[3:]
    parts = text.replace(",", " ").split()
    if not parts:
        raise ValueError("empty position input")
    return normalize_positions([int(part) for part in parts])


def run_self_test(bridge: CM530Bridge, move: bool) -> bool:
    print("== TEST07 communication self-test ==")
    print(f"Port: {bridge.port} @ {bridge.baud} 8N1")
    print("Firmware expected startup line: READY")

    ok = True
    ok = bridge.ping() and ok

    # Protocol negative tests. These confirm we are talking to the formal parser.
    err_range = bridge.send("AX,2000", ["ERR,RANGE"])
    ok = (err_range.line == "ERR,RANGE") and ok

    err_begin = bridge.send("BEGIN,7,3,1", ["ERR,BAD_ARG"])
    ok = (err_begin.line == "ERR,BAD_ARG") and ok

    if move:
        print("== Motion smoke test ==")
        ok = bridge.home() and ok
        ok = bridge.ax(SAFE_TEST) and ok
        ok = bridge.ax(HOME) and ok
        ok = bridge.trajectory([HOME, SAFE_TEST, HOME], dt_ms=300) and ok
        ok = bridge.stop() and ok
    else:
        print("Motion smoke test skipped. Add --move to send HOME/AX/PT commands.")

    print("TEST07 RESULT:", "PASS" if ok else "FAIL")
    return ok


def run_interactive(bridge: CM530Bridge) -> None:
    print("Interactive mode.")
    print("Commands: ping | home | stop | listen | test | move-test | q")
    print("Positions: 512  or  520 512 512 512  or  AX,520,512,512,512")

    while True:
        try:
            text = input("test07> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        if not text:
            continue

        cmd = text.lower()
        try:
            if cmd in {"q", "quit", "exit"}:
                return
            if cmd == "ping":
                bridge.ping()
                continue
            if cmd == "home":
                bridge.home()
                continue
            if cmd == "stop":
                bridge.stop()
                continue
            if cmd == "listen":
                bridge.listen(5.0)
                continue
            if cmd == "test":
                run_self_test(bridge, move=False)
                continue
            if cmd == "move-test":
                run_self_test(bridge, move=True)
                continue

            positions = parse_positions(text)
            print(f"Target j1/j2/j3/j4 = {positions}")
            bridge.ax(positions)
        except (ValueError, CM530ProtocolError, CM530TimeoutError) as exc:
            print(f"ERROR: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="TEST07 CM-530 communication baseline")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--listen", type=float, metavar="SECONDS", help="Listen for startup/debug lines")
    parser.add_argument("--self-test", action="store_true", help="Run PING and parser checks")
    parser.add_argument("--move", action="store_true", help="Allow self-test to move the arm")
    parser.add_argument("--once", help="Run one command: ping/home/stop or AX positions")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    bridge = CM530Bridge(args.port, args.baud, args.timeout, args.dry_run)

    try:
        with bridge:
            print(f"Opened {args.port} @ {args.baud} 8N1")
            if args.listen is not None:
                bridge.listen(args.listen)
                return 0

            if args.self_test:
                return 0 if run_self_test(bridge, move=args.move) else 1

            if args.once:
                cmd = args.once.strip().lower()
                if cmd == "ping":
                    return 0 if bridge.ping() else 1
                if cmd == "home":
                    return 0 if bridge.home() else 1
                if cmd == "stop":
                    return 0 if bridge.stop() else 1
                return 0 if bridge.ax(parse_positions(args.once)) else 1

            run_interactive(bridge)
            return 0
    except serial.SerialException as exc:
        print(f"SERIAL ERROR: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
