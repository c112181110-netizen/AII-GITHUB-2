#!/usr/bin/env python3
"""Convert PhantomX/MoveIt joint radians into AX-12A position integers.

The CM530 firmware receives only AX position integers. This module keeps the
calibration knobs local to ROS so TEST07.py and the CM530 protocol stay intact.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


JOINT_NAMES = (
    "phantomx_pincher_arm_shoulder_pan_joint",
    "phantomx_pincher_arm_shoulder_lift_joint",
    "phantomx_pincher_arm_elbow_flex_joint",
    "phantomx_pincher_arm_wrist_flex_joint",
)

AX_MIN = 0
AX_MAX = 1023
AX_CENTER = 512
AX_UNITS_PER_RAD = 1023.0 / math.radians(300.0)


def _as_len4(values: Sequence[float] | None, default: float) -> tuple[float, float, float, float]:
    if values is None:
        return (default, default, default, default)
    if len(values) != 4:
        raise ValueError("expected exactly 4 values")
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class AxCalibration:
    centers: tuple[float, float, float, float] = (AX_CENTER, AX_CENTER, AX_CENTER, AX_CENTER)
    offsets_rad: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    directions: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    min_positions: tuple[float, float, float, float] = (AX_MIN, AX_MIN, AX_MIN, AX_MIN)
    max_positions: tuple[float, float, float, float] = (AX_MAX, AX_MAX, AX_MAX, AX_MAX)
    units_per_rad: float = AX_UNITS_PER_RAD

    @classmethod
    def from_values(
        cls,
        centers: Sequence[float] | None = None,
        offsets_rad: Sequence[float] | None = None,
        directions: Sequence[float] | None = None,
        min_positions: Sequence[float] | None = None,
        max_positions: Sequence[float] | None = None,
        units_per_rad: float = AX_UNITS_PER_RAD,
    ) -> "AxCalibration":
        calibration = cls(
            centers=_as_len4(centers, AX_CENTER),
            offsets_rad=_as_len4(offsets_rad, 0.0),
            directions=_as_len4(directions, 1.0),
            min_positions=_as_len4(min_positions, AX_MIN),
            max_positions=_as_len4(max_positions, AX_MAX),
            units_per_rad=float(units_per_rad),
        )
        calibration.validate()
        return calibration

    def validate(self) -> None:
        if self.units_per_rad <= 0.0:
            raise ValueError("units_per_rad must be positive")

        for idx, direction in enumerate(self.directions):
            if direction == 0.0:
                raise ValueError(f"direction[{idx}] must not be zero")

        for idx, (min_pos, max_pos) in enumerate(zip(self.min_positions, self.max_positions)):
            if min_pos < AX_MIN or max_pos > AX_MAX or min_pos > max_pos:
                raise ValueError(
                    f"invalid AX clamp for joint {idx}: min={min_pos}, max={max_pos}"
                )


def clamp_int(value: float, lower: float, upper: float) -> int:
    return int(round(min(max(value, lower), upper)))


def radians_to_ax_positions(
    joint_radians: Sequence[float],
    calibration: AxCalibration | None = None,
) -> list[int]:
    """Convert 4 MoveIt joint radians to CM530 j1..j4 AX positions."""

    if len(joint_radians) != 4:
        raise ValueError("expected exactly 4 joint radians")

    calibration = calibration or AxCalibration()
    calibration.validate()

    positions: list[int] = []
    for idx, rad in enumerate(joint_radians):
        raw = (
            calibration.centers[idx]
            + calibration.directions[idx]
            * (float(rad) + calibration.offsets_rad[idx])
            * calibration.units_per_rad
        )
        positions.append(
            clamp_int(raw, calibration.min_positions[idx], calibration.max_positions[idx])
        )
    return positions


def extract_ordered_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
    ordered_names: Sequence[str] = JOINT_NAMES,
) -> list[float]:
    """Return joint positions ordered for CM530 j1..j4."""

    by_name = {name: float(value) for name, value in zip(names, positions)}
    missing = [name for name in ordered_names if name not in by_name]
    if missing:
        raise ValueError("IK response missing joints: " + ", ".join(missing))
    return [by_name[name] for name in ordered_names]


def format_joint_debug(values: Iterable[float]) -> str:
    return "[" + ", ".join(f"{float(value):.4f}" for value in values) + "]"
