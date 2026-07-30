"""UDP pose packet helpers for PC → robot Tianyi bridge."""

from __future__ import annotations

import json
from typing import Any


def encode_pose_packet(
    *,
    t: float,
    left: dict[str, Any] | None,
    right: dict[str, Any] | None,
) -> bytes:
    return json.dumps(
        {"t": float(t), "left": left, "right": right},
        separators=(",", ":"),
    ).encode("utf-8")


def decode_pose_packet(data: bytes) -> dict[str, Any]:
    payload = json.loads(data.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("pose packet must be object")
    return payload


def side_pose(
    *,
    active: bool,
    xyz: list[float] | tuple[float, float, float],
    quat_wxyz: list[float] | tuple[float, float, float, float],
) -> dict[str, Any]:
    return {
        "active": bool(active),
        "xyz": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
        "quat_wxyz": [
            float(quat_wxyz[0]),
            float(quat_wxyz[1]),
            float(quat_wxyz[2]),
            float(quat_wxyz[3]),
        ],
    }
