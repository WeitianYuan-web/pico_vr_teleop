#!/usr/bin/env python3
"""遥操作状态 UDP 桥：control 脚本发送，publisher 节点接收。"""

from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from typing import Any

DEFAULT_UDP_HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 17981
HAND_REG_TO_RAD = 0.1 * 3.141592653589793 / 180.0


@dataclass
class SideTeleopState:
    arm_joints: list[float]
    end_pose: dict[str, float]
    hand_joints: list[float]
    arm_valid: bool = False
    hand_valid: bool = False


@dataclass
class TeleopStateSnapshot:
    stamp: float
    left: SideTeleopState | None = None
    right: SideTeleopState | None = None


def hand_registers_to_radians(registers: list[int | float]) -> list[float]:
    """Inspire 手角度寄存器（0.1°）转弧度。"""
    return [float(v) * HAND_REG_TO_RAD for v in registers]


def encode_snapshot(snapshot: TeleopStateSnapshot) -> bytes:
    payload = {
        "stamp": snapshot.stamp,
        "left": _side_to_dict(snapshot.left),
        "right": _side_to_dict(snapshot.right),
    }
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_snapshot(data: bytes) -> TeleopStateSnapshot:
    raw = json.loads(data.decode("utf-8"))
    return TeleopStateSnapshot(
        stamp=float(raw.get("stamp", 0.0)),
        left=_side_from_dict(raw.get("left")),
        right=_side_from_dict(raw.get("right")),
    )


def _side_to_dict(side: SideTeleopState | None) -> dict[str, Any] | None:
    if side is None:
        return None
    return {
        "arm_valid": side.arm_valid,
        "hand_valid": side.hand_valid,
        "arm_joints": side.arm_joints,
        "end_pose": side.end_pose,
        "hand_joints": side.hand_joints,
    }


def _side_from_dict(raw: dict[str, Any] | None) -> SideTeleopState | None:
    if not raw:
        return None
    pose = raw.get("end_pose") or {}
    return SideTeleopState(
        arm_valid=bool(raw.get("arm_valid", False)),
        hand_valid=bool(raw.get("hand_valid", False)),
        arm_joints=[float(v) for v in raw.get("arm_joints", [])],
        end_pose={
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "z": float(pose.get("z", 0.0)),
            "qx": float(pose.get("qx", 0.0)),
            "qy": float(pose.get("qy", 0.0)),
            "qz": float(pose.get("qz", 0.0)),
            "qw": float(pose.get("qw", 1.0)),
        },
        hand_joints=[float(v) for v in raw.get("hand_joints", [])],
    )


class TeleopStateSender:
    """向 publisher 发送遥操作状态（UDP 单播）。"""

    def __init__(self, host: str = DEFAULT_UDP_HOST, port: int = DEFAULT_UDP_PORT) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_snapshot(self, snapshot: TeleopStateSnapshot) -> None:
        self._sock.sendto(encode_snapshot(snapshot), self._addr)

    def send_dict(self, payload: dict[str, Any]) -> None:
        self._sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self._addr)

    def close(self) -> None:
        self._sock.close()


class TeleopStateReceiver:
    """publisher 侧接收最新遥操作状态。"""

    def __init__(
        self,
        host: str = DEFAULT_UDP_HOST,
        port: int = DEFAULT_UDP_PORT,
        stale_timeout_s: float = 1.0,
    ) -> None:
        self.stale_timeout_s = stale_timeout_s
        self._latest: TeleopStateSnapshot | None = None
        self._latest_mono = 0.0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.setblocking(False)

    def poll(self) -> None:
        while True:
            try:
                data, _ = self._sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                break
            try:
                self._latest = decode_snapshot(data)
                self._latest_mono = time.monotonic()
            except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
                continue

    def get_latest(self) -> TeleopStateSnapshot | None:
        self.poll()
        if self._latest is None:
            return None
        if time.monotonic() - self._latest_mono > self.stale_timeout_s:
            return None
        return self._latest

    def close(self) -> None:
        self._sock.close()
