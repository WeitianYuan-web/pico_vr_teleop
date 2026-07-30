#!/usr/bin/env python3
"""Tianyi dual-arm WebXR teleop → UDP (default) or direct ROS endpose."""

from __future__ import annotations

import argparse
import asyncio
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Literal

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "../.."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from common.clutch import controller_relative_delta, target_rotation_from_controller_rel
from common.constants import BTN_A_INDEX, BTN_B_INDEX, HANDS
from common.filters import lerp_position, slerp_filter_quat
from common.math_se3 import transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop
from config import (
    DEFAULT_CONTROL_HZ,
    DEFAULT_FROM_FRAME,
    DEFAULT_TO_FRAME_LEFT,
    DEFAULT_TO_FRAME_RIGHT,
    DEFAULT_UDP_HOST,
    DEFAULT_UDP_PORT,
    DEFAULT_WS_URI_TIANYEE,
    R_HEADSET_TO_WORLD,
)
from udp_protocol import encode_pose_packet, side_pose

Side = Literal["left", "right"]


@dataclass
class SideState:
    side: Side
    name: str
    is_clutching: bool = False
    hold_pos: np.ndarray | None = None
    hold_quat_wxyz: np.ndarray | None = None
    ref_ee_pos: np.ndarray | None = None
    ref_ee_quat_wxyz: np.ndarray | None = None
    ref_controller_xyz: np.ndarray | None = None
    ref_controller_quat_wxyz: np.ndarray | None = None
    filt_pos: np.ndarray | None = None
    filt_quat_wxyz: np.ndarray | None = None
    desired_pos: np.ndarray | None = None
    desired_quat_wxyz: np.ndarray | None = None
    prev_b_pressed: bool = False


@dataclass
class DualTianyiVrTeleop:
    args: argparse.Namespace
    active_hands: tuple[str, ...] = field(init=False)
    sides: dict[str, SideState] = field(default_factory=dict)
    ros: object | None = None
    udp_sock: socket.socket | None = None
    _latest_vr_data: dict | None = None
    _last_status_len: int = 0

    def __post_init__(self) -> None:
        self.active_hands = HANDS if self.args.hands == "both" else (self.args.hands,)
        for side in self.active_hands:
            self.sides[side] = SideState(
                side=side,  # type: ignore[arg-type]
                name="左臂" if side == "left" else "右臂",
            )
        if self.args.transport == "ros":
            from ros_endpose import TianyiRosEndpose

            self.ros = TianyiRosEndpose(
                from_frame=self.args.from_frame,
                to_frame_left=self.args.to_frame_left,
                to_frame_right=self.args.to_frame_right,
            )
            if self.args.prepare:
                print("[Tianyi] prepare arm (enable/mode/auto_switch)...")
                self.ros.prepare_for_teleop()
            for side in self.active_hands:
                pos, quat = self.ros.wait_tf(side)  # type: ignore[union-attr]
                st = self.sides[side]
                st.hold_pos = pos.copy()
                st.hold_quat_wxyz = quat.copy()
                st.desired_pos = pos.copy()
                st.desired_quat_wxyz = quat.copy()
                print(f"[{st.name}] TCP {pos} quat_wxyz={quat}")
        else:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(
                f"[Tianyi] UDP → {self.args.udp_host}:{self.args.udp_port} "
                f"(robot bridge must be running)"
            )
            # Seed holds with zeros until first clutch snapshots from optional ros seed,
            # or wait for bridge TF via one-shot ros if available.
            if self.args.seed_tf:
                from ros_endpose import TianyiRosEndpose

                seed = TianyiRosEndpose(
                    from_frame=self.args.from_frame,
                    to_frame_left=self.args.to_frame_left,
                    to_frame_right=self.args.to_frame_right,
                    node_name="tianyee_vr_seed_tf",
                )
                try:
                    for side in self.active_hands:
                        pos, quat = seed.wait_tf(side)
                        st = self.sides[side]
                        st.hold_pos = pos.copy()
                        st.hold_quat_wxyz = quat.copy()
                        st.desired_pos = pos.copy()
                        st.desired_quat_wxyz = quat.copy()
                        print(f"[{st.name}] seed TCP {pos}")
                finally:
                    seed.shutdown()
            else:
                if not self._seed_from_udp_bridge():
                    for side in self.active_hands:
                        st = self.sides[side]
                        st.hold_pos = np.zeros(3)
                        st.hold_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
                        st.desired_pos = st.hold_pos.copy()
                        st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
                    print("[Tianyi] 警告: 未能从 bridge 取 TF，请确认机器人桥已启动")

    def _fetch_tf_from_udp_bridge(
        self, sides: tuple[str, ...] | None = None, timeout_s: float = 2.0
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """Query robot bridge for TCP poses. Does not mutate side state."""
        if self.udp_sock is None:
            return {}
        want = sides if sides is not None else tuple(self.active_hands)
        self.udp_sock.settimeout(timeout_s)
        try:
            import json

            req = json.dumps({"cmd": "get_tf", "t": time.time()}, separators=(",", ":")).encode(
                "utf-8"
            )
            self.udp_sock.sendto(req, (self.args.udp_host, int(self.args.udp_port)))
            data, _ = self.udp_sock.recvfrom(65535)
            pkt = json.loads(data.decode("utf-8"))
            out: dict[str, tuple[np.ndarray, np.ndarray]] = {}
            if not isinstance(pkt, dict):
                return out
            for side in want:
                side_data = pkt.get(side)
                if not isinstance(side_data, dict) or "xyz" not in side_data:
                    continue
                pos = np.asarray(side_data["xyz"], dtype=float)
                quat = np.asarray(side_data["quat_wxyz"], dtype=float)
                out[side] = (pos, quat)
            return out
        except Exception as exc:  # noqa: BLE001
            print(f"[Tianyi] bridge get_tf 失败: {exc}")
            return {}
        finally:
            self.udp_sock.settimeout(None)

    def _seed_from_udp_bridge(self, timeout_s: float = 2.0) -> bool:
        poses = self._fetch_tf_from_udp_bridge(timeout_s=timeout_s)
        if not poses:
            return False
        for side, (pos, quat) in poses.items():
            st = self.sides[side]
            st.hold_pos = pos.copy()
            st.hold_quat_wxyz = quat.copy()
            st.desired_pos = pos.copy()
            st.desired_quat_wxyz = quat.copy()
            print(f"[{st.name}] bridge TCP {pos}")
        return True

    def _current_tcp(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        if self.ros is not None:
            self.ros.spin_once(0.0)
            return self.ros.lookup_tcp(side)  # type: ignore[union-attr]
        poses = self._fetch_tf_from_udp_bridge(sides=(side,), timeout_s=0.3)
        if side in poses:
            return poses[side][0].copy(), poses[side][1].copy()
        st = self.sides[side]
        assert st.hold_pos is not None and st.hold_quat_wxyz is not None
        return st.hold_pos.copy(), st.hold_quat_wxyz.copy()

    def _release_clutch(self, st: SideState) -> None:
        if st.is_clutching:
            print(f"\n[{st.name}] Grip 断开")
            if st.desired_pos is not None and st.desired_quat_wxyz is not None:
                st.hold_pos = st.desired_pos.copy()
                st.hold_quat_wxyz = st.desired_quat_wxyz.copy()
        st.is_clutching = False
        st.ref_ee_pos = None
        st.ref_ee_quat_wxyz = None
        st.ref_controller_xyz = None
        st.ref_controller_quat_wxyz = None
        st.filt_pos = None
        st.filt_quat_wxyz = None

    def _update_from_controller(self, st: SideState, ctrl: dict) -> None:
        required = ("grip", "x", "y", "z", "qx", "qy", "qz", "qw")
        if not all(k in ctrl for k in required):
            return

        b_pressed = is_button_pressed(ctrl, BTN_B_INDEX)
        if b_pressed and not st.prev_b_pressed:
            try:
                pos, quat = self._current_tcp(st.side)
                st.hold_pos = pos
                st.hold_quat_wxyz = quat
                st.desired_pos = pos.copy()
                st.desired_quat_wxyz = quat.copy()
                self._release_clutch(st)
                print(f"\n[{st.name}] B 键：保持当前 TCP")
            except Exception as exc:  # noqa: BLE001
                print(f"\n[{st.name}] B 键刷新 TCP 失败: {exc}")
        st.prev_b_pressed = b_pressed

        grip_pressed = float(ctrl["grip"]) > self.args.grip_threshold
        if not grip_pressed:
            self._release_clutch(st)
            return

        if not st.is_clutching:
            try:
                pos, quat = self._current_tcp(st.side)
            except Exception as exc:  # noqa: BLE001
                print(f"\n[{st.name}] 无法读 TCP，跳过接合: {exc}")
                return
            st.ref_ee_pos = pos.copy()
            st.ref_ee_quat_wxyz = quat.copy()
            st.hold_pos = pos.copy()
            st.hold_quat_wxyz = quat.copy()
            st.desired_pos = pos.copy()
            st.desired_quat_wxyz = quat.copy()
            st.is_clutching = True
            st.ref_controller_xyz = None
            st.ref_controller_quat_wxyz = None
            st.filt_pos = None
            st.filt_quat_wxyz = None
            print(f"\n[{st.name}] Grip 接合")

        c_xyz, c_quat = transform_xr_controller(
            R_HEADSET_TO_WORLD,
            float(ctrl["x"]),
            float(ctrl["y"]),
            float(ctrl["z"]),
            float(ctrl["qx"]),
            float(ctrl["qy"]),
            float(ctrl["qz"]),
            float(ctrl["qw"]),
        )
        if st.ref_controller_xyz is None or st.ref_controller_quat_wxyz is None:
            st.ref_controller_xyz = c_xyz.copy()
            st.ref_controller_quat_wxyz = c_quat.copy()
            return

        assert st.ref_ee_pos is not None and st.ref_ee_quat_wxyz is not None
        delta_m = controller_relative_delta(
            st.ref_controller_xyz, c_xyz, self.args.position_scale
        )
        raw_pos = st.ref_ee_pos + delta_m
        st.filt_pos = lerp_position(st.filt_pos, raw_pos, self.args.pos_filter_alpha)

        if rotation_enabled(ctrl, self.args.rotation_mode, btn_a_index=BTN_A_INDEX):
            raw_q = target_rotation_from_controller_rel(
                st.ref_controller_quat_wxyz,
                c_quat,
                st.ref_ee_quat_wxyz,
                self.args.rotation_scale,
            )
            st.filt_quat_wxyz = slerp_filter_quat(
                st.filt_quat_wxyz, raw_q, self.args.rot_filter_alpha
            )
        else:
            st.filt_quat_wxyz = st.ref_ee_quat_wxyz.copy()

        st.desired_pos = st.filt_pos.copy()
        st.desired_quat_wxyz = st.filt_quat_wxyz.copy()

    def _consume_latest_vr_data(self) -> None:
        if self._latest_vr_data is None:
            return
        ctrls = self._latest_vr_data.get("controllers", [])
        for side in self.active_hands:
            ctrl = next((c for c in ctrls if c.get("handedness") == side), None)
            if ctrl is None:
                self._release_clutch(self.sides[side])
                continue
            self._update_from_controller(self.sides[side], ctrl)

    def _publish_targets(self) -> None:
        left_pkt = None
        right_pkt = None
        for side in self.active_hands:
            st = self.sides[side]
            if st.desired_pos is None or st.desired_quat_wxyz is None:
                continue
            active = bool(st.is_clutching)
            # While clutching stream desired; while released optionally stream hold to settle
            pos = st.desired_pos if active else st.hold_pos
            quat = st.desired_quat_wxyz if active else st.hold_quat_wxyz
            if pos is None or quat is None:
                continue
            if self.ros is not None and active:
                self.ros.publish_pose(side, pos, quat)  # type: ignore[union-attr]
            pkt = side_pose(active=active, xyz=pos, quat_wxyz=quat)
            if side == "left":
                left_pkt = pkt
            else:
                right_pkt = pkt

        if self.udp_sock is not None:
            # Only send when at least one side clutching to avoid noise
            if (left_pkt and left_pkt["active"]) or (right_pkt and right_pkt["active"]):
                payload = encode_pose_packet(t=time.time(), left=left_pkt, right=right_pkt)
                self.udp_sock.sendto(payload, (self.args.udp_host, int(self.args.udp_port)))

    def _on_vr_payload(self, payload: dict) -> None:
        self._latest_vr_data = payload

    async def _control_loop(self) -> None:
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        while True:
            t0 = time.time()
            if self.ros is not None:
                self.ros.spin_once(0.0)
            self._consume_latest_vr_data()
            self._publish_targets()
            self._print_status()
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, dt - elapsed))

    def _print_status(self) -> None:
        parts = []
        for side in self.active_hands:
            st = self.sides[side]
            flag = "ON " if st.is_clutching else "off"
            if st.desired_pos is not None:
                p = st.desired_pos
                parts.append(f"{st.name}:{flag}[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]")
            else:
                parts.append(f"{st.name}:{flag}")
        line = " | ".join(parts)
        pad = max(0, self._last_status_len - len(line))
        print("\r" + line + (" " * pad), end="", flush=True)
        self._last_status_len = len(line)

    async def run(self) -> None:
        await run_webxr_ws_loop(
            self.args.ws_uri,
            self._on_vr_payload,
            control_coro_factory=self._control_loop,
            connected_message="[Tianyi] WebXR 已连接；Grip 接合，B 刷新保持位姿",
        )

    def close(self) -> None:
        if self.udp_sock is not None:
            self.udp_sock.close()
            self.udp_sock = None
        if self.ros is not None:
            if self.args.prepare and not self.args.leave_enabled:
                try:
                    self.ros.call_enable(False)
                except Exception as exc:  # noqa: BLE001
                    print(f"[Tianyi] disable failed: {exc}")
            self.ros.shutdown()
            self.ros = None


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Tianyi dual-arm WebXR teleop")
    p.add_argument("--ws-uri", default=DEFAULT_WS_URI_TIANYEE)
    p.add_argument("--hands", choices=("both", "left", "right"), default="both")
    p.add_argument("--transport", choices=("udp", "ros"), default="udp")
    p.add_argument("--udp-host", default=DEFAULT_UDP_HOST)
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    p.add_argument("--seed-tf", action="store_true", help="UDP 模式下用 ROS TF 初始化 TCP")
    p.add_argument("--prepare", action="store_true", help="ROS 模式下自动 enable/mode3")
    p.add_argument("--leave-enabled", action="store_true")
    p.add_argument("--from-frame", default=DEFAULT_FROM_FRAME)
    p.add_argument("--to-frame-left", default=DEFAULT_TO_FRAME_LEFT)
    p.add_argument("--to-frame-right", default=DEFAULT_TO_FRAME_RIGHT)
    p.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    p.add_argument("--grip-threshold", type=float, default=0.5)
    p.add_argument("--position-scale", type=float, default=1.0)
    p.add_argument("--rotation-scale", type=float, default=1.0)
    p.add_argument("--pos-filter-alpha", type=float, default=0.35)
    p.add_argument("--rot-filter-alpha", type=float, default=0.35)
    p.add_argument(
        "--rotation-mode",
        choices=("always", "hold-a", "never"),
        default="always",
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    teleop = DualTianyiVrTeleop(args)
    try:
        asyncio.run(teleop.run())
    except KeyboardInterrupt:
        print("\n[Tianyi] 用户中断")
    finally:
        teleop.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
