#!/usr/bin/env python3
"""Tianyi dual-arm WebXR teleop → UDP (default) or direct ROS endpose."""

from __future__ import annotations

import argparse
import asyncio
import os
import socket
import sys
import threading
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

from common.clutch import controller_relative_delta
from common.constants import BTN_A_INDEX, BTN_B_INDEX, HANDS
from common.filters import lerp_position, slerp_filter_quat
from common.math_euler import euler_xyz_to_quat_wxyz, quat_wxyz_to_euler_xyz
from common.math_quat import quat_diff_as_angle_axis, quat_multiply_wxyz
from common.math_se3 import apply_delta_rotation, transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop
from config import (
    DEFAULT_CONTROL_HZ,
    DEFAULT_FROM_FRAME,
    DEFAULT_GRIP_ENGAGE,
    DEFAULT_GRIP_RELEASE,
    DEFAULT_HOME_COOLDOWN_S,
    DEFAULT_HOME_DURATION_S,
    DEFAULT_HOME_JOINT_DURATION_S,
    DEFAULT_HOME_MAX_STEP_M,
    DEFAULT_HOME_OFFSET_XYZ,
    DEFAULT_HOME_POSE,
    DEFAULT_HOME_RPY_DEG_LEFT,
    DEFAULT_HOME_RPY_DEG_RIGHT,
    DEFAULT_HOME_RPY_OFFSET_DEG,
    DEFAULT_HOME_SKIP_TOL_DEG,
    DEFAULT_HOME_SKIP_TOL_M,
    DEFAULT_HOME_XYZ_LEFT,
    DEFAULT_HOME_XYZ_RIGHT,
    DEFAULT_MAX_CMD_STEP_M,
    DEFAULT_POS_DEADZONE_M,
    DEFAULT_POS_FILTER_ALPHA,
    DEFAULT_RELEASE_FREEZE_S,
    DEFAULT_ROT_DEADZONE_DEG,
    DEFAULT_ROT_FILTER_ALPHA,
    DEFAULT_ROTATION_MODE,
    DEFAULT_TO_FRAME_LEFT,
    DEFAULT_TO_FRAME_RIGHT,
    DEFAULT_UDP_HOST,
    DEFAULT_UDP_PORT,
    DEFAULT_WS_URI_TIANYEE,
    HOME_POSE_PRESETS,
    HOME_Q_LEFT,
    HOME_Q_RIGHT,
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
    rot_ref_controller_quat_wxyz: np.ndarray | None = None
    rot_ref_ee_quat_wxyz: np.ndarray | None = None
    rotation_active: bool = False
    filt_pos: np.ndarray | None = None
    filt_quat_wxyz: np.ndarray | None = None
    desired_pos: np.ndarray | None = None
    desired_quat_wxyz: np.ndarray | None = None
    prev_b_pressed: bool = False
    last_home_time: float = 0.0
    home_pos: np.ndarray | None = None
    home_quat_wxyz: np.ndarray | None = None
    freeze_until: float = 0.0


@dataclass
class DualTianyiVrTeleop:
    args: argparse.Namespace
    active_hands: tuple[str, ...] = field(init=False)
    sides: dict[str, SideState] = field(default_factory=dict)
    ros: object | None = None
    udp_sock: socket.socket | None = None
    _latest_vr_data: dict | None = None
    _last_status_len: int = 0
    _homing: bool = False
    _home_button_pressed: bool = False
    _last_home_time: float = 0.0
    _state_sender: object | None = None
    _state_thread: threading.Thread | None = None
    _state_stop: threading.Event = field(default_factory=threading.Event)
    _last_state_error_log: float = 0.0

    def __post_init__(self) -> None:
        self.active_hands = HANDS if self.args.hands == "both" else (self.args.hands,)
        for side in self.active_hands:
            self.sides[side] = SideState(
                side=side,  # type: ignore[arg-type]
                name="左臂" if side == "left" else "右臂",
            )
        seeded_ok = True
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
            # Boot bridge skips --prepare; teleop must enable/mode3/activate endpose.
            if self.args.prepare:
                self._udp_prepare()
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
                if self._seed_from_udp_bridge():
                    seeded_ok = True
                else:
                    # prepare 已写入 TCP 时不要清零，否则后续会误判「远离 home」而乱动。
                    has_prepare_seed = all(
                        self.sides[side].hold_pos is not None
                        and float(np.linalg.norm(self.sides[side].hold_pos)) > 1e-3
                        for side in self.active_hands
                    )
                    if has_prepare_seed:
                        seeded_ok = True
                        print("[Tianyi] bridge get_tf 超时，沿用 prepare TCP 作为当前位置")
                    else:
                        seeded_ok = False
                        for side in self.active_hands:
                            st = self.sides[side]
                            st.hold_pos = np.zeros(3)
                            st.hold_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
                            st.desired_pos = st.hold_pos.copy()
                            st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
                        print("[Tianyi] 警告: 未能从 bridge 取 TF，请确认机器人桥已启动")

        self._apply_home_sequence(seeded_ok=seeded_ok, label="启动初始化")
        # prepare / home 会长时间占用 bridge；状态轮询放到之后，避免启动期刷 timed out。
        if self.args.transport == "udp":
            self._start_state_reporting()

    def _udp_request(self, payload: dict, timeout_s: float) -> dict | None:
        if self.udp_sock is None:
            return None
        import json

        self.udp_sock.settimeout(timeout_s)
        try:
            self.udp_sock.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"),
                (self.args.udp_host, int(self.args.udp_port)),
            )
            data, _ = self.udp_sock.recvfrom(65535)
            pkt = json.loads(data.decode("utf-8"))
            return pkt if isinstance(pkt, dict) else None
        except Exception as exc:  # noqa: BLE001
            print(f"[Tianyi] UDP 请求失败 ({payload.get('cmd')}): {exc}")
            return None
        finally:
            self.udp_sock.settimeout(None)

    @staticmethod
    def _collection_side_from_reply(raw: object) -> dict | None:
        if not isinstance(raw, dict):
            return None
        joints_raw = raw.get("joints")
        xyz_raw = raw.get("xyz")
        quat_raw = raw.get("quat_wxyz")
        joints = (
            [float(v) for v in joints_raw]
            if isinstance(joints_raw, list) and len(joints_raw) == 7
            else []
        )
        pose_ok = (
            isinstance(xyz_raw, list)
            and len(xyz_raw) == 3
            and isinstance(quat_raw, list)
            and len(quat_raw) == 4
        )
        if pose_ok:
            w, qx, qy, qz = [float(v) for v in quat_raw]
            x, y, z = [float(v) for v in xyz_raw]
            pose = {"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": w}
        else:
            pose = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        return {
            "arm_valid": len(joints) == 7 and pose_ok,
            "hand_valid": False,
            "arm_joints": joints,
            "end_pose": pose,
            "hand_joints": [],
        }

    def _start_state_reporting(self) -> None:
        hz = max(0.0, float(self.args.state_publish_hz))
        if hz <= 0.0 or self.args.transport != "udp":
            return
        publisher_dir = os.path.join(_ROOT, "publisher")
        if publisher_dir not in sys.path:
            sys.path.insert(0, publisher_dir)
        from teleop_state_bridge import TeleopStateSender

        self._state_sender = TeleopStateSender(
            self.args.state_udp_host,
            int(self.args.state_udp_port),
        )
        self._state_stop.clear()
        self._state_thread = threading.Thread(
            target=self._state_poll_loop,
            name="tianyee-state-poll",
            daemon=True,
        )
        self._state_thread.start()
        print(
            f"[Publisher] Tianyee 真实 7 轴状态 → "
            f"udp://{self.args.state_udp_host}:{self.args.state_udp_port} @ {hz:.0f}Hz"
        )

    def _state_poll_loop(self) -> None:
        hz = max(1.0, float(self.args.state_publish_hz))
        period = 1.0 / hz
        query_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bridge may briefly block on TF; allow > one control tick before declaring timeout.
        query_sock.settimeout(max(0.15, min(0.8, period * 3.0)))
        request = b'{"cmd":"get_state"}'
        try:
            while not self._state_stop.is_set():
                started = time.monotonic()
                try:
                    import json

                    query_sock.sendto(
                        request,
                        (self.args.udp_host, int(self.args.udp_port)),
                    )
                    data, _ = query_sock.recvfrom(65535)
                    reply = json.loads(data.decode("utf-8"))
                    if isinstance(reply, dict) and reply.get("cmd") == "get_state":
                        payload = {
                            "stamp": float(reply.get("t", time.time())),
                            "left": self._collection_side_from_reply(reply.get("left")),
                            "right": self._collection_side_from_reply(reply.get("right")),
                        }
                        if self._state_sender is not None:
                            self._state_sender.send_dict(payload)  # type: ignore[attr-defined]
                except (OSError, UnicodeDecodeError, ValueError) as exc:
                    now = time.monotonic()
                    if now - self._last_state_error_log >= 5.0:
                        print(f"\n[Publisher] Tianyee 状态轮询暂时失败: {exc}")
                        self._last_state_error_log = now
                remaining = period - (time.monotonic() - started)
                if remaining > 0.0:
                    self._state_stop.wait(remaining)
        finally:
            query_sock.close()

    def _udp_prepare(self) -> bool:
        print("[Tianyi] UDP prepare（使能 / mode3 / 激活 endpose）...")
        pkt = self._udp_request({"cmd": "prepare", "t": time.time()}, timeout_s=45.0)
        if not pkt or not pkt.get("ok", False):
            err = None if pkt is None else pkt.get("error")
            print(f"[Tianyi] prepare 失败: {err or 'no reply'}（请重新安装/重启 bridge）")
            return False
        print(f"[Tianyi] prepare 完成 ({pkt.get('elapsed_s', '?')}s)")
        for side in self.active_hands:
            side_data = pkt.get(side)
            if not isinstance(side_data, dict) or "xyz" not in side_data:
                continue
            pos = np.asarray(side_data["xyz"], dtype=float)
            quat = np.asarray(side_data.get("quat_wxyz", [1, 0, 0, 0]), dtype=float)
            st = self.sides[side]
            st.hold_pos = pos.copy()
            st.hold_quat_wxyz = quat.copy()
            st.desired_pos = pos.copy()
            st.desired_quat_wxyz = quat.copy()
            print(f"[{st.name}] prepare TCP {pos}")
        return True

    def _go_home_joints(self) -> bool:
        """Ask robot bridge to joint-space ready pose (elbows down)."""
        duration = float(self.args.home_joint_duration_s)
        print(f"[Tianyi] 关节就绪（肘朝下）约 {duration:.1f}s ...")
        if self.ros is not None:
            try:
                self.ros.move_joints_ready(  # type: ignore[union-attr]
                    q_left=list(self.args.home_q_left),
                    q_right=list(self.args.home_q_right),
                    duration_s=duration,
                )
                self.ros.activate_endpose_controllers()  # type: ignore[union-attr]
                self.ros.enable_auto_switch()  # type: ignore[union-attr]
                self.ros.hold_current_endpose(seconds=0.8)  # type: ignore[union-attr]
                for side in self.active_hands:
                    pos, quat = self.ros.wait_tf(side, timeout_s=3.0)  # type: ignore[union-attr]
                    st = self.sides[side]
                    st.hold_pos = pos.copy()
                    st.hold_quat_wxyz = quat.copy()
                    st.desired_pos = pos.copy()
                    st.desired_quat_wxyz = quat.copy()
                return True
            except Exception as exc:  # noqa: BLE001
                print(f"[Tianyi] 关节就绪失败: {exc}")
                return False

        pkt = self._udp_request(
            {
                "cmd": "go_home_joints",
                "t": time.time(),
                "duration_s": duration,
                "q_left": list(self.args.home_q_left),
                "q_right": list(self.args.home_q_right),
            },
            timeout_s=max(60.0, duration + 45.0),
        )
        if not pkt or not pkt.get("ok", True):
            err = None if pkt is None else pkt.get("error")
            print(f"[Tianyi] 关节就绪失败: {err or 'no reply'}（请重新安装/重启 bridge）")
            return False
        if not pkt.get("endpose_ready", False):
            print("[Tianyi] 警告: bridge 未确认切回 endpose，遥操作可能不跟随")
        else:
            print(f"[Tianyi] 关节就绪完成 ({pkt.get('elapsed_s', '?')}s)，endpose 已激活")
        for side in self.active_hands:
            side_data = pkt.get(side)
            if not isinstance(side_data, dict) or "xyz" not in side_data:
                continue
            pos = np.asarray(side_data["xyz"], dtype=float)
            quat = np.asarray(side_data["quat_wxyz"], dtype=float)
            st = self.sides[side]
            st.hold_pos = pos.copy()
            st.hold_quat_wxyz = quat.copy()
            st.desired_pos = pos.copy()
            st.desired_quat_wxyz = quat.copy()
            print(f"[{st.name}] 关节就绪后 TCP {pos}")
        return True

    def _apply_home_sequence(self, *, seeded_ok: bool, label: str) -> None:
        if self.args.no_home:
            self._set_home_targets(prefer_absolute=False)
            return

        # Optional elbow-down joint stage. Off by default: HOME_Q TCP ≠ fixed home XYZ.
        if getattr(self.args, "home_use_joints", False):
            if self._go_home_joints():
                seeded_ok = True
                # Keep absolute home RPY (TCP forward); cartesian stage will apply it.
            else:
                print("[Tianyi] 回退为笛卡尔回位")

        if not seeded_ok:
            print("[Tianyi] 跳过回位初始化（无有效 TCP）")
            self._set_home_targets(prefer_absolute=True)
            return

        self._set_home_targets(prefer_absolute=True)
        err_m = self._max_home_error_m()
        err_deg = self._max_home_rot_error_deg()
        skip_tol_m = float(getattr(self.args, "home_skip_tol_m", 0.04))
        skip_tol_deg = float(getattr(self.args, "home_skip_tol_deg", 12.0))
        if err_m <= skip_tol_m and err_deg <= skip_tol_deg:
            print(
                f"[Tianyi] {label}: 已在默认位姿附近 "
                f"(err={err_m:.3f}m / {err_deg:.1f}° ≤ {skip_tol_m:.3f}m / {skip_tol_deg:.1f}°)，跳过运动"
            )
            self._freeze_at_configured_home(label=label)
            return
        if err_m <= skip_tol_m and err_deg > skip_tol_deg:
            print(
                f"[Tianyi] {label}: 位置已近但朝向偏差 {err_deg:.1f}°，"
                f"回转到 TCP 朝前"
            )
        self._move_to_home(label=label)

    def _max_home_error_m(self) -> float:
        err = 0.0
        for side in self.active_hands:
            st = self.sides[side]
            if st.home_pos is None or st.hold_pos is None:
                continue
            err = max(err, float(np.linalg.norm(st.home_pos - st.hold_pos)))
        return err

    def _max_home_rot_error_deg(self) -> float:
        err = 0.0
        for side in self.active_hands:
            st = self.sides[side]
            if st.home_quat_wxyz is None or st.hold_quat_wxyz is None:
                continue
            aa = quat_diff_as_angle_axis(st.hold_quat_wxyz, st.home_quat_wxyz)
            err = max(err, float(np.linalg.norm(aa)) * 180.0 / np.pi)
        return err

    def _freeze_at_configured_home(self, *, label: str) -> None:
        """Lock bridge/controller at configured home without a travel trajectory."""
        goals: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for side in self.active_hands:
            st = self.sides[side]
            if st.home_pos is None or st.home_quat_wxyz is None:
                continue
            goals[side] = (st.home_pos.copy(), st.home_quat_wxyz.copy())
            st.hold_pos = st.home_pos.copy()
            st.hold_quat_wxyz = st.home_quat_wxyz.copy()
            st.desired_pos = st.hold_pos.copy()
            st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
            st.is_clutching = False
            st.freeze_until = time.time() + max(0.6, float(self.args.release_freeze_s))
        if not goals:
            return
        hold_until = time.time() + max(0.6, float(self.args.release_freeze_s))
        while time.time() < hold_until:
            self._send_inactive_poses(goals)
            time.sleep(0.05)
        for side in goals:
            print(f"[{self.sides[side].name}] {label}已到位 → {self.sides[side].hold_pos}")

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

    def _home_offset(self) -> np.ndarray:
        return np.asarray(self.args.home_offset_xyz, dtype=float)

    def _resolve_home_quat(self, side: str, current_quat_wxyz: np.ndarray) -> np.ndarray:
        """Absolute RPY (deg) wins; else apply RPY offset; else keep current."""
        abs_rpy = (
            self.args.home_rpy_left_deg if side == "left" else self.args.home_rpy_right_deg
        )
        if abs_rpy is not None:
            rx, ry, rz = [float(v) * np.pi / 180.0 for v in abs_rpy]
            return euler_xyz_to_quat_wxyz(rx, ry, rz)

        offset = np.asarray(self.args.home_rpy_offset_deg, dtype=float)
        if float(np.linalg.norm(offset)) < 1e-9:
            return current_quat_wxyz.copy()
        rx, ry, rz = [float(v) * np.pi / 180.0 for v in offset]
        dq = euler_xyz_to_quat_wxyz(rx, ry, rz)
        # 左乘：在腰系下叠加相对旋转
        q = quat_multiply_wxyz(dq, current_quat_wxyz)
        if q[0] < 0.0:
            q = -q
        n = float(np.linalg.norm(q))
        return q / n if n > 1e-12 else current_quat_wxyz.copy()

    def _configured_home_xyz(self, side: str) -> np.ndarray | None:
        raw = self.args.home_xyz_left if side == "left" else self.args.home_xyz_right
        if raw is None:
            return None
        return np.asarray(raw, dtype=float)

    def _set_home_targets(self, *, prefer_absolute: bool) -> None:
        """Record home TCP: fixed config XYZ (+offset) when available, else current+offset."""
        offset = self._home_offset()
        for side in self.active_hands:
            st = self.sides[side]
            if st.hold_pos is None or st.hold_quat_wxyz is None:
                continue
            abs_xyz = self._configured_home_xyz(side) if prefer_absolute else None
            if abs_xyz is not None:
                st.home_pos = abs_xyz + offset
                source = "fixed"
            else:
                st.home_pos = st.hold_pos + offset
                source = "current"
            st.home_quat_wxyz = self._resolve_home_quat(side, st.hold_quat_wxyz)
            rpy = quat_wxyz_to_euler_xyz(st.home_quat_wxyz)
            print(
                f"[{st.name}] home TCP {st.home_pos} "
                f"rpy_deg=({rpy[0]*180/np.pi:+.1f},{rpy[1]*180/np.pi:+.1f},{rpy[2]*180/np.pi:+.1f}) "
                f"({source}; offset dx={offset[0]:+.3f} dy={offset[1]:+.3f} dz={offset[2]:+.3f})"
            )

    def _send_active_poses(self, poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
        self._send_poses(poses, active=True)

    def _send_inactive_poses(self, poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
        """Explicitly release bridge active state while preserving exact targets."""
        self._send_poses(poses, active=False)

    def _send_poses(
        self,
        poses: dict[str, tuple[np.ndarray, np.ndarray]],
        *,
        active: bool,
    ) -> None:
        left_pkt = None
        right_pkt = None
        for side, (pos, quat) in poses.items():
            pkt = side_pose(active=active, xyz=pos, quat_wxyz=quat)
            if side == "left":
                left_pkt = pkt
            else:
                right_pkt = pkt
            if self.ros is not None:
                self.ros.publish_pose(side, pos, quat)  # type: ignore[union-attr]
                self.ros.spin_once(0.0)
        if self.udp_sock is not None and (left_pkt is not None or right_pkt is not None):
            payload = encode_pose_packet(t=time.time(), left=left_pkt, right=right_pkt)
            self.udp_sock.sendto(payload, (self.args.udp_host, int(self.args.udp_port)))

    def _move_to_home(self, *, label: str = "回初始位") -> None:
        starts: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        goals: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for side in self.active_hands:
            st = self.sides[side]
            if st.home_pos is None or st.home_quat_wxyz is None:
                continue
            try:
                pos, quat = self._current_tcp(side)
            except Exception:  # noqa: BLE001
                if st.hold_pos is None or st.hold_quat_wxyz is None:
                    continue
                pos, quat = st.hold_pos.copy(), st.hold_quat_wxyz.copy()
            starts[side] = (pos, quat)
            goals[side] = (st.home_pos.copy(), st.home_quat_wxyz.copy())
            self._release_clutch(st)

        if not goals:
            print(f"[Tianyi] {label}: 无 home 目标，跳过")
            return

        max_dist = 0.0
        for side, (p0, _) in starts.items():
            max_dist = max(max_dist, float(np.linalg.norm(goals[side][0] - p0)))
        step = max(1e-4, float(self.args.home_max_step_m))
        duration = max(0.2, float(self.args.home_duration_s))
        steps_by_dist = int(np.ceil(max_dist / step)) if max_dist > 1e-6 else 1
        steps_by_time = max(1, int(duration * float(self.args.control_hz)))
        steps = max(steps_by_dist, steps_by_time)
        dt = duration / steps

        print(
            f"[Tianyi] {label}: 回位偏移 {self._home_offset().tolist()}，"
            f"最大位移 {max_dist:.3f}m，{steps} 步 / {duration:.1f}s"
        )
        self._homing = True
        try:
            for i in range(steps + 1):
                a = i / steps
                poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
                for side, (p0, q0) in starts.items():
                    p1, q1 = goals[side]
                    pos = (1.0 - a) * p0 + a * p1
                    quat = slerp_filter_quat(q0, q1, a) if a < 1.0 else q1.copy()
                    poses[side] = (pos, quat)
                    st = self.sides[side]
                    st.desired_pos = pos.copy()
                    st.desired_quat_wxyz = quat.copy()
                    st.hold_pos = pos.copy()
                    st.hold_quat_wxyz = quat.copy()
                self._send_active_poses(poses)
                time.sleep(dt)

            # Hand off an explicit freeze at the saved home pose.  If we only
            # stop the active stream, the bridge watchdog may sample lagged TF
            # still near the pre-B pose and lock that instead — then the arms
            # crawl back after appearing to reach home.
            hold_s = max(0.6, float(self.args.release_freeze_s))
            hold_until = time.time() + hold_s
            while time.time() < hold_until:
                self._send_inactive_poses(goals)
                time.sleep(0.05)
        finally:
            self._homing = False

        for side in goals:
            st = self.sides[side]
            st.hold_pos = goals[side][0].copy()
            st.hold_quat_wxyz = goals[side][1].copy()
            st.desired_pos = st.hold_pos.copy()
            st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
            # Keep publishing inactive home via the normal control loop so a
            # late watchdog sample cannot resurrect the pre-B pose.
            st.freeze_until = time.time() + max(0.6, float(self.args.release_freeze_s))
            st.is_clutching = False
            print(f"[{st.name}] {label}完成 → {st.hold_pos}")

    def _go_home_from_button(self, st: SideState) -> None:
        now = time.time()
        if self._homing or now - self._last_home_time < float(self.args.home_cooldown_s):
            return
        self._last_home_time = now
        for s in self.sides.values():
            s.last_home_time = now
        print(f"\n[{st.name}] 回位键：左右臂直接回到配置的默认初始位")

        # The joint-space elbow-down stage is only an initialization aid.  Do
        # not repeat it for the B button: doing so makes both arms visibly dip
        # before rising.  B returns on one Cartesian path to the fixed home.
        missing_home = any(
            self.sides[side].home_pos is None
            or self.sides[side].home_quat_wxyz is None
            for side in self.active_hands
        )
        if missing_home:
            print("[Tianyi] 默认 home 目标缺失，回退完整初始化流程")
            self._apply_home_sequence(seeded_ok=True, label="B 回位恢复")
            return
        # Refresh goals from config so B always uses the fixed default, even if
        # hold/desired drifted during teleop.
        self._set_home_targets(prefer_absolute=True)
        self._move_to_home(label="B 回默认位")

    def _release_clutch(self, st: SideState) -> None:
        if st.is_clutching:
            print(f"\n[{st.name}] Grip 断开")
            if self.ros is not None:
                # 直连 ROS 时锁住松开瞬间的实际 TCP，而不是继续
                # 追踪松开前的命令目标。UDP 模式由机器人 bridge 处理。
                try:
                    pos, quat = self._current_tcp(st.side)
                    st.hold_pos = pos.copy()
                    st.hold_quat_wxyz = quat.copy()
                    st.desired_pos = pos.copy()
                    st.desired_quat_wxyz = quat.copy()
                except Exception as exc:  # noqa: BLE001
                    print(f"\n[{st.name}] 读取实际 TCP 失败，保持最后目标: {exc}")
            elif st.desired_pos is not None and st.desired_quat_wxyz is not None:
                # 本地值仅作回退；bridge 收到 active=false 后会锁住实际 TF。
                st.hold_pos = st.desired_pos.copy()
                st.hold_quat_wxyz = st.desired_quat_wxyz.copy()
            # 重复发送 active=false，降低单个 UDP 释放包丢失的风险。
            st.freeze_until = time.time() + float(self.args.release_freeze_s)
        st.is_clutching = False
        st.ref_ee_pos = None
        st.ref_ee_quat_wxyz = None
        st.ref_controller_xyz = None
        st.ref_controller_quat_wxyz = None
        st.rot_ref_controller_quat_wxyz = None
        st.rot_ref_ee_quat_wxyz = None
        st.rotation_active = False
        st.filt_pos = None
        st.filt_quat_wxyz = None

    @staticmethod
    def _apply_pos_deadzone(delta: np.ndarray, deadzone_m: float) -> np.ndarray:
        n = float(np.linalg.norm(delta))
        dz = max(0.0, float(deadzone_m))
        if n <= dz:
            return np.zeros(3, dtype=float)
        return delta * ((n - dz) / n)

    def _apply_rot_deadzone(
        self,
        ref_ctrl_q: np.ndarray,
        cur_ctrl_q: np.ndarray,
        ref_ee_q: np.ndarray,
        *,
        rotation_scale: float,
        deadzone_deg: float,
    ) -> np.ndarray:
        from common.math_quat import quat_diff_as_angle_axis

        rel_aa = quat_diff_as_angle_axis(ref_ctrl_q, cur_ctrl_q)
        ang = float(np.linalg.norm(rel_aa))
        if ang * 180.0 / np.pi < float(deadzone_deg):
            return ref_ee_q.copy()
        # 手柄姿态已在机器人世界系中：
        # q_delta_world = q_cur * inverse(q_ref)，应左乘到 TCP 基准姿态。
        return apply_delta_rotation(ref_ee_q, rel_aa * float(rotation_scale))

    def _clamp_cmd_step(self, st: SideState, target: np.ndarray) -> np.ndarray:
        if st.desired_pos is None:
            return target
        max_step = max(1e-4, float(self.args.max_cmd_step_m))
        delta = target - st.desired_pos
        n = float(np.linalg.norm(delta))
        if n <= max_step:
            return target
        return st.desired_pos + delta * (max_step / n)

    def _update_from_controller(self, st: SideState, ctrl: dict) -> None:
        required = ("grip", "x", "y", "z", "qx", "qy", "qz", "qw")
        if not all(k in ctrl for k in required):
            return

        if self._homing:
            return

        grip = float(ctrl["grip"])
        # 滞回：避免临界区反复接合/断开
        if st.is_clutching:
            grip_pressed = grip > float(self.args.grip_release)
        else:
            grip_pressed = grip > float(self.args.grip_engage)

        if not grip_pressed:
            self._release_clutch(st)
            return

        if not st.is_clutching:
            # UDP: 优先用本地 hold（与 bridge 松开时锁定的最后指令一致），
            # 避免 get_tf 卡住 bridge 主循环、触发 watchdog 与遥操作目标争抢。
            pos = None
            quat = None
            if (
                self.udp_sock is not None
                and st.hold_pos is not None
                and st.hold_quat_wxyz is not None
            ):
                pos = st.hold_pos.copy()
                quat = st.hold_quat_wxyz.copy()
            else:
                try:
                    pos, quat = self._current_tcp(st.side)
                except Exception as exc:  # noqa: BLE001
                    if st.hold_pos is None or st.hold_quat_wxyz is None:
                        print(f"\n[{st.name}] 无法读 TCP，跳过接合: {exc}")
                        return
                    print(f"\n[{st.name}] 读取 TCP 失败，使用本地 hold: {exc}")
                    pos = st.hold_pos.copy()
                    quat = st.hold_quat_wxyz.copy()
            assert pos is not None and quat is not None
            st.ref_ee_pos = pos.copy()
            st.ref_ee_quat_wxyz = quat.copy()
            st.hold_pos = pos.copy()
            st.hold_quat_wxyz = quat.copy()
            st.desired_pos = pos.copy()
            st.desired_quat_wxyz = quat.copy()
            st.is_clutching = True
            st.freeze_until = 0.0
            st.ref_controller_xyz = None
            st.ref_controller_quat_wxyz = None
            st.rot_ref_controller_quat_wxyz = None
            st.rot_ref_ee_quat_wxyz = None
            st.rotation_active = False
            st.filt_pos = pos.copy()
            st.filt_quat_wxyz = quat.copy()
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
        delta_m = self._apply_pos_deadzone(delta_m, self.args.pos_deadzone_m)
        raw_pos = st.ref_ee_pos + delta_m
        st.filt_pos = lerp_position(st.filt_pos, raw_pos, self.args.pos_filter_alpha)

        rotate_now = rotation_enabled(ctrl, self.args.rotation_mode, btn_a_index=BTN_A_INDEX)
        if rotate_now and not st.rotation_active:
            # hold-a 每次按下时重建姿态离合基准，避免将 A/X
            # 松开期间累积的手柄旋转一次性施加给机器人。
            st.rotation_active = True
            st.rot_ref_controller_quat_wxyz = c_quat.copy()
            assert st.filt_quat_wxyz is not None
            st.rot_ref_ee_quat_wxyz = st.filt_quat_wxyz.copy()
        elif not rotate_now and st.rotation_active:
            st.rotation_active = False
            st.rot_ref_controller_quat_wxyz = None
            st.rot_ref_ee_quat_wxyz = None

        if rotate_now:
            assert st.rot_ref_controller_quat_wxyz is not None
            assert st.rot_ref_ee_quat_wxyz is not None
            raw_q = self._apply_rot_deadzone(
                st.rot_ref_controller_quat_wxyz,
                c_quat,
                st.rot_ref_ee_quat_wxyz,
                rotation_scale=self.args.rotation_scale,
                deadzone_deg=self.args.rot_deadzone_deg,
            )
            st.filt_quat_wxyz = slerp_filter_quat(
                st.filt_quat_wxyz, raw_q, self.args.rot_filter_alpha
            )
        # 旋转未启用时保持当前 filt_quat，不退回 Grip 接合时的姿态。

        assert st.filt_pos is not None and st.filt_quat_wxyz is not None
        st.desired_pos = self._clamp_cmd_step(st, st.filt_pos)
        st.desired_quat_wxyz = st.filt_quat_wxyz.copy()

    def _consume_latest_vr_data(self) -> None:
        if self._latest_vr_data is None or self._homing:
            return
        ctrls = self._latest_vr_data.get("controllers", [])
        ctrl_by_side = {
            side: next((c for c in ctrls if c.get("handedness") == side), None)
            for side in self.active_hands
        }

        # Treat either hand's home button as one global edge. Record the edge
        # before entering the blocking dual-arm sequence so the same VR frame
        # cannot trigger once per hand after the first sequence returns.
        home_sources = [
            side
            for side, ctrl in ctrl_by_side.items()
            if ctrl is not None and is_button_pressed(ctrl, BTN_B_INDEX)
        ]
        home_pressed = bool(home_sources)
        home_rising = home_pressed and not self._home_button_pressed
        self._home_button_pressed = home_pressed
        if home_rising:
            self._go_home_from_button(self.sides[home_sources[0]])

        for side in self.active_hands:
            ctrl = ctrl_by_side[side]
            if ctrl is None:
                self._release_clutch(self.sides[side])
                continue
            self._update_from_controller(self.sides[side], ctrl)

    def _publish_targets(self) -> None:
        if self._homing:
            return
        now = time.time()
        left_pkt = None
        right_pkt = None
        for side in self.active_hands:
            st = self.sides[side]
            if st.desired_pos is None or st.desired_quat_wxyz is None:
                continue
            releasing = (not st.is_clutching) and now < float(st.freeze_until)
            # 空闲臂不进包：bridge 自行 hold，避免 active=false 与另一臂流抢控制。
            if not st.is_clutching and not releasing:
                continue
            active = bool(st.is_clutching)
            if st.is_clutching:
                pos, quat = st.desired_pos, st.desired_quat_wxyz
            else:
                # 松开：发送最后指令位姿，供 bridge 冻结（勿用滞后的实际 TF）
                pos, quat = st.hold_pos, st.hold_quat_wxyz
            if pos is None or quat is None:
                continue
            if self.ros is not None and (active or releasing):
                self.ros.publish_pose(side, pos, quat)  # type: ignore[union-attr]
            pkt = side_pose(active=active, xyz=pos, quat_wxyz=quat)
            if side == "left":
                left_pkt = pkt
            else:
                right_pkt = pkt

        if self.udp_sock is not None:
            if left_pkt is not None or right_pkt is not None:
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
            connected_message="[Tianyi] WebXR 已连接；Grip 接合，B 回初始位",
        )

    def close(self) -> None:
        self._state_stop.set()
        if self._state_thread is not None:
            self._state_thread.join(timeout=1.0)
            self._state_thread = None
        if self._state_sender is not None:
            try:
                self._state_sender.close()  # type: ignore[attr-defined]
            except OSError:
                pass
            self._state_sender = None
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
    p.add_argument("--state-udp-host", default="127.0.0.1", help="本机采集状态目标主机")
    p.add_argument("--state-udp-port", type=int, default=17981, help="本机采集状态 UDP 端口")
    p.add_argument(
        "--state-publish-hz",
        type=float,
        default=30.0,
        help="从机器人轮询并上报真实关节/TCP 状态的频率；0 关闭",
    )
    p.add_argument("--seed-tf", action="store_true", help="UDP 模式下用 ROS TF 初始化 TCP")
    p.add_argument(
        "--prepare",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="使能机械臂 + mode3 + 激活 endpose（UDP 默认开；--no-prepare 关闭）",
    )
    p.add_argument("--leave-enabled", action="store_true")
    p.add_argument("--no-home", action="store_true", help="启动时不执行机械臂回位")
    p.add_argument(
        "--home-pose",
        choices=tuple(HOME_POSE_PRESETS.keys()),
        default=DEFAULT_HOME_POSE,
        help="初始预设：默认笛卡尔到固定 DEFAULT_HOME_XYZ（不先走冲突的关节就绪）",
    )
    p.add_argument(
        "--home-joints-first",
        action="store_true",
        help="启动前先走关节肘下就绪（可能与固定 TCP 不一致，仅在肘姿态异常时用）",
    )
    p.add_argument(
        "--home-skip-tol-m",
        type=float,
        default=DEFAULT_HOME_SKIP_TOL_M,
        help="已在默认位附近时跳过启动运动的距离阈值（米）",
    )
    p.add_argument(
        "--home-skip-tol-deg",
        type=float,
        default=DEFAULT_HOME_SKIP_TOL_DEG,
        help="已在默认朝向附近时跳过启动运动的姿态阈值（度）",
    )
    p.add_argument(
        "--home-xyz-left",
        type=float,
        nargs=3,
        default=list(DEFAULT_HOME_XYZ_LEFT),
        metavar=("X", "Y", "Z"),
        help="左臂固定默认 TCP（腰系米）；启动与 B 共用",
    )
    p.add_argument(
        "--home-xyz-right",
        type=float,
        nargs=3,
        default=list(DEFAULT_HOME_XYZ_RIGHT),
        metavar=("X", "Y", "Z"),
        help="右臂固定默认 TCP（腰系米）；启动与 B 共用",
    )
    p.add_argument(
        "--home-offset-xyz",
        type=float,
        nargs=3,
        default=list(DEFAULT_HOME_OFFSET_XYZ),
        metavar=("DX", "DY", "DZ"),
        help="叠加在固定默认 TCP 上的微调（腰系米，默认 0 0 0）",
    )
    p.add_argument(
        "--home-rpy-offset-deg",
        type=float,
        nargs=3,
        default=list(DEFAULT_HOME_RPY_OFFSET_DEG),
        metavar=("RX", "RY", "RZ"),
        help="相对当前姿态叠加的欧拉 XYZ（度）；绝对 RPY 优先",
    )
    p.add_argument(
        "--home-rpy-left-deg",
        type=float,
        nargs=3,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="左臂绝对姿态欧拉 XYZ（度）；覆盖预设",
    )
    p.add_argument(
        "--home-rpy-right-deg",
        type=float,
        nargs=3,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="右臂绝对姿态欧拉 XYZ（度）；覆盖预设",
    )
    p.add_argument("--home-duration-s", type=float, default=DEFAULT_HOME_DURATION_S)
    p.add_argument(
        "--home-joint-duration-s",
        type=float,
        default=DEFAULT_HOME_JOINT_DURATION_S,
        help="关节就绪流式时长（秒）",
    )
    p.add_argument(
        "--home-q-left",
        type=float,
        nargs=7,
        default=list(HOME_Q_LEFT),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="左臂就绪关节角（rad）",
    )
    p.add_argument(
        "--home-q-right",
        type=float,
        nargs=7,
        default=list(HOME_Q_RIGHT),
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="右臂就绪关节角（rad）",
    )
    p.add_argument("--home-max-step-m", type=float, default=DEFAULT_HOME_MAX_STEP_M)
    p.add_argument("--home-cooldown-s", type=float, default=DEFAULT_HOME_COOLDOWN_S)
    p.add_argument("--from-frame", default=DEFAULT_FROM_FRAME)
    p.add_argument("--to-frame-left", default=DEFAULT_TO_FRAME_LEFT)
    p.add_argument("--to-frame-right", default=DEFAULT_TO_FRAME_RIGHT)
    p.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    p.add_argument(
        "--grip-engage",
        type=float,
        default=DEFAULT_GRIP_ENGAGE,
        help="Grip 接合阈值（滞回上沿）",
    )
    p.add_argument(
        "--grip-release",
        type=float,
        default=DEFAULT_GRIP_RELEASE,
        help="Grip 断开阈值（滞回下沿，须小于 engage）",
    )
    p.add_argument(
        "--grip-threshold",
        type=float,
        default=None,
        help="兼容旧参数：同时设置 engage/release（release=engage-0.2）",
    )
    p.add_argument("--position-scale", type=float, default=1.0)
    p.add_argument("--rotation-scale", type=float, default=1.0)
    p.add_argument(
        "--pos-filter-alpha",
        type=float,
        default=DEFAULT_POS_FILTER_ALPHA,
        help="位置滤波系数（越小越稳）",
    )
    p.add_argument(
        "--rot-filter-alpha",
        type=float,
        default=DEFAULT_ROT_FILTER_ALPHA,
        help="姿态滤波系数（越小越稳）",
    )
    p.add_argument(
        "--pos-deadzone-m",
        type=float,
        default=DEFAULT_POS_DEADZONE_M,
        help="手柄相对位移死区（米）",
    )
    p.add_argument(
        "--rot-deadzone-deg",
        type=float,
        default=DEFAULT_ROT_DEADZONE_DEG,
        help="手柄相对转角死区（度）",
    )
    p.add_argument(
        "--max-cmd-step-m",
        type=float,
        default=DEFAULT_MAX_CMD_STEP_M,
        help="每控制周期最大末端位移（米）",
    )
    p.add_argument(
        "--release-freeze-s",
        type=float,
        default=DEFAULT_RELEASE_FREEZE_S,
        help="松开 Grip 后重发 active=false 释放通知的时长（秒）",
    )
    p.add_argument(
        "--rotation-mode",
        choices=("always", "hold-a", "never"),
        default=DEFAULT_ROTATION_MODE,
        help="always=Grip 时跟转；hold-a=按住 A 才转；never=不转",
    )
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.prepare is None:
        # Boot bridge 不加 --prepare（避免打架自检）；遥操作启动时再使能。
        args.prepare = args.transport == "udp"
    if args.grip_threshold is not None:
        args.grip_engage = float(args.grip_threshold)
        args.grip_release = max(0.0, float(args.grip_threshold) - 0.2)
    if float(args.grip_release) >= float(args.grip_engage):
        args.grip_release = max(0.0, float(args.grip_engage) - 0.2)
    preset = HOME_POSE_PRESETS[args.home_pose]
    args.home_use_joints = bool(preset.get("use_joints", False)) or bool(
        getattr(args, "home_joints_first", False)
    )
    if args.home_rpy_left_deg is None:
        left = preset.get("left")
        args.home_rpy_left_deg = None if left is None else list(left)
    if args.home_rpy_right_deg is None:
        right = preset.get("right")
        args.home_rpy_right_deg = None if right is None else list(right)
    if args.home_rpy_left_deg is None and DEFAULT_HOME_RPY_DEG_LEFT is not None:
        args.home_rpy_left_deg = list(DEFAULT_HOME_RPY_DEG_LEFT)
    if args.home_rpy_right_deg is None and DEFAULT_HOME_RPY_DEG_RIGHT is not None:
        args.home_rpy_right_deg = list(DEFAULT_HOME_RPY_DEG_RIGHT)

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
