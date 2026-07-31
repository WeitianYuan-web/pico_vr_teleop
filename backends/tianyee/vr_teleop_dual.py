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
from common.math_euler import euler_xyz_to_quat_wxyz, quat_wxyz_to_euler_xyz
from common.math_quat import quat_multiply_wxyz
from common.math_se3 import transform_xr_controller
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
                    seeded_ok = False
                    for side in self.active_hands:
                        st = self.sides[side]
                        st.hold_pos = np.zeros(3)
                        st.hold_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
                        st.desired_pos = st.hold_pos.copy()
                        st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
                    print("[Tianyi] 警告: 未能从 bridge 取 TF，请确认机器人桥已启动")

        self._apply_home_sequence(seeded_ok=seeded_ok, label="启动初始化")

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
            timeout_s=max(25.0, duration + 20.0),
        )
        if not pkt or not pkt.get("ok", True):
            err = None if pkt is None else pkt.get("error")
            print(f"[Tianyi] 关节就绪失败: {err or 'no reply'}（请重启 bridge 以加载新代码）")
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
            self._capture_home_targets_from_current()
            return

        if getattr(self.args, "home_use_joints", False):
            if self._go_home_joints():
                seeded_ok = True
                # 关节到位后只做小位移微调，姿态保持（避免末端 IK 把肘翻上去）
                self.args.home_rpy_left_deg = None
                self.args.home_rpy_right_deg = None
            else:
                print("[Tianyi] 回退为笛卡尔抬手")

        if not seeded_ok:
            print("[Tianyi] 跳过抬手初始化（无有效 TCP）")
            self._capture_home_targets_from_current()
            return

        self._capture_home_targets_from_current()
        offset = self._home_offset()
        if getattr(self.args, "home_use_joints", False) and float(np.linalg.norm(offset)) < 1e-3:
            for side in self.active_hands:
                st = self.sides[side]
                if st.home_pos is not None and st.home_quat_wxyz is not None:
                    st.hold_pos = st.home_pos.copy()
                    st.hold_quat_wxyz = st.home_quat_wxyz.copy()
                    st.desired_pos = st.hold_pos.copy()
                    st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
            print(f"[Tianyi] {label}: 关节就绪完成（无额外笛卡尔偏移）")
            return
        self._move_to_home(label=label)

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

    def _capture_home_targets_from_current(self) -> None:
        """Record raised home = current TCP + offset (+ optional rotation)."""
        offset = self._home_offset()
        for side in self.active_hands:
            st = self.sides[side]
            if st.hold_pos is None or st.hold_quat_wxyz is None:
                continue
            st.home_pos = st.hold_pos + offset
            st.home_quat_wxyz = self._resolve_home_quat(side, st.hold_quat_wxyz)
            rpy = quat_wxyz_to_euler_xyz(st.home_quat_wxyz)
            print(
                f"[{st.name}] home TCP {st.home_pos} "
                f"rpy_deg=({rpy[0]*180/np.pi:+.1f},{rpy[1]*180/np.pi:+.1f},{rpy[2]*180/np.pi:+.1f}) "
                f"(offset dx={offset[0]:+.3f} dy={offset[1]:+.3f} dz={offset[2]:+.3f})"
            )

    def _send_active_poses(self, poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> None:
        left_pkt = None
        right_pkt = None
        for side, (pos, quat) in poses.items():
            pkt = side_pose(active=True, xyz=pos, quat_wxyz=quat)
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

    def _move_to_home(self, *, label: str = "回抬手位") -> None:
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
            f"[Tianyi] {label}: 抬手偏移 {self._home_offset().tolist()}，"
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
        finally:
            self._homing = False

        for side in goals:
            st = self.sides[side]
            st.hold_pos = goals[side][0].copy()
            st.hold_quat_wxyz = goals[side][1].copy()
            st.desired_pos = st.hold_pos.copy()
            st.desired_quat_wxyz = st.hold_quat_wxyz.copy()
            print(f"[{st.name}] {label}完成 → {st.hold_pos}")

    def _go_home_from_button(self, st: SideState) -> None:
        now = time.time()
        if self._homing or now - st.last_home_time < float(self.args.home_cooldown_s):
            return
        for s in self.sides.values():
            s.last_home_time = now
        print(f"\n[{st.name}] B 键：回到抬手位")
        self._apply_home_sequence(seeded_ok=True, label="B 回抬手位")

    def _release_clutch(self, st: SideState) -> None:
        if st.is_clutching:
            print(f"\n[{st.name}] Grip 断开")
            if st.desired_pos is not None and st.desired_quat_wxyz is not None:
                st.hold_pos = st.desired_pos.copy()
                st.hold_quat_wxyz = st.desired_quat_wxyz.copy()
            # 松开后短时继续发送冻结目标，避免末端 QP 继续追噪点
            st.freeze_until = time.time() + float(self.args.release_freeze_s)
        st.is_clutching = False
        st.ref_ee_pos = None
        st.ref_ee_quat_wxyz = None
        st.ref_controller_xyz = None
        st.ref_controller_quat_wxyz = None
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
        return target_rotation_from_controller_rel(
            ref_ctrl_q, cur_ctrl_q, ref_ee_q, rotation_scale
        )

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

        b_pressed = is_button_pressed(ctrl, BTN_B_INDEX)
        if b_pressed and not st.prev_b_pressed:
            self._go_home_from_button(st)
        st.prev_b_pressed = b_pressed

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
            # 用本地 hold/desired 作基准，避免每次 Grip 读滞后 TF 导致跳变/乱动
            if st.hold_pos is not None and st.hold_quat_wxyz is not None:
                pos = st.hold_pos.copy()
                quat = st.hold_quat_wxyz.copy()
            elif st.desired_pos is not None and st.desired_quat_wxyz is not None:
                pos = st.desired_pos.copy()
                quat = st.desired_quat_wxyz.copy()
            else:
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
            st.freeze_until = 0.0
            st.ref_controller_xyz = None
            st.ref_controller_quat_wxyz = None
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

        if rotation_enabled(ctrl, self.args.rotation_mode, btn_a_index=BTN_A_INDEX):
            raw_q = self._apply_rot_deadzone(
                st.ref_controller_quat_wxyz,
                c_quat,
                st.ref_ee_quat_wxyz,
                rotation_scale=self.args.rotation_scale,
                deadzone_deg=self.args.rot_deadzone_deg,
            )
            st.filt_quat_wxyz = slerp_filter_quat(
                st.filt_quat_wxyz, raw_q, self.args.rot_filter_alpha
            )
        else:
            st.filt_quat_wxyz = st.ref_ee_quat_wxyz.copy()

        assert st.filt_pos is not None and st.filt_quat_wxyz is not None
        st.desired_pos = self._clamp_cmd_step(st, st.filt_pos)
        st.desired_quat_wxyz = st.filt_quat_wxyz.copy()

    def _consume_latest_vr_data(self) -> None:
        if self._latest_vr_data is None or self._homing:
            return
        ctrls = self._latest_vr_data.get("controllers", [])
        for side in self.active_hands:
            ctrl = next((c for c in ctrls if c.get("handedness") == side), None)
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
            freezing = (not st.is_clutching) and now < float(st.freeze_until)
            active = bool(st.is_clutching or freezing)
            if st.is_clutching:
                pos, quat = st.desired_pos, st.desired_quat_wxyz
            else:
                pos, quat = st.hold_pos, st.hold_quat_wxyz
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
            connected_message="[Tianyi] WebXR 已连接；Grip 接合，B 回抬手位",
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
    p.add_argument("--no-home", action="store_true", help="启动时不抬手到初始位")
    p.add_argument(
        "--home-pose",
        choices=tuple(HOME_POSE_PRESETS.keys()),
        default=DEFAULT_HOME_POSE,
        help="初始姿态预设：hold_box=关节肘下就绪；keep=保持当前姿态",
    )
    p.add_argument(
        "--home-offset-xyz",
        type=float,
        nargs=3,
        default=list(DEFAULT_HOME_OFFSET_XYZ),
        metavar=("DX", "DY", "DZ"),
        help="关节就绪后的笛卡尔微调（腰系米，默认 0.06 0 0.05）",
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
        help="松开 Grip 后继续发送冻结目标的时长（秒）",
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
    if args.grip_threshold is not None:
        args.grip_engage = float(args.grip_threshold)
        args.grip_release = max(0.0, float(args.grip_threshold) - 0.2)
    if float(args.grip_release) >= float(args.grip_engage):
        args.grip_release = max(0.0, float(args.grip_engage) - 0.2)
    preset = HOME_POSE_PRESETS[args.home_pose]
    args.home_use_joints = bool(preset.get("use_joints", False))
    if args.home_rpy_left_deg is None:
        left = preset.get("left")
        args.home_rpy_left_deg = None if left is None else list(left)
    if args.home_rpy_right_deg is None:
        right = preset.get("right")
        args.home_rpy_right_deg = None if right is None else list(right)
    if args.home_rpy_left_deg is None and DEFAULT_HOME_RPY_DEG_LEFT is not None and not args.home_use_joints:
        args.home_rpy_left_deg = list(DEFAULT_HOME_RPY_DEG_LEFT)
    if args.home_rpy_right_deg is None and DEFAULT_HOME_RPY_DEG_RIGHT is not None and not args.home_use_joints:
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
