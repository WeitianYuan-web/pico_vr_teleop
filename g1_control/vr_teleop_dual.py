#!/usr/bin/env python3
"""Unitree G1 双臂 WebXR 遥操作（Pinocchio IK + DDS 关节 PD）。"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import ssl
import sys
import time
from dataclasses import dataclass, field
from typing import Literal

import numpy as np
import pinocchio as pin
import websockets

from config import (
    DEFAULT_CONTROL_HZ,
    DEFAULT_NETWORK_INTERFACE,
    DEFAULT_URDF_PATH,
    DEFAULT_WS_URI,
    R_HEADSET_TO_WORLD,
)
from g1_arm_controller import create_arm_controller
from g1_arm_ik import G1DualArmIK

HANDS = ("left", "right")
BTN_A_INDEX = 4
BTN_B_INDEX = 5
Side = Literal["left", "right"]


def matrix_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    m = rot
    trace = np.trace(m)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    if q[0] < 0.0:
        q = -q
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_multiply_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_inverse_wxyz(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)


def quaternion_to_angle_axis(quat: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    q = np.array(quat, dtype=float, copy=True)
    if q[0] < 0.0:
        q = -q
    angle = 2.0 * np.arccos(np.clip(q[0], -1.0, 1.0))
    if angle < eps:
        return np.zeros(3, dtype=float)
    sin_half = np.sin(angle / 2.0)
    if sin_half < eps:
        return np.zeros(3, dtype=float)
    return (q[1:] / sin_half) * angle


def apply_delta_pose(source_rot_wxyz: np.ndarray, delta_rot: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    angle = float(np.linalg.norm(delta_rot))
    if angle > eps:
        axis = delta_rot / angle
        half = angle * 0.5
        rot_delta = np.array([np.cos(half), *(axis * np.sin(half))], dtype=float)
    else:
        rot_delta = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    target = quat_multiply_wxyz(rot_delta, source_rot_wxyz)
    n = np.linalg.norm(target)
    if n < 1e-12:
        return source_rot_wxyz.copy()
    if target[0] < 0.0:
        target = -target
    return target / n


def slerp_quat_wxyz(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    a = np.array(q0, dtype=float, copy=True)
    b = np.array(q1, dtype=float, copy=True)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        out = a + t * (b - a)
        n = np.linalg.norm(out)
        return out / n if n > 1e-12 else a
    theta_0 = math.acos(np.clip(dot, -1.0, 1.0))
    sin_0 = math.sin(theta_0)
    theta = theta_0 * t
    s0 = math.sin(theta_0 - theta) / sin_0
    s1 = math.sin(theta) / sin_0
    out = s0 * a + s1 * b
    n = np.linalg.norm(out)
    return out / n if n > 1e-12 else a


def se3_from_pos_quat(pos_m: np.ndarray, quat_wxyz: np.ndarray) -> pin.SE3:
    w, x, y, z = quat_wxyz
    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return pin.SE3(rot, np.asarray(pos_m, dtype=float).reshape(3))


def quat_wxyz_from_se3(pose: pin.SE3) -> np.ndarray:
    return matrix_to_quat_wxyz(pose.rotation)


@dataclass
class SideState:
    side: Side
    name: str
    is_clutching: bool = False
    desired_pose: pin.SE3 | None = None
    ref_ee_pose: pin.SE3 | None = None
    ref_ee_quat_wxyz: np.ndarray | None = None
    ref_controller_xyz: np.ndarray | None = None
    ref_controller_quat_wxyz: np.ndarray | None = None
    filt_pos_m: np.ndarray | None = None
    filt_quat_wxyz: np.ndarray | None = None
    last_home_time: float = 0.0
    prev_b_pressed: bool = False


@dataclass
class DualG1VrTeleop:
    args: argparse.Namespace
    active_hands: tuple[str, ...] = field(init=False)
    sides: dict[str, SideState] = field(default_factory=dict)
    arm_ctrl: object | None = None
    arm_ik: G1DualArmIK | None = None
    cmd_q: np.ndarray = field(default_factory=lambda: np.zeros(14))
    _latest_vr_data: dict | None = None
    _last_status_len: int = 0
    _last_state_publish_time: float = 0.0
    _state_publish_interval: float = 0.02
    state_sender: object | None = None

    def __post_init__(self) -> None:
        self.active_hands = HANDS if self.args.hands == "both" else (self.args.hands,)
        self._state_publish_interval = 1.0 / max(1.0, float(self.args.state_publish_hz))
        if self.args.publish_state:
            publisher_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../publisher"))
            if publisher_dir not in sys.path:
                sys.path.insert(0, publisher_dir)
            from teleop_state_bridge import TeleopStateSender  # noqa: WPS433

            self.state_sender = TeleopStateSender(self.args.state_udp_host, self.args.state_udp_port)
            print(
                f"[Publisher] 状态上报已启用: udp://{self.args.state_udp_host}:{self.args.state_udp_port} "
                f"@ {self.args.state_publish_hz:.0f}Hz"
            )

    def _is_button_pressed(self, ctrl: dict, index: int) -> bool:
        buttons = ctrl.get("buttons")
        if not buttons or len(buttons) <= index:
            return False
        return bool(buttons[index].get("pressed", False))

    def _rotation_enabled(self, ctrl: dict) -> bool:
        if self.args.rotation_mode == "off":
            return False
        if self.args.rotation_mode == "hold-a":
            return self._is_button_pressed(ctrl, BTN_A_INDEX)
        return True

    def _transform_xr_controller(
        self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float
    ) -> tuple[np.ndarray, np.ndarray]:
        vr_pos = np.array([x, y, z], dtype=float)
        controller_xyz = R_HEADSET_TO_WORLD @ vr_pos
        controller_quat_wxyz = np.array([qw, qx, qy, qz], dtype=float)
        r_quat_wxyz = matrix_to_quat_wxyz(R_HEADSET_TO_WORLD)
        controller_quat_wxyz = quat_multiply_wxyz(
            quat_multiply_wxyz(r_quat_wxyz, controller_quat_wxyz),
            quat_inverse_wxyz(r_quat_wxyz),
        )
        return controller_xyz, controller_quat_wxyz

    def connect(self) -> None:
        self.arm_ik = G1DualArmIK(urdf_path=self.args.urdf)
        self.arm_ctrl = create_arm_controller(
            dry_run=self.args.dry_run,
            motion_mode=self.args.motion,
            simulation_mode=self.args.sim,
            network_interface=self.args.network_interface,
            arm_velocity_limit=self.args.arm_velocity_limit,
        )
        if not self.args.no_home and not self.args.dry_run:
            self.arm_ctrl.ctrl_dual_arm_go_home()
        self.cmd_q = self.arm_ctrl.get_current_dual_arm_q()
        left_pose, right_pose = self.arm_ik.forward_kinematics(self.cmd_q)
        pose_map = {"left": left_pose, "right": right_pose}
        for side in self.active_hands:
            self.sides[side] = SideState(
                side=side,  # type: ignore[arg-type]
                name="左臂" if side == "left" else "右臂",
                desired_pose=pose_map[side].copy(),
            )
            p = pose_map[side].translation
            print(f"[{self.sides[side].name}] 初始末端 xyz=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) m")

    def disconnect(self) -> None:
        if self.arm_ctrl is not None:
            try:
                self.arm_ctrl.close()
            except Exception:
                pass
            self.arm_ctrl = None
        print("[G1] 已退出")

    def _current_ee_pose(self, side: Side) -> pin.SE3:
        assert self.arm_ik is not None and self.arm_ctrl is not None
        q = self.arm_ctrl.get_current_dual_arm_q()
        left_pose, right_pose = self.arm_ik.forward_kinematics(q)
        return left_pose if side == "left" else right_pose

    def _go_home(self, side: SideState) -> None:
        assert self.arm_ctrl is not None and self.arm_ik is not None
        now = time.time()
        if now - side.last_home_time < self.args.home_cooldown_s:
            return
        side.last_home_time = now
        print(f"\n[{side.name}] B 键回初始关节")
        self.arm_ctrl.ctrl_dual_arm_go_home()
        self.cmd_q = self.arm_ctrl.get_current_dual_arm_q()
        left_pose, right_pose = self.arm_ik.forward_kinematics(self.cmd_q)
        for s in self.sides.values():
            s.desired_pose = (left_pose if s.side == "left" else right_pose).copy()
            self._release_clutch(s)

    def _release_clutch(self, side: SideState) -> None:
        if side.is_clutching:
            print(f"\n[{side.name}] Grip 断开")
        side.is_clutching = False
        side.ref_ee_pose = None
        side.ref_ee_quat_wxyz = None
        side.ref_controller_xyz = None
        side.ref_controller_quat_wxyz = None
        side.filt_pos_m = None
        side.filt_quat_wxyz = None

    def _update_from_controller(self, side: SideState, ctrl: dict) -> None:
        required = ("grip", "x", "y", "z", "qx", "qy", "qz", "qw")
        if not all(k in ctrl for k in required):
            return

        b_pressed = self._is_button_pressed(ctrl, BTN_B_INDEX)
        if b_pressed and not side.prev_b_pressed:
            self._go_home(side)
        side.prev_b_pressed = b_pressed

        grip_pressed = float(ctrl["grip"]) > self.args.grip_threshold
        if not grip_pressed:
            self._release_clutch(side)
            return

        if not side.is_clutching:
            ee = self._current_ee_pose(side.side)
            side.ref_ee_pose = ee.copy()
            side.desired_pose = ee.copy()
            side.ref_ee_quat_wxyz = quat_wxyz_from_se3(ee)
            side.is_clutching = True
            side.ref_controller_xyz = None
            side.ref_controller_quat_wxyz = None
            side.filt_pos_m = None
            side.filt_quat_wxyz = None
            print(f"\n[{side.name}] Grip 接合")

        c_xyz, c_quat = self._transform_xr_controller(
            float(ctrl["x"]),
            float(ctrl["y"]),
            float(ctrl["z"]),
            float(ctrl["qx"]),
            float(ctrl["qy"]),
            float(ctrl["qz"]),
            float(ctrl["qw"]),
        )
        if side.ref_controller_xyz is None or side.ref_controller_quat_wxyz is None:
            side.ref_controller_xyz = c_xyz.copy()
            side.ref_controller_quat_wxyz = c_quat.copy()
            return

        assert side.ref_ee_pose is not None and side.ref_ee_quat_wxyz is not None
        delta_m = (c_xyz - side.ref_controller_xyz) * self.args.position_scale
        raw_pos = side.ref_ee_pose.translation + delta_m
        alpha_pos = float(np.clip(self.args.pos_filter_alpha, 0.0, 1.0))
        if side.filt_pos_m is None or alpha_pos >= 0.999:
            side.filt_pos_m = raw_pos
        else:
            side.filt_pos_m = (1.0 - alpha_pos) * side.filt_pos_m + alpha_pos * raw_pos

        if self._rotation_enabled(ctrl):
            q_rel = quat_multiply_wxyz(quat_inverse_wxyz(side.ref_controller_quat_wxyz), c_quat)
            n_rel = np.linalg.norm(q_rel)
            q_rel = q_rel / n_rel if n_rel > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])
            rel_aa = quaternion_to_angle_axis(q_rel) * self.args.rotation_scale
            raw_q = apply_delta_pose(side.ref_ee_quat_wxyz, rel_aa)
            alpha_rot = float(np.clip(self.args.rot_filter_alpha, 0.0, 1.0))
            if side.filt_quat_wxyz is None or alpha_rot >= 0.999:
                side.filt_quat_wxyz = raw_q
            else:
                if np.dot(side.filt_quat_wxyz, raw_q) < 0.0:
                    raw_q = -raw_q
                side.filt_quat_wxyz = slerp_quat_wxyz(side.filt_quat_wxyz, raw_q, alpha_rot)
            side.desired_pose = se3_from_pos_quat(side.filt_pos_m, side.filt_quat_wxyz)
        else:
            side.desired_pose = se3_from_pos_quat(side.filt_pos_m, side.ref_ee_quat_wxyz)

    def _consume_latest_vr_data(self) -> None:
        if self._latest_vr_data is None:
            return
        ctrls = self._latest_vr_data.get("controllers", [])
        for side_name in self.active_hands:
            side = self.sides[side_name]
            ctrl = next((c for c in ctrls if c.get("handedness") == side_name), None)
            if ctrl is None:
                self._release_clutch(side)
                continue
            self._update_from_controller(side, ctrl)

    def _step_control(self) -> None:
        assert self.arm_ctrl is not None and self.arm_ik is not None
        any_clutch = any(s.is_clutching for s in self.sides.values())
        if not any_clutch:
            return

        # 未接合侧保持当前 FK 位姿，避免 IK 把空闲臂拽走
        left_pose, right_pose = self.arm_ik.forward_kinematics(self.arm_ctrl.get_current_dual_arm_q())
        target_left = left_pose
        target_right = right_pose
        left_side = self.sides.get("left")
        right_side = self.sides.get("right")
        if left_side is not None and left_side.is_clutching and left_side.desired_pose is not None:
            target_left = left_side.desired_pose
        if right_side is not None and right_side.is_clutching and right_side.desired_pose is not None:
            target_right = right_side.desired_pose

        q_now = self.arm_ctrl.get_current_dual_arm_q()
        dq_now = self.arm_ctrl.get_current_dual_arm_dq()
        sol_q, sol_tau = self.arm_ik.solve_ik(target_left, target_right, q_now, dq_now)
        self.cmd_q = sol_q
        self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tau)

    def _build_side_state(self, side: str) -> dict | None:
        assert self.arm_ik is not None and self.arm_ctrl is not None
        if side not in self.sides and self.args.hands != "both":
            # 仍上报两侧便于 publisher
            pass
        q = self.arm_ctrl.get_current_dual_arm_q()
        left_pose, right_pose = self.arm_ik.forward_kinematics(q)
        pose = left_pose if side == "left" else right_pose
        arm_joints = q[0:7].tolist() if side == "left" else q[7:14].tolist()
        quat = quat_wxyz_from_se3(pose)
        return {
            "arm_valid": True,
            "hand_valid": False,
            "arm_joints": [float(v) for v in arm_joints],
            "end_pose": {
                "x": float(pose.translation[0]),
                "y": float(pose.translation[1]),
                "z": float(pose.translation[2]),
                "qx": float(quat[1]),
                "qy": float(quat[2]),
                "qz": float(quat[3]),
                "qw": float(quat[0]),
            },
            "hand_joints": [0.0] * 6,
        }

    def _maybe_publish_state(self) -> None:
        if self.state_sender is None:
            return
        now = time.time()
        if now - self._last_state_publish_time < self._state_publish_interval:
            return
        payload = {
            "stamp": now,
            "left": self._build_side_state("left"),
            "right": self._build_side_state("right"),
        }
        self.state_sender.send_dict(payload)
        self._last_state_publish_time = now

    async def control_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.args.control_hz))
        while True:
            start = time.perf_counter()
            self._consume_latest_vr_data()
            self._step_control()
            self._maybe_publish_state()
            if self.args.print_status:
                status = []
                for side in self.active_hands:
                    p = self.sides[side].desired_pose.translation if self.sides[side].desired_pose else [0, 0, 0]
                    clutch = "ON" if self.sides[side].is_clutching else "off"
                    status.append(f"{side}:{clutch}({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")
                line = "\r[G1-VR] " + " | ".join(status)
                pad = " " * max(0, self._last_status_len - len(line))
                self._last_status_len = len(line)
                print(line + pad, end="", flush=True)
            elapsed = time.perf_counter() - start
            await asyncio.sleep(max(0.0, period - elapsed))

    async def ws_loop(self) -> None:
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE
        print(f"[Network] 连接 WebXR: {self.args.ws_uri}")
        while True:
            try:
                async with websockets.connect(self.args.ws_uri, ssl=ssl_ctx) as ws:
                    print("[Network] WebXR 已连接，按住 Grip 控制，B 键回零位")
                    control_task = asyncio.create_task(self.control_loop())
                    try:
                        while True:
                            msg = await ws.recv()
                            try:
                                payload = json.loads(msg)
                            except json.JSONDecodeError:
                                continue
                            self._latest_vr_data = payload
                    finally:
                        control_task.cancel()
                        try:
                            await control_task
                        except asyncio.CancelledError:
                            pass
            except Exception as exc:
                print(f"[Network] 连接中断: {exc}，2 秒后重连")
                await asyncio.sleep(2.0)

    def run(self) -> None:
        self.connect()
        try:
            asyncio.run(self.ws_loop())
        finally:
            if self.args.print_status:
                print("")
            if self.state_sender is not None:
                self.state_sender.close()
            self.disconnect()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Unitree G1 双臂 WebXR 遥操作")
    p.add_argument("--hands", choices=("both", "left", "right"), default="both")
    p.add_argument("--ws-uri", default=DEFAULT_WS_URI)
    p.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="G1 双臂 URDF 路径")
    p.add_argument(
        "--network-interface",
        default=DEFAULT_NETWORK_INTERFACE,
        help="DDS 网卡名（本机默认 enp12s0；可用 ip -br addr 查看）",
    )
    p.add_argument("--motion", action="store_true", help="运控共存模式，使用 rt/arm_sdk + weight")
    p.add_argument("--sim", action="store_true", help="仿真模式（DDS domain=1）")
    p.add_argument("--dry-run", action="store_true", help="不连真机，仅跑 WebXR+IK")
    p.add_argument("--no-home", action="store_true", help="启动时不回零位")
    p.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    p.add_argument("--arm-velocity-limit", type=float, default=20.0, help="关节速度限幅 (rad/s 量级裁剪)")
    p.add_argument("--position-scale", type=float, default=1.0)
    p.add_argument("--rotation-mode", choices=("always", "hold-a", "off"), default="always")
    p.add_argument("--rotation-scale", type=float, default=1.0)
    p.add_argument("--pos-filter-alpha", type=float, default=0.35)
    p.add_argument("--rot-filter-alpha", type=float, default=0.25)
    p.add_argument("--grip-threshold", type=float, default=0.5)
    p.add_argument("--home-cooldown-s", type=float, default=2.0)
    p.add_argument("--print-status", action="store_true")
    p.add_argument("--publish-state", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--state-udp-host", default="127.0.0.1")
    p.add_argument("--state-udp-port", type=int, default=17981)
    p.add_argument("--state-publish-hz", type=float, default=50.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    try:
        DualG1VrTeleop(args).run()
    except KeyboardInterrupt:
        print("\n[System] 用户中断")
        return 130
    except Exception as exc:
        print(f"[System] 错误: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
