#!/usr/bin/env python3
"""Unitree G1 双臂 WebXR 遥操作（Pinocchio IK + DDS 关节 PD）。"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Literal

import numpy as np
import pinocchio as pin

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from common.clutch import controller_relative_delta, target_rotation_from_controller_rel
from common.constants import BTN_A_INDEX, BTN_B_INDEX, HANDS
from common.filters import lerp_position, slerp_filter_quat
from common.math_quat import matrix_to_quat_wxyz
from common.math_se3 import transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop
from config import (
    DEFAULT_CONTROL_HZ,
    DEFAULT_NETWORK_INTERFACE,
    DEFAULT_URDF_PATH,
    DEFAULT_WS_URI,
    R_HEADSET_TO_WORLD,
)
from g1_arm_controller import create_arm_controller
from g1_arm_ik import G1DualArmIK

Side = Literal["left", "right"]


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
    hold_q: np.ndarray | None = None  # Grip 松开后冻结的 7 维关节
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
            publisher_dir = os.path.join(_PROJECT_ROOT, "publisher")
            if publisher_dir not in sys.path:
                sys.path.insert(0, publisher_dir)
            from teleop_state_bridge import TeleopStateSender  # noqa: WPS433

            self.state_sender = TeleopStateSender(self.args.state_udp_host, self.args.state_udp_port)
            print(
                f"[Publisher] 状态上报已启用: udp://{self.args.state_udp_host}:{self.args.state_udp_port} "
                f"@ {self.args.state_publish_hz:.0f}Hz"
            )

    def _is_button_pressed(self, ctrl: dict, index: int) -> bool:
        return is_button_pressed(ctrl, index)

    def _rotation_enabled(self, ctrl: dict) -> bool:
        return rotation_enabled(ctrl, self.args.rotation_mode, btn_a_index=BTN_A_INDEX)

    def _transform_xr_controller(
        self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float
    ) -> tuple[np.ndarray, np.ndarray]:
        return transform_xr_controller(R_HEADSET_TO_WORLD, x, y, z, qx, qy, qz, qw)

    def connect(self) -> None:
        self.arm_ik = G1DualArmIK(urdf_path=self.args.urdf)
        self.arm_ctrl = create_arm_controller(
            dry_run=self.args.dry_run,
            motion_mode=self.args.motion,
            simulation_mode=self.args.sim,
            network_interface=self.args.network_interface,
            arm_velocity_limit=self.args.arm_velocity_limit,
        )
        if not self.args.no_home:
            self._move_to_preferred_posture()
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

    def _move_to_preferred_posture(self, duration_s: float = 2.5) -> None:
        """
        /**
         * @brief 插值到肘部外展偏好姿态（非全零）
         */
        """
        assert self.arm_ctrl is not None and self.arm_ik is not None
        q0 = self.arm_ctrl.get_current_dual_arm_q()
        q1 = self.arm_ik.preferred_q()
        print("[G1] 移动到肘部外展偏好姿态 ...")
        steps = max(1, int(duration_s / 0.02))
        for i in range(steps + 1):
            a = i / steps
            q = (1.0 - a) * q0 + a * q1
            self.arm_ctrl.ctrl_dual_arm(q, np.zeros(14))
            time.sleep(0.02)
        self.cmd_q = self.arm_ctrl.get_current_dual_arm_q()

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
        # 任意一侧 B 键：双臂回肘部外展偏好姿态
        for s in self.sides.values():
            s.last_home_time = now
        print(f"\n[{side.name}] B 键回肘部外展偏好姿态")
        self._move_to_preferred_posture()
        left_pose, right_pose = self.arm_ik.forward_kinematics(self.cmd_q)
        for s in self.sides.values():
            s.desired_pose = (left_pose if s.side == "left" else right_pose).copy()
            s.hold_q = None
            self._release_clutch(s)

    def _release_clutch(self, side: SideState) -> None:
        if side.is_clutching:
            # 松开瞬间冻结该侧 7 关节，避免另一侧仍在 IK 时被偏好姿态慢慢拉走
            q = self.cmd_q
            if q is None or np.linalg.norm(q) < 1e-12:
                assert self.arm_ctrl is not None
                q = self.arm_ctrl.get_current_dual_arm_q()
            side.hold_q = (q[0:7].copy() if side.side == "left" else q[7:14].copy())
            print(f"\n[{side.name}] Grip 断开（关节已冻结保持）")
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
            side.hold_q = None
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
        delta_m = controller_relative_delta(
            side.ref_controller_xyz, c_xyz, self.args.position_scale
        )
        raw_pos = side.ref_ee_pose.translation + delta_m
        side.filt_pos_m = lerp_position(side.filt_pos_m, raw_pos, self.args.pos_filter_alpha)

        if self._rotation_enabled(ctrl):
            raw_q = target_rotation_from_controller_rel(
                side.ref_controller_quat_wxyz,
                c_quat,
                side.ref_ee_quat_wxyz,
                self.args.rotation_scale,
            )
            side.filt_quat_wxyz = slerp_filter_quat(
                side.filt_quat_wxyz, raw_q, self.args.rot_filter_alpha
            )
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
            # 双侧都松开：继续下发上次 cmd_q，保持冻结姿态（避免运控慢慢拉回）
            if np.any(np.abs(self.cmd_q) > 1e-9):
                self.arm_ctrl.ctrl_dual_arm(self.cmd_q, np.zeros(14))
            return

        # 未接合侧：用冻结关节对应的 FK 作为 IK 目标；接合侧用 desired
        q_hold = self.cmd_q.copy()
        left_side = self.sides.get("left")
        right_side = self.sides.get("right")
        if left_side is not None and not left_side.is_clutching and left_side.hold_q is not None:
            q_hold[0:7] = left_side.hold_q
        if right_side is not None and not right_side.is_clutching and right_side.hold_q is not None:
            q_hold[7:14] = right_side.hold_q

        left_pose, right_pose = self.arm_ik.forward_kinematics(q_hold)
        target_left = left_pose
        target_right = right_pose
        if left_side is not None and left_side.is_clutching and left_side.desired_pose is not None:
            target_left = left_side.desired_pose
        if right_side is not None and right_side.is_clutching and right_side.desired_pose is not None:
            target_right = right_side.desired_pose

        q_now = self.arm_ctrl.get_current_dual_arm_q()
        # 未接合侧用冻结关节作 IK 初值，减少求解器改动该侧
        if left_side is not None and not left_side.is_clutching and left_side.hold_q is not None:
            q_now[0:7] = left_side.hold_q
        if right_side is not None and not right_side.is_clutching and right_side.hold_q is not None:
            q_now[7:14] = right_side.hold_q

        dq_now = self.arm_ctrl.get_current_dual_arm_dq()
        # 未接合侧在 QP 内 mask DoF，切断双臂耦合（比事后写回更稳）
        lock_left = left_side is None or not left_side.is_clutching
        lock_right = right_side is None or not right_side.is_clutching
        sol_q, sol_tau = self.arm_ik.solve_ik(
            target_left,
            target_right,
            q_now,
            dq_now,
            lock_left=lock_left,
            lock_right=lock_right,
        )

        # 再保险：写回 hold_q，并对锁侧清零前馈
        if left_side is not None and not left_side.is_clutching and left_side.hold_q is not None:
            sol_q[0:7] = left_side.hold_q
            sol_tau[0:7] = 0.0
        if right_side is not None and not right_side.is_clutching and right_side.hold_q is not None:
            sol_q[7:14] = right_side.hold_q
            sol_tau[7:14] = 0.0

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
        await run_webxr_ws_loop(
            self.args.ws_uri,
            lambda payload: setattr(self, "_latest_vr_data", payload),
            control_coro_factory=self.control_loop,
            connected_message="[Network] WebXR 已连接，按住 Grip 控制，B 键回零位",
        )

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
