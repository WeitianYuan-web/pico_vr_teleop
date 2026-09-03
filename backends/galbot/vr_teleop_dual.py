#!/usr/bin/env python3
"""Galbot G1 dual-arm WebXR teleop (WBC stream or 1.7 Motion EE)."""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Literal, Sequence

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "../.."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from common.clutch import controller_relative_delta
from common.constants import BTN_A_INDEX, BTN_B_INDEX, HANDS
from common.filters import lerp_position, slerp_filter_quat, time_based_alpha
from common.math_euler import quat_wxyz_to_euler_xyz
from common.math_quat import quat_diff_as_angle_axis, slerp_quat_wxyz
from common.math_se3 import apply_delta_rotation, transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop
from config import (
    DEFAULT_AXIS_SIGN,
    DEFAULT_AXIS_SIGN_LEFT,
    DEFAULT_AXIS_SIGN_RIGHT,
    DEFAULT_CONTROL_HZ,
    DEFAULT_COORD_PRESET,
    DEFAULT_GRIP_ENGAGE,
    DEFAULT_GRIP_RELEASE,
    DEFAULT_HOME_COOLDOWN_S,
    DEFAULT_HOME_INTERP_S,
    DEFAULT_HOME_JOINT_TOL_RAD,
    DEFAULT_HOME_POS_TOL_M,
    DEFAULT_HOME_Q_LEFT,
    DEFAULT_HOME_Q_RIGHT,
    DEFAULT_HOME_SPEED_RAD_S,
    DEFAULT_HOME_TIMEOUT_S,
    DEFAULT_HOME_XYZ_LEFT,
    DEFAULT_HOME_XYZ_RIGHT,
    DEFAULT_LOCAL_IP,
    DEFAULT_MAX_STEP_M,
    DEFAULT_POS_DEADZONE_M,
    DEFAULT_POS_FILTER_ALPHA,
    DEFAULT_POSITION_SCALE,
    DEFAULT_ROT_DEADZONE_DEG,
    DEFAULT_ROT_FILTER_ALPHA,
    DEFAULT_ROTATION_MODE,
    DEFAULT_ROTATION_SCALE,
    DEFAULT_TELEOP_MAX_RAD_S,
    DEFAULT_WS_URI_GALBOT,
    resolve_axis_sign,
    resolve_headset_to_world,
)
from sdk_robot import EndEffectorPose, GalbotSdkRobot

Side = Literal["left", "right"]


def _lerp(a: Sequence[float], b: Sequence[float], ratio: float) -> list[float]:
    r = max(0.0, min(1.0, ratio))
    return [sa + (sb - sa) * r for sa, sb in zip(a, b)]


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
    home_pos: np.ndarray | None = None
    home_quat_wxyz: np.ndarray | None = None
    home_q: list[float] | None = None
    prev_grip: float = 0.0
    last_cmd_t: float = 0.0


@dataclass
class DualGalbotVrTeleop:
    args: argparse.Namespace
    active_hands: tuple[str, ...] = field(init=False)
    sides: dict[str, SideState] = field(default_factory=dict)
    robot: GalbotSdkRobot | None = None
    r_headset_to_world: np.ndarray = field(init=False)
    axis_sign_left: np.ndarray = field(init=False)
    axis_sign_right: np.ndarray = field(init=False)
    _latest_vr_data: dict | None = None
    _homing: bool = False
    _home_button_pressed: bool = False
    _last_home_time: float = 0.0
    _last_status_len: int = 0
    _last_status_print_t: float = 0.0
    _state_sender: object | None = None
    _last_state_t: float = 0.0
    _last_idle_refresh_t: float = 0.0

    def __post_init__(self) -> None:
        self.r_headset_to_world = resolve_headset_to_world(self.args.coord_preset)
        if (
            self.args.axis_sign_left is None
            and self.args.axis_sign_right is None
            and list(self.args.axis_sign) != list(DEFAULT_AXIS_SIGN)
        ):
            shared = resolve_axis_sign(self.args.axis_sign)
            self.axis_sign_left = shared.copy()
            self.axis_sign_right = shared.copy()
        else:
            left_raw = (
                self.args.axis_sign_left
                if self.args.axis_sign_left is not None
                else DEFAULT_AXIS_SIGN_LEFT
            )
            right_raw = (
                self.args.axis_sign_right
                if self.args.axis_sign_right is not None
                else DEFAULT_AXIS_SIGN_RIGHT
            )
            self.axis_sign_left = resolve_axis_sign(left_raw)
            self.axis_sign_right = resolve_axis_sign(right_raw)
        self.active_hands = HANDS if self.args.hands == "both" else (self.args.hands,)
        for side in self.active_hands:
            self.sides[side] = SideState(
                side=side,  # type: ignore[arg-type]
                name="左臂" if side == "left" else "右臂",
            )
        print(
            f"[Galbot] 坐标系: preset={self.args.coord_preset} "
            f"L_sign={self.axis_sign_left.tolist()} "
            f"R_sign={self.axis_sign_right.tolist()}"
        )

    def _axis_sign_for(self, side: str) -> np.ndarray:
        return self.axis_sign_left if side == "left" else self.axis_sign_right

    def _start_state_sender(self) -> None:
        if not self.args.publish_state:
            return
        publisher_dir = os.path.join(_ROOT, "publisher")
        if publisher_dir not in sys.path:
            sys.path.insert(0, publisher_dir)
        from teleop_state_bridge import TeleopStateSender

        self._state_sender = TeleopStateSender(
            self.args.state_udp_host, int(self.args.state_udp_port)
        )
        print(
            f"[Publisher] Galbot 状态 → udp://{self.args.state_udp_host}:"
            f"{self.args.state_udp_port} @ {self.args.state_publish_hz:.0f}Hz"
        )

    def _pose_from_side(self, side: str) -> EndEffectorPose | None:
        st = self.sides.get(side)
        if st is None or st.desired_pos is None or st.desired_quat_wxyz is None:
            return None
        q = st.desired_quat_wxyz
        return EndEffectorPose(
            x=float(st.desired_pos[0]),
            y=float(st.desired_pos[1]),
            z=float(st.desired_pos[2]),
            qw=float(q[0]),
            qx=float(q[1]),
            qy=float(q[2]),
            qz=float(q[3]),
        )

    def _seed_side_from_ee(self, side: str, ee: EndEffectorPose, *, as_home: bool) -> None:
        st = self.sides[side]
        pos = np.array([ee.x, ee.y, ee.z], dtype=float)
        quat = np.array([ee.qw, ee.qx, ee.qy, ee.qz], dtype=float)
        n = float(np.linalg.norm(quat))
        if n > 1e-12:
            quat = quat / n
        st.hold_pos = pos.copy()
        st.hold_quat_wxyz = quat.copy()
        st.desired_pos = pos.copy()
        st.desired_quat_wxyz = quat.copy()
        st.filt_pos = pos.copy()
        st.filt_quat_wxyz = quat.copy()
        if as_home:
            st.home_pos = pos.copy()
            st.home_quat_wxyz = quat.copy()

    def _configured_home_q(self, side: str) -> list[float]:
        raw = self.args.home_q_left if side == "left" else self.args.home_q_right
        return [float(v) for v in list(raw)[:7]]

    def _home_q_for(self, side: str) -> list[float]:
        st = self.sides.get(side)
        if st is not None and st.home_q is not None and len(st.home_q) >= 7:
            return [float(v) for v in st.home_q[:7]]
        return self._configured_home_q(side)

    def _capture_home_joints(self) -> None:
        assert self.robot is not None
        self.robot.refresh()
        for side in self.active_hands:
            q = self.robot.arm_joints.get(side)
            if q is not None and len(q) >= 7:
                self.sides[side].home_q = [float(v) for v in q[:7]]
            else:
                self.sides[side].home_q = self._configured_home_q(side)
            print(
                f"[Galbot] {side} home q: "
                f"[{', '.join(f'{v:+.3f}' for v in self.sides[side].home_q)}]"
            )

    def _release_clutch(self, st: SideState) -> None:
        st.is_clutching = False
        st.ref_ee_pos = None
        st.ref_ee_quat_wxyz = None
        st.ref_controller_xyz = None
        st.ref_controller_quat_wxyz = None
        st.rotation_active = False
        st.rot_ref_controller_quat_wxyz = None
        st.rot_ref_ee_quat_wxyz = None
        if st.desired_pos is not None:
            st.hold_pos = st.desired_pos.copy()
        if st.desired_quat_wxyz is not None:
            st.hold_quat_wxyz = st.desired_quat_wxyz.copy()

    @staticmethod
    def _apply_pos_deadzone(delta: np.ndarray, deadzone_m: float) -> np.ndarray:
        n = float(np.linalg.norm(delta))
        if n < deadzone_m:
            return np.zeros(3, dtype=float)
        return delta

    def _apply_rot_deadzone(
        self,
        st: SideState,
        ref_ctrl_q: np.ndarray,
        cur_ctrl_q: np.ndarray,
        ref_ee_q: np.ndarray,
        *,
        rotation_scale: float,
        deadzone_deg: float,
    ) -> np.ndarray:
        rel_aa = quat_diff_as_angle_axis(ref_ctrl_q, cur_ctrl_q)
        rel_aa = rel_aa * self._axis_sign_for(st.side)
        angle = float(np.linalg.norm(rel_aa))
        if angle * 180.0 / math.pi < deadzone_deg:
            return ref_ee_q.copy()
        return apply_delta_rotation(ref_ee_q, rel_aa * float(rotation_scale))

    def _clamp_cmd_step(self, st: SideState, target: np.ndarray) -> np.ndarray:
        assert st.desired_pos is not None
        now = time.time()
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        if st.last_cmd_t > 0.0:
            dt = min(0.1, max(1e-3, now - st.last_cmd_t))
        st.last_cmd_t = now
        max_vel = float(self.args.max_step_m) * max(1.0, float(self.args.control_hz))
        max_step = max_vel * dt
        delta = target - st.desired_pos
        n = float(np.linalg.norm(delta))
        if n <= max_step:
            return target.copy()
        return st.desired_pos + delta * (max_step / n)

    def _publish_ee_hold(self) -> None:
        assert self.robot is not None
        clutching = any(st.is_clutching for st in self.sides.values())
        if self.robot.uses_wbc:
            self.robot.refresh()
        elif clutching or self._homing:
            # 1.7 伺服热路径：每拍 Motion FK / 同步 IK 会把控制环卡成几 Hz。
            pass
        else:
            now = time.time()
            if now - self._last_idle_refresh_t >= 0.2:
                self._last_idle_refresh_t = now
                self.robot.refresh()
            return
        desired: dict[str, EndEffectorPose] = {}
        for side in ("left", "right"):
            if side not in self.sides:
                continue
            st = self.sides[side]
            if not self.robot.uses_wbc and not st.is_clutching and not self._homing:
                continue
            pose = self._pose_from_side(side)
            if pose is None:
                pose = self.robot.latest_ee.get(side)
            if pose is not None:
                desired[side] = pose
        if desired:
            self.robot.send_ee_commands(desired)

    def _home_ee_pose(self, side: str) -> EndEffectorPose | None:
        st = self.sides.get(side)
        if st is None or st.home_pos is None or st.home_quat_wxyz is None:
            return None
        q = st.home_quat_wxyz
        return EndEffectorPose(
            x=float(st.home_pos[0]),
            y=float(st.home_pos[1]),
            z=float(st.home_pos[2]),
            qw=float(q[0]),
            qx=float(q[1]),
            qy=float(q[2]),
            qz=float(q[3]),
        )

    def _current_or_desired_ee(self, side: str) -> EndEffectorPose | None:
        pose = self._pose_from_side(side)
        if pose is not None:
            return pose
        assert self.robot is not None
        measured = self.robot.latest_ee.get(side)
        return measured.copy() if measured is not None else None

    def _cartesian_go_home(self, *, label: str = "B 回位") -> None:
        assert self.robot is not None
        active = [s for s in ("left", "right") if s in self.sides]
        homes = {s: self._home_ee_pose(s) for s in active}
        if any(h is None for h in homes.values()):
            print(f"[Galbot] {label}: 无启动 home EE，跳过")
            return

        self.robot.refresh()
        starts: dict[str, EndEffectorPose] = {}
        for side in ("left", "right"):
            start = (
                self._current_or_desired_ee(side)
                if side in self.sides
                else (
                    self.robot.latest_ee[side].copy()
                    if self.robot.latest_ee.get(side) is not None
                    else None
                )
            )
            if start is None:
                print(f"[Galbot] {label}: {side} EE 不可用，跳过")
                return
            starts[side] = start

        goals = {
            "left": homes.get("left") or starts["left"],
            "right": homes.get("right") or starts["right"],
        }
        start_q = {
            s: np.array([starts[s].qw, starts[s].qx, starts[s].qy, starts[s].qz], dtype=float)
            for s in starts
        }
        goal_q = {
            s: np.array([goals[s].qw, goals[s].qx, goals[s].qy, goals[s].qz], dtype=float)
            for s in goals
        }
        for q in list(start_q.values()) + list(goal_q.values()):
            n = float(np.linalg.norm(q))
            if n > 1e-12:
                q /= n

        interp_s = max(0.0, float(self.args.home_interp_s))
        max_step = float(self.args.max_step_m)
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        print(
            f"[Galbot] {label}: EE 插值回 home "
            f"({interp_s:.1f}s, max_step={max_step:.4f}m)"
        )
        cmd = {s: starts[s].copy() for s in starts}
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            ratio = 1.0 if interp_s <= 0.0 else min(1.0, elapsed / interp_s)
            desired: dict[str, EndEffectorPose] = {}
            near = True
            for side in ("left", "right"):
                desired_pos = _lerp(
                    [starts[side].x, starts[side].y, starts[side].z],
                    [goals[side].x, goals[side].y, goals[side].z],
                    ratio,
                )
                cmd_q = slerp_quat_wxyz(start_q[side], goal_q[side], ratio)
                cur = np.array([cmd[side].x, cmd[side].y, cmd[side].z], dtype=float)
                tgt = np.array(desired_pos, dtype=float)
                delta = tgt - cur
                n = float(np.linalg.norm(delta))
                if n > max_step:
                    tgt = cur + delta * (max_step / n)
                    near = False
                elif n >= max_step * 0.5 and ratio < 1.0:
                    near = False
                cmd[side] = EndEffectorPose(
                    x=float(tgt[0]),
                    y=float(tgt[1]),
                    z=float(tgt[2]),
                    qw=float(cmd_q[0]),
                    qx=float(cmd_q[1]),
                    qy=float(cmd_q[2]),
                    qz=float(cmd_q[3]),
                )
                desired[side] = cmd[side]
            self.robot.send_ee_commands(desired)
            if ratio >= 1.0 and near:
                break
            if elapsed > max(interp_s, 0.1) * 3.0 + 2.0:
                print(f"[Galbot] {label}: 回位超时，停在当前目标")
                break
            time.sleep(dt)

        self.robot.send_ee_commands(goals)
        for side in self.sides:
            goal = goals[side]
            self._seed_side_from_ee(side, goal, as_home=False)
        print(f"[Galbot] {label}完成：已回到启动 home EE")

    def _go_home_from_button(self) -> None:
        now = time.time()
        if self._homing or now - self._last_home_time < float(self.args.home_cooldown_s):
            return
        self._last_home_time = now
        self._homing = True
        try:
            for st in self.sides.values():
                self._release_clutch(st)
            print("\n[Galbot] 回位键：关节回启动初始位（位置+姿态）")
            self._joint_go_home(label="B 回位")
        finally:
            self._homing = False

    def _joint_go_home(self, *, label: str = "B 回位") -> None:
        assert self.robot is not None
        q_left = self._home_q_for("left") if "left" in self.active_hands else None
        q_right = self._home_q_for("right") if "right" in self.active_hands else None
        print(
            f"[Galbot] {label}: 关节空间 "
            f"(speed={float(self.args.home_speed_rad_s):.2f} rad/s)；请让开臂工作空间"
        )
        ok = self.robot.move_arm_joints(
            left=q_left,
            right=q_right,
            speed_rad_s=float(self.args.home_speed_rad_s),
            timeout_s=float(self.args.home_timeout_s),
        )
        settle_deadline = time.time() + 1.0
        while time.time() < settle_deadline:
            self.robot.refresh()
            time.sleep(0.15)
        for side in self.active_hands:
            ee = self.robot.latest_ee.get(side)
            if ee is None:
                print(f"[Galbot] {label}: {side} 回位后仍无 EE")
                continue
            self._seed_side_from_ee(side, ee, as_home=False)
            print(
                f"[Galbot] {label} {side} EE "
                f"[{ee.x:+.3f},{ee.y:+.3f},{ee.z:+.3f}]"
            )
        if not ok:
            print(f"[Galbot] {label}未完全成功，已用当前末端当目标")
        else:
            print(f"[Galbot] {label}完成：已回到启动初始关节")

    def _update_from_controller(self, st: SideState, ctrl: dict) -> None:
        assert self.robot is not None
        grip = float(ctrl.get("grip", 0.0))
        engage = float(self.args.grip_engage)
        release = float(self.args.grip_release)
        was = st.is_clutching
        if not was and grip >= engage:
            st.is_clutching = True
        elif was and grip <= release:
            st.is_clutching = False
        st.prev_grip = grip

        if was and not st.is_clutching:
            self._release_clutch(st)
            print(f"\n[{st.name}] Grip 断开")
            return
        if not st.is_clutching:
            return

        if not was and st.is_clutching:
            if self.robot.local_ik is not None:
                self.robot.refresh(ee=False, joints=True)
                measured = self.robot.fk_ee(st.side)
            else:
                self.robot.refresh()
                measured = self.robot.latest_ee.get(st.side)
            if measured is not None:
                self._seed_side_from_ee(st.side, measured, as_home=False)
            assert st.hold_pos is not None and st.hold_quat_wxyz is not None
            st.ref_ee_pos = st.hold_pos.copy()
            st.ref_ee_quat_wxyz = st.hold_quat_wxyz.copy()
            st.filt_pos = st.hold_pos.copy()
            st.filt_quat_wxyz = st.hold_quat_wxyz.copy()
            st.ref_controller_xyz = None
            st.ref_controller_quat_wxyz = None
            st.rot_ref_controller_quat_wxyz = None
            st.rot_ref_ee_quat_wxyz = None
            st.rotation_active = False
            st.last_cmd_t = 0.0
            print(f"\n[{st.name}] Grip 接合")

        c_xyz, c_quat = transform_xr_controller(
            self.r_headset_to_world,
            float(ctrl["x"]),
            float(ctrl["y"]),
            float(ctrl["z"]),
            float(ctrl["qx"]),
            float(ctrl["qy"]),
            float(ctrl["qz"]),
            float(ctrl["qw"]),
        )
        c_xyz = c_xyz * self._axis_sign_for(st.side)
        if st.ref_controller_xyz is None or st.ref_controller_quat_wxyz is None:
            st.ref_controller_xyz = c_xyz.copy()
            st.ref_controller_quat_wxyz = c_quat.copy()
            return

        assert st.ref_ee_pos is not None and st.ref_ee_quat_wxyz is not None
        delta_m = controller_relative_delta(
            st.ref_controller_xyz, c_xyz, float(self.args.position_scale)
        )
        delta_m = self._apply_pos_deadzone(delta_m, float(self.args.pos_deadzone_m))
        raw_pos = st.ref_ee_pos + delta_m
        now = time.time()
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        if st.last_cmd_t > 0.0:
            dt = min(0.08, max(1e-3, now - st.last_cmd_t))
        hz = max(1.0, float(self.args.control_hz))
        a_pos = float(np.clip(self.args.pos_filter_alpha, 1e-3, 0.999))
        a_rot = float(np.clip(self.args.rot_filter_alpha, 1e-3, 0.999))
        tau_pos = -(1.0 / hz) / math.log(1.0 - a_pos)
        tau_rot = -(1.0 / hz) / math.log(1.0 - a_rot)
        st.filt_pos = lerp_position(st.filt_pos, raw_pos, time_based_alpha(dt, tau_pos))

        rotate_now = rotation_enabled(ctrl, self.args.rotation_mode, btn_a_index=BTN_A_INDEX)
        if rotate_now and not st.rotation_active:
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
                st,
                st.rot_ref_controller_quat_wxyz,
                c_quat,
                st.rot_ref_ee_quat_wxyz,
                rotation_scale=float(self.args.rotation_scale),
                deadzone_deg=float(self.args.rot_deadzone_deg),
            )
            st.filt_quat_wxyz = slerp_filter_quat(
                st.filt_quat_wxyz, raw_q, time_based_alpha(dt, tau_rot)
            )

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
        home_sources = [
            side
            for side, ctrl in ctrl_by_side.items()
            if ctrl is not None and is_button_pressed(ctrl, BTN_B_INDEX)
        ]
        home_pressed = bool(home_sources)
        home_rising = home_pressed and not self._home_button_pressed
        self._home_button_pressed = home_pressed
        if home_rising:
            self._go_home_from_button()
            return

        for side, ctrl in ctrl_by_side.items():
            if ctrl is None:
                continue
            self._update_from_controller(self.sides[side], ctrl)

    def _maybe_publish_state(self) -> None:
        if self._state_sender is None or self.robot is None:
            return
        now = time.time()
        period = 1.0 / max(1.0, float(self.args.state_publish_hz))
        if now - self._last_state_t < period:
            return
        self._last_state_t = now

        def side_payload(side: str) -> dict:
            st = self.sides.get(side)
            ee = self.robot.latest_ee.get(side)
            joints = self.robot.arm_joints.get(side)
            vels = self.robot.arm_velocities.get(side) or []
            if ee is None or joints is None:
                return {
                    "arm_valid": False,
                    "hand_valid": False,
                    "arm_joints": [],
                    "arm_velocities": [],
                    "end_pose": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "qx": 0.0,
                        "qy": 0.0,
                        "qz": 0.0,
                        "qw": 1.0,
                    },
                    "hand_joints": [],
                }
            if st is not None and st.desired_pos is not None and st.desired_quat_wxyz is not None:
                pos = st.desired_pos
                quat = st.desired_quat_wxyz
            else:
                pos = np.array([ee.x, ee.y, ee.z])
                quat = np.array([ee.qw, ee.qx, ee.qy, ee.qz])
            return {
                "arm_valid": True,
                "hand_valid": False,
                "arm_joints": [float(v) for v in joints],
                "arm_velocities": [float(v) for v in vels],
                "end_pose": {
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "qx": float(quat[1]),
                    "qy": float(quat[2]),
                    "qz": float(quat[3]),
                    "qw": float(quat[0]),
                },
                "hand_joints": [],
            }

        payload = {
            "stamp": now,
            "left": side_payload("left"),
            "right": side_payload("right"),
        }
        self._state_sender.send_dict(payload)  # type: ignore[attr-defined]

    def _print_status(self) -> None:
        now = time.time()
        if now - self._last_status_print_t < 0.5:
            return
        self._last_status_print_t = now
        parts = []
        for side in self.active_hands:
            st = self.sides[side]
            flag = "ON " if st.is_clutching else "off"
            if st.desired_pos is not None:
                p = st.desired_pos
                err_m = 0.0
                if self.robot is not None:
                    err_m = float(self.robot.ik_track_err_m.get(side, 0.0) or 0.0)
                extra = f" Δ{err_m*1000:.0f}mm" if st.is_clutching and err_m > 0.02 else ""
                parts.append(
                    f"{st.name}:{flag}[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]{extra}"
                )
            else:
                parts.append(f"{st.name}:{flag}")
        line = " | ".join(parts)
        pad = max(0, self._last_status_len - len(line))
        print("\r" + line + (" " * pad), end="", flush=True)
        self._last_status_len = len(line)

    def _on_vr_payload(self, payload: dict) -> None:
        self._latest_vr_data = payload

    async def _control_loop(self) -> None:
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        while True:
            t0 = time.time()
            self._consume_latest_vr_data()
            if not self._homing:
                self._publish_ee_hold()
            self._maybe_publish_state()
            self._print_status()
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, dt - elapsed))

    def _wait_for_ee(self, timeout_s: float) -> None:
        assert self.robot is not None
        deadline = time.time() + max(0.0, timeout_s)
        while True:
            missing = [
                side
                for side in self.active_hands
                if self.robot.latest_ee.get(side) is None
            ]
            if not missing:
                return
            if time.time() >= deadline:
                status = getattr(self.robot, "_last_motion_status", {})
                raise RuntimeError(
                    f"[Galbot] 启动时读不到 {missing} EE"
                    f"（motion_status={status}；1.7 需要 HPU 上 service_motion_plan 已起来）"
                )
            time.sleep(0.2)
            self.robot.refresh()

    @staticmethod
    def _joints_near(
        current: list[float] | None, target: Sequence[float], tol_rad: float
    ) -> bool:
        if current is None or len(current) < 7 or len(target) < 7:
            return False
        return max(abs(float(a) - float(b)) for a, b in zip(current[:7], target[:7])) <= tol_rad

    def _startup_home_xyz(self, side: str) -> np.ndarray:
        raw = self.args.home_xyz_left if side == "left" else self.args.home_xyz_right
        return np.asarray(list(raw), dtype=float).reshape(3)

    def _goto_startup_home(self) -> None:
        assert self.robot is not None
        if bool(self.args.no_home):
            print("[Galbot] --no-home：保持当前臂姿态作为 home")
            return
        # 默认关节回位。笛卡尔只对齐 xyz，肘/腕拧着也会被当成“已到位”。
        if not bool(self.args.home_use_cartesian):
            self._goto_startup_home_joints()
            return
        self.robot.refresh()
        goals: dict[str, EndEffectorPose] = {}
        for side in self.active_hands:
            ee = self.robot.latest_ee.get(side)
            if ee is None:
                print(f"[Galbot] {side} 无 EE，跳过启动回位")
                continue
            xyz = self._startup_home_xyz(side)
            dist = float(np.linalg.norm(np.array([ee.x, ee.y, ee.z]) - xyz))
            print(
                f"[Galbot] {side} 当前 EE [{ee.x:+.3f},{ee.y:+.3f},{ee.z:+.3f}] "
                f"→ 初始 [{xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f}] "
                f"(Δ{dist*1000:.0f} mm)"
            )
            if dist <= float(self.args.home_pos_tol_m):
                continue
            goals[side] = EndEffectorPose(
                x=float(xyz[0]),
                y=float(xyz[1]),
                z=float(xyz[2]),
                qw=ee.qw,
                qx=ee.qx,
                qy=ee.qy,
                qz=ee.qz,
            )
        if not goals:
            print("[Galbot] 双臂已在笛卡尔初始位附近，跳过启动回位")
            return
        print("[Galbot] 启动笛卡尔回初始位；请让开臂工作空间")
        ok = False
        self._homing = True
        try:
            ok = self.robot.move_ee_poses(
                goals, timeout_s=float(self.args.home_timeout_s)
            )
        finally:
            self._homing = False
        self.robot.refresh()
        for side, goal in goals.items():
            now = self.robot.latest_ee.get(side)
            if now is None:
                print(f"[Galbot] {side} 回位后仍无 EE")
                continue
            err = float(np.linalg.norm(np.array([now.x, now.y, now.z]) - np.array([goal.x, goal.y, goal.z])))
            print(
                f"[Galbot] {side} 回位后 EE [{now.x:+.3f},{now.y:+.3f},{now.z:+.3f}] "
                f"误差 {err*1000:.0f} mm"
            )
        if not ok:
            print("[Galbot] 启动回位未完全成功，改用当前末端当 home")

    def _goto_startup_home_joints(self) -> None:
        assert self.robot is not None
        q_left = list(self.args.home_q_left)
        q_right = list(self.args.home_q_right)
        tol = float(self.args.home_joint_tol_rad)
        self.robot.refresh()
        already = True
        for side, target in (("left", q_left), ("right", q_right)):
            if side not in self.active_hands:
                continue
            current = self.robot.arm_joints.get(side)
            if current is None or len(current) < 7:
                already = False
                print(f"[Galbot] {side} 无关节读数，仍尝试关节回位")
                continue
            dq_deg = [
                float(np.degrees(float(a) - float(b)))
                for a, b in zip(current[:7], target[:7])
            ]
            max_abs = max(abs(v) for v in dq_deg)
            print(
                f"[Galbot] {side} 关节 Δdeg "
                f"[{', '.join(f'{v:+.1f}' for v in dq_deg)}] "
                f"max|{max_abs:.1f}|°"
            )
            if not self._joints_near(current, target, tol):
                already = False
        if already:
            print("[Galbot] 双臂已在初始关节附近，跳过启动回位")
            return
        print(
            "[Galbot] 启动回初始关节 "
            f"(speed={float(self.args.home_speed_rad_s):.2f} rad/s，"
            f"timeout={float(self.args.home_timeout_s):.0f}s)；请让开臂工作空间"
        )
        self._homing = True
        try:
            ok = self.robot.move_arm_joints(
                left=q_left if "left" in self.active_hands else None,
                right=q_right if "right" in self.active_hands else None,
                speed_rad_s=float(self.args.home_speed_rad_s),
                timeout_s=float(self.args.home_timeout_s),
            )
        finally:
            self._homing = False
        if not ok:
            print("[Galbot] 启动回位未完全成功，改用当前末端当 home")
        settle_deadline = time.time() + 2.0
        while time.time() < settle_deadline:
            self.robot.refresh()
            time.sleep(0.2)

    def setup(self) -> None:
        self.robot = GalbotSdkRobot(
            dry_run=bool(self.args.dry_run),
            ee_wait_s=float(self.args.ee_wait_s),
            teleop_max_rad_s=float(self.args.teleop_max_rad_s),
        )
        self.robot.init()
        self._start_state_sender()
        assert self.robot is not None
        self._wait_for_ee(max(3.0, float(self.args.ee_wait_s)))
        self._goto_startup_home()
        self._wait_for_ee(3.0)
        for side in self.active_hands:
            ee = self.robot.latest_ee.get(side)
            if ee is None:
                raise RuntimeError(f"[Galbot] 回位后仍读不到 {side} EE")
            self._seed_side_from_ee(side, ee, as_home=True)
            rpy = quat_wxyz_to_euler_xyz(np.array([ee.qw, ee.qx, ee.qy, ee.qz]))
            print(
                f"[Galbot] {side} home EE: "
                f"[{ee.x:+.3f},{ee.y:+.3f},{ee.z:+.3f}] "
                f"rpy°[{math.degrees(rpy[0]):+.1f},"
                f"{math.degrees(rpy[1]):+.1f},"
                f"{math.degrees(rpy[2]):+.1f}]"
            )
        self._capture_home_joints()

    def close(self) -> None:
        if self._state_sender is not None:
            try:
                self._state_sender.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self._state_sender = None
        if self.robot is not None:
            try:
                self.robot.shutdown()
            except Exception:
                pass
            self.robot = None

    async def run(self) -> None:
        await run_webxr_ws_loop(
            self.args.ws_uri,
            self._on_vr_payload,
            control_coro_factory=self._control_loop,
            connected_message="[Galbot] WebXR 已连接；Grip 接合，B 关节回启动初始位",
        )


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Galbot G1 dual-arm WebXR teleop")
    p.add_argument("--ws-uri", default=DEFAULT_WS_URI_GALBOT)
    p.add_argument("--hands", choices=("both", "left", "right"), default="both")
    p.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    p.add_argument(
        "--home-interp-s",
        type=float,
        default=DEFAULT_HOME_INTERP_S,
        help="保留参数；B 已改走关节回位，不再用 EE 插值时长",
    )
    p.add_argument("--max-step-m", type=float, default=DEFAULT_MAX_STEP_M)
    p.add_argument("--home-cooldown-s", type=float, default=DEFAULT_HOME_COOLDOWN_S)
    p.add_argument(
        "--no-home",
        action="store_true",
        help="启动时不把臂送到初始位，只用当前末端当 home",
    )
    p.add_argument(
        "--home-xyz-left",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=list(DEFAULT_HOME_XYZ_LEFT),
        help="左臂启动初始末端 xyz（m，base_link）",
    )
    p.add_argument(
        "--home-xyz-right",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=list(DEFAULT_HOME_XYZ_RIGHT),
        help="右臂启动初始末端 xyz（m，base_link）",
    )
    p.add_argument(
        "--home-pos-tol-m",
        type=float,
        default=DEFAULT_HOME_POS_TOL_M,
        help="已在笛卡尔初始位附近则跳过启动回位",
    )
    p.add_argument(
        "--home-use-joints",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    p.add_argument(
        "--home-use-cartesian",
        action="store_true",
        help="启动只对齐末端 xyz、保留当时姿态（肘/腕可能拧着）",
    )
    p.add_argument(
        "--home-q-left",
        type=float,
        nargs=7,
        metavar="Q",
        default=list(DEFAULT_HOME_Q_LEFT),
        help="左臂 7 关节初始位（rad）",
    )
    p.add_argument(
        "--home-q-right",
        type=float,
        nargs=7,
        metavar="Q",
        default=list(DEFAULT_HOME_Q_RIGHT),
        help="右臂 7 关节初始位（rad）",
    )
    p.add_argument(
        "--home-speed-rad-s",
        type=float,
        default=DEFAULT_HOME_SPEED_RAD_S,
        help="启动/B 回初始位最大关节速度（rad/s）",
    )
    p.add_argument(
        "--home-timeout-s",
        type=float,
        default=DEFAULT_HOME_TIMEOUT_S,
        help="启动回位阻塞超时（秒）",
    )
    p.add_argument(
        "--home-joint-tol-rad",
        type=float,
        default=DEFAULT_HOME_JOINT_TOL_RAD,
        help="已在初始关节附近则跳过启动回位",
    )
    p.add_argument(
        "--teleop-max-rad-s",
        type=float,
        default=DEFAULT_TELEOP_MAX_RAD_S,
        help="遥操作关节速度上限（rad/s）",
    )
    p.add_argument("--grip-engage", type=float, default=DEFAULT_GRIP_ENGAGE)
    p.add_argument("--grip-release", type=float, default=DEFAULT_GRIP_RELEASE)
    p.add_argument("--pos-deadzone-m", type=float, default=DEFAULT_POS_DEADZONE_M)
    p.add_argument("--rot-deadzone-deg", type=float, default=DEFAULT_ROT_DEADZONE_DEG)
    p.add_argument("--pos-filter-alpha", type=float, default=DEFAULT_POS_FILTER_ALPHA)
    p.add_argument("--rot-filter-alpha", type=float, default=DEFAULT_ROT_FILTER_ALPHA)
    p.add_argument("--rotation-mode", default=DEFAULT_ROTATION_MODE)
    p.add_argument("--position-scale", type=float, default=DEFAULT_POSITION_SCALE)
    p.add_argument("--rotation-scale", type=float, default=DEFAULT_ROTATION_SCALE)
    p.add_argument(
        "--coord-preset",
        choices=("y_forward", "x_forward"),
        default=DEFAULT_COORD_PRESET,
        help="默认 x_forward（与 Unitree G1 / tianyee 相同；轴符号尚未实机确认）",
    )
    p.add_argument(
        "--axis-sign",
        type=float,
        nargs=3,
        metavar=("SX", "SY", "SZ"),
        default=list(DEFAULT_AXIS_SIGN),
        help="左右臂共用轴符号基准，默认 1 1 1",
    )
    p.add_argument(
        "--axis-sign-left",
        type=float,
        nargs=3,
        metavar=("SX", "SY", "SZ"),
        default=None,
        help="仅左臂轴符号（覆盖 --axis-sign）",
    )
    p.add_argument(
        "--axis-sign-right",
        type=float,
        nargs=3,
        metavar=("SX", "SY", "SZ"),
        default=None,
        help="仅右臂轴符号（覆盖 --axis-sign）",
    )
    p.add_argument("--publish-state", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--state-udp-host", default="127.0.0.1")
    p.add_argument("--state-udp-port", type=int, default=17981)
    p.add_argument("--state-publish-hz", type=float, default=30.0)
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="不连 Galbot SDK，用占位 EE 跑 clutch / WSS（调试用）",
    )
    p.add_argument(
        "--ee-wait-s",
        type=float,
        default=8.0,
        help="init 后等待左右末端位姿的秒数（1.7 Motion FK 通常要 1–2s 才有数）",
    )
    p.add_argument(
        "--expect-local-ip",
        default=DEFAULT_LOCAL_IP,
        help="若本机没有该 IP 则告警（默认 Embosa PC 192.168.1.99）",
    )
    return p


def _local_ipv4_addrs() -> str:
    try:
        import subprocess

        return subprocess.check_output(["ip", "-4", "-o", "addr"], text=True)
    except Exception:
        return ""


def _warn_local_ip(expect_ip: str) -> None:
    if not expect_ip:
        return
    out = _local_ipv4_addrs()
    if expect_ip in out:
        return
    print(
        f"[Galbot] 警告: 本机没有 IP {expect_ip}（Embosa 默认 PC 地址）。\n"
        "  对端默认 XCU 192.168.1.66 / HPU 192.168.1.88。\n"
        "  把网卡配到同网段，或改 /data/config/embosa_ip_config.json 后重启 SDK。\n"
        f"  当前 IPv4:\n{out or '    <无法读取 ip addr>'}"
    )


def main() -> int:
    args = build_arg_parser().parse_args()
    if float(args.grip_release) >= float(args.grip_engage):
        args.grip_release = max(0.0, float(args.grip_engage) - 0.2)
    if not args.dry_run:
        _warn_local_ip(str(args.expect_local_ip))

    teleop = DualGalbotVrTeleop(args)
    try:
        teleop.setup()
        asyncio.run(teleop.run())
    except KeyboardInterrupt:
        print("\n[Galbot] 用户中断")
    except RuntimeError as exc:
        print(f"\n[Galbot] {exc}")
        return 2
    finally:
        try:
            teleop.close()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
