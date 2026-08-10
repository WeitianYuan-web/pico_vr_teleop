#!/usr/bin/env python3
"""Noetix M1 dual-arm WebXR teleop (Cartesian mode via CycloneDDS)."""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Literal, Optional, Sequence

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
from common.math_quat import quat_diff_as_angle_axis, slerp_quat_wxyz
from common.math_se3 import apply_delta_rotation, transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop
from config import (
    DEFAULT_AXIS_SIGN,
    DEFAULT_AXIS_SIGN_LEFT,
    DEFAULT_AXIS_SIGN_RIGHT,
    DEFAULT_BOX_HOLD_S,
    DEFAULT_BOX_INTERP_S,
    DEFAULT_CART_HOME_INTERP_S,
    DEFAULT_CONTROL_HZ,
    DEFAULT_COORD_PRESET,
    DEFAULT_CYCLONEDDS_XML,
    DEFAULT_GRIP_ENGAGE,
    DEFAULT_GRIP_RELEASE,
    DEFAULT_HARDWARE_CONFIG,
    DEFAULT_HOME_COOLDOWN_S,
    DEFAULT_LEFT_BOX,
    DEFAULT_LOCAL_IP,
    DEFAULT_MAX_JOINT_STEP_RAD,
    DEFAULT_MAX_STEP_M,
    DEFAULT_MODE_SETTLE_S,
    DEFAULT_PEER_IP,
    DEFAULT_POS_DEADZONE_M,
    DEFAULT_POS_FILTER_ALPHA,
    DEFAULT_POSITION_SCALE,
    DEFAULT_RIGHT_BOX,
    DEFAULT_ROT_DEADZONE_DEG,
    DEFAULT_ROT_FILTER_ALPHA,
    DEFAULT_ROTATION_MODE,
    DEFAULT_ROTATION_SCALE,
    DEFAULT_WS_URI_NOETIX,
    resolve_axis_sign,
    resolve_headset_to_world,
)
from ros_cartesian import ArmCartesianPose, NoetixRosCartesian, RobotSnapshot

Side = Literal["left", "right"]


def _lerp(a: Sequence[float], b: Sequence[float], ratio: float) -> List[float]:
    r = max(0.0, min(1.0, ratio))
    return [sa + (sb - sa) * r for sa, sb in zip(a, b)]


def _rate_limit_vec(
    current: Sequence[float], target: Sequence[float], max_step: float
) -> List[float]:
    out: List[float] = []
    for c, t in zip(current, target):
        d = t - c
        if d > max_step:
            out.append(c + max_step)
        elif d < -max_step:
            out.append(c - max_step)
        else:
            out.append(t)
    return out


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
    prev_grip: float = 0.0


@dataclass
class DualNoetixVrTeleop:
    args: argparse.Namespace
    active_hands: tuple[str, ...] = field(init=False)
    sides: dict[str, SideState] = field(default_factory=dict)
    ros: NoetixRosCartesian | None = None
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
    _legs_hold: List[float] | None = None
    _right_grip: float = 2.30
    _left_grip: float = 2.30
    _spin_stop: threading.Event = field(default_factory=threading.Event)
    _spin_thread: threading.Thread | None = None

    def __post_init__(self) -> None:
        self.r_headset_to_world = resolve_headset_to_world(self.args.coord_preset)
        # Per-arm defaults: left flips X/Y (FB+LR inverted on hardware); right identity.
        # --axis-sign-left/right override a side; bare --axis-sign overrides both.
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
            f"[Noetix] 坐标系: preset={self.args.coord_preset} "
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
            f"[Publisher] Noetix 状态 → udp://{self.args.state_udp_host}:"
            f"{self.args.state_udp_port} @ {self.args.state_publish_hz:.0f}Hz"
        )

    def _start_ros_spin(self) -> None:
        assert self.ros is not None

        def _loop() -> None:
            import rclpy

            while not self._spin_stop.is_set() and rclpy.ok():
                rclpy.spin_once(self.ros, timeout_sec=0.01)

        self._spin_stop.clear()
        self._spin_thread = threading.Thread(target=_loop, name="noetix-ros-spin", daemon=True)
        self._spin_thread.start()

    def _stop_ros_spin(self) -> None:
        self._spin_stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None

    def _pose_from_side(self, side: str) -> ArmCartesianPose:
        st = self.sides[side]
        assert st.desired_pos is not None and st.desired_quat_wxyz is not None
        q = st.desired_quat_wxyz
        return ArmCartesianPose(
            x=float(st.desired_pos[0]),
            y=float(st.desired_pos[1]),
            z=float(st.desired_pos[2]),
            qw=float(q[0]),
            qx=float(q[1]),
            qy=float(q[2]),
            qz=float(q[3]),
        )

    def _seed_side_from_ee(self, side: str, ee: ArmCartesianPose, *, as_home: bool) -> None:
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
        delta = target - st.desired_pos
        n = float(np.linalg.norm(delta))
        max_step = float(self.args.max_step_m)
        if n <= max_step:
            return target.copy()
        return st.desired_pos + delta * (max_step / n)

    def _publish_cartesian_hold(self) -> None:
        assert self.ros is not None
        right = self._pose_from_side("right") if "right" in self.sides else None
        left = self._pose_from_side("left") if "left" in self.sides else None
        # Always send both arms; fill missing from latest measured.
        if right is None:
            assert self.ros.latest_right_ee is not None
            right = self.ros.latest_right_ee.copy()
        if left is None:
            assert self.ros.latest_left_ee is not None
            left = self.ros.latest_left_ee.copy()
        self.ros.publish_mode(2)
        self.ros.publish_cartesian(
            right, left, self._right_grip, self._left_grip, legs_hold=self._legs_hold
        )

    def _freeze_ee_pair(self) -> tuple[ArmCartesianPose, ArmCartesianPose]:
        """Snapshot current EE once; used as a fixed hold through mode switches."""
        assert self.ros is not None
        assert self.ros.latest_right_ee is not None
        assert self.ros.latest_left_ee is not None
        return self.ros.latest_right_ee.copy(), self.ros.latest_left_ee.copy()

    def _stream_cartesian_fixed(
        self,
        right: ArmCartesianPose,
        left: ArmCartesianPose,
        *,
        duration_s: float,
        also_request_mode2: bool = True,
    ) -> None:
        """Publish an unchanged EE target at control_hz (no chasing measured)."""
        assert self.ros is not None
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        t_end = time.time() + max(0.0, float(duration_s))
        while time.time() < t_end:
            if also_request_mode2:
                self.ros.publish_mode(2)
            self.ros.publish_cartesian(
                right, left, self._right_grip, self._left_grip, legs_hold=self._legs_hold
            )
            time.sleep(dt)

    def _switch_to_cartesian_smooth(
        self, right: ArmCartesianPose, left: ArmCartesianPose
    ) -> None:
        """
        Enter mode2 while streaming a *frozen* EE hold.

        Chasing live measured EE / snapping quat during the switch is what caused
        the large jerk after B-home joint motion finished.
        """
        assert self.ros is not None
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        switch_deadline = time.time() + float(self.args.mode_switch_timeout_s)
        print("[Noetix] 请求 mode2，冻结当前 EE 流式保持…")
        while time.time() < switch_deadline:
            self.ros.publish_mode(2)
            self.ros.publish_cartesian(
                right, left, self._right_grip, self._left_grip, legs_hold=self._legs_hold
            )
            if int(self.ros.work_mode) == 2:
                break
            time.sleep(dt)
        else:
            raise TimeoutError(
                f"Noetix mode2 timeout (work_mode={self.ros.work_mode})"
            )

        settle = float(self.args.mode_settle_s)
        if settle > 0.0:
            print(f"[Noetix] mode2 已确认，冻结 EE 再保持 {settle:.2f}s …")
            self._stream_cartesian_fixed(right, left, duration_s=settle, also_request_mode2=True)

    def _run_box_then_cartesian(self, *, label: str = "启动") -> None:
        assert self.ros is not None
        print(f"[Noetix] {label}: 等待 /Robot_Status_Topic snapshot ...")
        deadline = time.time() + float(self.args.snapshot_timeout_s)
        snap: Optional[RobotSnapshot] = None
        while time.time() < deadline:
            snap = self.ros.get_snapshot()
            if snap is not None:
                break
            time.sleep(0.05)
        if snap is None:
            raise TimeoutError("Noetix snapshot timeout")

        self._legs_hold = list(snap.legs)
        self._right_grip = self.ros.gains.gripper_open
        self._left_grip = self.ros.gains.gripper_open
        print(
            f"[Noetix] snapshot ok mode={snap.work_mode} "
            f"R_EE=({snap.right_ee.x:.3f},{snap.right_ee.y:.3f},{snap.right_ee.z:.3f}) "
            f"L_EE=({snap.left_ee.x:.3f},{snap.left_ee.y:.3f},{snap.left_ee.z:.3f})"
        )

        final_right_joints = list(self.args.right_box_joints)
        final_left_joints = list(self.args.left_box_joints)
        if not self.args.skip_box_pose:
            print(f"[Noetix] mode1 抱箱关节插值 {self.args.box_interp_s:.1f}s ...")
            right_cmd = list(snap.right_joints)
            left_cmd = list(snap.left_joints)
            t0 = time.time()
            dt = 1.0 / max(1.0, float(self.args.control_hz))
            while True:
                now = time.time()
                elapsed = now - t0
                ratio = (
                    1.0
                    if self.args.box_interp_s <= 0
                    else min(1.0, elapsed / float(self.args.box_interp_s))
                )
                desired_r = _lerp(snap.right_joints, final_right_joints, ratio)
                desired_l = _lerp(snap.left_joints, final_left_joints, ratio)
                right_cmd = _rate_limit_vec(
                    right_cmd, desired_r, float(self.args.max_joint_step_rad)
                )
                left_cmd = _rate_limit_vec(
                    left_cmd, desired_l, float(self.args.max_joint_step_rad)
                )
                self.ros.publish_mode(1)
                self.ros.publish_motor(
                    right_cmd,
                    left_cmd,
                    self._legs_hold,
                    self._right_grip,
                    self._left_grip,
                )
                if ratio >= 1.0 and elapsed >= float(self.args.box_interp_s) + float(
                    self.args.box_hold_s
                ):
                    break
                time.sleep(dt)
            # Extra mode1 hold at final joints so EE feedback settles before mode2.
            print("[Noetix] 抱箱关节到位，mode1 再稳一下…")
            hold_extra = max(0.3, float(self.args.box_hold_s) * 0.5)
            t_hold = time.time() + hold_extra
            while time.time() < t_hold:
                self.ros.publish_mode(1)
                self.ros.publish_motor(
                    final_right_joints,
                    final_left_joints,
                    self._legs_hold,
                    self._right_grip,
                    self._left_grip,
                )
                time.sleep(dt)
            print("[Noetix] 抱箱到位，切换笛卡尔 mode2 …")
        else:
            print("[Noetix] skip_box_pose：直接切笛卡尔 mode2")

        # Freeze EE *once* at the end of joint phase; do not chase measured during switch.
        freeze_r, freeze_l = self._freeze_ee_pair()
        print(
            f"[Noetix] 冻结 EE "
            f"R=({freeze_r.x:.3f},{freeze_r.y:.3f},{freeze_r.z:.3f}) "
            f"L=({freeze_l.x:.3f},{freeze_l.y:.3f},{freeze_l.z:.3f})"
        )
        self._switch_to_cartesian_smooth(freeze_r, freeze_l)

        if "right" in self.sides:
            self._seed_side_from_ee("right", freeze_r, as_home=True)
        if "left" in self.sides:
            self._seed_side_from_ee("left", freeze_l, as_home=True)
        print(f"[Noetix] {label}完成：笛卡尔 mode2 已激活，Grip 接合开始遥操作")

    def _current_or_desired_ee(self, side: str) -> ArmCartesianPose:
        st = self.sides[side]
        if st.desired_pos is not None and st.desired_quat_wxyz is not None:
            return self._pose_from_side(side)
        measured = (
            self.ros.latest_left_ee if side == "left" else self.ros.latest_right_ee
        )
        if measured is None:
            raise RuntimeError(f"Noetix {side} EE unavailable for cartesian home")
        return measured.copy()

    def _home_ee_pose(self, side: str) -> Optional[ArmCartesianPose]:
        st = self.sides[side]
        if st.home_pos is None or st.home_quat_wxyz is None:
            return None
        q = st.home_quat_wxyz
        return ArmCartesianPose(
            x=float(st.home_pos[0]),
            y=float(st.home_pos[1]),
            z=float(st.home_pos[2]),
            qw=float(q[0]),
            qx=float(q[1]),
            qy=float(q[2]),
            qz=float(q[3]),
        )

    def _cartesian_go_home(self, *, label: str = "B 回位") -> None:
        """
        Stay in mode2 and move EE targets back to the saved home pose.

        Avoids mode1↔mode2 switching, which was the main source of B-home jerk.
        Falls back to joint box bring-up only if mode2/home is unavailable.
        """
        assert self.ros is not None
        active = [s for s in ("right", "left") if s in self.sides]
        homes = {s: self._home_ee_pose(s) for s in active}
        if int(self.ros.work_mode) != 2 or any(h is None for h in homes.values()):
            print(f"[Noetix] {label}: 无笛卡尔 home/mode2，回退到抱箱切模式")
            self._run_box_then_cartesian(label=label)
            return

        # Both arms are always published together; inactive side holds measured EE.
        start_r = (
            self._current_or_desired_ee("right")
            if "right" in self.sides
            else (
                self.ros.latest_right_ee.copy()
                if self.ros.latest_right_ee is not None
                else homes.get("right")  # type: ignore[arg-type]
            )
        )
        start_l = (
            self._current_or_desired_ee("left")
            if "left" in self.sides
            else (
                self.ros.latest_left_ee.copy()
                if self.ros.latest_left_ee is not None
                else homes.get("left")  # type: ignore[arg-type]
            )
        )
        if start_r is None or start_l is None:
            print(f"[Noetix] {label}: EE 不可用，回退到抱箱切模式")
            self._run_box_then_cartesian(label=label)
            return

        home_r = homes.get("right") or start_r
        home_l = homes.get("left") or start_l
        cmd_r = start_r.copy()
        cmd_l = start_l.copy()
        start_r_q = np.array([start_r.qw, start_r.qx, start_r.qy, start_r.qz], dtype=float)
        start_l_q = np.array([start_l.qw, start_l.qx, start_l.qy, start_l.qz], dtype=float)
        goal_r_q = np.array([home_r.qw, home_r.qx, home_r.qy, home_r.qz], dtype=float)
        goal_l_q = np.array([home_l.qw, home_l.qx, home_l.qy, home_l.qz], dtype=float)
        for q in (start_r_q, start_l_q, goal_r_q, goal_l_q):
            n = float(np.linalg.norm(q))
            if n > 1e-12:
                q /= n

        interp_s = max(0.0, float(self.args.cart_home_interp_s))
        max_step = float(self.args.max_step_m)
        dt = 1.0 / max(1.0, float(self.args.control_hz))
        print(
            f"[Noetix] {label}: 保持 mode2，笛卡尔回 home "
            f"({interp_s:.1f}s, max_step={max_step:.4f}m)"
        )
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            ratio = 1.0 if interp_s <= 0.0 else min(1.0, elapsed / interp_s)
            desired_r_pos = _lerp(
                [start_r.x, start_r.y, start_r.z],
                [home_r.x, home_r.y, home_r.z],
                ratio,
            )
            desired_l_pos = _lerp(
                [start_l.x, start_l.y, start_l.z],
                [home_l.x, home_l.y, home_l.z],
                ratio,
            )
            cmd_r_pos = _rate_limit_vec(
                [cmd_r.x, cmd_r.y, cmd_r.z], desired_r_pos, max_step
            )
            cmd_l_pos = _rate_limit_vec(
                [cmd_l.x, cmd_l.y, cmd_l.z], desired_l_pos, max_step
            )
            cmd_r_q = slerp_quat_wxyz(start_r_q, goal_r_q, ratio)
            cmd_l_q = slerp_quat_wxyz(start_l_q, goal_l_q, ratio)
            cmd_r = ArmCartesianPose(
                x=float(cmd_r_pos[0]),
                y=float(cmd_r_pos[1]),
                z=float(cmd_r_pos[2]),
                qw=float(cmd_r_q[0]),
                qx=float(cmd_r_q[1]),
                qy=float(cmd_r_q[2]),
                qz=float(cmd_r_q[3]),
            )
            cmd_l = ArmCartesianPose(
                x=float(cmd_l_pos[0]),
                y=float(cmd_l_pos[1]),
                z=float(cmd_l_pos[2]),
                qw=float(cmd_l_q[0]),
                qx=float(cmd_l_q[1]),
                qy=float(cmd_l_q[2]),
                qz=float(cmd_l_q[3]),
            )
            self.ros.publish_mode(2)
            self.ros.publish_cartesian(
                cmd_r, cmd_l, self._right_grip, self._left_grip, legs_hold=self._legs_hold
            )
            near_r = np.linalg.norm(
                np.asarray(cmd_r_pos) - np.asarray([home_r.x, home_r.y, home_r.z])
            ) < max_step
            near_l = np.linalg.norm(
                np.asarray(cmd_l_pos) - np.asarray([home_l.x, home_l.y, home_l.z])
            ) < max_step
            if ratio >= 1.0 and near_r and near_l:
                break
            if elapsed > max(interp_s, 0.1) * 3.0 + 2.0:
                print(f"[Noetix] {label}: 笛卡尔回位超时，停止于当前目标")
                break
            time.sleep(dt)

        # Final exact home target, then re-seed teleop state (saved home unchanged).
        self.ros.publish_mode(2)
        self.ros.publish_cartesian(
            home_r, home_l, self._right_grip, self._left_grip, legs_hold=self._legs_hold
        )
        if "right" in self.sides:
            self._seed_side_from_ee("right", home_r, as_home=False)
        if "left" in self.sides:
            self._seed_side_from_ee("left", home_l, as_home=False)
        print(f"[Noetix] {label}完成：仍在 mode2，已回到启动 home EE")

    def _go_home_from_button(self) -> None:
        now = time.time()
        if self._homing or now - self._last_home_time < float(self.args.home_cooldown_s):
            return
        self._last_home_time = now
        self._homing = True
        try:
            for st in self.sides.values():
                self._release_clutch(st)
            print("\n[Noetix] 回位键：不切换模式，笛卡尔直接改目标回 home")
            self._cartesian_go_home(label="B 回位")
        finally:
            self._homing = False

    def _update_from_controller(self, st: SideState, ctrl: dict) -> None:
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
            # Re-seed from measured EE at engage.
            measured = (
                self.ros.latest_left_ee if st.side == "left" else self.ros.latest_right_ee
            )
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
        st.filt_pos = lerp_position(st.filt_pos, raw_pos, float(self.args.pos_filter_alpha))

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
                st.filt_quat_wxyz, raw_q, float(self.args.rot_filter_alpha)
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
        if self._state_sender is None or self.ros is None:
            return
        now = time.time()
        if not hasattr(self, "_last_state_t"):
            self._last_state_t = 0.0
        period = 1.0 / max(1.0, float(self.args.state_publish_hz))
        if now - self._last_state_t < period:
            return
        self._last_state_t = now

        def side_payload(side: str) -> dict:
            st = self.sides[side]
            ee = self.ros.latest_left_ee if side == "left" else self.ros.latest_right_ee
            joints = self.ros.left_joints if side == "left" else self.ros.right_joints
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
            # Prefer commanded pose when clutching for smoother viz; else measured.
            if st.desired_pos is not None and st.desired_quat_wxyz is not None:
                pos = st.desired_pos
                quat = st.desired_quat_wxyz
            else:
                pos = np.array([ee.x, ee.y, ee.z])
                quat = np.array([ee.qw, ee.qx, ee.qy, ee.qz])
            vels = []
            base = 0 if side == "right" else 8
            for i in range(7):
                vels.append(float(self.ros.motor_vel.get(base + i, 0.0)))
            return {
                "arm_valid": True,
                "hand_valid": False,
                "arm_joints": [float(v) for v in joints],
                "arm_velocities": vels,
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
            "left": side_payload("left") if "left" in self.sides else None,
            "right": side_payload("right") if "right" in self.sides else None,
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
                parts.append(f"{st.name}:{flag}[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}]")
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
                self._publish_cartesian_hold()
            self._maybe_publish_state()
            self._print_status()
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, dt - elapsed))

    def setup(self) -> None:
        import rclpy

        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros = NoetixRosCartesian(hardware_config=self.args.hardware_config)
        self._start_ros_spin()
        self._start_state_sender()
        self._run_box_then_cartesian(label="启动初始化")

    def close(self) -> None:
        self._stop_ros_spin()
        if self._state_sender is not None:
            try:
                self._state_sender.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self._state_sender = None
        if self.ros is not None:
            try:
                self.ros.destroy_node()
            except Exception:
                pass
            self.ros = None
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    async def run(self) -> None:
        await run_webxr_ws_loop(
            self.args.ws_uri,
            self._on_vr_payload,
            control_coro_factory=self._control_loop,
            connected_message="[Noetix] WebXR 已连接；Grip 接合，B 回抱箱位",
        )


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Noetix M1 dual-arm WebXR teleop")
    p.add_argument("--ws-uri", default=DEFAULT_WS_URI_NOETIX)
    p.add_argument("--hands", choices=("both", "left", "right"), default="both")
    p.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ)
    p.add_argument("--hardware-config", default=DEFAULT_HARDWARE_CONFIG)
    p.add_argument("--skip-box-pose", action="store_true")
    p.add_argument("--box-interp-s", type=float, default=DEFAULT_BOX_INTERP_S)
    p.add_argument("--box-hold-s", type=float, default=DEFAULT_BOX_HOLD_S)
    p.add_argument(
        "--cart-home-interp-s",
        type=float,
        default=DEFAULT_CART_HOME_INTERP_S,
        help="B 回位：保持 mode2，将 EE 目标插值回启动 home（秒）",
    )
    p.add_argument(
        "--right-box-joints",
        type=float,
        nargs=7,
        default=list(DEFAULT_RIGHT_BOX),
    )
    p.add_argument(
        "--left-box-joints",
        type=float,
        nargs=7,
        default=list(DEFAULT_LEFT_BOX),
    )
    p.add_argument("--max-joint-step-rad", type=float, default=DEFAULT_MAX_JOINT_STEP_RAD)
    p.add_argument("--max-step-m", type=float, default=DEFAULT_MAX_STEP_M)
    p.add_argument("--snapshot-timeout-s", type=float, default=30.0)
    p.add_argument("--mode-switch-timeout-s", type=float, default=20.0)
    p.add_argument(
        "--mode-settle-s",
        type=float,
        default=DEFAULT_MODE_SETTLE_S,
        help="mode2 确认后冻结 EE 保持时长，减轻回位切换抖动",
    )
    p.add_argument("--home-cooldown-s", type=float, default=DEFAULT_HOME_COOLDOWN_S)
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
        help="默认 x_forward（右臂已实机确认，与 G1/tianyee 相同）",
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
        "--cyclonedds-xml",
        default=DEFAULT_CYCLONEDDS_XML,
        help="CycloneDDS URI file (default cartesian_min_ws config)",
    )
    p.add_argument(
        "--expect-local-ip",
        default=DEFAULT_LOCAL_IP,
        help="Warn if this local IP is missing",
    )
    return p


def _local_ipv4_addrs() -> str:
    try:
        import subprocess

        return subprocess.check_output(["ip", "-4", "-o", "addr"], text=True)
    except Exception:
        return ""


def _require_local_ip(expect_ip: str, *, peer_ip: str = DEFAULT_PEER_IP) -> None:
    out = _local_ipv4_addrs()
    if expect_ip and expect_ip in out:
        return
    print(
        f"[Noetix] 错误: 本机没有 IP {expect_ip}，CycloneDDS 无法创建 domain。\n"
        f"  机器人对端: {peer_ip}\n"
        f"  请把网线接到有线网卡（本机常见 enp12s0），然后执行:\n"
        f"    sudo ip link set <iface> up\n"
        f"    sudo ip addr add {expect_ip}/24 dev <iface>\n"
        f"  或改 cyclonedds.xml / 传 --expect-local-ip / --cyclonedds-xml。\n"
        f"  当前 IPv4:\n{out or '    <无法读取 ip addr>'}"
    )
    raise SystemExit(2)


def main() -> int:
    args = build_arg_parser().parse_args()
    if float(args.grip_release) >= float(args.grip_engage):
        args.grip_release = max(0.0, float(args.grip_engage) - 0.2)

    # Ensure CycloneDDS profile is set before rclpy init.
    xml = str(args.cyclonedds_xml)
    if xml and os.path.isfile(xml):
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
        os.environ.setdefault("CYCLONEDDS_URI", f"file://{xml}")
        print(f"[Noetix] RMW={os.environ.get('RMW_IMPLEMENTATION')} CYCLONEDDS_URI={os.environ.get('CYCLONEDDS_URI')}")
    _require_local_ip(str(args.expect_local_ip))

    teleop = DualNoetixVrTeleop(args)
    try:
        teleop.setup()
        asyncio.run(teleop.run())
    except KeyboardInterrupt:
        print("\n[Noetix] 用户中断")
    except Exception as exc:
        # Surface Cyclone/DDS interface errors instead of a bare RCLError.
        msg = str(exc)
        if "error creating node" in msg.lower() or "rmw_create_node" in msg.lower():
            print(
                f"[Noetix] ROS 节点创建失败（通常是 CycloneDDS 网卡/IP 配置）: {exc}\n"
                f"  检查 CYCLONEDDS_URI 与本机是否有 {args.expect_local_ip}"
            )
            return 2
        raise
    finally:
        teleop.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
