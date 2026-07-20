#!/usr/bin/env python3
"""JAKA SDK 双臂 WebXR 遥操作（servo_p）。"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from dataclasses import dataclass
from typing import Literal

import numpy as np

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from common.clutch import controller_relative_delta, target_rotation_from_controller_rel
from common.constants import BTN_A_INDEX, BTN_B_INDEX, DEFAULT_WS_URI, HANDS
from common.coord_frames import HEADSET_TO_WORLD_Y_FORWARD
from common.filters import lerp_position, slerp_filter_quat
from common.math_euler import (
    euler_xyz_to_quat_wxyz,
    euler_xyz_to_quat_xyzw,
    quat_wxyz_to_euler_xyz,
)
from common.math_quat import (
    quat_inverse_wxyz,
    quat_multiply_wxyz,
    quaternion_to_angle_axis,
    slerp_quat_wxyz,
)
from common.math_se3 import transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop
from config import HOME_JOINT_DEG
from jaka_sdk_client import ERR_NAMES, JakaSdkError, JakaSdkRobot
from keyboard_teleop import format_pose, move_to_home, tracking_error

# JAKA 安装前向为 +Y
R_HEADSET_TO_WORLD = HEADSET_TO_WORLD_Y_FORWARD

LEFT_ARM_IP = "192.168.10.21"
RIGHT_ARM_IP = "192.168.10.11"
Side = Literal["left", "right"]


@dataclass
class ArmState:
    side: Side
    name: str
    ip: str
    robot: JakaSdkRobot
    cmd_pose: list[float]
    desired_pose: list[float]
    is_clutching: bool = False
    ref_ee_pose: list[float] | None = None
    ref_ee_quat_wxyz: np.ndarray | None = None
    ref_controller_xyz: np.ndarray | None = None
    ref_controller_quat_wxyz: np.ndarray | None = None
    filt_pos_mm: np.ndarray | None = None
    filt_quat_wxyz: np.ndarray | None = None
    last_home_time: float = 0.0
    prev_b_pressed: bool = False
    err_streak: int = 0
    tick: int = 0


class DualJakaVrTeleop:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.active_hands = HANDS if args.hands == "both" else (args.hands,)
        self.arms: dict[str, ArmState] = {}
        self._latest_vr_data: dict | None = None
        self._last_status_len = 0
        self._last_state_publish_time = 0.0
        self._state_publish_interval = 1.0 / max(1.0, float(args.state_publish_hz))
        self.state_sender = None
        if args.publish_state:
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
            publisher_dir = os.path.join(project_root, "publisher")
            if publisher_dir not in sys.path:
                sys.path.insert(0, publisher_dir)
            from teleop_state_bridge import TeleopStateSender  # noqa: WPS433

            self.state_sender = TeleopStateSender(args.state_udp_host, args.state_udp_port)
            print(
                f"[Publisher] 状态上报已启用: udp://{args.state_udp_host}:{args.state_udp_port} "
                f"@ {args.state_publish_hz:.0f}Hz"
            )

    def _is_button_pressed(self, ctrl: dict, index: int) -> bool:
        return is_button_pressed(ctrl, index)

    def _rotation_enabled(self, ctrl: dict) -> bool:
        return rotation_enabled(ctrl, self.args.rotation_mode, btn_a_index=BTN_A_INDEX)

    def _transform_xr_controller(
        self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float
    ) -> tuple[np.ndarray, np.ndarray]:
        return transform_xr_controller(R_HEADSET_TO_WORLD, x, y, z, qx, qy, qz, qw)

    def _connect_arm(self, side: Side, ip: str) -> ArmState:
        name = "左臂" if side == "left" else "右臂"
        robot = JakaSdkRobot(ip)
        last_exc: Exception | None = None
        for attempt in range(1, self.args.login_retry + 1):
            try:
                robot.login()
                break
            except Exception as exc:
                last_exc = exc
                if attempt < self.args.login_retry:
                    print(f"[{name}] login 失败，{self.args.login_retry - attempt} 次重试中...")
                    time.sleep(self.args.login_retry_interval_s)
        else:
            if last_exc is not None:
                raise JakaSdkError(f"[{name}] 登录失败（ip={ip}）: {last_exc}") from last_exc
            raise JakaSdkError(f"[{name}] 登录失败（ip={ip}）")
        robot.ensure_ready(skip_power=self.args.no_power)
        before = robot.get_rapid_rate()
        robot.set_rapid_rate(self.args.rapid_rate)
        after = robot.get_rapid_rate()
        print(f"[{name}] 速度倍率: {before * 100:.0f}% -> {after * 100:.0f}%")
        if not self.args.no_home:
            move_to_home(robot, HOME_JOINT_DEG, speed_deg_s=self.args.home_speed_deg_s)
        robot.prepare_servo(filter=self.args.filter)
        pose = robot.get_tcp_pos()
        print(f"[{name}] 当前 TCP: {format_pose(pose)}")
        return ArmState(
            side=side,
            name=name,
            ip=ip,
            robot=robot,
            cmd_pose=pose.copy(),
            desired_pose=pose.copy(),
        )

    def connect_arms(self) -> None:
        ip_map = {"left": self.args.left_ip, "right": self.args.right_ip}
        connected: list[ArmState] = []
        try:
            for side in self.active_hands:
                arm = self._connect_arm(side, ip_map[side])
                self.arms[side] = arm
                connected.append(arm)
        except Exception:
            for arm in reversed(connected):
                try:
                    self._disconnect_arm(arm)
                except Exception:
                    pass
            self.arms.clear()
            raise

    def _disconnect_arm(self, arm: ArmState) -> None:
        try:
            arm.robot.exit_servo()
        except Exception:
            pass
        try:
            if not self.args.no_shutdown and not self.args.no_power:
                arm.robot.disable_robot()
                arm.robot.power_off()
        except Exception:
            pass
        try:
            arm.robot.logout()
        except Exception:
            pass
        print(f"[{arm.name}] 已退出")

    def disconnect_arms(self) -> None:
        for side in self.active_hands:
            arm = self.arms.get(side)
            if arm is not None:
                self._disconnect_arm(arm)
        self.arms.clear()

    def _go_home(self, arm: ArmState) -> None:
        now = time.time()
        if now - arm.last_home_time < self.args.home_cooldown_s:
            return
        arm.last_home_time = now
        print(f"\n[{arm.name}] B 键回初始关节")
        arm.robot.exit_servo()
        move_to_home(arm.robot, HOME_JOINT_DEG, speed_deg_s=self.args.home_speed_deg_s)
        arm.robot.prepare_servo(filter=self.args.filter)
        pose = arm.robot.get_tcp_pos()
        arm.cmd_pose = pose.copy()
        arm.desired_pose = pose.copy()
        arm.is_clutching = False
        arm.ref_ee_pose = None
        arm.ref_ee_quat_wxyz = None
        arm.ref_controller_xyz = None
        arm.ref_controller_quat_wxyz = None
        arm.filt_pos_mm = None
        arm.filt_quat_wxyz = None

    def _release_clutch(self, arm: ArmState) -> None:
        if arm.is_clutching:
            print(f"\n[{arm.name}] Grip 断开")
        arm.is_clutching = False
        arm.ref_ee_pose = None
        arm.ref_ee_quat_wxyz = None
        arm.ref_controller_xyz = None
        arm.ref_controller_quat_wxyz = None
        arm.filt_pos_mm = None
        arm.filt_quat_wxyz = None

    def _update_from_controller(self, arm: ArmState, ctrl: dict) -> None:
        required = ("grip", "x", "y", "z", "qx", "qy", "qz", "qw")
        if not all(k in ctrl for k in required):
            return

        b_pressed = self._is_button_pressed(ctrl, BTN_B_INDEX)
        if b_pressed and not arm.prev_b_pressed:
            self._go_home(arm)
        arm.prev_b_pressed = b_pressed

        grip_pressed = float(ctrl["grip"]) > self.args.grip_threshold
        if not grip_pressed:
            self._release_clutch(arm)
            return

        if not arm.is_clutching:
            arm.ref_ee_pose = arm.robot.get_tcp_pos()
            arm.cmd_pose = arm.ref_ee_pose.copy()
            arm.desired_pose = arm.ref_ee_pose.copy()
            arm.ref_ee_quat_wxyz = euler_xyz_to_quat_wxyz(
                arm.ref_ee_pose[3], arm.ref_ee_pose[4], arm.ref_ee_pose[5]
            )
            arm.is_clutching = True
            arm.ref_controller_xyz = None
            arm.ref_controller_quat_wxyz = None
            arm.filt_pos_mm = None
            arm.filt_quat_wxyz = None
            print(f"\n[{arm.name}] Grip 接合")

        c_xyz, c_quat = self._transform_xr_controller(
            float(ctrl["x"]),
            float(ctrl["y"]),
            float(ctrl["z"]),
            float(ctrl["qx"]),
            float(ctrl["qy"]),
            float(ctrl["qz"]),
            float(ctrl["qw"]),
        )
        if arm.ref_controller_xyz is None or arm.ref_controller_quat_wxyz is None:
            arm.ref_controller_xyz = c_xyz.copy()
            arm.ref_controller_quat_wxyz = c_quat.copy()
            return

        ref = arm.ref_ee_pose if arm.ref_ee_pose is not None else arm.cmd_pose
        if arm.ref_ee_quat_wxyz is None:
            arm.ref_ee_quat_wxyz = euler_xyz_to_quat_wxyz(ref[3], ref[4], ref[5])

        # 平移：手柄增量（米）-> 基座 mm，再做一阶低通
        delta_m = controller_relative_delta(
            arm.ref_controller_xyz, c_xyz, self.args.position_scale
        )
        raw_pos = np.array(
            [
                ref[0] + float(delta_m[0] * 1000.0),
                ref[1] + float(delta_m[1] * 1000.0),
                ref[2] + float(delta_m[2] * 1000.0),
            ],
            dtype=float,
        )
        arm.filt_pos_mm = lerp_position(arm.filt_pos_mm, raw_pos, self.args.pos_filter_alpha)

        desired = ref.copy()
        desired[0] = float(arm.filt_pos_mm[0])
        desired[1] = float(arm.filt_pos_mm[1])
        desired[2] = float(arm.filt_pos_mm[2])

        if self._rotation_enabled(ctrl):
            raw_q = target_rotation_from_controller_rel(
                arm.ref_controller_quat_wxyz,
                c_quat,
                arm.ref_ee_quat_wxyz,
                self.args.rotation_scale,
            )
            arm.filt_quat_wxyz = slerp_filter_quat(
                arm.filt_quat_wxyz, raw_q, self.args.rot_filter_alpha
            )
            rx, ry, rz = quat_wxyz_to_euler_xyz(arm.filt_quat_wxyz)
            desired[3] = rx
            desired[4] = ry
            desired[5] = rz
        else:
            desired[3] = ref[3]
            desired[4] = ref[4]
            desired[5] = ref[5]

        arm.desired_pose = desired

    def _consume_latest_vr_data(self) -> None:
        if self._latest_vr_data is None:
            return
        ctrls = self._latest_vr_data.get("controllers", [])
        for side in self.active_hands:
            ctrl = next((c for c in ctrls if c.get("handedness") == side), None)
            if ctrl is None:
                self._release_clutch(self.arms[side])
                continue
            self._update_from_controller(self.arms[side], ctrl)

    def _step_servo(self, arm: ArmState, dt: float) -> None:
        if arm.is_clutching:
            target = arm.desired_pose
        else:
            target = arm.cmd_pose

        pos_step = self.args.speed_mm_s * dt
        rot_step = float(np.radians(self.args.speed_deg_s)) * dt
        candidate = arm.cmd_pose.copy()
        for i in range(3):
            d = target[i] - arm.cmd_pose[i]
            if abs(d) > pos_step:
                d = pos_step if d > 0 else -pos_step
            candidate[i] = arm.cmd_pose[i] + d

        # 姿态限速：在四元数空间 slerp，避免对欧拉角分量独立限幅导致乱动
        q_cmd = euler_xyz_to_quat_wxyz(arm.cmd_pose[3], arm.cmd_pose[4], arm.cmd_pose[5])
        q_tgt = euler_xyz_to_quat_wxyz(target[3], target[4], target[5])
        if np.dot(q_cmd, q_tgt) < 0.0:
            q_tgt = -q_tgt
        aa = quaternion_to_angle_axis(quat_multiply_wxyz(quat_inverse_wxyz(q_cmd), q_tgt))
        ang = float(np.linalg.norm(aa))
        if ang <= 1e-9 or rot_step <= 0.0:
            q_next = q_cmd
        elif ang <= rot_step:
            q_next = q_tgt
        else:
            q_next = slerp_quat_wxyz(q_cmd, q_tgt, rot_step / ang)
        rx, ry, rz = quat_wxyz_to_euler_xyz(q_next)
        candidate[3] = rx
        candidate[4] = ry
        candidate[5] = rz

        errno = arm.robot.servo_p(candidate)
        if errno == 0:
            arm.cmd_pose = candidate
            arm.err_streak = 0
        else:
            arm.err_streak += 1
            if arm.err_streak <= 5:
                name = ERR_NAMES.get(errno, "unknown")
                print(f"\n[{arm.name}] servo_p errno={errno} ({name})")
            arm.cmd_pose = arm.robot.get_tcp_pos()
            arm.desired_pose = arm.cmd_pose.copy()
            return

        if arm.tick % self.args.safety_check_interval == 0:
            actual = arm.robot.get_tcp_pos()
            joints = arm.robot.get_joint_pos_rad()
            ik = arm.robot.kine_inverse(joints, arm.cmd_pose, raise_on_error=False)
            if ik is None:
                arm.desired_pose = actual.copy()
                arm.cmd_pose = actual.copy()
                print(f"\n[{arm.name}] 逆解失败，重同步")
            else:
                pe, re = tracking_error(actual, arm.cmd_pose)
                if pe > self.args.max_track_err_mm or re > self.args.max_track_err_rad:
                    arm.desired_pose = actual.copy()
                    arm.cmd_pose = actual.copy()
                    print(f"\n[{arm.name}] 跟踪误差过大 xyz={pe:.1f}mm，重同步")

        arm.tick += 1

    def _build_side_state(self, side: str) -> dict | None:
        arm = self.arms.get(side)
        if arm is None:
            return None
        pose = arm.robot.get_tcp_pos()
        joints = arm.robot.get_joint_pos_rad()
        qx, qy, qz, qw = euler_xyz_to_quat_xyzw(pose[3], pose[4], pose[5])
        return {
            "arm_valid": True,
            "hand_valid": False,
            "arm_joints": [float(v) for v in joints],
            "end_pose": {
                "x": float(pose[0] / 1000.0),
                "y": float(pose[1] / 1000.0),
                "z": float(pose[2] / 1000.0),
                "qx": float(qx),
                "qy": float(qy),
                "qz": float(qz),
                "qw": float(qw),
            },
            "hand_joints": [0.0] * 6,
        }

    def _maybe_publish_state(self) -> None:
        if self.state_sender is None:
            return
        now = time.time()
        if now - self._last_state_publish_time < self._state_publish_interval:
            return
        payload = {"stamp": now, "left": None, "right": None}
        for side in HANDS:
            payload[side] = self._build_side_state(side)
        self.state_sender.send_dict(payload)
        self._last_state_publish_time = now

    async def control_loop(self) -> None:
        period = max(0.004, self.args.period)
        while True:
            start = time.perf_counter()
            self._consume_latest_vr_data()
            for side in self.active_hands:
                self._step_servo(self.arms[side], period)
            self._maybe_publish_state()
            if self.args.print_status:
                status = []
                for side in self.active_hands:
                    p = self.arms[side].cmd_pose
                    status.append(f"{side}:({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})")
                line = "\r[JAKA-VR] " + " | ".join(status)
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
            connected_message="[Network] WebXR 已连接，按住 Grip 控制，B 键回初始关节",
        )

    def run(self) -> None:
        self.connect_arms()
        try:
            asyncio.run(self.ws_loop())
        finally:
            if self.args.print_status:
                print("")
            if self.state_sender is not None:
                self.state_sender.close()
            self.disconnect_arms()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="JAKA 双臂 WebXR servo_p 遥操作")
    p.add_argument("--hands", choices=("both", "left", "right"), default="both")
    p.add_argument("--left-ip", default=LEFT_ARM_IP)
    p.add_argument("--right-ip", default=RIGHT_ARM_IP)
    p.add_argument("--ws-uri", default=DEFAULT_WS_URI)
    p.add_argument("--no-power", action="store_true")
    p.add_argument("--no-shutdown", action="store_true")
    p.add_argument("--no-home", action="store_true")
    p.add_argument("--filter", choices=("none", "lpf", "carte"), default="lpf")
    p.add_argument("--rapid-rate", type=float, default=0.9)
    p.add_argument("--period", type=float, default=0.008)
    p.add_argument("--speed-mm-s", type=float, default=160.0)
    p.add_argument("--speed-deg-s", type=float, default=60.0)
    p.add_argument("--position-scale", type=float, default=1.0)
    p.add_argument("--rotation-mode", choices=("always", "hold-a", "off"), default="always")
    p.add_argument("--rotation-scale", type=float, default=1.0)
    p.add_argument(
        "--pos-filter-alpha",
        type=float,
        default=0.35,
        help="平移目标一阶低通系数 (0~1，越大越跟手)",
    )
    p.add_argument(
        "--rot-filter-alpha",
        type=float,
        default=0.25,
        help="姿态目标 slerp 低通系数 (0~1，越大越跟手)",
    )
    p.add_argument("--grip-threshold", type=float, default=0.5)
    p.add_argument(
        "--home-speed-deg-s",
        type=float,
        default=45.0,
        help="回初始关节的关节速度 (deg/s)，原默认 15",
    )
    p.add_argument("--home-cooldown-s", type=float, default=2.0)
    p.add_argument("--safety-check-interval", type=int, default=50)
    p.add_argument("--max-track-err-mm", type=float, default=80.0)
    p.add_argument("--max-track-err-rad", type=float, default=float(np.radians(15.0)))
    p.add_argument("--print-status", action="store_true")
    p.add_argument("--login-retry", type=int, default=3)
    p.add_argument("--login-retry-interval-s", type=float, default=1.0)
    p.add_argument("--publish-state", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--state-udp-host", default="127.0.0.1")
    p.add_argument("--state-udp-port", type=int, default=17981)
    p.add_argument("--state-publish-hz", type=float, default=50.0)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    try:
        DualJakaVrTeleop(args).run()
    except KeyboardInterrupt:
        print("\n[System] 用户中断")
        return 130
    except JakaSdkError as exc:
        print(f"[System] 错误: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
