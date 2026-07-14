#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)
sys.path.append(os.path.join(_PROJECT_ROOT, "third_party", "pyAgxArm"))

try:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config
    from piper_placo_qp_ik import PiperPlacoConfig, PiperPlacoQPIK, pose6_to_transform, transform_to_pose6
    from run_piper_control import (
        detect_firmware_version,
        resolve_can_backend,
        wait_motion_done,
        wait_robot_comm_ready,
    )
except ImportError as exc:
    print(f"导入依赖失败: {exc}")
    sys.exit(1)

from common.clutch import target_rotation_from_controller_rel
from common.constants import BTN_A_INDEX, BTN_B_INDEX, DEFAULT_WS_URI, HANDS
from common.coord_frames import HEADSET_TO_WORLD_X_FORWARD
from common.filters import EMAFilter, time_based_alpha
from common.math_quat import (
    matrix_to_quat_wxyz,
    quat_diff_as_angle_axis,
    quat_wxyz_to_matrix,
)
from common.math_se3 import apply_delta_pose, transform_xr_controller
from common.vr_input import is_button_pressed, rotation_enabled
from common.ws_client import run_webxr_ws_loop

R_HEADSET_TO_WORLD = HEADSET_TO_WORLD_X_FORWARD

WS_URI = DEFAULT_WS_URI
SCALE_FACTOR = 1.0
ROTATION_SCALE = 1.0
CMD_RATE_HZ = 200
MAX_JOINT_VEL_RADPS = 3.0
JOINT_SMOOTH_TAU_S = 0.05
MAX_POS_SPEED = 0.8
POS_SMOOTH_TAU_S = 0.045
ROT_SMOOTH_TAU_S = 0.06
ELBOW_WEIGHT = 0.02
INIT_JOINTS = [0.0, 1.3, -0.9, 0.0, 0.5, 0.0]
INIT_JOINT_TIMEOUT = 6.0
INIT_JOINT_SETTLE = 0.5
INIT_COMM_READY_TIMEOUT = 15.0
INIT_POST_ENABLE_SETTLE = 0.3
INIT_ABS_X = None
INIT_ABS_Y = None
INIT_ABS_Z = None
INIT_ABS_WAIT = 1.0
HOME_COOLDOWN_S = 2.0
TCP_OFFSET_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, np.pi / 2]
MAX_ROT_RANGE_RAD = np.radians(120)


class WebXRPiperPlacoTeleop:
    def __init__(
        self,
        left_can_port=None,
        right_can_port=None,
        robot_model="piper_h",
        urdf_path=None,
        disable_on_exit=False,
        rotation_mode: str = "always",
        rotation_scale: float = ROTATION_SCALE,
        tcp_offset_pose=None,
        hands=HANDS,
        publish_state: bool = False,
        state_udp_host: str = "127.0.0.1",
        state_udp_port: int = 17981,
        state_publish_hz: float = 50.0,
    ):
        self.active_hands = tuple(hands)
        self.left_can_port = left_can_port
        self.right_can_port = right_can_port
        self.robot_model = robot_model
        self.disable_on_exit = disable_on_exit
        self.rotation_mode = rotation_mode
        self.rotation_scale = rotation_scale
        self.tcp_offset_pose = list(TCP_OFFSET_POSE if tcp_offset_pose is None else tcp_offset_pose)
        if urdf_path is None:
            urdf_path = os.path.join(
                _PROJECT_ROOT,
                "third_party/pyAgxArm/agx_arm_urdf-main/piper_h/urdf/piper_h_description.urdf",
            )
        self.urdf_path = urdf_path

        self.last_schema_warn_time = 0.0
        self._last_status_len = 0
        self._vr_frame_count = 0
        self._vr_rate_window_start = 0.0
        self._loop_iter_count = 0
        self._loop_rate_window_start = 0.0
        self._loop_last_iter_time = 0.0
        self.arms = {}
        for hand in HANDS:
            self.arms[hand] = SimpleNamespace(
                hand=hand,
                can_port=None,
                robot=None,
                gripper=None,
                qp_ik=None,
                is_clutching=False,
                ref_ee_xyz=None,
                ref_ee_quat_wxyz=None,
                ref_controller_xyz=None,
                ref_controller_quat_wxyz=None,
                pos_filter=EMAFilter(tau=POS_SMOOTH_TAU_S),
                rot_filter=EMAFilter(tau=ROT_SMOOTH_TAU_S),   # 绝对姿态滤波（比增量模式更平滑）
                last_gripper_time=0.0,
                prev_b_pressed=False,
                last_home_time=0.0,
                is_homing=False,
                home_pose6=None,
                home_quat_wxyz=None,                # 启动时 ee 初始朝向，用于朝前保护
                _prev_target_xyz=None,
                _prev_target_quat_wxyz=None,
                _joint_interp_q=None,
                _home_tcp_T=None,
                fw_ver=None,
                _last_frame_time=None,
            )
        self.arms["left"].can_port = self.left_can_port
        self.arms["right"].can_port = self.right_can_port
        self._latest_vr_data: dict | None = None

        self.R_headset_world = R_HEADSET_TO_WORLD.copy()
        self.T_flange_tcp = pose6_to_transform(self.tcp_offset_pose)
        self.T_tcp_flange = np.linalg.inv(self.T_flange_tcp)
        self.state_publish_hz = max(1.0, float(state_publish_hz))
        self._state_publish_interval = 1.0 / self.state_publish_hz
        self._last_state_publish_time = 0.0
        self.state_sender = None
        if publish_state:
            publisher_dir = os.path.join(_PROJECT_ROOT, "publisher")
            if publisher_dir not in sys.path:
                sys.path.insert(0, publisher_dir)
            from teleop_state_bridge import TeleopStateSender

            self.state_sender = TeleopStateSender(state_udp_host, state_udp_port)
            print(
                f"[Publisher] 状态上报已启用: udp://{state_udp_host}:{state_udp_port} "
                f"@ {self.state_publish_hz:.0f}Hz"
            )

    def _collect_arm_state(self, side: str) -> dict | None:
        if side not in self.active_hands:
            return None
        arm = self.arms[side]
        if arm.robot is None or arm.qp_ik is None:
            return None
        pose = self._robot_pose(arm)
        if pose is None:
            return None
        pos, rot, q = pose
        quat_wxyz = matrix_to_quat_wxyz(rot)
        return {
            "arm_valid": True,
            "arm_joints": [float(v) for v in q],
            "end_pose": {
                "x": float(pos[0]),
                "y": float(pos[1]),
                "z": float(pos[2]),
                "qx": float(quat_wxyz[1]),
                "qy": float(quat_wxyz[2]),
                "qz": float(quat_wxyz[3]),
                "qw": float(quat_wxyz[0]),
            },
        }

    def _collect_hand_state(self, side: str) -> dict | None:
        return None

    def _merge_side_state(self, side: str) -> dict | None:
        arm = self._collect_arm_state(side)
        hand = self._collect_hand_state(side)
        if arm is None and hand is None:
            return None
        merged = {
            "arm_valid": arm is not None,
            "hand_valid": hand is not None,
            "arm_joints": arm["arm_joints"] if arm else [0.0] * 6,
            "end_pose": (
                arm["end_pose"]
                if arm
                else {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
            ),
            "hand_joints": hand["hand_joints"] if hand else [0.0] * 6,
        }
        return merged

    def _build_state_snapshot(self) -> dict:
        payload = {"stamp": time.time(), "left": None, "right": None}
        for side in HANDS:
            side_state = self._merge_side_state(side)
            if side_state is not None:
                payload[side] = side_state
        return payload

    def _maybe_publish_state(self):
        if self.state_sender is None:
            return
        now = time.time()
        if now - self._last_state_publish_time < self._state_publish_interval:
            return
        self.state_sender.send_dict(self._build_state_snapshot())
        self._last_state_publish_time = now

    def _transform_xr_controller(self, vr_pos: np.ndarray, qx: float, qy: float, qz: float, qw: float):
        """将 WebXR 手柄位姿变换到机器人世界坐标系（对齐 XRoboToolkit _process_xr_pose）。"""
        return transform_xr_controller(
            self.R_headset_world,
            float(vr_pos[0]),
            float(vr_pos[1]),
            float(vr_pos[2]),
            qx,
            qy,
            qz,
            qw,
        )

    def _clamp_orientation_to_range(
        self, q: np.ndarray, ref_q: np.ndarray, max_angle: float
    ) -> np.ndarray:
        """将目标姿态四元数限定在 ref_q 的 max_angle 范围内（朝前保护）。"""
        diff = quat_diff_as_angle_axis(ref_q, q)
        angle = np.linalg.norm(diff)
        if angle <= max_angle or angle < 1e-9:
            return q
        clipped = diff * (max_angle / angle)
        _, clamped = apply_delta_pose(np.zeros(3), ref_q, np.zeros(3), clipped)
        return clamped

    def _process_xr_pose(
        self,
        arm: SimpleNamespace,
        vr_pos: np.ndarray,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        平移：增量控制（相对 grip 接合时的参考位置）。
        姿态：绝对控制（grip 接合时记录 controller 参考姿态，后续按相对四元数映射）。
        该实现遵循行业通用做法：使用 q_rel = q_ref^-1 * q_cur，
        即在手柄局部轴定义下计算相对旋转，再叠加到 ref_ee 姿态。
        返回 (delta_xyz, abs_target_quat_wxyz)。
        """
        controller_xyz, controller_quat_wxyz = self._transform_xr_controller(vr_pos, qx, qy, qz, qw)

        if arm.ref_controller_xyz is None:
            # 首次调用：记录平移和姿态参考。姿态后续按“绝对手柄相对参考”的方式计算。
            arm.ref_controller_xyz = controller_xyz.copy()
            arm.ref_controller_quat_wxyz = controller_quat_wxyz.copy()
            arm.rot_filter.value = None
            return np.zeros(3, dtype=float), arm.ref_ee_quat_wxyz.copy()

        # 平移：增量
        delta_xyz = arm.pos_filter.update(controller_xyz - arm.ref_controller_xyz, dt) * SCALE_FACTOR

        # 姿态：绝对控制（局部相对四元数法）
        raw_q = target_rotation_from_controller_rel(
            arm.ref_controller_quat_wxyz,
            controller_quat_wxyz,
            arm.ref_ee_quat_wxyz,
            self.rotation_scale,
        )
        # 保持四元数半球一致，再做 EMA 平滑
        prev_q = arm.rot_filter.value
        if prev_q is not None and np.dot(prev_q, raw_q) < 0:
            raw_q = -raw_q
        smoothed = arm.rot_filter.update(raw_q, dt)
        norm = np.linalg.norm(smoothed)
        target_quat = smoothed / norm if norm > 1e-12 else arm.ref_ee_quat_wxyz.copy()

        return delta_xyz, target_quat

    def _rotation_enabled(self, ctrl: dict) -> bool:
        return rotation_enabled(ctrl, self.rotation_mode, btn_a_index=BTN_A_INDEX)

    def _is_button_pressed(self, ctrl: dict, index: int) -> bool:
        return is_button_pressed(ctrl, index)

    def _is_b_button_pressed(self, ctrl: dict) -> bool:
        return self._is_button_pressed(ctrl, BTN_B_INDEX)

    def _move_to_init_joints(self, arm: SimpleNamespace, label: str = "初始化") -> bool:
        if arm.robot is None:
            return False
        print(f"\n[Robot-{arm.hand}] {label}：阶段1 move_j -> {INIT_JOINTS}")
        arm.robot.move_j(INIT_JOINTS)
        reached = wait_motion_done(
            arm.robot,
            timeout=INIT_JOINT_TIMEOUT,
            target_joints=INIT_JOINTS,
        )
        if reached:
            print(f"[Robot-{arm.hand}] {label}：阶段1到位")
        else:
            print(f"[Robot-{arm.hand}] {label}：阶段1等待超时，继续执行")
        if INIT_JOINT_SETTLE > 0:
            time.sleep(INIT_JOINT_SETTLE)
        return reached

    def _move_to_home_ee_pose(self, arm: SimpleNamespace, label: str = "初始化") -> bool:
        if arm.robot is None or arm.qp_ik is None or arm.home_pose6 is None:
            return False
        joints = arm.robot.get_joint_angles()
        if joints is not None:
            arm.qp_ik.sync_state_from_joint_positions(list(joints.msg))
        home_tcp_T = arm._home_tcp_T if arm._home_tcp_T is not None else pose6_to_transform(arm.home_pose6)
        home_flange_T = home_tcp_T @ self.T_tcp_flange
        arm.qp_ik.set_target_transform(home_flange_T)
        solved, q_sol = arm.qp_ik.solve()
        if not solved:
            print(f"[Robot-{arm.hand}] {label}：阶段2 QP 求解失败，保持阶段1姿态")
            return False
        arm.robot.move_j(q_sol.tolist())
        xyz = home_tcp_T[:3, 3]
        print(f"[Robot-{arm.hand}] {label}：阶段2 绝对末端 XYZ=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        wait_motion_done(arm.robot, timeout=INIT_JOINT_TIMEOUT, target_joints=q_sol.tolist())
        if INIT_ABS_WAIT > 0:
            time.sleep(INIT_ABS_WAIT)
        return True

    def _move_to_initial_pose(self, arm: SimpleNamespace, label: str = "初始化") -> bool:
        ok_joints = self._move_to_init_joints(arm, label)
        ok_pose = self._move_to_home_ee_pose(arm, label)
        return ok_joints or ok_pose

    def _release_clutch(self, arm: SimpleNamespace):
        if arm.is_clutching:
            print(f"\n[Teleop-{arm.hand}] 🔴 Grip 断开")
        arm.is_clutching = False
        arm.ref_ee_xyz = None
        arm.ref_ee_quat_wxyz = None
        arm.ref_controller_xyz = None
        arm.ref_controller_quat_wxyz = None
        arm.pos_filter.value = None
        arm.rot_filter.value = None
        arm._prev_target_xyz = None
        arm._prev_target_quat_wxyz = None
        arm._joint_interp_q = None

    def _go_home(self, arm: SimpleNamespace):
        now = time.time()
        if arm.is_homing or now - arm.last_home_time < HOME_COOLDOWN_S:
            return
        arm.is_homing = True
        arm.last_home_time = now
        self._release_clutch(arm)
        print(f"\n[Teleop-{arm.hand}] 🏠 B 键：回到初始位置")
        try:
            self._move_to_initial_pose(arm, label="回初始位姿")
        finally:
            arm.is_homing = False
            arm.prev_b_pressed = True

    def _connect_arm(self, arm: SimpleNamespace):
        interface, default_port = resolve_can_backend()
        port = arm.can_port or default_port
        print(f"\n[Robot-{arm.hand}] 连接: {interface}:{port}")
        probe_cfg = create_agx_arm_config(robot=self.robot_model, interface=interface, channel=port)
        probe = AgxArmFactory.create_arm(probe_cfg)
        probe.connect()
        robot_cfg, fw_ver = detect_firmware_version(probe, port, interface, self.robot_model)
        arm.fw_ver = fw_ver
        arm.robot = AgxArmFactory.create_arm(robot_cfg)
        arm.robot.connect()
        print(f"[Robot-{arm.hand}] 固件: {fw_ver}")
        arm.robot.set_tcp_offset(self.tcp_offset_pose)
        print(f"[Robot-{arm.hand}] TCP 偏移: {self.tcp_offset_pose}")
        while not arm.robot.enable():
            time.sleep(0.01)
        print(f"[Robot-{arm.hand}] 已使能")
        if not wait_robot_comm_ready(arm.robot, timeout=INIT_COMM_READY_TIMEOUT):
            print(f"[Robot-{arm.hand}] 警告: 通信未在 {INIT_COMM_READY_TIMEOUT:.0f}s 内稳定，仍尝试初始化")
        arm.robot.set_speed_percent(60)
        arm.robot.set_installation_pos(arm.robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
        arm.robot.set_motion_mode(arm.robot.OPTIONS.MOTION_MODE.J)
        if INIT_POST_ENABLE_SETTLE > 0:
            time.sleep(INIT_POST_ENABLE_SETTLE)
        self._move_to_init_joints(arm, label="启动初始化")
        try:
            arm.gripper = arm.robot.init_effector(arm.robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
            print(f"[Robot-{arm.hand}] 夹爪已加载")
        except Exception as exc:
            print(f"[Robot-{arm.hand}] 夹爪不可用: {exc}")
        arm.qp_ik = PiperPlacoQPIK(
            PiperPlacoConfig(
                urdf_path=self.urdf_path,
                ee_frame="link6",
                dt=1.0 / CMD_RATE_HZ,
                position_weight=1.0,
                orientation_weight=0.05,
                elbow_weight=ELBOW_WEIGHT,
                manipulability_weight=1e-2,
                joints_regularization_weight=1e-4,
            )
        )
        joints = arm.robot.get_joint_angles()
        if joints is not None:
            arm.qp_ik.sync_state_from_joint_positions(list(joints.msg))
        tcp_T = self._get_current_tcp_transform(arm)
        home_tcp_T = tcp_T.copy()
        if INIT_ABS_X is not None:
            home_tcp_T[0, 3] = INIT_ABS_X
        if INIT_ABS_Y is not None:
            home_tcp_T[1, 3] = INIT_ABS_Y
        if INIT_ABS_Z is not None:
            home_tcp_T[2, 3] = INIT_ABS_Z
        arm.home_pose6 = transform_to_pose6(home_tcp_T).tolist()
        arm._home_tcp_T = home_tcp_T
        arm.home_quat_wxyz = matrix_to_quat_wxyz(home_tcp_T[:3, :3])
        if INIT_ABS_X is not None or INIT_ABS_Y is not None or INIT_ABS_Z is not None:
            self._move_to_home_ee_pose(arm, label="启动初始化")
        print(f"[Robot-{arm.hand}] Placo QP IK 初始化完成")

    def connect_robots(self):
        for hand in self.active_hands:
            self._connect_arm(self.arms[hand])

    def _get_current_flange_transform(self, arm: SimpleNamespace) -> np.ndarray:
        return arm.qp_ik.current_flange_transform()

    def _get_current_tcp_transform(self, arm: SimpleNamespace) -> np.ndarray:
        return self._get_current_flange_transform(arm) @ self.T_flange_tcp

    def _robot_pose(self, arm: SimpleNamespace):
        joints = arm.robot.get_joint_angles()
        if joints is None:
            return None
        q = list(joints.msg)
        arm.qp_ik.sync_state_from_joint_positions(q)
        tcp_T = self._get_current_tcp_transform(arm)
        return tcp_T[:3, 3].copy(), tcp_T[:3, :3].copy(), np.array(q, dtype=float)

    def _interpolate_joint_command(
        self, arm: SimpleNamespace, current_q: np.ndarray, target_q: np.ndarray, dt: float
    ) -> np.ndarray:
        if arm._joint_interp_q is None:
            arm._joint_interp_q = current_q.copy()
        alpha = time_based_alpha(dt, JOINT_SMOOTH_TAU_S)
        arm._joint_interp_q = arm._joint_interp_q + alpha * (target_q - arm._joint_interp_q)
        max_step = MAX_JOINT_VEL_RADPS * dt
        dq = np.clip(arm._joint_interp_q - current_q, -max_step, max_step)
        return current_q + dq

    def process_vr_data(self, data: dict, hand: str):
        arm = self.arms[hand]
        ctrl = next((c for c in data.get("controllers", []) if c.get("handedness") == hand), None)
        if ctrl is None:
            if arm.is_clutching:
                self._release_clutch(arm)
            return
        required_keys = ("grip", "trigger", "x", "y", "z", "qx", "qy", "qz", "qw")
        if not all(k in ctrl for k in required_keys):
            now = time.time()
            if now - self.last_schema_warn_time > 2.0:
                self.last_schema_warn_time = now
                print("\n[Teleop] 数据字段不匹配，当前仅支持新版协议: controllers[].{grip,trigger,x,y,z,qx,qy,qz,qw}")
            return
        clutch_val = ctrl["grip"]
        trigger_val = ctrl["trigger"]
        is_clutch_pressed = clutch_val > 0.5
        now = time.time()
        # 本臂两次实际控制 tick 之间的真实间隔，供滤波/插值/限速统一使用，
        # 与 CMD_RATE_HZ 额定值以及实际达到的循环频率解耦。
        nominal_dt = 1.0 / CMD_RATE_HZ
        if arm._last_frame_time is not None:
            real_dt = max(nominal_dt * 0.2, min(now - arm._last_frame_time, nominal_dt * 8.0))
        else:
            real_dt = nominal_dt
        arm._last_frame_time = now
        b_pressed = self._is_b_button_pressed(ctrl)
        if b_pressed and not arm.prev_b_pressed:
            self._go_home(arm)
        arm.prev_b_pressed = b_pressed
        if arm.is_homing:
            return
        if arm.gripper is not None and now - arm.last_gripper_time > 0.1:
            target_width = 0.07 * (1.0 - float(trigger_val))
            arm.gripper.move_gripper_m(target_width, force=1.0)
            arm.last_gripper_time = now
        vr_pos = np.array([ctrl["x"], ctrl["y"], ctrl["z"]], dtype=float)
        if is_clutch_pressed and not arm.is_clutching:
            pose = self._robot_pose(arm)
            if pose is None:
                return
            rob_pos, rob_rot, _ = pose
            arm.is_clutching = True
            arm.ref_ee_xyz = rob_pos.copy()
            arm.ref_ee_quat_wxyz = matrix_to_quat_wxyz(rob_rot)
            arm.ref_controller_xyz = None
            arm.ref_controller_quat_wxyz = None
            arm.pos_filter.value = None
            arm.rot_filter.value = None
            print(f"\n[Teleop-{hand}] 🟢 Grip 接合 (z={rob_pos[2]:.3f})")
            return
        if (not is_clutch_pressed) and arm.is_clutching:
            self._release_clutch(arm)
            return
        if not (is_clutch_pressed and arm.is_clutching):
            return
        # 返回：平移增量 + 绝对目标姿态四元数
        delta_xyz, target_quat_wxyz = self._process_xr_pose(
            arm, vr_pos, ctrl["qx"], ctrl["qy"], ctrl["qz"], ctrl["qw"], real_dt
        )
        if arm.ref_ee_xyz is None or arm.ref_ee_quat_wxyz is None:
            return

        # 姿态：rotation_mode 关闭时锁定为参考 ee 姿态
        rotation_active = self._rotation_enabled(ctrl)
        if not rotation_active:
            target_quat_wxyz = arm.ref_ee_quat_wxyz.copy()

        # 朝前保护：将目标姿态限定在初始 ee 朝向的 MAX_ROT_RANGE_RAD 以内
        if arm.home_quat_wxyz is not None:
            target_quat_wxyz = self._clamp_orientation_to_range(
                target_quat_wxyz, arm.home_quat_wxyz, MAX_ROT_RANGE_RAD
            )

        # 平移目标 = 参考位置 + 增量
        target_xyz = arm.ref_ee_xyz + delta_xyz

        # 位置速度限幅：使用统一测得的真实 dt（real_dt），而非固定 1/CMD_RATE_HZ。
        # 若循环因双臂+双手 IO 耗时导致实际周期长于额定值，仍按真实 dt 限速，避免
        # “限幅按额定周期计算、但实际更新间隔更长”造成的抖动/超调。
        max_pos_step = MAX_POS_SPEED * real_dt
        if arm._prev_target_xyz is not None:
            pos_delta = target_xyz - arm._prev_target_xyz
            pos_norm = np.linalg.norm(pos_delta)
            if pos_norm > max_pos_step:
                target_xyz = arm._prev_target_xyz + pos_delta * (max_pos_step / pos_norm)
        arm._prev_target_xyz = target_xyz.copy()
        arm._prev_target_quat_wxyz = target_quat_wxyz.copy()

        if np.allclose(delta_xyz, 0.0) and np.allclose(
            quat_diff_as_angle_axis(arm.ref_ee_quat_wxyz, target_quat_wxyz), 0.0
        ):
            return

        target_tcp_T = np.eye(4, dtype=float)
        target_tcp_T[:3, :3] = quat_wxyz_to_matrix(target_quat_wxyz)
        target_tcp_T[:3, 3] = target_xyz
        target_flange_T = target_tcp_T @ self.T_tcp_flange
        pose = self._robot_pose(arm)
        if pose is None:
            return
        _, _, current_q = pose
        arm.qp_ik.set_target_transform(target_flange_T)
        solved, q_sol = arm.qp_ik.solve()
        if not solved:
            warn = f"\r[Teleop-{hand}] ⚠️ Placo QP 未收敛，保持原地。"
            pad = " " * max(0, self._last_status_len - len(warn))
            sys.stdout.write(warn + pad)
            self._last_status_len = len(warn)
            sys.stdout.flush()
            return
        q_cmd = self._interpolate_joint_command(arm, current_q, q_sol, real_dt)
        arm.robot.move_j(q_cmd.tolist())
        rot_mode_tag = ("ABS" if rotation_active else "LOCK")
        dev_deg = np.degrees(np.linalg.norm(
            quat_diff_as_angle_axis(arm.home_quat_wxyz, target_quat_wxyz)
        )) if arm.home_quat_wxyz is not None else 0.0
        max_rot_deg = np.degrees(MAX_ROT_RANGE_RAD)
        status = (
            f"\r[Teleop-{hand}][{rot_mode_tag}] XYZ=({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f}) "
            f"q=({target_quat_wxyz[0]:.2f},{target_quat_wxyz[1]:.2f},{target_quat_wxyz[2]:.2f},{target_quat_wxyz[3]:.2f}) "
            f"偏{dev_deg:.1f}°/{max_rot_deg:.0f}°"
        )
        pad = " " * max(0, self._last_status_len - len(status))
        sys.stdout.write(status + pad)
        self._last_status_len = len(status)
        sys.stdout.flush()

    async def ws_loop(self):
        rot_hint = {
            "always": "平移+旋转",
            "hold-a": "平移(Grip) + 旋转(Grip+A)",
            "off": "仅平移",
        }.get(self.rotation_mode, "平移+旋转")
        mode_tag = (
            "双臂模式"
            if len(self.active_hands) == 2
            else f"单臂模式({self.active_hands[0]})"
        )
        connected = (
            f"[Network] ✅ 已连接，{mode_tag}：按住 Grip 开始控制（{rot_hint}），按 B 回到初始位置"
        )

        def on_payload(payload: dict) -> None:
            self._latest_vr_data = payload
            self._track_vr_rate()

        await run_webxr_ws_loop(
            WS_URI,
            on_payload,
            control_coro_factory=self.control_loop,
            connected_message=connected,
        )

    def _track_vr_rate(self):
        """统计 WebXR 数据实际到达频率（每 3 秒打印一次），用于排查抖动是否源于上行数据率不足。"""
        now = time.time()
        if self._vr_rate_window_start == 0.0:
            self._vr_rate_window_start = now
        self._vr_frame_count += 1
        elapsed = now - self._vr_rate_window_start
        if elapsed >= 3.0:
            hz = self._vr_frame_count / elapsed
            print(f"\n[频率监测] WebXR 上传实际帧率 ≈ {hz:.1f} Hz（预期 45 Hz）")
            self._vr_frame_count = 0
            self._vr_rate_window_start = now

    def _track_loop_rate(self):
        """统计控制循环实际迭代频率（每 3 秒打印一次），用于确认是否达到 CMD_RATE_HZ。"""
        now = time.time()
        if self._loop_rate_window_start == 0.0:
            self._loop_rate_window_start = now
        self._loop_iter_count += 1
        elapsed = now - self._loop_rate_window_start
        if elapsed >= 3.0:
            hz = self._loop_iter_count / elapsed
            print(f"\n[频率监测] 机械臂控制循环实际频率 ≈ {hz:.1f} Hz（额定 {CMD_RATE_HZ} Hz）")
            self._loop_iter_count = 0
            self._loop_rate_window_start = now

    async def control_loop(self):
        interval = 1.0 / CMD_RATE_HZ
        while True:
            iter_start = time.time()
            if self._latest_vr_data is not None:
                for hand in self.active_hands:
                    self.process_vr_data(self._latest_vr_data, hand)
            self._maybe_publish_state()
            self._track_loop_rate()
            elapsed = time.time() - iter_start
            await asyncio.sleep(max(0.0, interval - elapsed))

    def run(self):
        try:
            self.connect_robots()
            asyncio.run(self.ws_loop())
        except KeyboardInterrupt:
            print("\n[System] 收到退出信号")
        finally:
            if self.state_sender is not None:
                self.state_sender.close()
            for hand in self.active_hands:
                arm = self.arms[hand]
                if arm.robot is None:
                    continue
                if self.disable_on_exit:
                    arm.robot.disable()
                arm.robot.disconnect()
                if self.disable_on_exit:
                    print(f"[Robot-{hand}] 已断开（已失能）")
                else:
                    print(f"[Robot-{hand}] 已断开（未失能）")


def resolve_arm_can_ports(
    hands: tuple[str, ...],
    left_can_port: str | None,
    right_can_port: str | None,
) -> tuple[str | None, str | None]:
    """按控制模式解析左右臂 CAN 端口；双臂默认 left=can0、right=can1。"""
    _, default_port = resolve_can_backend()
    if len(hands) == 2:
        left = left_can_port or default_port
        if right_can_port:
            right = right_can_port
        elif default_port == "can0":
            right = "can1"
        else:
            right = default_port
        if left == right:
            raise ValueError(
                f"双臂模式左右臂不能使用同一 CAN 端口 ({left})，"
                "请指定 --left-can-port 与 --right-can-port"
            )
        return left, right
    if hands[0] == "left":
        return left_can_port or default_port, None
    return None, right_can_port or default_port


def parse_args():
    parser = argparse.ArgumentParser(description="WebXR -> 双 Piper 遥操作（Placo QP）")
    parser.add_argument(
        "--hands",
        choices=("both", "left", "right"),
        default="both",
        help="控制模式：both=双臂；left=仅左臂；right=仅右臂（单臂模式）",
    )
    parser.add_argument("--left-can-port", default=None, help="左臂 CAN 端口，双臂默认 can0")
    parser.add_argument("--right-can-port", default=None, help="右臂 CAN 端口，双臂默认 can1")
    parser.add_argument("--robot-model", default="piper_h", help="机械臂型号")
    parser.add_argument("--disable-on-exit", action="store_true", help="退出时执行 disable（默认仅断开不失能）")
    parser.add_argument(
        "--rotation-mode",
        choices=("always", "hold-a", "off"),
        default="always",
        help="旋转控制：always=Grip 时平移+旋转；hold-a=Grip+A 才旋转；off=仅平移",
    )
    parser.add_argument("--rotation-scale", type=float, default=ROTATION_SCALE, help="旋转增量缩放系数")
    parser.add_argument(
        "--tcp-offset",
        type=str,
        default="0,0,0,0,0,0",
        help="TCP 偏移(法兰坐标系) x,y,z,roll,pitch,yaw；单位 m/rad",
    )
    parser.add_argument("--publish-state", action="store_true", help="向 publisher 上报臂/手状态（UDP）")
    parser.add_argument("--state-udp-host", default="127.0.0.1", help="状态上报目标主机")
    parser.add_argument("--state-udp-port", type=int, default=17981, help="状态上报目标端口")
    parser.add_argument("--state-publish-hz", type=float, default=50.0, help="状态上报频率 Hz")
    return parser.parse_args()


def _parse_pose6(text: str):
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != 6:
        raise ValueError("--tcp-offset 必须是 6 个逗号分隔浮点数，例如: 0,0,0.10,0,0,0")
    return vals


if __name__ == "__main__":
    args = parse_args()
    hands = HANDS if args.hands == "both" else (args.hands,)
    left_can_port, right_can_port = resolve_arm_can_ports(
        hands, args.left_can_port, args.right_can_port
    )
    if len(hands) == 2:
        print(f"[System] 双臂 CAN 映射: left={left_can_port}, right={right_can_port}")
    WebXRPiperPlacoTeleop(
        left_can_port=left_can_port,
        right_can_port=right_can_port,
        robot_model=args.robot_model,
        disable_on_exit=args.disable_on_exit,
        rotation_mode=args.rotation_mode,
        rotation_scale=args.rotation_scale,
        tcp_offset_pose=_parse_pose6(args.tcp_offset),
        hands=hands,
        publish_state=args.publish_state,
        state_udp_host=args.state_udp_host,
        state_udp_port=args.state_udp_port,
        state_publish_hz=args.state_publish_hz,
    ).run()
