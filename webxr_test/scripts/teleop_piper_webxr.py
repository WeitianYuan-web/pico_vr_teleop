#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import os
import ssl
import sys
import time
from types import SimpleNamespace

import numpy as np
import websockets

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../pyAgxArm")))

try:
    from pyAgxArm import AgxArmFactory, create_agx_arm_config
    from piper_placo_qp_ik import PiperPlacoConfig, PiperPlacoQPIK, pose6_to_transform, transform_to_pose6
    from run_piper_control import detect_firmware_version, resolve_can_backend, wait_motion_done
except ImportError as exc:
    print(f"导入依赖失败: {exc}")
    sys.exit(1)


# 与 XRoboToolkit geometry.R_HEADSET_TO_WORLD 一致：头显/手柄坐标 -> 机器人世界坐标
R_HEADSET_TO_WORLD = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=float,
)


WS_URI = "wss://localhost:8081"
HANDS = ("left", "right")
SCALE_FACTOR = 1.0
ROTATION_SCALE = 1.0
CMD_RATE_HZ = 100                    # 控制频率 Hz
MAX_JOINT_STEP_RAD = 0.03            # 每步最大关节增量 rad（100 Hz × 0.03 ≈ 3 rad/s 上限）
MAX_POS_SPEED = 0.8                  # 笛卡尔位置速度上限 m/s
JOINT_INTERP_ALPHA = 0.75            # 关节命令插值系数（越大越跟手）
ELBOW_WEIGHT = 0.02                  # 肘部偏好任务权重（0 关闭）
INIT_JOINTS = [0.0, 1.8, -0.9, 0.0, 0.5, 0.0]  # 启动先 move_j 到抬起姿态（绝对关节角）
INIT_JOINT_TIMEOUT = 6.0
INIT_JOINT_SETTLE = 0.5
INIT_ABS_X = None  # 例如 0.25；None 表示保持当前
INIT_ABS_Y = None  # 例如 0.00
INIT_ABS_Z = None  # 例如 0.20
INIT_ABS_WAIT = 1.0
BTN_A_INDEX = 4  # WebXR xr-standard：右手柄 A 键（按住时启用旋转）
BTN_B_INDEX = 5  # WebXR xr-standard：右手柄 B 键
HOME_COOLDOWN_S = 2.0
# ee 帧 = link6 法兰帧绕 Z 轴转 90°（无平移偏置）
TCP_OFFSET_POSE = [0.0, 0.0, 0.0, 0.0, 0.0, np.pi / 2]
# 姿态绝对控制：目标朝向偏离初始 ee 朝向的最大角度（朝前保护）
MAX_ROT_RANGE_RAD = np.radians(60)


class EMAFilter:
    def __init__(self, alpha: float = 0.2):
        self.alpha = alpha
        self.value = None

    def update(self, x: np.ndarray) -> np.ndarray:
        if self.value is None:
            self.value = x.copy()
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return self.value


def matrix_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
    """旋转矩阵转四元数 (w, x, y, z)。"""
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
    quat = np.array([w, x, y, z], dtype=float)
    if quat[0] < 0.0:
        quat = -quat
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return quat / norm


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


def quat_inverse_wxyz(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
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
    axis = q[1:] / sin_half
    return axis * angle


def quat_diff_as_angle_axis(q1: np.ndarray, q2: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    delta_q = quat_multiply_wxyz(q2, quat_inverse_wxyz(q1))
    return quaternion_to_angle_axis(delta_q, eps)


def apply_delta_pose(
    source_pos: np.ndarray,
    source_rot_wxyz: np.ndarray,
    delta_pos: np.ndarray,
    delta_rot: np.ndarray,
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """与 XRoboToolkit apply_delta_pose 一致：delta 左乘到 source 姿态。"""
    target_pos = source_pos + delta_pos
    angle = np.linalg.norm(delta_rot)
    if angle > eps:
        axis = delta_rot / angle
        half = angle / 2.0
        rot_delta = np.array([np.cos(half), *(axis * np.sin(half))], dtype=float)
    else:
        rot_delta = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    target_rot = quat_multiply_wxyz(rot_delta, source_rot_wxyz)
    return target_pos, target_rot


def quat_wxyz_to_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return quat_xyzw_to_matrix(x, y, z, w)


def quat_xyzw_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=float)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


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
            urdf_path = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "../../pyAgxArm/agx_arm_urdf-main/piper_h/urdf/piper_h_description.urdf",
                )
            )
        self.urdf_path = urdf_path

        self.last_schema_warn_time = 0.0
        self._last_status_len = 0
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
                pos_filter=EMAFilter(alpha=0.2),
                rot_filter=EMAFilter(alpha=0.15),   # 绝对姿态滤波（比增量模式更平滑）
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
            )
        self.arms["left"].can_port = self.left_can_port
        self.arms["right"].can_port = self.right_can_port
        self._latest_vr_data: dict | None = None

        self.R_headset_world = R_HEADSET_TO_WORLD.copy()
        self.T_flange_tcp = pose6_to_transform(self.tcp_offset_pose)
        self.T_tcp_flange = np.linalg.inv(self.T_flange_tcp)

    def _transform_xr_controller(self, vr_pos: np.ndarray, qx: float, qy: float, qz: float, qw: float):
        """将 WebXR 手柄位姿变换到机器人世界坐标系（对齐 XRoboToolkit _process_xr_pose）。"""
        controller_xyz = self.R_headset_world @ vr_pos
        controller_quat_wxyz = np.array([qw, qx, qy, qz], dtype=float)

        r_quat_wxyz = matrix_to_quat_wxyz(self.R_headset_world)
        controller_quat_wxyz = quat_multiply_wxyz(
            quat_multiply_wxyz(r_quat_wxyz, controller_quat_wxyz),
            quat_inverse_wxyz(r_quat_wxyz),
        )
        return controller_xyz, controller_quat_wxyz

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
        delta_xyz = arm.pos_filter.update(controller_xyz - arm.ref_controller_xyz) * SCALE_FACTOR

        # 姿态：绝对控制（局部相对四元数法）
        # q_rel 表示“从 grip 接合参考姿态到当前姿态”的相对旋转（手柄局部轴语义）
        q_rel = quat_multiply_wxyz(quat_inverse_wxyz(arm.ref_controller_quat_wxyz), controller_quat_wxyz)
        norm_rel = np.linalg.norm(q_rel)
        if norm_rel > 1e-12:
            q_rel = q_rel / norm_rel
        else:
            q_rel = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        # rotation_scale 仍可用于姿态灵敏度调节（1.0 = 真实等比例跟随）
        rel_aa = quaternion_to_angle_axis(q_rel)
        rel_aa = rel_aa * self.rotation_scale
        _, raw_q = apply_delta_pose(
            np.zeros(3, dtype=float),
            arm.ref_ee_quat_wxyz,
            np.zeros(3, dtype=float),
            rel_aa,
        )
        # 保持四元数半球一致，再做 EMA 平滑
        prev_q = arm.rot_filter.value
        if prev_q is not None and np.dot(prev_q, raw_q) < 0:
            raw_q = -raw_q
        smoothed = arm.rot_filter.update(raw_q)
        norm = np.linalg.norm(smoothed)
        target_quat = smoothed / norm if norm > 1e-12 else arm.ref_ee_quat_wxyz.copy()

        return delta_xyz, target_quat

    def _rotation_enabled(self, ctrl: dict) -> bool:
        if self.rotation_mode == "off":
            return False
        if self.rotation_mode == "hold-a":
            return self._is_button_pressed(ctrl, BTN_A_INDEX)
        return True

    def _is_button_pressed(self, ctrl: dict, index: int) -> bool:
        buttons = ctrl.get("buttons")
        if not buttons or len(buttons) <= index:
            return False
        return bool(buttons[index].get("pressed", False))

    def _is_b_button_pressed(self, ctrl: dict) -> bool:
        return self._is_button_pressed(ctrl, BTN_B_INDEX)

    def _move_to_init_joints(self, arm: SimpleNamespace, label: str = "初始化") -> bool:
        if arm.robot is None:
            return False
        print(f"\n[Robot-{arm.hand}] {label}：阶段1 move_j -> {INIT_JOINTS}")
        arm.robot.move_j(INIT_JOINTS)
        reached = wait_motion_done(arm.robot, timeout=INIT_JOINT_TIMEOUT)
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
        arm.robot.set_speed_percent(60)
        arm.robot.set_installation_pos(arm.robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
        arm.robot.set_motion_mode(arm.robot.OPTIONS.MOTION_MODE.J)
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

    def _interpolate_joint_command(self, arm: SimpleNamespace, current_q: np.ndarray, target_q: np.ndarray) -> np.ndarray:
        if arm._joint_interp_q is None:
            arm._joint_interp_q = current_q.copy()
        arm._joint_interp_q = arm._joint_interp_q + JOINT_INTERP_ALPHA * (target_q - arm._joint_interp_q)
        dq = np.clip(arm._joint_interp_q - current_q, -MAX_JOINT_STEP_RAD, MAX_JOINT_STEP_RAD)
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
            arm, vr_pos, ctrl["qx"], ctrl["qy"], ctrl["qz"], ctrl["qw"]
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

        # 位置速度限幅（绝对姿态模式下姿态不做额外限幅，EMA 已提供平滑）
        dt_ctrl = 1.0 / CMD_RATE_HZ
        max_pos_step = MAX_POS_SPEED * dt_ctrl
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
        q_cmd = self._interpolate_joint_command(arm, current_q, q_sol)
        arm.robot.move_j(q_cmd.tolist())
        rot_mode_tag = ("ABS" if rotation_active else "LOCK")
        dev_deg = np.degrees(np.linalg.norm(
            quat_diff_as_angle_axis(arm.home_quat_wxyz, target_quat_wxyz)
        )) if arm.home_quat_wxyz is not None else 0.0
        status = (
            f"\r[Teleop-{hand}][{rot_mode_tag}] XYZ=({target_xyz[0]:.3f},{target_xyz[1]:.3f},{target_xyz[2]:.3f}) "
            f"q=({target_quat_wxyz[0]:.2f},{target_quat_wxyz[1]:.2f},{target_quat_wxyz[2]:.2f},{target_quat_wxyz[3]:.2f}) "
            f"偏{dev_deg:.1f}°/60°"
        )
        pad = " " * max(0, self._last_status_len - len(status))
        sys.stdout.write(status + pad)
        self._last_status_len = len(status)
        sys.stdout.flush()

    async def ws_loop(self):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

        print(f"\n[Network] 连接 WebXR 数据: {WS_URI}")
        while True:
            try:
                async with websockets.connect(WS_URI, ssl=ssl_ctx) as ws:
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
                    print(f"[Network] ✅ 已连接，{mode_tag}：按住 Grip 开始控制（{rot_hint}），按 B 回到初始位置")
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
                print(f"\n[Network] 连接中断: {exc}，2 秒后重连")
                await asyncio.sleep(2.0)

    async def control_loop(self):
        interval = 1.0 / CMD_RATE_HZ
        while True:
            if self._latest_vr_data is not None:
                for hand in self.active_hands:
                    self.process_vr_data(self._latest_vr_data, hand)
            await asyncio.sleep(interval)

    def run(self):
        try:
            self.connect_robots()
            asyncio.run(self.ws_loop())
        except KeyboardInterrupt:
            print("\n[System] 收到退出信号")
        finally:
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


def parse_args():
    parser = argparse.ArgumentParser(description="WebXR -> 双 Piper 遥操作（Placo QP）")
    parser.add_argument(
        "--hands",
        choices=("both", "left", "right"),
        default="both",
        help="控制模式：both=双臂；left=仅左臂；right=仅右臂（单臂模式）",
    )
    parser.add_argument("--left-can-port", default=None, help="左臂 CAN 端口，默认自动检测")
    parser.add_argument("--right-can-port", default=None, help="右臂 CAN 端口，默认自动检测")
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
    return parser.parse_args()


def _parse_pose6(text: str):
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != 6:
        raise ValueError("--tcp-offset 必须是 6 个逗号分隔浮点数，例如: 0,0,0.10,0,0,0")
    return vals


if __name__ == "__main__":
    args = parse_args()
    hands = HANDS if args.hands == "both" else (args.hands,)
    WebXRPiperPlacoTeleop(
        left_can_port=args.left_can_port,
        right_can_port=args.right_can_port,
        robot_model=args.robot_model,
        disable_on_exit=args.disable_on_exit,
        rotation_mode=args.rotation_mode,
        rotation_scale=args.rotation_scale,
        tcp_offset_pose=_parse_pose6(args.tcp_offset),
        hands=hands,
    ).run()
