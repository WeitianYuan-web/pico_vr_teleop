#!/usr/bin/env python3
from __future__ import annotations

import math
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple

import numpy as np

try:
    import placo
except ImportError as exc:
    raise ImportError("需要安装 placo 才能使用 QP 遥操作。") from exc


@dataclass(frozen=True)
class PiperPlacoConfig:
    urdf_path: Path
    ee_frame: str = "link6"
    elbow_link: str = "link3"
    dt: float = 0.02
    position_weight: float = 1.0
    orientation_weight: float = 0.1
    elbow_weight: float = 0.0
    manipulability_weight: float = 1e-2
    joints_regularization_weight: float = 1e-4


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(r: np.ndarray) -> np.ndarray:
    pitch = math.asin(-max(-1.0, min(1.0, r[2, 0])))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-r[0, 1], r[1, 1])
    else:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    return np.array([roll, pitch, yaw], dtype=float)


def pose6_to_transform(pose6: Sequence[float]) -> np.ndarray:
    t = np.array(pose6[:3], dtype=float)
    r = rpy_to_matrix(*pose6[3:6])
    T = np.eye(4, dtype=float)
    T[:3, :3] = r
    T[:3, 3] = t
    return T


def transform_to_pose6(T: np.ndarray) -> np.ndarray:
    xyz = T[:3, 3].astype(float)
    rpy = matrix_to_rpy(T[:3, :3])
    return np.concatenate([xyz, rpy], axis=0)


class PiperPlacoQPIK:
    def __init__(self, config: PiperPlacoConfig):
        self.config = config
        self._resolved_urdf_path = self._prepare_urdf_for_placo(config.urdf_path)
        self.robot = placo.RobotWrapper(str(self._resolved_urdf_path))
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = config.dt
        self.solver.mask_fbase(True)

        self.robot.update_kinematics()
        ee_T = self.robot.get_T_world_frame(config.ee_frame)

        # 位置任务（高权重）：位置误差由专用位置任务主导
        self.position_task = self.solver.add_position_task(config.ee_frame, ee_T[:3, 3])
        self.position_task.configure("ee_position", "soft", config.position_weight)

        # 全姿态任务（低权重）：主要提供姿态引导，位置可被上方任务覆盖
        self.frame_task = self.solver.add_frame_task(config.ee_frame, ee_T)
        self.frame_task.configure("ee_pose", "soft", config.orientation_weight)

        self.elbow_task = None
        if config.elbow_weight > 0.0:
            elbow_T = self.robot.get_T_world_frame(config.elbow_link)
            self.elbow_task = self.solver.add_position_task(config.elbow_link, elbow_T[:3, 3])
            self.elbow_task.configure("elbow_preference", "soft", config.elbow_weight)

        self.manip_task = self.solver.add_manipulability_task(config.ee_frame, "both", 1.0)
        self.manip_task.configure("manipulability", "soft", config.manipulability_weight)

        self.joints_task = self.solver.add_joints_task()
        self.joints_task.configure("joints_regularization", "soft", config.joints_regularization_weight)
        self._joint_names = list(self.robot.joint_names())
        self.joints_task.set_joints({name: 0.0 for name in self._joint_names})

        # 从 URDF 模型提取关节限位（用于硬约束剪裁）
        n = len(self._joint_names)
        self.joint_lower = np.array(self.robot.model.lowerPositionLimit[7:7 + n], dtype=float)
        self.joint_upper = np.array(self.robot.model.upperPositionLimit[7:7 + n], dtype=float)

    def sync_state_from_joint_positions(self, joints: Sequence[float]) -> None:
        joints_arr = np.asarray(joints, dtype=float)
        self.robot.state.q[7 : 7 + len(joints_arr)] = joints_arr
        self.robot.update_kinematics()
        self.joints_task.set_joints({name: float(q) for name, q in zip(self._joint_names, joints_arr)})

    def set_target_pose6(self, target_pose6: Sequence[float]) -> None:
        T = pose6_to_transform(target_pose6)
        self.frame_task.T_world_frame = T
        self.position_task.target_world = T[:3, 3]

    def set_target_transform(self, T: np.ndarray) -> None:
        """直接以 4×4 矩阵设置目标帧，无 RPY 转换（与 XRoboToolkit T_world_frame 对齐）。"""
        self.frame_task.T_world_frame = T
        self.position_task.target_world = T[:3, 3]

    def set_elbow_target_world(self, xyz: Sequence[float]) -> None:
        if self.elbow_task is not None:
            self.elbow_task.target_world = np.asarray(xyz, dtype=float)

    def current_flange_transform(self) -> np.ndarray:
        """返回末端帧当前的 4×4 世界变换矩阵（无 RPY 转换）。"""
        return np.array(self.robot.get_T_world_frame(self.config.ee_frame), dtype=float)

    def current_pose6(self) -> np.ndarray:
        return transform_to_pose6(self.robot.get_T_world_frame(self.config.ee_frame))

    def solve(self) -> Tuple[bool, np.ndarray]:
        try:
            self.solver.solve(True)
        except RuntimeError:
            return False, np.zeros(len(self._joint_names), dtype=float)

        q = np.array(self.robot.state.q[7 : 7 + len(self._joint_names)], dtype=float)
        # 硬约束：按 URDF 上下限剪裁（Placo 已通过速度限制处理，此处再做一层保险）
        q = np.clip(q, self.joint_lower, self.joint_upper)
        return True, q

    @staticmethod
    def _prepare_urdf_for_placo(urdf_path: Path) -> Path:
        """
        /**
         * @brief 将 URDF 中的 package:// 资源路径重写为本地绝对路径，兼容 Placo 加载。
         *
         * 典型替换：
         * package://agx_arm_description/agx_arm_urdf/piper_h/meshes/...
         * -> /abs/path/to/agx_arm_urdf-main/piper_h/meshes/...
         */
        """
        urdf_path = Path(urdf_path).resolve()
        text = urdf_path.read_text(encoding="utf-8")

        package_prefix = "package://agx_arm_description/agx_arm_urdf/"
        if package_prefix not in text:
            return urdf_path

        # .../agx_arm_urdf-main/piper_h/urdf/piper_h_description.urdf
        # -> asset_root = .../agx_arm_urdf-main
        asset_root = urdf_path.parents[2]
        replacement = asset_root.as_posix().rstrip("/") + "/"
        rewritten = text.replace(package_prefix, replacement)

        tmp = tempfile.NamedTemporaryFile(prefix="piper_placo_", suffix=".urdf", delete=False)
        tmp_path = Path(tmp.name)
        tmp.close()
        tmp_path.write_text(rewritten, encoding="utf-8")
        return tmp_path
