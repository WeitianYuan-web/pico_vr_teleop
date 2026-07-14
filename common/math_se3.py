"""位姿增量与 WebXR 手柄坐标变换。"""

from __future__ import annotations

import numpy as np

from common.math_quat import (
    matrix_to_quat_wxyz,
    quat_inverse_wxyz,
    quat_multiply_wxyz,
)


def transform_xr_controller(
    r_headset_to_world: np.ndarray,
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[np.ndarray, np.ndarray]:
    """
    /**
     * @brief 将 WebXR 手柄位姿变换到机器人世界系
     * @return (controller_xyz, controller_quat_wxyz)
     */
    """
    vr_pos = np.array([x, y, z], dtype=float)
    controller_xyz = r_headset_to_world @ vr_pos
    controller_quat_wxyz = np.array([qw, qx, qy, qz], dtype=float)
    r_quat_wxyz = matrix_to_quat_wxyz(r_headset_to_world)
    controller_quat_wxyz = quat_multiply_wxyz(
        quat_multiply_wxyz(r_quat_wxyz, controller_quat_wxyz),
        quat_inverse_wxyz(r_quat_wxyz),
    )
    return controller_xyz, controller_quat_wxyz


def apply_delta_rotation(
    source_rot_wxyz: np.ndarray,
    delta_rot_aa: np.ndarray,
    *,
    eps: float = 1e-6,
) -> np.ndarray:
    """
    /**
     * @brief 将角轴增量左乘到源姿态（与 XRoboToolkit / JAKA / G1 一致）
     */
    """
    angle = float(np.linalg.norm(delta_rot_aa))
    if angle > eps:
        axis = delta_rot_aa / angle
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


def apply_delta_pose(
    source_pos: np.ndarray,
    source_rot_wxyz: np.ndarray,
    delta_pos: np.ndarray,
    delta_rot_aa: np.ndarray,
    *,
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """
    /**
     * @brief 平移 + 姿态增量（Piper / XRoboToolkit 完整版）
     */
    """
    target_pos = np.asarray(source_pos, dtype=float) + np.asarray(delta_pos, dtype=float)
    target_rot = apply_delta_rotation(source_rot_wxyz, delta_rot_aa, eps=eps)
    return target_pos, target_rot
