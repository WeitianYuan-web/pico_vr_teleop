"""Grip 离合增量位姿内核（不含各机型 home / hold_q）。"""

from __future__ import annotations

import numpy as np

from common.math_quat import quat_inverse_wxyz, quat_multiply_wxyz, quaternion_to_angle_axis
from common.math_se3 import apply_delta_rotation


def controller_relative_delta(
    ref_controller_xyz: np.ndarray,
    current_controller_xyz: np.ndarray,
    position_scale: float = 1.0,
) -> np.ndarray:
    """手柄平移增量（米制；调用方负责 mm 换算）。"""
    return (current_controller_xyz - ref_controller_xyz) * float(position_scale)


def target_rotation_from_controller_rel(
    ref_controller_quat: np.ndarray,
    current_controller_quat: np.ndarray,
    ref_ee_quat: np.ndarray,
    rotation_scale: float = 1.0,
) -> np.ndarray:
    """
    /**
     * @brief 由手柄相对旋转得到目标 ee 姿态
     *
     * q_rel = q_ref_ctrl^-1 * q_cur_ctrl，再按 rotation_scale 缩放到 ee。
     */
    """
    q_rel = quat_multiply_wxyz(quat_inverse_wxyz(ref_controller_quat), current_controller_quat)
    n_rel = np.linalg.norm(q_rel)
    q_rel = q_rel / n_rel if n_rel > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    rel_aa = quaternion_to_angle_axis(q_rel) * float(rotation_scale)
    return apply_delta_rotation(ref_ee_quat, rel_aa)
