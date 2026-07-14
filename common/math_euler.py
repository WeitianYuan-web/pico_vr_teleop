"""欧拉角 XYZ ↔ 四元数（JAKA TCP rx/ry/rz 约定）。"""

from __future__ import annotations

import math

import numpy as np


def euler_xyz_to_quat_wxyz(rx: float, ry: float, rz: float) -> np.ndarray:
    """
    /**
     * @brief JAKA TCP 姿态 (rx,ry,rz) 欧拉 XYZ 转四元数 (w,x,y,z)
     */
    """
    cx = math.cos(rx * 0.5)
    sx = math.sin(rx * 0.5)
    cy = math.cos(ry * 0.5)
    sy = math.sin(ry * 0.5)
    cz = math.cos(rz * 0.5)
    sz = math.sin(rz * 0.5)

    qw = cx * cy * cz + sx * sy * sz
    qx = sx * cy * cz - cx * sy * sz
    qy = cx * sy * cz + sx * cy * sz
    qz = cx * cy * sz - sx * sy * cz
    q = np.array([qw, qx, qy, qz], dtype=float)
    if q[0] < 0.0:
        q = -q
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_wxyz_to_euler_xyz(q: np.ndarray) -> tuple[float, float, float]:
    """
    /**
     * @brief 四元数 (w,x,y,z) 转欧拉 XYZ（与 JAKA TCP rx/ry/rz 一致）
     */
    """
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    rx = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        ry = math.copysign(math.pi / 2.0, sinp)
    else:
        ry = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    rz = math.atan2(siny_cosp, cosy_cosp)
    return rx, ry, rz


def euler_xyz_to_quat_xyzw(rx: float, ry: float, rz: float) -> tuple[float, float, float, float]:
    """JAKA TCP 姿态 (rx,ry,rz) 转四元数 (x,y,z,w)。"""
    q = euler_xyz_to_quat_wxyz(rx, ry, rz)
    return float(q[1]), float(q[2]), float(q[3]), float(q[0])
