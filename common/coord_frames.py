"""头显/手柄坐标系到机器人世界系的预设旋转矩阵。"""

from __future__ import annotations

from typing import Literal

import numpy as np

# /**
#  * @brief WebXR (Y-up, -Z 前) -> 机器人系 (Z-up, +X 前)
#  *
#  * Piper / G1 / XRoboToolkit 共用：
#  *   手柄前 (-Z) -> +X
#  *   手柄右 (+X) -> -Y
#  *   手柄上 (+Y) -> +Z
#  */
HEADSET_TO_WORLD_X_FORWARD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)

# /**
#  * @brief WebXR (Y-up, -Z 前) -> JAKA 基座系 (Z-up, +Y 前)
#  *
#  * 等价于 X_FORWARD 后再绕 Z +90°：
#  *   手柄前 (-Z) -> +Y
#  *   手柄右 (+X) -> +X
#  *   手柄上 (+Y) -> +Z
#  */
HEADSET_TO_WORLD_Y_FORWARD = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def get_headset_to_world(preset: Literal["x_forward", "y_forward"]) -> np.ndarray:
    """返回命名预设矩阵的副本。"""
    if preset == "x_forward":
        return HEADSET_TO_WORLD_X_FORWARD.copy()
    if preset == "y_forward":
        return HEADSET_TO_WORLD_Y_FORWARD.copy()
    raise ValueError(f"未知坐标系预设: {preset}")
