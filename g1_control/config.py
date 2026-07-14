"""G1 双臂遥操作默认配置。"""

from __future__ import annotations

import os

import numpy as np

# /**
#  * @brief WebXR (Y-up, -Z 前) -> G1 躯干系 (Z-up, +X 前, +Y 左)
#  *
#  *   手柄前 (-Z) -> 躯干 +X
#  *   手柄右 (+X) -> 躯干 -Y
#  *   手柄上 (+Y) -> 躯干 +Z
#  */
R_HEADSET_TO_WORLD = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_URDF_PATH = os.path.join(_PKG_DIR, "assets", "g1_dual_arm.urdf")

LEFT_EE_FRAME = "left_rubber_hand"
RIGHT_EE_FRAME = "right_rubber_hand"

ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

# 本机有线口（G1 网段 192.168.123.x）；可用 ip -br addr 确认
DEFAULT_NETWORK_INTERFACE: str | None = "enp12s0"
DEFAULT_WS_URI = "wss://localhost:8081"
DEFAULT_CONTROL_HZ = 100.0
DEFAULT_ARM_VELOCITY_LIMIT = 20.0
