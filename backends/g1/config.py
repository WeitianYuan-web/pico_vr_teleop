"""G1 双臂遥操作默认配置。"""

from __future__ import annotations

import os
import sys

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_PKG_DIR, "../.."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from common.constants import DEFAULT_WS_URI
from common.coord_frames import HEADSET_TO_WORLD_X_FORWARD

# /**
#  * @brief WebXR (Y-up, -Z 前) -> G1 躯干系 (Z-up, +X 前, +Y 左)
#  */
R_HEADSET_TO_WORLD = HEADSET_TO_WORLD_X_FORWARD

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
DEFAULT_CONTROL_HZ = 100.0
DEFAULT_ARM_VELOCITY_LIMIT = 20.0
