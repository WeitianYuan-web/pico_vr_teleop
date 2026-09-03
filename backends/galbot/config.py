"""Galbot G1 VR teleop defaults.

Do not confuse with backends/g1 (Unitree G1). This backend talks to Galaxy
Robotics Galbot G1 via Galbot SDK: WBC EE stream on SDK 1.8+, or
Motion IK + streaming set_joint_commands on SDK 1.7 / GBS 1.15.
"""

from __future__ import annotations

import os

import numpy as np

from common.constants import DEFAULT_WS_URI
from common.coord_frames import (
    HEADSET_TO_WORLD_X_FORWARD,
    HEADSET_TO_WORLD_Y_FORWARD,
    get_headset_to_world,
)

# Galbot world: +X forward, +Y left, +Z up (same as Piper / Unitree G1 / tianyee).
# Axis signs are identity until a real robot confirms a side is mirrored.
DEFAULT_COORD_PRESET = "x_forward"
R_HEADSET_TO_WORLD = HEADSET_TO_WORLD_X_FORWARD
DEFAULT_AXIS_SIGN = (1.0, 1.0, 1.0)
DEFAULT_AXIS_SIGN_LEFT = (1.0, 1.0, 1.0)
DEFAULT_AXIS_SIGN_RIGHT = (1.0, 1.0, 1.0)

COORD_PRESETS = {
    "y_forward": HEADSET_TO_WORLD_Y_FORWARD,
    "x_forward": HEADSET_TO_WORLD_X_FORWARD,
}

DEFAULT_WS_URI_GALBOT = DEFAULT_WS_URI
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_HOME_INTERP_S = 4.0
DEFAULT_HOME_COOLDOWN_S = 2.0
# 回初始位用阻塞关节规划，可比跟手快。0.75 rad/s ≈ 43°/s。
DEFAULT_HOME_SPEED_RAD_S = 0.75
DEFAULT_HOME_TIMEOUT_S = 20.0
# 跟手关节流硬限速。原先 1.5 接近 URDF 上限，手柄一抖臂就甩。
DEFAULT_TELEOP_MAX_RAD_S = 0.45
# 本机实测、看起来正常的双臂姿态。左右 j2 限位相同（±1.608）；
# 这组预备位两肩都抬到外展附近，不是右边限位更窄。
DEFAULT_HOME_Q_LEFT = (1.258, -1.493, -0.542, -1.759, -0.222, -0.230, -0.100)
DEFAULT_HOME_Q_RIGHT = (-1.478, 1.521, 0.552, 2.081, 0.164, -0.404, -0.102)
DEFAULT_HOME_JOINT_TOL_RAD = 0.08
# 笛卡尔预备位（base_link，m）。与上面关节的 pinocchio FK 对齐。
DEFAULT_HOME_XYZ_LEFT = (0.496, 0.255, 0.368)
DEFAULT_HOME_XYZ_RIGHT = (0.422, -0.210, 0.377)
DEFAULT_HOME_POS_TOL_M = 0.04
# Per tick at 50 Hz ≈ 0.3 m/s. Local IK is incremental; don't stair-step Cartesian.
DEFAULT_MAX_STEP_M = 0.006

DEFAULT_GRIP_ENGAGE = 0.55
DEFAULT_GRIP_RELEASE = 0.35
DEFAULT_POS_DEADZONE_M = 0.005
DEFAULT_ROT_DEADZONE_DEG = 2.5
DEFAULT_POS_FILTER_ALPHA = 0.28
DEFAULT_ROT_FILTER_ALPHA = 0.20
DEFAULT_ROTATION_MODE = "always"
DEFAULT_POSITION_SCALE = 1.0
DEFAULT_ROTATION_SCALE = 1.0

# Official Galbot SDK default Embosa addresses (PC / XCU / HPU).
DEFAULT_LOCAL_IP = "192.168.1.99"
DEFAULT_XCU_IP = "192.168.1.66"
DEFAULT_HPU_IP = "192.168.1.88"
DEFAULT_EMBOSA_CONFIG = "/data/config/embosa_ip_config.json"
DEFAULT_SYSTEM_CFG = "/data/config/system.cfg"

LEFT_EE_FRAME = "left_arm_end_effector_mount_link"
RIGHT_EE_FRAME = "right_arm_end_effector_mount_link"
EE_FRAMES = {"left": LEFT_EE_FRAME, "right": RIGHT_EE_FRAME}
WBC_POSE_KEYS = {"left": "lee_pose", "right": "ree_pose"}
ARM_JOINT_GROUPS = {"left": "left_arm", "right": "right_arm"}
ARM_CHAINS = {"left": "left_arm", "right": "right_arm"}
LEFT_ARM_JOINT_NAMES = tuple(f"left_arm_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINT_NAMES = tuple(f"right_arm_joint{i}" for i in range(1, 8))
ARM_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
SUPPORT_JOINT_NAMES = tuple(f"leg_joint{i}" for i in range(1, 6)) + tuple(
    f"head_joint{i}" for i in range(1, 3)
)

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "../.."))
DEFAULT_URDF_PATH = os.path.join(_HERE, "assets", "galbot_one_golf_cali.urdf")
DEFAULT_SDK_TREE_17 = os.path.join(_ROOT, "third_party", "GalbotSDK-V1.7.3")
DEFAULT_SDK_TREE = os.path.join(_ROOT, "third_party", "GalbotSDK-main")
DEFAULT_GALBOT_HOME = os.environ.get("GALBOT_HOME", "")
# 本机机器人是 GBS 1.15.15 → SDK 1.7.3（独立目录，不覆盖 /opt/galbot 的 1.9.1）
# GBS 1.17 → SDK 1.9 /opt/galbot
DEFAULT_GALBOT_HOME_17 = "/opt/galbot-1.7.3"
DEFAULT_GALBOT_HOME_19 = "/opt/galbot"
DEFAULT_PLATFORM = "linux-x86_64-gcc940"


def resolve_headset_to_world(preset: str) -> np.ndarray:
    key = str(preset).strip().lower()
    if key in COORD_PRESETS:
        return COORD_PRESETS[key].copy()
    return get_headset_to_world(key)  # type: ignore[arg-type]


def resolve_axis_sign(values) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float).reshape(3)
    arr = np.where(np.abs(arr) < 1e-12, 1.0, np.sign(arr))
    return arr
