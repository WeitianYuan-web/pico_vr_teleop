"""Noetix M1 VR teleop defaults."""

from __future__ import annotations

import os

import numpy as np

from common.constants import DEFAULT_WS_URI
from common.coord_frames import (
    HEADSET_TO_WORLD_X_FORWARD,
    HEADSET_TO_WORLD_Y_FORWARD,
    get_headset_to_world,
)

# 实机反馈：右臂在 x_forward（与 G1/tianyee 相同）下平移轴正确。
# 左臂：上下正常，前后与左右均反 → 默认对左臂取 X/Y 反号。
DEFAULT_COORD_PRESET = "x_forward"
R_HEADSET_TO_WORLD = HEADSET_TO_WORLD_X_FORWARD
DEFAULT_AXIS_SIGN = (1.0, 1.0, 1.0)
DEFAULT_AXIS_SIGN_LEFT = (-1.0, -1.0, 1.0)
DEFAULT_AXIS_SIGN_RIGHT = (1.0, 1.0, 1.0)

COORD_PRESETS = {
    "y_forward": HEADSET_TO_WORLD_Y_FORWARD,
    "x_forward": HEADSET_TO_WORLD_X_FORWARD,
}


def resolve_headset_to_world(preset: str) -> np.ndarray:
    key = str(preset).strip().lower()
    if key in COORD_PRESETS:
        return COORD_PRESETS[key].copy()
    return get_headset_to_world(key)  # type: ignore[arg-type]


def resolve_axis_sign(values) -> np.ndarray:
    arr = np.asarray(list(values), dtype=float).reshape(3)
    arr = np.where(np.abs(arr) < 1e-12, 1.0, np.sign(arr))
    return arr

DEFAULT_WS_URI_NOETIX = DEFAULT_WS_URI
DEFAULT_CONTROL_HZ = 100.0  # match robot control period (~10 ms)

# CycloneDDS peer network used by cartesian_min_ws
DEFAULT_LOCAL_IP = "192.168.127.40"
DEFAULT_PEER_IP = "192.168.127.20"

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "../.."))
DEFAULT_CARTESIAN_WS = os.path.join(_ROOT, "third_party", "cartesian_min_ws")
DEFAULT_CYCLONEDDS_XML = os.path.join(
    DEFAULT_CARTESIAN_WS,
    "src",
    "noetix_python_controller",
    "config",
    "cyclonedds.xml",
)
DEFAULT_HARDWARE_CONFIG = os.path.join(
    DEFAULT_CARTESIAN_WS,
    "src",
    "noetix_python_controller",
    "config",
    "hardware_config.yaml",
)

# Default "抱箱子" joint poses (rad), joints 0..6 (no gripper)
DEFAULT_RIGHT_BOX = (-0.55, 0.45, 0.20, -1.05, 0.0, 0.15, 0.0)
DEFAULT_LEFT_BOX = (0.55, -0.45, -0.20, 1.05, 0.0, -0.15, 0.0)
DEFAULT_BOX_INTERP_S = 4.0
DEFAULT_BOX_HOLD_S = 1.0
DEFAULT_CART_HOME_INTERP_S = 4.0  # B-home stays in mode2, interpolate EE to saved home
DEFAULT_MODE_SETTLE_S = 0.8  # hold frozen EE after mode2 ack to avoid switch jerk
DEFAULT_MAX_JOINT_STEP_RAD = 0.003
DEFAULT_MAX_STEP_M = 0.002  # per control tick at 100Hz ≈ 0.2 m/s
DEFAULT_HOME_COOLDOWN_S = 2.0

DEFAULT_GRIP_ENGAGE = 0.55
DEFAULT_GRIP_RELEASE = 0.35
DEFAULT_POS_DEADZONE_M = 0.004
DEFAULT_ROT_DEADZONE_DEG = 2.0
DEFAULT_POS_FILTER_ALPHA = 0.45
DEFAULT_ROT_FILTER_ALPHA = 0.40
DEFAULT_ROTATION_MODE = "always"
DEFAULT_POSITION_SCALE = 1.0
DEFAULT_ROTATION_SCALE = 1.0

ROBOT_STATUS_TOPIC = "/Robot_Status_Topic"
MOTOR_CMD_TOPIC = "/Motor_Cmd_Topic"
CARTESIAN_CMD_TOPIC = "/Cartesian_Cmd_Topic"
CONTROL_MODE_TOPIC = "/Control_Mode_Topic"

ARM_JOINTS = 7
ARM_DOF = 8
LEG_DOF = 6
LEG_BASE_ID = 16
RIGHT_GRIPPER_ID = 7
LEFT_GRIPPER_ID = 15
