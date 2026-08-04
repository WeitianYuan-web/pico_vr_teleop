"""Tianyi / XARM VR teleop defaults."""

from __future__ import annotations

from common.constants import DEFAULT_WS_URI
from common.coord_frames import HEADSET_TO_WORLD_X_FORWARD

# 天轶文档：整机 X 前、Y 左、Z 上 → 与 Piper/G1 相同 x_forward 头显映射
R_HEADSET_TO_WORLD = HEADSET_TO_WORLD_X_FORWARD

DEFAULT_WS_URI_TIANYEE = DEFAULT_WS_URI
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_FROM_FRAME = "waist_yaw_link"
DEFAULT_TO_FRAME_LEFT = "left_tcp_link"
DEFAULT_TO_FRAME_RIGHT = "right_tcp_link"

DEFAULT_UDP_HOST = "192.168.41.1"
DEFAULT_UDP_PORT = 19011

# ---------------------------------------------------------------------------
# TCP 坐标系（left/right_tcp_link，相对 waist_yaw_link）
# URDF: tcp 固连于 wrist_roll，origin xyz=(0,0,-0.085)，rpy=0
#   → 手指/工具伸出方向 = TCP 的 -Z
# 整机：+X 前，+Y 左，+Z 上
# ---------------------------------------------------------------------------

# 固定默认 TCP（腰系米）。启动与 B 共用。
# 相对肘下就绪实测位再收：x 更小=往后，|y| 更小=往内。
DEFAULT_HOME_XYZ_LEFT = (0.35, 0.35, 0.08)
DEFAULT_HOME_XYZ_RIGHT = (0.35, -0.35, 0.08)

# 叠加在绝对 home 之上的微调（米）；默认不再额外偏移。
DEFAULT_HOME_OFFSET_XYZ = (0.0, 0.0, 0.0)
DEFAULT_HOME_DURATION_S = 2.0
DEFAULT_HOME_COOLDOWN_S = 2.0
DEFAULT_HOME_MAX_STEP_M = 0.015
# 当前 TCP 已在默认位附近时，跳过启动/回位运动，避免无意义抖动
DEFAULT_HOME_SKIP_TOL_M = 0.04

# 关节空间就绪（肘朝下弯）：顺序 pitch,roll,yaw,elbow,elbow_yaw,wrist_pitch,wrist_roll
# 参考算法手册 joint_space 示例，略放低肩俯仰
HOME_Q_LEFT = (-0.20, 0.40, 0.25, -1.15, -0.20, -0.10, 0.0)
HOME_Q_RIGHT = (-0.20, -0.40, -0.25, -1.15, 0.20, -0.10, 0.0)
DEFAULT_HOME_JOINT_DURATION_S = 3.0

# hold_box：固定 XYZ + 绝对 RPY（TCP -Z 朝前 / +X）；keep：只改位置、姿态保持当前。
HOME_POSE_PRESETS = {
    "keep": {"left": None, "right": None, "use_joints": False},
    "hold_box": {
        "left": (90.0, 0.0, -90.0),
        "right": (-90.0, 0.0, 90.0),
        "use_joints": False,
    },
}

DEFAULT_HOME_POSE = "hold_box"

# 默认绝对 RPY（度）：TCP -Z → 整机 +X（前方）；左右掌心相对
DEFAULT_HOME_RPY_DEG_LEFT = (90.0, 0.0, -90.0)
DEFAULT_HOME_RPY_DEG_RIGHT = (-90.0, 0.0, 90.0)
DEFAULT_HOME_RPY_OFFSET_DEG = (0.0, 0.0, 0.0)
# 姿态误差超过该值（度）时，即使位置已近也继续回位
DEFAULT_HOME_SKIP_TOL_DEG = 12.0

# 兼容旧文档中的抱箱 RPY（与默认相同）
HOME_RPY_HOLD_BOX_LEFT_DEG = (90.0, 0.0, -90.0)
HOME_RPY_HOLD_BOX_RIGHT_DEG = (-90.0, 0.0, 90.0)

# 遥操作稳定性（末端 QP 跟踪慢，需抑制噪声 + 松开时锁住目标）
DEFAULT_GRIP_ENGAGE = 0.55
DEFAULT_GRIP_RELEASE = 0.35
DEFAULT_POS_DEADZONE_M = 0.004
DEFAULT_ROT_DEADZONE_DEG = 2.0
DEFAULT_MAX_CMD_STEP_M = 0.010  # 每控制周期最大位移（50Hz ≈ 0.5m/s）
DEFAULT_RELEASE_FREEZE_S = 0.25  # active=false 释放通知重发时长
DEFAULT_POS_FILTER_ALPHA = 0.45
DEFAULT_ROT_FILTER_ALPHA = 0.40
DEFAULT_ROTATION_MODE = "always"  # Grip 同时跟随平移与姿态；仍可选 hold-a
