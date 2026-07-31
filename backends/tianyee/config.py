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

# 笛卡尔微调（关节就绪之后）：略前伸、少量抬高（已调低）
DEFAULT_HOME_OFFSET_XYZ = (0.06, 0.0, 0.05)
DEFAULT_HOME_DURATION_S = 2.0
DEFAULT_HOME_COOLDOWN_S = 2.0
DEFAULT_HOME_MAX_STEP_M = 0.015

# 关节空间就绪（肘朝下弯）：顺序 pitch,roll,yaw,elbow,elbow_yaw,wrist_pitch,wrist_roll
# 参考算法手册 joint_space 示例，略放低肩俯仰
HOME_Q_LEFT = (-0.20, 0.40, 0.25, -1.15, -0.20, -0.10, 0.0)
HOME_Q_RIGHT = (-0.20, -0.40, -0.25, -1.15, 0.20, -0.10, 0.0)
DEFAULT_HOME_JOINT_DURATION_S = 3.0

# hold_box：先走关节就绪（肘下），姿态保持关节到位后的 TCP；keep：不调关节
HOME_POSE_PRESETS = {
    "keep": {"left": None, "right": None, "use_joints": False},
    "hold_box": {"left": None, "right": None, "use_joints": True},
}

DEFAULT_HOME_POSE = "hold_box"

# 可选笛卡尔绝对 RPY（度）；hold_box 默认不用，避免 IK 把肘翻上去
DEFAULT_HOME_RPY_DEG_LEFT = None
DEFAULT_HOME_RPY_DEG_RIGHT = None
DEFAULT_HOME_RPY_OFFSET_DEG = (0.0, 0.0, 0.0)

# 兼容旧文档中的抱箱 RPY（仅手动 --home-rpy-*-deg 时用）
HOME_RPY_HOLD_BOX_LEFT_DEG = (90.0, 0.0, -90.0)
HOME_RPY_HOLD_BOX_RIGHT_DEG = (-90.0, 0.0, 90.0)

# 遥操作稳定性（末端 QP 跟踪慢，需抑制噪声 + 松开时锁住目标）
DEFAULT_GRIP_ENGAGE = 0.55
DEFAULT_GRIP_RELEASE = 0.35
DEFAULT_POS_DEADZONE_M = 0.004
DEFAULT_ROT_DEADZONE_DEG = 2.0
DEFAULT_MAX_CMD_STEP_M = 0.008  # 每控制周期最大位移（50Hz ≈ 0.4m/s）
DEFAULT_RELEASE_FREEZE_S = 0.45
DEFAULT_POS_FILTER_ALPHA = 0.25
DEFAULT_ROT_FILTER_ALPHA = 0.25
DEFAULT_ROTATION_MODE = "hold-a"  # Grip 仅平移；按住 A 才旋转，减少乱动
