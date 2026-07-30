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

# 可选：B 键回退的腰系末端位姿 [x,y,z, qw,qx,qy,qz]；None 表示仅松开离合
DEFAULT_HOME_POSE_LEFT = None
DEFAULT_HOME_POSE_RIGHT = None
