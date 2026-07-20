"""JAKA Python SDK 路径与机器人配置。"""

from pathlib import Path

# 机械臂 IP（与 tcp_ip/config.py 一致）
ROBOT_IP = "192.168.10.21"

_SDK_ROOT = Path(__file__).resolve().parents[1] / (
    "20260104145805A007/SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu"
)

SDK_LIB_DIR = str(_SDK_ROOT)

# 遥操作初始关节角（度）J1-J6
HOME_JOINT_DEG: list[float] = [-90.0, 95.0, 120.0, 40.0, -95.0, 0.0]
