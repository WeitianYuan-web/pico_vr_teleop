"""WebXR / 遥操作共用常量。"""

from __future__ import annotations

HANDS: tuple[str, ...] = ("left", "right")
BTN_A_INDEX: int = 4  # WebXR xr-standard：A 键（常用于按住启用旋转）
BTN_B_INDEX: int = 5  # WebXR xr-standard：B 键（常用于回零）
DEFAULT_WS_URI: str = "wss://localhost:8081"
