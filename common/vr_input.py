"""WebXR 手柄按钮与旋转模式判定。"""

from __future__ import annotations

from typing import Literal

from common.constants import BTN_A_INDEX


def is_button_pressed(ctrl: dict, index: int) -> bool:
    buttons = ctrl.get("buttons")
    if not buttons or len(buttons) <= index:
        return False
    return bool(buttons[index].get("pressed", False))


def rotation_enabled(
    ctrl: dict,
    mode: Literal["always", "hold-a", "off"] | str,
    *,
    btn_a_index: int = BTN_A_INDEX,
) -> bool:
    if mode == "off":
        return False
    if mode == "hold-a":
        return is_button_pressed(ctrl, btn_a_index)
    return True
