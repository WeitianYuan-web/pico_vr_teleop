"""VR Trigger → /hand_cmd JointState（不碰串口）。"""

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

from mapping import FINGER_MAPPINGS, angles_to_named_positions

# 与 dual_arm_dual_hand_webxr / Inspire profile 一致
HAND_FULL_CLOSE_BY_MODEL = {
    "rh56f2": [900, 900, 900, 900, 1100, 500],
    "f2": [900, 900, 900, 900, 1100, 500],
    "rh56h1": [870, 870, 870, 870, 950, 700],
    "h1": [870, 870, 870, 870, 950, 700],
}

DEFAULT_HAND_MODEL = "rh56f2"


def default_open_pose(model: str = DEFAULT_HAND_MODEL) -> List[int]:
    """张开端点：映射表 actuator_at_rad_lower（rad=0）。"""
    _ = model
    return [m.actuator_at_rad_lower for m in FINGER_MAPPINGS]


def default_close_pose(model: str = DEFAULT_HAND_MODEL) -> List[int]:
    key = str(model).strip().lower() or DEFAULT_HAND_MODEL
    return list(HAND_FULL_CLOSE_BY_MODEL.get(key, HAND_FULL_CLOSE_BY_MODEL[DEFAULT_HAND_MODEL]))


def lerp_hand_pose(open_pose: Sequence[int], close_pose: Sequence[int], alpha: float) -> List[int]:
    a = max(0.0, min(float(alpha), 1.0))
    n = min(len(open_pose), len(close_pose))
    return [int(open_pose[i] + a * (close_pose[i] - open_pose[i])) for i in range(n)]


def trigger_to_alpha(trigger: float, hand_min: float, hand_max: float) -> float:
    t = max(0.0, min(float(trigger), 1.0))
    lo = max(0.0, min(float(hand_min), 1.0))
    hi = max(lo, min(float(hand_max), 1.0))
    return lo + t * (hi - lo)


def alpha_to_joint_state(
    alpha: float,
    side: str,
    *,
    open_pose: Optional[Sequence[int]] = None,
    close_pose: Optional[Sequence[int]] = None,
    model: str = DEFAULT_HAND_MODEL,
) -> Optional[Tuple[List[str], List[float]]]:
    """alpha → (joint names, rad positions) for /hand_cmd/{side}."""
    prefix = "left_" if side == "left" else "right_"
    open_p = list(open_pose) if open_pose is not None else default_open_pose(model)
    close_p = list(close_pose) if close_pose is not None else default_close_pose(model)
    angles = lerp_hand_pose(open_p, close_p, alpha)
    return angles_to_named_positions(angles, prefix)
