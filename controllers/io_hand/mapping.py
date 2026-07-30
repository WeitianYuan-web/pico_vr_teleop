"""RH56F2：IO 重定向 JointState（rad）→ 电缸寄存器角度。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

CYLINDER_NAMES: Tuple[str, ...] = (
    "电缸1 (Pinky)",
    "电缸2 (Ring)",
    "电缸3 (Middle)",
    "电缸4 (Index)",
    "电缸5 (Thumb Flexion)",
    "电缸6 (Thumb Abduction)",
)


@dataclass(frozen=True)
class FingerMapping:
    joint_suffixes: Tuple[str, ...]
    actuator_at_rad_lower: int
    actuator_at_rad_upper: int
    rad_sum_lower: float
    rad_sum_upper: float


# 与原 inspire_rh56f2_teleop_bridge.py / Rh56F2Profile 一致
FINGER_MAPPINGS: Tuple[FingerMapping, ...] = (
    FingerMapping(("pinky_1_joint",), 1740, 900, 0.0, 1.47),
    FingerMapping(("ring_1_joint",), 1740, 900, 0.0, 1.47),
    FingerMapping(("middle_1_joint",), 1740, 900, 0.0, 1.47),
    FingerMapping(("index_1_joint",), 1740, 900, 0.0, 1.47),
    FingerMapping(("thumb_2_joint",), 1450, 1100, 0.0, 0.79),
    FingerMapping(("thumb_1_joint",), 1750, 500, 0.0, 2.0),
)


def map_named_positions_to_angles(
    names: Sequence[str],
    positions: Sequence[float],
    joint_prefix: str,
) -> Optional[List[int]]:
    if not names or not positions or len(names) != len(positions):
        return None

    joint_dic: Dict[str, float] = dict(zip(names, positions))
    angle_vals: List[int] = []

    for mapping in FINGER_MAPPINGS:
        joint_names = [f"{joint_prefix}{suffix}" for suffix in mapping.joint_suffixes]
        if any(name not in joint_dic for name in joint_names):
            return None

        rad_sum = sum(joint_dic[name] for name in joint_names)
        span = mapping.rad_sum_upper - mapping.rad_sum_lower
        if span <= 0.0:
            ratio = 0.0
        else:
            ratio = (rad_sum - mapping.rad_sum_lower) / span
            ratio = max(0.0, min(1.0, ratio))

        angle_val = int(
            mapping.actuator_at_rad_lower
            + ratio * (mapping.actuator_at_rad_upper - mapping.actuator_at_rad_lower)
        )
        angle_vals.append(angle_val)

    return angle_vals


def angles_to_named_positions(
    angles: Sequence[int],
    joint_prefix: str,
) -> Optional[Tuple[List[str], List[float]]]:
    """6 路寄存器角度 → JointState 名/rad（供 /hand_cmd 发布，再经 map_named_positions_to_angles 还原）。"""
    if len(angles) != len(FINGER_MAPPINGS):
        return None
    names: List[str] = []
    positions: List[float] = []
    for angle, mapping in zip(angles, FINGER_MAPPINGS):
        span_a = mapping.actuator_at_rad_upper - mapping.actuator_at_rad_lower
        if span_a == 0:
            ratio = 0.0
        else:
            ratio = (float(angle) - mapping.actuator_at_rad_lower) / float(span_a)
            ratio = max(0.0, min(1.0, ratio))
        rad = mapping.rad_sum_lower + ratio * (mapping.rad_sum_upper - mapping.rad_sum_lower)
        # 单关节后缀：整段 rad 放在第一个 joint 上
        suffix = mapping.joint_suffixes[0]
        names.append(f"{joint_prefix}{suffix}")
        positions.append(rad)
        for extra in mapping.joint_suffixes[1:]:
            names.append(f"{joint_prefix}{extra}")
            positions.append(0.0)
    return names, positions
