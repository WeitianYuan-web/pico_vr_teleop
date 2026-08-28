"""IO 重定向 JointState（rad）→ 寄存器角度。默认 RH56F2；RH5DG2 为 13 DOF。"""

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

G2_CHANNEL_NAMES: Tuple[str, ...] = (
    "pinky_mcp",
    "ring_mcp",
    "middle_mcp",
    "middle_yaw",
    "index_mcp",
    "index_yaw",
    "pinky_pip",
    "ring_pip",
    "middle_pip",
    "index_pip",
    "thumb_yaw",
    "thumb_mcp",
    "thumb_dip",
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

# 与 io_exotrans2hand inspire_rh5dg2_teleop_bridge.py 一致
G2_FINGER_MAPPINGS: Tuple[FingerMapping, ...] = (
    FingerMapping(("pinky_mcp_joint",), 1650, 900, 0.0, 1.31),
    FingerMapping(("ring_mcp_joint",), 1650, 900, 0.0, 1.31),
    FingerMapping(("middle_mcp_joint",), 1650, 900, 0.0, 1.31),
    FingerMapping(("middle_yaw_joint",), -150, 150, -0.26, 0.26),
    FingerMapping(("index_mcp_joint",), 1650, 900, 0.0, 1.31),
    FingerMapping(("index_yaw_joint",), -150, 150, -0.26, 0.26),
    FingerMapping(("pinky_pip_joint",), 1900, 1050, 0.0, 1.48),
    FingerMapping(("ring_pip_joint",), 1900, 1050, 0.0, 1.48),
    FingerMapping(("middle_pip_joint",), 1900, 1050, 0.0, 1.48),
    FingerMapping(("index_pip_joint",), 1900, 1050, 0.0, 1.48),
    FingerMapping(("thumb_yaw_joint",), 1750, 650, 0.0, 1.92),
    FingerMapping(("thumb_mcp_joint",), 1600, 1250, 0.1, 0.61),
    FingerMapping(("thumb_dip_joint",), 2040, 1500, 0.1, 0.94),
)

MODEL_ALIASES = {
    "rh56f2": "rh56f2",
    "f2": "rh56f2",
    "inspire_rh56f2": "rh56f2",
    "rh5dg2": "rh5dg2",
    "g2": "rh5dg2",
    "dg2": "rh5dg2",
    "064": "rh5dg2",
    "inspire_rh5dg2": "rh5dg2",
}


@dataclass(frozen=True)
class HandProfile:
    key: str
    sdk_model: str
    io_hand: str
    channel_names: Tuple[str, ...]
    mappings: Tuple[FingerMapping, ...]
    state_joint_names: Tuple[str, ...]
    default_force: int
    default_speed: int
    default_baud: int

    @property
    def dof(self) -> int:
        return len(self.mappings)


HAND_PROFILES: Dict[str, HandProfile] = {
    "rh56f2": HandProfile(
        key="rh56f2",
        sdk_model="rh56f2",
        io_hand="Inspire_RH56F2",
        channel_names=CYLINDER_NAMES,
        mappings=FINGER_MAPPINGS,
        state_joint_names=tuple(f"finger_{i}" for i in range(1, 7)),
        default_force=6000,
        default_speed=4000,
        default_baud=115200,
    ),
    "rh5dg2": HandProfile(
        key="rh5dg2",
        sdk_model="rh5dg2",
        io_hand="Inspire_RH5DG2",
        channel_names=G2_CHANNEL_NAMES,
        mappings=G2_FINGER_MAPPINGS,
        state_joint_names=tuple(m.joint_suffixes[0] for m in G2_FINGER_MAPPINGS),
        default_force=1000,
        default_speed=2500,
        default_baud=921600,
    ),
}


def normalize_hand_model(model: str) -> str:
    key = str(model).strip().lower().rstrip("/")
    if key not in MODEL_ALIASES:
        raise ValueError(
            f"未知手型 {model!r}，支持: rh56f2 / f2 / rh5dg2 / g2 / dg2"
        )
    return MODEL_ALIASES[key]


def resolve_hand_profile(model: str) -> HandProfile:
    return HAND_PROFILES[normalize_hand_model(model)]


def map_named_positions_to_angles(
    names: Sequence[str],
    positions: Sequence[float],
    joint_prefix: str,
    mappings: Optional[Sequence[FingerMapping]] = None,
) -> Optional[List[int]]:
    table = tuple(mappings) if mappings is not None else FINGER_MAPPINGS
    if not names or not positions or len(names) != len(positions):
        return None

    joint_dic: Dict[str, float] = dict(zip(names, positions))
    angle_vals: List[int] = []

    for mapping in table:
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
    mappings: Optional[Sequence[FingerMapping]] = None,
) -> Optional[Tuple[List[str], List[float]]]:
    """寄存器角度 → JointState 名/rad（再经 map_named_positions_to_angles 还原）。"""
    table = tuple(mappings) if mappings is not None else FINGER_MAPPINGS
    if len(angles) != len(table):
        return None
    names: List[str] = []
    positions: List[float] = []
    for angle, mapping in zip(angles, table):
        span_a = mapping.actuator_at_rad_upper - mapping.actuator_at_rad_lower
        if span_a == 0:
            ratio = 0.0
        else:
            ratio = (float(angle) - mapping.actuator_at_rad_lower) / float(span_a)
            ratio = max(0.0, min(1.0, ratio))
        rad = mapping.rad_sum_lower + ratio * (mapping.rad_sum_upper - mapping.rad_sum_lower)
        suffix = mapping.joint_suffixes[0]
        names.append(f"{joint_prefix}{suffix}")
        positions.append(rad)
        for extra in mapping.joint_suffixes[1:]:
            names.append(f"{joint_prefix}{extra}")
            positions.append(0.0)
    return names, positions
