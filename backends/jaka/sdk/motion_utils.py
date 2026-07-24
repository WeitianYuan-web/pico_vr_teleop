"""JAKA 运动辅助：回零、跟踪误差、位姿打印。供 VR 遥操作复用。"""

from __future__ import annotations

import math

from jaka_sdk_client import JakaSdkRobot


def format_pose(pose: list[float]) -> str:
    return (
        f"xyz=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) mm, "
        f"rpy=({math.degrees(pose[3]):.2f}, {math.degrees(pose[4]):.2f}, "
        f"{math.degrees(pose[5]):.2f}) deg"
    )


def tracking_error(actual: list[float], target: list[float]) -> tuple[float, float]:
    pos = math.sqrt(sum((actual[i] - target[i]) ** 2 for i in range(3)))
    rot = max(abs(actual[i] - target[i]) for i in range(3, 6))
    return pos, rot


def move_to_home(
    robot: JakaSdkRobot,
    home_joints_deg: list[float],
    *,
    speed_deg_s: float = 15.0,
) -> None:
    joints = ", ".join(f"J{i + 1}={v:.1f}" for i, v in enumerate(home_joints_deg))
    print(f"移动到初始关节角(deg): {joints} @ {speed_deg_s:.1f} deg/s")
    robot.joint_move_deg(home_joints_deg, block=True, speed_deg_s=speed_deg_s)
    actual = robot.get_joint_pos_deg()
    err = sum(abs(actual[i] - home_joints_deg[i]) for i in range(6))
    tcp = robot.get_tcp_pos()
    print(
        f"到位: {[round(v, 2) for v in actual]} deg (关节总误差 {err:.2f} deg), "
        f"TCP: {format_pose(tcp)}"
    )
