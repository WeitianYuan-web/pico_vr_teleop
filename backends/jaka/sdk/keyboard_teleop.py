#!/usr/bin/env python3
"""JAKA SDK servo_p 笛卡尔键盘遥操作。

- 长按移动、松手停止，空格也可立即停止
- 目标位姿独立连续积分，仅在开始/停止/重置时同步实际 TCP
- 逆解失败 / 跟踪误差过大时自动停车并重同步

用法::

    ./run.sh keyboard_teleop.py --no-power --no-shutdown
"""

from __future__ import annotations

import argparse
import math
import os
import select
import struct
import sys
import termios
import time
from pathlib import Path

from config import HOME_JOINT_DEG, ROBOT_IP
from jaka_sdk_client import ERR_NAMES, JakaSdkError, JakaSdkRobot

SERVO_PERIOD_S = 0.008
DEFAULT_SPEED_MM_S = 160.0
DEFAULT_SPEED_DEG_S = 40.0
DEFAULT_ACCEL_MM_S2 = 500.0
DEFAULT_ACCEL_DEG_S2 = 120.0
DEFAULT_RAPID_RATE = 0.9
DEFAULT_FILTER = "lpf"
DEFAULT_SAFETY_CHECK_INTERVAL = 50
MAX_TRACK_ERR_MM = 80.0
MAX_TRACK_ERR_RAD = math.radians(15.0)


KEY_CODE_TO_NAME = {
    1: "esc",
    16: "q",
    17: "w",
    18: "e",
    19: "r",
    22: "u",
    23: "i",
    24: "o",
    25: "p",
    30: "a",
    31: "s",
    32: "d",
    35: "h",
    36: "j",
    37: "k",
    38: "l",
    45: "x",
    57: " ",
}


class LinuxInputKeyboard:
    """读取 Linux /dev/input 真实按键按下/松开事件。"""

    EVENT = struct.Struct("llHHi")
    EV_KEY = 0x01

    def __init__(self, device: str | None = None) -> None:
        self.path = Path(device) if device else self._default_keyboard_device()
        self._fd: int | None = None

    def __enter__(self) -> LinuxInputKeyboard:
        try:
            self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError as exc:
            raise JakaSdkError(
                f"无法读取键盘设备 {self.path}。请使用 sudo 运行，"
                "或把当前用户加入 input 组后重新登录。"
            ) from exc
        except FileNotFoundError as exc:
            raise JakaSdkError(f"键盘设备不存在: {self.path}") from exc
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def read_events(self) -> list[tuple[str, int]]:
        if self._fd is None:
            return []
        events: list[tuple[str, int]] = []
        ready, _, _ = select.select([self._fd], [], [], 0)
        if not ready:
            return events
        try:
            data = os.read(self._fd, self.EVENT.size * 32)
        except BlockingIOError:
            return events
        for offset in range(0, len(data) - self.EVENT.size + 1, self.EVENT.size):
            _, _, event_type, code, value = self.EVENT.unpack_from(data, offset)
            if event_type != self.EV_KEY:
                continue
            key = KEY_CODE_TO_NAME.get(code)
            if key is not None:
                events.append((key, value))
        return events

    @staticmethod
    def _default_keyboard_device() -> Path:
        candidates: list[Path] = []
        candidates.extend(sorted(Path("/dev/input/by-id").glob("*event-kbd")))
        candidates.extend(sorted(Path("/dev/input/by-path").glob("*event-kbd")))
        candidates.extend(sorted(Path("/dev/input").glob("event*")))
        if not candidates:
            raise JakaSdkError("未找到 /dev/input 键盘设备")
        return candidates[0]


class TerminalEchoGuard:
    """运行时关闭终端回显，避免 /dev/input 控制时字符刷屏。"""

    def __init__(self) -> None:
        self._fd = sys.stdin.fileno()
        self._old_termios: list | None = None

    def __enter__(self) -> TerminalEchoGuard:
        if not sys.stdin.isatty():
            return self
        self._old_termios = termios.tcgetattr(self._fd)
        attrs = termios.tcgetattr(self._fd)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(self._fd, termios.TCSADRAIN, attrs)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old_termios is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)
            termios.tcflush(self._fd, termios.TCIFLUSH)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="JAKA SDK servo_p 键盘遥操作")
    p.add_argument("--no-power", action="store_true")
    p.add_argument("--no-shutdown", action="store_true")
    p.add_argument("--no-home", action="store_true")
    return p.parse_args()


def format_pose(pose: list[float]) -> str:
    return (
        f"xyz=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) mm, "
        f"rpy=({math.degrees(pose[3]):.2f}, {math.degrees(pose[4]):.2f}, "
        f"{math.degrees(pose[5]):.2f}) deg"
    )


def print_help(
    home_joints_deg: list[float],
) -> None:
    joints = ", ".join(f"J{i + 1}={v:.1f}" for i, v in enumerate(home_joints_deg))
    print(
        f"""
=== servo_p 笛卡尔键盘遥操作 ===
速度倍率: {DEFAULT_RAPID_RATE * 100:.0f}% | 平移: {DEFAULT_SPEED_MM_S} mm/s | 旋转: {DEFAULT_SPEED_DEG_S} deg/s
加速度限制: 平移 {DEFAULT_ACCEL_MM_S2} mm/s^2 | 旋转 {DEFAULT_ACCEL_DEG_S2} deg/s^2
控制周期: {SERVO_PERIOD_S * 1000:.1f} ms | 滤波: {DEFAULT_FILTER}
初始关节角(deg): {joints}

  W/S  X+/X-        A/D  Y+/Y-        Q/E  Z+/Z-
  I/K  Rx+/Rx-      J/L  Ry+/Ry-      U/O  Rz+/Rz-
  空格  停止          R  目标对齐实际   H  关节回初始点
  P    打印位姿        X  退出

逻辑: 读取 /dev/input 真实按下/松开事件；长按移动，松手停止。
支持方向合成: 可同时按多个方向键进行斜向/复合运动。
"""
    )


def cartesian_speed_cmd(key: str, speed_mm_s: float, speed_deg_s: float) -> list[float] | None:
    """将按键映射为笛卡尔速度 [vx,vy,vz,wx,wy,wz]（mm/s 与 rad/s）。"""
    rad_s = math.radians(speed_deg_s)
    return {
        "w": [speed_mm_s, 0, 0, 0, 0, 0],
        "s": [-speed_mm_s, 0, 0, 0, 0, 0],
        "a": [0, speed_mm_s, 0, 0, 0, 0],
        "d": [0, -speed_mm_s, 0, 0, 0, 0],
        "q": [0, 0, speed_mm_s, 0, 0, 0],
        "e": [0, 0, -speed_mm_s, 0, 0, 0],
        "i": [0, 0, 0, rad_s, 0, 0],
        "k": [0, 0, 0, -rad_s, 0, 0],
        "j": [0, 0, 0, 0, rad_s, 0],
        "l": [0, 0, 0, 0, -rad_s, 0],
        "u": [0, 0, 0, 0, 0, rad_s],
        "o": [0, 0, 0, 0, 0, -rad_s],
    }.get(key)


def compose_speed_cmd(keys: list[str], speed_mm_s: float, speed_deg_s: float) -> list[float]:
    """将多个按键方向合成为单个速度向量，并限制线/角速度上限。"""
    cmd = [0.0] * 6
    for key in keys:
        delta = cartesian_speed_cmd(key, speed_mm_s, speed_deg_s)
        if delta is None:
            continue
        for idx in range(6):
            cmd[idx] += delta[idx]

    lin_mag = math.sqrt(sum(cmd[idx] * cmd[idx] for idx in range(3)))
    if lin_mag > speed_mm_s > 0:
        scale = speed_mm_s / lin_mag
        for idx in range(3):
            cmd[idx] *= scale

    ang_limit = math.radians(speed_deg_s)
    ang_mag = math.sqrt(sum(cmd[idx] * cmd[idx] for idx in range(3, 6)))
    if ang_mag > ang_limit > 0:
        scale = ang_limit / ang_mag
        for idx in range(3, 6):
            cmd[idx] *= scale

    return cmd


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
    """
    /**
     * @brief 关节空间移动到初始姿态
     * @param robot JAKA SDK 客户端
     * @param home_joints_deg 目标关节角（度）
     * @param speed_deg_s 关节运动速度（度/秒）
     */
    """
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


def sync_target_to_actual(robot: JakaSdkRobot) -> list[float]:
    return robot.get_tcp_pos()


def integrate_target(target: list[float], speed_cmd: list[float], dt_s: float) -> list[float]:
    return [target[i] + speed_cmd[i] * dt_s for i in range(6)]


def ramp_speed(current: list[float], target: list[float], accel: list[float], dt_s: float) -> list[float]:
    next_speed: list[float] = []
    for cur, goal, limit in zip(current, target, accel, strict=True):
        step = abs(limit) * dt_s
        if goal > cur:
            next_speed.append(min(goal, cur + step))
        elif goal < cur:
            next_speed.append(max(goal, cur - step))
        else:
            next_speed.append(cur)
    return next_speed


def run_servo_p_teleop(
    robot: JakaSdkRobot,
    args: argparse.Namespace,
    home_joints_deg: list[float],
) -> None:
    target_speed_cmd = [0.0] * 6
    smooth_speed_cmd = [0.0] * 6
    accel_limit = [
        DEFAULT_ACCEL_MM_S2,
        DEFAULT_ACCEL_MM_S2,
        DEFAULT_ACCEL_MM_S2,
        math.radians(DEFAULT_ACCEL_DEG_S2),
        math.radians(DEFAULT_ACCEL_DEG_S2),
        math.radians(DEFAULT_ACCEL_DEG_S2),
    ]
    mapper = lambda k: cartesian_speed_cmd(k, DEFAULT_SPEED_MM_S, DEFAULT_SPEED_DEG_S)
    moving_prev = False
    pressed_motion_keys: list[str] = []
    safety_interval = DEFAULT_SAFETY_CHECK_INTERVAL
    period_s = max(0.004, SERVO_PERIOD_S)

    def resync_target() -> list[float]:
        return sync_target_to_actual(robot)

    if not args.no_home:
        try:
            move_to_home(robot, home_joints_deg)
        except JakaSdkError as exc:
            print(f"警告: 无法自动回初始点 ({exc})")
            print("将使用当前位姿继续；若示教器有防护停止请先清除，或加 --no-home")

    robot.prepare_servo(filter=DEFAULT_FILTER)
    target = sync_target_to_actual(robot)
    print(f"当前 TCP: {format_pose(target)}")
    print("已进入 servo_p。按住移动键运动，松手停止。")
    input("确认安全后按 Enter 开始...")

    err_streak = 0
    tick = 0
    next_tick = time.perf_counter()

    try:
        with TerminalEchoGuard(), LinuxInputKeyboard() as kb:
            print(f"键盘设备: {kb.path}")
            while True:
                dt_s = period_s

                events = kb.read_events()

                if events:
                    for key, value in events:
                        is_press = value == 1
                        is_down = value in (1, 2)
                        is_up = value == 0
                        if is_down and key in ("x", "esc"):
                            return
                        if is_press and key == " ":
                            target_speed_cmd[:] = [0.0] * 6
                            smooth_speed_cmd[:] = [0.0] * 6
                            pressed_motion_keys.clear()
                            target = resync_target()
                            continue
                        if is_press and key == "p":
                            actual = robot.get_tcp_pos()
                            pe, re = tracking_error(actual, target)
                            print(f"\n目标: {format_pose(target)}")
                            print(f"实际: {format_pose(actual)}")
                            print(f"误差: xyz={pe:.2f} mm, rot={math.degrees(re):.2f} deg")
                            continue
                        if is_press and key == "r":
                            target_speed_cmd[:] = [0.0] * 6
                            smooth_speed_cmd[:] = [0.0] * 6
                            pressed_motion_keys.clear()
                            target = resync_target()
                            print("\n已对齐实际 TCP")
                            continue
                        if is_press and key == "h":
                            target_speed_cmd[:] = [0.0] * 6
                            smooth_speed_cmd[:] = [0.0] * 6
                            pressed_motion_keys.clear()
                            robot.exit_servo()
                            move_to_home(robot, home_joints_deg)
                            robot.prepare_servo(filter=DEFAULT_FILTER)
                            target = resync_target()
                            print("已回初始关节角")
                            continue
                        if mapper(key) is not None:
                            if is_down:
                                if key in pressed_motion_keys:
                                    pressed_motion_keys.remove(key)
                                pressed_motion_keys.append(key)
                            elif is_up and key in pressed_motion_keys:
                                pressed_motion_keys.remove(key)

                            new_cmd = compose_speed_cmd(
                                pressed_motion_keys,
                                DEFAULT_SPEED_MM_S,
                                DEFAULT_SPEED_DEG_S,
                            )
                            start_moving = (
                                not any(abs(v) > 1e-12 for v in target_speed_cmd)
                                and any(abs(v) > 1e-12 for v in new_cmd)
                            )
                            if start_moving:
                                target = resync_target()
                            target_speed_cmd[:] = new_cmd

                actual: list[float] | None = None
                smooth_speed_cmd = ramp_speed(
                    smooth_speed_cmd,
                    target_speed_cmd,
                    accel_limit,
                    dt_s,
                )
                moving = any(smooth_speed_cmd)

                if moving:
                    candidate = integrate_target(target, smooth_speed_cmd, dt_s)
                    errno = robot.servo_p(candidate)
                    if errno == 0:
                        target = candidate
                        err_streak = 0

                        if tick % safety_interval == 0:
                            actual = robot.get_tcp_pos()
                            joints = robot.get_joint_pos_rad()
                            ik = robot.kine_inverse(joints, target, raise_on_error=False)
                            if ik is None:
                                target_speed_cmd[:] = [0.0] * 6
                                smooth_speed_cmd[:] = [0.0] * 6
                                target = actual
                                print("\n[奇异点] 逆解失败，已停车并重同步")
                            else:
                                pe, re = tracking_error(actual, target)
                                if pe > MAX_TRACK_ERR_MM or re > MAX_TRACK_ERR_RAD:
                                    target_speed_cmd[:] = [0.0] * 6
                                    smooth_speed_cmd[:] = [0.0] * 6
                                    target = actual
                                    print(
                                        f"\n[跟踪过大] xyz={pe:.1f}mm，已停车重同步"
                                    )
                    else:
                        err_streak += 1
                        name = ERR_NAMES.get(errno, "?")
                        if err_streak <= 5:
                            print(f"\n[servo_p errno={errno} ({name})]")
                        actual = resync_target()
                        target_speed_cmd[:] = [0.0] * 6
                        smooth_speed_cmd[:] = [0.0] * 6
                        target = actual
                else:
                    if moving_prev:
                        actual = resync_target()
                        target = actual
                    robot.servo_p(target)
                moving_prev = moving
                tick += 1

                next_tick += period_s
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()
    finally:
        target_speed_cmd[:] = [0.0] * 6
        smooth_speed_cmd[:] = [0.0] * 6
        robot.exit_servo()
        print("\n已退出伺服模式")


def main() -> int:
    args = parse_args()
    home_joints_deg = HOME_JOINT_DEG.copy()

    try:
        with JakaSdkRobot(ROBOT_IP) as robot:
            robot.ensure_ready(skip_power=args.no_power)

            try:
                before = robot.get_rapid_rate()
                robot.set_rapid_rate(DEFAULT_RAPID_RATE)
                print(f"速度倍率: {before * 100:.0f}% -> {robot.get_rapid_rate() * 100:.0f}%")
            except JakaSdkError as exc:
                print(f"警告: 无法设置速度倍率 ({exc})")

            print_help(home_joints_deg)
            run_servo_p_teleop(robot, args, home_joints_deg)

            if not args.no_shutdown and not args.no_power:
                robot.disable_robot()
                robot.power_off()
            elif args.no_shutdown:
                print("保留上电/使能 (--no-shutdown)")

    except KeyboardInterrupt:
        print("\n用户中断")
        return 130
    except JakaSdkError as exc:
        print(f"\n错误: {exc}", file=sys.stderr)
        return 1

    print("遥操作结束")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
