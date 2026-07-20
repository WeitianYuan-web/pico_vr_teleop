#!/usr/bin/env python3
"""JAKA 机械臂 servo_p 键盘遥操作示例。

通过 TCP/IP 进入笛卡尔伺服模式，以约 125Hz 发送 servo_p 指令。
按键设置平移/旋转速度，松开后按 Space 停止运动。

用法::

    python3 keyboard_servo_p_teleop.py
    python3 keyboard_servo_p_teleop.py --no-power          # 已在示教器上电使能时
    python3 keyboard_servo_p_teleop.py --no-shutdown        # 结束后不下电
    python3 keyboard_servo_p_teleop.py --speed-mm 12 --speed-deg 8
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import tty
import time
from typing import TextIO

from config import ROBOT_IP
from jaka_tcp_client import JakaTcpClient, JakaTcpError


SERVO_PERIOD_S = 0.008


class TerminalKeyboard:
    """非阻塞终端按键读取。"""

    def __init__(self, stream: TextIO | None = None) -> None:
        self._stream = stream or sys.stdin
        self._fd = self._stream.fileno()
        self._old_termios: list | None = None

    def __enter__(self) -> TerminalKeyboard:
        self._old_termios = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._old_termios is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)

    def read_keys(self) -> list[str]:
        """读取当前缓冲区内的全部按键。"""
        keys: list[str] = []
        while True:
            ready, _, _ = select.select([self._stream], [], [], 0)
            if not ready:
                break
            ch = self._stream.read(1)
            if ch == "\x1b":
                if select.select([self._stream], [], [], 0)[0]:
                    ch += self._stream.read(2)
                else:
                    keys.append("\x1b")
                    break
            keys.append(ch)
        return keys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="JAKA servo_p 键盘遥操作")
    parser.add_argument("--ip", default=ROBOT_IP, help="机械臂 IP")
    parser.add_argument(
        "--speed-mm",
        type=float,
        default=10.0,
        help="平移速度 (mm/s)，默认 10",
    )
    parser.add_argument(
        "--speed-deg",
        type=float,
        default=6.0,
        help="旋转速度 (度/s)，默认 6",
    )
    parser.add_argument("--period", type=float, default=SERVO_PERIOD_S, help="servo_p 周期 (秒)")
    parser.add_argument(
        "--no-power",
        action="store_true",
        help="跳过程序内上电/使能（已在示教器/App 完成时使用）",
    )
    parser.add_argument(
        "--mode",
        choices=("servo_p", "servo_j"),
        default="servo_p",
        help="servo_p=笛卡尔伺服(默认); servo_j=逆解后关节伺服(更稳)",
    )
    parser.add_argument(
        "--no-shutdown",
        action="store_true",
        help="结束后不下使能/断电",
    )
    return parser.parse_args()


def print_help(speed_mm: float, speed_deg: float, period: float, mode: str) -> None:
    tick_mm = speed_mm * period
    tick_deg = speed_deg * period
    print(
        f"""
=== servo 键盘遥操作 ({mode}) ===
平移速度: {speed_mm} mm/s ({tick_mm:.3f} mm/周期)
旋转速度: {speed_deg} deg/s ({tick_deg:.3f} deg/周期)
控制周期: {period * 1000:.1f} ms

  W/S  X+/X-        A/D  Y+/Y-        Q/E  Z+/Z-
  I/K  Rx+/Rx-      J/L  Ry+/Ry-      U/O  Rz+/Rz-
  空格  停止运动      R  重置到当前实际位姿
  P    打印目标/实际位姿              X  退出

提示: 按住移动键持续运动；servo_p 不动可试 --mode servo_j
"""
    )


def wait_for_state(
    robot: JakaTcpClient,
    *,
    powered: bool | None = None,
    enabled: bool | None = None,
    timeout: float = 25.0,
    interval: float = 0.5,
) -> dict:
    """轮询机器人状态直到满足条件。"""
    deadline = time.monotonic() + timeout
    last_state: dict = {}
    while time.monotonic() < deadline:
        last_state = robot.get_robot_state()
        power_ok = powered is None or JakaTcpClient.is_powered(last_state) == powered
        enable_ok = enabled is None or JakaTcpClient.is_enabled(last_state) == enabled
        if power_ok and enable_ok:
            return last_state
        time.sleep(interval)
    return last_state


def ensure_robot_ready(robot: JakaTcpClient, *, skip_power: bool) -> dict:
    state = robot.get_robot_state()
    print(
        f"当前状态: power={state.get('power')} enable={state.get('enable')} "
        f"errcode={state.get('errcode')} msg={state.get('msg', '')}"
    )

    if skip_power:
        if not JakaTcpClient.is_powered(state) or not JakaTcpClient.is_enabled(state):
            raise RuntimeError(
                "已指定 --no-power，但机械臂未上电或未使能。"
                "请先在示教器/App 上电并使能。"
            )
        return state

    if not JakaTcpClient.is_powered(state):
        print("发送上电指令（控制器上电通常需要约 8 秒）...")
        resp = robot.power_on()
        print(f"power_on 响应: errorCode={resp.get('errorCode')} errorMsg={resp.get('errorMsg')}")
        state = wait_for_state(robot, powered=True, timeout=25.0)
        print(
            f"上电轮询结果: power={state.get('power')} errcode={state.get('errcode')} "
            f"msg={state.get('msg', '')}"
        )

    if JakaTcpClient.is_powered(state) and not JakaTcpClient.is_enabled(state):
        print("发送使能指令...")
        resp = robot.enable_robot()
        print(f"enable_robot 响应: errorCode={resp.get('errorCode')} errorMsg={resp.get('errorMsg')}")
        state = wait_for_state(robot, powered=True, enabled=True, timeout=15.0)
        print(
            f"使能轮询结果: enable={state.get('enable')} errcode={state.get('errcode')} "
            f"msg={state.get('msg', '')}"
        )

    if not JakaTcpClient.is_powered(state) or not JakaTcpClient.is_enabled(state):
        raise RuntimeError(
            "机械臂未能上电/使能。请检查急停、安全回路，或在示教器手动上电使能后使用 --no-power"
        )
    return state


def read_initial_pose(robot: JakaTcpClient) -> list[float]:
    pose = robot.get_actual_tcp_pos()
    if len(pose) != 6:
        pose = robot.get_tcp_pos()
    if len(pose) != 6:
        raise RuntimeError(f"无法读取 TCP 位姿: {pose!r}")
    return [float(v) for v in pose]


def format_pose(pose: list[float]) -> str:
    return (
        f"xyz=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) mm, "
        f"rpy=({pose[3]:.2f}, {pose[4]:.2f}, {pose[5]:.2f}) deg"
    )


def tick_velocity(key: str, tick_mm: float, tick_deg: float) -> list[float] | None:
    """将按键映射为每周期位姿增量。"""
    mapping = {
        "w": [tick_mm, 0, 0, 0, 0, 0],
        "s": [-tick_mm, 0, 0, 0, 0, 0],
        "a": [0, tick_mm, 0, 0, 0, 0],
        "d": [0, -tick_mm, 0, 0, 0, 0],
        "q": [0, 0, tick_mm, 0, 0, 0],
        "e": [0, 0, -tick_mm, 0, 0, 0],
        "i": [0, 0, 0, tick_deg, 0, 0],
        "k": [0, 0, 0, -tick_deg, 0, 0],
        "j": [0, 0, 0, 0, tick_deg, 0],
        "l": [0, 0, 0, 0, -tick_deg, 0],
        "u": [0, 0, 0, 0, 0, tick_deg],
        "o": [0, 0, 0, 0, 0, -tick_deg],
    }
    return mapping.get(key)


def handle_keys(
    keys: list[str],
    target: list[float],
    velocity: list[float],
    *,
    tick_mm: float,
    tick_deg: float,
) -> str | None:
    """处理按键，更新速度与目标。返回 'quit' 表示退出。"""
    for key in keys:
        lower = key.lower()
        if lower in ("x",) or key == "\x03":
            return "quit"
        if key == "\x1b":
            return "quit"
        if lower == " ":
            velocity[:] = [0.0] * 6
            continue
        if lower == "p":
            return "print"
        if lower == "r":
            return "reset"

        delta = tick_velocity(lower, tick_mm, tick_deg)
        if delta is not None:
            velocity[:] = delta
    return None


def run_teleop(robot: JakaTcpClient, args: argparse.Namespace) -> None:
    target_cart = read_initial_pose(robot)
    ref_joint = robot.get_joint_pos()
    velocity = [0.0] * 6
    tick_mm = args.speed_mm * args.period
    tick_deg = args.speed_deg * args.period

    print(f"初始 TCP: {format_pose(target_cart)}")
    print(f"控制模式: {args.mode}")
    input("确认安全后按 Enter 进入伺服模式...")

    robot.prepare_servo_mode(filter_preset="lpf")
    print("已进入伺服模式。按住移动键开始运动，空格停止，X 退出。")

    error_count = 0
    tick_index = 0
    next_tick = time.perf_counter()
    target_joint = list(ref_joint)
    ik_interval = 5

    try:
        with TerminalKeyboard() as keyboard:
            while True:
                keys = keyboard.read_keys()
                if keys:
                    action = handle_keys(keys, target_cart, velocity, tick_mm=tick_mm, tick_deg=tick_deg)
                    if action == "quit":
                        return
                    if action == "reset":
                        target_cart = read_initial_pose(robot)
                        ref_joint = robot.get_joint_pos()
                        target_joint = list(ref_joint)
                        velocity[:] = [0.0] * 6
                        print(f"\n已重置: {format_pose(target_cart)}")
                    elif action == "print":
                        actual = read_initial_pose(robot)
                        print(f"\n目标: {format_pose(target_cart)}")
                        print(f"实际: {format_pose(actual)}")
                elif not any(velocity):
                    pass
                else:
                    # 无新按键时保持上一周期速度，由空格清零。
                    pass

                delta = velocity.copy()
                for i in range(6):
                    target_cart[i] += delta[i]

                if args.mode == "servo_p":
                    # 官方 SDK 示例使用绝对位姿 servo_p(ABS)。
                    robot.servo_p(
                        target_cart,
                        rel_flag=0,
                        step_num=1,
                        raise_on_error=False,
                        wait_response=False,
                    )
                else:
                    if tick_index % ik_interval == 0:
                        try:
                            ref_joint = robot.get_joint_pos()
                            target_joint = robot.kine_inverse(
                                ref_joint, target_cart, raise_on_error=False
                            )
                        except JakaTcpError:
                            pass
                    robot.servo_j(
                        target_joint,
                        rel_flag=0,
                        step_num=1,
                        raise_on_error=False,
                        wait_response=False,
                    )

                for resp in robot.drain_responses():
                        if str(resp.get("errorCode", "")) != "0":
                            error_count += 1
                            if error_count <= 10:
                                print(
                                    f"\n[servo 错误 #{error_count}] "
                                    f"cmd={resp.get('cmdName')} "
                                    f"errorCode={resp.get('errorCode')} "
                                    f"errorMsg={resp.get('errorMsg')}"
                                )
                        elif error_count:
                            print("\nservo 恢复正常")
                            error_count = 0

                tick_index += 1
                if tick_index % 60 == 0:
                    print(
                        f"\r目标: {format_pose(target_cart)} | 速度: "
                        f"{[round(v, 3) for v in velocity]}    ",
                        end="",
                        flush=True,
                    )

                next_tick += args.period
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()
    finally:
        velocity[:] = [0.0] * 6
        robot.drain_responses()
        print("\n退出伺服模式...")
        robot.servo_move_enable(False)


def main() -> int:
    args = parse_args()
    print_help(args.speed_mm, args.speed_deg, args.period, args.mode)

    try:
        with JakaTcpClient(args.ip, timeout=8.0) as robot:
            robot.connect()
            print(f"已连接 {args.ip}:10001，正在探测控制器...")
            version = robot.probe(retries=3, retry_delay=1.5)
            print(f"控制器: {version.get('robot_name')} {version.get('version')}")

            ensure_robot_ready(robot, skip_power=args.no_power)
            run_teleop(robot, args)

            if not args.no_shutdown and not args.no_power:
                robot.disable_robot()
                robot.power_off()
                print("已下使能并断电")
            elif args.no_shutdown:
                print("已保留上电/使能状态 (--no-shutdown)")

    except KeyboardInterrupt:
        print("\n用户中断，已安全退出。")
        return 130
    except (JakaTcpError, TimeoutError, ConnectionError, OSError, ValueError, RuntimeError) as exc:
        print(f"\n错误: {exc}", file=sys.stderr)
        return 1

    print("遥操作结束")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
