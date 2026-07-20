#!/usr/bin/env python3
"""JAKA SDK 双臂 servo_p 笛卡尔键盘遥操作（双进程、左右分键）。"""

from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import select
import signal
import sys
import time
from typing import Literal

from config import HOME_JOINT_DEG
from jaka_sdk_client import ERR_NAMES, JakaSdkError, JakaSdkRobot
from keyboard_teleop import (
    KEY_CODE_TO_NAME,
    LinuxInputKeyboard,
    TerminalEchoGuard,
    cartesian_speed_cmd,
    compose_speed_cmd,
    format_pose,
    integrate_target,
    ramp_speed,
    tracking_error,
)

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

LEFT_ARM_IP = "192.168.10.21"
RIGHT_ARM_IP = "192.168.10.11"

DUAL_KEY_CODE_TO_NAME = {
    **KEY_CODE_TO_NAME,
    71: "kp7",
    72: "kp8",
    73: "kp9",
    74: "kpminus",
    75: "kp4",
    76: "kp5",
    77: "kp6",
    78: "kpplus",
    79: "kp1",
    80: "kp2",
    81: "kp3",
    82: "kp0",
    96: "kpenter",
    102: "homekey",
    107: "end",
}

LEFT_MOTION_KEYS = frozenset({"w", "s", "a", "d", "q", "e", "i", "k", "j", "l", "u", "o"})
RIGHT_MOTION_KEYS = frozenset(
    {"kp8", "kp2", "kp4", "kp6", "kp7", "kp9", "kp1", "kp3", "kp5", "kp0", "kpplus", "kpminus"}
)
RIGHT_KEY_ALIAS = {
    "kp8": "w",
    "kp2": "s",
    "kp4": "a",
    "kp6": "d",
    "kp7": "q",
    "kp9": "e",
    "kp1": "i",
    "kp3": "k",
    "kp5": "j",
    "kp0": "l",
    "kpplus": "u",
    "kpminus": "o",
}

Side = Literal["left", "right"]


class DualInputKeyboard(LinuxInputKeyboard):
    """扩展小键盘键位映射。"""

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
            key = DUAL_KEY_CODE_TO_NAME.get(code)
            if key is not None:
                events.append((key, value))
        return events


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="JAKA SDK 双臂 servo_p 键盘遥操作")
    p.add_argument("--no-power", action="store_true")
    p.add_argument("--no-shutdown", action="store_true")
    p.add_argument("--no-home", action="store_true")
    return p.parse_args()


def print_help(home_joints_deg: list[float]) -> None:
    joints = ", ".join(f"J{i + 1}={v:.1f}" for i, v in enumerate(home_joints_deg))
    print(
        f"""
=== 双臂 servo_p 笛卡尔键盘遥操作 ===
左臂 IP: {LEFT_ARM_IP} | 右臂 IP: {RIGHT_ARM_IP}
速度倍率: {DEFAULT_RAPID_RATE * 100:.0f}% | 平移: {DEFAULT_SPEED_MM_S} mm/s | 旋转: {DEFAULT_SPEED_DEG_S} deg/s
控制周期: {SERVO_PERIOD_S * 1000:.1f} ms | 滤波: {DEFAULT_FILTER}
初始关节角(deg): {joints}

左臂:
  W/S  X+/X-        A/D  Y+/Y-        Q/E  Z+/Z-
  I/K  Rx+/Rx-      J/L  Ry+/Ry-      U/O  Rz+/Rz-
  空格  停止          R  对齐实际       H  回初始点

右臂 (小键盘):
  8/2  X+/X-        4/6  Y+/Y-        7/9  Z+/Z-
  1/3  Rx+/Rx-      5/0  Ry+/Ry-      +/-  Rz+/Rz-
  Enter 停止         End  对齐实际     Home 回初始点

  P  打印双臂位姿      X  退出

左右臂独立进程 + 独立 SDK 实例，真正并行控制。
"""
    )


def move_to_home(robot: JakaSdkRobot, home_joints_deg: list[float], name: str) -> None:
    joints = ", ".join(f"J{i + 1}={v:.1f}" for i, v in enumerate(home_joints_deg))
    print(f"{name}: 移动到初始关节角(deg): {joints}", flush=True)
    robot.joint_move_deg(home_joints_deg, block=True, speed_deg_s=15.0)
    actual = robot.get_joint_pos_deg()
    err = sum(abs(actual[i] - home_joints_deg[i]) for i in range(6))
    tcp = robot.get_tcp_pos()
    print(
        f"{name}: 到位 {[round(v, 2) for v in actual]} deg (关节总误差 {err:.2f} deg), "
        f"TCP: {format_pose(tcp)}",
        flush=True,
    )


def power_up_arm(robot: JakaSdkRobot, skip_power: bool, name: str) -> None:
    """上电使能 + 设置速度倍率。注意：必须在进入伺服模式之前调用，
    否则回零用的 joint_move 会在伺服模式下执行而导致 servo_p 报接口错误。"""
    robot.ensure_ready(skip_power=skip_power)
    before = robot.get_rapid_rate()
    robot.set_rapid_rate(DEFAULT_RAPID_RATE)
    after = robot.get_rapid_rate()
    print(f"{name} 速度倍率: {before * 100:.0f}% -> {after * 100:.0f}%", flush=True)


def speed_cmd_for_side(side: Side, key: str) -> list[float] | None:
    if side == "left":
        return cartesian_speed_cmd(key, DEFAULT_SPEED_MM_S, DEFAULT_SPEED_DEG_S)
    alias = RIGHT_KEY_ALIAS.get(key)
    if alias is None:
        return None
    return cartesian_speed_cmd(alias, DEFAULT_SPEED_MM_S, DEFAULT_SPEED_DEG_S)


def compose_speed_for_side(side: Side, keys: list[str]) -> list[float]:
    if side == "left":
        return compose_speed_cmd(keys, DEFAULT_SPEED_MM_S, DEFAULT_SPEED_DEG_S)

    cmd = [0.0] * 6
    for key in keys:
        delta = speed_cmd_for_side("right", key)
        if delta is None:
            continue
        for idx in range(6):
            cmd[idx] += delta[idx]

    lin_mag = math.sqrt(sum(cmd[idx] * cmd[idx] for idx in range(3)))
    if lin_mag > DEFAULT_SPEED_MM_S > 0:
        scale = DEFAULT_SPEED_MM_S / lin_mag
        for idx in range(3):
            cmd[idx] *= scale

    ang_limit = math.radians(DEFAULT_SPEED_DEG_S)
    ang_mag = math.sqrt(sum(cmd[idx] * cmd[idx] for idx in range(3, 6)))
    if ang_mag > ang_limit > 0:
        scale = ang_limit / ang_mag
        for idx in range(3, 6):
            cmd[idx] *= scale
    return cmd


def publish_state(state_queue: mp.Queue, target: list[float], actual: list[float]) -> None:
    while True:
        try:
            state_queue.get_nowait()
        except Exception:
            break
    try:
        state_queue.put_nowait((target.copy(), actual.copy()))
    except Exception:
        pass


def drain_latest_state(state_queue: mp.Queue) -> tuple[list[float], list[float]] | None:
    latest: tuple[list[float], list[float]] | None = None
    while True:
        try:
            latest = state_queue.get_nowait()
        except Exception:
            break
    return latest


def arm_control_process(
    ip: str,
    name: str,
    side: Side,
    home_joints_deg: list[float],
    skip_power: bool,
    skip_home: bool,
    skip_shutdown: bool,
    event_queue: mp.Queue,
    stop_event: mp.synchronize.Event,
    ready_event: mp.synchronize.Event,
    state_queue: mp.Queue,
) -> None:
    accel_limit = [
        DEFAULT_ACCEL_MM_S2,
        DEFAULT_ACCEL_MM_S2,
        DEFAULT_ACCEL_MM_S2,
        math.radians(DEFAULT_ACCEL_DEG_S2),
        math.radians(DEFAULT_ACCEL_DEG_S2),
        math.radians(DEFAULT_ACCEL_DEG_S2),
    ]
    target_speed_cmd = [0.0] * 6
    smooth_speed_cmd = [0.0] * 6
    pressed_motion_keys: list[str] = []
    moving_prev = False
    period_s = max(0.004, SERVO_PERIOD_S)
    safety_interval = DEFAULT_SAFETY_CHECK_INTERVAL
    tick = 0
    err_streak = 0

    # 忽略 SIGINT，退出由主进程通过 stop_event 统一协调，避免子进程抛出
    # 未捕获的 KeyboardInterrupt 堆栈。
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    try:
        with JakaSdkRobot(ip) as robot:
            power_up_arm(robot, skip_power, name)
            if not skip_home:
                try:
                    move_to_home(robot, home_joints_deg, name)
                except JakaSdkError as exc:
                    print(f"{name} 警告: 无法自动回初始点 ({exc})", flush=True)

            robot.prepare_servo(filter=DEFAULT_FILTER)
            target = robot.get_tcp_pos()
            actual = target.copy()
            publish_state(state_queue, target, actual)
            ready_event.set()

            next_tick = time.perf_counter()
            while not stop_event.is_set():
                while True:
                    try:
                        key, value = event_queue.get_nowait()
                    except Exception:
                        break

                    is_press = value == 1
                    is_down = value in (1, 2)

                    if is_press and key == "stop":
                        target_speed_cmd[:] = [0.0] * 6
                        smooth_speed_cmd[:] = [0.0] * 6
                        pressed_motion_keys.clear()
                        target = robot.get_tcp_pos()
                        continue
                    if is_press and key == "resync":
                        target_speed_cmd[:] = [0.0] * 6
                        smooth_speed_cmd[:] = [0.0] * 6
                        pressed_motion_keys.clear()
                        target = robot.get_tcp_pos()
                        print(f"\n{name} 已对齐实际 TCP", flush=True)
                        continue
                    if is_press and key == "home":
                        target_speed_cmd[:] = [0.0] * 6
                        smooth_speed_cmd[:] = [0.0] * 6
                        pressed_motion_keys.clear()
                        robot.exit_servo()
                        move_to_home(robot, home_joints_deg, name)
                        robot.prepare_servo(filter=DEFAULT_FILTER)
                        target = robot.get_tcp_pos()
                        print(f"{name} 已回初始关节角", flush=True)
                        continue

                    if speed_cmd_for_side(side, key) is not None:
                        if is_down:
                            if key in pressed_motion_keys:
                                pressed_motion_keys.remove(key)
                            pressed_motion_keys.append(key)
                        elif value == 0 and key in pressed_motion_keys:
                            pressed_motion_keys.remove(key)

                        new_cmd = compose_speed_for_side(side, pressed_motion_keys)
                        start_moving = (
                            not any(abs(v) > 1e-12 for v in target_speed_cmd)
                            and any(abs(v) > 1e-12 for v in new_cmd)
                        )
                        if start_moving:
                            target = robot.get_tcp_pos()
                        target_speed_cmd[:] = new_cmd

                dt_s = period_s
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
                                print(f"\n[{name}] 逆解失败，已停车并重同步", flush=True)
                            else:
                                pe, re = tracking_error(actual, target)
                                if pe > MAX_TRACK_ERR_MM or re > MAX_TRACK_ERR_RAD:
                                    target_speed_cmd[:] = [0.0] * 6
                                    smooth_speed_cmd[:] = [0.0] * 6
                                    target = actual
                                    print(
                                        f"\n[{name}] 跟踪过大 xyz={pe:.1f}mm，已停车重同步",
                                        flush=True,
                                    )
                    else:
                        err_streak += 1
                        if err_streak <= 5:
                            err_name = ERR_NAMES.get(errno, "interface_error")
                            print(
                                f"\n[{name}] servo_p errno={errno} ({err_name})",
                                flush=True,
                            )
                        target = robot.get_tcp_pos()
                        target_speed_cmd[:] = [0.0] * 6
                        smooth_speed_cmd[:] = [0.0] * 6
                else:
                    if moving_prev:
                        target = robot.get_tcp_pos()
                    robot.servo_p(target)
                    actual = target

                if tick % safety_interval == 0:
                    publish_state(state_queue, target, actual)

                moving_prev = moving
                tick += 1

                next_tick += period_s
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()

            target_speed_cmd[:] = [0.0] * 6
            smooth_speed_cmd[:] = [0.0] * 6
            robot.exit_servo()
            if not skip_shutdown and not skip_power:
                robot.disable_robot()
                robot.power_off()
    except JakaSdkError as exc:
        ready_event.set()
        print(f"\n{name} 错误: {exc}", file=sys.stderr, flush=True)
    finally:
        print(f"{name} 已退出伺服模式", flush=True)


def route_key(key: str, value: int, left_q: mp.Queue, right_q: mp.Queue) -> str | None:
    """将按键路由到左/右臂事件队列，返回全局命令。"""
    is_press = value == 1
    is_down = value in (1, 2)

    if is_down and key in ("x", "esc"):
        return "exit"
    if is_press and key == "p":
        return "print"

    if key in LEFT_MOTION_KEYS:
        left_q.put((key, value))
        return None
    if key in RIGHT_MOTION_KEYS:
        right_q.put((key, value))
        return None

    if is_press and key == " ":
        left_q.put(("stop", value))
        return None
    if is_press and key == "r":
        left_q.put(("resync", value))
        return None
    if is_press and key == "h":
        left_q.put(("home", value))
        return None

    if is_press and key == "kpenter":
        right_q.put(("stop", value))
        return None
    if is_press and key == "end":
        right_q.put(("resync", value))
        return None
    if is_press and key == "homekey":
        right_q.put(("home", value))
        return None

    return None


def run_dual_teleop(args: argparse.Namespace, home_joints_deg: list[float]) -> None:
    ctx = mp.get_context("spawn")
    left_q: mp.Queue = ctx.Queue()
    right_q: mp.Queue = ctx.Queue()
    left_state_q: mp.Queue = ctx.Queue(maxsize=1)
    right_state_q: mp.Queue = ctx.Queue(maxsize=1)
    stop_event = ctx.Event()
    left_ready = ctx.Event()
    right_ready = ctx.Event()

    left_proc = ctx.Process(
        target=arm_control_process,
        name="left-arm",
        args=(
            LEFT_ARM_IP,
            "左臂",
            "left",
            home_joints_deg,
            args.no_power,
            args.no_home,
            args.no_shutdown,
            left_q,
            stop_event,
            left_ready,
            left_state_q,
        ),
        daemon=False,
    )
    right_proc = ctx.Process(
        target=arm_control_process,
        name="right-arm",
        args=(
            RIGHT_ARM_IP,
            "右臂",
            "right",
            home_joints_deg,
            args.no_power,
            args.no_home,
            args.no_shutdown,
            right_q,
            stop_event,
            right_ready,
            right_state_q,
        ),
        daemon=False,
    )

    left_proc.start()
    right_proc.start()

    if not left_ready.wait(timeout=180.0) or not right_ready.wait(timeout=180.0):
        stop_event.set()
        left_proc.join(timeout=5.0)
        right_proc.join(timeout=5.0)
        raise JakaSdkError("双臂初始化超时")

    left_state = drain_latest_state(left_state_q)
    right_state = drain_latest_state(right_state_q)
    if left_state:
        print(f"左臂当前 TCP: {format_pose(left_state[0])}")
    if right_state:
        print(f"右臂当前 TCP: {format_pose(right_state[0])}")
    print("双臂已进入 servo_p。左右分键独立控制，松手停止。")
    input("确认安全后按 Enter 开始...")

    try:
        with TerminalEchoGuard(), DualInputKeyboard() as kb:
            print(f"键盘设备: {kb.path}")
            while not stop_event.is_set():
                if not left_proc.is_alive() or not right_proc.is_alive():
                    print("\n子进程异常退出", file=sys.stderr)
                    break

                for key, value in kb.read_events():
                    cmd = route_key(key, value, left_q, right_q)
                    if cmd == "exit":
                        stop_event.set()
                        break
                    if cmd == "print":
                        left_state = drain_latest_state(left_state_q)
                        right_state = drain_latest_state(right_state_q)
                        if left_state:
                            lt, la = left_state
                            lpe, lre = tracking_error(la, lt)
                            print(f"\n左臂目标: {format_pose(lt)}")
                            print(f"左臂实际: {format_pose(la)}")
                            print(f"左臂误差: xyz={lpe:.2f} mm, rot={math.degrees(lre):.2f} deg")
                        if right_state:
                            rt, ra = right_state
                            rpe, rre = tracking_error(ra, rt)
                            print(f"右臂目标: {format_pose(rt)}")
                            print(f"右臂实际: {format_pose(ra)}")
                            print(f"右臂误差: xyz={rpe:.2f} mm, rot={math.degrees(rre):.2f} deg")

                if stop_event.is_set():
                    break
                time.sleep(0.001)
    finally:
        stop_event.set()
        left_proc.join(timeout=8.0)
        right_proc.join(timeout=8.0)
        if left_proc.is_alive():
            left_proc.terminate()
        if right_proc.is_alive():
            right_proc.terminate()


def main() -> int:
    args = parse_args()
    home_joints_deg = HOME_JOINT_DEG.copy()

    try:
        print_help(home_joints_deg)
        run_dual_teleop(args, home_joints_deg)
    except KeyboardInterrupt:
        print("\n用户中断")
        return 130
    except JakaSdkError as exc:
        print(f"\n错误: {exc}", file=sys.stderr)
        return 1

    if args.no_shutdown:
        print("保留上电/使能 (--no-shutdown)")
    print("双臂遥操作结束")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
