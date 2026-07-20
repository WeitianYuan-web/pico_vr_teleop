#!/usr/bin/env python3
"""servo_p / servo_j / moveL / joint_move 真机诊断脚本。

用法::

    python3 servo_p_test.py --no-power
    python3 servo_p_test.py --mode joint_move
    python3 servo_p_test.py --mode moveL --axis x --delta 0.08
    python3 servo_p_test.py --mode servo_p_abs --filter lpf
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from config import ROBOT_IP
from jaka_tcp_client import JakaTcpClient, JakaTcpError

PERIOD_S = 0.008
DEFAULT_TICKS = 125  # 约 1 秒


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="JAKA 运动诊断")
    parser.add_argument("--ip", default=ROBOT_IP)
    parser.add_argument("--no-power", action="store_true")
    parser.add_argument(
        "--mode",
        choices=("servo_p_abs", "servo_p_rel", "servo_j_abs", "moveL", "moveL_abs", "joint_move"),
        default="joint_move",
        help="joint_move=关节运动验证(默认, 最可靠)",
    )
    parser.add_argument(
        "--axis",
        choices=("x", "y", "z", "rx", "ry", "rz"),
        default="x",
        help="笛卡尔运动轴",
    )
    parser.add_argument("--joint", type=int, default=2, help="joint_move 测试关节(1-6)")
    parser.add_argument("--delta", type=float, default=0.08, help="每周期/相对增量(mm, deg 或关节度)")
    parser.add_argument("--ticks", type=int, default=DEFAULT_TICKS, help="伺服发送周期数")
    parser.add_argument("--period", type=float, default=PERIOD_S)
    parser.add_argument("--blocking", action="store_true", help="伺服每帧等待响应")
    parser.add_argument(
        "--filter",
        choices=("none", "lpf", "carte"),
        default="lpf",
        help="伺服滤波器预设",
    )
    return parser.parse_args()


def axis_index(axis: str) -> int:
    return {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}[axis]


def ensure_ready(robot: JakaTcpClient, *, skip_power: bool) -> None:
    state = robot.get_robot_state()
    if skip_power:
        if not JakaTcpClient.is_powered(state) or not JakaTcpClient.is_enabled(state):
            raise RuntimeError("已指定 --no-power，但机械臂未上电或未使能")
        return

    if not JakaTcpClient.is_powered(state):
        robot.power_on()
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            state = robot.get_robot_state()
            if JakaTcpClient.is_powered(state):
                break
            time.sleep(0.5)
    if not JakaTcpClient.is_enabled(state):
        robot.enable_robot()
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            state = robot.get_robot_state()
            if JakaTcpClient.is_enabled(state):
                break
            time.sleep(0.5)


def read_pose(robot: JakaTcpClient) -> list[float]:
    pose = robot.get_actual_tcp_pos()
    if len(pose) != 6:
        pose = robot.get_tcp_pos()
    if len(pose) != 6:
        raise RuntimeError("无法读取 TCP 位姿")
    return [float(v) for v in pose]


def print_diagnostics(info: dict[str, Any]) -> None:
    robot = info["robot"]
    motion = info["motion"]
    drag = info["drag"]
    rapid = info["rapid"]
    print("--- 运动前状态 ---")
    print(
        f"  power={robot.get('power')} enable={robot.get('enable')} "
        f"errcode={robot.get('errcode')} msg={robot.get('msg', '')!r}"
    )
    print(
        f"  inpos={motion.get('inpos')} paused={motion.get('paused')} "
        f"queue={motion.get('queue')} active={motion.get('active_queue')}"
    )
    print(
        f"  drag={drag.get('status')} rapid={rapid.get('value')} "
        f"servo_mode={info['servo']} user_id={info.get('user_id')} tool_id={info.get('tool_id')}"
    )
    if drag.get("status") in (True, 1, "1"):
        print("  警告: 处于拖拽模式，运动指令可能无效，请在示教器关闭拖拽")
    if not JakaTcpClient.is_enabled(robot):
        print("  警告: 未使能，运动指令不会执行")
    if str(robot.get("errcode", "0")) not in ("0", "0x0", ""):
        print(f"  警告: 控制器 errcode={robot.get('errcode')}")


def summarize_responses(responses: list[dict]) -> tuple[int, int, list[dict]]:
    errors = [r for r in responses if str(r.get("errorCode", "")) != "0"]
    return len(responses) - len(errors), len(errors), errors


def run_joint_test(robot: JakaTcpClient, args: argparse.Namespace, joint_before: list[float]) -> None:
    j = max(1, min(6, args.joint)) - 1
    delta = max(args.delta, 1.0) if args.delta < 1.0 else args.delta
    target = joint_before.copy()
    target[j] += delta
    print(f"joint_move: J{j + 1} {joint_before[j]:.3f} -> {target[j]:.3f} (+{delta:.3f} deg)")

    t0 = time.perf_counter()
    resp = robot.joint_move(target, rel_flag=0, speed=15.0, accel=30.0, wait=True)
    elapsed = time.perf_counter() - t0
    print(f"joint_move 响应: errorCode={resp.get('errorCode')} errorMsg={resp.get('errorMsg')!r}")
    print(f"joint_move 耗时: {elapsed:.3f} s")

    joint_after = robot.get_joint_pos()
    print(f"运动后关节: {joint_after}")
    print(f"J{j + 1} 变化: {joint_after[j] - joint_before[j]:+.3f} deg")


def run_move_l_test(
    robot: JakaTcpClient,
    args: argparse.Namespace,
    pose_before: list[float],
    joint_before: list[float],
    *,
    absolute: bool,
) -> None:
    axis = axis_index(args.axis)
    total_delta = args.delta * args.ticks
    delta_vec = [0.0] * 6
    delta_vec[axis] = total_delta
    label = "绝对" if absolute else "相对"
    print(f"moveL {label}: 轴={args.axis}, 总增量={total_delta:.3f}")

    rapid_before = robot.get_rapid_rate().get("value")
    if rapid_before is not None and float(rapid_before) < 0.99:
        print(f"  将 rapid_rate 从 {rapid_before} 提升到 1.0")
        robot.set_rapid_rate(1.0)

    if absolute:
        target = pose_before.copy()
        target[axis] += total_delta
        cmd_pos = target
        rel_flag = 0
    else:
        target = pose_before.copy()
        target[axis] += total_delta
        cmd_pos = delta_vec
        rel_flag = 1

    try:
        ik_joint = robot.kine_inverse(joint_before, target)
        print(f"  逆解成功: {[round(v, 3) for v in ik_joint]}")
    except JakaTcpError as exc:
        print(f"  逆解失败(笛卡尔运动可能无法执行): {exc}")

    t0 = time.perf_counter()
    resp = robot.move_l(cmd_pos, rel_flag=rel_flag, speed=20.0, accel=50.0, wait=True)
    elapsed = time.perf_counter() - t0
    motion = robot.get_motion_state()
    pose_after = read_pose(robot)
    joint_after = robot.get_joint_pos()

    print(f"moveL 响应: errorCode={resp.get('errorCode')} errorMsg={resp.get('errorMsg')!r}")
    print(f"moveL 总耗时(含等待到位): {elapsed:.3f} s")
    print(
        f"运动后 motion: inpos={motion.get('inpos')} paused={motion.get('paused')} "
        f"err_add_line={motion.get('err_add_line')} queue={motion.get('queue')}"
    )
    print(f"运动后 TCP: {pose_after}")
    print(f"TCP 变化({args.axis}): {pose_after[axis] - pose_before[axis]:+.3f}")
    print(f"关节变化: {[round(joint_after[i] - joint_before[i], 3) for i in range(6)]}")


def run_servo_test(
    robot: JakaTcpClient,
    args: argparse.Namespace,
    pose_before: list[float],
    joint_before: list[float],
) -> None:
    axis = axis_index(args.axis)
    wait_response = args.blocking
    abs_target = pose_before.copy()
    delta_vec = [0.0] * 6
    delta_vec[axis] = args.delta
    target_joint = list(joint_before)

    robot.prepare_servo_mode(filter_preset=args.filter)

    all_responses: list[dict] = []
    periods: list[float] = []
    next_tick = time.perf_counter()
    t0 = time.perf_counter()

    try:
        for i in range(args.ticks):
            tick_start = time.perf_counter()

            if args.mode == "servo_p_rel":
                robot.servo_p(delta_vec, rel_flag=1, wait_response=wait_response, raise_on_error=False)
            elif args.mode == "servo_p_abs":
                abs_target[axis] = pose_before[axis] + args.delta * (i + 1)
                robot.servo_p(abs_target, rel_flag=0, wait_response=wait_response, raise_on_error=False)
            else:
                target_cart = pose_before.copy()
                target_cart[axis] += args.delta * (i + 1)
                if i % 5 == 0:
                    try:
                        ref_joint = robot.get_joint_pos()
                        target_joint = robot.kine_inverse(ref_joint, target_cart, raise_on_error=False)
                    except JakaTcpError:
                        pass
                robot.servo_j(target_joint, rel_flag=0, wait_response=wait_response, raise_on_error=False)

            if not wait_response:
                all_responses.extend(robot.drain_responses())

            periods.append(time.perf_counter() - tick_start)
            next_tick += args.period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()
    finally:
        all_responses.extend(robot.drain_responses())
        robot.servo_move_enable(False)

    elapsed = time.perf_counter() - t0
    ok_count, err_count, errors = summarize_responses(all_responses)
    pose_after = read_pose(robot)
    joint_after = robot.get_joint_pos()

    print(f"\n滤波器: {args.filter}")
    print(f"发送 {args.ticks} 帧，耗时 {elapsed:.3f} s，平均周期 {elapsed / args.ticks * 1000:.2f} ms")
    if periods:
        print(
            f"单帧处理: avg={sum(periods) / len(periods) * 1000:.2f} ms, "
            f"max={max(periods) * 1000:.2f} ms"
        )
    print(f"响应统计: 成功={ok_count}, 失败={err_count}, 收到={len(all_responses)}")
    print(f"运动后 TCP: {pose_after}")
    print(f"TCP 变化({args.axis}): {pose_after[axis] - pose_before[axis]:+.3f}")
    print(f"运动后关节: {joint_after}")
    if errors:
        print(f"首条错误: {errors[0]}")
        if err_count > 1:
            print(f"共 {err_count} 条错误")
    else:
        print("未检测到 errorCode!=0 的响应")


def main() -> int:
    args = parse_args()

    print(f"模式: {args.mode}, 轴: {args.axis}, 增量: {args.delta}")
    if args.mode.startswith("servo"):
        print(f"滤波器: {args.filter}, 阻塞: {args.blocking}, 周期: {args.period * 1000:.1f} ms")

    try:
        with JakaTcpClient(args.ip, timeout=8.0) as robot:
            robot.connect()
            robot.probe()
            ensure_ready(robot, skip_power=args.no_power)

            pose_before = read_pose(robot)
            joint_before = robot.get_joint_pos()
            print(f"运动前 TCP: {pose_before}")
            print(f"运动前关节: {joint_before}")

            input("确认安全后按 Enter 开始测试...")

            info = robot.ensure_motion_ready()
            print_diagnostics(info)

            if args.mode == "joint_move":
                run_joint_test(robot, args, joint_before)
            elif args.mode == "moveL":
                run_move_l_test(robot, args, pose_before, joint_before, absolute=False)
            elif args.mode == "moveL_abs":
                run_move_l_test(robot, args, pose_before, joint_before, absolute=True)
            else:
                run_servo_test(robot, args, pose_before, joint_before)

    except KeyboardInterrupt:
        print("\n用户中断")
        return 130
    except (JakaTcpError, TimeoutError, ConnectionError, OSError, ValueError, RuntimeError) as exc:
        print(f"\n错误: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
