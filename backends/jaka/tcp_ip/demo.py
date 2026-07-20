#!/usr/bin/env python3
"""JAKA 机械臂 TCP/IP 简单控制示例。

流程：连接 -> 查询版本/状态 -> 上电 -> 使能 -> 读取关节角 -> （可选）小幅运动 -> 下使能

用法::

    python3 demo.py                 # 仅查询状态，不运动
    python3 demo.py --move          # 执行小幅关节运动（请确保工作空间安全）
    python3 demo.py --ip 192.168.10.11
"""

from __future__ import annotations

import argparse
import sys
import time

from config import ROBOT_IP, ROBOT_USERNAME
from jaka_tcp_client import JakaTcpClient, JakaTcpError


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="JAKA TCP/IP 简单控制示例")
    parser.add_argument("--ip", default=ROBOT_IP, help="机械臂控制器 IP")
    parser.add_argument(
        "--move",
        action="store_true",
        help="执行小幅关节相对运动（默认仅查询状态）",
    )
    parser.add_argument(
        "--no-shutdown",
        action="store_true",
        help="结束后不下使能/断电（保持上电使能）",
    )
    return parser.parse_args()


def print_section(title: str) -> None:
    print(f"\n=== {title} ===")


def wait_for_state(
    robot: JakaTcpClient,
    *,
    power: int | None = None,
    enable: bool | None = None,
    timeout: float = 15.0,
    interval: float = 1.0,
) -> dict:
    """轮询 get_robot_state，直到满足目标状态或超时。"""
    deadline = time.monotonic() + timeout
    last_state: dict = {}
    while time.monotonic() < deadline:
        last_state = robot.get_robot_state()
        power_ok = power is None or last_state.get("power") == power
        enable_ok = enable is None or last_state.get("enable") is enable
        if power_ok and enable_ok:
            return last_state
        time.sleep(interval)
    return last_state


def main() -> int:
    args = parse_args()

    print(f"目标机械臂: {args.ip}")
    print(
        "说明: TCP/IP 协议（10001 端口）无需账号密码；"
        f"配置的账号 {ROBOT_USERNAME!r} 仅用于 SDK 登录。"
    )

    try:
        with JakaTcpClient(args.ip) as robot:
            print_section("连接 10001 端口")
            robot.connect(enable_status_port=False)
            print("连接成功")

            print_section("查询控制器版本")
            version = robot.get_version()
            print(
                f"robot_name={version.get('robot_name')}, "
                f"version={version.get('version')}, "
                f"robot_id={version.get('robot_id')}"
            )

            print_section("查询机器人状态")
            state = robot.get_robot_state()
            print(
                f"power={state.get('power')}, enable={state.get('enable')}, "
                f"errcode={state.get('errcode')}"
            )

            if not state.get("power"):
                print_section("上电")
                resp = robot.power_on()
                print(f"power_on 响应: errorCode={resp.get('errorCode')}, errorMsg={resp.get('errorMsg')}")
                state = wait_for_state(robot, power=1, timeout=20.0)
                print(f"上电后 power={state.get('power')}, errcode={state.get('errcode')}, msg={state.get('msg')}")

            if state.get("power") and not state.get("enable"):
                print_section("上使能")
                resp = robot.enable_robot()
                print(f"enable_robot 响应: errorCode={resp.get('errorCode')}, errorMsg={resp.get('errorMsg')}")
                state = wait_for_state(robot, power=1, enable=True, timeout=15.0)
                print(f"使能后 enable={state.get('enable')}, errcode={state.get('errcode')}, msg={state.get('msg')}")

            if not state.get("power"):
                print("警告: 机械臂未上电，请检查急停、安全回路或示教器状态。")
            elif not state.get("enable"):
                print("警告: 机械臂未使能，运动指令将被跳过。")

            print_section("读取关节位置（度）")
            joints = robot.get_joint_pos()
            print(f"joint_pos = {joints}")

            if args.move:
                if not state.get("power") or not state.get("enable"):
                    print("机械臂未上电或未使能，跳过运动。")
                elif len(joints) < 6:
                    raise RuntimeError("关节位置数据异常，取消运动")

                print_section("执行小幅相对运动（J2 +3°）")
                target = joints.copy()
                target[1] += 3.0
                print(f"目标关节角: {target}")
                robot.joint_move(target, rel_flag=0, speed=15.0, accel=30.0, wait=True)
                print("运动完成")

                final_joints = robot.get_joint_pos()
                print(f"运动后关节角: {final_joints}")

                print_section("回到原关节位置")
                robot.joint_move(joints, rel_flag=0, speed=15.0, accel=30.0, wait=True)
                print(f"回位后关节角: {robot.get_joint_pos()}")
            else:
                print("\n未指定 --move，跳过运动。若需测试运动请加: python3 demo.py --move")

            if not args.no_shutdown and state.get("power"):
                print_section("下使能并断电")
                if state.get("enable"):
                    robot.disable_robot()
                    time.sleep(0.5)
                robot.power_off()
                print("已下使能并断电")

    except (JakaTcpError, TimeoutError, ConnectionError, OSError, ValueError) as exc:
        print(f"\n错误: {exc}", file=sys.stderr)
        return 1

    print("\n完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
